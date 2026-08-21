// Copyright 2026

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/safety/footprint_samples.hpp"
#include "minco_planner/trajectory/minco_s3.hpp"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"

namespace minco_planner
{

namespace
{

using Point = Eigen::Vector2d;

MincoTimeAllocator makeTimeAllocator(const MincoTrajectoryOptimizerParams & params)
{
  MincoTimeAllocatorParams allocator_params;
  allocator_params.reference_speed = params.reference_speed;
  allocator_params.max_velocity = params.max_velocity;
  allocator_params.max_acceleration = params.max_acceleration;
  allocator_params.max_lateral_acceleration = params.max_lateral_acceleration;
  allocator_params.min_segment_time = params.min_segment_time;
  return MincoTimeAllocator(allocator_params);
}

nav_msgs::msg::Path makeGuidePath(
  const std_msgs::msg::Header & header, const std::vector<Point> & points)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(points.size());
  for (const Point & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

/**
 * @brief 用给定航点与段时长求解一条 MINCO S3（jerk 级五次分段）轨迹。
 * @param waypoints    航点序列，首尾为边界位置，中间为内部约束点。
 * @param durations    每段时长 [s]，长度必须为 waypoints.size()-1。
 * @param head_state   首端边界条件 2x3 = [位置 | 速度 | 加速度]（世界系）。
 *                     速度/加速度非零即为"带初速重规划"，让新轨迹从当前运动状态起接。
 * @param minco        输出：求解后的分段多项式。
 * @return true=带状 LU 分解与回代成功且系数全为有限值。
 * @note 由 optimize 与 refineWaypointsWithEsdf 的每轮迭代调用。
 *       数学上首端三行直接就是多项式在 t=0 处的 0/1/2 阶导数：
 *       \f$ p(0)=c_0,\ \dot p(0)=c_1,\ \ddot p(0)=2c_2 \f$，
 *       所以 head.col(1)/col(2) 会分别成为方程右端第 1、2 行。
 *       尾端仍锁为零速零加速度：目标点必须停稳（安全要求，不随重规划放开）。
 */
bool solveMinco(
  const std::vector<Point> & waypoints,
  const Eigen::VectorXd & durations,
  MincoS3 & minco,
  const Eigen::Matrix<double, 2, 3> & head_state)
{
  Eigen::Matrix<double, 2, 3> head = head_state;
  Eigen::Matrix<double, 2, 3> tail = Eigen::Matrix<double, 2, 3>::Zero();
  head.col(0) = waypoints.front();
  tail.col(0) = waypoints.back();
  Eigen::MatrixXd inner_points(2, static_cast<int>(waypoints.size()) - 2);
  for (int i = 0; i < inner_points.cols(); ++i) {
    inner_points.col(i) = waypoints[static_cast<std::size_t>(i + 1)];
  }
  return minco.solve(head, tail, inner_points, durations);
}

std::vector<Point> densifyWaypoints(const std::vector<Point> & waypoints, double spacing)
{
  if (waypoints.size() <= 1 || spacing <= 1e-6) {
    return waypoints;
  }

  std::vector<Point> dense;
  dense.reserve(waypoints.size());
  dense.push_back(waypoints.front());
  for (std::size_t index = 0; index + 1 < waypoints.size(); ++index) {
    const Point & start = waypoints[index];
    const Point & end = waypoints[index + 1];
    const int steps = std::max(1, static_cast<int>(std::ceil((end - start).norm() / spacing)));
    for (int step = 1; step <= steps; ++step) {
      dense.push_back(start + (end - start) * static_cast<double>(step) / steps);
    }
  }
  return dense;
}

Point limitNorm(const Point & value, double maximum_norm)
{
  const double norm = value.norm();
  if (maximum_norm > 0.0 && norm > maximum_norm) {
    return value * (maximum_norm / norm);
  }
  return value;
}

/**
 * @brief 把可选的重规划初值裁剪成 MINCO 首端边界矩阵。
 * @param initial_state 可为空；空或 valid=false 时返回全零（等价于原静止假设）。
 * @param params        提供 initial_state_max_speed / max_acceleration 两个保护上限。
 * @return 2x3 首端边界 [位置 | 速度 | 加速度]，位置列留零由 solveMinco 覆盖。
 * @note 每次 optimize 入口调用一次。裁剪原因：定位/里程计的速度存在噪声与外推
 *       误差，非有限值或异常大的值会让首段多项式直接冲出速度可行域，
 *       进而被时间缩放整体拉长（表现为"重规划后全程变慢"）。
 */
Eigen::Matrix<double, 2, 3> makeHeadState(
  const InitialKinematicState * initial_state,
  const MincoTrajectoryOptimizerParams & params)
{
  Eigen::Matrix<double, 2, 3> head = Eigen::Matrix<double, 2, 3>::Zero();
  if (initial_state == nullptr || !initial_state->valid) {
    return head;
  }
  if (!initial_state->velocity.allFinite() || !initial_state->acceleration.allFinite()) {
    return head;
  }
  // 播种量同时受"初值保护上限"和"轨迹动力学上限"约束：若首端速度本身就超过
  // max_velocity，时间缩放永远无法把峰值压回可行域（缩放不改变边界条件），
  // 结果是白白把整条轨迹拉长 max_time_scaling_iterations 次。
  double speed_limit = std::max(0.0, params.initial_state_max_speed);
  if (params.max_velocity > 0.0) {
    speed_limit = std::min(speed_limit, params.max_velocity);
  }
  double acceleration_limit = std::max(0.0, params.initial_state_max_acceleration);
  if (params.max_acceleration > 0.0) {
    acceleration_limit = std::min(acceleration_limit, params.max_acceleration);
  }
  head.col(1) = limitNorm(initial_state->velocity, speed_limit);
  head.col(2) = limitNorm(initial_state->acceleration, acceleration_limit);
  return head;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double interpolateReferenceYaw(const ReferenceTrajectory & reference, double progress)
{
  if (reference.points.empty()) {
    return 0.0;
  }
  if (reference.points.size() == 1U || reference.totalTime() <= 1e-6) {
    return reference.points.front().yaw;
  }

  const double target_time = std::max(0.0, std::min(1.0, progress)) * reference.totalTime();
  const auto upper = std::lower_bound(
    reference.points.begin(), reference.points.end(), target_time,
    [](const ReferencePoint & point, double time) {return point.t < time;});
  if (upper == reference.points.begin()) {
    return upper->yaw;
  }
  if (upper == reference.points.end()) {
    return reference.points.back().yaw;
  }

  const ReferencePoint & before = *(upper - 1);
  const double duration = std::max(1e-6, upper->t - before.t);
  const double alpha = std::max(0.0, std::min(1.0, (target_time - before.t) / duration));
  return normalizeAngle(before.yaw + alpha * normalizeAngle(upper->yaw - before.yaw));
}

bool queryFootprintEsdf(
  const ats_rc_esdf::RcTraversabilityEsdfProvider & esdf,
  const Point & position,
  double yaw,
  const std::vector<Point> & samples,
  ats_rc_esdf::EsdfQueryResult & result)
{
  result = ats_rc_esdf::EsdfQueryResult {};
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  bool found = false;
  for (const Point & sample : samples) {
    ats_rc_esdf::EsdfQueryResult query;
    const Point world = position + Point(
      cos_yaw * sample.x() - sin_yaw * sample.y(),
      sin_yaw * sample.x() + cos_yaw * sample.y());
    if (!esdf.query(world.x(), world.y(), query) || !std::isfinite(query.distance)) {
      continue;
    }
    if (!found || query.distance < result.distance) {
      result = query;
      found = true;
    }
  }
  return found;
}

double minimumSampleClearance(
  const MincoS3 & minco,
  double sample_dt,
  const ats_rc_esdf::RcTraversabilityEsdfProvider & esdf,
  bool footprint_aware,
  const ReferenceTrajectory * footprint_orientation,
  const std::vector<Point> & footprint_samples)
{
  double minimum = std::numeric_limits<double>::infinity();
  double elapsed_duration = 0.0;
  const double total_duration = std::max(1e-6, [&minco]() {
      double total = 0.0;
      for (int piece = 0; piece < minco.pieceCount(); ++piece) {
        total += minco.pieceDuration(piece);
      }
      return total;
    }());
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const double duration = minco.pieceDuration(piece);
    const int steps = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
    for (int step = 0; step <= steps; ++step) {
      const double local_time = duration * static_cast<double>(step) / steps;
      const MincoSample sample = minco.sample(piece, local_time);
      ats_rc_esdf::EsdfQueryResult query;
      const bool query_ok = footprint_aware ? queryFootprintEsdf(
        esdf, sample.position, interpolateReferenceYaw(
          *footprint_orientation, (elapsed_duration + local_time) / total_duration),
        footprint_samples, query) : esdf.query(sample.position.x(), sample.position.y(), query);
      if (!query_ok || !std::isfinite(query.distance)) {
        return -std::numeric_limits<double>::infinity();
      }
      minimum = std::min(minimum, query.distance);
    }
    elapsed_duration += duration;
  }
  return minimum;
}

std::vector<Point> refineWaypointsWithEsdf(
  const std::vector<Point> & input_waypoints,
  const MincoTrajectoryOptimizerParams & params,
  const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation,
  const Eigen::Matrix<double, 2, 3> & head_state)
{
  if (!params.esdf_obstacle_optimization_enabled || !esdf || !esdf->available()) {
    return input_waypoints;
  }

  // 有 yaw 参考时查询旋转后的矩形采样点；否则保持兼容的质心 ESDF 修正。
  const bool footprint_aware = params.esdf_footprint_optimization_enabled &&
    footprint_orientation && !footprint_orientation->empty();
  const double compatibility_clearance = std::max(0.0, footprint_aware ?
    params.esdf_footprint_clearance : params.esdf_obstacle_clearance);
  const double configured_trigger = footprint_aware ?
    params.esdf_footprint_trigger_clearance : params.esdf_obstacle_trigger_clearance;
  const double configured_target = footprint_aware ?
    params.esdf_footprint_target_clearance : params.esdf_obstacle_target_clearance;
  const double target_clearance = configured_target > 0.0 ?
    configured_target : compatibility_clearance;
  const double trigger_clearance = configured_trigger > 0.0 ?
    std::min(configured_trigger, target_clearance) : target_clearance;
  const double maximum_step = std::max(0.0, params.esdf_obstacle_max_step);
  const double trust_region = std::max(0.0, params.esdf_obstacle_trust_region);
  const double maximum_deviation = std::max(0.0, params.esdf_obstacle_max_deviation);
  if (input_waypoints.size() < 2 || target_clearance <= 0.0 || maximum_step <= 0.0) {
    return input_waypoints;
  }

  // Sample the continuous MINCO curve first.  Sampling itself must not turn a
  // free-space straight line into a sequence of hard interpolation constraints.
  // Internal controls are inserted only after a real clearance trigger needs
  // geometric freedom to move an otherwise endpoint-only segment.
  std::vector<Point> original_waypoints = input_waypoints;
  std::vector<Point> waypoints = original_waypoints;
  bool inserted_clearance_controls = false;
  const double reference_speed = std::max(0.05, params.reference_speed);
  const double sample_dt = std::max(0.02, params.sample_spacing * 0.5) / reference_speed;
  const MincoTimeAllocator time_allocator = makeTimeAllocator(params);
  const std::vector<Point> footprint_samples = footprint_aware ?
    makeRectangularFootprintSamples(
    params.footprint_length, params.footprint_width, params.footprint_safety_margin,
    params.esdf_footprint_sample_spacing) : std::vector<Point> {};

  // 每轮先解出连续 MINCO，再把低净空采样点的梯度分配到相邻内部控制点。
  for (int iteration = 0; iteration < std::max(0, params.esdf_obstacle_max_iterations);
    ++iteration)
  {
    const MincoTimeAllocation allocation = time_allocator.allocate(waypoints, head_state.col(1).norm());
    if (!allocation.valid) {
      break;
    }
    const Eigen::VectorXd durations = allocation.durations;
    MincoS3 minco;
    // 净空修正必须在与最终轨迹相同的首端边界下评估：带初速时首段形状会外扩，
    // 若这里仍按零初速求解，修正量就落在一条实际不会被执行的曲线上。
    if (!solveMinco(waypoints, durations, minco, head_state)) {
      break;
    }
    const double total_duration = std::max(1e-6, durations.sum());
    double elapsed_duration = 0.0;

    std::vector<Point> corrections(waypoints.size(), Point::Zero());
    std::vector<double> weights(waypoints.size(), 0.0);
    bool needs_correction = false;
    for (int piece = 0; piece < minco.pieceCount(); ++piece) {
      const double duration = minco.pieceDuration(piece);
      const int steps = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
      for (int step = 1; step < steps; ++step) {
        const double fraction = static_cast<double>(step) / steps;
        const MincoSample sample = minco.sample(piece, duration * fraction);
        ats_rc_esdf::EsdfQueryResult query;
        const bool query_ok = footprint_aware ? queryFootprintEsdf(
          *esdf, sample.position,
          interpolateReferenceYaw(*footprint_orientation,
          (elapsed_duration + duration * fraction) / total_duration),
          footprint_samples, query) :
          esdf->query(sample.position.x(), sample.position.y(), query);
        if (!query_ok || query.distance >= trigger_clearance ||
          query.gradient.squaredNorm() < 1e-10)
        {
          continue;
        }

        Point correction_direction = query.gradient.normalized();
        if (sample.velocity.squaredNorm() > 1e-10) {
          const Point tangent = sample.velocity.normalized();
          const Point normal = correction_direction - tangent * correction_direction.dot(tangent);
          if (normal.squaredNorm() > 1e-10) {
            correction_direction = normal.normalized();
          }
        }
        const double bounded_step = std::min(
          maximum_step, trust_region > 0.0 ? trust_region : maximum_step);
        const Point correction = correction_direction * std::min(
          bounded_step, 0.5 * (target_clearance - query.distance));
        const std::size_t start_index = static_cast<std::size_t>(piece);
        const std::size_t end_index = start_index + 1U;
        corrections[start_index] += (1.0 - fraction) * correction;
        corrections[end_index] += fraction * correction;
        weights[start_index] += 1.0 - fraction;
        weights[end_index] += fraction;
        needs_correction = true;
      }
      elapsed_duration += duration;
    }
    if (!needs_correction) {
      break;
    }

    if (!inserted_clearance_controls && waypoints.size() == 2U) {
      const std::vector<Point> densified = densifyWaypoints(
        input_waypoints, std::max(0.05, params.esdf_obstacle_control_point_spacing));
      if (densified.size() > waypoints.size()) {
        original_waypoints = densified;
        waypoints = densified;
        inserted_clearance_controls = true;
        continue;
      }
    }

    std::vector<Point> smoothed_corrections = corrections;
    const double smoothing_weight = std::max(0.0, std::min(0.5, params.esdf_obstacle_smoothing_weight));
    // First- and second-neighbour smoothing makes independently measured ESDF
    // normals a compact deformation rather than a point-wise zig-zag.
    for (std::size_t index = 1; index + 1 < corrections.size(); ++index) {
      if (weights[index] <= 1e-9) {
        continue;
      }
      Point previous = Point::Zero();
      if (weights[index - 1U] > 1e-9) {
        previous = corrections[index - 1U] / weights[index - 1U];
      }
      const Point current = corrections[index] / weights[index];
      Point next = Point::Zero();
      if (weights[index + 1U] > 1e-9) {
        next = corrections[index + 1U] / weights[index + 1U];
      }
      smoothed_corrections[index] = weights[index] * (
        (1.0 - 2.0 * smoothing_weight) * current + smoothing_weight * (previous + next));
    }

    std::vector<Point> proposed = waypoints;
    bool proposed_change = false;
    // 首尾点锁定为任务起终点，只允许移动内部点，并限制相对 JPS 引导线的偏离。
    for (std::size_t index = 1; index + 1 < waypoints.size(); ++index) {
      if (weights[index] <= 1e-9) {
        continue;
      }
      const Point step = limitNorm(smoothed_corrections[index] / weights[index], maximum_step);
      Point candidate = waypoints[index] + step;
      candidate = original_waypoints[index] + limitNorm(
        candidate - original_waypoints[index], maximum_deviation);
      if ((candidate - waypoints[index]).norm() > 1e-6) {
        proposed[index] = candidate;
        proposed_change = true;
      }
    }
    if (!proposed_change) {
      break;
    }
    const double current_minimum = minimumSampleClearance(
      minco, sample_dt, *esdf, footprint_aware, footprint_orientation, footprint_samples);
    bool accepted = false;
    for (int backtrack = 0; backtrack <= std::max(0, params.esdf_obstacle_backtracking_steps);
      ++backtrack)
    {
      const double scale = std::ldexp(1.0, -backtrack);
      std::vector<Point> trial = waypoints;
      for (std::size_t index = 1; index + 1 < trial.size(); ++index) {
        trial[index] += scale * (proposed[index] - waypoints[index]);
      }
      const MincoTimeAllocation trial_allocation = time_allocator.allocate(
        trial, head_state.col(1).norm());
      MincoS3 trial_minco;
      if (!trial_allocation.valid || !solveMinco(trial, trial_allocation.durations, trial_minco, head_state)) {
        continue;
      }
      const double trial_minimum = minimumSampleClearance(
        trial_minco, sample_dt, *esdf, footprint_aware, footprint_orientation, footprint_samples);
      if (trial_minimum + 1e-6 >= current_minimum) {
        waypoints = std::move(trial);
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      break;
    }
  }
  return waypoints;
}

void findDynamicExtrema(
  const MincoS3 & minco,
  double sample_spacing,
  double reference_speed,
  double & max_velocity,
  double & max_acceleration,
  double & max_jerk,
  std::vector<double> & segment_peak_velocities,
  std::vector<double> & segment_peak_accelerations,
  std::vector<double> & segment_peak_jerks)
{
  max_velocity = 0.0;
  max_acceleration = 0.0;
  max_jerk = 0.0;
  segment_peak_velocities.assign(static_cast<std::size_t>(minco.pieceCount()), 0.0);
  segment_peak_accelerations.assign(static_cast<std::size_t>(minco.pieceCount()), 0.0);
  segment_peak_jerks.assign(static_cast<std::size_t>(minco.pieceCount()), 0.0);
  const double sample_dt = sample_spacing / std::max(0.05, reference_speed);
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const int steps = std::max(
      2, static_cast<int>(std::ceil(minco.pieceDuration(piece) / sample_dt)));
    for (int step = 0; step <= steps; ++step) {
      const auto sample = minco.sample(
        piece, minco.pieceDuration(piece) * static_cast<double>(step) / steps);
      max_velocity = std::max(max_velocity, sample.velocity.norm());
      max_acceleration = std::max(max_acceleration, sample.acceleration.norm());
      max_jerk = std::max(max_jerk, sample.jerk.norm());
      segment_peak_velocities[static_cast<std::size_t>(piece)] = std::max(
        segment_peak_velocities[static_cast<std::size_t>(piece)], sample.velocity.norm());
      segment_peak_accelerations[static_cast<std::size_t>(piece)] = std::max(
        segment_peak_accelerations[static_cast<std::size_t>(piece)], sample.acceleration.norm());
      segment_peak_jerks[static_cast<std::size_t>(piece)] = std::max(
        segment_peak_jerks[static_cast<std::size_t>(piece)], sample.jerk.norm());
    }
  }
}

}  // namespace

MincoTrajectoryOptimizer::MincoTrajectoryOptimizer(MincoTrajectoryOptimizerParams params)
: params_(params)
{
}

void MincoTrajectoryOptimizer::setParams(const MincoTrajectoryOptimizerParams & params)
{
  params_ = params;
}

bool MincoTrajectoryOptimizer::esdfObstacleOptimizationEnabled() const
{
  return params_.esdf_obstacle_optimization_enabled;
}

bool MincoTrajectoryOptimizer::esdfFootprintOptimizationEnabled() const
{
  return params_.esdf_obstacle_optimization_enabled && params_.esdf_footprint_optimization_enabled;
}

ReferenceTrajectory MincoTrajectoryOptimizer::optimize(
  const nav_msgs::msg::Path & raw_path,
  const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation,
  const InitialKinematicState * initial_state,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker,
  MincoOptimizationTrace * trace) const
{
  ReferenceTrajectory trajectory;
  const auto optimization_started = std::chrono::steady_clock::now();
  trajectory.header = raw_path.header;
  if (trace) {
    *trace = MincoOptimizationTrace();
  }
  const auto finishTrace = [trace, optimization_started](const std::string & failure_reason) {
      if (!trace) {
        return;
      }
      trace->failure_reason = failure_reason;
      trace->solver_wall_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - optimization_started).count();
    };
  const auto recordDynamicTrace = [trace](
      const Eigen::VectorXd & durations, double peak_velocity, double peak_acceleration,
      double peak_jerk) {
      if (!trace) {
        return;
      }
      trace->segment_durations.clear();
      trace->segment_durations.reserve(static_cast<std::size_t>(durations.size()));
      for (int index = 0; index < durations.size(); ++index) {
        trace->segment_durations.push_back(durations(index));
      }
      trace->peak_velocity = peak_velocity;
      trace->peak_acceleration = peak_acceleration;
      trace->peak_jerk = peak_jerk;
    };
  PathGeometryPreprocessor preprocessor(params_.geometry_preprocessor);
  const PathGeometryResult preprocessing = preprocessor.preprocess(
    raw_path, planning_grid, safety_checker);
  std::vector<Point> waypoints = preprocessing.waypoints;
  if (trace) {
    trace->preprocessed_guide = preprocessing.guide_path;
  }
  if (waypoints.empty()) {
    finishTrace("geometry_preprocess_empty");
    return trajectory;
  }
  if (waypoints.size() == 1) {
    ReferencePoint point;
    point.x = waypoints.front().x();
    point.y = waypoints.front().y();
    trajectory.points.push_back(point);
    finishTrace("");
    return trajectory;
  }

  const double reference_speed = std::max(0.05, params_.reference_speed);
  const double sample_spacing = std::max(0.02, params_.sample_spacing);
  const Eigen::Matrix<double, 2, 3> head_state = makeHeadState(initial_state, params_);
  const std::vector<Point> pre_refinement_waypoints = waypoints;
  waypoints = refineWaypointsWithEsdf(waypoints, params_, esdf, footprint_orientation, head_state);
  if (trace) {
    trace->esdf_refined_guide = makeGuidePath(raw_path.header, waypoints);
    trace->esdf_geometry_refined = waypoints.size() == pre_refinement_waypoints.size() &&
      !std::equal(waypoints.begin(), waypoints.end(), pre_refinement_waypoints.begin(),
      [](const Point & first, const Point & second) {return (first - second).norm() <= 1e-6;});
  }
  const MincoTimeAllocator time_allocator = makeTimeAllocator(params_);
  const MincoTimeAllocation initial_allocation = time_allocator.allocate(
    waypoints, head_state.col(1).norm());
  if (!initial_allocation.valid) {
    finishTrace("time_allocation_invalid");
    return trajectory;
  }
  Eigen::VectorXd durations = initial_allocation.durations;
  MincoS3 minco;
  if (!solveMinco(waypoints, durations, minco, head_state)) {
    finishTrace("minco_s3_initial_solve_failed");
    return trajectory;
  }

  bool dynamic_limits_satisfied = false;
  bool local_time_scaled = false;
  bool uniform_time_scaled = false;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_jerk = 0.0;
  std::vector<double> segment_peak_velocities;
  std::vector<double> segment_peak_accelerations;
  std::vector<double> segment_peak_jerks;
  // Segment-wise scaling keeps a high-curvature corner slow without globally
  // stretching unrelated straight segments.  MINCO is re-solved each round.
  const int maximum_scaling_iterations = std::max(0, params_.max_time_scaling_iterations);
  for (int iteration = 0; iteration <= maximum_scaling_iterations;
    ++iteration)
  {
    peak_velocity = 0.0;
    peak_acceleration = 0.0;
    peak_jerk = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration, peak_jerk,
      segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks);
    recordDynamicTrace(durations, peak_velocity, peak_acceleration, peak_jerk);
    const bool velocity_ok = params_.max_velocity <= 0.0 ||
      peak_velocity <= params_.max_velocity + 1e-6;
    const bool acceleration_ok = params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6;
    const bool jerk_ok = params_.max_jerk <= 0.0 || peak_jerk <= params_.max_jerk + 1e-6;
    if (velocity_ok && acceleration_ok && jerk_ok) {
      dynamic_limits_satisfied = true;
      break;
    }
    if (iteration == maximum_scaling_iterations) {
      break;
    }
    if (!time_allocator.applyLocalDynamicScaling(
        durations, segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks,
        params_.max_velocity, params_.max_acceleration, params_.max_jerk))
    {
      break;
    }
    local_time_scaled = true;
    if (!solveMinco(waypoints, durations, minco, head_state)) {
      trajectory.points.clear();
      finishTrace("minco_s3_scaled_solve_failed");
      return trajectory;
    }
  }
  if (!dynamic_limits_satisfied) {
    double required_scale = 1.0;
    if (params_.max_velocity > 0.0) {
      required_scale = std::max(required_scale, peak_velocity / params_.max_velocity);
    }
    if (params_.max_acceleration > 0.0) {
      required_scale = std::max(
        required_scale, std::sqrt(peak_acceleration / params_.max_acceleration));
    }
    if (params_.max_jerk > 0.0) {
      required_scale = std::max(
        required_scale, std::cbrt(peak_jerk / params_.max_jerk));
    }
    const double uniform_scale = std::max(
      std::max(1.01, params_.time_scaling_factor), 1.01 * required_scale);
    durations *= uniform_scale;
    if (!solveMinco(waypoints, durations, minco, head_state)) {
      finishTrace("minco_s3_uniform_scaled_solve_failed");
      return trajectory;
    }
    peak_velocity = 0.0;
    peak_acceleration = 0.0;
    peak_jerk = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration, peak_jerk,
      segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks);
    recordDynamicTrace(durations, peak_velocity, peak_acceleration, peak_jerk);
    dynamic_limits_satisfied =
      (params_.max_velocity <= 0.0 || peak_velocity <= params_.max_velocity + 1e-6) &&
      (params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6) &&
      (params_.max_jerk <= 0.0 || peak_jerk <= params_.max_jerk + 1e-6);
    uniform_time_scaled = true;
  }
  if (!dynamic_limits_satisfied) {
    finishTrace("dynamic_limits_unsatisfied");
    return trajectory;
  }
  if (trace) {
    trace->local_time_scaled = local_time_scaled;
    trace->uniform_time_scaled = uniform_time_scaled;
    trace->failure_reason.clear();
  }
  finishTrace("");

  double accumulated_time = 0.0;
  double accumulated_distance = 0.0;
  Point previous_position = waypoints.front();
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const double duration = minco.pieceDuration(piece);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(duration * reference_speed / sample_spacing)));
    const int first_step = piece == 0 ? 0 : 1;
    for (int step = first_step; step <= steps; ++step) {
      const double local_time = duration * static_cast<double>(step) / steps;
      const MincoSample sample = minco.sample(piece, local_time);
      if (!trajectory.points.empty()) {
        accumulated_distance += (sample.position - previous_position).norm();
      }
      ReferencePoint point;
      point.t = accumulated_time + local_time;
      point.s = accumulated_distance;
      point.x = sample.position.x();
      point.y = sample.position.y();
      point.vx = sample.velocity.x();
      point.vy = sample.velocity.y();
      point.ax = sample.acceleration.x();
      point.ay = sample.acceleration.y();
      point.v = sample.velocity.norm();
      trajectory.points.push_back(point);
      previous_position = sample.position;
    }
    accumulated_time += duration;
  }
  return trajectory;
}

}  // namespace minco_planner
