// Copyright 2026

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
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

std::vector<Point> extractWaypoints(const nav_msgs::msg::Path & path)
{
  std::vector<Point> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const Point point(pose.pose.position.x, pose.pose.position.y);
    if (points.empty() || (point - points.back()).norm() > 1e-6) {
      points.push_back(point);
    }
  }
  if (points.size() <= 2) {
    return points;
  }

  std::vector<Point> simplified;
  simplified.reserve(points.size());
  simplified.push_back(points.front());
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Point incoming = points[i] - simplified.back();
    const Point outgoing = points[i + 1] - points[i];
    const double cross = incoming.x() * outgoing.y() - incoming.y() * outgoing.x();
    const double scale = std::max(1e-9, incoming.norm() * outgoing.norm());
    if (std::abs(cross) / scale > 1e-3 || incoming.dot(outgoing) <= 0.0) {
      simplified.push_back(points[i]);
    }
  }
  simplified.push_back(points.back());
  return simplified;
}

Eigen::VectorXd allocateDurations(
  const std::vector<Point> & waypoints,
  double reference_speed,
  double min_segment_time)
{
  Eigen::VectorXd durations(static_cast<int>(waypoints.size()) - 1);
  for (int i = 0; i < durations.size(); ++i) {
    durations(i) = std::max(
      min_segment_time,
      (waypoints[static_cast<std::size_t>(i + 1)] -
      waypoints[static_cast<std::size_t>(i)]).norm() / reference_speed);
  }
  return durations;
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
  const double minimum_clearance = std::max(0.0, params.esdf_obstacle_clearance);
  const double footprint_clearance = std::max(0.0, params.esdf_footprint_clearance);
  const double maximum_step = std::max(0.0, params.esdf_obstacle_max_step);
  const double maximum_deviation = std::max(0.0, params.esdf_obstacle_max_deviation);
  const double required_clearance = footprint_aware ? footprint_clearance : minimum_clearance;
  if (input_waypoints.size() < 2 || required_clearance <= 0.0 || maximum_step <= 0.0) {
    return input_waypoints;
  }

  const std::vector<Point> original_waypoints = densifyWaypoints(
    input_waypoints, std::max(0.05, params.esdf_obstacle_control_point_spacing));
  std::vector<Point> waypoints = original_waypoints;
  const double reference_speed = std::max(0.05, params.reference_speed);
  const double sample_dt = std::max(0.02, params.sample_spacing * 0.5) / reference_speed;
  const std::vector<Point> footprint_samples = footprint_aware ?
    makeRectangularFootprintSamples(
    params.footprint_length, params.footprint_width, params.footprint_safety_margin,
    params.esdf_footprint_sample_spacing) : std::vector<Point> {};

  // 每轮先解出连续 MINCO，再把低净空采样点的梯度分配到相邻内部控制点。
  for (int iteration = 0; iteration < std::max(0, params.esdf_obstacle_max_iterations);
    ++iteration)
  {
    const Eigen::VectorXd durations = allocateDurations(
      waypoints, reference_speed, std::max(0.01, params.min_segment_time));
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
        if (!query_ok || query.distance >= required_clearance ||
          query.gradient.squaredNorm() < 1e-10)
        {
          continue;
        }

        const Point correction = query.gradient.normalized() * std::min(
          maximum_step, 0.5 * (required_clearance - query.distance));
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

    bool changed = false;
    // 首尾点锁定为任务起终点，只允许移动内部点，并限制相对 JPS 引导线的偏离。
    for (std::size_t index = 1; index + 1 < waypoints.size(); ++index) {
      if (weights[index] <= 1e-9) {
        continue;
      }
      const Point step = limitNorm(corrections[index] / weights[index], maximum_step);
      Point candidate = waypoints[index] + step;
      candidate = original_waypoints[index] + limitNorm(
        candidate - original_waypoints[index], maximum_deviation);
      if ((candidate - waypoints[index]).norm() > 1e-6) {
        waypoints[index] = candidate;
        changed = true;
      }
    }
    if (!changed) {
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
  double & max_acceleration)
{
  max_velocity = 0.0;
  max_acceleration = 0.0;
  const double sample_dt = sample_spacing / std::max(0.05, reference_speed);
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const int steps = std::max(
      2, static_cast<int>(std::ceil(minco.pieceDuration(piece) / sample_dt)));
    for (int step = 0; step <= steps; ++step) {
      const auto sample = minco.sample(
        piece, minco.pieceDuration(piece) * static_cast<double>(step) / steps);
      max_velocity = std::max(max_velocity, sample.velocity.norm());
      max_acceleration = std::max(max_acceleration, sample.acceleration.norm());
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
  const InitialKinematicState * initial_state) const
{
  ReferenceTrajectory trajectory;
  trajectory.header = raw_path.header;
  std::vector<Point> waypoints = extractWaypoints(raw_path);
  if (waypoints.empty()) {
    return trajectory;
  }
  if (waypoints.size() == 1) {
    ReferencePoint point;
    point.x = waypoints.front().x();
    point.y = waypoints.front().y();
    trajectory.points.push_back(point);
    return trajectory;
  }

  const double reference_speed = std::max(0.05, params_.reference_speed);
  const double sample_spacing = std::max(0.02, params_.sample_spacing);
  const Eigen::Matrix<double, 2, 3> head_state = makeHeadState(initial_state, params_);
  waypoints = refineWaypointsWithEsdf(
    waypoints, params_, esdf, footprint_orientation, head_state);
  Eigen::VectorXd durations = allocateDurations(
    waypoints, reference_speed, std::max(0.01, params_.min_segment_time));
  MincoS3 minco;
  if (!solveMinco(waypoints, durations, minco, head_state)) {
    return trajectory;
  }

  // 若速度或加速度超限，只整体拉长各段时间，不改变已经通过安全检查的几何形状。
  for (int iteration = 0; iteration < std::max(0, params_.max_time_scaling_iterations);
    ++iteration)
  {
    double peak_velocity = 0.0;
    double peak_acceleration = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration);
    const bool velocity_ok = params_.max_velocity <= 0.0 ||
      peak_velocity <= params_.max_velocity + 1e-6;
    const bool acceleration_ok = params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6;
    if (velocity_ok && acceleration_ok) {
      break;
    }
    double scale = std::max(1.01, params_.time_scaling_factor);
    if (!velocity_ok) {
      scale = std::max(scale, peak_velocity / params_.max_velocity);
    }
    if (!acceleration_ok) {
      scale = std::max(scale, std::sqrt(peak_acceleration / params_.max_acceleration));
    }
    durations *= scale;
    if (!solveMinco(waypoints, durations, minco, head_state)) {
      trajectory.points.clear();
      return trajectory;
    }
  }

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
