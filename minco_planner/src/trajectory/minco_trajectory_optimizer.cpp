// Copyright 2026

#include "minco_trajectory_optimizer_helpers.hpp"

namespace minco_planner
{

using namespace optimizer_detail;

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

ReferenceTrajectory MincoTrajectoryOptimizer::optimizeOnce(
  const nav_msgs::msg::Path & raw_path,
  const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation,
  const InitialKinematicState * initial_state,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker,
  MincoOptimizationTrace * trace,
  const TerminalKinematicState * terminal_state,
  const std::vector<double> * precomputed_blocked_squared,
  bool allow_shortcut) const
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
    raw_path,
    allow_shortcut ? planning_grid : nullptr,
    allow_shortcut ? safety_checker : nullptr,
    allow_shortcut ? precomputed_blocked_squared : nullptr);
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
  Eigen::Matrix<double, 2, 3> head_state = Eigen::Matrix<double, 2, 3>::Zero();
  if (!makeHeadState(initial_state, params_, head_state)) {
    finishTrace("non_finite_initial_state");
    return trajectory;
  }
  Eigen::Matrix<double, 2, 3> tail_state = Eigen::Matrix<double, 2, 3>::Zero();
  if (!makeTailState(terminal_state, params_, tail_state)) {
    finishTrace("non_finite_terminal_state");
    return trajectory;
  }
  const bool has_tail_state = terminal_state != nullptr && terminal_state->valid;
  const double guide_spacing = std::max(0.0, params_.guide_control_point_spacing);
  if (guide_spacing > 1e-6) {
    waypoints = densifyWaypoints(waypoints, guide_spacing);
  }
  waypoints = smoothGuideWithEsdf(waypoints, params_, esdf);
  const std::vector<Point> pre_refinement_waypoints = waypoints;
  waypoints = refineWaypointsWithEsdf(
    waypoints, params_, esdf, footprint_orientation, head_state,
    has_tail_state ? &tail_state : nullptr);
  if (guide_spacing > 1e-6) {
    waypoints = densifyWaypoints(waypoints, guide_spacing);
  }
  if (waypoints.size() >= 2U) {
    Point direction = waypoints[1] - waypoints.front();
    const double length = direction.norm();
    if (length > 1e-6) {
      direction /= length;
      const double along = head_state.col(1).dot(direction);
      if (along < 0.0) {
        head_state.col(1) -= direction * along;
      }
    }
    if (has_tail_state && !terminal_state->exact_splice) {
      Point tail_direction = waypoints.back() - waypoints[waypoints.size() - 2U];
      const double tail_length = tail_direction.norm();
      if (tail_length > 1e-6) {
        tail_direction /= tail_length;
        const double along_tail = tail_state.col(1).dot(tail_direction);
        if (along_tail < 0.0) {
          tail_state.col(1) -= tail_direction * along_tail;
        }
      }
    }
  }
  if (trace) {
    trace->esdf_refined_guide = makeGuidePath(raw_path.header, waypoints);
    trace->esdf_geometry_refined = waypoints.size() == pre_refinement_waypoints.size() &&
      !std::equal(waypoints.begin(), waypoints.end(), pre_refinement_waypoints.begin(),
      [](const Point & first, const Point & second) {return (first - second).norm() <= 1e-6;});
  }
  // 整条轨迹联合优化：在加密引导上按弧长重采样内点，时长初值取分配器结果，
  // 全部内点与全部时长在一个 L-BFGS 问题里同时优化。
  const std::vector<Point> dense_guide = waypoints;
  const double effective_joint_spacing = allow_shortcut ?
    std::max(0.05, params_.joint_waypoint_spacing) :
    std::max(0.05, std::min(params_.joint_waypoint_spacing, 0.50));
  waypoints = MincoJointOptimizer::resampleByArcLength(waypoints, effective_joint_spacing);
  const MincoTimeAllocator time_allocator = makeTimeAllocator(params_);
  std::vector<double> waypoint_speed_caps;
  const std::vector<double> * speed_caps_ptr = nullptr;
  if (esdf && esdf->available() && params_.clearance_speed_min > 0.0 &&
    params_.clearance_speed_high > params_.clearance_speed_low)
  {
    const double alloc_v_max = params_.max_velocity > 0.0 ?
      std::min(params_.max_velocity, std::max(0.05, params_.reference_speed)) :
      std::max(0.05, params_.reference_speed);
    waypoint_speed_caps.reserve(waypoints.size());
    for (const Point & wp : waypoints) {
      const double dist = esdf->getDistance(wp.x(), wp.y());
      waypoint_speed_caps.push_back(
        clearanceSpeedLimitAndDerivative(
          dist, params_.clearance_speed_min, params_.clearance_speed_low,
          params_.clearance_speed_high, alloc_v_max));
    }
    speed_caps_ptr = &waypoint_speed_caps;
  }
  const MincoTimeAllocation initial_allocation = time_allocator.allocate(
    waypoints, head_state.col(1).norm(), has_tail_state ? tail_state.col(1).norm() : 0.0,
    speed_caps_ptr);
  if (!initial_allocation.valid) {
    finishTrace("time_allocation_invalid");
    return trajectory;
  }
  const MincoJointOptimizer joint_optimizer(makeJointParams(params_));
  const bool joint_obstacle = esdf && esdf->available() && esdfObstacleOptimizationEnabled();
  const bool joint_footprint = joint_obstacle && esdfFootprintOptimizationEnabled() &&
    footprint_orientation && !footprint_orientation->points.empty();
  JointDistanceQuery distance_query;
  // 值与梯度同源的距离场（见 consistent_distance_field.hpp）；几何不符时回退 provider 查询。
  ConsistentDistanceField consistent_field;
  JointDistanceFieldMode field_mode = JointDistanceFieldMode::kLegacy;
  const bool consistent_ready = joint_obstacle && planning_grid != nullptr &&
    parseJointDistanceFieldMode(params_.joint_distance_field, field_mode) &&
    consistent_field.build(*esdf, *planning_grid, field_mode);
  if (consistent_ready) {
    distance_query = [&consistent_field](const Point & position, double & distance, Point & gradient) {
        return consistent_field.query(position, distance, gradient);
      };
  } else if (joint_obstacle) {
    distance_query = [esdf](const Point & position, double & distance, Point & gradient) {
        ats_rc_esdf::EsdfQueryResult query;
        if (!esdf->query(position.x(), position.y(), query) || !std::isfinite(query.distance) ||
          !query.gradient.allFinite())
        {
          return false;
        }
        distance = query.distance;
        gradient = query.gradient;
        return true;
      };
  }
  JointYawQuery yaw_query;
  if (joint_footprint) {
    yaw_query = [footprint_orientation](double progress) {
        return interpolateReferenceYaw(*footprint_orientation, progress);
      };
  }
  const MincoJointResult joint = joint_optimizer.optimize(
    head_state, waypoints, initial_allocation.durations, distance_query, yaw_query,
    joint_obstacle ? std::vector<Point>() : dense_guide,
    has_tail_state ? &tail_state : nullptr);
  if (trace) {
    trace->joint_piece_count = static_cast<int>(initial_allocation.durations.size());
    trace->joint_iterations = joint.iterations;
    trace->joint_evaluations = joint.evaluations;
    trace->joint_termination = joint.termination;
    trace->joint_initial_cost = joint.initial_cost;
    trace->joint_final_cost = joint.final_cost;
    trace->joint_wall_time_ms = joint.wall_time_ms;
    trace->joint_constraints_satisfied = joint.constraints_satisfied;
    trace->joint_max_center_violation = joint.stats.max_center_violation;
    trace->joint_max_footprint_violation = joint.stats.max_footprint_violation;
    trace->joint_max_guide_excess = joint.stats.max_guide_excess;
    trace->joint_max_velocity_excess = joint.stats.max_velocity_excess;
    trace->joint_max_acceleration_excess = joint.stats.max_acceleration_excess;
    trace->joint_max_lateral_excess = joint.stats.max_lateral_excess;
  }
  if (!joint.solved) {
    finishTrace("joint_optimization_failed");
    return trajectory;
  }
  waypoints = joint.waypoints;
  Eigen::VectorXd durations = joint.durations;
  MincoS3 minco;
  if (!solveMinco(waypoints, durations, minco, head_state, has_tail_state ? &tail_state : nullptr)) {
    finishTrace("minco_s3_initial_solve_failed");
    return trajectory;
  }

  bool dynamic_limits_satisfied = false;
  bool local_time_scaled = false;
  bool uniform_time_scaled = false;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_jerk = 0.0;
  double peak_lateral = 0.0;
  std::vector<double> segment_peak_velocities;
  std::vector<double> segment_peak_accelerations;
  std::vector<double> segment_peak_jerks;
  std::vector<double> segment_peak_laterals;
  const auto effectiveAccelerations = [&](const std::vector<double> & laterals) {
      std::vector<double> scaled = segment_peak_accelerations;
      if (params_.max_lateral_acceleration > 0.0 && params_.max_acceleration > 0.0) {
        for (std::size_t index = 0; index < scaled.size() && index < laterals.size(); ++index) {
          scaled[index] = std::max(
            scaled[index],
            laterals[index] * params_.max_acceleration / params_.max_lateral_acceleration);
        }
      }
      return scaled;
    };
  // Segment-wise scaling keeps a high-curvature corner slow without globally
  // stretching unrelated straight segments.  MINCO is re-solved each round.
  const int maximum_scaling_iterations = std::max(0, params_.max_time_scaling_iterations);
  // A fixed nonzero boundary velocity breaks exact inverse-time scaling.
  // Reserve one existing iteration for a second coupled solve, rather than
  // spending every iteration locally and assuming one uniform pass is enough.
  const int local_scaling_iterations = std::max(0, maximum_scaling_iterations - 1);
  const int uniform_scaling_iterations = maximum_scaling_iterations - local_scaling_iterations + 1;
  // 固定的非零首端速度下，拉长时长并不一定让速度峰值下降：首段被压到最小时长、第二段很长时，
  // 峰值速度趋于与种子速度成正比的常数（2025 红框 d221：种子 2.03 m/s、T1/T0 = 27.7，
  // 峰值 1.99 -> 8.3 -> 13.9 -> 16.0 -> 27.9 m/s，段时长一路乘到 1e3-3e4 s）。循环若继续按
  // peak/limit 拉长只会把时长推到不可用的量级，所以连续两轮速度都超限且峰值没降时停止，
  // 交给 optimize 的首端种子回退去处理。判据表达式一字未改，这里只决定“要不要继续拉长”。
  bool scaling_non_contractive = false;
  double previous_round_peak_velocity = 0.0;
  for (int iteration = 0; iteration <= local_scaling_iterations;
    ++iteration)
  {
    peak_velocity = 0.0;
    peak_acceleration = 0.0;
    peak_jerk = 0.0;
    peak_lateral = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration, peak_jerk,
      peak_lateral, segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks,
      segment_peak_laterals);
    recordDynamicTrace(durations, peak_velocity, peak_acceleration, peak_jerk);
    const bool velocity_ok = params_.max_velocity <= 0.0 ||
      peak_velocity <= params_.max_velocity + 1e-6;
    const bool acceleration_ok = params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6;
    const bool jerk_ok = params_.max_jerk <= 0.0 || peak_jerk <= params_.max_jerk + 1e-6;
    const bool lateral_ok = params_.max_lateral_acceleration <= 0.0 ||
      peak_lateral <= params_.max_lateral_acceleration + 1e-6;
    if (velocity_ok && acceleration_ok && jerk_ok && lateral_ok) {
      dynamic_limits_satisfied = true;
      break;
    }
    if (iteration == local_scaling_iterations) {
      break;
    }
    if (iteration > 0 && !velocity_ok && params_.max_velocity > 0.0 &&
      previous_round_peak_velocity > params_.max_velocity + 1e-6 &&
      peak_velocity >= previous_round_peak_velocity)
    {
      scaling_non_contractive = true;
      break;
    }
    previous_round_peak_velocity = peak_velocity;
    if (!time_allocator.applyLocalDynamicScaling(
        durations, segment_peak_velocities, effectiveAccelerations(segment_peak_laterals),
        segment_peak_jerks,
        params_.max_velocity, params_.max_acceleration, params_.max_jerk))
    {
      break;
    }
    local_time_scaled = true;
    if (!solveMinco(waypoints, durations, minco, head_state, has_tail_state ? &tail_state : nullptr)) {
      trajectory.points.clear();
      finishTrace("minco_s3_scaled_solve_failed");
      return trajectory;
    }
  }
  for (int iteration = 0;
    !dynamic_limits_satisfied && !scaling_non_contractive &&
    iteration < uniform_scaling_iterations;
    ++iteration)
  {
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
    if (params_.max_lateral_acceleration > 0.0) {
      required_scale = std::max(
        required_scale, std::sqrt(peak_lateral / params_.max_lateral_acceleration));
    }
    const double uniform_scale = std::max(
      std::max(1.01, params_.time_scaling_factor), 1.01 * required_scale);
    durations *= uniform_scale;
    if (!solveMinco(waypoints, durations, minco, head_state, has_tail_state ? &tail_state : nullptr)) {
      finishTrace("minco_s3_uniform_scaled_solve_failed");
      return trajectory;
    }
    peak_velocity = 0.0;
    peak_acceleration = 0.0;
    peak_jerk = 0.0;
    peak_lateral = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration, peak_jerk,
      peak_lateral, segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks,
      segment_peak_laterals);
    recordDynamicTrace(durations, peak_velocity, peak_acceleration, peak_jerk);
    dynamic_limits_satisfied =
      (params_.max_velocity <= 0.0 || peak_velocity <= params_.max_velocity + 1e-6) &&
      (params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6) &&
      (params_.max_jerk <= 0.0 || peak_jerk <= params_.max_jerk + 1e-6) &&
      (params_.max_lateral_acceleration <= 0.0 ||
      peak_lateral <= params_.max_lateral_acceleration + 1e-6);
    uniform_time_scaled = true;
  }
  if (!dynamic_limits_satisfied) {
    finishTrace(scaling_non_contractive ? "dynamic_scaling_non_contractive" :
      "dynamic_limits_unsatisfied");
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

// 首端种子回退。测得的首端速度只是前馈提示，动态限幅门禁（max_velocity/acceleration/jerk/
// 横向加速度）、矩形足迹门禁与提交门作用在最终轨迹上，一项都没放宽。完整种子让时间缩放
// 发散（dynamic_limits_unsatisfied 或 dynamic_scaling_non_contractive）时，把种子速度和加速度
// 按 {0.5, 0.25, 0} 缩小后重解整条优化；0 档等价于“静止起步”的历史行为。
// 触发条件极窄：种子非零且失败原因是动态限幅，其余输入与失败原因都只跑一次，行为不变。
// 回退会让新参考的起始前馈小于里程计速度；这类阶跃在 makeHeadState 的限幅里本来就存在，
// 并且只发生在原本必败的请求上。
ReferenceTrajectory MincoTrajectoryOptimizer::optimize(
  const nav_msgs::msg::Path & raw_path,
  const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation,
  const InitialKinematicState * initial_state,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker,
  MincoOptimizationTrace * trace,
  const TerminalKinematicState * terminal_state,
  const std::vector<double> * precomputed_blocked_squared,
  bool allow_shortcut) const
{
  static constexpr double kSeedLadder[] = {1.0, 0.5, 0.25, 0.0};
  const bool head_seeded = initial_state != nullptr && initial_state->valid &&
    (initial_state->velocity.squaredNorm() > 1e-12 ||
    initial_state->acceleration.squaredNorm() > 1e-12);
  const bool tail_seeded = terminal_state != nullptr && terminal_state->valid &&
    (terminal_state->velocity.squaredNorm() > 1e-12 ||
    terminal_state->acceleration.squaredNorm() > 1e-12);
  const bool seeded = head_seeded || tail_seeded;
  ReferenceTrajectory result;
  MincoOptimizationTrace attempt_trace;
  const std::size_t rungs = seeded ? 4U : 1U;
  for (std::size_t rung = 0; rung < rungs; ++rung) {
    InitialKinematicState scaled_head;
    const InitialKinematicState * head_seed = initial_state;
    if (head_seeded && kSeedLadder[rung] != 1.0) {
      scaled_head = *initial_state;
      scaled_head.velocity *= kSeedLadder[rung];
      scaled_head.acceleration *= kSeedLadder[rung];
      head_seed = &scaled_head;
    }
    TerminalKinematicState scaled_tail;
    const TerminalKinematicState * tail_seed = terminal_state;
    if (tail_seeded && kSeedLadder[rung] != 1.0) {
      scaled_tail = *terminal_state;
      scaled_tail.velocity *= kSeedLadder[rung];
      scaled_tail.acceleration *= kSeedLadder[rung];
      tail_seed = &scaled_tail;
    }
    result = optimizeOnce(
      raw_path, esdf, footprint_orientation, head_seed, planning_grid, safety_checker,
      &attempt_trace, tail_seed, precomputed_blocked_squared, allow_shortcut);
    attempt_trace.seed_scale = kSeedLadder[rung];
    const bool dynamic_failure = attempt_trace.failure_reason == "dynamic_limits_unsatisfied" ||
      attempt_trace.failure_reason == "dynamic_scaling_non_contractive";
    if (result.valid() || !dynamic_failure) {
      break;
    }
  }
  if (trace != nullptr) {
    *trace = attempt_trace;
  }
  return result;
}

ReferenceTrajectory MincoTrajectoryOptimizer::buildContinuationTail(
  const nav_msgs::msg::Path & tail_path,
  const InitialKinematicState & junction_state,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker,
  const std::vector<double> * precomputed_blocked_squared) const
{
  ReferenceTrajectory trajectory;
  trajectory.header = tail_path.header;
  PathGeometryPreprocessor preprocessor(params_.geometry_preprocessor);
  const PathGeometryResult preprocessing = preprocessor.preprocess(
    tail_path, planning_grid, safety_checker, precomputed_blocked_squared);
  std::vector<Point> waypoints = preprocessing.waypoints;
  if (waypoints.size() < 2U) {
    return trajectory;
  }
  waypoints = MincoJointOptimizer::resampleByArcLength(
    waypoints, std::max(0.25, params_.joint_waypoint_spacing));
  if (waypoints.size() < 2U) {
    return trajectory;
  }
  Eigen::Matrix<double, 2, 3> head_state = Eigen::Matrix<double, 2, 3>::Zero();
  if (!makeHeadState(&junction_state, params_, head_state)) {
    return trajectory;
  }
  const double reference_speed = std::max(0.05, params_.reference_speed);
  const double sample_spacing = std::max(0.02, params_.sample_spacing);
  const MincoTimeAllocator time_allocator = makeTimeAllocator(params_);
  const MincoTimeAllocation allocation =
    time_allocator.allocate(waypoints, head_state.col(1).norm(), 0.0);
  if (!allocation.valid) {
    return trajectory;
  }
  Eigen::VectorXd durations = allocation.durations;
  MincoS3 minco;
  if (!solveMinco(waypoints, durations, minco, head_state, nullptr)) {
    return trajectory;
  }
  bool dynamic_limits_satisfied = false;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_jerk = 0.0;
  double peak_lateral = 0.0;
  std::vector<double> segment_peak_velocities;
  std::vector<double> segment_peak_accelerations;
  std::vector<double> segment_peak_jerks;
  std::vector<double> segment_peak_laterals;
  const auto effectiveAccelerations = [&](const std::vector<double> & laterals) {
      std::vector<double> scaled = segment_peak_accelerations;
      if (params_.max_lateral_acceleration > 0.0 && params_.max_acceleration > 0.0) {
        for (std::size_t index = 0; index < scaled.size() && index < laterals.size(); ++index) {
          scaled[index] = std::max(
            scaled[index],
            laterals[index] * params_.max_acceleration / params_.max_lateral_acceleration);
        }
      }
      return scaled;
    };
  const int max_iters = std::max(1, params_.max_time_scaling_iterations);
  for (int iter = 0; iter <= max_iters; ++iter) {
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration, peak_jerk,
      peak_lateral, segment_peak_velocities, segment_peak_accelerations, segment_peak_jerks,
      segment_peak_laterals);
    const bool velocity_ok = params_.max_velocity <= 0.0 ||
      peak_velocity <= params_.max_velocity + 1e-6;
    const bool acceleration_ok = params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6;
    const bool jerk_ok = params_.max_jerk <= 0.0 || peak_jerk <= params_.max_jerk + 1e-6;
    const bool lateral_ok = params_.max_lateral_acceleration <= 0.0 ||
      peak_lateral <= params_.max_lateral_acceleration + 1e-6;
    if (velocity_ok && acceleration_ok && jerk_ok && lateral_ok) {
      dynamic_limits_satisfied = true;
      break;
    }
    if (iter == max_iters) {
      break;
    }
    if (!time_allocator.applyLocalDynamicScaling(
        durations, segment_peak_velocities, effectiveAccelerations(segment_peak_laterals),
        segment_peak_jerks, params_.max_velocity, params_.max_acceleration, params_.max_jerk))
    {
      double required_scale = 1.05;
      if (params_.max_velocity > 0.0) {
        required_scale = std::max(required_scale, 1.02 * peak_velocity / params_.max_velocity);
      }
      if (params_.max_acceleration > 0.0) {
        required_scale = std::max(
          required_scale, 1.02 * std::sqrt(peak_acceleration / params_.max_acceleration));
      }
      if (params_.max_jerk > 0.0) {
        required_scale = std::max(
          required_scale, 1.02 * std::cbrt(peak_jerk / params_.max_jerk));
      }
      if (params_.max_lateral_acceleration > 0.0) {
        required_scale = std::max(
          required_scale, 1.02 * std::sqrt(peak_lateral / params_.max_lateral_acceleration));
      }
      durations *= required_scale;
    }
    if (!solveMinco(waypoints, durations, minco, head_state, nullptr)) {
      return trajectory;
    }
  }
  if (!dynamic_limits_satisfied) {
    return trajectory;
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
