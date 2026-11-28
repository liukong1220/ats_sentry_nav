#include "ats_swerve_mpc_node_helpers.hpp"

namespace ats_swerve_mpc {

void AtsSwerveMpcNode::onControlTimer() {
  const auto cycle_start = std::chrono::steady_clock::now();
  const std::optional<std::chrono::steady_clock::time_point> previous_cycle_start =
      previous_control_cycle_start_;
  previous_control_cycle_start_ = cycle_start;
  const std::uint64_t cycle_sequence = ++control_cycle_sequence_;
  bool execution_lease_valid = !execution_command_enabled_;
  bool gimbal_valid = !require_gimbal_status_;
  const bool localization_fresh =
      !require_localization_status_ || localization_tracking_.load();
  // 1. 紧急停止检查
  if (!localization_fresh) {
    engageFailStop();
    return;
  }
  if (execution_command_enabled_) {
    const auto lease_check_time = std::chrono::steady_clock::now();
    bool supersede_grace_expired = false;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (supersede_deadline_ && lease_check_time > *supersede_deadline_) {
        // 宽限截止仍没有新 generation 的 EXECUTE：与无宽限时 superseded 的停车相同。
        supersede_deadline_.reset();
        if (active_execution_command_ &&
            active_execution_command_->map_generation == supersede_old_generation_) {
          active_execution_command_.reset();
          supersede_grace_expired = true;
        }
      }
      if (map_ready_ && last_map_ready_signal_ && !mapReadyFreshLocked()) {
        const std::uint64_t retire_floor = ready_lease_generation_ != 0
          ? ready_lease_generation_
          : planner_generation_;
        retired_planner_generation_ = std::max(
          retired_planner_generation_, retire_floor);
        planner_generation_usable_ = false;
        active_execution_command_.reset();
        map_ready_ = false;
      }
      execution_lease_valid = last_execution_command_signal_ &&
        lease_check_time >= *last_execution_command_signal_ &&
        std::chrono::duration<double>(
          lease_check_time - *last_execution_command_signal_).count() <=
          execution_command_timeout_ && active_execution_command_ &&
        activeExecutionValidLocked(lease_check_time) &&
        gimbalExecutionValidLocked(*active_execution_command_);
      gimbal_valid = active_execution_command_ &&
                     gimbalExecutionValidLocked(*active_execution_command_);
    }
    if (supersede_grace_expired) {
      engageFailStop(false);
      return;
    }
    if (!execution_lease_valid || fail_stop_engaged_.load()) {
      engageFailStop();
      return;
    }
  } else if (emergency_stop_watchdog_enabled_ &&
      (emergency_stop_watchdog_.stopRequired() || fail_stop_engaged_.load())) {
    engageFailStop();
    return;
  }
  // 2. 获取当前状态（不可用时必须输出确定性零速度，不能静默 return）
  State current = State::Zero();
  Control measured_velocity = Control::Zero();
  if (!odometryUsable(current, &measured_velocity)) {
    publishZeroCommandForFailure("里程计不可用");
    return;
  }
  // 3. 获取目标、投影、参考序列
  State goal = State::Zero();
  TrajectoryProjection projection;
  std::vector<Se2Reference> references;
  bool trajectory_expired = false;
  double rotation_delay = 0.0;
  double reference_terminal_speed = 0.0;
  double remaining_translation = 0.0;
  // 参考提取（含 trajectory_mutex_ 等待、投影和 horizon 构造）单独计时，
  // 使其不再隐含在 state_trajectory_snapshot 内无法归因。
  const auto reference_extraction_start = std::chrono::steady_clock::now();
  double reference_extraction_ms = 0.0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (trajectory_tracker_.empty()) {
      // 无参考轨迹同样属于失败兜底路径：必须持续发布零速度，
      // 否则底盘会保持上一条 Twist 一直滑行（实车失控的直接来源）。
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "【通信超时】尚未收到有效参考轨迹（话题 '%s'），"
                           "MPC 保持零速度。",
                           trajectory_topic_.c_str());
      publishZeroCommandForFailure(nullptr);
      return;
    }
    goal = trajectory_tracker_.goal();
    rotation_delay = trajectory_tracker_.commandLatencyCompensation();
    reference_terminal_speed =
        trajectory_tracker_.terminalSpeed(terminal_speed_window_sec_);
    projection = trajectory_tracker_.project(current, last_control_(2));
    remaining_translation = trajectory_tracker_.remainingTranslation(projection);
    references = trajectory_tracker_.buildHorizon(
        projection, controller_->config().horizon, controller_->config().dt,
        last_rollout_horizon_scale_);
    trajectory_expired =
        trajectory_deadline_.nanoseconds() > 0 && now() > trajectory_deadline_;
  }
  reference_extraction_ms = elapsedMilliseconds(reference_extraction_start,
                                                std::chrono::steady_clock::now());
  // 4. 判断是否达到目标或轨迹过期
  const double goal_position_error =
      (goal.head<2>() - current.head<2>()).norm();
  // 外移掉头回程（terminal_yaw_relocation）在去程可能经过目标点附近，
  // 用剩余弧长与直线距离的最大值判定终点接近，避免在去程被终点限速锁死。
  const double terminal_approach_distance =
      std::max(goal_position_error, remaining_translation);
  const double goal_yaw_error = std::abs(normalizeAngle(goal(2) - current(2)));
  if (trajectory_expired) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【轨迹过期】参考轨迹已超过有效期，停止跟踪并输出零速度；"
                         "请检查规划器发布频率与时间戳同步。");
    publishZeroCommandForFailure(nullptr);
    return;
  }
  const double nominal_control_period = 1.0 / std::max(1.0, control_rate_hz_);
  const double max_step_dt = std::max(
      controller_->config().dt, 2.0 * nominal_control_period);
  double actual_step_dt = nominal_control_period;
  const bool first_active_cycle = get_clock()->ros_time_is_active()
      ? !last_control_update_stamp_.has_value()
      : !last_control_update_steady_.has_value();
  if (get_clock()->ros_time_is_active()) {
    const auto update_stamp = now();
    if (last_control_update_stamp_) {
      const auto elapsed_ns = (update_stamp - *last_control_update_stamp_).nanoseconds();
      if (elapsed_ns < 0) {
        // A backward clock must not keep driving with a held command.
        const auto previous_update_stamp = *last_control_update_stamp_;
        engageFailStop();
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (last_stop_stamp_ < previous_update_stamp) {
          last_stop_stamp_ = previous_update_stamp;
        }
        // This stop is not authorized by a sequenced Goal Manager STOP.
        // Keep the pre-rewind reference ineligible even with a newer heartbeat.
        last_stop_command_.reset();
        return;
      }
      const auto period_ns = static_cast<std::int64_t>(
        std::llround(1e9 * nominal_control_period));
      if (elapsed_ns < period_ns) {
        // Slow simulation clocks must not advance dynamics/warm-start on every
        // wall tick. All safety/odometry/expiry checks above still run; renew
        // the checked command heartbeat until one ROS control period elapses.
        publishCommand(last_control_);
        return;
      }
      actual_step_dt = std::clamp(
          static_cast<double>(elapsed_ns) * 1e-9, 1e-3, max_step_dt);
    }
    last_control_update_stamp_ = update_stamp;
  } else {
    if (last_control_update_steady_) {
      const double elapsed_s = std::chrono::duration<double>(
          cycle_start - *last_control_update_steady_).count();
      if (std::isfinite(elapsed_s) && elapsed_s > 0.0) {
        actual_step_dt = std::clamp(elapsed_s, 1e-3, max_step_dt);
      }
    }
    last_control_update_steady_ = cycle_start;
  }
  if (terminal_approach_distance <= goal_position_tolerance_ &&
      goal_yaw_error <= goal_yaw_tolerance_) {
    publishTerminalDecelerationCommand(actual_step_dt);
    return;
  }
  // 5. 检查参考序列长度
  if (references.size() <
      static_cast<std::size_t>(controller_->config().horizon + 1)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "【规划失败】参考序列长度 %zu 不足预测时域所需 %d 点，输出零速度。",
        references.size(),
        static_cast<int>(controller_->config().horizon + 1));
    publishZeroCommandForFailure(nullptr);
    return;
  }
  // 6. 校验规划地图快照健康与 epoch 绑定，并调用 MPC（SQP-RTI 或 iLQR）求解
  std::shared_ptr<const ats_navigation_interfaces::msg::PlanningMapSnapshot>
      active_map_snapshot;
  Eigen::Vector3d snapshot_from_odom = Eigen::Vector3d::Zero();
  bool map_snapshot_fresh = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    map_snapshot_fresh = planningSnapshotUsableLocked(
        cycle_start, &active_map_snapshot, &snapshot_from_odom);
  }
  if (require_planning_snapshot_ && !map_snapshot_fresh) {
    publishZeroCommandForFailure("规划地图快照过期、未就绪或 epoch 不一致");
    return;
  }

  const auto snapshot_start = cycle_start;
  ControlCycleSnapshot snapshot;
  snapshot.cycle_sequence = cycle_sequence;
  snapshot.steady_start = cycle_start;
  const double now_sec_for_delay =
      get_clock()->ros_time_is_active()
          ? now().seconds()
          : std::chrono::duration<double>(cycle_start.time_since_epoch()).count();
  snapshot.current_state = advanceStateThroughCommandDelay(
      current, now_sec_for_delay, rotation_delay);
  // When resuming active tracking after a brief replanning stop while the chassis
  // is still rolling (e.g. descending a narrow ramp crest), seed last_control_ from
  // the measured body velocity so swerve steer angles match the physical rolling
  // motion instead of snapping sideways from (0, 0, 0).
  if (first_active_cycle && last_control_.isZero(1e-6) &&
      measured_velocity.allFinite() &&
      measured_velocity.head<2>().norm() > 0.05) {
    last_control_ = controller_->clampBodyAndWheelLimits(measured_velocity);
  }
  snapshot.references = references;
  snapshot.last_control_before_solve = last_control_;
  snapshot.first_step_dt = actual_step_dt;
  snapshot.reference_fresh = projection.valid && !trajectory_expired &&
                             references.size() >= static_cast<std::size_t>(
                                 controller_->config().horizon + 1);
  snapshot.localization_fresh = localization_fresh;
  snapshot.execution_lease_valid = execution_lease_valid;
  snapshot.gimbal_valid = gimbal_valid;
  snapshot.emergency_stop_active = fail_stop_engaged_.load() ||
      (emergency_stop_watchdog_enabled_ &&
       emergency_stop_watchdog_.stopRequired());
  snapshot.map_fresh = map_snapshot_fresh;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    snapshot.reference_frame = trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
    snapshot.reference_deadline_ns = trajectory_deadline_.nanoseconds();
    if (active_execution_command_) {
      const auto &command = *active_execution_command_;
      snapshot.manager_incarnation = command.manager_incarnation;
      snapshot.command_sequence = command.command_sequence;
      snapshot.goal_id = command.goal_id;
      snapshot.localization_epoch = command.localization_epoch;
      snapshot.map_generation = command.map_generation;
      snapshot.map_publication_sequence = command.map_publication_sequence;
      snapshot.reference_stamp_ns =
          rclcpp::Time(command.reference.header.stamp).nanoseconds();
      if (!command.reference.header.frame_id.empty()) {
        snapshot.reference_frame = command.reference.header.frame_id;
      }
    }
  }
  // 在 iLQR 求解前冻结 canonical digest。随后 shadow 重新计算该摘要，若任何输入在
  // 两者之间被改写，就不能宣称它们使用同一控制周期输入。
  snapshot.identity_digest_valid =
      controlCycleSnapshotIdentityInputsFinite(snapshot);
  snapshot.identity_digest = snapshot.identity_digest_valid ?
      controlCycleSnapshotIdentityDigest(snapshot) : 0;
  ControlCycleTelemetrySample telemetry;
  telemetry.cycle_sequence = snapshot.cycle_sequence;
  telemetry.snapshot_identity_digest = snapshot.identity_digest;
  telemetry.manager_incarnation = snapshot.manager_incarnation;
  telemetry.command_sequence = snapshot.command_sequence;
  telemetry.goal_id = snapshot.goal_id;
  telemetry.localization_epoch = snapshot.localization_epoch;
  telemetry.map_generation = snapshot.map_generation;
  telemetry.map_publication_sequence = snapshot.map_publication_sequence;
  telemetry.reference_stamp_ns = snapshot.reference_stamp_ns;
  telemetry.reference_deadline_ns = snapshot.reference_deadline_ns;
  std::snprintf(telemetry.reference_frame.data(), telemetry.reference_frame.size(),
                "%s", snapshot.reference_frame.c_str());
  telemetry.execution_lease_valid = snapshot.execution_lease_valid;
  telemetry.reference_fresh = snapshot.reference_fresh;
  if (previous_cycle_start) {
    recordTiming(telemetry, ControlCycleTimingStage::kTimerInterarrival,
                 elapsedMilliseconds(*previous_cycle_start, cycle_start));
  }
  recordTiming(telemetry, ControlCycleTimingStage::kStateTrajectorySnapshot,
               elapsedMilliseconds(snapshot_start, std::chrono::steady_clock::now()));
  recordTiming(telemetry, ControlCycleTimingStage::kReferenceExtraction,
               reference_extraction_ms);

  Se2MpcResult result;
  RolloutSafetyEvaluation rollout_safety;
  const auto active_solve_start = std::chrono::steady_clock::now();
  const bool qp_ok = solveSqpRti(
      snapshot, active_map_snapshot.get(), snapshot_from_odom, telemetry,
      result, rollout_safety);
  recordTiming(telemetry, ControlCycleTimingStage::kActiveSolve,
               elapsedMilliseconds(active_solve_start,
                                   std::chrono::steady_clock::now()));
  telemetry.rollout_safety_level = static_cast<int>(rollout_safety.level);
  telemetry.cross_track_error_m = projection.cross_track_error;

  if (!qp_ok || !result.success || result.controls.empty()) {
    if (rollout_safety.level == RolloutSafetyLevel::kUnsafeStop) {
      publishZeroCommandForFailure("预测滚动轨迹触发近端硬碰撞门禁");
    } else {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "【MPC求解失败】OSQP SQP-RTI 未获得可信解（status=%s reason=%s iter=%d "
          "耗时=%.2f ms 横向误差=%.3f m）；已触发确定性零速度 fail-stop。",
          ltvQpSolverStatusName(telemetry.status),
          telemetry.rejection_reason.data(), telemetry.iterations,
          telemetry.osqp_wall_solve_ms, projection.cross_track_error);
      publishZeroCommandForFailure(nullptr);
    }
    finalizeControlTelemetry(telemetry, cycle_start);
    return;
  }
  if (rollout_safety.level == RolloutSafetyLevel::kUnsafeStop) {
    publishZeroCommandForFailure("预测滚动轨迹触发近端硬碰撞门禁");
    finalizeControlTelemetry(telemetry, cycle_start);
    return;
  }
  // 7. 应用第一个控制量（含健康预测期 SLOWDOWN/BRAKE 平滑减速与终点区限速）并发布
  last_control_ = result.controls.front();
  if ((rollout_safety.level == RolloutSafetyLevel::kSlowdown ||
       rollout_safety.level == RolloutSafetyLevel::kBrake) &&
      rollout_safety.speed_scale < 1.0 - 1e-6) {
    last_rollout_horizon_scale_ =
        std::clamp(rollout_safety.speed_scale, 0.25, 1.0);
    Control scaled_target = last_control_;
    if (rollout_safety.level == RolloutSafetyLevel::kSlowdown) {
      // kSlowdown 表示预测无碰撞但靠近墙面（如 0.80 m 窄隧道）；仅限制高速接近，
      // 保留 0.35 m/s 慢速蠕行下限，避免在无碰撞窄通道重启时把起步速度乘衰至零。
      constexpr double kSlowdownCrawlFloorMps = 0.35;
      const double linear_speed = scaled_target.head<2>().norm();
      if (linear_speed > kSlowdownCrawlFloorMps) {
        const double target_speed =
            std::max(kSlowdownCrawlFloorMps,
                     linear_speed * rollout_safety.speed_scale);
        scaled_target.head<2>() *= (target_speed / linear_speed);
      }
    } else {
      // kBrake 表示预测在制动窗内进入负净空，三轴等比减速（含 wz），
      // 防止线速度刹停而角速度继续旋转把矩形角点扫进弯道内墙。
      scaled_target *= rollout_safety.speed_scale;
    }
    last_control_ = controller_->clampStepIncrement(
        scaled_target, snapshot.last_control_before_solve,
        snapshot.first_step_dt);
    if (!result.controls.empty()) {
      result.controls.front() = last_control_;
      controller_->commitWarmControls(result.controls);
    }
  } else {
    last_rollout_horizon_scale_ =
        std::min(1.0, last_rollout_horizon_scale_ + 0.15);
  }
  // 终点区限速（报告 5.1.10 第 1 项）：MPC 的模型里没有底盘约 0.4 s 的响应滞后，离终点
  // 0.15-0.3 m 时仍给 0.9-1.0 m/s，冲过终点再拉回。终点区内按 |v| <= d / tau 限速，相当于
  // 一阶趋近，tau 大于 e 倍滞后时不过冲。只对终点速度为零的 reference 生效。
  if (terminal_approach_time_constant_ > 0.0 &&
      terminal_approach_distance <= terminal_approach_radius_ &&
      reference_terminal_speed <= terminal_stop_speed_threshold_) {
    last_control_ = limitTerminalApproachSpeed(
        last_control_, terminal_approach_distance, terminal_approach_time_constant_);
    if (terminal_approach_distance <= goal_position_tolerance_) {
      // Once translation has reached the endpoint, the physical chassis still
      // needs braking distance for its stationary yaw tail. Waiting until the
      // yaw tolerance is reached can overshoot a footprint-checked terminal
      // orientation (d26). Reuse the configured approach time constant and
      // preserve the existing angular acceleration limit during deceleration.
      const double deceleration_floor = std::max(
          0.0, std::abs(snapshot.last_control_before_solve(2)) -
                   controller_->config().max_awz * snapshot.first_step_dt);
      const double yaw_rate_limit = std::max(
          goal_yaw_error / terminal_approach_time_constant_, deceleration_floor);
      last_control_(2) = std::clamp(last_control_(2), -yaw_rate_limit, yaw_rate_limit);
    }
    if (!result.controls.empty()) {
      result.controls.front() = last_control_;
      controller_->commitWarmControls(result.controls);
    }
  }
  const auto command_publish_start = std::chrono::steady_clock::now();
  publishCommand(last_control_);
  recordTiming(telemetry, ControlCycleTimingStage::kCommandPublish,
               elapsedMilliseconds(command_publish_start,
                                   std::chrono::steady_clock::now()));
  for (int axis = 0; axis < 3; ++axis) {
    telemetry.published_first_control[static_cast<std::size_t>(axis)] =
        last_control_(axis);
  }
  const double max_pub_wheel = controller_->computeMaxModuleSpeed(last_control_);
  telemetry.published_wheel_speed_max_mps = max_pub_wheel;
  telemetry.wheel_speed_violation_mps =
      controller_->config().max_wheel_speed > 1e-6
          ? std::max(0.0, max_pub_wheel - controller_->config().max_wheel_speed)
          : 0.0;
  const auto diagnostics_log_start = std::chrono::steady_clock::now();
  reportSolverDiagnostics(result);
  double logging_publish_ms = elapsedMilliseconds(
      diagnostics_log_start, std::chrono::steady_clock::now());
  // 8. 发布调试路径
  if (publish_debug_paths_) {
    publishPath(result.states, predicted_path_pub_);
    std::vector<State> reference_states;
    reference_states.reserve(references.size());
    for (const auto &reference : references) {
      reference_states.push_back(reference.state);
    }
    publishPath(reference_states, horizon_path_pub_);
  }
  const auto debug_log_start = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(get_logger(),
               "【MPC调试】vx=%.3f vy=%.3f wz=%.3f 横向误差=%.3f "
               "进度比例=%.2f 求解=%.2fms",
               last_control_(0), last_control_(1), last_control_(2),
               projection.cross_track_error,
               trajectory_tracker_.progressScale(projection.cross_track_error),
               result.solve_time_ms);
  logging_publish_ms += elapsedMilliseconds(debug_log_start,
                                             std::chrono::steady_clock::now());
  recordTiming(telemetry, ControlCycleTimingStage::kLoggingPublish,
               logging_publish_ms);
  finalizeControlTelemetry(telemetry, cycle_start);
}

/**
 * @brief 完成本控制周期的性能记录并按明确预算累积根因计数。
 * @details OSQP update/solve 各自以 `qp_time_limit_ms` 复核；LTV 构造、candidate
 *          audit、telemetry、日志与完整 callback 都以运行时 `control_period_ms` 复核。
 *          同一个周期可有多个根因，因此总数只能作为 root-cause events，绝不能替代分布。
 */
void AtsSwerveMpcNode::finalizeControlTelemetry(
    ControlCycleTelemetrySample telemetry,
    std::chrono::steady_clock::time_point cycle_start) {
  const double control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);

  ControlCycleTimingDistribution callback_distribution;
  ControlCycleTimingDistribution qp_backend_phase_distribution;
  ControlCycleTimingDistribution qp_complete_phase_distribution;
  const auto aggregation_start = std::chrono::steady_clock::now();
  if (telemetry.cycle_sequence % kTelemetrySummaryInterval == 0) {
    std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
    callback_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kFullCallback);
    qp_backend_phase_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kQpBackendPhase);
    qp_complete_phase_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kQpCompletePhase);
  }
  recordTiming(telemetry, ControlCycleTimingStage::kPercentileAggregation,
               elapsedMilliseconds(aggregation_start, std::chrono::steady_clock::now()));

  const auto telemetry_log_start = std::chrono::steady_clock::now();
  if (telemetry.cycle_sequence % kTelemetrySummaryInterval == 0) {
    RCLCPP_INFO(
        get_logger(),
        "控制周期遥测 序号=%llu 模式=%s 频率=%.1fHz 周期=%.3fms "
        "qp_status=%s iter=%d prim=%.2e dual=%.2e slack=%.3f "
        "qp_wall(update/solve/backend/complete)=%.3f/%.3f/%.3f/%.3fms "
        "callback_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_backend_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_complete_p50/p95/p99=%.3f/%.3f/%.3fms "
        "same_snapshot=%s feasible=%s reject=%s",
        static_cast<unsigned long long>(telemetry.cycle_sequence), solver_mode_.c_str(),
        control_rate_hz_, control_period_ms, ltvQpSolverStatusName(telemetry.status),
        telemetry.iterations, telemetry.primal_residual, telemetry.dual_residual,
        telemetry.slack_maximum, telemetry.osqp_wall_update_ms,
        telemetry.osqp_wall_solve_ms,
        telemetry.osqp_wall_qp_phase_ms, telemetry.qp_complete_phase_ms,
        callback_distribution.p50_ms, callback_distribution.p95_ms,
        callback_distribution.p99_ms, qp_backend_phase_distribution.p50_ms,
        qp_backend_phase_distribution.p95_ms, qp_backend_phase_distribution.p99_ms,
        qp_complete_phase_distribution.p50_ms, qp_complete_phase_distribution.p95_ms,
        qp_complete_phase_distribution.p99_ms,
        telemetry.same_snapshot_identity ? "true" : "false",
        telemetry.candidate_feasible ? "true" : "false",
        telemetry.rejection_reason.data());
  }
  const std::size_t logging_index = controlCycleTimingStageIndex(
      ControlCycleTimingStage::kLoggingPublish);
  const double telemetry_log_ms = elapsedMilliseconds(
      telemetry_log_start, std::chrono::steady_clock::now());
  telemetry.stage_ms[logging_index] += telemetry_log_ms;
  telemetry.stage_recorded[logging_index] = std::isfinite(telemetry.stage_ms[logging_index]) &&
      telemetry.stage_ms[logging_index] >= 0.0;
  recordTiming(telemetry, ControlCycleTimingStage::kFullCallback,
               elapsedMilliseconds(cycle_start, std::chrono::steady_clock::now()));

  const auto exceeded = [&telemetry](ControlCycleTimingStage stage,
                                     double budget_ms) {
    const std::size_t index = controlCycleTimingStageIndex(stage);
    return telemetry.stage_recorded[index] &&
           telemetry.stage_ms[index] > budget_ms;
  };
  const double candidate_audit_ms =
      telemetry.stage_ms[controlCycleTimingStageIndex(
          ControlCycleTimingStage::kPrimalReconstructionRollout)] +
      telemetry.stage_ms[controlCycleTimingStageIndex(
          ControlCycleTimingStage::kCandidateHardCheck)];
  const bool candidate_audit_overrun =
      (telemetry.stage_recorded[controlCycleTimingStageIndex(
           ControlCycleTimingStage::kPrimalReconstructionRollout)] ||
       telemetry.stage_recorded[controlCycleTimingStageIndex(
           ControlCycleTimingStage::kCandidateHardCheck)]) &&
      candidate_audit_ms > control_period_ms;

  std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
  if (!control_telemetry_.push(telemetry, cycle_start)) {
    return;
  }
  if (telemetry.qp_attempted &&
      telemetry.status == LtvQpSolverStatus::kTimeLimit) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpTimeLimitStatus);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kOsqpSolve, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpSolveBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kFullCallback, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kCallbackPeriodOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpProblemBuild, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpProblemBuildBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kOsqpNumericUpdate, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpUpdateBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpBackendPhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpPhaseBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpCompletePhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpCompletePhaseBudgetOverrun);
  }
  if (telemetry.qp_attempted && candidate_audit_overrun) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpCandidateAuditBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kPercentileAggregation, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kTelemetryAggregationBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kLoggingPublish, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kLoggingPublishBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kTimerInterarrival, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kTimerInterarrivalOverrun);
  }
}

/**
 * @brief 复制当前固定遥测窗口，序列化为供回归脚本保存的只读 JSON。
 * @details service 在 ROS executor 中运行；控制 timer 只在短临界区写固定数组。服务不访问
 *          tracker、求解器、command publisher 或 safety gate，因此不能改变任何控制行为。
 */
void AtsSwerveMpcNode::dumpControlTelemetry(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  ControlCycleTelemetryRing snapshot;
  {
    std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
    snapshot = control_telemetry_;
  }
  ControlCycleTelemetryMetadata metadata;
  metadata.solver_mode = solver_mode_;
  metadata.control_rate_hz = control_rate_hz_;
  metadata.control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);
  metadata.qp_max_iterations = qp_solver_settings_.max_iterations;
  metadata.qp_time_limit_ms = qp_solver_settings_.time_limit_ms;
  metadata.qp_max_primal_residual = qp_solver_settings_.max_primal_residual;
  metadata.qp_max_dual_residual = qp_solver_settings_.max_dual_residual;
  metadata.qp_max_tracking_slack = qp_solver_settings_.max_tracking_slack;
  metadata.qp_max_hard_constraint_violation =
      qp_solver_settings_.max_hard_constraint_violation;
  metadata.ros_domain_id = currentRosDomainId();
  metadata.use_sim_time = get_parameter_or<bool>("use_sim_time", false);
  response->success = true;
  response->message = snapshot.toJson(metadata);
}

/**
 * @brief 所有非紧急停止的失败/退出路径统一出口：发布确定性零速度。
 * @param reason_zh 可选的中文原因短语；为 nullptr 时不额外打印（调用处已打印）。
 * @note 每个控制周期最多调用一次。零速度必须"持续发布"而非只发一次，
 *       因为下游底盘固件按最新 Twist 执行，停止发布等于让上一条速度一直生效。
 */
void AtsSwerveMpcNode::publishZeroCommandForFailure(const char *reason_zh) {
  last_control_.setZero();
  last_control_update_stamp_.reset();
  last_control_update_steady_.reset();
  qp_warm_start_valid_ = false;
  last_rollout_horizon_scale_ = 1.0;
  if (qp_solver_) {
    qp_solver_->resetWarmStart();
  }
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    command_delay_history_.clear();
  }
  publishCommand(last_control_);
  controller_->reset();
  if (reason_zh != nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制兜底】%s，已输出确定性零速度。", reason_zh);
  }
}

Control AtsSwerveMpcNode::limitTerminalApproachSpeed(
    const Control & command, double goal_distance, double time_constant) {
  const double limit = std::max(0.0, goal_distance) / time_constant;
  const double speed = command.head<2>().norm();
  if (speed <= limit || speed <= 0.0) {
    return command;
  }
  Control limited = command;
  limited.head<2>() *= limit / speed;
  return limited;
}

/**
 * @brief 终点容差圆内的减速出口：上一拍指令每轴按 max_a*step_dt 降向零，降到零后才清零。
 * @details 报告 5.1.9 B：容差圆内直接发零时指令从约 0.5 m/s 阶跃到 0，底盘带着加速度
 *          限幅与约 0.4 s 滞后滑出 0.15-0.25 m，MPC 再从零起步拉回，形成终点极限环。
 *          这里每拍按实际经过时间步长 step_dt 衰减，与 MPC 自身的增量约束同一物理口径；
 *          车在衰减途中滑出容差圆时，MPC 从这条衰减后的指令接着求解，而不是从零。
 *          warm start 每拍都清掉：衰减期间不求解，旧 warm controls 没有随时间左移。
 *          轨迹过期、急停与其它失败路径不经过这里，仍直接发零。
 */
void AtsSwerveMpcNode::publishTerminalDecelerationCommand(double step_dt) {
  const Se2MpcConfig &config = controller_->config();
  const double effective_dt =
      (std::isfinite(step_dt) && step_dt > 0.0) ? step_dt : config.dt;
  const Control step(config.max_ax * effective_dt, config.max_ay * effective_dt,
                     config.max_awz * effective_dt);
  Control next = Control::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double magnitude = std::max(0.0, std::abs(last_control_(axis)) - step(axis));
    next(axis) = std::copysign(magnitude, last_control_(axis));
  }
  if (next.isZero(0.0)) {
    publishZeroCommandForFailure(nullptr);
    return;
  }
  last_control_ = next;
  publishCommand(last_control_);
  controller_->reset();
}

/**
 * @brief 校验动力学参数是否处于实车合理区间。
 * @param config 已加载的 MPC 配置。
 * @note 节点构造期调用一次。参数越界在仿真里往往看不出问题，
 *       但实车会直接表现为超速、舵轮堵转或约束整体失效，因此必须显式告警。
 */
void AtsSwerveMpcNode::validateDynamicsParameters(
    const Se2MpcConfig &config) {
  if (config.dt <= 0.0 || config.horizon <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】dt=%.4f s、horizon=%d 非法，MPC 无法求解。",
                 config.dt, config.horizon);
  }
  const double control_period = 1.0 / std::max(1.0, control_rate_hz_);
  if (control_period > 1.2 * config.dt) {
    RCLCPP_WARN(
        get_logger(),
        "【动力学参数不匹配】控制周期 %.4f s（%.1f Hz）慢于预测步长 dt=%.4f s 超过 20%%，"
        "建议提高 control_rate_hz 或调大预测步长 dt。",
        control_period, control_rate_hz_, config.dt);
  }
  if (config.wheel_base_x <= 0.0 || config.wheel_base_y <= 0.0) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】轮心半轴偏置 wheel_base_x=%.3f m、"
                 "wheel_base_y=%.3f m 必须为正值（实车约 0.270 m）；"
                 "当前配置下四舵轮模块级约束退化，仅车体平移限幅生效。",
                 config.wheel_base_x, config.wheel_base_y);
  }
  if (config.max_wheel_speed <= 1e-6) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】单轮最大线速度 max_wheel_speed=%.3f m/s 未配置，"
                 "轮速上限约束完全失效，实车存在超速风险。",
                 config.max_wheel_speed);
  } else {
    // 全向底盘同时平移与自转时，单轮速度上界为 hypot(v, wz*R)，
    // 其中 R=hypot(wheel_base_x, wheel_base_y) 为轮心到底盘中心的距离。
    const double corner_radius =
        std::hypot(std::max(0.0, config.wheel_base_x),
                   std::max(0.0, config.wheel_base_y));
    const double worst_case =
        std::hypot(std::hypot(config.max_vx, config.max_vy),
                   config.max_wz * corner_radius);
    if (worst_case > config.max_wheel_speed * 1.5) {
      RCLCPP_WARN(get_logger(),
                  "【动力学参数不一致】车体速度上限组合出的最坏单轮速度 %.3f m/s "
                  "已达单轮上限 %.3f m/s 的 %.2f 倍，MPC 会长期工作在轮速饱和区，"
                  "建议下调 max_vx/max_vy/max_wz。",
                  worst_case, config.max_wheel_speed,
                  worst_case / config.max_wheel_speed);
    }
  }
  if (config.max_steer_rate <= 1e-6) {
    RCLCPP_WARN(get_logger(),
                "【动力学参数越界】舵轮最大转向角速度 max_steer_rate=%.3f rad/s "
                "未配置，舵角速率约束失效，急转向时可能要求舵轮瞬间掉头。",
                config.max_steer_rate);
  }
  if (config.max_wheel_acceleration <= 1e-6) {
    RCLCPP_WARN(get_logger(),
                "【动力学参数越界】单轮最大线加速度 max_wheel_acceleration=%.3f "
                "m/s^2 未配置，轮端加速度约束失效。",
                config.max_wheel_acceleration);
  }
}

/**
 * @brief 输出求解饱和与实时性诊断日志。
 * @param result 本周期 MPC 求解结果。
 * @note 每个成功周期调用一次，全部使用 THROTTLE 避免刷屏。
 */
void AtsSwerveMpcNode::reportSolverDiagnostics(const Se2MpcResult &result) {
  if (!result.module_limits_active) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "【动力学参数越界】四舵轮模块级约束未生效"
                          "（wheel_base_* 或 max_wheel_speed 缺失），"
                          "当前仅车体级限幅在保护实车，请立即检查参数文件。");
  }
  if (result.wheel_limit_saturated) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【控制饱和】单轮速度触及上限 %.3f m/s，指令已整体等比缩放；"
                         "跟踪精度会下降，检查参考速度是否超出底盘能力。",
                         controller_->config().max_wheel_speed);
  } else if (result.body_limit_saturated) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制饱和】车体速度指令触及 vx/vy/wz 上限"
                         "（%.2f/%.2f/%.2f），已限幅输出。",
                         controller_->config().max_vx,
                         controller_->config().max_vy,
                         controller_->config().max_wz);
  }
  if (result.increment_limited) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制饱和】舵角速率/轮加速度约束触发增量回退，"
                         "本周期实际指令小于 MPC 期望值。");
  }
  const double control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);
  if (result.solve_time_ms > solve_time_warn_ratio_ * control_period_ms) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【实时性告警】MPC 求解耗时 %.2f ms，已占控制周期 %.2f ms 的 "
                         "%.0f%%，接近超时会导致控制周期抖动。",
                         result.solve_time_ms, control_period_ms,
                         100.0 * result.solve_time_ms / control_period_ms);
  }
}

/**
 * @brief 执行节点级 fail-stop。
 * @details 首次进入时清空 reference、controller warm state 和 frame/deadline；每次调用都通过
 *          同一 command publisher 重发零速度，防止下游保持最后一条非零 Twist。
 */
void AtsSwerveMpcNode::engageFailStop(
    bool record_stop_stamp, std::optional<CommandOrder> stop_command) {
  const bool was_engaged = fail_stop_engaged_.exchange(true);
  if (!was_engaged) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_tracker_.clear();
    trajectory_frame_.clear();
    trajectory_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (record_stop_stamp) {
      last_stop_stamp_ = now();
      last_stop_command_ = stop_command;
    }
    controller_->reset();
    qp_warm_start_valid_ = false;
    if (qp_solver_) {
      qp_solver_->resetWarmStart();
    }
  }
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    command_delay_history_.clear();
  }
  last_control_.setZero();
  last_control_update_stamp_.reset();
  last_control_update_steady_.reset();
  publishCommand(last_control_);
}

/** @brief 将内部车体系 Control 映射到唯一 `geometry_msgs::msg::Twist` 发布通道。 */
void AtsSwerveMpcNode::publishCommand(const Control &command) {
  const double stamp_sec =
      get_clock()->ros_time_is_active()
          ? now().seconds()
          : std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    if (!command_delay_history_.empty() &&
        stamp_sec < command_delay_history_.back().stamp_sec) {
      command_delay_history_.clear();
    }
    command_delay_history_.push_back(CommandHistorySample{stamp_sec, command});
    while (command_delay_history_.size() > 64U ||
           (command_delay_history_.size() > 2U &&
            stamp_sec - command_delay_history_.front().stamp_sec > 0.5)) {
      command_delay_history_.pop_front();
    }
  }
  geometry_msgs::msg::Twist message;
  message.linear.x = command(0);
  message.linear.y = command(1);
  message.angular.z = command(2);
  command_pub_->publish(message);
}

/** @brief 发布调试状态序列为 Path；仅供观察，不是 QP/iLQR 的控制输入。 */
void AtsSwerveMpcNode::publishPath(
    const std::vector<State> &states,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) const {
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id =
      trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
  path.poses.reserve(states.size());
  for (const auto &state : states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state(0);
    pose.pose.position.y = state(1);
    pose.pose.orientation.z = std::sin(0.5 * state(2));
    pose.pose.orientation.w = std::cos(0.5 * state(2));
    path.poses.push_back(pose);
  }
  publisher->publish(path);
}

/** @brief 计算最短 yaw 差，供终点判定和定位跳变检测复用。 */
double AtsSwerveMpcNode::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}


} // namespace ats_swerve_mpc
