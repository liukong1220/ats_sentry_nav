#include "ats_swerve_mpc_node_helpers.hpp"

namespace ats_swerve_mpc {

void AtsSwerveMpcNode::onPath(const nav_msgs::msg::Path::SharedPtr message) {
  if (execution_command_enabled_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "【参考丢弃】ExecutionCommand 已拥有 MPC 授权，忽略旧版 Path。");
    return;
  }
  installPath(*message);
}

/**
 * @brief 校验 frame/时间并把 Path 转为 tracker 的相对时间 reference。
 * @details 成功时设置 reference deadline；默认同时重置 iLQR warm start 与 last control，
 *          仅 keep_control_state=true（同一 goal/epoch/generation 内换轨）时保留二者。
 *          失败不覆盖原有安全状态，调用方负责进入 fail-stop。
 */
bool AtsSwerveMpcNode::installPath(
    const nav_msgs::msg::Path & message,
    bool keep_control_state,
    std::optional<CommandOrder> command,
    const ats_navigation_interfaces::msg::ExecutionCommand * execution_command) {
  if (require_localization_status_ && !localization_tracking_.load()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "【参考丢弃】定位未处于 TRACKING，忽略参考轨迹。");
    return false;
  }
  if (message.poses.size() < 2) {
    RCLCPP_WARN(get_logger(), "【参考丢弃】参考轨迹点数少于 2，已忽略。");
    return false;
  }
  if (!frame_id_.empty() && !message.header.frame_id.empty() &&
      message.header.frame_id != frame_id_) {
    RCLCPP_ERROR(get_logger(),
                 "【参考丢弃】轨迹坐标系为 '%s'，MPC 期望 '%s'。",
                 message.header.frame_id.c_str(), frame_id_.c_str());
    return false;
  }
  const std::size_t pose_count = message.poses.size();
  bool has_derivatives = false;
  if (execution_command != nullptr) {
    const bool any_derivatives =
      !execution_command->reference_vx.empty() ||
      !execution_command->reference_vy.empty() ||
      !execution_command->reference_ax.empty() ||
      !execution_command->reference_ay.empty() ||
      !execution_command->reference_yaw_rate.empty();
    if (any_derivatives) {
      if (execution_command->reference_vx.size() != pose_count ||
          execution_command->reference_vy.size() != pose_count ||
          execution_command->reference_ax.size() != pose_count ||
          execution_command->reference_ay.size() != pose_count ||
          execution_command->reference_yaw_rate.size() != pose_count) {
        RCLCPP_ERROR(get_logger(),
                     "【参考丢弃】ExecutionCommand 导数数组长度与 poses=%zu 不一致。",
                     pose_count);
        return false;
      }
      has_derivatives = true;
    }
  }
  std::vector<TimedState> parsed;
  parsed.reserve(pose_count);
  const double header_time = rclcpp::Time(message.header.stamp).seconds();
  bool monotonic = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < pose_count; ++i) {
    const auto &pose = message.poses[i];
    double pose_time = rclcpp::Time(pose.header.stamp).seconds();
    if (pose_time <= 0.0) {
      pose_time = header_time + fallback_path_dt_ * static_cast<double>(i);
    }
    monotonic = monotonic && pose_time > previous_time;
    TimedState timed;
    timed.time = pose_time;
    timed.state << pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation);
    if (!timed.state.allFinite()) {
      return false;
    }
    if (has_derivatives) {
      timed.world_velocity << execution_command->reference_vx[i],
          execution_command->reference_vy[i];
      timed.world_acceleration << execution_command->reference_ax[i],
          execution_command->reference_ay[i];
      timed.yaw_rate = execution_command->reference_yaw_rate[i];
      if (!timed.world_velocity.allFinite() ||
          !timed.world_acceleration.allFinite() ||
          !std::isfinite(timed.yaw_rate)) {
        RCLCPP_ERROR(get_logger(),
                     "【参考丢弃】ExecutionCommand 导数包含非有限值（i=%zu）。", i);
        return false;
      }
      timed.has_derivatives = true;
    }
    parsed.push_back(timed);
    previous_time = pose_time;
  }
  if (!monotonic) {
    const double start_time = header_time > 0.0 ? header_time : now().seconds();
    for (std::size_t i = 0; i < parsed.size(); ++i) {
      parsed[i].time = start_time + fallback_path_dt_ * static_cast<double>(i);
      parsed[i].has_derivatives = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool missing_stamp =
        message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0;
    // 序号在停车 STOP 之后的 EXECUTE 由 Goal Manager 在停车之后提交，其 reference 戳可与
    // 停车戳落在同一仿真时钟格（domain 213）。其它来源的停车没有序号可比，相等仍拒绝。
    const bool ordered_after_stop = command && last_stop_command_ &&
      command->incarnation == last_stop_command_->incarnation &&
      command->sequence > last_stop_command_->sequence;
    const rclcpp::Time reference_stamp(message.header.stamp);
    if (last_stop_stamp_.nanoseconds() > 0 &&
        (missing_stamp ||
         reference_stamp < last_stop_stamp_ ||
         (reference_stamp == last_stop_stamp_ && !ordered_after_stop))) {
      RCLCPP_WARN(
          get_logger(),
          "【参考丢弃】轨迹时间早于最近一次急停，拒绝旧轨迹。");
      return false;
    }
    trajectory_tracker_.setTrajectory(std::move(parsed));
    trajectory_frame_ =
        message.header.frame_id.empty() ? frame_id_ : message.header.frame_id;
    const double maximum_tracking_duration =
        trajectory_tracker_.duration() /
        std::max(1e-3, trajectory_tracker_.minimumProgressScale());
    trajectory_deadline_ = now() + rclcpp::Duration::from_seconds(
                                       maximum_tracking_duration +
                                       std::max(0.0, trajectory_timeout_));
  }
  // 任何新轨迹切换均重置 OSQP 对偶热启动，避免旧参考的 active-set 乘子污染新轨迹求解。
  qp_warm_start_valid_ = false;
  last_rollout_horizon_scale_ = 1.0;
  if (qp_solver_) {
    qp_solver_->resetWarmStart();
  }
  if (!keep_control_state) {
    controller_->reset();
    {
      std::lock_guard<std::mutex> history_lock(command_history_mutex_);
      command_delay_history_.clear();
    }
    last_control_.setZero();
    last_control_update_stamp_.reset();
    last_control_update_steady_.reset();
  }
  // 保留时不调用 reset()：iLQR 的 warm_controls_ 留在原处，下一拍 initializeControls 照常
  // 左移一格并与新 reference 的前馈 0.7/0.3 混合；last_control_ 保持上一拍已发布的指令。
  return true;
}

/**
 * @brief 接收唯一的执行授权和 reference 原子快照。
 * @details 严格拒绝 incarnation/sequence 倒退、gimbal 或 localization epoch 不匹配；STOP
 *          与任何非法 EXECUTE 都走既有急停，而不是只停止 QP shadow。
 */
void AtsSwerveMpcNode::onExecutionCommand(
    const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr message) {
  const rclcpp::Time receipt = now();
  // rclcpp::Time throws on a negative sec; a malformed stamp must take the same
  // fail-closed rejection path as an expired one instead of aborting the node.
  const bool stamp_valid = message && message->header.stamp.sec >= 0 &&
    message->header.stamp.nanosec < 1000000000U;
  const rclcpp::Time command_stamp = stamp_valid ?
    rclcpp::Time(message->header.stamp, receipt.get_clock_type()) : receipt;
  if (!message || message->manager_incarnation == 0 ||
      message->command_sequence == 0) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=invalid receipt_ns=%lld",
        static_cast<long long>(receipt.nanoseconds()));
    engageFailStop();
    return;
  }
  // The command lease is measured from the producer timestamp as well as DDS
  // receipt time.  A transient-local sample can be delivered immediately
  // after startup even though it was emitted by an older process; accepting it
  // here would let a stale/future authorization install a reference before the
  // arbiter sees it.  Keep the MPC gate identical to the arbiter gate so either
  // consumer fails closed on the same clock contract.
  const rclcpp::Duration command_age = receipt - command_stamp;
  const std::int64_t min_allowed_age_ns =
      -rclcpp::Duration::from_seconds(execution_command_future_tolerance_sec_)
           .nanoseconds();
  if (!stamp_valid || command_age.nanoseconds() < min_allowed_age_ns ||
      command_age > rclcpp::Duration::from_seconds(execution_command_timeout_)) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=timestamp mode=%u incarnation=%llu "
        "sequence=%llu stamp_ns=%lld receipt_ns=%lld age_ns=%lld limit_ns=%lld",
        static_cast<unsigned>(message->mode),
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<long long>(command_age.nanoseconds()),
        static_cast<long long>(
            rclcpp::Duration::from_seconds(execution_command_timeout_).nanoseconds()));
    engageFailStop();
    return;
  }
  const std::int64_t effective_age_ns =
      std::max<std::int64_t>(0, command_age.nanoseconds());
  bool install_reference = false;
  bool keep_control_state = false;
  bool stop = message->mode ==
    ats_navigation_interfaces::msg::ExecutionCommand::MODE_STOP;
  {
    std::unique_lock<std::mutex> lock(trajectory_mutex_);
    if (message->manager_incarnation < last_execution_command_incarnation_) {
      RCLCPP_WARN(
          get_logger(),
          "【执行授权拒绝】TRACE execution_command rejected=old_incarnation incarnation=%llu sequence=%llu "
          "stamp_ns=%lld receipt_ns=%lld active_incarnation=%llu",
          static_cast<unsigned long long>(message->manager_incarnation),
          static_cast<unsigned long long>(message->command_sequence),
          static_cast<long long>(command_stamp.nanoseconds()),
          static_cast<long long>(receipt.nanoseconds()),
          static_cast<unsigned long long>(last_execution_command_incarnation_));
      active_execution_command_.reset();
      lock.unlock();
      engageFailStop();
      return;
    }
    if (message->manager_incarnation > last_execution_command_incarnation_) {
      // Goal Manager 重启后其进程内 sequence 可以从头计数。新 incarnation 的首条
      // 消息必须是 STOP，避免旧进程迟到的 EXECUTE 替换已受保护的 reference。
      if (!stop) {
        stop = true;
        active_execution_command_.reset();
      } else {
        last_execution_command_incarnation_ = message->manager_incarnation;
        last_execution_command_sequence_ = message->command_sequence;
        active_execution_command_.reset();
      }
    } else if (message->command_sequence <= last_execution_command_sequence_) {
      RCLCPP_WARN(
          get_logger(),
          "【执行授权拒绝】TRACE execution_command rejected=replay incarnation=%llu sequence=%llu "
          "stamp_ns=%lld receipt_ns=%lld last_sequence=%llu",
          static_cast<unsigned long long>(message->manager_incarnation),
          static_cast<unsigned long long>(message->command_sequence),
          static_cast<long long>(command_stamp.nanoseconds()),
          static_cast<long long>(receipt.nanoseconds()),
          static_cast<unsigned long long>(last_execution_command_sequence_));
      active_execution_command_.reset();
      lock.unlock();
      engageFailStop();
      return;
    } else {
      last_execution_command_sequence_ = message->command_sequence;
    }
    last_execution_command_signal_ = std::chrono::steady_clock::now() -
      std::chrono::nanoseconds(effective_age_ns);
    const bool grace_active = supersedeGraceActiveLocked(std::chrono::steady_clock::now());
    if (!stop && grace_active &&
        message->mode == ats_navigation_interfaces::msg::ExecutionCommand::MODE_EXECUTE &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        message->map_generation == supersede_old_generation_) {
      // 换代宽限内 Goal Manager 仍按旧 generation 心跳：不安装、不停车，旧 reference
      // 继续执行到宽限截止或新 generation 的 EXECUTE 到达（报告 5.1.9 D）。
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "【换代宽限】旧 generation %llu 的心跳不安装，等待 generation %llu 的执行授权。",
          static_cast<unsigned long long>(message->map_generation),
          static_cast<unsigned long long>(planner_generation_));
      return;
    }
    if (stop) {
      active_execution_command_.reset();
    } else if (message->mode !=
        ats_navigation_interfaces::msg::ExecutionCommand::MODE_EXECUTE ||
      message->goal_id == 0 || message->reference.poses.size() < 2 ||
      (require_gimbal_status_ && message->gimbal_request_sequence == 0) ||
      (message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_GIMBAL_COMPENSATED &&
       message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_BODY_YAW_FOLLOW &&
       message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_SPIN) ||
      !gimbalExecutionValidLocked(*message) ||
      !mapExecutionValidLocked(*message) ||
      (require_localization_status_ &&
       (!localization_tracking_.load() || !localization_epoch_ ||
        *localization_epoch_ != message->localization_epoch))) {
      stop = true;
      active_execution_command_.reset();
    } else {
      const bool same_goal_epoch_generation = active_execution_command_ &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        active_execution_command_->map_generation == message->map_generation;
      // 换代宽限内到达的新 generation：同一 goal 与 epoch，车还在旧 reference 上按巡航
      // 速度跑，和同 generation 换轨一样沿用 last_control_ 与 warm start。
      const bool successor_generation = grace_active &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        message->map_generation > active_execution_command_->map_generation;
      const bool same_reference = same_goal_epoch_generation &&
        active_execution_command_->map_publication_sequence ==
          message->map_publication_sequence &&
        active_execution_command_->reference.header.stamp.sec ==
          message->reference.header.stamp.sec &&
        active_execution_command_->reference.header.stamp.nanosec ==
          message->reference.header.stamp.nanosec;
      if (!same_reference) {
        install_reference = true;
        // 同一 goal、同一定位 epoch、同一 planner generation 内的换轨保留 last_control_ 与
        // iLQR warm start（报告 9.4 第 3 项）：reference 变了，但车的速度没变，从零起步会让
        // max_a*dt 增量限幅把换轨第一拍压到约 0.1 m/s，等同一次急刹。其余情形照旧清零：
        // 首次安装（无 active command）、STOP/急停/FAILED/MAP_UNREADY 之后（这些路径都已清掉
        // active command 或置 fail_stop_engaged_）、generation/epoch/goal 变化。
        keep_control_state = (same_goal_epoch_generation || successor_generation) &&
          !fail_stop_engaged_.load();
      }
      active_execution_command_ = *message;
      supersede_deadline_.reset();
    }
  }
  if (stop) {
    RCLCPP_WARN(
        get_logger(),
        "【执行授权停止】TRACE execution_command accepted=0 stop=1 mode=%u incarnation=%llu sequence=%llu "
        "stamp_ns=%lld receipt_ns=%lld goal=%llu map_generation=%llu map_sequence=%llu",
        static_cast<unsigned>(message->mode),
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<unsigned long long>(message->goal_id),
        static_cast<unsigned long long>(message->map_generation),
        static_cast<unsigned long long>(message->map_publication_sequence));
    std::optional<CommandOrder> stop_command;
    if (message->mode == ats_navigation_interfaces::msg::ExecutionCommand::MODE_STOP) {
      stop_command = CommandOrder{message->manager_incarnation, message->command_sequence};
    }
    engageFailStop(true, stop_command);
    return;
  }
  if (install_reference &&
      !installPath(message->reference, keep_control_state,
                   CommandOrder{message->manager_incarnation, message->command_sequence},
                   message.get())) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=reference_install incarnation=%llu sequence=%llu "
        "stamp_ns=%lld receipt_ns=%lld goal=%llu poses=%zu",
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<unsigned long long>(message->goal_id), message->reference.poses.size());
    engageFailStop();
    return;
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "【执行授权接受】TRACE execution_command accepted=1 incarnation=%llu sequence=%llu "
      "stamp_ns=%lld receipt_ns=%lld goal=%llu epoch=%llu map_generation=%llu "
      "map_sequence=%llu local_generation_authorized=1 content_lineage_pending=1 poses=%zu",
      static_cast<unsigned long long>(message->manager_incarnation),
      static_cast<unsigned long long>(message->command_sequence),
      static_cast<long long>(command_stamp.nanoseconds()),
      static_cast<long long>(receipt.nanoseconds()),
      static_cast<unsigned long long>(message->goal_id),
      static_cast<unsigned long long>(message->localization_epoch),
      static_cast<unsigned long long>(message->map_generation),
      static_cast<unsigned long long>(message->map_publication_sequence),
      message->reference.poses.size());
  fail_stop_engaged_.store(false);
}

// PlannerStatus proves only MINCO's local immutable map identity. It is not
// adapter source-generation or atomic publication/content lineage evidence.
void AtsSwerveMpcNode::onPlannerStatus(
    const ats_navigation_interfaces::msg::PlannerStatus::SharedPtr message) {
  using Status = ats_navigation_interfaces::msg::PlannerStatus;
  if (!message || message->state == Status::STATE_ACCEPTED ||
      message->localization_epoch == 0 || message->map_generation == 0) {
    return;  // Scheduling telemetry cannot establish or erase map authority.
  }
  if (message->header.frame_id.empty() || message->header.stamp.sec < 0 ||
      message->header.stamp.nanosec >= 1000000000u) {
    return;
  }
  const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
  if (stamp_ns <= 0) {
    return;
  }
  bool stop = false;
  bool superseded_only = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (message->localization_epoch < planner_epoch_ ||
        (message->localization_epoch == planner_epoch_ &&
         message->map_generation < planner_generation_)) {
      return;
    }
    const bool changed_epoch = message->localization_epoch > planner_epoch_;
    const bool changed_identity = changed_epoch ||
      message->map_generation > planner_generation_;
    if (!changed_identity && stamp_ns <= planner_status_stamp_ns_) {
      return;
    }
    if (changed_epoch) {
      retired_planner_generation_ = 0;
    }
    if (changed_identity) {
      planner_generation_usable_ = false;
      stop = true;
    }
    planner_epoch_ = message->localization_epoch;
    planner_generation_ = message->map_generation;
    planner_status_stamp_ns_ = stamp_ns;
    if (message->state == Status::STATE_FAILED ||
        message->failure_reason != Status::FAILURE_NONE) {
      planner_generation_usable_ = false;
      stop = true;
      if (message->failure_reason == Status::FAILURE_MAP_UNREADY) {
        retired_planner_generation_ = std::max(
          retired_planner_generation_, planner_generation_);
      }
    } else if (message->state == Status::STATE_REFERENCE_READY) {
      planner_generation_usable_ = planner_generation_ > retired_planner_generation_;
      if (planner_generation_usable_ && mapReadyFreshLocked()) {
        ready_lease_generation_ = planner_generation_;
      }
      // 同一 epoch 内更新的 generation 就绪：旧命令照常作废并停车，但这不是危险停车，
      // 不推进 last_stop_stamp_。旧 reference 不可能复活：EXECUTE 必须带新 generation
      // （mapExecutionValidLocked），而新 generation 的命令只能在本状态之后由 Goal
      // Manager 提交。domain 112：替换 EXECUTE 的 reference 戳 44.4225 s 早于这里的
      // 停车戳，被 installPath 判为"早于最近一次急停"，无停车重规划因此每次都停住。
      // legacy Path 模式没有 generation 绑定，仍按原规则推进停车戳。
      superseded_only = execution_command_enabled_ && changed_identity && !changed_epoch;
      // 换代宽限（报告 5.1.9 D）：旧命令不立即作废，最多再执行 supersede_grace_sec，
      // 等新 generation 的 EXECUTE。新 generation 已标为可用，旧 generation 的 EXECUTE
      // 仍不能安装；宽限截止时控制周期按 superseded 停车。宽限已开始时不延长。
      if (superseded_only && supersede_grace_sec_ > 0.0 && active_execution_command_ &&
          !fail_stop_engaged_.load() &&
          active_execution_command_->localization_epoch == message->localization_epoch) {
        if (!supersede_deadline_) {
          supersede_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(supersede_grace_sec_));
          supersede_old_generation_ = active_execution_command_->map_generation;
        }
        stop = false;
      }
    } else {
      planner_generation_usable_ = false;
      stop = true;
    }
    if (stop) {
      active_execution_command_.reset();
      supersede_deadline_.reset();
    }
  }
  if (stop) {
    engageFailStop(!superseded_only);
  }
}

bool AtsSwerveMpcNode::mapReadyFreshLocked() const {
  if (!map_ready_ || !last_map_ready_signal_) {
    return false;
  }
  const auto age = std::chrono::steady_clock::now() - *last_map_ready_signal_;
  return age >= std::chrono::steady_clock::duration::zero() &&
    std::chrono::duration<double>(age).count() <= map_ready_timeout_sec_;
}

bool AtsSwerveMpcNode::supersedeGraceActiveLocked(
    std::chrono::steady_clock::time_point now) const {
  return supersede_deadline_ && now <= *supersede_deadline_ &&
    active_execution_command_ && !fail_stop_engaged_.load();
}

bool AtsSwerveMpcNode::activeExecutionValidLocked(
    std::chrono::steady_clock::time_point now) const {
  if (!active_execution_command_) {
    return false;
  }
  const auto & command = *active_execution_command_;
  if (mapExecutionValidLocked(command)) {
    return true;
  }
  // 宽限内旧命令仍可执行：地图心跳新鲜、新 generation 已就绪，旧 generation 没被退役。
  return supersedeGraceActiveLocked(now) && mapReadyFreshLocked() &&
    planner_generation_usable_ && command.localization_epoch == planner_epoch_ &&
    command.map_generation == supersede_old_generation_ &&
    command.map_generation > retired_planner_generation_;
}

bool AtsSwerveMpcNode::mapExecutionValidLocked(
    const ats_navigation_interfaces::msg::ExecutionCommand & command) const {
  return mapReadyFreshLocked() && planner_generation_usable_ &&
    command.localization_epoch != 0 && command.localization_epoch == planner_epoch_ &&
    command.map_generation != 0 && command.map_generation == planner_generation_ &&
    command.map_generation > retired_planner_generation_;
}

void AtsSwerveMpcNode::onMapReady(const std_msgs::msg::Bool::SharedPtr message) {
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool expired =
      map_ready_ && last_map_ready_signal_ && !mapReadyFreshLocked();
    stop = !message->data || expired;
    if (stop) {
      // Retire only the identity covered by the expired/false lease. A newer
      // generation published while unready must remain eligible after a fresh
      // REFERENCE_READY + EXECUTE; the recovery heartbeat must not raise the
      // floor past it.
      const std::uint64_t retire_floor =
        (expired && ready_lease_generation_ != 0)
          ? ready_lease_generation_
          : planner_generation_;
      retired_planner_generation_ = std::max(
        retired_planner_generation_, retire_floor);
      planner_generation_usable_ = false;
      active_execution_command_.reset();
      supersede_deadline_.reset();
    }
    map_ready_ = message->data;
    last_map_ready_signal_ = std::chrono::steady_clock::now();
    if (message->data) {
      ready_lease_generation_ = planner_generation_;
    }
  }
  if (stop) {
    engageFailStop();
  }
}

void AtsSwerveMpcNode::onPlanningSnapshot(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot::SharedPtr
        message) {
  const bool payload_valid =
      message != nullptr && validPlanningSnapshotPayload(*message);
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!payload_valid) {
      planning_snapshot_.reset();
      planning_snapshot_ready_ = false;
      stop = require_planning_snapshot_ && active_execution_command_.has_value();
    } else {
      if (planning_snapshot_ &&
          planning_snapshot_->localization_epoch == message->localization_epoch &&
          message->publication_sequence <
              planning_snapshot_->publication_sequence) {
        return;
      }
      planning_snapshot_ = message;
      planning_snapshot_ready_ = true;
      last_planning_snapshot_signal_ = std::chrono::steady_clock::now();
    }
  }
  if (stop) {
    engageFailStop();
  }
}


} // namespace ats_swerve_mpc
