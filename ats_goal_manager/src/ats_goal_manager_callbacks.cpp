#include "ats_goal_manager_node_impl.hpp"

namespace ats_goal_manager {

void AtsGoalManagerNode::onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
  geometry_msgs::msg::PoseStamped target;
  std::string reason;
  if (!normalizePose(*message, goal_frame_, target, reason)) {
    RCLCPP_ERROR(get_logger(), "Rejected /goal_pose: %s", reason.c_str());
    publishEmergencyStop(true);
    return;
  }
  startGoal(target, nullptr, default_goal_timeout_sec_);
}

void AtsGoalManagerNode::startGoal(const geometry_msgs::msg::PoseStamped &target,
               const std::shared_ptr<GoalHandleNavigateToPose> &handle,
               double timeout_sec, double position_tolerance) {
  std::shared_ptr<GoalHandleNavigateToPose> preempted;
  std::uint64_t id = 0;
  std::uint64_t localization_epoch = 0;
  std::uint64_t map_publication_sequence = 0;
  bool dispatch_now = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_) {
      preempted = active_goal_->action_handle;
      lifecycle_.preempt();
    }
    candidate_reference_.reset();
    planner_ready_status_.reset();
    active_execution_command_.reset();
    pending_yaw_authority_request_.reset();
    terminal_converged_since_.reset();
    id = ++next_goal_id_;
    lifecycle_.start(id, mapReadyLocked() && localizationHealthyLocked());
    active_goal_ = ActiveGoal{id,
                              target,
                              GoalDeadline(now().nanoseconds(),
                                           secondsToNanoseconds(timeout_sec)),
                              handle,
                              false,
                              0,
                              0,
                              0,
                              std::nullopt,
                              false,
                              0};
    active_goal_->requested_position_tolerance = position_tolerance;
    progress_watchdog_.resetGoal(id);
    // Prefer live fusion epoch; when require_localization_status=false the
    // status callback used to skip bookkeeping so this stayed 0 while adapter
    // snapshots carried epoch>=1 and recoverySnapshotReadyLocked never armed
    // (d28: waiting_for_map despite ready=1 heartbeats). Fall back to the
    // latest map-status epoch for Gazebo.
    localization_epoch = localization_epoch_.value_or(map_status_localization_epoch_);
    map_publication_sequence = map_status_publication_sequence_;
    active_goal_->localization_epoch = localization_epoch;
    if (lifecycle_.state() == GoalLifecycleState::kWaitingForMap) {
      active_goal_->waiting_since = std::chrono::steady_clock::now();
    }
    fail_stop_ = true;
    noteStopReason("start_goal");
    dispatch_now = lifecycle_.state() == GoalLifecycleState::kPlanning;
  }
  // 先停止旧任务；即使旧 MINCO 回调晚到，也会因 goal_id 不匹配而被丢弃。
  publishEmergencyStop(true);
  publishExecutionStop(
    id, localization_epoch, PlannerStatus::FAILURE_NONE, 0,
    map_publication_sequence);
  if (preempted) {
    completeStandaloneAction(preempted,
                             NavigateToPose::Result::RESULT_PREEMPTED,
                             "preempted by a newer ATS goal");
  }
  if (dispatch_now) {
    if (!publishPlannerGoal(id, localization_epoch, target)) {
      suspendActiveGoal("start_goal_dispatch_failed", id);
    }
  }
}

bool AtsGoalManagerNode::publishPlannerGoal(std::uint64_t id, std::uint64_t localization_epoch,
                        const geometry_msgs::msg::PoseStamped &canonical_target,
                        std::uint64_t required_snapshot_sequence) {
  geometry_msgs::msg::PoseStamped target;
  std::string reason;
  if (!normalizePose(canonical_target, planning_frame_, target, reason)) {
    RCLCPP_WARN(get_logger(), "Deferring goal %llu dispatch: %s",
                static_cast<unsigned long long>(id), reason.c_str());
    return false;
  }
  PlannerGoal request;
  request.header.stamp = now();
  request.header.frame_id = planning_frame_;
  request.goal_id = id;
  request.localization_epoch = localization_epoch;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_ || active_goal_->id != id ||
        active_goal_->localization_epoch != localization_epoch ||
        !mapReadyLocked() || !localizationHealthyLocked()) {
      return false;
    }
    if (required_snapshot_sequence != 0U &&
      !planningSnapshotUsableLocked(required_snapshot_sequence, localization_epoch)) {
      return false;
    }
    if (!has_current_pose_ || !has_current_velocity_ || !odom_tf_healthy_) {
      return false;
    }
    request.map_publication_sequence = required_snapshot_sequence != 0U ?
      required_snapshot_sequence : map_status_publication_sequence_;
    request.plan_request_sequence = ++active_goal_->next_plan_request_sequence;
    active_goal_->expected_plan_request_sequence = request.plan_request_sequence;
    active_goal_->requested_planning_target = target;
    request.position_tolerance = active_goal_->requested_position_tolerance;
    if (active_goal_->action_handle) {
      request.goal_uuid = active_goal_->action_handle->get_goal_id();
    }
    request.start_pose = planner_start_in_planning_frame_ ? current_planning_pose_ : current_pose_;
    request.start_pose.header.stamp = current_localization_stamp_;
    request.start_twist.linear.x = current_world_vx_;
    request.start_twist.linear.y = current_world_vy_;
    request.start_twist.angular.z = current_world_wz_;
    request.preserve_active_reference =
      active_goal_->replacement_pending && active_execution_command_.has_value() && !fail_stop_;
  }
  request.goal_pose = target;
  RCLCPP_INFO(get_logger(),
    "S3 goal dispatch goal=%llu epoch=%llu request=%llu map_tf_stamp_ns=%lld target=(%.3f,%.3f) frame=%s",
    static_cast<unsigned long long>(id), static_cast<unsigned long long>(localization_epoch),
    static_cast<unsigned long long>(request.plan_request_sequence),
    static_cast<long long>(rclcpp::Time(target.header.stamp).nanoseconds()),
    target.pose.position.x, target.pose.position.y, planning_frame_.c_str());
  planner_goal_pub_->publish(request);
  return true;
}

// site 只用于日志：急停跳变时打印是哪个调用点挂起了目标（5.1.5 E2）。
void AtsGoalManagerNode::suspendActiveGoal(
    const char *site, std::uint64_t id,
    std::optional<std::chrono::steady_clock::time_point> waiting_since,
    std::uint64_t wait_for_publication_after) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_ || active_goal_->id != id) {
      return;
    }
    const bool already_waiting =
        lifecycle_.state() == GoalLifecycleState::kWaitingForMap;
    noteStopReason(std::string("suspend:") + site);
    if (!already_waiting) {
      RCLCPP_WARN(
        get_logger(),
        "P2 suspend goal=%llu site=%s state=%s replacement_pending=%d "
        "committed_request=%llu expected_request=%llu map_sequence=%llu",
        static_cast<unsigned long long>(id), site, stateName(lifecycle_.state()).c_str(),
        active_goal_->replacement_pending ? 1 : 0,
        static_cast<unsigned long long>(active_goal_->committed_plan_request_sequence),
        static_cast<unsigned long long>(active_goal_->expected_plan_request_sequence),
        static_cast<unsigned long long>(map_status_publication_sequence_));
    }
    const auto previous_waiting_since = active_goal_->waiting_since;
    candidate_reference_.reset();
    planner_ready_status_.reset();
    active_execution_command_.reset();
    pending_yaw_authority_request_.reset();
    terminal_converged_since_.reset();
    fail_stop_ = true;
    active_goal_->replacement_pending = false;
    lifecycle_.start(id, false);
    active_goal_->waiting_since = waiting_since.value_or(
        already_waiting && previous_waiting_since
            ? *previous_waiting_since
            : std::chrono::steady_clock::now());
    active_goal_->wait_for_publication_after = std::max(
      active_goal_->wait_for_publication_after, wait_for_publication_after);
  }
  publishEmergencyStop(true);
  publishExecutionStop(id, 0, PlannerStatus::FAILURE_NONE, 0, 0);
}

void AtsGoalManagerNode::onPlannerStatus(const PlannerStatus::SharedPtr message) {
  if (message->state == PlannerStatus::STATE_FAILED) {
    bool matches = false;
    bool transient_failure = false;
    bool reuse_newer_snapshot = false;
    std::uint64_t reused_publication_sequence = 0;
    std::uint64_t wait_for_publication_after = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      matches =
          active_goal_ && active_goal_->id == message->goal_id &&
          active_goal_->localization_epoch == message->localization_epoch &&
          (active_goal_->expected_plan_request_sequence ==
             message->plan_request_sequence ||
           // 正在执行的参考自己失效（运行期不安全/快照变化）：无论替换是否进行中、
           // 规划器用哪个请求序号报告，都照原路径急停挂起。
           (active_execution_command_ &&
            active_goal_->committed_plan_request_sequence ==
              message->plan_request_sequence));
      // 哪些失败按瞬时故障挂起重规划由 plannerFailureIsTransient 决定（含各项理由）。
      // 起点被围、NO_PATH、起点/目标占据在动态占据地图上都可能一次刷新就解除（domain 215、
      // 229）；FAILURE_OPTIMIZER/REPAIR 同理（报告 5.1.10，domain 221）。上界由 onTick 里
      // 挂起态的 no_executable_plan 预算给出，不再只依赖外部目标超时。
      transient_failure =
          matches && plannerFailureIsTransient(message->failure_reason);
      if (transient_failure) {
        active_goal_->recovering = true;
        const bool snapshot_changed =
          message->failure_reason == PlannerStatus::FAILURE_SNAPSHOT_CHANGED;
        if (snapshot_changed) {
          // The failure identifies the old committed publication.  A later
          // status/snapshot pair is eligible for the next request only when
          // it has already arrived as one coherent, valid map identity.
          wait_for_publication_after = message->map_publication_sequence;
          reuse_newer_snapshot =
            map_status_ready_ &&
            map_status_publication_sequence_ > message->map_publication_sequence &&
            planningSnapshotUsableLocked(
              map_status_publication_sequence_, active_goal_->localization_epoch);
          if (reuse_newer_snapshot) {
            reused_publication_sequence = map_status_publication_sequence_;
          }
        }
        if (!reuse_newer_snapshot) {
          // Cross-topic status/snapshot delivery is unordered.  Without a
          // complete newer pair, discard the local readiness evidence and
          // keep the recovery path fail-closed until the adapter republishes.
          map_ready_signal_ = false;
          map_status_ready_ = false;
        }
        if (message->failure_reason == PlannerStatus::FAILURE_RUNTIME_UNSAFE ||
          message->failure_reason == PlannerStatus::FAILURE_FOOTPRINT)
        {
          wait_for_publication_after = message->map_publication_sequence;
        }
      }
    }
    if (matches) {
      if (transient_failure) {
        if (reuse_newer_snapshot) {
          RCLCPP_INFO(
            get_logger(),
            "Recovering snapshot-changed goal=%llu from already received map publication "
            "%llu after invalidated publication %llu.",
            static_cast<unsigned long long>(message->goal_id),
            static_cast<unsigned long long>(reused_publication_sequence),
            static_cast<unsigned long long>(message->map_publication_sequence));
        }
        // planning grid 与 ready/status 是独立 topic，跨 topic 不具备原子顺序。
        // 尚无完整的更新 snapshot 时等待下一次 map status；已经收到严格更新
        // 的同序号 snapshot 时可立即派发新请求，旧 command 仍先经 suspend 清除。
        suspendActiveGoal(
          "planner_transient_failure", message->goal_id, std::nullopt,
          wait_for_publication_after);
      } else {
        finishActive(NavigateToPose::Result::RESULT_PLANNING_FAILED,
                     plannerFailureMessage(message->failure_reason),
                     GoalLifecycleState::kFailed);
      }
    }
    return;
  }
  if (message->state == PlannerStatus::STATE_ACCEPTED &&
      message->failure_reason == PlannerStatus::FAILURE_REPLAN_ADVISED) {
    ReplanTriggerInfo info;
    info.reason = ReplanTriggerReason::kFarObstacleConflict;
    onReplanAdvised(*message, info);
    return;
  }
  if (message->state == PlannerStatus::STATE_ACCEPTED &&
      message->failure_reason != PlannerStatus::FAILURE_NONE) {
    bool never_committed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      never_committed = active_goal_ && !active_execution_command_ &&
        active_goal_->id == message->goal_id &&
        active_goal_->localization_epoch == message->localization_epoch &&
        active_goal_->expected_plan_request_sequence == message->plan_request_sequence;
    }
    if (never_committed) {
      // MINCO may still guard a candidate rejected by this final authority.
      // Without an EXECUTE there is no old reference to keep driving; use
      // the ordinary failure/recovery path instead of waiting indefinitely.
      auto failed = std::make_shared<PlannerStatus>(*message);
      failed->state = PlannerStatus::STATE_FAILED;
      onPlannerStatus(failed);
      return;
    }
    // 无停车重规划的替换请求失败（规划器以 ACCEPTED 报告，旧参考仍受守护）：
    // 清除替换标记并记录冷却时间，避免高频失败死循环。
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ && active_goal_->replacement_pending &&
        active_goal_->id == message->goal_id &&
        active_goal_->localization_epoch == message->localization_epoch &&
        active_goal_->expected_plan_request_sequence == message->plan_request_sequence) {
      active_goal_->replacement_pending = false;
      const auto failed_at = std::chrono::steady_clock::now();
      if (active_goal_->pending_replan_reason == ReplanTriggerReason::kRollingReplan) {
        active_goal_->last_rolling_replan_fail_time = failed_at;
      } else {
        active_goal_->last_non_urgent_replan_time = failed_at;
      }
      RCLCPP_WARN(
        get_logger(),
        "【无停车重规划失败】目标=%llu 请求=%llu 原因=%u，继续执行当前参考。",
        static_cast<unsigned long long>(message->goal_id),
        static_cast<unsigned long long>(message->plan_request_sequence),
        static_cast<unsigned>(message->failure_reason));
    }
    return;
  }
  if (message->state != PlannerStatus::STATE_REFERENCE_READY) {
    return;
  }
  bool protected_mode_switch = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ && active_goal_->id == message->goal_id &&
        active_goal_->localization_epoch == message->localization_epoch &&
        active_goal_->expected_plan_request_sequence ==
          message->plan_request_sequence) {
      protected_mode_switch = active_execution_command_ &&
        (active_execution_command_->yaw_authority != message->yaw_authority ||
         active_execution_command_->requires_gimbal_lock !=
           message->requires_gimbal_lock);
      if (protected_mode_switch) {
        // An authority transition invalidates tracker/warm-start ownership.
        // The new candidate can be authorized only after STOP and a fresh ack.
        active_execution_command_.reset();
        pending_yaw_authority_request_.reset();
        fail_stop_ = true;
        noteStopReason("planner_status_protected_mode_switch");
        // referenceReady() is deliberately valid only from kPlanning.  Keep
        // the goal active but require this newly planned reference to cross
        // the protected request/ack gate before re-entering tracking.
        lifecycle_.start(active_goal_->id, true);
      }
      planner_ready_status_ = *message;
    }
  }
  if (protected_mode_switch) {
    publishEmergencyStop(true);
    publishExecutionStop(
      message->goal_id, message->localization_epoch,
      PlannerStatus::FAILURE_NONE, message->map_generation,
      message->map_publication_sequence);
  }
  tryCommitReference();
}

double AtsGoalManagerNode::effectiveRetargetThreshold(double distance_to_goal) const {
  if (!std::isfinite(distance_to_goal) || distance_to_goal <= retarget_terminal_distance_m_) {
    return retarget_threshold_;
  }
  if (distance_to_goal >= retarget_cruise_distance_m_) {
    return retarget_cruise_threshold_;
  }
  const double ratio =
    (distance_to_goal - retarget_terminal_distance_m_) /
    std::max(1e-6, retarget_cruise_distance_m_ - retarget_terminal_distance_m_);
  return retarget_threshold_ + ratio * (retarget_cruise_threshold_ - retarget_threshold_);
}

void AtsGoalManagerNode::onReplanAdvised(const PlannerStatus & message) {
  onReplanAdvised(message, ReplanTriggerInfo{});
}

void AtsGoalManagerNode::onReplanAdvised(
    const PlannerStatus & message,
    const ReplanTriggerInfo & trigger) {
  std::optional<geometry_msgs::msg::PoseStamped> target;
  std::uint64_t localization_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!replan_without_stop_enabled_ || !active_goal_ || active_goal_->cancel_requested ||
        active_goal_->id != message.goal_id ||
        active_goal_->localization_epoch != message.localization_epoch ||
        lifecycle_.state() != GoalLifecycleState::kTracking ||
        !active_execution_command_ || fail_stop_ ||
        active_goal_->replacement_pending ||
        active_goal_->committed_plan_request_sequence != message.plan_request_sequence) {
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (trigger.reason == ReplanTriggerReason::kRollingReplan) {
      if (active_goal_->last_rolling_replan_fail_time &&
          steady_now - *active_goal_->last_rolling_replan_fail_time <
          secondsToDuration(0.5)) {
        return;
      }
    } else {
      const bool terminal_retarget =
        trigger.reason == ReplanTriggerReason::kEndpointRetarget &&
        std::isfinite(trigger.distance_to_goal_m) &&
        trigger.distance_to_goal_m <= retarget_terminal_distance_m_;
      const double cooldown_sec = terminal_retarget
        ? std::min(replan_advised_cooldown_sec_, 0.4)
        : replan_advised_cooldown_sec_;
      if (cooldown_sec > 0.0 && active_goal_->last_non_urgent_replan_time &&
          steady_now - *active_goal_->last_non_urgent_replan_time <
          secondsToDuration(cooldown_sec)) {
        return;
      }
      active_goal_->last_non_urgent_replan_time = steady_now;
    }
    active_goal_->replacement_pending = true;
    active_goal_->pending_replan_reason = trigger.reason;
    target = active_goal_->target;
    localization_epoch = active_goal_->localization_epoch;
  }
  switch (trigger.reason) {
    case ReplanTriggerReason::kEndpointRetarget:
      RCLCPP_INFO(
        get_logger(),
        "【无停车重规划-目标重投影】目标=%llu 目标重投影偏移=%.3f m (阈值=%.3f m, 距目标=%.2f m)，"
        "继续执行当前参考并重规划。",
        static_cast<unsigned long long>(message.goal_id),
        trigger.drift_m, trigger.threshold_m, trigger.distance_to_goal_m);
      break;
    case ReplanTriggerReason::kRollingReplan:
      RCLCPP_INFO(
        get_logger(),
        "【无停车重规划-滚动推进】目标=%llu 已推进=%.2f m (距子目标=%.2f m, 距终点=%.2f m)，"
        "继续执行当前参考并滚动重规划。",
        static_cast<unsigned long long>(message.goal_id),
        trigger.advanced_m, trigger.dist_to_subgoal_m, trigger.distance_to_goal_m);
      break;
    case ReplanTriggerReason::kFarObstacleConflict:
    default:
      RCLCPP_WARN(
        get_logger(),
        "【无停车重规划-远端障碍冲突】目标=%llu 参考远端冲突，继续执行当前参考并从当前位姿重规划。",
        static_cast<unsigned long long>(message.goal_id));
      break;
  }
  if (!publishPlannerGoal(message.goal_id, localization_epoch, *target)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ && active_goal_->id == message.goal_id) {
      active_goal_->replacement_pending = false;
    }
  }
}

void AtsGoalManagerNode::onCandidateReference(const nav_msgs::msg::Path::SharedPtr message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidate_reference_ = *message;
  }
  tryCommitReference();
}

void AtsGoalManagerNode::onMapReady(const std_msgs::msg::Bool::SharedPtr message) {
  bool fail_active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_ready_signal_ = message->data;
    // 与 map_ready_signal_ 分开保存,理由同 map_status_heartbeat_ready_:
    // 前者会被瞬时故障就地清掉,不能用来判断地图源是否还活着。
    map_ready_heartbeat_value_ = message->data;
    last_map_ready_signal_ = std::chrono::steady_clock::now();
    fail_active = !require_map_status_ && active_goal_ && !message->data;
  }
  if (fail_active) {
    finishActive(NavigateToPose::Result::RESULT_MAP_UNREADY,
                 "ROGMap adapter reported not-ready",
                 GoalLifecycleState::kFailed);
  }
}

void AtsGoalManagerNode::onMapStatus(const PlanningMapStatus::SharedPtr message) {
  std::optional<std::uint64_t> suspend_goal_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_status_ready_ = message->ready;
    // 与 map_status_ready_ 分开保存：后者会被瞬时故障和定位挂起就地清掉，
    // 这个只记录 adapter 最后一次心跳自报的 ready，用于判断地图源存活。
    map_status_heartbeat_ready_ = message->ready;
    map_status_localization_epoch_ = message->localization_epoch;
    map_status_generation_ = message->rog_generation;
    map_status_publication_sequence_ = message->publication_sequence;
    last_map_status_signal_ = std::chrono::steady_clock::now();
    const bool current_epoch =
        !localization_epoch_ ||
        message->localization_epoch == *localization_epoch_;
    if (active_goal_ && current_epoch && !message->ready) {
      active_goal_->recovering = true;
      suspend_goal_id = active_goal_->id;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "P2 goal map heartbeat ready=%d publication_sequence=%llu source_generation=%llu "
    "localization_epoch=%llu received_ns=%lld",
    message->ready ? 1 : 0,
    static_cast<unsigned long long>(message->publication_sequence),
    static_cast<unsigned long long>(message->rog_generation),
    static_cast<unsigned long long>(message->localization_epoch),
    static_cast<long long>(now().nanoseconds()));
  if (suspend_goal_id) {
    // map status 与 grid/TF 更新跨 topic，不具备原子顺序。先急停并进入有界等待；
    // 恢复后重规划，持续超时才由 tick 返回 MAP_UNREADY。
    suspendActiveGoal("map_status_not_ready", *suspend_goal_id);
    return;
  }
  // The adapter publishes the immutable snapshot and its heartbeat on two
  // DDS topics.  A candidate can arrive between them, so retry the exact
  // sequence gate once the status side of that pair is visible.
  tryCommitReference();
}

void AtsGoalManagerNode::onPlanningSnapshot(const PlanningMapSnapshot::SharedPtr message) {
  std::optional<std::uint64_t> suspend_goal_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (localization_epoch_ && message->localization_epoch != *localization_epoch_) {
      return;
    }
    if (planning_snapshot_ &&
      planning_snapshot_->localization_epoch == message->localization_epoch &&
      message->publication_sequence < planning_snapshot_->publication_sequence)
    {
      return;
    }
    planning_snapshot_ = *message;
    last_planning_snapshot_signal_ = std::chrono::steady_clock::now();
    if (active_goal_ && !message->ready &&
      message->localization_epoch == active_goal_->localization_epoch)
    {
      active_goal_->recovering = true;
      suspend_goal_id = active_goal_->id;
    }
  }
  if (suspend_goal_id) {
    // Snapshot and map status are separate DDS topics.  An unavailable
    // snapshot must stop execution even if its status callback arrives next.
    suspendActiveGoal("planning_snapshot_unavailable", *suspend_goal_id);
    return;
  }
  // A candidate can legitimately arrive just before the same adapter
  // publication's snapshot.  Re-run the commit gate; it still requires the
  // exact goal, localization, request and publication identities below.
  tryCommitReference();
}

bool AtsGoalManagerNode::localizationStateAllowsPlanning(std::uint8_t state)
{
  // CONFIRMED: post-accept hold. DEGRADED: soft quality loss with map-frame
  // still held. Excluding DEGRADED races DualMap ready during GICP reject
  // flaps (recovery230: ready=1 for ~0.3s then 700x not-tracking, 0 commits).
  return state == LocalizationStatus::STATE_TRACKING ||
         state == LocalizationStatus::STATE_CONFIRMED ||
         state == LocalizationStatus::STATE_DEGRADED;
}


void AtsGoalManagerNode::onLocalizationStatus(const LocalizationStatus::SharedPtr message) {
  // When localization is not required (Gazebo), still bookkeep epoch/state so
  // goal.localization_epoch matches adapter snapshots. Do not run the
  // fail-closed TRACKING invalidate path.
  if (!require_localization_status_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_localization_status_signal_ = std::chrono::steady_clock::now();
    localization_status_ = message->state;
    localization_epoch_ = message->epoch;
    has_accepted_localization_ = rclcpp::Time(message->last_observation_stamp).nanoseconds() > 0 &&
      !message->odom_identity_latched;
    return;
  }
  bool stop_active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool previously_healthy =
        localizationStateAllowsPlanning(localization_status_);
    const bool epoch_changed =
        localization_epoch_ && *localization_epoch_ != message->epoch;
    last_localization_status_signal_ = std::chrono::steady_clock::now();
    localization_status_ = message->state;
    localization_epoch_ = message->epoch;
    has_accepted_localization_ = rclcpp::Time(message->last_observation_stamp).nanoseconds() > 0 &&
      !message->odom_identity_latched;
    const bool healthy = localizationStateAllowsPlanning(message->state) &&
      (!require_accepted_localization_ || has_accepted_localization_);
    if (active_goal_ && (!healthy || epoch_changed || !previously_healthy)) {
      candidate_reference_.reset();
      planner_ready_status_.reset();
      active_execution_command_.reset();
      fail_stop_ = true;
      noteStopReason("localization_status_unhealthy_or_epoch_changed");
      active_goal_->localization_epoch = message->epoch;
      active_goal_->recovering = true;
      if (lifecycle_.state() != GoalLifecycleState::kWaitingForMap ||
          !active_goal_->waiting_since) {
        lifecycle_.start(active_goal_->id, false);
        active_goal_->waiting_since = std::chrono::steady_clock::now();
      }
      // 定位状态恢复与 map status 是两个 topic；本地先清 ready，必须等待
      // adapter 发布同 localization epoch 的新快照后才能重新规划。
      map_ready_signal_ = false;
      map_status_ready_ = false;
      stop_active = true;
    }
  }
  if (stop_active) {
      publishEmergencyStop(true);
      publishExecutionStop(
        0, message->epoch, PlannerStatus::FAILURE_NONE, 0, 0);
  }
}

void AtsGoalManagerNode::onGimbalYawStatus(const GimbalYawStatus::SharedPtr message) {
  bool stop_active = false;
  std::optional<std::uint64_t> recover_goal_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gimbal_status_ = *message;
    last_gimbal_status_signal_ = std::chrono::steady_clock::now();
    if (active_execution_command_ &&
        !gimbalStatusSatisfiesLocked(
          active_execution_command_->yaw_authority,
          active_execution_command_->requires_gimbal_lock,
          active_execution_command_->gimbal_feedback_sequence,
          active_execution_command_->gimbal_request_sequence)) {
      active_execution_command_.reset();
      fail_stop_ = true;
      noteStopReason("gimbal_status_unsatisfied");
      stop_active = true;
      if (active_goal_) {
        active_goal_->recovering = true;
        recover_goal_id = active_goal_->id;
      }
    }
  }
  if (stop_active) {
    publishEmergencyStop(true);
  }
  if (recover_goal_id) {
    // A lost yaw lease invalidates the tracker.  Re-enter planning so the
    // eventual release can only use a newly timed candidate and fresh ACK.
    suspendActiveGoal("gimbal_status_recover", *recover_goal_id);
  }
  tryCommitReference();
}

void AtsGoalManagerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
  geometry_msgs::msg::PoseStamped input;
  input.header = message->header;
  input.pose = message->pose.pose;
  geometry_msgs::msg::PoseStamped normalized;
  geometry_msgs::msg::PoseStamped planning_normalized;
  std::string reason;
  if (!normalizePose(input, goal_frame_, normalized, reason) ||
    !normalizePose(input, planning_frame_, planning_normalized, reason, false)) {
    std::optional<std::uint64_t> suspend_goal_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_tf_healthy_ = false;
      if (active_goal_) {
        active_goal_->recovering = true;
        suspend_goal_id = active_goal_->id;
      }
    }
    if (suspend_goal_id) {
      suspendActiveGoal("odom_tf_unhealthy", *suspend_goal_id);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_ = normalized;
  current_planning_pose_ = planning_normalized;
  const double yaw = tf2::getYaw(normalized.pose.orientation);
  const double vx_body = message->twist.twist.linear.x;
  const double vy_body = message->twist.twist.linear.y;
  const double wz = message->twist.twist.angular.z;
  if (!std::isfinite(yaw) || !std::isfinite(vx_body) || !std::isfinite(vy_body) ||
      !std::isfinite(wz)) {
    has_current_velocity_ = false;
    odom_tf_healthy_ = true;
    has_current_pose_ = true;
    has_current_planning_pose_ = true;
    return;
  }
  terminal_motion_.add(
    rclcpp::Time(message->header.stamp).seconds(),
    planning_normalized.pose.position.x, planning_normalized.pose.position.y,
    tf2::getYaw(planning_normalized.pose.orientation));
  const double start_yaw = planner_start_in_planning_frame_ ?
    tf2::getYaw(planning_normalized.pose.orientation) : yaw;
  current_world_vx_ = vx_body * std::cos(start_yaw) - vy_body * std::sin(start_yaw);
  current_world_vy_ = vx_body * std::sin(start_yaw) + vy_body * std::cos(start_yaw);
  current_world_wz_ = wz;
  current_linear_velocity_ = std::hypot(vx_body, vy_body);
  current_angular_velocity_ = std::abs(wz);
  current_localization_stamp_ = message->header.stamp;
  has_current_velocity_ = true;
  has_current_pose_ = true;
  has_current_planning_pose_ = true;
  odom_tf_healthy_ = true;
}

// 调用方持有 mutex_。未启用、无活动 command 或坐标系不一致时退回直线距离。
// 参考轨迹在 planning_frame(odom)下,current_pose_ 在 goal_frame(map)下;只比
// current_pose_ 会恒判不一致而退回直线距离(domain 87 红点目标绕行时直线距离
// 增大,转弯途中被误判停滞急停)。按参考的坐标系挑同系位姿。
PlanProgressWatchdog::Clock::time_point AtsGoalManagerNode::progressMotionTime() {
  using Clock = PlanProgressWatchdog::Clock;
  if (!get_clock()->ros_time_is_active()) {
    return Clock::now();
  }
  // This synthetic time_point is used ONLY for motion progress. Map leases,
  // missing-plan/contact budgets and health checks retain their steady clock.
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(
    std::chrono::nanoseconds(now().nanoseconds())));
}

double AtsGoalManagerNode::progressDistanceLocked(double straight_distance) {
  if (!progress_along_reference_ || !active_execution_command_ || !has_current_pose_) {
    return straight_distance;
  }
  const std::string & reference_frame = active_execution_command_->reference.header.frame_id;
  const geometry_msgs::msg::PoseStamped * pose = nullptr;
  if (has_current_planning_pose_ &&
    current_planning_pose_.header.frame_id == reference_frame)
  {
    pose = &current_planning_pose_;
  } else if (current_pose_.header.frame_id == reference_frame) {
    pose = &current_pose_;
  }
  if (pose == nullptr) {
    return straight_distance;
  }
  const auto & poses = active_execution_command_->reference.poses;
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(poses.size());
  ys.reserve(poses.size());
  for (const auto & pose : poses) {
    xs.push_back(pose.pose.position.x);
    ys.push_back(pose.pose.position.y);
  }
  const double along = remainingReferenceArcLength(
    xs, ys, pose->pose.position.x, pose->pose.position.y,
    progress_reference_hint_);
  return std::isfinite(along) ? along : straight_distance;
}


} // namespace ats_goal_manager
