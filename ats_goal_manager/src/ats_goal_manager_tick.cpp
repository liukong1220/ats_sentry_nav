#include "ats_goal_manager_node_impl.hpp"

namespace ats_goal_manager {

void AtsGoalManagerNode::onTick() {
  std::optional<ActiveGoal> transform_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    transform_goal = active_goal_;
  }
  geometry_msgs::msg::PoseStamped latest_target;
  std::string transform_reason;
  if (transform_goal && !normalizePose(transform_goal->target, planning_frame_, latest_target, transform_reason)) {
    suspendActiveGoal("latest_goal_transform_unavailable", transform_goal->id);
    return;
  }
  std::optional<PlannerStatus> endpoint_retarget;
  ReplanTriggerInfo endpoint_retarget_info;
  std::optional<ActiveGoal> snapshot;
  bool dispatch = false;
  bool cancel = false;
  bool timeout = false;
  bool map_stale = false;
  bool map_wait_timeout = false;
  bool localization_wait_timeout = false;
  bool localization_suspended = false;
  bool reached = false;
  std::optional<ActiveGoal> progress_replan;
  std::uint64_t progress_replan_snapshot_sequence = 0;
  std::uint32_t progress_replan_count = 0;
  std::optional<std::string> progress_failure;
  std::optional<std::uint64_t> progress_suspend_goal_id;
  std::uint64_t progress_suspend_after_publication = 0;
  std::optional<std::uint64_t> gimbal_recovery_goal_id;
  double distance = std::numeric_limits<double>::infinity();
  std::uint64_t dispatch_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_) {
      snapshot = active_goal_;
        if (active_execution_command_ &&
          !gimbalStatusSatisfiesLocked(
            active_execution_command_->yaw_authority,
            active_execution_command_->requires_gimbal_lock,
            active_execution_command_->gimbal_feedback_sequence,
            active_execution_command_->gimbal_request_sequence)) {
        active_execution_command_.reset();
        candidate_reference_.reset();
        planner_ready_status_.reset();
          fail_stop_ = true;
          noteStopReason("tick_gimbal_status_unsatisfied");
        if (active_goal_) {
          active_goal_->recovering = true;
          gimbal_recovery_goal_id = active_goal_->id;
        }
      }
      const std::int64_t clock_now_ns = now().nanoseconds();
      const double elapsed =
          1e-9 * static_cast<double>(active_goal_->deadline.elapsed(clock_now_ns));
      cancel = active_goal_->cancel_requested ||
               (active_goal_->action_handle &&
                active_goal_->action_handle->is_canceling());
      timeout = active_goal_->deadline.expired(clock_now_ns);
      const bool map_ready = mapReadyLocked();
      const bool localization_ready = localizationHealthyLocked();
      const bool recovery_snapshot_ready =
        recoverySnapshotReadyLocked(*active_goal_);
      if (!localization_ready &&
          lifecycle_.state() != GoalLifecycleState::kWaitingForMap) {
        candidate_reference_.reset();
        planner_ready_status_.reset();
        fail_stop_ = true;
        noteStopReason("tick_localization_not_ready");
        lifecycle_.start(active_goal_->id, false);
        active_goal_->waiting_since = std::chrono::steady_clock::now();
        map_ready_signal_ = false;
        map_status_ready_ = false;
        localization_suspended = true;
      }
      if (lifecycle_.mapReady(active_goal_->id) && map_ready &&
          localization_ready && recovery_snapshot_ready) {
        dispatch = lifecycle_.mapBecameReady(active_goal_->id);
        active_goal_->waiting_since.reset();
        dispatch_epoch = active_goal_->localization_epoch;
      }
      map_stale = lifecycle_.state() != GoalLifecycleState::kWaitingForMap &&
                  !map_ready;
      if (map_stale) {
        const auto steady_now = std::chrono::steady_clock::now();
        const double heartbeat_age = last_map_status_signal_ ?
          std::chrono::duration<double>(steady_now - *last_map_status_signal_).count() :
          std::numeric_limits<double>::infinity();
        const double snapshot_age = last_planning_snapshot_signal_ ?
          std::chrono::duration<double>(steady_now - *last_planning_snapshot_signal_).count() :
          std::numeric_limits<double>::infinity();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "P2 goal map lease lost goal=%llu plan_request_sequence=%llu ready=%d "
          "heartbeat_age=%.3f status_sequence=%llu source_generation=%llu "
          "snapshot_age=%.3f snapshot_sequence=%llu snapshot_source_generation=%llu",
          static_cast<unsigned long long>(active_goal_->id),
          static_cast<unsigned long long>(active_goal_->expected_plan_request_sequence),
          map_status_ready_ ? 1 : 0, heartbeat_age,
          static_cast<unsigned long long>(map_status_publication_sequence_),
          static_cast<unsigned long long>(map_status_generation_), snapshot_age,
          static_cast<unsigned long long>(
            planning_snapshot_ ? planning_snapshot_->publication_sequence : 0U),
          static_cast<unsigned long long>(
            planning_snapshot_ ? planning_snapshot_->source_generation : 0U));
      }
      const bool wait_expired =
          lifecycle_.state() == GoalLifecycleState::kWaitingForMap &&
          active_goal_->waiting_since &&
          std::chrono::steady_clock::now() - *active_goal_->waiting_since >=
              secondsToDuration(map_wait_timeout_sec_);
      localization_wait_timeout = wait_expired && !localization_ready;
      map_wait_timeout = wait_expired && localization_ready &&
        (!map_ready || !recovery_snapshot_ready);
      // 上面两条都要求"某处不健康"。基础设施全健康、只是规划器反复
      // 报瞬时故障时,两条都不成立;而 evaluate() 只在 kTracking 调用,
      // 所以这类目标在 goal manager 内部没有上界,只能等外部目标超时。
      // 补上第三条:同一个 no_executable_plan 预算跨挂起/重派 churn 累计。
      if (lifecycle_.state() == GoalLifecycleState::kWaitingForMap) {
        SuspendedPlanGate suspended_gate;
        suspended_gate.goal_active = true;
        suspended_gate.cancel_or_preempt = cancel;
        suspended_gate.map_source_alive = mapSourceAliveLocked();
        suspended_gate.localization_fresh = localization_ready;
        suspended_gate.identity.goal_id = active_goal_->id;
        suspended_gate.identity.localization_epoch =
          active_goal_->localization_epoch;
        if (progress_watchdog_.evaluateSuspended(suspended_gate) ==
          PlanProgressDecision::kNoExecutablePlan)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Progress watchdog suspended no-executable-plan budget exhausted: "
            "goal=%llu planner_failure=%u map_source_alive=%d map_ready=%d "
            "localization=%d recovery_snapshot=%d",
            static_cast<unsigned long long>(active_goal_->id),
            planner_ready_status_ ?
              static_cast<unsigned>(planner_ready_status_->failure_reason) : 0U,
            suspended_gate.map_source_alive ? 1 : 0, map_ready ? 1 : 0,
            localization_ready ? 1 : 0, recovery_snapshot_ready ? 1 : 0);
          progress_failure =
            "progress watchdog exhausted the bounded suspended-replan wait";
        }
      }
      if (lifecycle_.state() == GoalLifecycleState::kTracking &&
          has_current_pose_ && has_current_planning_pose_ && transform_goal &&
          transform_goal->id == active_goal_->id) {
        distance = std::hypot(latest_target.pose.position.x - current_planning_pose_.pose.position.x,
          latest_target.pose.position.y - current_planning_pose_.pose.position.y);
        const double yaw_delta =
            tf2::getYaw(latest_target.pose.orientation) -
            tf2::getYaw(current_planning_pose_.pose.orientation);
        const double yaw_error =
            std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
        const double position_tolerance = active_goal_->requested_position_tolerance > 0.0 ?
          active_goal_->requested_position_tolerance : goal_position_tolerance_;
        const bool on_partial_subgoal =
          active_execution_command_ && active_execution_command_->is_partial_subgoal;
        // 局部推进子目标只作为单次 NavigateToPose 内的滚动跳板；未规划到真实终点前绝不判 SUCCEEDED。
        const bool pose_converged = !on_partial_subgoal &&
          distance <= position_tolerance &&
          yaw_error <= goal_yaw_tolerance_;
        bool velocity_converged = false;
        if (terminal_motion_.enabled()) {
          double window_linear = 0.0;
          double window_angular = 0.0;
          velocity_converged = has_current_velocity_ &&
            terminal_motion_.averageSpeed(window_linear, window_angular) &&
            window_linear <= terminal_linear_velocity_tolerance_ &&
            window_angular <= terminal_angular_velocity_tolerance_;
        } else {
          velocity_converged = has_current_velocity_ &&
            current_linear_velocity_ <= terminal_linear_velocity_tolerance_ &&
            current_angular_velocity_ <= terminal_angular_velocity_tolerance_;
        }
        if (pose_converged && velocity_converged) {
          if (!terminal_converged_since_) {
            terminal_converged_since_ = std::chrono::steady_clock::now();
          }
          reached = std::chrono::steady_clock::now() - *terminal_converged_since_ >=
            secondsToDuration(terminal_dwell_sec_);
        } else {
          terminal_converged_since_.reset();
        }
        // Progress watchdog exists to catch "not closing XY distance".
        // Once inside the position disk, remaining yaw/velocity dwell is a
        // terminal-alignment wait, not a progress stall. Treating yaw-only
        // waits as stalls exhausts bounded replans at distance≈0.15 m when
        // MINCO terminal-yaw relocation is footprint-blocked (Gazebo
        // domains 207/209).
        // Success disk (goal_position_tolerance_) and progress-hold disk
        // (progress_hold_distance_m_) are intentionally separate. Watchdog
        // replan storms stop inside the hold disk so tracking can finish
        // closing the last metres; SUCCEEDED still requires the success disk.
        const bool in_progress_hold = distance <=
          std::max(progress_hold_distance_m_, position_tolerance);
        if (replan_without_stop_enabled_ && active_execution_command_ && !fail_stop_ &&
          !active_goal_->replacement_pending &&
          active_goal_->committed_planning_target &&
          active_execution_command_->reference.header.frame_id == planning_frame_ &&
          !active_execution_command_->reference.poses.empty())
        {
          const auto & requested = active_goal_->committed_planning_target->pose.position;
          const double drift = std::hypot(requested.x - latest_target.pose.position.x,
            requested.y - latest_target.pose.position.y);
          const double retarget_threshold = effectiveRetargetThreshold(distance);
          const auto tick_steady_now = std::chrono::steady_clock::now();
          const double retarget_cooldown = distance <= retarget_terminal_distance_m_
            ? std::min(replan_advised_cooldown_sec_, 0.4)
            : replan_advised_cooldown_sec_;
          const bool retarget_cooldown_ok = retarget_cooldown <= 0.0 ||
            !active_goal_->last_non_urgent_replan_time ||
            (tick_steady_now - *active_goal_->last_non_urgent_replan_time) >=
              secondsToDuration(retarget_cooldown);
          if (drift > retarget_threshold && retarget_cooldown_ok) {
            endpoint_retarget = PlannerStatus();
            endpoint_retarget->goal_id = active_goal_->id;
            endpoint_retarget->localization_epoch = active_goal_->localization_epoch;
            endpoint_retarget->plan_request_sequence = active_goal_->committed_plan_request_sequence;
            endpoint_retarget_info.reason = ReplanTriggerReason::kEndpointRetarget;
            endpoint_retarget_info.drift_m = drift;
            endpoint_retarget_info.threshold_m = retarget_threshold;
            endpoint_retarget_info.distance_to_goal_m = distance;
            terminal_converged_since_.reset();
            reached = false;
            RCLCPP_INFO(get_logger(), "S3 endpoint retarget drift=%.4f threshold=%.4f goal=%llu map_tf_stamp_ns=%lld",
              drift, retarget_threshold, static_cast<unsigned long long>(active_goal_->id),
              static_cast<long long>(rclcpp::Time(latest_target.header.stamp).nanoseconds()));
          } else if (rolling_replan_enabled_ &&
            (active_execution_command_->is_partial_subgoal ||
             (active_execution_command_->has_spliced_tail &&
              distance > std::max(progress_hold_distance_m_, rolling_replan_stop_distance_m_))))
          {
            const rclcpp::Time tick_now = now();
            const double min_period_sec = 1.0 / std::max(0.5, rolling_replan_hz_);
            const bool period_ready = !active_goal_->last_rolling_commit_time ||
              (tick_now - *active_goal_->last_rolling_commit_time).seconds() >= min_period_sec;
            double advanced_m = 0.0;
            if (active_goal_->last_rolling_commit_pos) {
              advanced_m = std::hypot(
                current_planning_pose_.pose.position.x - active_goal_->last_rolling_commit_pos->x,
                current_planning_pose_.pose.position.y - active_goal_->last_rolling_commit_pos->y);
            }
            const auto & subgoal_pose = active_execution_command_->reference.poses.back().pose;
            const auto & subgoal_end = subgoal_pose.position;
            const double dist_to_subgoal = std::hypot(
              subgoal_end.x - current_planning_pose_.pose.position.x,
              subgoal_end.y - current_planning_pose_.pose.position.y);
            const double current_yaw = tf2::getYaw(current_planning_pose_.pose.orientation);
            const double subgoal_yaw = tf2::getYaw(subgoal_pose.orientation);
            const double yaw_to_subgoal = std::abs(
              std::remainder(subgoal_yaw - current_yaw, 2.0 * M_PI));
            const double since_commit_sec = active_goal_->last_rolling_commit_time ?
              (tick_now - *active_goal_->last_rolling_commit_time).seconds() : 10.0;
            const bool short_turn_in_progress =
              dist_to_subgoal <= 0.75 && yaw_to_subgoal > 0.25 && since_commit_sec < 3.80;
            const bool short_step_settled =
              dist_to_subgoal <= 0.15 || since_commit_sec >= 0.45;
            const std::uint64_t latest_pub = planning_snapshot_ ?
              planning_snapshot_->publication_sequence : 0U;
            const double subgoal_lookahead_m = std::max(rolling_replan_advance_m_, 2.5);
            const bool partial_trigger = active_execution_command_->is_partial_subgoal &&
              !short_turn_in_progress &&
              ((dist_to_subgoal <= subgoal_lookahead_m && advanced_m >= 0.35) ||
               (latest_pub > active_goal_->last_rolling_publication_sequence &&
                (advanced_m >= 0.4 * rolling_replan_advance_m_ ||
                 (dist_to_subgoal <= 0.60 && short_step_settled))));
            const double spliced_refresh_dist_m = std::max(4.5, rolling_replan_advance_m_ + 2.0);
            const bool spliced_trigger = !active_execution_command_->is_partial_subgoal &&
              active_execution_command_->has_spliced_tail &&
              (advanced_m >= rolling_replan_advance_m_ ||
               dist_to_subgoal <= spliced_refresh_dist_m);
            if (period_ready && (partial_trigger || spliced_trigger)) {
              endpoint_retarget = PlannerStatus();
              endpoint_retarget->goal_id = active_goal_->id;
              endpoint_retarget->localization_epoch = active_goal_->localization_epoch;
              endpoint_retarget->plan_request_sequence =
                active_goal_->committed_plan_request_sequence;
              endpoint_retarget_info.reason = ReplanTriggerReason::kRollingReplan;
              endpoint_retarget_info.distance_to_goal_m = distance;
              endpoint_retarget_info.advanced_m = advanced_m;
              endpoint_retarget_info.dist_to_subgoal_m = dist_to_subgoal;
              terminal_converged_since_.reset();
              reached = false;
            }
          }
        }
        if (pose_converged || in_progress_hold) {
          // 目标圈内没有"推进"可言，但也不是停滞：刷新停滞计时。看门狗只在圈外被调用，
          // 不刷新的话，车停在圈内、定位抖动让距离短暂越过圈沿时，会拿陈旧计时当场判停滞，
          // 一次抖动耗掉一次重规划预算(e5 d124)。
          progress_watchdog_.holdProgress(active_goal_->id, progressMotionTime());
        }
        if (require_planning_snapshot_ && !pose_converged && !in_progress_hold) {
          PlanningSnapshotSafetyResult snapshot_safety;
          // Status and snapshot are separate DDS samples.  A newer ready
          // heartbeat can be observed just before its matching snapshot;
          // keep checking against the last coherent snapshot while its
          // lease is valid.  Candidate commit still requires an exact
          // publication sequence in tryCommitReference().
          const bool snapshot_usable = planning_snapshot_ &&
            planningSnapshotUsableLocked(
              planning_snapshot_->publication_sequence,
              active_goal_->localization_epoch);
          const geometry_msgs::msg::Pose *snapshot_pose = nullptr;
          if (snapshot_usable && planning_snapshot_ && has_current_pose_ &&
            planning_snapshot_->header.frame_id == current_pose_.header.frame_id)
          {
            snapshot_pose = &current_pose_.pose;
          } else if (snapshot_usable && planning_snapshot_ &&
            has_current_planning_pose_ &&
            planning_snapshot_->header.frame_id == current_planning_pose_.header.frame_id)
          {
            snapshot_pose = &current_planning_pose_.pose;
          }
          if (snapshot_pose) {
            snapshot_safety = checkPlanningSnapshotFootprint(
              *planning_snapshot_, *snapshot_pose, planning_snapshot_safety_params_);
          }
          PlanProgressGate gate;
          gate.goal_active = true;
          gate.cancel_or_preempt = cancel;
          gate.emergency_stop = fail_stop_ || lifecycle_.emergencyStopRequired();
          gate.map_fresh = map_ready && snapshot_usable;
          gate.localization_fresh = localization_ready;
          gate.tf_healthy = odom_tf_healthy_ && snapshot_pose != nullptr;
          gate.robot_inside_map = snapshot_safety.inside_map;
          gate.robot_cell_free = snapshot_safety.robot_cell_free;
          gate.footprint_safe = snapshot_safety.footprint_safe;
          if (!gate.footprint_safe && snapshot_safety.robot_cell_free && snapshot_pose &&
            planningSnapshotContactShallow(
              *planning_snapshot_, *snapshot_pose, planning_snapshot_safety_params_,
              ego_contact_max_depth_m_))
          {
            gate.footprint_safe = true;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Progress watchdog shallow contact: depth<=%.3f m pose=(%.3f, %.3f)",
              ego_contact_max_depth_m_, snapshot_pose->position.x,
              snapshot_pose->position.y);
          }
          if (!gate.footprint_safe && snapshot_pose) {
            // 每次足迹不通过时记下接触深度(足迹每边内缩多少才通过)，用于日志与有界放行。
            gate.footprint_contact_depth_m = planningSnapshotContactDepth(
              *planning_snapshot_, *snapshot_pose, planning_snapshot_safety_params_);
          }
          gate.has_current_reference = active_execution_command_.has_value();
          gate.distance_to_goal_m = progressDistanceLocked(distance);
          gate.identity.goal_id = active_goal_->id;
          gate.identity.localization_epoch = active_goal_->localization_epoch;
          gate.identity.map_generation = active_execution_command_ ?
            active_execution_command_->map_generation : 0;
          gate.identity.map_publication_sequence = active_execution_command_ ?
            active_execution_command_->map_publication_sequence : 0;
          gate.identity.plan_request_sequence = active_goal_->expected_plan_request_sequence;
          if (active_execution_command_ && planning_snapshot_) {
            // Keep the reference identity stable across adapter refreshes;
            // the current snapshot is a safety gate, not an excuse to reset
            // a stalled timer every publication.
            gate.identity.source_generation = planning_snapshot_->source_generation;
          }
          gate.motion_time = progressMotionTime();
          const PlanProgressDecision decision = progress_watchdog_.evaluate(gate);
          if (progress_watchdog_.contactHoldActive()) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Progress watchdog contact hold: depth=%.3f m (hold <= %.3f m) pose=(%.3f, %.3f)",
              gate.footprint_contact_depth_m, contact_hold_max_depth_m_,
              snapshot_pose ? snapshot_pose->position.x :
                std::numeric_limits<double>::quiet_NaN(),
              snapshot_pose ? snapshot_pose->position.y :
                std::numeric_limits<double>::quiet_NaN());
          }
          if (progress_watchdog_.egoEscapeActive()) {
            // 自身位姿仍被占据，但有界等待已用尽。这里放行的是
            // "让执行器把车挪出去"，不是放弃 footprint 语义：
            // 地图/定位/TF 仍必须健康，而且
            // 后续仍受 stall/replan 预算与目标超时约束。artifact 需要能区分
            // 这一段与正常推进。
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Progress watchdog ego-blocked escape: cell_free=%d footprint=%d "
              "decision=%d replans=%u pose=(%.3f, %.3f)",
              gate.robot_cell_free, gate.footprint_safe,
              static_cast<int>(decision), progress_watchdog_.consecutiveReplans(),
              snapshot_pose ? snapshot_pose->position.x :
                std::numeric_limits<double>::quiet_NaN(),
              snapshot_pose ? snapshot_pose->position.y :
                std::numeric_limits<double>::quiet_NaN());
          }
          if (decision == PlanProgressDecision::kReplan && planning_snapshot_) {
            progress_replan = *active_goal_;
            progress_replan_snapshot_sequence = planning_snapshot_->publication_sequence;
            progress_replan_count = progress_watchdog_.consecutiveReplans();
            candidate_reference_.reset();
            planner_ready_status_.reset();
            active_execution_command_.reset();
            pending_yaw_authority_request_.reset();
            terminal_converged_since_.reset();
            fail_stop_ = true;
            noteStopReason("progress_watchdog_replan");
            active_goal_->recovering = true;
            lifecycle_.start(active_goal_->id, true);
            active_goal_->waiting_since.reset();
            snapshot = active_goal_;
          } else if (decision == PlanProgressDecision::kExhausted) {
            // 这条分支原先只把原因写进 action result,不落任何日志,于是
            // "有界重规划预算耗尽"在 launch 日志里不可观测:domain 228 的
            // freeze 用例同时出现 replan=1/2 与 result message
            // "progress watchdog exhausted bounded replans",但日志侧零命中,
            // 外部只能误判为看门狗没有触发。与相邻的 kNoExecutablePlan /
            // kUnsafe 对齐,终止决策必须自证。
            // 日志文本用 progress_failure 自身拼装,而不是另写一份措辞:
            // action result 与 launch 日志因此不可能漂移,外部判据只需匹配
            // 这一个字符串。
            progress_failure = "progress watchdog exhausted bounded replans";
            RCLCPP_ERROR(
              get_logger(),
              "P2 goal terminal decision: %s goal=%llu replans=%u "
              "emergency_stop=%d cell_free=%d footprint=%d distance=%.3f "
              "pose=(%.3f, %.3f)",
              progress_failure->c_str(),
              static_cast<unsigned long long>(gate.identity.goal_id),
              progress_watchdog_.consecutiveReplans(),
              gate.emergency_stop ? 1 : 0, gate.robot_cell_free ? 1 : 0,
              gate.footprint_safe ? 1 : 0, gate.distance_to_goal_m,
              snapshot_pose ? snapshot_pose->position.x :
                std::numeric_limits<double>::quiet_NaN(),
              snapshot_pose ? snapshot_pose->position.y :
                std::numeric_limits<double>::quiet_NaN());
          } else if (decision == PlanProgressDecision::kNoExecutablePlan) {
            // 基础设施健康却始终没有可执行计划:或是从未拿到参考,
            // 或是参考还在但急停一直压着。继续等下去只会等到外部
            // 超时,而超时不携带任何原因;这里干净失败并把最后一次
            // 规划器失败原因写进 artifact,
            // 便于区分"规划器拒绝提交"与"目标本身不可达"。
            RCLCPP_ERROR(
              get_logger(),
              "Progress watchdog no-executable-plan budget exhausted: goal=%lu "
              "planner_failure=%u emergency_stop=%d cell_free=%d footprint=%d "
              "pose=(%.3f, %.3f)",
              static_cast<unsigned long>(gate.identity.goal_id),
              planner_ready_status_ ?
                static_cast<unsigned>(planner_ready_status_->failure_reason) : 0U,
              gate.emergency_stop, gate.robot_cell_free, gate.footprint_safe,
              snapshot_pose ? snapshot_pose->position.x :
                std::numeric_limits<double>::quiet_NaN(),
              snapshot_pose ? snapshot_pose->position.y :
                std::numeric_limits<double>::quiet_NaN());
            progress_failure =
              "progress watchdog exhausted the bounded no-executable-plan wait";
          } else if (decision == PlanProgressDecision::kUnsafe) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Progress watchdog unsafe gate: map_fresh=%d localization=%d tf=%d "
              "inside=%d cell_free=%d footprint=%d reference=%d snapshot_frame=%s "
              "current_frame=%s pose=(%.3f, %.3f) contact_depth=%.3f",
              gate.map_fresh, gate.localization_fresh, gate.tf_healthy,
              gate.robot_inside_map, gate.robot_cell_free, gate.footprint_safe,
              gate.has_current_reference,
              planning_snapshot_ ? planning_snapshot_->header.frame_id.c_str() : "<none>",
              current_pose_.header.frame_id.c_str(),
              snapshot_pose ? snapshot_pose->position.x : std::numeric_limits<double>::quiet_NaN(),
              snapshot_pose ? snapshot_pose->position.y : std::numeric_limits<double>::quiet_NaN(),
              gate.footprint_contact_depth_m);
            // The current command is no longer safe, but one coherent map
            // sample must not make the task terminal. Stop immediately and
            // keep the goal bounded by its normal action/map-wait deadlines.
            // A healthy unsafe snapshot requires a strictly newer
            // publication before replanning, so it cannot spin on the same
            // occupied start state or revive the previous reference.
            active_goal_->recovering = true;
            progress_suspend_goal_id = active_goal_->id;
            if (gate.map_fresh && gate.localization_fresh && gate.tf_healthy &&
              planning_snapshot_)
            {
              progress_suspend_after_publication =
                planning_snapshot_->publication_sequence;
            }
          }
        }
      }
      publishFeedbackLocked(elapsed, distance);
    }
  }

  if (dispatch && snapshot) {
    if (!publishPlannerGoal(snapshot->id, dispatch_epoch, snapshot->target)) {
      suspendActiveGoal("tick_dispatch_failed", snapshot->id, snapshot->waiting_since);
    }
  }
  if (endpoint_retarget && !progress_replan && !progress_suspend_goal_id && !gimbal_recovery_goal_id) {
    onReplanAdvised(*endpoint_retarget, endpoint_retarget_info);
  }
  if (progress_replan) {
    publishEmergencyStop(true);
    publishExecutionStop(
      progress_replan->id, progress_replan->localization_epoch,
      PlannerStatus::FAILURE_NONE, 0, progress_replan_snapshot_sequence);
    RCLCPP_WARN(
      get_logger(),
      "Progress watchdog replan=%u goal=%llu snapshot publication=%llu after no distance progress.",
      progress_replan_count,
      static_cast<unsigned long long>(progress_replan->id),
      static_cast<unsigned long long>(progress_replan_snapshot_sequence));
    if (!publishPlannerGoal(
        progress_replan->id, progress_replan->localization_epoch,
        progress_replan->target, progress_replan_snapshot_sequence)) {
      suspendActiveGoal("progress_replan_dispatch_failed", progress_replan->id);
    }
  }
  if (gimbal_recovery_goal_id && !progress_replan) {
    suspendActiveGoal("tick_gimbal_recovery", *gimbal_recovery_goal_id);
  }
  if (progress_suspend_goal_id && !progress_replan && !gimbal_recovery_goal_id) {
    suspendActiveGoal(
      "progress_watchdog_suspend", *progress_suspend_goal_id, std::nullopt,
      progress_suspend_after_publication);
  }
  if (localization_suspended) {
    publishEmergencyStop(true);
  }
  if (cancel) {
    finishActive(NavigateToPose::Result::RESULT_CANCELED, "goal canceled",
                 GoalLifecycleState::kCanceled);
  } else if (timeout) {
    finishActive(NavigateToPose::Result::RESULT_TIMEOUT, "goal timeout",
                 GoalLifecycleState::kTimedOut);
  } else if (map_stale) {
    finishActive(NavigateToPose::Result::RESULT_MAP_UNREADY,
                 "map ready heartbeat lease expired",
                 GoalLifecycleState::kFailed);
  } else if (map_wait_timeout) {
    finishActive(NavigateToPose::Result::RESULT_MAP_UNREADY,
                 "map did not become ready before deadline",
                 GoalLifecycleState::kFailed);
  } else if (localization_wait_timeout) {
    finishActive(NavigateToPose::Result::RESULT_TF_FAILED,
                 "localization did not recover before deadline",
                 GoalLifecycleState::kFailed);
  } else if (reached) {
    finishActive(NavigateToPose::Result::RESULT_SUCCEEDED, "goal reached",
                 GoalLifecycleState::kSucceeded);
  } else if (progress_failure) {
    finishActive(NavigateToPose::Result::RESULT_PLANNING_FAILED,
                 *progress_failure, GoalLifecycleState::kFailed);
  }

  bool stop = true;
  std::optional<ExecutionCommand> execution_heartbeat;
  std::uint64_t stop_goal_id = 0;
  std::uint64_t stop_localization_epoch = 0;
  std::uint64_t stop_map_generation = 0;
  std::uint64_t stop_map_publication_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop = fail_stop_ || lifecycle_.emergencyStopRequired();
    if (stop) {
      noteStopReason(std::string("tick fail_stop=") + (fail_stop_ ? "1" : "0") +
        " state=" + stateName(lifecycle_.state()));
    }
    if (!stop && active_execution_command_) {
      execution_heartbeat = active_execution_command_;
    } else if (active_goal_) {
      stop_goal_id = active_goal_->id;
      stop_localization_epoch = active_goal_->localization_epoch;
      stop_map_publication_sequence = map_status_publication_sequence_;
    }
  }
  const auto tick_now = std::chrono::steady_clock::now();
  bool publish_heartbeat = !last_heartbeat_publish_time_ ||
    (tick_now - *last_heartbeat_publish_time_) >=
      secondsToDuration(emergency_stop_heartbeat_period_sec_);
  {
    std::lock_guard<std::mutex> lock(emergency_stop_log_mutex_);
    if (!last_emergency_stop_published_ || *last_emergency_stop_published_ != stop) {
      publish_heartbeat = true;
    }
  }
  if (publish_heartbeat) {
    last_heartbeat_publish_time_ = tick_now;
    if (execution_heartbeat) {
      publishExecutionCommand(*execution_heartbeat);
    } else {
      publishExecutionStop(
        stop_goal_id, stop_localization_epoch, PlannerStatus::FAILURE_NONE,
        stop_map_generation, stop_map_publication_sequence);
    }
    publishEmergencyStop(stop);
  }
}


} // namespace ats_goal_manager
