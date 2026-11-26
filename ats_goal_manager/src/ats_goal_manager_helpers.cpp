#include "ats_goal_manager_node_impl.hpp"

namespace ats_goal_manager {

void AtsGoalManagerNode::finishActive(std::uint8_t result_code, const std::string &message,
                  GoalLifecycleState terminal_state) {
  std::optional<ActiveGoal> finished;
  geometry_msgs::msg::PoseStamped final_pose;
  double distance = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_) {
      return;
    }
    finished = active_goal_;
    if (has_current_pose_) {
      final_pose = current_pose_;
      distance = std::hypot(active_goal_->target.pose.position.x -
                                current_pose_.pose.position.x,
                            active_goal_->target.pose.position.y -
                                current_pose_.pose.position.y);
    }
    switch (terminal_state) {
    case GoalLifecycleState::kSucceeded:
      lifecycle_.succeed();
      break;
    case GoalLifecycleState::kCanceled:
      lifecycle_.cancel();
      break;
    case GoalLifecycleState::kPreempted:
      lifecycle_.preempt();
      break;
    case GoalLifecycleState::kTimedOut:
      lifecycle_.timeout();
      break;
    default:
      lifecycle_.fail();
      break;
    }
    active_goal_.reset();
    candidate_reference_.reset();
    planner_ready_status_.reset();
    active_execution_command_.reset();
    pending_yaw_authority_request_.reset();
    terminal_converged_since_.reset();
    fail_stop_ = true;
    noteStopReason("finish_active");
  }
  publishEmergencyStop(true);
  publishExecutionStop(
    finished->id, finished->localization_epoch, PlannerStatus::FAILURE_NONE,
    0, 0);
  RCLCPP_INFO(
    get_logger(),
    "TRACE action_result code=%u terminal_state=%s goal=%llu message=%s "
    "final_distance_m=%.3f final_pose=(%.3f,%.3f,%.3f)",
    static_cast<unsigned>(result_code), stateName(terminal_state).c_str(),
    static_cast<unsigned long long>(finished->id), message.c_str(), distance,
    final_pose.pose.position.x, final_pose.pose.position.y,
    tf2::getYaw(final_pose.pose.orientation));
  if (finished->action_handle) {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->result_code = result_code;
    result->message = message;
    result->final_pose = final_pose;
    result->final_distance = distance;
    if (result_code == NavigateToPose::Result::RESULT_SUCCEEDED) {
      finished->action_handle->succeed(result);
    } else if (result_code == NavigateToPose::Result::RESULT_CANCELED) {
      finished->action_handle->canceled(result);
    } else {
      finished->action_handle->abort(result);
    }
  }
}

void AtsGoalManagerNode::completeStandaloneAction(
    const std::shared_ptr<GoalHandleNavigateToPose> &handle,
    std::uint8_t result_code, const std::string &message) {
  if (!handle) {
    return;
  }
  auto result = std::make_shared<NavigateToPose::Result>();
  result->result_code = result_code;
  result->message = message;
  result->final_distance = std::numeric_limits<double>::infinity();
  handle->abort(result);
}

bool AtsGoalManagerNode::mapReadyLocked() const {
  if (require_map_status_) {
    const bool lease_ok =
        last_map_status_signal_ &&
        std::chrono::steady_clock::now() - *last_map_status_signal_ <=
            secondsToDuration(map_ready_timeout_sec_);
    // When localization status is not required (Gazebo GICP flicker), do not
    // force expected_epoch=0. Adapter still bookkeeps a real epoch (>=1); the
    // old `? epoch : 0` form made mapReadyLocked() permanently false after
    // health-probe epoch alignment (map_status_epoch=1 vs expected 0) and
    // goals stuck in waiting_for_map until map_wait deadline.
    const bool epoch_ok =
        !require_localization_status_ ||
        map_status_localization_epoch_ == localization_epoch_.value_or(0);
    return lease_ok && map_status_ready_ && epoch_ok;
  }
  return map_ready_signal_ && last_map_ready_signal_ &&
         std::chrono::steady_clock::now() - *last_map_ready_signal_ <=
             secondsToDuration(map_ready_timeout_sec_);
}

// 地图源存活判定：心跳租约未过期，且最后一次心跳自报 ready。
// 与 mapReadyLocked() 的差别是不看 map_status_ready_——瞬时规划失败会
// 就地把那个标志清成 false（见 onPlannerStatus），而那次清零正是要
// 度量的 churn。用它做预算的健康条件会让预算每个心跳周期归零，于是
// 预算永远到不了期。注入 adapter_lease 时租约过期，注入 service_timeout
// 或 unknown 时心跳自报 ready=0，两者都仍让等待保持无界。
bool AtsGoalManagerNode::mapSourceAliveLocked() const {
  if (!require_map_status_) {
    return map_ready_heartbeat_value_ && last_map_ready_signal_ &&
           std::chrono::steady_clock::now() - *last_map_ready_signal_ <=
               secondsToDuration(map_ready_timeout_sec_);
  }
  const bool lease_ok =
      last_map_status_signal_ &&
      std::chrono::steady_clock::now() - *last_map_status_signal_ <=
          secondsToDuration(map_ready_timeout_sec_);
  const bool epoch_ok =
      !require_localization_status_ ||
      map_status_localization_epoch_ == localization_epoch_.value_or(0);
  return lease_ok && map_status_heartbeat_ready_ && epoch_ok;
}

bool AtsGoalManagerNode::planningSnapshotUsableLocked(
    std::uint64_t publication_sequence, std::uint64_t localization_epoch,
    bool allow_newer_publication) const {
  if (!planning_snapshot_ || !last_planning_snapshot_signal_ ||
    !validPlanningSnapshot(*planning_snapshot_) ||
    (allow_newer_publication
     ? planning_snapshot_->publication_sequence < publication_sequence
     : planning_snapshot_->publication_sequence != publication_sequence) ||
    planning_snapshot_->localization_epoch != localization_epoch ||
    std::chrono::steady_clock::now() < *last_planning_snapshot_signal_)
  {
    return false;
  }
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *last_planning_snapshot_signal_).count() <=
    planning_snapshot_timeout_sec_;
}

bool AtsGoalManagerNode::candidateReferenceMapUsableLocked(
    geometry_msgs::msg::TransformStamped * used_transform) const {
  if (!planner_ready_status_ || !candidate_reference_ ||
    planner_ready_status_->gate_collision_count != 0U ||
    planner_ready_status_->map_publication_sequence == 0U ||
    planner_ready_status_->map_publication_sequence > map_status_publication_sequence_)
  {
    return false;
  }
  if (!require_planning_snapshot_) {
    return planner_ready_status_->map_publication_sequence == map_status_publication_sequence_;
  }
  if (!planningSnapshotUsableLocked(
      map_status_publication_sequence_, planner_ready_status_->localization_epoch)) {
    return false;
  }
  // Identity is not a safety certificate, including for an equal publication:
  // experimental MINCO contact-escape candidates may contain collisions. The
  // execution authority always checks the complete reference independently.
  nav_msgs::msg::Path validation_path = *candidate_reference_;
  const std::string & frame = planning_snapshot_->header.frame_id;
  geometry_msgs::msg::TransformStamped validation_transform;
  validation_transform.header.frame_id = frame;
  validation_transform.header.stamp = now();
  validation_transform.child_frame_id = validation_path.header.frame_id;
  validation_transform.transform.rotation.w = 1.0;
  if (validation_path.header.frame_id != frame) {
    if (validation_path.header.frame_id.empty()) {
      return false;
    }
    try {
      // One coherent latest transform for the whole path; never transform
      // future reference timestamps individually or block under mutex_. The
      // enclosing epoch/lease gates reject obsolete localization contracts.
      const auto transform = tf_buffer_->lookupTransform(
        frame, validation_path.header.frame_id, tf2::TimePointZero);
      validation_transform = transform;
      const auto stamp = rclcpp::Time(transform.header.stamp);
      const double age = (now() - stamp).seconds();
      if (stamp.nanoseconds() > 0 &&
          (age > map_transform_timeout_sec_ || age < -tf_future_tolerance_sec_)) {
        return false;
      }
      for (auto & pose : validation_path.poses) {
        if (!pose.header.frame_id.empty() &&
          pose.header.frame_id != validation_path.header.frame_id) {
          return false;
        }
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(pose, transformed, transform);
        pose = std::move(transformed);
      }
      validation_path.header.frame_id = frame;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  const bool safe = planningSnapshotReferenceSafe(
    *planning_snapshot_, validation_path, planning_snapshot_safety_params_);
  if (safe && used_transform) {
    *used_transform = validation_transform;
  }
  return safe;
}

bool AtsGoalManagerNode::recoverySnapshotReadyLocked(const ActiveGoal & goal) const {
  if (goal.wait_for_publication_after == 0U) {
    return true;
  }
  return map_status_publication_sequence_ > goal.wait_for_publication_after &&
    planningSnapshotUsableLocked(
      map_status_publication_sequence_, goal.localization_epoch);
}

bool AtsGoalManagerNode::localizationHealthyLocked() const {
  if (!require_localization_status_) {
    return true;
  }
  if (!last_localization_status_signal_) {
    return false;
  }
  const bool lease_ok =
      std::chrono::steady_clock::now() - *last_localization_status_signal_ <=
      secondsToDuration(localization_status_timeout_sec_);
  return lease_ok &&
         localizationStateAllowsPlanning(localization_status_) &&
         (!require_accepted_localization_ || has_accepted_localization_) &&
         odom_tf_healthy_;
}

bool AtsGoalManagerNode::gimbalStatusSatisfiesLocked(
  std::uint8_t yaw_authority, bool requires_gimbal_lock,
  std::uint64_t minimum_feedback_sequence,
  std::uint64_t required_request_sequence) const {
  if (!require_gimbal_status_) {
    return true;
  }
  if (!gimbal_status_ || !last_gimbal_status_signal_) {
    return false;
  }
  const bool lease_ok = std::chrono::steady_clock::now() - *last_gimbal_status_signal_ <=
    secondsToDuration(gimbal_status_timeout_sec_);
  if (!lease_ok || !gimbal_status_->tf_healthy ||
    gimbal_status_->yaw_authority != yaw_authority ||
    (enforce_gimbal_lock_on_execution_ && requires_gimbal_lock && !gimbal_status_->locked) ||
    (minimum_feedback_sequence > 0 &&
     gimbal_status_->sequence < minimum_feedback_sequence) ||
    (required_request_sequence > 0 &&
     gimbal_status_->request_sequence != required_request_sequence)) {
    return false;
  }
  return !pending_yaw_authority_request_ ||
    gimbal_status_->request_sequence == pending_yaw_authority_request_->request_sequence;
}

void AtsGoalManagerNode::requestYawAuthorityLocked(const PlannerStatus & status) {
  if (!require_gimbal_status_) {
    return;
  }
  if (pending_yaw_authority_request_ &&
    pending_yaw_authority_request_->goal_id == status.goal_id &&
    pending_yaw_authority_request_->localization_epoch == status.localization_epoch &&
    pending_yaw_authority_request_->map_generation == status.map_generation &&
    pending_yaw_authority_request_->map_publication_sequence ==
      status.map_publication_sequence &&
    pending_yaw_authority_request_->yaw_authority == status.yaw_authority &&
    pending_yaw_authority_request_->require_gimbal_lock == status.requires_gimbal_lock) {
    return;
  }
  YawAuthorityRequest request;
  request.header.stamp = now();
  request.header.frame_id = planning_frame_;
  request.request_sequence = ++next_yaw_authority_request_sequence_;
  request.goal_id = status.goal_id;
  request.localization_epoch = status.localization_epoch;
  request.map_generation = status.map_generation;
  request.map_publication_sequence = status.map_publication_sequence;
  request.yaw_authority = status.yaw_authority;
  request.require_gimbal_lock = status.requires_gimbal_lock;
  pending_yaw_authority_request_ = request;
  yaw_authority_request_pub_->publish(request);
}

bool AtsGoalManagerNode::normalizePose(const geometry_msgs::msg::PoseStamped &input,
                   const std::string &target_frame,
                   geometry_msgs::msg::PoseStamped &output,
                   std::string &reason,
                   bool use_latest_transform) const {
  if (!finitePose(input.pose)) {
    reason =
        "goal/odometry pose contains a non-finite value or zero quaternion";
    return false;
  }
  geometry_msgs::msg::PoseStamped source = input;
  if (source.header.frame_id.empty()) {
    source.header.frame_id = target_frame;
  }
  if (source.header.frame_id == target_frame) {
    output = source;
    output.header.frame_id = target_frame;
    return true;
  }
  try {
    const auto transform = use_latest_transform
                               ? tf_buffer_->lookupTransform(
                                     target_frame, source.header.frame_id,
                                     tf2::TimePointZero, tf2::durationFromSec(0.1))
                               : tf_buffer_->lookupTransform(
                                     target_frame, source.header.frame_id,
                                     rclcpp::Time(source.header.stamp),
                                     rclcpp::Duration::from_seconds(0.1));
    if (use_latest_transform && rclcpp::Time(transform.header.stamp).nanoseconds() > 0) {
      const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      if (age > map_transform_timeout_sec_ || age < -0.2) {
        reason = "latest goal transform is stale or future dated";
        return false;
      }
    }
    tf2::doTransform(source, output, transform);
    output.header.frame_id = target_frame;
    return finitePose(output.pose);
  } catch (const tf2::TransformException &exception) {
    reason = "TF transform to " + target_frame + " failed: " + exception.what();
    return false;
  }
}

void AtsGoalManagerNode::publishFeedbackLocked(double elapsed_sec, double distance) {
  if (!active_goal_ || !active_goal_->action_handle) {
    return;
  }
  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  feedback->goal_id = active_goal_->id;
  feedback->state = actionState(lifecycle_.state());
  feedback->distance_remaining = distance;
  feedback->elapsed_sec = elapsed_sec;
  feedback->status = stateName(lifecycle_.state());
  if (has_current_pose_) {
    feedback->current_pose = current_pose_;
  }
  active_goal_->action_handle->publish_feedback(feedback);
}

std::uint8_t AtsGoalManagerNode::actionState(GoalLifecycleState state) {
  switch (state) {
  case GoalLifecycleState::kWaitingForMap:
    return NavigateToPose::Feedback::STATE_WAITING_FOR_MAP;
  case GoalLifecycleState::kPlanning:
    return NavigateToPose::Feedback::STATE_PLANNING;
  case GoalLifecycleState::kTracking:
    return NavigateToPose::Feedback::STATE_TRACKING;
  case GoalLifecycleState::kCanceled:
    return NavigateToPose::Feedback::STATE_CANCELING;
  case GoalLifecycleState::kIdle:
    return NavigateToPose::Feedback::STATE_ACCEPTED;
  default:
    return NavigateToPose::Feedback::STATE_STOPPED;
  }
}

std::string AtsGoalManagerNode::stateName(GoalLifecycleState state) {
  switch (state) {
  case GoalLifecycleState::kWaitingForMap:
    return "waiting_for_map";
  case GoalLifecycleState::kPlanning:
    return "planning";
  case GoalLifecycleState::kTracking:
    return "tracking";
  case GoalLifecycleState::kCanceled:
    return "canceled";
  case GoalLifecycleState::kPreempted:
    return "preempted";
  case GoalLifecycleState::kTimedOut:
    return "timeout";
  case GoalLifecycleState::kSucceeded:
    return "succeeded";
  case GoalLifecycleState::kFailed:
    return "failed";
  default:
    return "idle";
  }
}

void AtsGoalManagerNode::rebasePathTimestamps(nav_msgs::msg::Path &path,
                          const rclcpp::Time &base_time) const {
  if (path.poses.empty()) {
    return;
  }
  const rclcpp::Time old_base(path.poses.front().header.stamp);
  path.header.stamp = base_time;
  for (auto &pose : path.poses) {
    const rclcpp::Duration relative =
        rclcpp::Time(pose.header.stamp) - old_base;
    pose.header.stamp = base_time + relative;
    pose.header.frame_id = planning_frame_;
  }
  path.header.frame_id = planning_frame_;
}

void AtsGoalManagerNode::publishEmergencyStop(bool stop) {
  const rclcpp::Time transition_stamp = now();
  std_msgs::msg::Bool message;
  message.data = stop;
  emergency_stop_pub_->publish(message);
  bool transition = false;
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(emergency_stop_log_mutex_);
    transition = !last_emergency_stop_published_ ||
      *last_emergency_stop_published_ != stop;
    last_emergency_stop_published_ = stop;
    if (transition) {
      reason = stop ? (stop_reason_.empty() ? std::string("unattributed") : stop_reason_) :
        std::string("released");
      if (stop) {
        reference_commit_barrier_.recordStop(
          transition_stamp.nanoseconds(), stop_to_reference_guard_sec_);
      }
    }
    if (!stop || transition) {
      stop_reason_.clear();
    }
  }
  if (transition) {
    RCLCPP_WARN(
      get_logger(), "P2 emergency-stop transition value=%d stamp_ns=%lld reason=%s",
      stop ? 1 : 0, static_cast<long long>(transition_stamp.nanoseconds()), reason.c_str());
  }
}

// 记录"下一次急停跳变"的原因（只用于日志）。同一跳变前有多个来源时保留第一个，
// 后到的追加在后面，便于看出先后。可在持有 mutex_ 时调用（锁顺序 mutex_ -> log mutex）。
void AtsGoalManagerNode::noteStopReason(const std::string &reason) {
  std::lock_guard<std::mutex> lock(emergency_stop_log_mutex_);
  if (stop_reason_.empty()) {
    stop_reason_ = reason;
  } else if (stop_reason_.size() < 256U) {
    stop_reason_ += "+" + reason;
  }
}

void AtsGoalManagerNode::publishExecutionCommand(ExecutionCommand command) {
  command.manager_incarnation = manager_incarnation_;
  command.command_sequence = ++next_execution_command_sequence_;
  const rclcpp::Time publish_stamp = now();
  // /clock is delivered independently to producer and consumer nodes. A
  // consumer can therefore observe a just-published sample against an older
  // clock tick. Backdate only the producer lease; consumers still reject any
  // genuinely future or expired sample with their hard timestamp gates.
  // Sim-time nodes read 0 until the first /clock sample arrives. Time minus
  // Duration does not throw in Humble, it yields a negative nanosecond count,
  // and that serialises as sec=-1. Every consumer builds an rclcpp::Time from
  // this stamp, which throws on a negative sec and kills the arbiter and MPC
  // at startup. Never backdate below the epoch.
  const rclcpp::Duration backdate =
    rclcpp::Duration::from_seconds(execution_command_stamp_backdate_sec_);
  const rclcpp::Time lease_stamp = publish_stamp.nanoseconds() > backdate.nanoseconds() ?
    publish_stamp - backdate :
    rclcpp::Time(0, 0, publish_stamp.get_clock_type());
  command.header.stamp = lease_stamp;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "TRACE execution_command emit mode=%u incarnation=%llu sequence=%llu "
    "stamp_ns=%lld goal=%llu epoch=%llu map_generation=%llu map_sequence=%llu "
    "reference_stamp_ns=%lld poses=%zu",
    static_cast<unsigned>(command.mode),
    static_cast<unsigned long long>(command.manager_incarnation),
    static_cast<unsigned long long>(command.command_sequence),
    static_cast<long long>(lease_stamp.nanoseconds()),
    static_cast<unsigned long long>(command.goal_id),
    static_cast<unsigned long long>(command.localization_epoch),
    static_cast<unsigned long long>(command.map_generation),
    static_cast<unsigned long long>(command.map_publication_sequence),
    static_cast<long long>(rclcpp::Time(command.reference.header.stamp).nanoseconds()),
    command.reference.poses.size());
  execution_command_pub_->publish(command);
}

void AtsGoalManagerNode::publishExecutionStop(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint8_t failure_reason, std::uint64_t map_generation,
    std::uint64_t map_publication_sequence) {
  ExecutionCommand command;
  command.header.stamp = now();
  command.header.frame_id = planning_frame_;
  command.mode = ExecutionCommand::MODE_STOP;
  command.goal_id = goal_id;
  command.localization_epoch = localization_epoch;
  command.map_generation = map_generation;
  command.map_publication_sequence = map_publication_sequence;
  command.failure_reason = failure_reason;
  publishExecutionCommand(std::move(command));
}


} // namespace ats_goal_manager
