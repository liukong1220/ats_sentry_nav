#include "ats_goal_manager_node_impl.hpp"

namespace ats_goal_manager {

AtsGoalManagerNode::AtsGoalManagerNode()
    : Node("ats_goal_manager"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
      tf_listener_(
          std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
  loadParameters();
  manager_incarnation_ = managerIncarnation();

  planner_goal_pub_ =
      create_publisher<PlannerGoal>(planner_goal_topic_, rclcpp::QoS(10));
  reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      reference_path_topic_, rclcpp::QoS(10).reliable());
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());
  execution_command_pub_ = create_publisher<ExecutionCommand>(
      execution_command_topic_, rclcpp::QoS(1).reliable().transient_local());
  yaw_authority_request_pub_ = create_publisher<YawAuthorityRequest>(
      yaw_authority_request_topic_, rclcpp::QoS(1).reliable().transient_local());
  input_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&AtsGoalManagerNode::onTopicGoal, this,
                std::placeholders::_1));
  planner_status_sub_ = create_subscription<PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&AtsGoalManagerNode::onPlannerStatus, this,
                std::placeholders::_1));
  candidate_reference_sub_ = create_subscription<nav_msgs::msg::Path>(
      candidate_reference_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&AtsGoalManagerNode::onCandidateReference, this,
                std::placeholders::_1));
  map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onMapReady, this,
                std::placeholders::_1));
  map_status_sub_ = create_subscription<PlanningMapStatus>(
      map_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onMapStatus, this,
                std::placeholders::_1));
  planning_snapshot_sub_ = create_subscription<PlanningMapSnapshot>(
      planning_snapshot_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onPlanningSnapshot, this,
                std::placeholders::_1));
  localization_status_sub_ = create_subscription<LocalizationStatus>(
      localization_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onLocalizationStatus, this,
                std::placeholders::_1));
  gimbal_status_sub_ = create_subscription<GimbalYawStatus>(
      gimbal_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onGimbalYawStatus, this,
                std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AtsGoalManagerNode::onOdometry, this,
                std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, action_name_,
      std::bind(&AtsGoalManagerNode::onActionGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&AtsGoalManagerNode::onActionCancel, this,
                std::placeholders::_1),
      std::bind(&AtsGoalManagerNode::onActionAccepted, this,
                std::placeholders::_1));
  const double tick_period_sec =
      tick_rate_hz_ > 0.0 ? (1.0 / tick_rate_hz_) : emergency_stop_heartbeat_period_sec_;
  tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.005, tick_period_sec)),
      std::bind(&AtsGoalManagerNode::onTick, this));

  // 上电和任何无任务状态都保持急停，禁止 DDS late joiner 获得旧 reference
  // 后自行运动。
  publishEmergencyStop(true);
  publishExecutionStop(0, 0, PlannerStatus::FAILURE_NONE, 0, 0);
  RCLCPP_INFO(get_logger(),
              "ATS goal manager ready: action='%s' topic='%s' planner='%s'",
              action_name_.c_str(), input_goal_topic_.c_str(),
              planner_goal_topic_.c_str());
}

void AtsGoalManagerNode::loadParameters() {
  input_goal_topic_ =
      declare_parameter<std::string>("input_goal_topic", "/goal_pose");
  planner_goal_topic_ = declare_parameter<std::string>(
      "planner_goal_topic", "/ats_goal_manager/planner_goal");
  planner_status_topic_ = declare_parameter<std::string>(
      "planner_status_topic", "/minco/planning_status");
  candidate_reference_topic_ = declare_parameter<std::string>(
      "candidate_reference_topic", "/minco/reference_path_candidate");
  reference_path_topic_ = declare_parameter<std::string>(
      "reference_path_topic", "/minco/reference_path");
  emergency_stop_topic_ = declare_parameter<std::string>(
      "emergency_stop_topic", "/planner/emergency_stop");
  execution_command_topic_ = declare_parameter<std::string>(
      "execution_command_topic", "/planner/execution_command");
  yaw_authority_request_topic_ = declare_parameter<std::string>(
      "yaw_authority_request_topic", "/gimbal/yaw_authority_request");
  gimbal_status_topic_ = declare_parameter<std::string>(
      "gimbal_status_topic", "/gimbal/yaw_status");
  map_ready_topic_ = declare_parameter<std::string>("map_ready_topic",
                                                    "/rog_map_adapter/ready");
  map_status_topic_ = declare_parameter<std::string>(
      "map_status_topic", "/rog_map_adapter/status");
  planning_snapshot_topic_ = declare_parameter<std::string>(
      "planning_snapshot_topic", "/rog_map_adapter/planning_snapshot");
  localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
  action_name_ =
      declare_parameter<std::string>("action_name", "/ats_navigate_to_pose");
  goal_frame_ = declare_parameter<std::string>("goal_frame", "map");
  planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
  map_ready_timeout_sec_ =
      std::max(0.1, declare_parameter<double>("map_ready_timeout_sec", 3.0));
  localization_status_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("localization_status_timeout_sec", 1.0));
  require_localization_status_ =
      declare_parameter<bool>("require_localization_status", false);
  require_accepted_localization_ = declare_parameter<bool>("require_accepted_localization", false);
  planner_start_in_planning_frame_ = declare_parameter<bool>("planner_start_in_planning_frame", false);
  retarget_threshold_ = std::max(0.0, declare_parameter<double>("retarget_threshold", 0.03));
  retarget_cruise_threshold_ = std::max(
      retarget_threshold_,
      declare_parameter<double>("retarget_cruise_threshold", 0.20));
  retarget_terminal_distance_m_ = std::max(
      0.1, declare_parameter<double>("retarget_terminal_distance_m", 1.5));
  retarget_cruise_distance_m_ = std::max(
      retarget_terminal_distance_m_ + 0.1,
      declare_parameter<double>("retarget_cruise_distance_m", 2.0));
  replan_advised_cooldown_sec_ = std::max(
      0.0, declare_parameter<double>("replan_advised_cooldown_sec", 1.5));
  map_transform_timeout_sec_ = std::max(0.1,
    declare_parameter<double>("map_transform_timeout_sec", 1.0));
  require_map_status_ = declare_parameter<bool>("require_map_status", true);
  require_gimbal_status_ = declare_parameter<bool>("require_gimbal_status", true);
  enforce_gimbal_lock_on_execution_ =
      declare_parameter<bool>("enforce_gimbal_lock_on_execution", true);
  gimbal_status_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("gimbal_status_timeout_sec", 0.5));
  emergency_stop_heartbeat_period_sec_ = std::max(
      0.02,
      declare_parameter<double>("emergency_stop_heartbeat_period_sec", 0.1));
  tick_rate_hz_ = std::max(0.0, declare_parameter<double>("tick_rate_hz", 0.0));
  tf_future_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("tf_future_tolerance_sec", 0.2));
  stop_to_reference_guard_sec_ = StopReferenceBarrier::boundedGuardSeconds(
    declare_parameter<double>("stop_to_reference_guard_sec", 0.01));
  execution_command_stamp_backdate_sec_ = std::clamp(
    declare_parameter<double>("execution_command_stamp_backdate_sec", 0.02),
    0.0, 0.25);
  map_wait_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("map_wait_timeout_sec", 5.0));
  default_goal_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("default_goal_timeout_sec", 120.0));
  goal_position_tolerance_ = std::max(
      0.0, declare_parameter<double>("goal_position_tolerance", 0.08));
  max_goal_position_tolerance_ = std::max(
      goal_position_tolerance_, declare_parameter<double>(
          "max_goal_position_tolerance", goal_position_tolerance_));
  // Hold radius for progress-watchdog suppression. Defaults to the success
  // disk; Gazebo widens this so replan storms stop before the success latch.
  progress_hold_distance_m_ = std::max(
      goal_position_tolerance_,
      declare_parameter<double>("progress_hold_distance_m", goal_position_tolerance_));
  goal_yaw_tolerance_ =
      std::max(0.0, declare_parameter<double>("goal_yaw_tolerance", 0.15));
  terminal_linear_velocity_tolerance_ = std::max(
      0.0, declare_parameter<double>("terminal_linear_velocity_tolerance", 0.05));
  terminal_angular_velocity_tolerance_ = std::max(
      0.0, declare_parameter<double>("terminal_angular_velocity_tolerance", 0.10));
  terminal_dwell_sec_ = std::max(
      0.0, declare_parameter<double>("terminal_dwell_sec", 0.30));
  // 终点"已停稳"的速度判据。> 0 时用最近这么久的位姿位移折算平均速度，代替里程计瞬时 twist：
  // 车静止时 Point-LIO 的 twist 噪声中位数就有 0.064 m/s，按 0.05 m/s 的瞬时判据只有约三成采样通过
  // (terminal_motion_window.hpp)。0 保持原判据。
  terminal_motion_.setWindow(
      declare_parameter<double>("terminal_velocity_window_sec", 0.0));
  // 可选：进度看门狗用"沿已提交参考的剩余弧长"代替到目标的直线距离。
  // 默认 false 保持原判据；RMUC 仿真 profile 打开（绕行路线直线距离先增后减）。
  progress_along_reference_ =
    declare_parameter<bool>("progress_along_reference", false);
  // 收到 FAILURE_REPLAN_ADVISED 时不急停：旧参考继续执行（仍受 MINCO 10 Hz 运行期复检
  // 守护），从当前位姿发起完整重规划，新参考就绪后直接替换。默认关闭保持原行为。
  replan_without_stop_enabled_ =
    declare_parameter<bool>("replan_without_stop_enabled", false);
  rolling_replan_enabled_ =
    declare_parameter<bool>("rolling_replan_enabled", true);
  rolling_replan_hz_ = std::clamp(
    declare_parameter<double>("rolling_replan_hz", 10.0), 0.5, 25.0);
  rolling_replan_advance_m_ = std::max(
    0.3, declare_parameter<double>("rolling_replan_advance_m", 2.0));
  rolling_replan_stop_distance_m_ = std::max(
    0.0, declare_parameter<double>("rolling_replan_stop_distance_m", 1.5));
  PlanProgressWatchdogParams progress_params;
  progress_params.progress_min_delta_m = std::max(
    0.0, declare_parameter<double>("progress_min_delta_m", 0.10));
  progress_params.replan_stall_timeout_sec = std::max(
    0.01, declare_parameter<double>("replan_stall_timeout_sec", 4.0));
  progress_params.replan_min_interval_sec = std::max(
    0.0, declare_parameter<double>("replan_min_interval_sec", 2.0));
  progress_params.ego_blocked_escape_enabled =
    declare_parameter<bool>("ego_blocked_escape_enabled", false);
  progress_params.ego_blocked_escape_timeout_sec = std::max(
    0.0, declare_parameter<double>("ego_blocked_escape_timeout_sec", 3.0));
  progress_params.no_executable_plan_timeout_sec = std::max(
    0.0, declare_parameter<double>("no_executable_plan_timeout_sec", 30.0));
  const auto max_consecutive_replans = declare_parameter<int>("max_consecutive_replans", 2);
  progress_params.max_consecutive_replans = max_consecutive_replans > 0
    ? static_cast<std::uint32_t>(max_consecutive_replans)
    : 0U;
  // 足迹瞬态接触的有界放行(plan_progress_watchdog.hpp)。默认关闭，行为不变。
  progress_params.contact_hold_enabled =
    declare_parameter<bool>("contact_hold_enabled", false);
  progress_params.contact_hold_max_depth_m = std::max(
    0.0, declare_parameter<double>("contact_hold_max_depth", 0.12));
  progress_params.contact_hold_max_sec = std::max(
    0.0, declare_parameter<double>("contact_hold_max_sec", 1.5));
  contact_hold_max_depth_m_ = progress_params.contact_hold_max_depth_m;
  progress_watchdog_.setParams(progress_params);
  planning_snapshot_timeout_sec_ = std::max(
    0.1, declare_parameter<double>("planning_snapshot_timeout_sec", map_ready_timeout_sec_));
  require_planning_snapshot_ =
    declare_parameter<bool>("require_planning_snapshot", false);
  planning_snapshot_safety_params_.footprint_length = std::max(
    0.0, declare_parameter<double>("footprint_length", 0.58));
  planning_snapshot_safety_params_.footprint_width = std::max(
    0.0, declare_parameter<double>("footprint_width", 0.58));
  planning_snapshot_safety_params_.footprint_safety_margin = std::max(
    0.0, declare_parameter<double>("footprint_safety_margin", 0.05));
  // 与 minco_planner escape_from_contact_max_contact_depth 同一口径。0 保持
  // fail-closed；> 0 时车体压进占据格不超过该深度的擦边接触不再急停，交给
  // 规划器的有界逃逸与 stall/replan 预算处理。
  ego_contact_max_depth_m_ = std::max(
    0.0, declare_parameter<double>("ego_contact_max_depth", 0.0));
}

rclcpp_action::GoalResponse
AtsGoalManagerNode::onActionGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const NavigateToPose::Goal> goal) {
  if (!goal || !finitePose(goal->goal_pose.pose) ||
      !std::isfinite(goal->position_tolerance) || goal->position_tolerance < 0.0 ||
      goal->position_tolerance > max_goal_position_tolerance_) {
    RCLCPP_WARN(
        get_logger(),
        "Rejected action goal with invalid pose or out-of-range position tolerance.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
AtsGoalManagerNode::onActionCancel(const std::shared_ptr<GoalHandleNavigateToPose> handle) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ && active_goal_->action_handle == handle) {
      active_goal_->cancel_requested = true;
      fail_stop_ = true;
      noteStopReason("action_cancel");
    }
  }
  // cancel 回调立即续发急停；最终 result 由 tick 统一收敛，避免与 planner
  // 回调竞态提交 reference。
  publishEmergencyStop(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
AtsGoalManagerNode::onActionAccepted(const std::shared_ptr<GoalHandleNavigateToPose> handle) {
  geometry_msgs::msg::PoseStamped target;
  std::string reason;
  if (!normalizePose(handle->get_goal()->goal_pose, goal_frame_, target, reason)) {
    completeStandaloneAction(handle, NavigateToPose::Result::RESULT_TF_FAILED,
                             reason);
    publishEmergencyStop(true);
    return;
  }
  const auto &timeout = handle->get_goal()->timeout;
  const double requested_timeout =
      static_cast<double>(timeout.sec) +
      1e-9 * static_cast<double>(timeout.nanosec);
  startGoal(target, handle,
            requested_timeout > 0.0 ? requested_timeout
                                    : default_goal_timeout_sec_,
            handle->get_goal()->position_tolerance);
}


void AtsGoalManagerNode::tryCommitReference() {
  std::optional<std::uint64_t> stale_candidate_goal;
  std::uint64_t candidate_publication_sequence = 0;
  std::uint64_t current_publication_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A candidate may see its immutable snapshot before the corresponding
    // status callback.  Wait for that status rather than treating the
    // half-pair as a failure.  Once a newer status is installed, an older
    // candidate needs a complete footprint recheck on the latest immutable
    // snapshot. Wait for the coherent latest status/snapshot pair first.
    if (active_goal_ && candidate_reference_ && planner_ready_status_ &&
        sameStamp(candidate_reference_->header.stamp,
                  planner_ready_status_->reference_stamp) &&
        planner_ready_status_->goal_id == active_goal_->id &&
        planner_ready_status_->localization_epoch == active_goal_->localization_epoch &&
        planner_ready_status_->plan_request_sequence ==
          active_goal_->expected_plan_request_sequence &&
        (planner_ready_status_->map_publication_sequence == 0U ||
         (map_status_publication_sequence_ != 0U &&
           planner_ready_status_->map_publication_sequence <=
            map_status_publication_sequence_ &&
          (!require_planning_snapshot_ || planningSnapshotUsableLocked(
            map_status_publication_sequence_, active_goal_->localization_epoch)) &&
          !candidateReferenceMapUsableLocked()))) {
      stale_candidate_goal = active_goal_->id;
      candidate_publication_sequence =
        planner_ready_status_->map_publication_sequence;
      current_publication_sequence = map_status_publication_sequence_;
    }
  }
  if (stale_candidate_goal) {
    std::optional<geometry_msgs::msg::PoseStamped> replacement_target;
    std::uint64_t replacement_epoch = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_ && active_goal_->id == *stale_candidate_goal &&
          active_goal_->replacement_pending && active_execution_command_ &&
          lifecycle_.state() == GoalLifecycleState::kTracking) {
        candidate_reference_.reset();
        planner_ready_status_.reset();
        replacement_target = active_goal_->target;
        replacement_epoch = active_goal_->localization_epoch;
      }
    }
    if (replacement_target) {
      // 替换候选过期：旧参考仍在执行且受运行期复检守护，直接按最新地图再派一次。
      RCLCPP_WARN(
        get_logger(),
        "【无停车重规划】目标=%llu 的替换候选基于过期地图发布 %llu（当前 %llu），重新派发。",
        static_cast<unsigned long long>(*stale_candidate_goal),
        static_cast<unsigned long long>(candidate_publication_sequence),
        static_cast<unsigned long long>(current_publication_sequence));
      if (!publishPlannerGoal(*stale_candidate_goal, replacement_epoch, *replacement_target)) {
        suspendActiveGoal("replacement_stale_redispatch_failed", *stale_candidate_goal);
      }
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Rejecting candidate for goal %llu: map publication sequence %llu is stale; current=%llu. "
      "Stopping and dispatching a fresh planner request.",
      static_cast<unsigned long long>(*stale_candidate_goal),
      static_cast<unsigned long long>(candidate_publication_sequence),
      static_cast<unsigned long long>(current_publication_sequence));
    suspendActiveGoal("stale_candidate_rejected", *stale_candidate_goal);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  geometry_msgs::msg::TransformStamped validation_from_reference;
  if (!active_goal_ || active_goal_->cancel_requested ||
      !candidate_reference_ || !planner_ready_status_ || !mapReadyLocked() ||
      !localizationHealthyLocked() ||
      planner_ready_status_->goal_id != active_goal_->id ||
      planner_ready_status_->localization_epoch !=
          active_goal_->localization_epoch ||
      planner_ready_status_->plan_request_sequence !=
          active_goal_->expected_plan_request_sequence ||
      (localization_epoch_ &&
       planner_ready_status_->localization_epoch != *localization_epoch_) ||
      planner_ready_status_->map_publication_sequence == 0U ||
      !candidateReferenceMapUsableLocked(&validation_from_reference) ||
      candidate_reference_->poses.size() < 2 ||
      !sameStamp(candidate_reference_->header.stamp,
                 planner_ready_status_->reference_stamp)) {
    return;
  }
  const std::uint8_t yaw_authority = planner_ready_status_->yaw_authority;
  const bool requires_gimbal_lock = planner_ready_status_->requires_gimbal_lock;
  if (yaw_authority != PlannerStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED &&
      yaw_authority != PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW &&
      yaw_authority != PlannerStatus::YAW_AUTHORITY_SPIN) {
    fail_stop_ = true;
    noteStopReason("commit_unsupported_yaw_authority");
    return;
  }
  // With a serial gimbal enabled, every reference is bound to a fresh
  // request/ack.  A default mode status with request_sequence=0 is not an
  // authorization for that profile; the Gazebo profile deliberately uses
  // zero because its chassis adapter consumes simulated joint state instead.
  const bool replacing = active_goal_->replacement_pending && active_execution_command_ &&
    lifecycle_.state() == GoalLifecycleState::kTracking &&
    active_execution_command_->yaw_authority == yaw_authority &&
    active_execution_command_->requires_gimbal_lock == requires_gimbal_lock;
  // 替换且航向权限不变时沿用已确认的云台授权；权限变化走 onPlannerStatus 的保护切换（急停）。
  if (!replacing) {
    requestYawAuthorityLocked(*planner_ready_status_);
  }
  // The Gazebo profile has no serial gimbal-status producer.  In that
  // explicitly selected profile the chassis adapter owns the yaw transform,
  // so a zero request sequence is the deliberate "no gimbal lease"
  // representation.  Real-vehicle profiles keep the request/ack gate.
  // 替换时 pending_yaw_authority_request_ 已在上一次提交末尾清空（见本函数结尾），
  // 而 replacing 又跳过了 requestYawAuthorityLocked，所以替换不能再要求"有待确认的请求"
  // （5.1.6：domain 111 的无停车替换每次都在这里急停）。改为按正在执行的命令已确认的
  // request/feedback 序号复核云台租约，与 onTick 对 active command 的持续复核是同一道门。
  const bool gimbal_authorized = replacing ?
    gimbalStatusSatisfiesLocked(
      yaw_authority, requires_gimbal_lock,
      active_execution_command_->gimbal_feedback_sequence,
      active_execution_command_->gimbal_request_sequence) :
    (pending_yaw_authority_request_ &&
     gimbalStatusSatisfiesLocked(yaw_authority, requires_gimbal_lock, 0));
  if (require_gimbal_status_ && !gimbal_authorized) {
    fail_stop_ = true;
    noteStopReason(replacing ? "commit_replacement_gimbal_lease_lost" :
      "commit_gimbal_ack_missing");
    return;
  }
  // MPC records the STOP using its own /clock callback. A newly committed
  // reference stamped in the same callback can therefore look older at the
  // consumer even though it was produced after the STOP. Do not weaken the
  // MPC stale-reference gate; wait one bounded window before final commit.
  const rclcpp::Time commit_stamp = now();
  if (!reference_commit_barrier_.permits(commit_stamp.nanoseconds())) {
    return;
  }
  if (replacing) {
    if (!lifecycle_.referenceReplaced(active_goal_->id, true)) {
      return;
    }
  } else if (!lifecycle_.referenceReady(active_goal_->id, true)) {
    return;
  }

  // A single execution command is the only MPC authorization.  It packages
  // the final retimed reference and all version fields in one DDS sample;
  // the legacy Path/Bool topics below are diagnostics only.
  nav_msgs::msg::Path committed = *candidate_reference_;
  if (planner_ready_status_->map_publication_sequence < map_status_publication_sequence_) {
    RCLCPP_INFO(get_logger(),
      "S3 candidate revalidated goal=%llu request=%llu planned_publication=%llu "
      "validated_publication=%llu frame=%s",
      static_cast<unsigned long long>(active_goal_->id),
      static_cast<unsigned long long>(planner_ready_status_->plan_request_sequence),
      static_cast<unsigned long long>(planner_ready_status_->map_publication_sequence),
      static_cast<unsigned long long>(map_status_publication_sequence_),
      committed.header.frame_id.c_str());
  }
  rebasePathTimestamps(committed, commit_stamp);
  ExecutionCommand command;
  command.header = committed.header;
  command.mode = ExecutionCommand::MODE_EXECUTE;
  command.goal_id = active_goal_->id;
  command.localization_epoch = active_goal_->localization_epoch;
  command.map_generation = planner_ready_status_->map_generation;
  command.map_publication_sequence =
    planner_ready_status_->map_publication_sequence;
  command.failure_reason = PlannerStatus::FAILURE_NONE;
  command.yaw_authority = yaw_authority;
  command.requires_gimbal_lock = requires_gimbal_lock;
  command.gimbal_request_sequence = replacing ?
    active_execution_command_->gimbal_request_sequence :
    (pending_yaw_authority_request_ ? pending_yaw_authority_request_->request_sequence : 0);
  command.gimbal_feedback_sequence = gimbal_status_ ? gimbal_status_->sequence : 0;
  command.reference = committed;
  const std::size_t expected_pts = committed.poses.size();
  const bool has_derivatives = !planner_ready_status_->reference_vx.empty();
  if (has_derivatives) {
    const auto allFiniteVec = [](const std::vector<double> & v) {
        return std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);});
      };
    if (planner_ready_status_->reference_vx.size() != expected_pts ||
      planner_ready_status_->reference_vy.size() != expected_pts ||
      planner_ready_status_->reference_ax.size() != expected_pts ||
      planner_ready_status_->reference_ay.size() != expected_pts ||
      planner_ready_status_->reference_yaw_rate.size() != expected_pts ||
      !allFiniteVec(planner_ready_status_->reference_vx) ||
      !allFiniteVec(planner_ready_status_->reference_vy) ||
      !allFiniteVec(planner_ready_status_->reference_ax) ||
      !allFiniteVec(planner_ready_status_->reference_ay) ||
      !allFiniteVec(planner_ready_status_->reference_yaw_rate))
    {
      fail_stop_ = true;
      noteStopReason("commit_malformed_reference_derivatives");
      return;
    }
    command.reference_vx = planner_ready_status_->reference_vx;
    command.reference_vy = planner_ready_status_->reference_vy;
    command.reference_ax = planner_ready_status_->reference_ax;
    command.reference_ay = planner_ready_status_->reference_ay;
    command.reference_yaw_rate = planner_ready_status_->reference_yaw_rate;
  }
  command.is_partial_subgoal = planner_ready_status_->is_partial_subgoal;
  command.has_spliced_tail = planner_ready_status_->has_spliced_tail;
  if (require_planning_snapshot_) {
    // The full-path check above and this assignment share mutex_. Preserve
    // MINCO provenance; bind the actual final validation independently.
    command.validation_map_publication_sequence = planning_snapshot_->publication_sequence;
    command.validation_map_source_generation = planning_snapshot_->source_generation;
    command.validation_from_reference = validation_from_reference;
  }
  fillCandidateDigest(command.candidate_content_digest,
                      planner_ready_status_->content_digest);
  fail_stop_ = false;
  active_goal_->recovering = false;
  active_goal_->wait_for_publication_after = 0;
  active_goal_->committed_plan_request_sequence = planner_ready_status_->plan_request_sequence;
  active_goal_->committed_planning_target = active_goal_->requested_planning_target;
  if (has_current_planning_pose_) {
    active_goal_->last_rolling_commit_pos = current_planning_pose_.pose.position;
  } else if (!committed.poses.empty()) {
    active_goal_->last_rolling_commit_pos = committed.poses.front().pose.position;
  }
  active_goal_->last_rolling_commit_time = commit_stamp;
  active_goal_->last_rolling_publication_sequence =
    planning_snapshot_ ? planning_snapshot_->publication_sequence :
    planner_ready_status_->map_publication_sequence;
  if (replacing) {
    RCLCPP_INFO(
      get_logger(), "【无停车重规划】目标=%llu 新参考已替换（请求序号=%llu），全程未急停。",
      static_cast<unsigned long long>(active_goal_->id),
      static_cast<unsigned long long>(planner_ready_status_->plan_request_sequence));
  }
  active_goal_->replacement_pending = false;
  active_execution_command_ = command;
  PlanProgressIdentity progress_identity;
  progress_identity.goal_id = active_goal_->id;
  progress_identity.localization_epoch = active_goal_->localization_epoch;
  progress_identity.map_generation = planner_ready_status_->map_generation;
  progress_identity.map_publication_sequence =
    planner_ready_status_->map_publication_sequence;
  progress_identity.plan_request_sequence =
    planner_ready_status_->plan_request_sequence;
  if (planning_snapshot_) {
    progress_identity.source_generation = planning_snapshot_->source_generation;
  }
  progress_reference_hint_ = 0;
  if (has_current_pose_) {
    progress_watchdog_.observeReference(
      progress_identity,
      progressDistanceLocked(
        std::hypot(active_goal_->target.pose.position.x - current_pose_.pose.position.x,
                   active_goal_->target.pose.position.y - current_pose_.pose.position.y)),
      progressMotionTime());
  }
  RCLCPP_INFO(
    get_logger(),
    "P2 reference commit goal=%llu localization_epoch=%llu plan_request_sequence=%llu "
    "source_generation=%llu map_generation=%llu map_publication_sequence=%llu "
    "reference_stamp_ns=%lld poses=%zu published_digest=%s occupancy_digest=%s "
    "gate_collisions=%u gate_indices=%s grid_topic=%s validation_frame=%s "
    "validated_publication_sequence=%llu validated_source_generation=%llu",
    static_cast<unsigned long long>(command.goal_id),
    static_cast<unsigned long long>(command.localization_epoch),
    static_cast<unsigned long long>(planner_ready_status_->plan_request_sequence),
    static_cast<unsigned long long>(
      planning_snapshot_ ? planning_snapshot_->source_generation : 0U),
    static_cast<unsigned long long>(command.map_generation),
    static_cast<unsigned long long>(command.map_publication_sequence),
    static_cast<long long>(rclcpp::Time(command.reference.header.stamp).nanoseconds()),
    command.reference.poses.size(),
    planner_ready_status_->content_digest.c_str(),
    planner_ready_status_->occupancy_digest.c_str(),
    planner_ready_status_->gate_collision_count,
    planner_ready_status_->gate_collision_indices.c_str(),
    planner_ready_status_->grid_topic.c_str(),
    planner_ready_status_->validation_frame.c_str(),
    static_cast<unsigned long long>(command.validation_map_publication_sequence),
    static_cast<unsigned long long>(command.validation_map_source_generation));
  publishEmergencyStop(false);
  publishExecutionCommand(command);
  reference_path_pub_->publish(committed);
  candidate_reference_.reset();
  planner_ready_status_.reset();
  pending_yaw_authority_request_.reset();
}


} // namespace ats_goal_manager


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_goal_manager::AtsGoalManagerNode>());
  rclcpp::shutdown();
  return 0;
}
