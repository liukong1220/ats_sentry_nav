#ifndef ATS_GOAL_MANAGER__ATS_GOAL_MANAGER_NODE_IMPL_HPP_
#define ATS_GOAL_MANAGER__ATS_GOAL_MANAGER_NODE_IMPL_HPP_

// Copyright 2026

#include "ats_goal_manager/goal_deadline.hpp"
#include "ats_goal_manager/goal_lifecycle.hpp"
#include "ats_goal_manager/plan_progress_watchdog.hpp"
#include "ats_goal_manager/planner_failure_policy.hpp"
#include "ats_goal_manager/planning_snapshot_safety.hpp"
#include "ats_goal_manager/terminal_motion_window.hpp"
#include "ats_goal_manager/stop_reference_barrier.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "ats_navigation_interfaces/action/navigate_to_pose.hpp"
#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "ats_navigation_interfaces/msg/yaw_authority_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "builtin_interfaces/msg/time.hpp"

namespace ats_goal_manager {



using NavigateToPose = ats_navigation_interfaces::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ServerGoalHandle<NavigateToPose>;
using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;
using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
using GimbalYawStatus = ats_navigation_interfaces::msg::GimbalYawStatus;
using PlanningMapStatus = ats_navigation_interfaces::msg::PlanningMapStatus;
using PlanningMapSnapshot = ats_navigation_interfaces::msg::PlanningMapSnapshot;
using PlannerGoal = ats_navigation_interfaces::msg::PlannerGoal;
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
using YawAuthorityRequest = ats_navigation_interfaces::msg::YawAuthorityRequest;

inline void fillCandidateDigest(std::array<std::uint8_t, 32> & out, const std::string & hex) {
  out.fill(0U);
  auto from_hex = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
      return c - 'A' + 10;
    }
    return -1;
  };
  for (std::size_t i = 0; i + 1U < hex.size() && i / 2U < out.size(); i += 2U) {
    const int high = from_hex(hex[i]);
    const int low = from_hex(hex[i + 1U]);
    if (high < 0 || low < 0) {
      break;
    }
    out[i / 2U] = static_cast<std::uint8_t>((high << 4) | low);
  }
}

inline const char *plannerFailureMessage(std::uint8_t failure_reason) {
  switch (failure_reason) {
  case PlannerStatus::FAILURE_INVALID_GOAL:
    return "planner rejected invalid goal";
  case PlannerStatus::FAILURE_MAP_UNREADY:
    return "planning map is unavailable";
  case PlannerStatus::FAILURE_START_TF:
  case PlannerStatus::FAILURE_GOAL_TF:
  case PlannerStatus::FAILURE_REFERENCE_TF:
    return "planner transform is unavailable";
  case PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED:
    return "start or goal is occupied";
  case PlannerStatus::FAILURE_NO_PATH:
    return "planner found no path";
  case PlannerStatus::FAILURE_START_ENCLOSED:
    return "robot start cell is enclosed";
  case PlannerStatus::FAILURE_OPTIMIZER:
  case PlannerStatus::FAILURE_REPAIR:
    return "trajectory optimization failed";
  case PlannerStatus::FAILURE_FOOTPRINT:
  case PlannerStatus::FAILURE_RUNTIME_UNSAFE:
    return "trajectory footprint is unsafe";
  case PlannerStatus::FAILURE_SNAPSHOT_CHANGED:
    return "planning snapshot changed before commit";
  default:
    return "MINCO planning failed";
  }
}

inline bool sameStamp(const builtin_interfaces::msg::Time &left,
               const builtin_interfaces::msg::Time &right) {
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

inline bool finitePose(const geometry_msgs::msg::Pose &pose) {
  const double q_norm = pose.orientation.x * pose.orientation.x +
                        pose.orientation.y * pose.orientation.y +
                        pose.orientation.z * pose.orientation.z +
                        pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && q_norm > 1e-8;
}

inline std::chrono::steady_clock::duration secondsToDuration(double seconds) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, seconds)));
}

inline std::int64_t secondsToNanoseconds(double seconds) {
  return static_cast<std::int64_t>(std::llround(std::max(0.0, seconds) * 1e9));
}

inline std::uint64_t managerIncarnation() {
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count();
  return nanoseconds > 0 ? static_cast<std::uint64_t>(nanoseconds) : 1;
}


class AtsGoalManagerNode : public rclcpp::Node {
public:
  AtsGoalManagerNode();

private:
  enum class ReplanTriggerReason {
    kFarObstacleConflict,
    kEndpointRetarget,
    kRollingReplan,
  };

  struct ReplanTriggerInfo {
    ReplanTriggerReason reason{ReplanTriggerReason::kFarObstacleConflict};
    double drift_m{0.0};
    double threshold_m{0.0};
    double distance_to_goal_m{0.0};
    double advanced_m{0.0};
    double dist_to_subgoal_m{0.0};
  };

  struct ActiveGoal {
    std::uint64_t id{0};
    geometry_msgs::msg::PoseStamped target;
    // 按节点时钟计（use_sim_time 下是 /clock），见 goal_deadline.hpp。
    GoalDeadline deadline;
    std::shared_ptr<GoalHandleNavigateToPose> action_handle;
    bool cancel_requested{false};
    std::uint64_t localization_epoch{0};
    std::uint64_t next_plan_request_sequence{0};
    std::uint64_t expected_plan_request_sequence{0};
    std::optional<std::chrono::steady_clock::time_point> waiting_since;
    bool recovering{false};
    std::uint64_t wait_for_publication_after{0};
    // 当前已下发执行的参考对应的规划请求序号。
    std::uint64_t committed_plan_request_sequence{0};
    // Keep the requested target separate from MINCO's relocated endpoint.
    std::optional<geometry_msgs::msg::PoseStamped> requested_planning_target{std::nullopt};
    std::optional<geometry_msgs::msg::PoseStamped> committed_planning_target{std::nullopt};
    // 无停车重规划进行中：旧参考继续执行，expected_plan_request_sequence 指向新请求。
    bool replacement_pending{false};
    ReplanTriggerReason pending_replan_reason{ReplanTriggerReason::kFarObstacleConflict};
    std::optional<std::chrono::steady_clock::time_point> last_non_urgent_replan_time{std::nullopt};
    std::optional<std::chrono::steady_clock::time_point> last_rolling_replan_fail_time{std::nullopt};
    double requested_position_tolerance{0.0};
    std::optional<geometry_msgs::msg::Point> last_rolling_commit_pos{std::nullopt};
    std::optional<rclcpp::Time> last_rolling_commit_time{std::nullopt};
    std::uint64_t last_rolling_publication_sequence{0};
  };


  void loadParameters();
  rclcpp_action::GoalResponse onActionGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse onActionCancel(
      const std::shared_ptr<GoalHandleNavigateToPose> handle);
  void onActionAccepted(
      const std::shared_ptr<GoalHandleNavigateToPose> handle);
  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void startGoal(const geometry_msgs::msg::PoseStamped &target,
                 const std::shared_ptr<GoalHandleNavigateToPose> &handle,
                 double timeout_sec, double position_tolerance = 0.0);
  bool publishPlannerGoal(std::uint64_t id, std::uint64_t localization_epoch,
                          const geometry_msgs::msg::PoseStamped &canonical_target,
                          std::uint64_t required_snapshot_sequence = 0);
  void suspendActiveGoal(
      const char *site, std::uint64_t id,
      std::optional<std::chrono::steady_clock::time_point> waiting_since = std::nullopt,
      std::uint64_t wait_for_publication_after = 0);
  void onPlannerStatus(const PlannerStatus::SharedPtr message);
  double effectiveRetargetThreshold(double distance_to_goal) const;
  void onReplanAdvised(const PlannerStatus & message);
  void onReplanAdvised(
      const PlannerStatus & message, const ReplanTriggerInfo & trigger);
  void onCandidateReference(const nav_msgs::msg::Path::SharedPtr message);
  void onMapReady(const std_msgs::msg::Bool::SharedPtr message);
  void onMapStatus(const PlanningMapStatus::SharedPtr message);
  void onPlanningSnapshot(const PlanningMapSnapshot::SharedPtr message);
  static bool localizationStateAllowsPlanning(std::uint8_t state);
  void onLocalizationStatus(const LocalizationStatus::SharedPtr message);
  void onGimbalYawStatus(const GimbalYawStatus::SharedPtr message);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  PlanProgressWatchdog::Clock::time_point progressMotionTime();
  double progressDistanceLocked(double straight_distance);
  void tryCommitReference();
  void onTick();
  void finishActive(std::uint8_t result_code, const std::string &message,
                    GoalLifecycleState terminal_state);
  void completeStandaloneAction(
      const std::shared_ptr<GoalHandleNavigateToPose> &handle,
      std::uint8_t result_code, const std::string &message);
  bool mapReadyLocked() const;
  bool mapSourceAliveLocked() const;
  bool planningSnapshotUsableLocked(
      std::uint64_t publication_sequence, std::uint64_t localization_epoch,
      bool allow_newer_publication = false) const;
  bool candidateReferenceMapUsableLocked(
      geometry_msgs::msg::TransformStamped * used_transform = nullptr) const;
  bool recoverySnapshotReadyLocked(const ActiveGoal & goal) const;
  bool localizationHealthyLocked() const;
  bool gimbalStatusSatisfiesLocked(
      std::uint8_t yaw_authority, bool requires_gimbal_lock,
      std::uint64_t minimum_feedback_sequence,
      std::uint64_t required_request_sequence = 0) const;
  void requestYawAuthorityLocked(const PlannerStatus & status);
  bool normalizePose(const geometry_msgs::msg::PoseStamped &input,
                     const std::string &target_frame,
                     geometry_msgs::msg::PoseStamped &output,
                     std::string &reason,
                     bool use_latest_transform = true) const;
  void publishFeedbackLocked(double elapsed_sec, double distance);
  static std::uint8_t actionState(GoalLifecycleState state);
  static std::string stateName(GoalLifecycleState state);
  void rebasePathTimestamps(nav_msgs::msg::Path &path,
                            const rclcpp::Time &base_time) const;
  void publishEmergencyStop(bool stop);
  void noteStopReason(const std::string &reason);
  void publishExecutionCommand(ExecutionCommand command);
  void publishExecutionStop(
      std::uint64_t goal_id, std::uint64_t localization_epoch,
      std::uint8_t failure_reason, std::uint64_t map_generation,
      std::uint64_t map_publication_sequence);

  std::string input_goal_topic_;
  std::string planner_goal_topic_;
  std::string planner_status_topic_;
  std::string candidate_reference_topic_;
  std::string reference_path_topic_;
  std::string emergency_stop_topic_;
  std::string execution_command_topic_;
  std::string yaw_authority_request_topic_;
  std::string gimbal_status_topic_;
  std::string map_ready_topic_;
  std::string map_status_topic_;
  std::string planning_snapshot_topic_;
  std::string localization_status_topic_;
  std::string odom_topic_;
  std::string action_name_;
  std::string goal_frame_;
  std::string planning_frame_;
  double map_ready_timeout_sec_{3.0};
  double localization_status_timeout_sec_{1.0};
  double emergency_stop_heartbeat_period_sec_{0.1};
  double tick_rate_hz_{0.0};
  double tf_future_tolerance_sec_{0.2};
  bool enforce_gimbal_lock_on_execution_{true};
  std::optional<std::chrono::steady_clock::time_point> last_heartbeat_publish_time_;
  double stop_to_reference_guard_sec_{0.01};
  double map_wait_timeout_sec_{5.0};
  double default_goal_timeout_sec_{120.0};
  double execution_command_stamp_backdate_sec_{0.02};
  double goal_position_tolerance_{0.08};
  double max_goal_position_tolerance_{0.08};
  double progress_hold_distance_m_{0.08};
  bool progress_along_reference_{false};
  std::size_t progress_reference_hint_{0};
  double goal_yaw_tolerance_{0.15};
  TerminalMotionWindow terminal_motion_;
  double contact_hold_max_depth_m_{0.12};
  double terminal_linear_velocity_tolerance_{0.05};
  double terminal_angular_velocity_tolerance_{0.10};
  double terminal_dwell_sec_{0.30};
  double planning_snapshot_timeout_sec_{5.0};
  double gimbal_status_timeout_sec_{0.5};
  bool require_localization_status_{false};
  bool require_accepted_localization_{false};
  bool planner_start_in_planning_frame_{false};
  bool has_accepted_localization_{false};
  double retarget_threshold_{0.03};
  double retarget_cruise_threshold_{0.20};
  double retarget_terminal_distance_m_{1.5};
  double retarget_cruise_distance_m_{2.0};
  double replan_advised_cooldown_sec_{1.5};
  double map_transform_timeout_sec_{1.0};
  bool require_map_status_{true};
  bool require_gimbal_status_{true};
  bool require_planning_snapshot_{false};
  bool rolling_replan_enabled_{true};
  double rolling_replan_hz_{10.0};
  double rolling_replan_advance_m_{2.0};
  double rolling_replan_stop_distance_m_{1.5};

  std::mutex mutex_;
  std::mutex emergency_stop_log_mutex_;
  GoalLifecycle lifecycle_;
  std::uint64_t next_goal_id_{0};
  std::optional<ActiveGoal> active_goal_;
  std::optional<nav_msgs::msg::Path> candidate_reference_;
  std::optional<PlannerStatus> planner_ready_status_;
  std::optional<ExecutionCommand> active_execution_command_;
  std::optional<YawAuthorityRequest> pending_yaw_authority_request_;
  std::optional<GimbalYawStatus> gimbal_status_;
  bool map_ready_signal_{false};
  bool map_status_ready_{false};
  bool map_status_heartbeat_ready_{false};
  bool map_ready_heartbeat_value_{false};
  bool fail_stop_{true};
  bool replan_without_stop_enabled_{false};
  std::optional<bool> last_emergency_stop_published_;
  StopReferenceBarrier reference_commit_barrier_;
  std::string stop_reason_;
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;
  std::optional<std::chrono::steady_clock::time_point> last_map_status_signal_;
  std::optional<std::chrono::steady_clock::time_point> last_planning_snapshot_signal_;
  std::optional<PlanningMapSnapshot> planning_snapshot_;
  std::uint64_t map_status_localization_epoch_{0};
  std::uint64_t map_status_generation_{0};
  std::uint64_t map_status_publication_sequence_{0};
  std::uint64_t manager_incarnation_{0};
  std::atomic<std::uint64_t> next_execution_command_sequence_{0};
  std::atomic<std::uint64_t> next_yaw_authority_request_sequence_{0};
  std::optional<std::chrono::steady_clock::time_point>
      last_localization_status_signal_;
  std::optional<std::chrono::steady_clock::time_point>
      last_gimbal_status_signal_;
  std::optional<std::chrono::steady_clock::time_point> terminal_converged_since_;
  std::optional<std::uint64_t> localization_epoch_;
  std::uint8_t localization_status_{LocalizationStatus::STATE_UNINITIALIZED};
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped current_planning_pose_;
  builtin_interfaces::msg::Time current_localization_stamp_;
  bool has_current_pose_{false};
  bool has_current_planning_pose_{false};
  double current_linear_velocity_{std::numeric_limits<double>::infinity()};
  double current_angular_velocity_{std::numeric_limits<double>::infinity()};
  double current_world_vx_{0.0};
  double current_world_vy_{0.0};
  double current_world_wz_{0.0};
  bool has_current_velocity_{false};
  bool odom_tf_healthy_{false};
  PlanProgressWatchdog progress_watchdog_;
  PlanningSnapshotSafetyParams planning_snapshot_safety_params_;
  double ego_contact_max_depth_m_{0.0};

  rclcpp::Publisher<PlannerGoal>::SharedPtr planner_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<ExecutionCommand>::SharedPtr execution_command_pub_;
  rclcpp::Publisher<YawAuthorityRequest>::SharedPtr yaw_authority_request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      input_goal_sub_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr candidate_reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::Subscription<PlanningMapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<PlanningMapSnapshot>::SharedPtr planning_snapshot_sub_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<GimbalYawStatus>::SharedPtr gimbal_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__ATS_GOAL_MANAGER_NODE_IMPL_HPP_
