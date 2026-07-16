// Copyright 2026

#include "ats_goal_manager/goal_lifecycle.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "ats_navigation_interfaces/action/navigate_to_pose.hpp"
#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
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

namespace ats_goal_manager
{

namespace
{

using NavigateToPose = ats_navigation_interfaces::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using PlannerGoal = ats_navigation_interfaces::msg::PlannerGoal;
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;

bool sameStamp(
  const builtin_interfaces::msg::Time & left, const builtin_interfaces::msg::Time & right)
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const double q_norm =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && q_norm > 1e-8;
}

std::chrono::steady_clock::duration secondsToDuration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, seconds)));
}

}  // namespace

class AtsGoalManagerNode : public rclcpp::Node
{
public:
  AtsGoalManagerNode()
  : Node("ats_goal_manager"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    planner_goal_pub_ = create_publisher<PlannerGoal>(planner_goal_topic_, rclcpp::QoS(10));
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(1));
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());
    input_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&AtsGoalManagerNode::onTopicGoal, this, std::placeholders::_1));
    planner_status_sub_ = create_subscription<PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&AtsGoalManagerNode::onPlannerStatus, this, std::placeholders::_1));
    candidate_reference_sub_ = create_subscription<nav_msgs::msg::Path>(
      candidate_reference_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&AtsGoalManagerNode::onCandidateReference, this, std::placeholders::_1));
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsGoalManagerNode::onMapReady, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AtsGoalManagerNode::onOdometry, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, action_name_,
      std::bind(&AtsGoalManagerNode::onActionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AtsGoalManagerNode::onActionCancel, this, std::placeholders::_1),
      std::bind(&AtsGoalManagerNode::onActionAccepted, this, std::placeholders::_1));
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(emergency_stop_heartbeat_period_sec_),
      std::bind(&AtsGoalManagerNode::onTick, this));

    // 上电和任何无任务状态都保持急停，禁止 DDS late joiner 获得旧 reference 后自行运动。
    publishEmergencyStop(true);
    RCLCPP_INFO(
      get_logger(), "ATS goal manager ready: action='%s' topic='%s' planner='%s'",
      action_name_.c_str(), input_goal_topic_.c_str(), planner_goal_topic_.c_str());
  }

private:
  struct ActiveGoal
  {
    std::uint64_t id{0};
    geometry_msgs::msg::PoseStamped target;
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::duration timeout{};
    std::shared_ptr<GoalHandleNavigateToPose> action_handle;
    bool cancel_requested{false};
  };

  void loadParameters()
  {
    input_goal_topic_ = declare_parameter<std::string>("input_goal_topic", "/goal_pose");
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
    map_ready_topic_ = declare_parameter<std::string>("map_ready_topic", "/rog_map_adapter/ready");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
    action_name_ = declare_parameter<std::string>("action_name", "/ats_navigate_to_pose");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    map_ready_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("map_ready_timeout_sec", 3.0));
    emergency_stop_heartbeat_period_sec_ = std::max(
      0.02, declare_parameter<double>("emergency_stop_heartbeat_period_sec", 0.1));
    map_wait_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("map_wait_timeout_sec", 5.0));
    default_goal_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("default_goal_timeout_sec", 120.0));
    goal_position_tolerance_ = std::max(
      0.0, declare_parameter<double>("goal_position_tolerance", 0.08));
    goal_yaw_tolerance_ = std::max(
      0.0, declare_parameter<double>("goal_yaw_tolerance", 0.15));
  }

  rclcpp_action::GoalResponse onActionGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (!goal || !finitePose(goal->goal_pose.pose)) {
      RCLCPP_WARN(get_logger(), "Rejected action goal with non-finite pose or zero quaternion.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onActionCancel(const std::shared_ptr<GoalHandleNavigateToPose> handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_ && active_goal_->action_handle == handle) {
        active_goal_->cancel_requested = true;
        fail_stop_ = true;
      }
    }
    // cancel 回调立即续发急停；最终 result 由 tick 统一收敛，避免与 planner 回调竞态提交 reference。
    publishEmergencyStop(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onActionAccepted(const std::shared_ptr<GoalHandleNavigateToPose> handle)
  {
    geometry_msgs::msg::PoseStamped target;
    std::string reason;
    if (!normalizePose(handle->get_goal()->goal_pose, target, reason)) {
      completeStandaloneAction(handle, NavigateToPose::Result::RESULT_TF_FAILED, reason);
      publishEmergencyStop(true);
      return;
    }
    const auto & timeout = handle->get_goal()->timeout;
    const double requested_timeout =
      static_cast<double>(timeout.sec) + 1e-9 * static_cast<double>(timeout.nanosec);
    startGoal(target, handle, requested_timeout > 0.0 ? requested_timeout : default_goal_timeout_sec_);
  }

  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    geometry_msgs::msg::PoseStamped target;
    std::string reason;
    if (!normalizePose(*message, target, reason)) {
      RCLCPP_ERROR(get_logger(), "Rejected /goal_pose: %s", reason.c_str());
      publishEmergencyStop(true);
      return;
    }
    startGoal(target, nullptr, default_goal_timeout_sec_);
  }

  void startGoal(
    const geometry_msgs::msg::PoseStamped & target,
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    double timeout_sec)
  {
    std::shared_ptr<GoalHandleNavigateToPose> preempted;
    std::uint64_t id = 0;
    bool dispatch_now = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_) {
        preempted = active_goal_->action_handle;
        lifecycle_.preempt();
      }
      candidate_reference_.reset();
      planner_ready_status_.reset();
      id = ++next_goal_id_;
      lifecycle_.start(id, mapReadyLocked());
      active_goal_ = ActiveGoal{
        id, target, std::chrono::steady_clock::now(), secondsToDuration(timeout_sec), handle, false};
      fail_stop_ = true;
      dispatch_now = lifecycle_.state() == GoalLifecycleState::kPlanning;
    }
    // 先停止旧任务；即使旧 MINCO 回调晚到，也会因 goal_id 不匹配而被丢弃。
    publishEmergencyStop(true);
    if (preempted) {
      completeStandaloneAction(
        preempted, NavigateToPose::Result::RESULT_PREEMPTED, "preempted by a newer ATS goal");
    }
    if (dispatch_now) {
      publishPlannerGoal(id, target);
    }
  }

  void publishPlannerGoal(std::uint64_t id, const geometry_msgs::msg::PoseStamped & target)
  {
    PlannerGoal request;
    request.header.stamp = now();
    request.header.frame_id = planning_frame_;
    request.goal_id = id;
    request.goal_pose = target;
    planner_goal_pub_->publish(request);
  }

  void onPlannerStatus(const PlannerStatus::SharedPtr message)
  {
    if (message->state == PlannerStatus::STATE_FAILED) {
      bool matches = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        matches = active_goal_ && active_goal_->id == message->goal_id;
      }
      if (matches) {
        finishActive(
          NavigateToPose::Result::RESULT_PLANNING_FAILED,
          message->reason.empty() ? "MINCO planning failed" : message->reason,
          GoalLifecycleState::kFailed);
      }
      return;
    }
    if (message->state != PlannerStatus::STATE_REFERENCE_READY) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_ && active_goal_->id == message->goal_id) {
        planner_ready_status_ = *message;
      }
    }
    tryCommitReference();
  }

  void onCandidateReference(const nav_msgs::msg::Path::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_reference_ = *message;
    }
    tryCommitReference();
  }

  void onMapReady(const std_msgs::msg::Bool::SharedPtr message)
  {
    bool fail_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_ready_signal_ = message->data;
      last_map_ready_signal_ = std::chrono::steady_clock::now();
      fail_active = active_goal_ && !message->data;
    }
    if (fail_active) {
      finishActive(
        NavigateToPose::Result::RESULT_MAP_UNREADY, "ROGMap adapter reported not-ready",
        GoalLifecycleState::kFailed);
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    geometry_msgs::msg::PoseStamped input;
    input.header = message->header;
    input.pose = message->pose.pose;
    geometry_msgs::msg::PoseStamped normalized;
    std::string reason;
    if (!normalizePose(input, normalized, reason)) {
      bool has_active = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        has_active = active_goal_.has_value();
      }
      if (has_active) {
        finishActive(NavigateToPose::Result::RESULT_TF_FAILED, reason, GoalLifecycleState::kFailed);
      }
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = normalized;
    has_current_pose_ = true;
  }

  void tryCommitReference()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_ || active_goal_->cancel_requested || !candidate_reference_ ||
      !planner_ready_status_ || !mapReadyLocked() ||
      planner_ready_status_->goal_id != active_goal_->id ||
      candidate_reference_->poses.size() < 2 ||
      !sameStamp(candidate_reference_->header.stamp, planner_ready_status_->reference_stamp))
    {
      return;
    }
    if (!lifecycle_.referenceReady(active_goal_->id, true)) {
      return;
    }

    // 仅在当前 map heartbeat、candidate stamp 和 goal_id 同时复核成功后提交。
    // 保留 MINCO 的相对采样时间，并在同一互斥区严格先解除急停、再发布 reference。
    nav_msgs::msg::Path committed = *candidate_reference_;
    rebasePathTimestamps(committed, now());
    fail_stop_ = false;
    publishEmergencyStop(false);
    reference_path_pub_->publish(committed);
    candidate_reference_.reset();
    planner_ready_status_.reset();
  }

  void onTick()
  {
    std::optional<ActiveGoal> snapshot;
    bool dispatch = false;
    bool cancel = false;
    bool timeout = false;
    bool map_stale = false;
    bool map_wait_timeout = false;
    bool reached = false;
    double distance = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_) {
        snapshot = active_goal_;
        const auto elapsed = std::chrono::steady_clock::now() - active_goal_->started;
        cancel = active_goal_->cancel_requested ||
          (active_goal_->action_handle && active_goal_->action_handle->is_canceling());
        timeout = elapsed >= active_goal_->timeout;
        const bool map_ready = mapReadyLocked();
        if (lifecycle_.mapReady(active_goal_->id) && map_ready) {
          dispatch = lifecycle_.mapBecameReady(active_goal_->id);
        }
        map_stale = lifecycle_.state() != GoalLifecycleState::kWaitingForMap && !map_ready;
        map_wait_timeout = lifecycle_.state() == GoalLifecycleState::kWaitingForMap &&
          elapsed >= secondsToDuration(map_wait_timeout_sec_);
        if (lifecycle_.state() == GoalLifecycleState::kTracking && has_current_pose_) {
          distance = std::hypot(
            active_goal_->target.pose.position.x - current_pose_.pose.position.x,
            active_goal_->target.pose.position.y - current_pose_.pose.position.y);
          const double yaw_delta = tf2::getYaw(active_goal_->target.pose.orientation) -
            tf2::getYaw(current_pose_.pose.orientation);
          const double yaw_error = std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
          reached = distance <= goal_position_tolerance_ && yaw_error <= goal_yaw_tolerance_;
        }
        publishFeedbackLocked(elapsed, distance);
      }
    }

    if (dispatch && snapshot) {
      publishPlannerGoal(snapshot->id, snapshot->target);
    }
    if (cancel) {
      finishActive(NavigateToPose::Result::RESULT_CANCELED, "goal canceled", GoalLifecycleState::kCanceled);
    } else if (timeout) {
      finishActive(NavigateToPose::Result::RESULT_TIMEOUT, "goal timeout", GoalLifecycleState::kTimedOut);
    } else if (map_stale) {
      finishActive(
        NavigateToPose::Result::RESULT_MAP_UNREADY, "map ready heartbeat lease expired",
        GoalLifecycleState::kFailed);
    } else if (map_wait_timeout) {
      finishActive(
        NavigateToPose::Result::RESULT_MAP_UNREADY, "map did not become ready before deadline",
        GoalLifecycleState::kFailed);
    } else if (reached) {
      finishActive(
        NavigateToPose::Result::RESULT_SUCCEEDED, "goal reached", GoalLifecycleState::kSucceeded);
    }

    bool stop = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop = fail_stop_ || lifecycle_.emergencyStopRequired();
    }
    publishEmergencyStop(stop);
  }

  void finishActive(
    std::uint8_t result_code, const std::string & message, GoalLifecycleState terminal_state)
  {
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
        distance = std::hypot(
          active_goal_->target.pose.position.x - current_pose_.pose.position.x,
          active_goal_->target.pose.position.y - current_pose_.pose.position.y);
      }
      switch (terminal_state) {
        case GoalLifecycleState::kSucceeded: lifecycle_.succeed(); break;
        case GoalLifecycleState::kCanceled: lifecycle_.cancel(); break;
        case GoalLifecycleState::kPreempted: lifecycle_.preempt(); break;
        case GoalLifecycleState::kTimedOut: lifecycle_.timeout(); break;
        default: lifecycle_.fail(); break;
      }
      active_goal_.reset();
      candidate_reference_.reset();
      planner_ready_status_.reset();
      fail_stop_ = true;
    }
    publishEmergencyStop(true);
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

  void completeStandaloneAction(
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    std::uint8_t result_code, const std::string & message)
  {
    if (!handle) {
      return;
    }
    auto result = std::make_shared<NavigateToPose::Result>();
    result->result_code = result_code;
    result->message = message;
    result->final_distance = std::numeric_limits<double>::infinity();
    handle->abort(result);
  }

  bool mapReadyLocked() const
  {
    return map_ready_signal_ && last_map_ready_signal_ &&
           std::chrono::steady_clock::now() - *last_map_ready_signal_ <=
             secondsToDuration(map_ready_timeout_sec_);
  }

  bool normalizePose(
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output,
    std::string & reason) const
  {
    if (!finitePose(input.pose)) {
      reason = "goal/odometry pose contains a non-finite value or zero quaternion";
      return false;
    }
    geometry_msgs::msg::PoseStamped source = input;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = planning_frame_;
    }
    if (source.header.frame_id == planning_frame_) {
      output = source;
      output.header.frame_id = planning_frame_;
      return true;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        planning_frame_, source.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
      tf2::doTransform(source, output, transform);
      output.header.frame_id = planning_frame_;
      return finitePose(output.pose);
    } catch (const tf2::TransformException & exception) {
      reason = "TF transform to " + planning_frame_ + " failed: " + exception.what();
      return false;
    }
  }

  void publishFeedbackLocked(
    const std::chrono::steady_clock::duration & elapsed, double distance)
  {
    if (!active_goal_ || !active_goal_->action_handle) {
      return;
    }
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->goal_id = active_goal_->id;
    feedback->state = actionState(lifecycle_.state());
    feedback->distance_remaining = distance;
    feedback->elapsed_sec = std::chrono::duration<double>(elapsed).count();
    feedback->status = stateName(lifecycle_.state());
    if (has_current_pose_) {
      feedback->current_pose = current_pose_;
    }
    active_goal_->action_handle->publish_feedback(feedback);
  }

  static std::uint8_t actionState(GoalLifecycleState state)
  {
    switch (state) {
      case GoalLifecycleState::kWaitingForMap: return NavigateToPose::Feedback::STATE_WAITING_FOR_MAP;
      case GoalLifecycleState::kPlanning: return NavigateToPose::Feedback::STATE_PLANNING;
      case GoalLifecycleState::kTracking: return NavigateToPose::Feedback::STATE_TRACKING;
      case GoalLifecycleState::kCanceled: return NavigateToPose::Feedback::STATE_CANCELING;
      case GoalLifecycleState::kIdle: return NavigateToPose::Feedback::STATE_ACCEPTED;
      default: return NavigateToPose::Feedback::STATE_STOPPED;
    }
  }

  static std::string stateName(GoalLifecycleState state)
  {
    switch (state) {
      case GoalLifecycleState::kWaitingForMap: return "waiting_for_map";
      case GoalLifecycleState::kPlanning: return "planning";
      case GoalLifecycleState::kTracking: return "tracking";
      case GoalLifecycleState::kCanceled: return "canceled";
      case GoalLifecycleState::kPreempted: return "preempted";
      case GoalLifecycleState::kTimedOut: return "timeout";
      case GoalLifecycleState::kSucceeded: return "succeeded";
      case GoalLifecycleState::kFailed: return "failed";
      default: return "idle";
    }
  }

  void rebasePathTimestamps(nav_msgs::msg::Path & path, const rclcpp::Time & base_time) const
  {
    if (path.poses.empty()) {
      return;
    }
    const rclcpp::Time old_base(path.poses.front().header.stamp);
    path.header.stamp = base_time;
    for (auto & pose : path.poses) {
      const rclcpp::Duration relative = rclcpp::Time(pose.header.stamp) - old_base;
      pose.header.stamp = base_time + relative;
      pose.header.frame_id = planning_frame_;
    }
    path.header.frame_id = planning_frame_;
  }

  void publishEmergencyStop(bool stop)
  {
    std_msgs::msg::Bool message;
    message.data = stop;
    emergency_stop_pub_->publish(message);
  }

  std::string input_goal_topic_;
  std::string planner_goal_topic_;
  std::string planner_status_topic_;
  std::string candidate_reference_topic_;
  std::string reference_path_topic_;
  std::string emergency_stop_topic_;
  std::string map_ready_topic_;
  std::string odom_topic_;
  std::string action_name_;
  std::string planning_frame_;
  double map_ready_timeout_sec_{3.0};
  double emergency_stop_heartbeat_period_sec_{0.1};
  double map_wait_timeout_sec_{5.0};
  double default_goal_timeout_sec_{120.0};
  double goal_position_tolerance_{0.08};
  double goal_yaw_tolerance_{0.15};

  std::mutex mutex_;
  GoalLifecycle lifecycle_;
  std::uint64_t next_goal_id_{0};
  std::optional<ActiveGoal> active_goal_;
  std::optional<nav_msgs::msg::Path> candidate_reference_;
  std::optional<PlannerStatus> planner_ready_status_;
  bool map_ready_signal_{false};
  bool fail_stop_{true};
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_current_pose_{false};

  rclcpp::Publisher<PlannerGoal>::SharedPtr planner_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_goal_sub_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr candidate_reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace ats_goal_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_goal_manager::AtsGoalManagerNode>());
  rclcpp::shutdown();
  return 0;
}
