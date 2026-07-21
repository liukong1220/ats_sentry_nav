// Copyright 2026

#include "ats_goal_manager/goal_lifecycle.hpp"

#include <chrono>
#include <atomic>
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
#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_status.hpp"
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

namespace ats_goal_manager {

namespace {

using NavigateToPose = ats_navigation_interfaces::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ServerGoalHandle<NavigateToPose>;
using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;
using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
using GimbalYawStatus = ats_navigation_interfaces::msg::GimbalYawStatus;
using PlanningMapStatus = ats_navigation_interfaces::msg::PlanningMapStatus;
using PlannerGoal = ats_navigation_interfaces::msg::PlannerGoal;
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
using YawAuthorityRequest = ats_navigation_interfaces::msg::YawAuthorityRequest;

const char *plannerFailureMessage(std::uint8_t failure_reason) {
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

bool sameStamp(const builtin_interfaces::msg::Time &left,
               const builtin_interfaces::msg::Time &right) {
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

bool finitePose(const geometry_msgs::msg::Pose &pose) {
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

std::chrono::steady_clock::duration secondsToDuration(double seconds) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, seconds)));
}

} // namespace

class AtsGoalManagerNode : public rclcpp::Node {
public:
  AtsGoalManagerNode()
      : Node("ats_goal_manager"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
        tf_listener_(
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    loadParameters();

    planner_goal_pub_ =
        create_publisher<PlannerGoal>(planner_goal_topic_, rclcpp::QoS(10));
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        reference_path_topic_, rclcpp::QoS(1));
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
    tick_timer_ = create_wall_timer(
        std::chrono::duration<double>(emergency_stop_heartbeat_period_sec_),
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

private:
  struct ActiveGoal {
    std::uint64_t id{0};
    geometry_msgs::msg::PoseStamped target;
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::duration timeout{};
    std::shared_ptr<GoalHandleNavigateToPose> action_handle;
    bool cancel_requested{false};
    std::uint64_t localization_epoch{0};
    std::optional<std::chrono::steady_clock::time_point> waiting_since;
    bool recovering{false};
  };

  void loadParameters() {
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
    require_map_status_ = declare_parameter<bool>("require_map_status", true);
    require_gimbal_status_ = declare_parameter<bool>("require_gimbal_status", true);
    gimbal_status_timeout_sec_ = std::max(
        0.1, declare_parameter<double>("gimbal_status_timeout_sec", 0.5));
    emergency_stop_heartbeat_period_sec_ = std::max(
        0.02,
        declare_parameter<double>("emergency_stop_heartbeat_period_sec", 0.1));
    map_wait_timeout_sec_ =
        std::max(0.0, declare_parameter<double>("map_wait_timeout_sec", 5.0));
    default_goal_timeout_sec_ = std::max(
        0.1, declare_parameter<double>("default_goal_timeout_sec", 120.0));
    goal_position_tolerance_ = std::max(
        0.0, declare_parameter<double>("goal_position_tolerance", 0.08));
    goal_yaw_tolerance_ =
        std::max(0.0, declare_parameter<double>("goal_yaw_tolerance", 0.15));
    terminal_linear_velocity_tolerance_ = std::max(
        0.0, declare_parameter<double>("terminal_linear_velocity_tolerance", 0.05));
    terminal_angular_velocity_tolerance_ = std::max(
        0.0, declare_parameter<double>("terminal_angular_velocity_tolerance", 0.10));
    terminal_dwell_sec_ = std::max(
        0.0, declare_parameter<double>("terminal_dwell_sec", 0.30));
  }

  rclcpp_action::GoalResponse
  onActionGoal(const rclcpp_action::GoalUUID &,
               std::shared_ptr<const NavigateToPose::Goal> goal) {
    if (!goal || !finitePose(goal->goal_pose.pose)) {
      RCLCPP_WARN(
          get_logger(),
          "Rejected action goal with non-finite pose or zero quaternion.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  onActionCancel(const std::shared_ptr<GoalHandleNavigateToPose> handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_ && active_goal_->action_handle == handle) {
        active_goal_->cancel_requested = true;
        fail_stop_ = true;
      }
    }
    // cancel 回调立即续发急停；最终 result 由 tick 统一收敛，避免与 planner
    // 回调竞态提交 reference。
    publishEmergencyStop(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  onActionAccepted(const std::shared_ptr<GoalHandleNavigateToPose> handle) {
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
                                      : default_goal_timeout_sec_);
  }

  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    geometry_msgs::msg::PoseStamped target;
    std::string reason;
    if (!normalizePose(*message, goal_frame_, target, reason)) {
      RCLCPP_ERROR(get_logger(), "Rejected /goal_pose: %s", reason.c_str());
      publishEmergencyStop(true);
      return;
    }
    startGoal(target, nullptr, default_goal_timeout_sec_);
  }

  void startGoal(const geometry_msgs::msg::PoseStamped &target,
                 const std::shared_ptr<GoalHandleNavigateToPose> &handle,
                 double timeout_sec) {
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
                                std::chrono::steady_clock::now(),
                                secondsToDuration(timeout_sec),
                                handle,
                                false,
                                0,
                                std::nullopt,
                                false};
      localization_epoch = localization_epoch_.value_or(0);
      map_publication_sequence = map_status_publication_sequence_;
      active_goal_->localization_epoch = localization_epoch;
      if (lifecycle_.state() == GoalLifecycleState::kWaitingForMap) {
        active_goal_->waiting_since = std::chrono::steady_clock::now();
      }
      fail_stop_ = true;
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
        suspendActiveGoal(id);
      }
    }
  }

  bool publishPlannerGoal(std::uint64_t id, std::uint64_t localization_epoch,
                          const geometry_msgs::msg::PoseStamped &canonical_target) {
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
      request.map_publication_sequence = map_status_publication_sequence_;
    }
    request.goal_pose = target;
    planner_goal_pub_->publish(request);
    return true;
  }

  void suspendActiveGoal(
      std::uint64_t id,
      std::optional<std::chrono::steady_clock::time_point> waiting_since = std::nullopt) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_goal_ || active_goal_->id != id) {
        return;
      }
      const bool already_waiting =
          lifecycle_.state() == GoalLifecycleState::kWaitingForMap;
      const auto previous_waiting_since = active_goal_->waiting_since;
      candidate_reference_.reset();
      planner_ready_status_.reset();
      active_execution_command_.reset();
      pending_yaw_authority_request_.reset();
      terminal_converged_since_.reset();
      fail_stop_ = true;
      lifecycle_.start(id, false);
      active_goal_->waiting_since = waiting_since.value_or(
          already_waiting && previous_waiting_since
              ? *previous_waiting_since
              : std::chrono::steady_clock::now());
    }
    publishEmergencyStop(true);
    publishExecutionStop(id, 0, PlannerStatus::FAILURE_NONE, 0, 0);
  }

  void onPlannerStatus(const PlannerStatus::SharedPtr message) {
    if (message->state == PlannerStatus::STATE_FAILED) {
      bool matches = false;
      bool transient_failure = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        matches =
            active_goal_ && active_goal_->id == message->goal_id &&
            active_goal_->localization_epoch == message->localization_epoch;
        transient_failure =
            matches &&
            (message->failure_reason == PlannerStatus::FAILURE_MAP_UNREADY ||
             message->failure_reason == PlannerStatus::FAILURE_RUNTIME_UNSAFE ||
             message->failure_reason == PlannerStatus::FAILURE_SNAPSHOT_CHANGED ||
             message->failure_reason == PlannerStatus::FAILURE_START_TF ||
             message->failure_reason == PlannerStatus::FAILURE_GOAL_TF ||
             message->failure_reason == PlannerStatus::FAILURE_REFERENCE_TF ||
             (active_goal_->recovering &&
              message->failure_reason ==
                PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED));
        if (transient_failure) {
          active_goal_->recovering = true;
          map_ready_signal_ = false;
          map_status_ready_ = false;
        }
      }
      if (matches) {
        if (transient_failure) {
          // planning grid 与 ready/status 是独立 topic，跨 topic 不具备原子顺序。
          // 恢复期 snapshot 尚未安装或仍是 blocked grid 时等待下一次 map status。
          suspendActiveGoal(message->goal_id);
        } else {
          finishActive(NavigateToPose::Result::RESULT_PLANNING_FAILED,
                       plannerFailureMessage(message->failure_reason),
                       GoalLifecycleState::kFailed);
        }
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
          active_goal_->localization_epoch == message->localization_epoch) {
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

  void onCandidateReference(const nav_msgs::msg::Path::SharedPtr message) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_reference_ = *message;
    }
    tryCommitReference();
  }

  void onMapReady(const std_msgs::msg::Bool::SharedPtr message) {
    bool fail_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_ready_signal_ = message->data;
      last_map_ready_signal_ = std::chrono::steady_clock::now();
      fail_active = !require_map_status_ && active_goal_ && !message->data;
    }
    if (fail_active) {
      finishActive(NavigateToPose::Result::RESULT_MAP_UNREADY,
                   "ROGMap adapter reported not-ready",
                   GoalLifecycleState::kFailed);
    }
  }

  void onMapStatus(const PlanningMapStatus::SharedPtr message) {
    std::optional<std::uint64_t> suspend_goal_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_status_ready_ = message->ready;
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
      if (active_goal_ && message->ready && current_epoch &&
          lifecycle_.state() == GoalLifecycleState::kWaitingForMap) {
        active_goal_->waiting_since.reset();
      }
    }
    if (suspend_goal_id) {
      // map status 与 grid/TF 更新跨 topic，不具备原子顺序。先急停并进入有界等待；
      // 恢复后重规划，持续超时才由 tick 返回 MAP_UNREADY。
      suspendActiveGoal(*suspend_goal_id);
    }
  }

  void onLocalizationStatus(const LocalizationStatus::SharedPtr message) {
    if (!require_localization_status_) {
      return;
    }
    bool stop_active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool previously_healthy =
          localization_status_ == LocalizationStatus::STATE_TRACKING;
      const bool epoch_changed =
          localization_epoch_ && *localization_epoch_ != message->epoch;
      last_localization_status_signal_ = std::chrono::steady_clock::now();
      localization_status_ = message->state;
      localization_epoch_ = message->epoch;
      const bool healthy = message->state == LocalizationStatus::STATE_TRACKING;
      if (active_goal_ && (!healthy || epoch_changed || !previously_healthy)) {
        candidate_reference_.reset();
        planner_ready_status_.reset();
        active_execution_command_.reset();
        fail_stop_ = true;
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

  void onGimbalYawStatus(const GimbalYawStatus::SharedPtr message) {
    bool stop_active = false;
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
        stop_active = true;
      }
    }
    if (stop_active) {
      publishEmergencyStop(true);
    }
    tryCommitReference();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
    geometry_msgs::msg::PoseStamped input;
    input.header = message->header;
    input.pose = message->pose.pose;
    geometry_msgs::msg::PoseStamped normalized;
    std::string reason;
    if (!normalizePose(input, goal_frame_, normalized, reason, false)) {
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
        suspendActiveGoal(*suspend_goal_id);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = normalized;
    current_linear_velocity_ = std::hypot(
      message->twist.twist.linear.x, message->twist.twist.linear.y);
    current_angular_velocity_ = std::abs(message->twist.twist.angular.z);
    has_current_velocity_ = std::isfinite(current_linear_velocity_) &&
      std::isfinite(current_angular_velocity_);
    has_current_pose_ = true;
    odom_tf_healthy_ = true;
  }

  void tryCommitReference() {
    std::optional<std::uint64_t> stale_candidate_goal;
    std::uint64_t candidate_publication_sequence = 0;
    std::uint64_t current_publication_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Candidate/reference acceptance is intentionally bound to the current
      // map publication.  If a long planning callback returns an older local
      // snapshot, safely discard it and re-enter map waiting so the existing
      // lifecycle path dispatches a fresh request.  Returning silently here
      // leaves the action stuck in kPlanning until its deadline.
      if (active_goal_ && candidate_reference_ && planner_ready_status_ &&
          planner_ready_status_->goal_id == active_goal_->id &&
          planner_ready_status_->localization_epoch == active_goal_->localization_epoch &&
          planner_ready_status_->map_publication_sequence !=
            map_status_publication_sequence_) {
        stale_candidate_goal = active_goal_->id;
        candidate_publication_sequence =
          planner_ready_status_->map_publication_sequence;
        current_publication_sequence = map_status_publication_sequence_;
      }
    }
    if (stale_candidate_goal) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting candidate for goal %llu: map publication sequence %llu is stale; current=%llu. "
        "Stopping and dispatching a fresh planner request.",
        static_cast<unsigned long long>(*stale_candidate_goal),
        static_cast<unsigned long long>(candidate_publication_sequence),
        static_cast<unsigned long long>(current_publication_sequence));
      suspendActiveGoal(*stale_candidate_goal);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_ || active_goal_->cancel_requested ||
        !candidate_reference_ || !planner_ready_status_ || !mapReadyLocked() ||
        !localizationHealthyLocked() ||
        planner_ready_status_->goal_id != active_goal_->id ||
        planner_ready_status_->localization_epoch !=
            active_goal_->localization_epoch ||
        (localization_epoch_ &&
         planner_ready_status_->localization_epoch != *localization_epoch_) ||
        planner_ready_status_->map_publication_sequence !=
            map_status_publication_sequence_ ||
        candidate_reference_->poses.size() < 2 ||
        !sameStamp(candidate_reference_->header.stamp,
                   planner_ready_status_->reference_stamp)) {
      return;
    }
    const std::uint8_t yaw_authority = planner_ready_status_->yaw_authority;
    const bool requires_gimbal_lock = planner_ready_status_->requires_gimbal_lock;
    if (yaw_authority != PlannerStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED &&
        yaw_authority != PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW) {
      fail_stop_ = true;
      return;
    }
    // Every reference, including GIMBAL_COMPENSATED, is bound to a fresh
    // request/ack.  A default mode status with request_sequence=0 is not an
    // authorization for this goal/snapshot.
    requestYawAuthorityLocked(*planner_ready_status_);
    if (!pending_yaw_authority_request_ ||
        !gimbalStatusSatisfiesLocked(yaw_authority, requires_gimbal_lock, 0)) {
      fail_stop_ = true;
      return;
    }
    if (!lifecycle_.referenceReady(active_goal_->id, true)) {
      return;
    }

    // A single execution command is the only MPC authorization.  It packages
    // the final retimed reference and all version fields in one DDS sample;
    // the legacy Path/Bool topics below are diagnostics only.
    nav_msgs::msg::Path committed = *candidate_reference_;
    rebasePathTimestamps(committed, now());
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
    command.gimbal_request_sequence = pending_yaw_authority_request_->request_sequence;
    command.gimbal_feedback_sequence = gimbal_status_ ? gimbal_status_->sequence : 0;
    command.reference = committed;
    fail_stop_ = false;
    active_goal_->recovering = false;
    active_execution_command_ = command;
    publishExecutionCommand(command);
    publishEmergencyStop(false);
    reference_path_pub_->publish(committed);
    candidate_reference_.reset();
    planner_ready_status_.reset();
    pending_yaw_authority_request_.reset();
  }

  void onTick() {
    std::optional<ActiveGoal> snapshot;
    bool dispatch = false;
    bool cancel = false;
    bool timeout = false;
    bool map_stale = false;
    bool map_wait_timeout = false;
    bool localization_wait_timeout = false;
    bool localization_suspended = false;
    bool reached = false;
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
        }
        const auto elapsed =
            std::chrono::steady_clock::now() - active_goal_->started;
        cancel = active_goal_->cancel_requested ||
                 (active_goal_->action_handle &&
                  active_goal_->action_handle->is_canceling());
        timeout = elapsed >= active_goal_->timeout;
        const bool map_ready = mapReadyLocked();
        const bool localization_ready = localizationHealthyLocked();
        if (!localization_ready &&
            lifecycle_.state() != GoalLifecycleState::kWaitingForMap) {
          candidate_reference_.reset();
          planner_ready_status_.reset();
          fail_stop_ = true;
          lifecycle_.start(active_goal_->id, false);
          active_goal_->waiting_since = std::chrono::steady_clock::now();
          map_ready_signal_ = false;
          map_status_ready_ = false;
          localization_suspended = true;
        }
        if (lifecycle_.mapReady(active_goal_->id) && map_ready &&
            localization_ready) {
          dispatch = lifecycle_.mapBecameReady(active_goal_->id);
          active_goal_->waiting_since.reset();
          dispatch_epoch = active_goal_->localization_epoch;
        }
        map_stale = lifecycle_.state() != GoalLifecycleState::kWaitingForMap &&
                    !map_ready;
        const bool wait_expired =
            lifecycle_.state() == GoalLifecycleState::kWaitingForMap &&
            active_goal_->waiting_since &&
            std::chrono::steady_clock::now() - *active_goal_->waiting_since >=
                secondsToDuration(map_wait_timeout_sec_);
        localization_wait_timeout = wait_expired && !localization_ready;
        map_wait_timeout = wait_expired && localization_ready && !map_ready;
        if (lifecycle_.state() == GoalLifecycleState::kTracking &&
            has_current_pose_) {
          distance = std::hypot(active_goal_->target.pose.position.x -
                                    current_pose_.pose.position.x,
                                active_goal_->target.pose.position.y -
                                    current_pose_.pose.position.y);
          const double yaw_delta =
              tf2::getYaw(active_goal_->target.pose.orientation) -
              tf2::getYaw(current_pose_.pose.orientation);
          const double yaw_error =
              std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
          const bool pose_converged = distance <= goal_position_tolerance_ &&
            yaw_error <= goal_yaw_tolerance_;
          const bool velocity_converged = has_current_velocity_ &&
            current_linear_velocity_ <= terminal_linear_velocity_tolerance_ &&
            current_angular_velocity_ <= terminal_angular_velocity_tolerance_;
          if (pose_converged && velocity_converged) {
            if (!terminal_converged_since_) {
              terminal_converged_since_ = std::chrono::steady_clock::now();
            }
            reached = std::chrono::steady_clock::now() - *terminal_converged_since_ >=
              secondsToDuration(terminal_dwell_sec_);
          } else {
            terminal_converged_since_.reset();
          }
        }
        publishFeedbackLocked(elapsed, distance);
      }
    }

    if (dispatch && snapshot) {
      if (!publishPlannerGoal(snapshot->id, dispatch_epoch, snapshot->target)) {
        suspendActiveGoal(snapshot->id, snapshot->waiting_since);
      }
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
      if (!stop && active_execution_command_) {
        execution_heartbeat = active_execution_command_;
      } else if (active_goal_) {
        stop_goal_id = active_goal_->id;
        stop_localization_epoch = active_goal_->localization_epoch;
        stop_map_publication_sequence = map_status_publication_sequence_;
      }
    }
    if (execution_heartbeat) {
      publishExecutionCommand(*execution_heartbeat);
    } else {
      publishExecutionStop(
        stop_goal_id, stop_localization_epoch, PlannerStatus::FAILURE_NONE,
        stop_map_generation, stop_map_publication_sequence);
    }
    publishEmergencyStop(stop);
  }

  void finishActive(std::uint8_t result_code, const std::string &message,
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
    }
    publishEmergencyStop(true);
    publishExecutionStop(
      finished->id, finished->localization_epoch, PlannerStatus::FAILURE_NONE,
      0, 0);
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

  bool mapReadyLocked() const {
    if (require_map_status_) {
      const bool lease_ok =
          last_map_status_signal_ &&
          std::chrono::steady_clock::now() - *last_map_status_signal_ <=
              secondsToDuration(map_ready_timeout_sec_);
      const std::uint64_t expected_epoch =
          require_localization_status_ ? localization_epoch_.value_or(0) : 0;
      return lease_ok && map_status_ready_ &&
             map_status_localization_epoch_ == expected_epoch;
    }
    return map_ready_signal_ && last_map_ready_signal_ &&
           std::chrono::steady_clock::now() - *last_map_ready_signal_ <=
               secondsToDuration(map_ready_timeout_sec_);
  }

  bool localizationHealthyLocked() const {
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
           localization_status_ == LocalizationStatus::STATE_TRACKING &&
           odom_tf_healthy_;
  }

  bool gimbalStatusSatisfiesLocked(
    std::uint8_t yaw_authority, bool requires_gimbal_lock,
    std::uint64_t minimum_feedback_sequence,
    std::uint64_t required_request_sequence = 0) const {
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
      (requires_gimbal_lock && !gimbal_status_->locked) ||
      (minimum_feedback_sequence > 0 &&
       gimbal_status_->sequence < minimum_feedback_sequence) ||
      (required_request_sequence > 0 &&
       gimbal_status_->request_sequence != required_request_sequence)) {
      return false;
    }
    return !pending_yaw_authority_request_ ||
      gimbal_status_->request_sequence == pending_yaw_authority_request_->request_sequence;
  }

  void requestYawAuthorityLocked(const PlannerStatus & status) {
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

  bool normalizePose(const geometry_msgs::msg::PoseStamped &input,
                     const std::string &target_frame,
                     geometry_msgs::msg::PoseStamped &output,
                     std::string &reason,
                     bool use_latest_transform = true) const {
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
      tf2::doTransform(source, output, transform);
      output.header.frame_id = target_frame;
      return finitePose(output.pose);
    } catch (const tf2::TransformException &exception) {
      reason = "TF transform to " + target_frame + " failed: " + exception.what();
      return false;
    }
  }

  void publishFeedbackLocked(const std::chrono::steady_clock::duration &elapsed,
                             double distance) {
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

  static std::uint8_t actionState(GoalLifecycleState state) {
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

  static std::string stateName(GoalLifecycleState state) {
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

  void rebasePathTimestamps(nav_msgs::msg::Path &path,
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

  void publishEmergencyStop(bool stop) {
    std_msgs::msg::Bool message;
    message.data = stop;
    emergency_stop_pub_->publish(message);
  }

  void publishExecutionCommand(ExecutionCommand command) {
    command.command_sequence = ++next_execution_command_sequence_;
    command.header.stamp = now();
    execution_command_pub_->publish(command);
  }

  void publishExecutionStop(
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
  std::string localization_status_topic_;
  std::string odom_topic_;
  std::string action_name_;
  std::string goal_frame_;
  std::string planning_frame_;
  double map_ready_timeout_sec_{3.0};
  double localization_status_timeout_sec_{1.0};
  double emergency_stop_heartbeat_period_sec_{0.1};
  double map_wait_timeout_sec_{5.0};
  double default_goal_timeout_sec_{120.0};
  double goal_position_tolerance_{0.08};
  double goal_yaw_tolerance_{0.15};
  double terminal_linear_velocity_tolerance_{0.05};
  double terminal_angular_velocity_tolerance_{0.10};
  double terminal_dwell_sec_{0.30};
  double gimbal_status_timeout_sec_{0.5};
  bool require_localization_status_{false};
  bool require_map_status_{true};
  bool require_gimbal_status_{true};

  std::mutex mutex_;
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
  bool fail_stop_{true};
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;
  std::optional<std::chrono::steady_clock::time_point> last_map_status_signal_;
  std::uint64_t map_status_localization_epoch_{0};
  std::uint64_t map_status_generation_{0};
  std::uint64_t map_status_publication_sequence_{0};
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
  bool has_current_pose_{false};
  double current_linear_velocity_{std::numeric_limits<double>::infinity()};
  double current_angular_velocity_{std::numeric_limits<double>::infinity()};
  bool has_current_velocity_{false};
  bool odom_tf_healthy_{false};

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
  rclcpp::Subscription<LocalizationStatus>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<GimbalYawStatus>::SharedPtr gimbal_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace ats_goal_manager

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_goal_manager::AtsGoalManagerNode>());
  rclcpp::shutdown();
  return 0;
}
