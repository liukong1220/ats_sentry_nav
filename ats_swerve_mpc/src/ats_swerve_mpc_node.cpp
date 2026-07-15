// Copyright 2026

#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_swerve_mpc
{

namespace
{

Eigen::Vector3d vectorParameter(
  rclcpp::Node & node, const std::string & name, const Eigen::Vector3d & defaults)
{
  const std::vector<double> values =
    node.declare_parameter<std::vector<double>>(name, {defaults(0), defaults(1), defaults(2)});
  if (values.size() != 3) {
    RCLCPP_WARN(node.get_logger(), "Parameter '%s' must contain exactly 3 values.", name.c_str());
    return defaults;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

}  // namespace

AtsSwerveMpcNode::AtsSwerveMpcNode(const rclcpp::NodeOptions & options)
: Node("ats_swerve_mpc", options)
{
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
  trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "/minco/reference_path");
  command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_gimbal_yaw_odom");
  emergency_stop_topic_ =
    declare_parameter<std::string>("emergency_stop_topic", "/planner/emergency_stop");
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", control_rate_hz_);
  fallback_path_dt_ = declare_parameter<double>("fallback_path_dt", fallback_path_dt_);
  trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", trajectory_timeout_);
  emergency_stop_timeout_ = std::max(
    0.1, declare_parameter<double>("emergency_stop_timeout", emergency_stop_timeout_));
  emergency_stop_watchdog_.setTimeout(emergency_stop_timeout_);
  goal_position_tolerance_ =
    declare_parameter<double>("goal_position_tolerance", goal_position_tolerance_);
  goal_yaw_tolerance_ = declare_parameter<double>("goal_yaw_tolerance", goal_yaw_tolerance_);
  publish_debug_paths_ = declare_parameter<bool>("publish_debug_paths", publish_debug_paths_);
  controller_ = std::make_unique<Se2MpcController>(loadConfig());
  trajectory_tracker_.setConfig(loadTrackerConfig());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AtsSwerveMpcNode::onOdometry, this, std::placeholders::_1));
  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
    trajectory_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&AtsSwerveMpcNode::onPath, this, std::placeholders::_1));
  if (!emergency_stop_topic_.empty()) {
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsSwerveMpcNode::onEmergencyStop, this, std::placeholders::_1));
  } else {
    emergency_stop_watchdog_enabled_ = false;
    fail_stop_engaged_.store(false);
    RCLCPP_WARN(get_logger(), "Emergency-stop input is explicitly disabled.");
  }
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_, rclcpp::QoS(10));
  predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/predicted_path", rclcpp::QoS(1));
  horizon_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/reference_horizon", rclcpp::QoS(1));
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
    std::bind(&AtsSwerveMpcNode::onControlTimer, this));
  publishCommand(Control::Zero());
  RCLCPP_INFO(
    get_logger(), "ATS swerve MPC ready: odom='%s' trajectory='%s' command='%s' horizon=%d dt=%.3f",
    odom_topic_.c_str(), trajectory_topic_.c_str(), command_topic_.c_str(),
    controller_->config().horizon, controller_->config().dt);
}

Se2MpcConfig AtsSwerveMpcNode::loadConfig()
{
  Se2MpcConfig config;
  config.horizon = declare_parameter<int>("horizon", config.horizon);
  config.dt = declare_parameter<double>("dt", config.dt);
  config.state_weight = vectorParameter(*this, "state_weight", config.state_weight);
  config.control_weight = vectorParameter(*this, "control_weight", config.control_weight);
  config.control_delta_weight =
    vectorParameter(*this, "control_delta_weight", config.control_delta_weight);
  config.terminal_weight = vectorParameter(*this, "terminal_weight", config.terminal_weight);
  config.max_vx = declare_parameter<double>("max_vx", config.max_vx);
  config.max_vy = declare_parameter<double>("max_vy", config.max_vy);
  config.max_wz = declare_parameter<double>("max_wz", config.max_wz);
  config.max_ax = declare_parameter<double>("max_ax", config.max_ax);
  config.max_ay = declare_parameter<double>("max_ay", config.max_ay);
  config.max_awz = declare_parameter<double>("max_awz", config.max_awz);
  config.max_iterations = declare_parameter<int>("max_iterations", config.max_iterations);
  config.regularization = declare_parameter<double>("regularization", config.regularization);
  config.line_search_decay =
    declare_parameter<double>("line_search_decay", config.line_search_decay);
  config.min_line_search_step =
    declare_parameter<double>("min_line_search_step", config.min_line_search_step);
  config.convergence_tolerance =
    declare_parameter<double>("convergence_tolerance", config.convergence_tolerance);
  return config;
}

TrajectoryTrackerConfig AtsSwerveMpcNode::loadTrackerConfig()
{
  TrajectoryTrackerConfig config;
  config.backward_search_window =
    declare_parameter<double>("projection_backward_window", config.backward_search_window);
  config.forward_search_window =
    declare_parameter<double>("projection_forward_window", config.forward_search_window);
  config.cross_track_slowdown_start =
    declare_parameter<double>("cross_track_slowdown_start", config.cross_track_slowdown_start);
  config.cross_track_slowdown_end =
    declare_parameter<double>("cross_track_slowdown_end", config.cross_track_slowdown_end);
  config.min_progress_scale =
    declare_parameter<double>("min_reference_progress_scale", config.min_progress_scale);
  config.command_latency_compensation =
    declare_parameter<double>("command_latency_compensation", config.command_latency_compensation);
  return config;
}

void AtsSwerveMpcNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_state_(0) = message->pose.pose.position.x;
  current_state_(1) = message->pose.pose.position.y;
  current_state_(2) = tf2::getYaw(message->pose.pose.orientation);
  has_odometry_ = current_state_.allFinite();
}

void AtsSwerveMpcNode::onPath(const nav_msgs::msg::Path::SharedPtr message)
{
  if (message->poses.size() < 2) {
    RCLCPP_WARN(get_logger(), "Ignoring trajectory with fewer than two poses.");
    return;
  }
  if (
    !frame_id_.empty() && !message->header.frame_id.empty() &&
    message->header.frame_id != frame_id_) {
    RCLCPP_ERROR(
      get_logger(), "Ignoring trajectory in frame '%s'; MPC frame is '%s'.",
      message->header.frame_id.c_str(), frame_id_.c_str());
    return;
  }
  std::vector<TimedState> parsed;
  parsed.reserve(message->poses.size());
  const double header_time = rclcpp::Time(message->header.stamp).seconds();
  bool monotonic = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < message->poses.size(); ++i) {
    const auto & pose = message->poses[i];
    double pose_time = rclcpp::Time(pose.header.stamp).seconds();
    if (pose_time <= 0.0) {
      pose_time = header_time + fallback_path_dt_ * static_cast<double>(i);
    }
    monotonic = monotonic && pose_time > previous_time;
    TimedState timed;
    timed.time = pose_time;
    timed.state << pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation);
    if (!timed.state.allFinite()) {
      return;
    }
    parsed.push_back(timed);
    previous_time = pose_time;
  }
  if (!monotonic) {
    const double start_time = header_time > 0.0 ? header_time : now().seconds();
    for (std::size_t i = 0; i < parsed.size(); ++i) {
      parsed[i].time = start_time + fallback_path_dt_ * static_cast<double>(i);
    }
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool missing_stamp =
      message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0;
    if (last_stop_stamp_.nanoseconds() > 0 &&
      (missing_stamp || rclcpp::Time(message->header.stamp) <= last_stop_stamp_))
    {
      RCLCPP_WARN(get_logger(), "Ignoring a trajectory older than the latest emergency stop.");
      return;
    }
    trajectory_tracker_.setTrajectory(std::move(parsed));
    trajectory_frame_ = message->header.frame_id.empty() ? frame_id_ : message->header.frame_id;
    const double maximum_tracking_duration =
      trajectory_tracker_.duration() / std::max(1e-3, trajectory_tracker_.minimumProgressScale());
    trajectory_deadline_ =
      now() + rclcpp::Duration::from_seconds(
                maximum_tracking_duration + std::max(0.0, trajectory_timeout_));
  }
  controller_->reset();
  last_control_.setZero();
}

void AtsSwerveMpcNode::onEmergencyStop(const std_msgs::msg::Bool::SharedPtr message)
{
  const bool first_signal = !emergency_stop_signal_received_.exchange(true);
  emergency_stop_watchdog_.update(message->data);
  if (first_signal) {
    fail_stop_engaged_.store(false);
    engageFailStop();
  }
  if (message->data) {
    engageFailStop();
  } else {
    fail_stop_engaged_.store(false);
  }
}

void AtsSwerveMpcNode::onControlTimer()
{
  if (
    emergency_stop_watchdog_enabled_ &&
    (emergency_stop_watchdog_.stopRequired() || fail_stop_engaged_.load()))
  {
    engageFailStop();
    return;
  }
  State current;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_odometry_) {
      return;
    }
    current = current_state_;
  }
  State goal = State::Zero();
  TrajectoryProjection projection;
  std::vector<Se2Reference> references;
  bool trajectory_expired = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (trajectory_tracker_.empty()) {
      return;
    }
    goal = trajectory_tracker_.goal();
    projection = trajectory_tracker_.project(current.head<2>());
    references = trajectory_tracker_.buildHorizon(
      projection, controller_->config().horizon, controller_->config().dt);
    trajectory_expired = trajectory_deadline_.nanoseconds() > 0 && now() > trajectory_deadline_;
  }
  const double goal_position_error = (goal.head<2>() - current.head<2>()).norm();
  const double goal_yaw_error = std::abs(normalizeAngle(goal(2) - current(2)));
  if (
    (goal_position_error <= goal_position_tolerance_ && goal_yaw_error <= goal_yaw_tolerance_) ||
    trajectory_expired) {
    last_control_.setZero();
    publishCommand(last_control_);
    controller_->reset();
    return;
  }

  if (references.size() < static_cast<std::size_t>(controller_->config().horizon + 1)) {
    last_control_.setZero();
    publishCommand(last_control_);
    return;
  }
  const Se2MpcResult result = controller_->solve(current, references, last_control_);
  if (!result.success || result.controls.empty()) {
    last_control_.setZero();
    publishCommand(last_control_);
    controller_->reset();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "SE2 MPC solve failed.");
    return;
  }
  last_control_ = result.controls.front();
  publishCommand(last_control_);
  if (publish_debug_paths_) {
    publishPath(result.states, predicted_path_pub_);
    std::vector<State> reference_states;
    reference_states.reserve(references.size());
    for (const auto & reference : references) {
      reference_states.push_back(reference.state);
    }
    publishPath(reference_states, horizon_path_pub_);
  }
  RCLCPP_DEBUG(
    get_logger(),
    "MPC vx=%.3f vy=%.3f wz=%.3f cross_track=%.3f progress_scale=%.2f cost=%.3f "
    "solve=%.2fms",
    last_control_(0), last_control_(1), last_control_(2), projection.cross_track_error,
    trajectory_tracker_.progressScale(projection.cross_track_error), result.cost,
    result.solve_time_ms);
}

void AtsSwerveMpcNode::engageFailStop()
{
  const bool was_engaged = fail_stop_engaged_.exchange(true);
  if (!was_engaged) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_tracker_.clear();
    trajectory_frame_.clear();
    trajectory_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_stop_stamp_ = now();
    controller_->reset();
  }
  last_control_.setZero();
  publishCommand(last_control_);
}

void AtsSwerveMpcNode::publishCommand(const Control & command)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = command(0);
  message.linear.y = command(1);
  message.angular.z = command(2);
  command_pub_->publish(message);
}

void AtsSwerveMpcNode::publishPath(
  const std::vector<State> & states,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
  path.poses.reserve(states.size());
  for (const auto & state : states) {
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

double AtsSwerveMpcNode::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace ats_swerve_mpc
