// Copyright 2026

#include "trajectory_optimizer/control/trajectory_speed_governor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace trajectory_optimizer
{

namespace
{

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA colorFromSpeedRatio(double ratio)
{
  const double clamped = std::max(0.0, std::min(1.0, ratio));
  return makeColor(
    static_cast<float>(0.85 - 0.35 * clamped),
    static_cast<float>(0.55 + 0.25 * clamped),
    static_cast<float>(0.25 + 0.15 * clamped),
    0.88f);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

double shortestAngularDistance(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

template<typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> & publisher)
{
  return publisher &&
         (publisher->get_subscription_count() > 0 ||
         publisher->get_intra_process_subscription_count() > 0);
}

}  // namespace

TrajectorySpeedGovernor::TrajectorySpeedGovernor(const rclcpp::NodeOptions & options)
: Node("trajectory_speed_governor", options)
{
  declare_parameter<std::string>("profile_topic", "trajectory_profile");
  declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_controller");
  declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel_controller_governed");
  declare_parameter<std::string>("marker_topic", marker_topic_);
  declare_parameter<bool>("enabled", enabled_);
  declare_parameter<double>("min_speed_scale", min_speed_scale_);
  declare_parameter<double>("curvature_brake_gain", curvature_brake_gain_);
  declare_parameter<int>("curvature_window_points", curvature_window_points_);
  declare_parameter<double>("curvature_peak_weight", curvature_peak_weight_);
  declare_parameter<bool>("use_profile_target_speed_limit", use_profile_target_speed_limit_);
  declare_parameter<double>(
    "profile_target_speed_limit_margin", profile_target_speed_limit_margin_);
  declare_parameter<double>("speed_scale_filter_gain", speed_scale_filter_gain_);
  declare_parameter<double>("speed_scale_rise_rate", speed_scale_rise_rate_);
  declare_parameter<double>("speed_scale_fall_rate", speed_scale_fall_rate_);
  declare_parameter<bool>("heading_guard_enabled", heading_guard_enabled_);
  declare_parameter<std::string>("heading_guard_path_topic", heading_guard_path_topic_);
  declare_parameter<std::string>(
    "heading_guard_traversability_grid_topic", heading_guard_traversability_grid_topic_);
  declare_parameter<std::string>("heading_guard_base_frame", heading_guard_base_frame_);
  declare_parameter<int>(
    "heading_guard_obstacle_value_threshold", heading_guard_obstacle_value_threshold_);
  declare_parameter<int>(
    "heading_guard_lethal_value_threshold", heading_guard_lethal_value_threshold_);
  declare_parameter<bool>("heading_guard_unknown_is_obstacle", heading_guard_unknown_is_obstacle_);
  declare_parameter<double>("heading_guard_enter_clearance_m", heading_guard_enter_clearance_m_);
  declare_parameter<double>("heading_guard_exit_clearance_m", heading_guard_exit_clearance_m_);
  declare_parameter<double>("heading_guard_footprint_length_m", heading_guard_footprint_length_m_);
  declare_parameter<double>("heading_guard_footprint_width_m", heading_guard_footprint_width_m_);
  declare_parameter<double>("heading_guard_lateral_speed_limit", heading_guard_lateral_speed_limit_);
  declare_parameter<double>("heading_guard_stop_error_rad", heading_guard_stop_error_rad_);
  declare_parameter<double>("heading_guard_yaw_kp", heading_guard_yaw_kp_);
  declare_parameter<double>("heading_guard_max_angular_speed", heading_guard_max_angular_speed_);

  get_parameter("profile_topic", profile_topic_);
  get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  get_parameter("marker_topic", marker_topic_);
  get_parameter("enabled", enabled_);
  get_parameter("min_speed_scale", min_speed_scale_);
  get_parameter("curvature_brake_gain", curvature_brake_gain_);
  get_parameter("curvature_window_points", curvature_window_points_);
  get_parameter("curvature_peak_weight", curvature_peak_weight_);
  get_parameter("use_profile_target_speed_limit", use_profile_target_speed_limit_);
  get_parameter("profile_target_speed_limit_margin", profile_target_speed_limit_margin_);
  get_parameter("speed_scale_filter_gain", speed_scale_filter_gain_);
  get_parameter("speed_scale_rise_rate", speed_scale_rise_rate_);
  get_parameter("speed_scale_fall_rate", speed_scale_fall_rate_);
  get_parameter("heading_guard_enabled", heading_guard_enabled_);
  get_parameter("heading_guard_path_topic", heading_guard_path_topic_);
  get_parameter(
    "heading_guard_traversability_grid_topic", heading_guard_traversability_grid_topic_);
  get_parameter("heading_guard_base_frame", heading_guard_base_frame_);
  get_parameter(
    "heading_guard_obstacle_value_threshold", heading_guard_obstacle_value_threshold_);
  get_parameter("heading_guard_lethal_value_threshold", heading_guard_lethal_value_threshold_);
  get_parameter("heading_guard_unknown_is_obstacle", heading_guard_unknown_is_obstacle_);
  get_parameter("heading_guard_enter_clearance_m", heading_guard_enter_clearance_m_);
  get_parameter("heading_guard_exit_clearance_m", heading_guard_exit_clearance_m_);
  get_parameter("heading_guard_footprint_length_m", heading_guard_footprint_length_m_);
  get_parameter("heading_guard_footprint_width_m", heading_guard_footprint_width_m_);
  get_parameter("heading_guard_lateral_speed_limit", heading_guard_lateral_speed_limit_);
  get_parameter("heading_guard_stop_error_rad", heading_guard_stop_error_rad_);
  get_parameter("heading_guard_yaw_kp", heading_guard_yaw_kp_);
  get_parameter("heading_guard_max_angular_speed", heading_guard_max_angular_speed_);

  heading_guard_enter_clearance_m_ = std::max(0.0, heading_guard_enter_clearance_m_);
  heading_guard_exit_clearance_m_ = std::max(
    heading_guard_enter_clearance_m_, heading_guard_exit_clearance_m_);
  heading_guard_footprint_length_m_ = std::max(0.01, heading_guard_footprint_length_m_);
  heading_guard_footprint_width_m_ = std::max(0.01, heading_guard_footprint_width_m_);
  heading_guard_lateral_speed_limit_ = std::max(0.0, heading_guard_lateral_speed_limit_);
  heading_guard_stop_error_rad_ = std::max(0.01, heading_guard_stop_error_rad_);
  heading_guard_yaw_kp_ = std::max(0.0, heading_guard_yaw_kp_);
  heading_guard_max_angular_speed_ = std::max(0.0, heading_guard_max_angular_speed_);

  profile_sub_ = create_subscription<sp_msgs::msg::TrajectoryProfileMsg>(
    profile_topic_, 10,
    std::bind(&TrajectorySpeedGovernor::profileCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&TrajectorySpeedGovernor::cmdVelCallback, this, std::placeholders::_1));
  governed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  if (heading_guard_enabled_) {
    heading_guard_esdf_ = std::make_shared<RcTraversabilityEsdfProvider>();
    heading_guard_esdf_->configureRollingWindow(true, 0.0, 0.0);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    heading_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      heading_guard_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&TrajectorySpeedGovernor::headingPathCallback, this, std::placeholders::_1));
    heading_traversability_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      heading_guard_traversability_grid_topic_, 10,
      std::bind(
        &TrajectorySpeedGovernor::headingTraversabilityCallback, this, std::placeholders::_1));
  }
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&TrajectorySpeedGovernor::publishGovernedCmd, this));
}

void TrajectorySpeedGovernor::profileCallback(
  const sp_msgs::msg::TrajectoryProfileMsg::SharedPtr msg)
{
  if (msg->points.empty()) {
    current_speed_scale_ = 1.0;
    profile_speed_cap_ = std::numeric_limits<double>::infinity();
    return;
  }

  const std::size_t window_points = std::max(
    std::size_t(1),
    std::min(
      msg->points.size(),
      static_cast<std::size_t>(std::max(1, curvature_window_points_))));

  double window_max_abs_curvature = 0.0;
  double window_avg_abs_curvature = 0.0;
  double profile_speed_scale = 1.0;
  double window_min_target_speed = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < window_points; ++i) {
    const double abs_curvature = std::abs(msg->points[i].curvature);
    window_max_abs_curvature = std::max(window_max_abs_curvature, abs_curvature);
    window_avg_abs_curvature += abs_curvature;
    const double speed_limit = msg->points[i].speed_limit;
    if (speed_limit > 1e-3) {
      const double point_speed_scale =
        std::max(0.0, std::min(1.0, msg->points[i].speed / speed_limit));
      profile_speed_scale = std::min(profile_speed_scale, point_speed_scale);
    }
    if (msg->points[i].speed_limit > 1e-3) {
      window_min_target_speed = std::min(window_min_target_speed, msg->points[i].speed_limit);
    }
  }
  window_avg_abs_curvature /= static_cast<double>(window_points);
  profile_speed_cap_ = window_min_target_speed;

  const double peak_weight = std::max(0.0, std::min(1.0, curvature_peak_weight_));
  const double effective_curvature =
    peak_weight * window_max_abs_curvature +
    (1.0 - peak_weight) * window_avg_abs_curvature;
  const double curvature_scale =
    1.0 / (1.0 + curvature_brake_gain_ * effective_curvature);
  const double target_speed_scale = std::max(
    min_speed_scale_, std::min(1.0, std::min(curvature_scale, profile_speed_scale)));
  const double filter_gain = std::max(0.0, std::min(1.0, speed_scale_filter_gain_));
  current_speed_scale_ += (target_speed_scale - current_speed_scale_) * filter_gain;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Speed governor scale: target=%.3f filtered=%.3f applied=%.3f profile=%.3f curvature=%.3f v_cap=%.3f kappa_avg=%.3f kappa_max=%.3f window_points=%zu",
    target_speed_scale,
    current_speed_scale_,
    applied_speed_scale_,
    profile_speed_scale,
    curvature_scale,
    std::isfinite(profile_speed_cap_) ? profile_speed_cap_ : -1.0,
    window_avg_abs_curvature,
    window_max_abs_curvature,
    window_points);

  publishProfileMarkers(*msg);
}

void TrajectorySpeedGovernor::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  has_cmd_vel_ = true;
}

void TrajectorySpeedGovernor::headingPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.size() < 2 || msg->header.frame_id.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(heading_guard_mutex_);
  latest_heading_path_ = *msg;
}

void TrajectorySpeedGovernor::headingTraversabilityCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->header.frame_id.empty() || !heading_guard_esdf_) {
    return;
  }

  heading_guard_esdf_->updateGrid(
    *msg,
    heading_guard_obstacle_value_threshold_,
    heading_guard_unknown_is_obstacle_,
    heading_guard_lethal_value_threshold_);
  std::lock_guard<std::mutex> lock(heading_guard_mutex_);
  heading_guard_grid_frame_ = msg->header.frame_id;
}

void TrajectorySpeedGovernor::publishGovernedCmd()
{
  if (!has_cmd_vel_) {
    return;
  }

  geometry_msgs::msg::Twist governed = latest_cmd_vel_;
  if (!enabled_) {
    governed_cmd_pub_->publish(governed);
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = 0.0;
  if (last_publish_steady_time_.time_since_epoch().count() != 0) {
    dt = std::chrono::duration<double>(now - last_publish_steady_time_).count();
  }
  last_publish_steady_time_ = now;

  const double rise_step = std::max(0.0, speed_scale_rise_rate_) * dt;
  const double fall_step = std::max(0.0, speed_scale_fall_rate_) * dt;
  applied_speed_scale_ += std::clamp(current_speed_scale_ - applied_speed_scale_, -fall_step, rise_step);
  applied_speed_scale_ = std::max(min_speed_scale_, std::min(1.0, applied_speed_scale_));

  governed.linear.x *= applied_speed_scale_;
  governed.linear.y *= applied_speed_scale_;
  governed.angular.z *= std::sqrt(applied_speed_scale_);

  if (use_profile_target_speed_limit_ && std::isfinite(profile_speed_cap_)) {
    // profile 已经综合了曲率 / 障碍 / 坡度限速。
    // 这里再做一次“近端绝对速度硬上限”，避免这些限制只体现在 profile，
    // 却在 controller 输出波动时被实际 cmd_vel 放大掉。
    const double margin = std::max(0.0, profile_target_speed_limit_margin_);
    const double allowed_linear_speed = std::max(0.0, profile_speed_cap_ + margin);
    const double linear_norm = std::hypot(governed.linear.x, governed.linear.y);
    if (linear_norm > allowed_linear_speed && linear_norm > 1e-6) {
      const double scale = allowed_linear_speed / linear_norm;
      governed.linear.x *= scale;
      governed.linear.y *= scale;
    }
  }

  applyNarrowCorridorHeadingGuard(governed);

  governed_cmd_pub_->publish(governed);
}

void TrajectorySpeedGovernor::applyNarrowCorridorHeadingGuard(
  geometry_msgs::msg::Twist & governed)
{
  if (!heading_guard_enabled_ || !heading_guard_esdf_ || !tf_buffer_) {
    return;
  }

  nav_msgs::msg::Path path;
  std::string grid_frame;
  bool previously_narrow = false;
  {
    std::lock_guard<std::mutex> lock(heading_guard_mutex_);
    path = latest_heading_path_;
    grid_frame = heading_guard_grid_frame_;
    previously_narrow = narrow_corridor_active_;
  }
  if (path.poses.size() < 2 || path.header.frame_id.empty() || grid_frame.empty()) {
    return;
  }

  try {
    const auto base_in_path = tf_buffer_->lookupTransform(
      path.header.frame_id, heading_guard_base_frame_, tf2::TimePointZero);
    const auto base_in_grid = tf_buffer_->lookupTransform(
      grid_frame, heading_guard_base_frame_, tf2::TimePointZero);

    const double half_length = heading_guard_footprint_length_m_ * 0.5;
    const double half_width = heading_guard_footprint_width_m_ * 0.5;
    const std::vector<Eigen::Vector2d> footprint_samples {
      {0.0, 0.0},
      {half_length, half_width},
      {half_length, -half_width},
      {-half_length, half_width},
      {-half_length, -half_width},
    };
    const Eigen::Vector2d base_position(
      base_in_grid.transform.translation.x, base_in_grid.transform.translation.y);
    const double base_yaw_in_grid = yawFromQuaternion(base_in_grid.transform.rotation);
    const double footprint_clearance = heading_guard_esdf_->getFootprintClearance(
      base_position, base_yaw_in_grid, footprint_samples);

    bool narrow = previously_narrow;
    if (std::isfinite(footprint_clearance)) {
      narrow = previously_narrow ?
        footprint_clearance < heading_guard_exit_clearance_m_ :
        footprint_clearance <= heading_guard_enter_clearance_m_;
    }
    {
      std::lock_guard<std::mutex> lock(heading_guard_mutex_);
      narrow_corridor_active_ = narrow;
    }
    if (!narrow) {
      return;
    }

    const double robot_x = base_in_path.transform.translation.x;
    const double robot_y = base_in_path.transform.translation.y;
    std::size_t closest_index = 0;
    double closest_distance_squared = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      const auto & point = path.poses[i].pose.position;
      const double dx = point.x - robot_x;
      const double dy = point.y - robot_y;
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        closest_index = i;
      }
    }

    std::size_t next_index = std::min(closest_index + 1, path.poses.size() - 1);
    std::size_t previous_index = closest_index;
    if (next_index == closest_index && closest_index > 0) {
      previous_index = closest_index - 1;
    }
    const auto & start = path.poses[previous_index].pose.position;
    const auto & end = path.poses[next_index].pose.position;
    const double tangent_x = end.x - start.x;
    const double tangent_y = end.y - start.y;
    if (std::hypot(tangent_x, tangent_y) < 1e-3) {
      return;
    }

    const double desired_yaw = std::atan2(tangent_y, tangent_x);
    const double current_yaw = yawFromQuaternion(base_in_path.transform.rotation);
    const double heading_error = shortestAngularDistance(current_yaw, desired_yaw);
    const double abs_heading_error = std::abs(heading_error);
    const double alignment_scale = std::max(
      0.0, 1.0 - abs_heading_error / heading_guard_stop_error_rad_);

    // A narrow corridor permits only body-forward motion. The controller still
    // chooses its global trajectory; this layer enforces the chassis envelope.
    governed.linear.x = std::max(0.0, governed.linear.x) * alignment_scale;
    governed.linear.y = std::max(
      -heading_guard_lateral_speed_limit_,
      std::min(heading_guard_lateral_speed_limit_, governed.linear.y));
    if (abs_heading_error >= heading_guard_stop_error_rad_) {
      governed.linear.x = 0.0;
      governed.linear.y = 0.0;
    }
    const double corrected_angular_speed =
      governed.angular.z + heading_guard_yaw_kp_ * heading_error;
    governed.angular.z = std::max(
      -heading_guard_max_angular_speed_,
      std::min(heading_guard_max_angular_speed_, corrected_angular_speed));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Narrow-corridor heading guard: clearance=%.3fm yaw_error=%.3frad forward_scale=%.2f",
      footprint_clearance, heading_error, alignment_scale);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Narrow-corridor heading guard is waiting for TF: %s", ex.what());
  }
}

void TrajectorySpeedGovernor::publishProfileMarkers(
  const sp_msgs::msg::TrajectoryProfileMsg & msg)
{
  if (!marker_pub_ || msg.points.empty() || !hasSubscribers(marker_pub_)) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker line;
  line.header = msg.header;
  line.ns = "trajectory_profile";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.022;
  line.color = makeColor(0.28f, 0.72f, 0.78f, 0.86f);

  visualization_msgs::msg::Marker points;
  points.header = msg.header;
  points.ns = "trajectory_profile";
  points.id = 1;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.055;
  points.scale.y = 0.055;
  points.scale.z = 0.055;

  visualization_msgs::msg::Marker text;
  text.header = msg.header;
  text.ns = "trajectory_profile";
  text.id = 2;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.scale.z = 0.15;
  text.color = makeColor(0.92f, 0.92f, 0.88f, 0.90f);
  text.pose.position = msg.points.back().point;
  text.pose.position.z += 0.35;
  text.text =
    "kappa_max=" + std::to_string(msg.max_abs_curvature).substr(0, 4) +
    " cost=" + std::to_string(msg.total_cost).substr(0, 6);

  for (const auto & sample : msg.points) {
    line.points.push_back(sample.point);
    points.points.push_back(sample.point);
    const double denom = std::max(1e-3, sample.speed_limit);
    points.colors.push_back(colorFromSpeedRatio(sample.speed / denom));
  }

  markers.markers.push_back(line);
  markers.markers.push_back(points);
  markers.markers.push_back(text);
  marker_pub_->publish(markers);
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::TrajectorySpeedGovernor)
