// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_
#define TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace trajectory_optimizer
{

class TrajectorySpeedGovernor : public rclcpp::Node
{
public:
  explicit TrajectorySpeedGovernor(const rclcpp::NodeOptions & options);

private:
  void profileCallback(const sp_msgs::msg::TrajectoryProfileMsg::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void headingPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void headingTraversabilityCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publishGovernedCmd();
  void publishProfileMarkers(const sp_msgs::msg::TrajectoryProfileMsg & msg);
  void applyNarrowCorridorHeadingGuard(geometry_msgs::msg::Twist & governed);

  rclcpp::Subscription<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr heading_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr heading_traversability_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr governed_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string profile_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string marker_topic_{"trajectory_profile_markers"};
  bool enabled_ = true;
  double min_speed_scale_ = 0.35;
  double curvature_brake_gain_ = 0.7;
  int curvature_window_points_ = 12;
  double curvature_peak_weight_ = 0.35;
  // 任务2：不仅看曲率缩放比例，还要真正服从 profile 给出的近端绝对目标速度。
  bool use_profile_target_speed_limit_ = true;
  double profile_target_speed_limit_margin_ = 0.05;
  double speed_scale_filter_gain_ = 0.25;
  double speed_scale_rise_rate_ = 4.0;
  double speed_scale_fall_rate_ = 1.8;
  double current_speed_scale_ = 1.0;
  double applied_speed_scale_ = 1.0;
  double profile_speed_cap_ = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Twist latest_cmd_vel_;
  bool has_cmd_vel_ = false;
  std::chrono::steady_clock::time_point last_publish_steady_time_;

  // The swerve chassis remains omnidirectional in open areas. This guard only
  // activates when RC-ESDF reports that the full footprint is in a narrow corridor.
  bool heading_guard_enabled_ = true;
  std::string heading_guard_path_topic_{"local_elastic_path"};
  std::string heading_guard_traversability_grid_topic_{"traversability_grid"};
  std::string heading_guard_base_frame_{"gimbal_yaw_odom"};
  int heading_guard_obstacle_value_threshold_ = 50;
  int heading_guard_lethal_value_threshold_ = 90;
  bool heading_guard_unknown_is_obstacle_ = false;
  double heading_guard_enter_clearance_m_ = 0.20;
  double heading_guard_exit_clearance_m_ = 0.30;
  double heading_guard_footprint_length_m_ = 0.60;
  double heading_guard_footprint_width_m_ = 0.50;
  double heading_guard_lateral_speed_limit_ = 0.0;
  double heading_guard_stop_error_rad_ = 0.22;
  double heading_guard_yaw_kp_ = 2.5;
  double heading_guard_max_angular_speed_ = 1.50;
  std::shared_ptr<RcTraversabilityEsdfProvider> heading_guard_esdf_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex heading_guard_mutex_;
  nav_msgs::msg::Path latest_heading_path_;
  std::string heading_guard_grid_frame_;
  bool narrow_corridor_active_ = false;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_
