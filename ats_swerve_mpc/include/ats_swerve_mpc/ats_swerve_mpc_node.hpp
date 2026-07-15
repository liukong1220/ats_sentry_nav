// Copyright 2026

#ifndef ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
#define ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ats_swerve_mpc/emergency_stop_watchdog.hpp"
#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/trajectory_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ats_swerve_mpc
{

class AtsSwerveMpcNode : public rclcpp::Node
{
public:
  explicit AtsSwerveMpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onPath(const nav_msgs::msg::Path::SharedPtr message);
  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr message);
  void onControlTimer();
  void engageFailStop();
  Se2MpcConfig loadConfig();
  TrajectoryTrackerConfig loadTrackerConfig();
  void publishCommand(const Control & command);
  void publishPath(
    const std::vector<State> & states,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher) const;
  static double normalizeAngle(double angle);

  std::unique_ptr<Se2MpcController> controller_;
  TrajectoryTracker trajectory_tracker_;
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string command_topic_;
  std::string emergency_stop_topic_;
  std::string frame_id_;
  double control_rate_hz_ = 20.0;
  double fallback_path_dt_ = 0.1;
  double trajectory_timeout_ = 0.5;
  double emergency_stop_timeout_ = 0.5;
  double goal_position_tolerance_ = 0.08;
  double goal_yaw_tolerance_ = 0.15;
  bool publish_debug_paths_ = true;

  mutable std::mutex state_mutex_;
  State current_state_ = State::Zero();
  bool has_odometry_ = false;
  mutable std::mutex trajectory_mutex_;
  std::string trajectory_frame_;
  rclcpp::Time trajectory_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stop_stamp_{0, 0, RCL_ROS_TIME};
  Control last_control_ = Control::Zero();
  EmergencyStopWatchdog emergency_stop_watchdog_;
  bool emergency_stop_watchdog_enabled_ = true;
  std::atomic<bool> emergency_stop_signal_received_{false};
  std::atomic<bool> fail_stop_engaged_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr horizon_path_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
