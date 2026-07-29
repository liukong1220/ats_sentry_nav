// Copyright 2026

#ifndef ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
#define ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_swerve_mpc/emergency_stop_watchdog.hpp"
#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/trajectory_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

/*节点通过 ROS 2 话题接收 里程计 和 参考轨迹，利用 MPC 求解最优速度指令（vx, vy,
ω）
并发布到控制话题。系统包含了轨迹时间有效性、目标收敛检测、紧急停止信号处理、诊断信息发布等*/
namespace ats_swerve_mpc {

class AtsSwerveMpcNode : public rclcpp::Node {
public:
  explicit AtsSwerveMpcNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onPath(const nav_msgs::msg::Path::SharedPtr message);
  void onExecutionCommand(
      const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr message);
  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr message);
  void onLocalizationStatus(
      const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
      message);
  void onGimbalYawStatus(
      const ats_navigation_interfaces::msg::GimbalYawStatus::SharedPtr message);
  void onControlTimer();
  void engageFailStop();
  bool installPath(const nav_msgs::msg::Path & message);
  bool gimbalExecutionValidLocked(
    const ats_navigation_interfaces::msg::ExecutionCommand & command) const;
  /// 发布确定性零速度并复位控制器内部状态（所有失败/退出路径共用）。
  void publishZeroCommandForFailure(const char * reason_zh);
  /// 校验里程计时效性与位姿跳变，返回 true 表示状态可用于本周期求解。
  bool odometryUsable(State & current);
  /// 检查动力学参数是否落在实车合理区间，越界时输出中文 ERROR/WARN。
  void validateDynamicsParameters(const Se2MpcConfig & config);
  /// 输出控制饱和与求解耗时的分级中文日志。
  void reportSolverDiagnostics(const Se2MpcResult & result);
  Se2MpcConfig loadConfig();
  TrajectoryTrackerConfig loadTrackerConfig();
  void publishCommand(const Control &command);
  void publishPath(
      const std::vector<State> &states,
      const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) const;
  static double normalizeAngle(double angle);

  std::unique_ptr<Se2MpcController> controller_;
  TrajectoryTracker trajectory_tracker_;
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string execution_command_topic_;
  std::string command_topic_;
  std::string emergency_stop_topic_;
  std::string localization_status_topic_;
  std::string gimbal_status_topic_;
  std::string frame_id_;
  double control_rate_hz_ = 20.0;
  double fallback_path_dt_ = 0.1;
  double trajectory_timeout_ = 0.5;
  double emergency_stop_timeout_ = 0.5;
  double execution_command_timeout_ = 0.5;
  double goal_position_tolerance_ = 0.08;
  double goal_yaw_tolerance_ = 0.15;
  bool publish_debug_paths_ = true;
  bool require_localization_status_ = false;
  bool require_gimbal_status_ = false;
  double gimbal_status_timeout_ = 0.5;
  // 里程计允许的最大数据年龄 [s]，超时视为定位链路中断。
  double odometry_timeout_ = 0.25;
  // 单周期允许的最大位置跳变 [m]，超过判定为定位重定位/跳变。
  double odometry_jump_position_ = 0.5;
  // 单周期允许的最大航向跳变 [rad]，超过判定为定位跳变。
  double odometry_jump_yaw_ = 0.8;
  // 求解耗时告警阈值占控制周期的比例（0~1）。
  double solve_time_warn_ratio_ = 0.6;

  mutable std::mutex state_mutex_;
  State current_state_ = State::Zero();
  bool has_odometry_ = false;
  // 最近一帧里程计的消息时间戳与本地接收时刻，用于超时与跳变检测。
  std::optional<rclcpp::Time> last_odometry_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_odometry_signal_;
  std::optional<State> previous_odometry_state_;
  bool odometry_jump_detected_ = false;
  mutable std::mutex trajectory_mutex_;
  std::string trajectory_frame_;
  rclcpp::Time trajectory_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stop_stamp_{0, 0, RCL_ROS_TIME};
  Control last_control_ = Control::Zero();
  EmergencyStopWatchdog emergency_stop_watchdog_;
  bool emergency_stop_watchdog_enabled_ = true;
  bool execution_command_enabled_ = false;
  std::atomic<bool> emergency_stop_signal_received_{false};
  std::atomic<bool> fail_stop_engaged_{true};
  std::atomic<bool> localization_tracking_{false};
  std::optional<std::uint64_t> localization_epoch_;
  std::optional<std::chrono::steady_clock::time_point> last_execution_command_signal_;
  std::uint64_t last_execution_command_sequence_{0};
  std::optional<ats_navigation_interfaces::msg::ExecutionCommand>
      active_execution_command_;
  std::optional<ats_navigation_interfaces::msg::GimbalYawStatus> gimbal_status_;
  std::optional<std::chrono::steady_clock::time_point> last_gimbal_status_signal_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::ExecutionCommand>::SharedPtr
      execution_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::
      SharedPtr localization_status_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::GimbalYawStatus>::SharedPtr
      gimbal_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr horizon_path_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
