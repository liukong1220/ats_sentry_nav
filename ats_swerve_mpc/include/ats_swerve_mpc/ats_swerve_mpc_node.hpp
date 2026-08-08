// Copyright 2026

#ifndef ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
#define ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_

#include <array>
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
#include "ats_swerve_mpc/qp/control_cycle_snapshot.hpp"
#include "ats_swerve_mpc/qp/control_cycle_telemetry.hpp"
#include "ats_swerve_mpc/qp/ltv_qp_osqp_solver.hpp"
#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/trajectory_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @brief ATS 四驱四转舵轮的 ROS 2 控制链唯一速度发布节点。
 * @details 节点接收世界系里程计、时间化 reference、ExecutionCommand、急停、定位与
 *          云台健康输入，在严格的 frame、freshness、lease 与急停门之后调用 iLQR，
 *          并通过唯一的 `/cmd_vel_mpc` publisher 输出车体系 `[vx,vy,wz]`。当
 *          `solver_mode=qp_shadow` 时，QP 只读取 iLQR 求解前冻结的快照记录诊断，
 *          不得改变 tracker、warm start、急停语义或任何 topic ownership。
 */
namespace ats_swerve_mpc {

class AtsSwerveMpcNode : public rclcpp::Node {
public:
  /** @brief 创建 ROS 控制节点、加载 iLQR/QP 参数并建立唯一速度发布者。 */
  explicit AtsSwerveMpcNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /** @brief 接收里程计并维护供控制 timer 读取的最新状态、时间与跳变证据。 */
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  /** @brief 兼容 legacy Path；ExecutionCommand 启用时该入口不得重新授权运动。 */
  void onPath(const nav_msgs::msg::Path::SharedPtr message);
  /** @brief 接收原子执行授权，校验 identity/gimbal/localization 后安装 reference 或急停。 */
  void onExecutionCommand(
      const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr message);
  /** @brief 处理 legacy emergency stop；false 不能越过结构化 ExecutionCommand 授权。 */
  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr message);
  /** @brief 接收定位状态并在非 TRACKING 或 epoch 切换时触发 fail-stop。 */
  void onLocalizationStatus(
      const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
      message);
  /** @brief 接收云台 yaw 健康回执；与当前执行 lease 不匹配时立即停止。 */
  void onGimbalYawStatus(
      const ats_navigation_interfaces::msg::GimbalYawStatus::SharedPtr message);
  /** @brief 控制周期唯一入口：完整检查健康、求解 iLQR、唯一发布并可选 shadow。 */
  void onControlTimer();
  /** @brief 进入急停状态、清空 tracker/warm state 并通过既有唯一发布者发送零速度。 */
  void engageFailStop();
  /** @brief 解析并安装 legacy Path，建立 frame、deadline 和控制器 reset 契约。 */
  bool installPath(const nav_msgs::msg::Path & message);
  /** @brief 在 trajectory mutex 内校验 gimbal 回执的新鲜性和请求/反馈序列一致性。 */
  bool gimbalExecutionValidLocked(
    const ats_navigation_interfaces::msg::ExecutionCommand & command) const;
  /** @brief 发布确定性零速度并复位控制器内部状态，供所有失败/退出路径共用。 */
  void publishZeroCommandForFailure(const char * reason_zh);
  /** @brief 校验里程计时效性与位姿跳变；仅返回 true 时状态才可进入本周期求解。 */
  bool odometryUsable(State & current);
  /** @brief 检查动力学参数是否落在实车合理区间，并对越界项输出中文 ERROR/WARN。 */
  void validateDynamicsParameters(const Se2MpcConfig & config);
  /** @brief 输出车体/轮级饱和与 iLQR 求解耗时的分级中文诊断日志。 */
  void reportSolverDiagnostics(const Se2MpcResult & result);
  /** @brief 对同周期 iLQR 名义轨迹构造/求解 QP，仅记录复核结果。 */
  void runQpShadow(const ControlCycleSnapshot &snapshot,
                   ControlCycleTelemetrySample &telemetry);
  /**
   * @brief 提交本次 iLQR/QP 周期的定长 telemetry，并拆分所有 deadline 根因。
   * @details 本函数只更新固定 128 槽数组及饱和计数，不进行 JSON、文件或阻塞 I/O；它不能
   *          改写 iLQR command、tracker、warm-start 或任何外部安全状态。
   */
  void finalizeControlTelemetry(
      ControlCycleTelemetrySample telemetry,
      std::chrono::steady_clock::time_point cycle_start);
  /** @brief 在非控制回调中复制固定 telemetry 环并返回 JSON，不产生文件或控制副作用。 */
  void dumpControlTelemetry(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /** @brief 从 ROS 参数加载 SE(2) 物理约束和 iLQR 权重，不改变 frame 语义。 */
  Se2MpcConfig loadConfig();
  /** @brief 从 ROS 参数加载有限路径搜索窗、偏差降速和时延补偿。 */
  TrajectoryTrackerConfig loadTrackerConfig();
  /** @brief 通过本节点唯一 publisher 将车体系 [vx,vy,wz] 转为 Twist。 */
  void publishCommand(const Control &command);
  /** @brief 发布仅用于可视化的预测/reference Path，不参与底盘控制所有权。 */
  void publishPath(
      const std::vector<State> &states,
      const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) const;
  /** @brief 为终点误差和里程计跳变判据提供最短 yaw 角差。 */
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
  std::string solver_mode_ = "ilqr";
  LtvQpSolverSettings qp_solver_settings_;
  std::unique_ptr<LtvQpOsqpSolver> qp_solver_;
  LtvQpProblem qp_problem_buffer_;
  LtvQpWarmStart qp_warm_start_;
  bool qp_warm_start_valid_ = false;
  // 0 保持最近 128 槽诊断；正数时只导出同一 Execute/reference/map 身份的固定窗口。
  std::size_t telemetry_sampling_window_cycles_ = 0;
  // 计时环同时记录 ilqr baseline 与 qp_shadow，服务导出时复制，控制 timer 不做字符串序列化。
  ControlCycleTelemetryRing control_telemetry_;
  mutable std::mutex control_telemetry_mutex_;
  std::optional<std::chrono::steady_clock::time_point>
      previous_control_cycle_start_;
  std::uint64_t control_cycle_sequence_ = 0;

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
  std::uint64_t last_execution_command_incarnation_{0};
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      control_telemetry_dump_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
