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
#include "ats_swerve_mpc/qp/ltv_qp_osqp_solver.hpp"
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
  /// 发布确定性零速度并复位控制器内部状态（所有失败/退出路径共用）。
  void publishZeroCommandForFailure(const char * reason_zh);
  /// 校验里程计时效性与位姿跳变，返回 true 表示状态可用于本周期求解。
  bool odometryUsable(State & current);
  /// 检查动力学参数是否落在实车合理区间，越界时输出中文 ERROR/WARN。
  void validateDynamicsParameters(const Se2MpcConfig & config);
  /// 输出控制饱和与求解耗时的分级中文日志。
  void reportSolverDiagnostics(const Se2MpcResult & result);
  /**
   * @brief 单控制周期的只读输入/身份快照。
   * @details iLQR 先以该快照的 current/reference/last_control 求解，qp_shadow
   *          随后只读取同一份副本；它不携带可发布的 QP Twist。
   */
  struct ControlCycleSnapshot {
    std::uint64_t cycle_sequence = 0;
    std::chrono::steady_clock::time_point steady_start{};
    State current_state = State::Zero();
    std::vector<Se2Reference> references;
    Control last_control_before_solve = Control::Zero();
    Se2MpcResult ilqr_result;

    std::uint64_t manager_incarnation = 0;
    std::uint64_t command_sequence = 0;
    std::uint64_t goal_id = 0;
    std::uint64_t localization_epoch = 0;
    std::uint64_t map_generation = 0;
    std::uint64_t map_publication_sequence = 0;
    std::string reference_frame;
    std::int64_t reference_stamp_ns = 0;
    std::int64_t reference_deadline_ns = 0;
    bool reference_fresh = false;
    bool emergency_stop_active = true;
    bool localization_fresh = false;
    bool gimbal_valid = false;
    bool execution_lease_valid = false;
    // Twist-only 节点尚无独立 map/footprint 健康 producer；接入真实 snapshot 前必须
    // 保持 false，令 qp_shadow candidate 以 fail-closed 方式拒绝。
    bool map_fresh = false;
  };

  /**
   * @brief 有界 shadow 诊断槽位。
   * @details 固定容量避免 timer 中累积路径、控制序列或无界日志；fallback 始终为 0，
   *          因为 shadow 从不接管 iLQR 控制输出。
   */
  struct QpShadowTelemetry {
    std::uint64_t cycle_sequence = 0;
    bool same_snapshot_identity = false;
    LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
    int iterations = 0;
    bool warm_start_used = false;
    double solve_time_ms = 0.0;
    double update_time_ms = 0.0;
    double callback_elapsed_ms = 0.0;
    double primal_residual = 0.0;
    double dual_residual = 0.0;
    double hard_constraint_margin = 0.0;
    double slack_maximum = 0.0;
    std::array<double, 3> qp_first_control{{0.0, 0.0, 0.0}};
    std::array<double, 3> ilqr_first_control{{0.0, 0.0, 0.0}};
    std::array<double, 3> first_control_delta{{0.0, 0.0, 0.0}};
    bool candidate_feasible = false;
    std::array<char, 96> rejection_reason{{}};
    std::uint64_t deadline_miss_count = 0;
    std::uint64_t candidate_reject_count = 0;
    std::uint64_t fallback_count = 0;
    bool collision_gate = false;
    bool map_gate = false;
  };

  /** @brief 对同周期 iLQR 名义轨迹构造/求解 QP，仅记录复核结果。 */
  void runQpShadow(const ControlCycleSnapshot &snapshot);
  /** @brief 写入固定 telemetry ring，并按窗口输出 solve/callback p50/p95/p99。 */
  void recordQpShadowTelemetry(
      const ControlCycleSnapshot &snapshot,
      const LtvQpSolveResult &result,
      const LtvQpCandidateAudit &audit,
      const LtvQpPrimalCandidate *candidate);
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
  static constexpr std::size_t kQpShadowTelemetryCapacity = 128;
  std::array<QpShadowTelemetry, kQpShadowTelemetryCapacity>
      qp_shadow_telemetry_{};
  std::size_t qp_shadow_telemetry_count_ = 0;
  std::size_t qp_shadow_telemetry_cursor_ = 0;
  std::uint64_t qp_shadow_deadline_miss_count_ = 0;
  std::uint64_t qp_shadow_candidate_reject_count_ = 0;
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
  rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HPP_
