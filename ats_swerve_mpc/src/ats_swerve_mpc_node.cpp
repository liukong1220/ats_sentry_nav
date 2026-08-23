// Copyright 2026

#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_swerve_mpc {

namespace {
/** @brief 读取三元素参数并在维度错误时保留物理安全默认值。 */
Eigen::Vector3d vectorParameter(rclcpp::Node &node, const std::string &name,
                                const Eigen::Vector3d &defaults) {
  const std::vector<double> values =
      node.declare_parameter<std::vector<double>>(
          name, {defaults(0), defaults(1), defaults(2)});
  if (values.size() != 3) {
    RCLCPP_WARN(node.get_logger(),
                "Parameter '%s' must contain exactly 3 values.", name.c_str());
    return defaults;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

/** @brief 把 steady-clock 相邻时刻转换为毫秒，统一 timer 内所有阶段的时间单位。 */
double elapsedMilliseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return 1000.0 * std::chrono::duration<double>(end - start).count();
}

/** @brief 填写一个有限阶段计时，禁止让无效 duration 混入后续 p50/p95/p99。 */
void recordTiming(ControlCycleTelemetrySample &telemetry,
                  ControlCycleTimingStage stage, double milliseconds) {
  const std::size_t index = controlCycleTimingStageIndex(stage);
  telemetry.stage_ms[index] = milliseconds;
  telemetry.stage_recorded[index] = std::isfinite(milliseconds) && milliseconds >= 0.0;
}

/** @brief 读取当前 DDS domain；缺失或格式错误时返回 -1 而非伪造 domain。 */
int currentRosDomainId() {
  const char *value = std::getenv("ROS_DOMAIN_ID");
  if (value == nullptr || *value == '\0') {
    return -1;
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 0 || parsed > 232) {
    return -1;
  }
  return static_cast<int>(parsed);
}

/**
 * @brief 提取真实 LTV 矩阵的有限尺度指标，供离线收敛归因而非在线调参。
 * @details 只扫描已经预分配的 dense buffer；不复制矩阵、不修改 QP 数值，也不把指标用于
 *          candidate 准入。zero-delta residual 指 $z=0$ 时动态等式的最大绝对残差。
 */
void recordQpProblemMetrics(const LtvQpProblem &problem,
                            ControlCycleTelemetrySample &telemetry) {
  if (!problem.valid || problem.hessian.rows() <= 0 ||
      problem.hessian.rows() != problem.hessian.cols() ||
      problem.equality_matrix.rows() < 3 || !problem.hessian.allFinite() ||
      !problem.equality_matrix.allFinite() || !problem.inequality_matrix.allFinite() ||
      !problem.equality_lower.allFinite()) {
    return;
  }
  telemetry.hessian_diagonal_minimum = problem.hessian.diagonal().minCoeff();
  telemetry.hessian_diagonal_maximum = problem.hessian.diagonal().maxCoeff();
  telemetry.constraint_row_l2_minimum = std::numeric_limits<double>::infinity();
  telemetry.constraint_row_l2_maximum = 0.0;
  const auto update_row_range = [&telemetry](const Eigen::MatrixXd &matrix) {
    for (int row = 0; row < matrix.rows(); ++row) {
      const double norm = matrix.row(row).norm();
      telemetry.constraint_row_l2_minimum =
          std::min(telemetry.constraint_row_l2_minimum, norm);
      telemetry.constraint_row_l2_maximum =
          std::max(telemetry.constraint_row_l2_maximum, norm);
    }
  };
  update_row_range(problem.equality_matrix);
  update_row_range(problem.inequality_matrix);
  telemetry.nonzero_bound_abs_minimum = std::numeric_limits<double>::infinity();
  telemetry.bound_abs_maximum = 0.0;
  const auto update_bound_range = [&telemetry](const Eigen::VectorXd &bounds) {
    for (int index = 0; index < bounds.size(); ++index) {
      const double magnitude = std::abs(bounds(index));
      if (!std::isfinite(magnitude)) {
        continue;
      }
      telemetry.bound_abs_maximum = std::max(telemetry.bound_abs_maximum, magnitude);
      // 忽略 nominal +/- limit 相减留下的机器精度残差，只统计可解释的约束尺度。
      if (magnitude > 1e-12) {
        telemetry.nonzero_bound_abs_minimum =
            std::min(telemetry.nonzero_bound_abs_minimum, magnitude);
      }
    }
  };
  update_bound_range(problem.equality_lower);
  update_bound_range(problem.equality_upper);
  update_bound_range(problem.inequality_lower);
  update_bound_range(problem.inequality_upper);
  update_bound_range(problem.lower_bound);
  update_bound_range(problem.upper_bound);
  telemetry.zero_delta_dynamic_equality_residual = problem.equality_lower.tail(
      problem.equality_lower.size() - 3).cwiseAbs().maxCoeff();
  telemetry.qp_problem_metrics_recorded =
      std::isfinite(telemetry.hessian_diagonal_minimum) &&
      std::isfinite(telemetry.hessian_diagonal_maximum) &&
      std::isfinite(telemetry.constraint_row_l2_minimum) &&
      std::isfinite(telemetry.constraint_row_l2_maximum) &&
      std::isfinite(telemetry.nonzero_bound_abs_minimum) &&
      std::isfinite(telemetry.bound_abs_maximum) &&
      std::isfinite(telemetry.zero_delta_dynamic_equality_residual);
}

constexpr std::uint64_t kTelemetrySummaryInterval = 16;

} // namespace

/**
 * @brief 初始化 ATS iLQR 主链、输入订阅、唯一速度发布者和可选 OSQP shadow。
 * @details `solver_mode=ilqr` 是默认；`qp_shadow` 仅预分配后端和缓冲，`qp` 显式拒绝，
 *          因而构造过程不会改变 `/cmd_vel/autonomy_raw`、急停或底盘所有权。
 */
AtsSwerveMpcNode::AtsSwerveMpcNode(const rclcpp::NodeOptions &options)
    : Node("ats_swerve_mpc", options) {
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
  trajectory_topic_ = declare_parameter<std::string>("trajectory_topic",
                                                     "/minco/reference_path");
  execution_command_topic_ = declare_parameter<std::string>(
      "execution_command_topic", "/planner/execution_command");
  command_topic_ = declare_parameter<std::string>("command_topic",
                                                  "/cmd_vel/autonomy_raw");
  emergency_stop_topic_ = declare_parameter<std::string>(
      "emergency_stop_topic", "/planner/emergency_stop");
  localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");
  gimbal_status_topic_ = declare_parameter<std::string>(
      "gimbal_status_topic", "/gimbal/yaw_status");
  require_gimbal_status_ = declare_parameter<bool>("require_gimbal_status", false);
  gimbal_status_timeout_ = std::max(
      0.1, declare_parameter<double>("gimbal_status_timeout", gimbal_status_timeout_));
  require_localization_status_ =
      declare_parameter<bool>("require_localization_status", false);
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  control_rate_hz_ =
      declare_parameter<double>("control_rate_hz", control_rate_hz_);
  fallback_path_dt_ =
      declare_parameter<double>("fallback_path_dt", fallback_path_dt_);
  trajectory_timeout_ =
      declare_parameter<double>("trajectory_timeout", trajectory_timeout_);
  emergency_stop_timeout_ =
      std::max(0.1, declare_parameter<double>("emergency_stop_timeout",
                                              emergency_stop_timeout_));
  emergency_stop_watchdog_.setTimeout(emergency_stop_timeout_);
  execution_command_timeout_ = std::max(
      0.1, declare_parameter<double>("execution_command_timeout",
                                     execution_command_timeout_));
  execution_command_enabled_ = !execution_command_topic_.empty();
  goal_position_tolerance_ = declare_parameter<double>(
      "goal_position_tolerance", goal_position_tolerance_);
  goal_yaw_tolerance_ =
      declare_parameter<double>("goal_yaw_tolerance", goal_yaw_tolerance_);
  publish_debug_paths_ =
      declare_parameter<bool>("publish_debug_paths", publish_debug_paths_);
  // 里程计健康监测阈值：超时判定定位链路中断，跳变判定重定位/漂移突变。
  odometry_timeout_ =
      std::max(0.02, declare_parameter<double>("odometry_timeout",
                                               odometry_timeout_));
  odometry_jump_position_ = std::max(
      0.05, declare_parameter<double>("odometry_jump_position",
                                      odometry_jump_position_));
  odometry_jump_yaw_ = std::max(
      0.05, declare_parameter<double>("odometry_jump_yaw", odometry_jump_yaw_));
  solve_time_warn_ratio_ = std::clamp(
      declare_parameter<double>("solve_time_warn_ratio",
                                solve_time_warn_ratio_),
      0.1, 1.0);
  solver_mode_ = declare_parameter<std::string>("solver_mode", "ilqr");
  if (solver_mode_ != "ilqr" && solver_mode_ != "qp_shadow" &&
      solver_mode_ != "qp") {
    throw std::invalid_argument(
        "solver_mode must be one of ilqr, qp_shadow, qp");
  }
  if (solver_mode_ == "qp") {
    throw std::invalid_argument(
        "solver_mode=qp is reserved and cannot publish QP control in this phase");
  }
  qp_solver_settings_.max_iterations = declare_parameter<int>(
      "qp_max_iterations", 400);
  qp_solver_settings_.time_limit_ms = declare_parameter<double>(
      "qp_time_limit_ms", 10.0);
  qp_solver_settings_.max_primal_residual = declare_parameter<double>(
      "qp_max_primal_residual", 1e-4);
  qp_solver_settings_.max_dual_residual = declare_parameter<double>(
      "qp_max_dual_residual", 1e-4);
  qp_solver_settings_.max_tracking_slack = declare_parameter<double>(
      "qp_max_tracking_slack", 0.0);
  qp_solver_settings_.max_hard_constraint_violation = declare_parameter<double>(
      "qp_max_hard_constraint_violation", 1e-7);
  const int requested_sampling_cycles = declare_parameter<int>(
      "telemetry_sampling_window_cycles", 0);
  if (requested_sampling_cycles < 0 ||
      requested_sampling_cycles > static_cast<int>(ControlCycleTelemetryRing::kCapacity)) {
    throw std::invalid_argument(
        "telemetry_sampling_window_cycles must be in [0, 128]");
  }
  telemetry_sampling_window_cycles_ =
      static_cast<std::size_t>(requested_sampling_cycles);
  control_telemetry_.configureSamplingWindow(telemetry_sampling_window_cycles_);
  const Se2MpcConfig mpc_config = loadConfig();
  // 在创建 Se2MpcController 前统一拒绝无法表示或超过 dense 资源审计上限的 horizon；
  // 这样 solver_mode=ilqr 也不会因异常参数先分配不可控的预测缓存。
  const auto ltv_dimensions = checkedLtvQpDimensions(mpc_config.horizon);
  if (!ltv_dimensions.valid) {
    throw std::invalid_argument(std::string("invalid LTV-QP dimensions: ") +
                                ltv_dimensions.validation_error);
  }
  if (!std::isfinite(mpc_config.dt) || mpc_config.dt <= 0.0) {
    throw std::invalid_argument("MPC dt must be finite and positive");
  }
  validateDynamicsParameters(mpc_config);
  controller_ = std::make_unique<Se2MpcController>(mpc_config);
  if (solver_mode_ == "qp_shadow") {
    qp_problem_buffer_ = LtvQpBuilder::allocate(ltv_dimensions.horizon);
    if (!qp_problem_buffer_.hasExpectedLayout()) {
      throw std::invalid_argument(
          "LTV-QP buffer allocation did not produce the checked layout");
    }
    qp_solver_ = std::make_unique<LtvQpOsqpSolver>(
        ltv_dimensions, qp_solver_settings_);
    if (!qp_solver_->initialized()) {
      RCLCPP_ERROR(get_logger(),
                   "OSQP v1.0.0 shadow 后端 setup 失败；保持 iLQR 主链，"
                   "不生成伪造 QP 结果。");
      qp_solver_.reset();
    }
  }
  trajectory_tracker_.setConfig(loadTrackerConfig());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AtsSwerveMpcNode::onOdometry, this, std::placeholders::_1));
  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      trajectory_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&AtsSwerveMpcNode::onPath, this, std::placeholders::_1));
  if (execution_command_enabled_) {
    execution_command_sub_ = create_subscription<
        ats_navigation_interfaces::msg::ExecutionCommand>(
      execution_command_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsSwerveMpcNode::onExecutionCommand, this,
                std::placeholders::_1));
    // ExecutionCommand carries its own stop/execute lease.  The legacy Bool
    // remains a stop-only failsafe and cannot reauthorize a reference.
    emergency_stop_watchdog_enabled_ = false;
  }
  if (!emergency_stop_topic_.empty()) {
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&AtsSwerveMpcNode::onEmergencyStop, this,
                  std::placeholders::_1));
  } else {
    emergency_stop_watchdog_enabled_ = false;
    fail_stop_engaged_.store(false);
    RCLCPP_WARN(get_logger(), "Emergency-stop input is explicitly disabled.");
  }
  if (!localization_status_topic_.empty()) {
    localization_status_sub_ =
        create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
            localization_status_topic_,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AtsSwerveMpcNode::onLocalizationStatus, this,
                      std::placeholders::_1));
  } else if (require_localization_status_) {
    throw std::invalid_argument(
        "require_localization_status=true requires a non-empty status topic");
  } else {
    localization_tracking_.store(true);
  }
  if (require_gimbal_status_ && gimbal_status_topic_.empty()) {
    throw std::invalid_argument(
        "require_gimbal_status=true requires a non-empty gimbal status topic");
  }
  if (!gimbal_status_topic_.empty()) {
    gimbal_status_sub_ = create_subscription<
        ats_navigation_interfaces::msg::GimbalYawStatus>(
      gimbal_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsSwerveMpcNode::onGimbalYawStatus, this,
                std::placeholders::_1));
  }
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_,
                                                             rclcpp::QoS(10));
  predicted_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/predicted_path", rclcpp::QoS(1));
  horizon_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/reference_horizon", rclcpp::QoS(1));
  // 导出服务只复制固定遥测环并返回字符串，不发布控制、不会重新运行 QP，也不改变 warm-start。
  control_telemetry_dump_service_ = create_service<std_srvs::srv::Trigger>(
      "~/dump_control_telemetry",
      std::bind(&AtsSwerveMpcNode::dumpControlTelemetry, this,
                std::placeholders::_1, std::placeholders::_2));
  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
      std::bind(&AtsSwerveMpcNode::onControlTimer, this));
  publishCommand(Control::Zero());
  RCLCPP_INFO(get_logger(),
              "【启动就绪】四舵轮 MPC 已启动：里程计='%s' 参考轨迹='%s' "
              "指令输出='%s' 预测步数=%d 步长=%.3f s 控制频率=%.1f Hz",
              odom_topic_.c_str(), trajectory_topic_.c_str(),
              command_topic_.c_str(), controller_->config().horizon,
              controller_->config().dt, control_rate_hz_);
  RCLCPP_INFO(get_logger(),
              "LTV-QP solver_mode=%s（qp_shadow 仅诊断，iLQR 保持唯一输出 owner）",
              solver_mode_.c_str());
}

/** @brief 加载 iLQR/四轮物理参数，保持控制为 body-frame [vx,vy,wz] 的既有契约。 */
Se2MpcConfig AtsSwerveMpcNode::loadConfig() {
  Se2MpcConfig config;
  config.horizon = declare_parameter<int>("horizon", config.horizon);
  config.dt = declare_parameter<double>("dt", config.dt);
  config.state_weight =
      vectorParameter(*this, "state_weight", config.state_weight);
  config.control_weight =
      vectorParameter(*this, "control_weight", config.control_weight);
  config.control_delta_weight = vectorParameter(*this, "control_delta_weight",
                                                config.control_delta_weight);
  config.terminal_weight =
      vectorParameter(*this, "terminal_weight", config.terminal_weight);
  config.max_vx = declare_parameter<double>("max_vx", config.max_vx);
  config.max_vy = declare_parameter<double>("max_vy", config.max_vy);
  config.max_wz = declare_parameter<double>("max_wz", config.max_wz);
  config.max_ax = declare_parameter<double>("max_ax", config.max_ax);
  config.max_ay = declare_parameter<double>("max_ay", config.max_ay);
  config.max_awz = declare_parameter<double>("max_awz", config.max_awz);
  config.wheel_base_x =
      declare_parameter<double>("wheel_base_x", config.wheel_base_x);
  config.wheel_base_y =
      declare_parameter<double>("wheel_base_y", config.wheel_base_y);
  const double wheel_radius =
      std::max(1e-6, declare_parameter<double>("wheel_radius", 0.0425));
  const double drive_gear_ratio =
      std::max(1e-6, declare_parameter<double>("drive_gear_ratio", 1.0));
  const double steer_gear_ratio =
      std::max(1e-6, declare_parameter<double>("steer_gear_ratio", 1.0));
  const double motor_max_rpm =
      std::max(0.0, declare_parameter<double>("motor_max_rpm", 450.0));
  const double steer_max_rpm =
      std::max(0.0, declare_parameter<double>("steer_max_rpm", 120.0));
  const double actuator_redundancy =
      std::max(1.0, declare_parameter<double>("actuator_redundancy", 1.2));
  constexpr double kRpmToRadiansPerSecond = 2.0 * 3.14159265358979323846 / 60.0;
  const double raw_wheel_speed =
      wheel_radius * motor_max_rpm * kRpmToRadiansPerSecond / drive_gear_ratio;
  const double conservative_wheel_speed = raw_wheel_speed / actuator_redundancy;
  const double raw_steer_rate =
      steer_max_rpm * kRpmToRadiansPerSecond / steer_gear_ratio;
  const double conservative_steer_rate = raw_steer_rate / actuator_redundancy;
  config.max_wheel_speed = declare_parameter<double>(
      "max_wheel_speed", conservative_wheel_speed);
  config.max_wheel_acceleration = declare_parameter<double>(
      "max_wheel_acceleration", 2.0);
  config.max_steer_rate =
      declare_parameter<double>("max_steer_rate", conservative_steer_rate);
  RCLCPP_INFO(
      get_logger(),
      "【动力学参数】驱动轮：轮径 %.4f m、电机上限 %.0f rpm、减速比 %.2f → "
      "理论轮速 %.3f m/s，按冗余系数 %.2f 降额 %.3f m/s，最终生效 %.3f m/s；"
      "转向：电机上限 %.0f rpm、减速比 %.2f → 理论 %.3f rad/s，"
      "降额 %.3f rad/s，最终生效 %.3f rad/s。",
      wheel_radius, motor_max_rpm, drive_gear_ratio, raw_wheel_speed,
      actuator_redundancy, conservative_wheel_speed, config.max_wheel_speed,
      steer_max_rpm, steer_gear_ratio, raw_steer_rate, conservative_steer_rate,
      config.max_steer_rate);
  config.max_iterations =
      declare_parameter<int>("max_iterations", config.max_iterations);
  config.regularization =
      declare_parameter<double>("regularization", config.regularization);
  config.line_search_decay =
      declare_parameter<double>("line_search_decay", config.line_search_decay);
  config.min_line_search_step = declare_parameter<double>(
      "min_line_search_step", config.min_line_search_step);
  config.convergence_tolerance = declare_parameter<double>(
      "convergence_tolerance", config.convergence_tolerance);
  return config;
}

/** @brief 加载路径投影、横向偏差降速和时延补偿参数。 */
TrajectoryTrackerConfig AtsSwerveMpcNode::loadTrackerConfig() {
  TrajectoryTrackerConfig config;
  config.backward_search_window = declare_parameter<double>(
      "projection_backward_window", config.backward_search_window);
  config.forward_search_window = declare_parameter<double>(
      "projection_forward_window", config.forward_search_window);
  config.cross_track_slowdown_start = declare_parameter<double>(
      "cross_track_slowdown_start", config.cross_track_slowdown_start);
  config.cross_track_slowdown_end = declare_parameter<double>(
      "cross_track_slowdown_end", config.cross_track_slowdown_end);
  config.min_progress_scale = declare_parameter<double>(
      "min_reference_progress_scale", config.min_progress_scale);
  config.command_latency_compensation = declare_parameter<double>(
      "command_latency_compensation", config.command_latency_compensation);
  return config;
}

/**
 * @brief 里程计（Point-LIO 定位输出）回调：刷新世界系状态并做数据异常检测。
 * @param message /localization 上的 nav_msgs::msg::Odometry。
 * @note 由 ROS 执行器在每帧定位到达时调用（SensorDataQoS，约 50~100 Hz）。
 *       除位姿本身外，这里额外记录消息时间戳与本地接收时刻，
 *       控制周期据此判断"定位超时"和"定位跳变"——实车上定位丢失往往先表现为
 *       里程计停更或位姿瞬跳，若不检测会让 MPC 用过期状态继续输出速度。
 */
void AtsSwerveMpcNode::onOdometry(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  State incoming;
  incoming(0) = message->pose.pose.position.x;
  incoming(1) = message->pose.pose.position.y;
  incoming(2) = tf2::getYaw(message->pose.pose.orientation);
  if (!incoming.allFinite()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "【传感器异常】里程计位姿包含 NaN/Inf，已丢弃该帧："
                          "话题 '%s'，请检查定位节点输出。",
                          odom_topic_.c_str());
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_odometry_ = false;
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  // 位姿跳变检测：与上一帧比较，超过阈值说明定位发生重定位或漂移突变。
  if (previous_odometry_state_) {
    const double position_jump =
        (incoming.head<2>() - previous_odometry_state_->head<2>()).norm();
    const double yaw_jump =
        std::abs(normalizeAngle(incoming(2) - (*previous_odometry_state_)(2)));
    if (position_jump > odometry_jump_position_ ||
        yaw_jump > odometry_jump_yaw_) {
      odometry_jump_detected_ = true;
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 500,
          "【定位跳变】里程计单帧位姿跳变过大：位置 %.3f m（阈值 %.3f），"
          "航向 %.3f rad（阈值 %.3f）；本周期将输出零速度以防失控。",
          position_jump, odometry_jump_position_, yaw_jump, odometry_jump_yaw_);
    }
  }
  previous_odometry_state_ = incoming;
  current_state_ = incoming;
  last_odometry_stamp_ = rclcpp::Time(message->header.stamp);
  last_odometry_signal_ = std::chrono::steady_clock::now();
  has_odometry_ = true;
}

/**
 * @brief 校验里程计可用性（是否收到、是否超时、是否刚发生跳变）。
 * @param current 输出参数：可用时写入当前世界系状态 [x, y, yaw]。
 * @return true=状态可用于本周期 MPC 求解；false=必须走零速度兜底。
 * @note 每个控制周期在求解前调用一次。
 */
bool AtsSwerveMpcNode::odometryUsable(State &current) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_odometry_ || !last_odometry_signal_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【通信超时】尚未收到里程计话题 '%s' 的有效数据，"
                         "MPC 保持零速度等待定位。",
                         odom_topic_.c_str());
    return false;
  }
  const double age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         *last_odometry_signal_)
                         .count();
  if (age > odometry_timeout_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500,
                          "【传感器超时】里程计已 %.3f s 未更新（阈值 %.3f s），"
                          "疑似定位/LiDAR 链路中断，输出零速度。",
                          age, odometry_timeout_);
    return false;
  }
  if (odometry_jump_detected_) {
    // 跳变只封锁一个周期，随后由新帧重新建立连续性。
    odometry_jump_detected_ = false;
    return false;
  }
  current = current_state_;
  return true;
}

/**
 * @brief 处理 legacy Path 兼容入口。
 * @details 一旦启用 ExecutionCommand，该回调只记录警告且不修改 tracker，防止独立 DDS
 *          topic 绕开原子执行授权；legacy 模式下才委托 installPath。
 */
void AtsSwerveMpcNode::onPath(const nav_msgs::msg::Path::SharedPtr message) {
  if (execution_command_enabled_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring legacy Path because ExecutionCommand owns MPC authorization.");
    return;
  }
  installPath(*message);
}

/**
 * @brief 校验 frame/时间并把 Path 转为 tracker 的相对时间 reference。
 * @details 成功时设置 reference deadline、重置 iLQR warm start 与 last control；失败不
 *          覆盖原有安全状态，调用方负责进入 fail-stop。
 */
bool AtsSwerveMpcNode::installPath(const nav_msgs::msg::Path & message) {
  if (require_localization_status_ && !localization_tracking_.load()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring trajectory while localization is not TRACKING.");
    return false;
  }
  if (message.poses.size() < 2) {
    RCLCPP_WARN(get_logger(), "Ignoring trajectory with fewer than two poses.");
    return false;
  }
  if (!frame_id_.empty() && !message.header.frame_id.empty() &&
      message.header.frame_id != frame_id_) {
    RCLCPP_ERROR(get_logger(),
                 "Ignoring trajectory in frame '%s'; MPC frame is '%s'.",
                 message.header.frame_id.c_str(), frame_id_.c_str());
    return false;
  }
  std::vector<TimedState> parsed;
  parsed.reserve(message.poses.size());
  const double header_time = rclcpp::Time(message.header.stamp).seconds();
  bool monotonic = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < message.poses.size(); ++i) {
    const auto &pose = message.poses[i];
    double pose_time = rclcpp::Time(pose.header.stamp).seconds();
    if (pose_time <= 0.0) {
      pose_time = header_time + fallback_path_dt_ * static_cast<double>(i);
    }
    monotonic = monotonic && pose_time > previous_time;
    TimedState timed;
    timed.time = pose_time;
    timed.state << pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation);
    if (!timed.state.allFinite()) {
      return false;
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
        message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0;
    if (last_stop_stamp_.nanoseconds() > 0 &&
        (missing_stamp ||
         rclcpp::Time(message.header.stamp) <= last_stop_stamp_)) {
      RCLCPP_WARN(
          get_logger(),
          "Ignoring a trajectory older than the latest emergency stop.");
      return false;
    }
    trajectory_tracker_.setTrajectory(std::move(parsed));
    trajectory_frame_ =
        message.header.frame_id.empty() ? frame_id_ : message.header.frame_id;
    const double maximum_tracking_duration =
        trajectory_tracker_.duration() /
        std::max(1e-3, trajectory_tracker_.minimumProgressScale());
    trajectory_deadline_ = now() + rclcpp::Duration::from_seconds(
                                       maximum_tracking_duration +
                                       std::max(0.0, trajectory_timeout_));
  }
  controller_->reset();
  last_control_.setZero();
  return true;
}

/**
 * @brief 接收唯一的执行授权和 reference 原子快照。
 * @details 严格拒绝 incarnation/sequence 倒退、gimbal 或 localization epoch 不匹配；STOP
 *          与任何非法 EXECUTE 都走既有急停，而不是只停止 QP shadow。
 */
void AtsSwerveMpcNode::onExecutionCommand(
    const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr message) {
  if (!message || message->manager_incarnation == 0 ||
      message->command_sequence == 0) {
    engageFailStop();
    return;
  }
  bool install_reference = false;
  bool stop = message->mode ==
    ats_navigation_interfaces::msg::ExecutionCommand::MODE_STOP;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (message->manager_incarnation < last_execution_command_incarnation_) {
      return;
    }
    if (message->manager_incarnation > last_execution_command_incarnation_) {
      // Goal Manager 重启后其进程内 sequence 可以从头计数。新 incarnation 的首条
      // 消息必须是 STOP，避免旧进程迟到的 EXECUTE 替换已受保护的 reference。
      if (!stop) {
        stop = true;
        active_execution_command_.reset();
      } else {
        last_execution_command_incarnation_ = message->manager_incarnation;
        last_execution_command_sequence_ = message->command_sequence;
        active_execution_command_.reset();
      }
    } else if (message->command_sequence <= last_execution_command_sequence_) {
      return;
    } else {
      last_execution_command_sequence_ = message->command_sequence;
    }
    last_execution_command_signal_ = std::chrono::steady_clock::now();
    if (stop) {
      active_execution_command_.reset();
    } else if (message->mode !=
        ats_navigation_interfaces::msg::ExecutionCommand::MODE_EXECUTE ||
      message->goal_id == 0 || message->reference.poses.size() < 2 ||
      (require_gimbal_status_ && message->gimbal_request_sequence == 0) ||
      (message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_GIMBAL_COMPENSATED &&
       message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_BODY_YAW_FOLLOW) ||
      !gimbalExecutionValidLocked(*message) ||
      (require_localization_status_ &&
       (!localization_tracking_.load() || !localization_epoch_ ||
        *localization_epoch_ != message->localization_epoch))) {
      stop = true;
      active_execution_command_.reset();
    } else {
      const bool same_reference = active_execution_command_ &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        active_execution_command_->map_generation == message->map_generation &&
        active_execution_command_->map_publication_sequence ==
          message->map_publication_sequence &&
        active_execution_command_->reference.header.stamp.sec ==
          message->reference.header.stamp.sec &&
        active_execution_command_->reference.header.stamp.nanosec ==
          message->reference.header.stamp.nanosec;
      if (!same_reference) {
        install_reference = true;
      }
      active_execution_command_ = *message;
    }
  }
  if (stop) {
    engageFailStop();
    return;
  }
  if (install_reference && !installPath(message->reference)) {
    engageFailStop();
    return;
  }
  fail_stop_engaged_.store(false);
}

/**
 * @brief 处理 legacy emergency-stop 心跳。
 * @details structured ExecutionCommand 模式下 legacy false 永远不能恢复运动；其它模式中
 *          watchdog 的 stop 状态拥有优先权，并由 engageFailStop 实际发布零速度。
 */
void AtsSwerveMpcNode::onEmergencyStop(
    const std_msgs::msg::Bool::SharedPtr message) {
  if (execution_command_enabled_) {
    // legacy false 不能解除结构化 ExecutionCommand 建立的停止状态。
    if (message->data) {
      engageFailStop();
    }
    return;
  }
  const bool first_signal = !emergency_stop_signal_received_.exchange(true);
  emergency_stop_watchdog_.update(message->data);
  if (first_signal) {
    fail_stop_engaged_.store(false);
    engageFailStop();
  }
  if (message->data) {
    engageFailStop();
  } else if (!require_localization_status_ || localization_tracking_.load()) {
    fail_stop_engaged_.store(false);
  } else {
    engageFailStop();
  }
}

/** @brief 只接受 TRACKING 定位状态；epoch 切换或失跟立即使当前轨迹和 warm-start 失效。 */
void AtsSwerveMpcNode::onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
        message) {
  if (message->state !=
      ats_navigation_interfaces::msg::LocalizationStatus::STATE_TRACKING) {
    localization_tracking_.store(false);
    engageFailStop();
    return;
  }
  if (localization_epoch_ && *localization_epoch_ != message->epoch) {
    engageFailStop();
  }
  localization_epoch_ = message->epoch;
  localization_tracking_.store(true);
}

/** @brief 更新云台回执并在 active command 的 yaw 权限失效时触发急停。 */
void AtsSwerveMpcNode::onGimbalYawStatus(
    const ats_navigation_interfaces::msg::GimbalYawStatus::SharedPtr message) {
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    gimbal_status_ = *message;
    last_gimbal_status_signal_ = std::chrono::steady_clock::now();
    stop = active_execution_command_ && !gimbalExecutionValidLocked(*active_execution_command_);
  }
  if (stop) {
    engageFailStop();
  }
}

/**
 * @brief 在 trajectory mutex 已持有时验证云台回执的新鲜性、authority 和序列。
 * @details 未要求云台状态时返回 true；要求时任何缺失、超时或 lock 不满足都不可授权控制。
 */
bool AtsSwerveMpcNode::gimbalExecutionValidLocked(
  const ats_navigation_interfaces::msg::ExecutionCommand & command) const {
  if (!require_gimbal_status_) {
    return true;
  }
  if (!gimbal_status_ || !last_gimbal_status_signal_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *last_gimbal_status_signal_).count();
  if (age > gimbal_status_timeout_ || !gimbal_status_->tf_healthy ||
    gimbal_status_->yaw_authority != command.yaw_authority ||
    gimbal_status_->request_sequence != command.gimbal_request_sequence ||
    gimbal_status_->sequence < command.gimbal_feedback_sequence) {
    return false;
  }
  return !command.requires_gimbal_lock || gimbal_status_->locked;
}

/**
 * @brief iLQR 唯一控制周期。
 * @details 依次执行输入健康/急停、reference 新鲜度、目标到达检查、iLQR 求解和唯一 Twist
 *          发布；随后 qp_shadow 仅读取冻结的 ControlCycleSnapshot，不能重读 ROS 状态或
 *          改动 tracker、last_control、emergency stop 与 publisher 所有权。
 */
void AtsSwerveMpcNode::onControlTimer() {
  const auto cycle_start = std::chrono::steady_clock::now();
  const std::optional<std::chrono::steady_clock::time_point> previous_cycle_start =
      previous_control_cycle_start_;
  previous_control_cycle_start_ = cycle_start;
  const std::uint64_t cycle_sequence = ++control_cycle_sequence_;
  bool execution_lease_valid = !execution_command_enabled_;
  bool gimbal_valid = !require_gimbal_status_;
  const bool localization_fresh =
      !require_localization_status_ || localization_tracking_.load();
  // 1. 紧急停止检查
  if (!localization_fresh) {
    engageFailStop();
    return;
  }
  if (execution_command_enabled_) {
    const auto lease_check_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      execution_lease_valid = last_execution_command_signal_ &&
        lease_check_time >= *last_execution_command_signal_ &&
        std::chrono::duration<double>(
          lease_check_time - *last_execution_command_signal_).count() <=
          execution_command_timeout_ && active_execution_command_ &&
        gimbalExecutionValidLocked(*active_execution_command_);
      gimbal_valid = active_execution_command_ &&
                     gimbalExecutionValidLocked(*active_execution_command_);
    }
    if (!execution_lease_valid || fail_stop_engaged_.load()) {
      engageFailStop();
      return;
    }
  } else if (emergency_stop_watchdog_enabled_ &&
      (emergency_stop_watchdog_.stopRequired() || fail_stop_engaged_.load())) {
    engageFailStop();
    return;
  }
  // 2. 获取当前状态（不可用时必须输出确定性零速度，不能静默 return）
  State current = State::Zero();
  if (!odometryUsable(current)) {
    publishZeroCommandForFailure("里程计不可用");
    return;
  }
  // 3. 获取目标、投影、参考序列
  State goal = State::Zero();
  TrajectoryProjection projection;
  std::vector<Se2Reference> references;
  bool trajectory_expired = false;
  // 参考提取（含 trajectory_mutex_ 等待、投影和 horizon 构造）单独计时，
  // 使其不再隐含在 state_trajectory_snapshot 内无法归因。
  const auto reference_extraction_start = std::chrono::steady_clock::now();
  double reference_extraction_ms = 0.0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (trajectory_tracker_.empty()) {
      // 无参考轨迹同样属于失败兜底路径：必须持续发布零速度，
      // 否则底盘会保持上一条 Twist 一直滑行（实车失控的直接来源）。
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "【通信超时】尚未收到有效参考轨迹（话题 '%s'），"
                           "MPC 保持零速度。",
                           trajectory_topic_.c_str());
      publishZeroCommandForFailure(nullptr);
      return;
    }
    goal = trajectory_tracker_.goal();
    projection = trajectory_tracker_.project(current);
    references = trajectory_tracker_.buildHorizon(
        projection, controller_->config().horizon, controller_->config().dt);
    trajectory_expired =
        trajectory_deadline_.nanoseconds() > 0 && now() > trajectory_deadline_;
  }
  reference_extraction_ms = elapsedMilliseconds(reference_extraction_start,
                                                std::chrono::steady_clock::now());
  // 4. 判断是否达到目标或轨迹过期
  const double goal_position_error =
      (goal.head<2>() - current.head<2>()).norm();
  const double goal_yaw_error = std::abs(normalizeAngle(goal(2) - current(2)));
  if ((goal_position_error <= goal_position_tolerance_ &&
       goal_yaw_error <= goal_yaw_tolerance_) ||
      trajectory_expired) {
    if (trajectory_expired) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "【轨迹过期】参考轨迹已超过有效期，停止跟踪并输出零速度；"
                           "请检查规划器发布频率与时间戳同步。");
    }
    publishZeroCommandForFailure(nullptr);
    return;
  }
  // 5. 检查参考序列长度
  if (references.size() <
      static_cast<std::size_t>(controller_->config().horizon + 1)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "【规划失败】参考序列长度 %zu 不足预测时域所需 %d 点，输出零速度。",
        references.size(),
        static_cast<int>(controller_->config().horizon + 1));
    publishZeroCommandForFailure(nullptr);
    return;
  }
  // 6. 调用 MPC 求解
  const auto snapshot_start = cycle_start;
  ControlCycleSnapshot snapshot;
  snapshot.cycle_sequence = cycle_sequence;
  snapshot.steady_start = cycle_start;
  snapshot.current_state = current;
  snapshot.references = references;
  snapshot.last_control_before_solve = last_control_;
  snapshot.reference_fresh = projection.valid && !trajectory_expired &&
                             references.size() >= static_cast<std::size_t>(
                                 controller_->config().horizon + 1);
  snapshot.localization_fresh = localization_fresh;
  snapshot.execution_lease_valid = execution_lease_valid;
  snapshot.gimbal_valid = gimbal_valid;
  snapshot.emergency_stop_active = fail_stop_engaged_.load() ||
      (emergency_stop_watchdog_enabled_ &&
       emergency_stop_watchdog_.stopRequired());
  // 节点尚无独立 map/footprint 健康生产者；在真实 snapshot 接入前必须 fail-closed。
  snapshot.map_fresh = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    snapshot.reference_frame = trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
    snapshot.reference_deadline_ns = trajectory_deadline_.nanoseconds();
    if (active_execution_command_) {
      const auto &command = *active_execution_command_;
      snapshot.manager_incarnation = command.manager_incarnation;
      snapshot.command_sequence = command.command_sequence;
      snapshot.goal_id = command.goal_id;
      snapshot.localization_epoch = command.localization_epoch;
      snapshot.map_generation = command.map_generation;
      snapshot.map_publication_sequence = command.map_publication_sequence;
      snapshot.reference_stamp_ns =
          rclcpp::Time(command.reference.header.stamp).nanoseconds();
      if (!command.reference.header.frame_id.empty()) {
        snapshot.reference_frame = command.reference.header.frame_id;
      }
    }
  }
  // 在 iLQR 求解前冻结 canonical digest。随后 shadow 重新计算该摘要，若任何输入在
  // 两者之间被改写，就不能宣称它们使用同一控制周期输入。
  snapshot.identity_digest_valid =
      controlCycleSnapshotIdentityInputsFinite(snapshot);
  snapshot.identity_digest = snapshot.identity_digest_valid ?
      controlCycleSnapshotIdentityDigest(snapshot) : 0;
  ControlCycleTelemetrySample telemetry;
  telemetry.cycle_sequence = snapshot.cycle_sequence;
  telemetry.snapshot_identity_digest = snapshot.identity_digest;
  telemetry.manager_incarnation = snapshot.manager_incarnation;
  telemetry.command_sequence = snapshot.command_sequence;
  telemetry.goal_id = snapshot.goal_id;
  telemetry.localization_epoch = snapshot.localization_epoch;
  telemetry.map_generation = snapshot.map_generation;
  telemetry.map_publication_sequence = snapshot.map_publication_sequence;
  telemetry.reference_stamp_ns = snapshot.reference_stamp_ns;
  telemetry.reference_deadline_ns = snapshot.reference_deadline_ns;
  std::snprintf(telemetry.reference_frame.data(), telemetry.reference_frame.size(),
                "%s", snapshot.reference_frame.c_str());
  telemetry.execution_lease_valid = snapshot.execution_lease_valid;
  telemetry.reference_fresh = snapshot.reference_fresh;
  if (previous_cycle_start) {
    recordTiming(telemetry, ControlCycleTimingStage::kTimerInterarrival,
                 elapsedMilliseconds(*previous_cycle_start, cycle_start));
  }
  recordTiming(telemetry, ControlCycleTimingStage::kStateTrajectorySnapshot,
               elapsedMilliseconds(snapshot_start, std::chrono::steady_clock::now()));
  recordTiming(telemetry, ControlCycleTimingStage::kReferenceExtraction,
               reference_extraction_ms);

  const auto ilqr_start = std::chrono::steady_clock::now();
  const Se2MpcResult result =
      controller_->solve(snapshot.current_state, snapshot.references,
                         snapshot.last_control_before_solve);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrSolve,
               elapsedMilliseconds(ilqr_start, std::chrono::steady_clock::now()));
  // iLQR 内部拆分：仅为把 solve 的耗时归属到具体阶段，它们是 kIlqrSolve 的
  // 子项而非额外串行开销；kIlqrJacobian 又内含于 kIlqrBackwardPass。
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrWarmStart,
               result.stage_timing.warm_start_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrRollout,
               result.stage_timing.rollout_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrBackwardPass,
               result.stage_timing.backward_pass_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrJacobian,
               result.stage_timing.jacobian_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrLineSearch,
               result.stage_timing.line_search_ms);
  if (!result.success || result.controls.empty()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "【MPC求解失败】iLQR 未获得可信解：迭代 %d 次、被接受 %d 次、代价 %.4f、"
        "耗时 %.2f ms、横向误差 %.3f m；已切换零速度兜底并复位控制器。",
        result.iterations, result.accepted_iterations, result.cost,
        result.solve_time_ms, projection.cross_track_error);
    publishZeroCommandForFailure(nullptr);
    return;
  }
  // 7. 应用第一个控制量并发布
  last_control_ = result.controls.front();
  const auto command_publish_start = std::chrono::steady_clock::now();
  publishCommand(last_control_);
  recordTiming(telemetry, ControlCycleTimingStage::kIlqrCommandPublish,
               elapsedMilliseconds(command_publish_start,
                                   std::chrono::steady_clock::now()));
  for (int axis = 0; axis < 3; ++axis) {
    telemetry.ilqr_first_control[static_cast<std::size_t>(axis)] = last_control_(axis);
  }
  const auto diagnostics_log_start = std::chrono::steady_clock::now();
  reportSolverDiagnostics(result);
  double logging_publish_ms = elapsedMilliseconds(
      diagnostics_log_start, std::chrono::steady_clock::now());
  snapshot.ilqr_result = result;
  if (solver_mode_ == "qp_shadow") {
    runQpShadow(snapshot, telemetry);
  }
  // 8. 发布调试路径
  if (publish_debug_paths_) {
    publishPath(result.states, predicted_path_pub_);
    std::vector<State> reference_states;
    reference_states.reserve(references.size());
    for (const auto &reference : references) {
      reference_states.push_back(reference.state);
    }
    publishPath(reference_states, horizon_path_pub_);
  }
  const auto debug_log_start = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(get_logger(),
               "MPC vx=%.3f vy=%.3f wz=%.3f cross_track=%.3f "
               "progress_scale=%.2f cost=%.3f "
               "solve=%.2fms",
               last_control_(0), last_control_(1), last_control_(2),
               projection.cross_track_error,
               trajectory_tracker_.progressScale(projection.cross_track_error),
               result.cost, result.solve_time_ms);
  logging_publish_ms += elapsedMilliseconds(debug_log_start,
                                             std::chrono::steady_clock::now());
  recordTiming(telemetry, ControlCycleTimingStage::kLoggingPublish,
               logging_publish_ms);
  finalizeControlTelemetry(telemetry, cycle_start);
}

/**
 * @brief 对已发布 iLQR 同周期名义轨迹执行 OSQP shadow 审计。
 * @details 用 snapshot 的 current/reference/solve 前 last_control 构建固定结构 LTV-QP，
 *          再从 primal 重建 delta_u、做共享模型非线性 rollout 和 hard-check；无论 solved、
 *          timeout 或 reject 都不发布 QP 控制，也不影响 iLQR warm-start。
 */
void AtsSwerveMpcNode::runQpShadow(
    const ControlCycleSnapshot &snapshot,
    ControlCycleTelemetrySample &telemetry) {
  LtvQpSolveResult result;
  LtvQpPrimalCandidate candidate;
  LtvQpCandidateAudit audit;
  const bool backend_available = qp_solver_ && qp_solver_->initialized();
  const auto problem_build_start = std::chrono::steady_clock::now();
  const bool problem_built = backend_available && LtvQpBuilder::build(
      snapshot.current_state, snapshot.ilqr_result.states,
      snapshot.ilqr_result.controls, snapshot.references,
      snapshot.last_control_before_solve, controller_->config(),
      qp_problem_buffer_);
  if (problem_built) {
    recordQpProblemMetrics(qp_problem_buffer_, telemetry);
  }
  recordTiming(telemetry, ControlCycleTimingStage::kQpProblemBuild,
               elapsedMilliseconds(problem_build_start,
                                   std::chrono::steady_clock::now()));
  if (!backend_available) {
    result.status = LtvQpSolverStatus::kBackendUnavailable;
  } else if (!problem_built) {
    result.status = LtvQpSolverStatus::kInvalidProblem;
  } else {
    const LtvQpProblem &problem = qp_problem_buffer_;
    result = qp_solver_->solveLtvProblem(
        problem, qp_solver_settings_,
        qp_warm_start_valid_ ? &qp_warm_start_ : nullptr);
  }
  if (problem_built) {
    recordTiming(telemetry, ControlCycleTimingStage::kOsqpNumericUpdate,
                 result.wall_update_time_ms);
    recordTiming(telemetry, ControlCycleTimingStage::kQpBackendPhase,
                 result.wall_qp_phase_time_ms);
    recordTiming(telemetry, ControlCycleTimingStage::kQpCompletePhase,
                 result.wall_complete_qp_phase_time_ms);
    recordTiming(telemetry, ControlCycleTimingStage::kOsqpSolve,
                 result.wall_solve_time_ms);
  }
  const LtvQpProblem &problem = qp_problem_buffer_;
  if (result.status == LtvQpSolverStatus::kSolved &&
      result.primal_solution.size() == problem.decisionSize() &&
      result.dual_solution.size() == qp_solver_->constraintStructure().rows &&
      result.primal_solution.allFinite() && result.dual_solution.allFinite()) {
    qp_warm_start_.primal = result.primal_solution;
    qp_warm_start_.dual = result.dual_solution;
    qp_warm_start_valid_ = true;
  } else {
    if (qp_solver_) {
      qp_solver_->resetWarmStart();
    }
    qp_warm_start_valid_ = false;
  }
  LtvQpCandidateSafety safety;
  safety.inputs_healthy = snapshot.localization_fresh &&
                          snapshot.reference_fresh &&
                          snapshot.execution_lease_valid &&
                          snapshot.gimbal_valid;
  safety.emergency_stop_active = snapshot.emergency_stop_active;
  safety.collision_free = false;
  safety.localization_fresh = snapshot.localization_fresh;
  safety.reference_fresh = snapshot.reference_fresh;
  safety.execution_lease_valid = snapshot.execution_lease_valid;
  safety.gimbal_valid = snapshot.gimbal_valid;
  safety.map_fresh = snapshot.map_fresh;
  if (result.status == LtvQpSolverStatus::kSolved && problem_built) {
    const auto reconstruction_start = std::chrono::steady_clock::now();
    candidate = LtvQpCandidateReconstructor::reconstruct(
        snapshot.current_state, problem, snapshot.ilqr_result.controls,
        controller_->config(), result);
    recordTiming(telemetry,
                 ControlCycleTimingStage::kPrimalReconstructionRollout,
                 elapsedMilliseconds(reconstruction_start,
                                     std::chrono::steady_clock::now()));
    const auto audit_start = std::chrono::steady_clock::now();
    audit = LtvQpCandidateValidator::validate(
        problem, snapshot.ilqr_result.controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), qp_solver_settings_, safety, result, candidate);
    recordTiming(telemetry, ControlCycleTimingStage::kCandidateHardCheck,
                 elapsedMilliseconds(audit_start, std::chrono::steady_clock::now()));
  } else {
    const auto audit_start = std::chrono::steady_clock::now();
    audit = LtvQpCandidateValidator::validate(
        problem, snapshot.ilqr_result.controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), qp_solver_settings_, safety, result);
    recordTiming(telemetry, ControlCycleTimingStage::kCandidateHardCheck,
                 elapsedMilliseconds(audit_start, std::chrono::steady_clock::now()));
  }

  telemetry.qp_shadow_attempted = true;
  telemetry.same_snapshot_identity = snapshot.identity_digest_valid &&
      snapshot.identity_digest == controlCycleSnapshotIdentityDigest(snapshot);
  telemetry.status = result.status;
  telemetry.iterations = result.iterations;
  telemetry.warm_start_used = result.warm_start_used;
  telemetry.osqp_reported_update_ms = result.update_time_ms;
  telemetry.osqp_reported_solve_ms = result.solve_time_ms;
  telemetry.osqp_wall_update_ms = result.wall_update_time_ms;
  telemetry.osqp_wall_solve_ms = result.wall_solve_time_ms;
  telemetry.qp_complete_phase_ms = result.wall_complete_qp_phase_time_ms;
  telemetry.osqp_wall_qp_phase_ms = result.wall_qp_phase_time_ms;
  telemetry.primal_residual = result.primal_residual;
  telemetry.dual_residual = result.dual_residual;
  telemetry.hard_constraint_margin =
      qp_solver_settings_.max_hard_constraint_violation -
      std::max(result.hard_constraint_maximum_violation,
               audit.actual_hard_constraint_maximum_violation);
  telemetry.slack_maximum = std::max(result.slack_maximum,
                                     audit.actual_slack_maximum);
  telemetry.candidate_feasible = audit.feasible;
  const char *reason = audit.rejection_reason.empty() ? "none" :
                       audit.rejection_reason.c_str();
  std::snprintf(telemetry.rejection_reason.data(),
                telemetry.rejection_reason.size(), "%s", reason);
  telemetry.collision_gate = safety.collision_free;
  telemetry.map_gate = safety.map_fresh;
  if (candidate.valid && !candidate.controls.empty()) {
    const Control &qp = candidate.controls.front();
    for (int axis = 0; axis < 3; ++axis) {
      const std::size_t index = static_cast<std::size_t>(axis);
      telemetry.qp_first_control[index] = qp(axis);
      telemetry.first_control_delta[index] = qp(axis) - telemetry.ilqr_first_control[index];
    }
  }
}

/**
 * @brief 完成本控制周期的性能记录并按明确预算累积根因计数。
 * @details OSQP update/solve 各自以 `qp_time_limit_ms` 复核；iLQR、LTV 构造、candidate
 *          audit、telemetry、日志与完整 callback 都以运行时 `control_period_ms` 复核。
 *          同一个周期可有多个根因，因此总数只能作为 root-cause events，绝不能替代分布。
 */
void AtsSwerveMpcNode::finalizeControlTelemetry(
    ControlCycleTelemetrySample telemetry,
    std::chrono::steady_clock::time_point cycle_start) {
  const double control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);

  ControlCycleTimingDistribution callback_distribution;
  ControlCycleTimingDistribution qp_backend_phase_distribution;
  ControlCycleTimingDistribution qp_complete_phase_distribution;
  const auto aggregation_start = std::chrono::steady_clock::now();
  if (telemetry.cycle_sequence % kTelemetrySummaryInterval == 0) {
    std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
    callback_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kFullCallback);
    qp_backend_phase_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kQpBackendPhase);
    qp_complete_phase_distribution = control_telemetry_.distribution(
        ControlCycleTimingStage::kQpCompletePhase);
  }
  recordTiming(telemetry, ControlCycleTimingStage::kPercentileAggregation,
               elapsedMilliseconds(aggregation_start, std::chrono::steady_clock::now()));

  const auto telemetry_log_start = std::chrono::steady_clock::now();
  if (telemetry.cycle_sequence % kTelemetrySummaryInterval == 0) {
    RCLCPP_INFO(
        get_logger(),
        "控制周期 telemetry cycle=%llu mode=%s rate=%.1fHz period=%.3fms "
        "qp_status=%s iter=%d qp_wall(update/solve/backend/complete)=%.3f/%.3f/%.3f/%.3fms "
        "callback_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_backend_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_complete_p50/p95/p99=%.3f/%.3f/%.3fms "
        "same_snapshot=%s feasible=%s reject=%s",
        static_cast<unsigned long long>(telemetry.cycle_sequence), solver_mode_.c_str(),
        control_rate_hz_, control_period_ms, ltvQpSolverStatusName(telemetry.status),
        telemetry.iterations, telemetry.osqp_wall_update_ms, telemetry.osqp_wall_solve_ms,
        telemetry.osqp_wall_qp_phase_ms, telemetry.qp_complete_phase_ms,
        callback_distribution.p50_ms, callback_distribution.p95_ms,
        callback_distribution.p99_ms, qp_backend_phase_distribution.p50_ms,
        qp_backend_phase_distribution.p95_ms, qp_backend_phase_distribution.p99_ms,
        qp_complete_phase_distribution.p50_ms, qp_complete_phase_distribution.p95_ms,
        qp_complete_phase_distribution.p99_ms,
        telemetry.same_snapshot_identity ? "true" : "false",
        telemetry.candidate_feasible ? "true" : "false",
        telemetry.rejection_reason.data());
  }
  const std::size_t logging_index = controlCycleTimingStageIndex(
      ControlCycleTimingStage::kLoggingPublish);
  const double telemetry_log_ms = elapsedMilliseconds(
      telemetry_log_start, std::chrono::steady_clock::now());
  telemetry.stage_ms[logging_index] += telemetry_log_ms;
  telemetry.stage_recorded[logging_index] = std::isfinite(telemetry.stage_ms[logging_index]) &&
      telemetry.stage_ms[logging_index] >= 0.0;
  recordTiming(telemetry, ControlCycleTimingStage::kFullCallback,
               elapsedMilliseconds(cycle_start, std::chrono::steady_clock::now()));

  const auto exceeded = [&telemetry](ControlCycleTimingStage stage,
                                     double budget_ms) {
    const std::size_t index = controlCycleTimingStageIndex(stage);
    return telemetry.stage_recorded[index] &&
           telemetry.stage_ms[index] > budget_ms;
  };
  const double candidate_audit_ms =
      telemetry.stage_ms[controlCycleTimingStageIndex(
          ControlCycleTimingStage::kPrimalReconstructionRollout)] +
      telemetry.stage_ms[controlCycleTimingStageIndex(
          ControlCycleTimingStage::kCandidateHardCheck)];
  const bool candidate_audit_overrun =
      (telemetry.stage_recorded[controlCycleTimingStageIndex(
           ControlCycleTimingStage::kPrimalReconstructionRollout)] ||
       telemetry.stage_recorded[controlCycleTimingStageIndex(
           ControlCycleTimingStage::kCandidateHardCheck)]) &&
      candidate_audit_ms > control_period_ms;

  std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
  if (!control_telemetry_.push(telemetry, cycle_start)) {
    return;
  }
  if (telemetry.qp_shadow_attempted &&
      telemetry.status == LtvQpSolverStatus::kTimeLimit) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpTimeLimitStatus);
  }
  if (telemetry.qp_shadow_attempted && exceeded(
          ControlCycleTimingStage::kOsqpSolve, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpSolveBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kFullCallback, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kCallbackPeriodOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kIlqrSolve, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kIlqrSolveBudgetOverrun);
  }
  if (telemetry.qp_shadow_attempted && exceeded(
          ControlCycleTimingStage::kQpProblemBuild, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpProblemBuildBudgetOverrun);
  }
  if (telemetry.qp_shadow_attempted && exceeded(
          ControlCycleTimingStage::kOsqpNumericUpdate, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpUpdateBudgetOverrun);
  }
  if (telemetry.qp_shadow_attempted && exceeded(
          ControlCycleTimingStage::kQpBackendPhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpPhaseBudgetOverrun);
  }
  if (telemetry.qp_shadow_attempted && exceeded(
          ControlCycleTimingStage::kQpCompletePhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpCompletePhaseBudgetOverrun);
  }
  if (telemetry.qp_shadow_attempted && candidate_audit_overrun) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpCandidateAuditBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kPercentileAggregation, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kTelemetryAggregationBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kLoggingPublish, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kLoggingPublishBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kTimerInterarrival, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kTimerInterarrivalOverrun);
  }
}

/**
 * @brief 复制当前固定遥测窗口，序列化为供回归脚本保存的只读 JSON。
 * @details service 在 ROS executor 中运行；控制 timer 只在短临界区写固定数组。服务不访问
 *          tracker、求解器、command publisher 或 safety gate，因此不能改变任何控制行为。
 */
void AtsSwerveMpcNode::dumpControlTelemetry(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  ControlCycleTelemetryRing snapshot;
  {
    std::lock_guard<std::mutex> lock(control_telemetry_mutex_);
    snapshot = control_telemetry_;
  }
  ControlCycleTelemetryMetadata metadata;
  metadata.solver_mode = solver_mode_;
  metadata.control_rate_hz = control_rate_hz_;
  metadata.control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);
  metadata.qp_max_iterations = qp_solver_settings_.max_iterations;
  metadata.qp_time_limit_ms = qp_solver_settings_.time_limit_ms;
  metadata.qp_max_primal_residual = qp_solver_settings_.max_primal_residual;
  metadata.qp_max_dual_residual = qp_solver_settings_.max_dual_residual;
  metadata.qp_max_tracking_slack = qp_solver_settings_.max_tracking_slack;
  metadata.qp_max_hard_constraint_violation =
      qp_solver_settings_.max_hard_constraint_violation;
  metadata.ros_domain_id = currentRosDomainId();
  metadata.use_sim_time = get_parameter_or<bool>("use_sim_time", false);
  response->success = true;
  response->message = snapshot.toJson(metadata);
}

/**
 * @brief 所有非紧急停止的失败/退出路径统一出口：发布确定性零速度。
 * @param reason_zh 可选的中文原因短语；为 nullptr 时不额外打印（调用处已打印）。
 * @note 每个控制周期最多调用一次。零速度必须"持续发布"而非只发一次，
 *       因为下游底盘固件按最新 Twist 执行，停止发布等于让上一条速度一直生效。
 */
void AtsSwerveMpcNode::publishZeroCommandForFailure(const char *reason_zh) {
  last_control_.setZero();
  publishCommand(last_control_);
  controller_->reset();
  if (reason_zh != nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制兜底】%s，已输出确定性零速度。", reason_zh);
  }
}

/**
 * @brief 校验动力学参数是否处于实车合理区间。
 * @param config 已加载的 MPC 配置。
 * @note 节点构造期调用一次。参数越界在仿真里往往看不出问题，
 *       但实车会直接表现为超速、舵轮堵转或约束整体失效，因此必须显式告警。
 */
void AtsSwerveMpcNode::validateDynamicsParameters(
    const Se2MpcConfig &config) {
  if (config.dt <= 0.0 || config.horizon <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】dt=%.4f s、horizon=%d 非法，MPC 无法求解。",
                 config.dt, config.horizon);
  }
  const double control_period = 1.0 / std::max(1.0, control_rate_hz_);
  if (std::abs(config.dt - control_period) > 0.2 * control_period) {
    RCLCPP_WARN(
        get_logger(),
        "【动力学参数不匹配】预测步长 dt=%.4f s 与控制周期 %.4f s（%.1f Hz）"
        "偏差超过 20%%，预测时间轴与实际执行时间轴错位，跟踪会出现系统性滞后。",
        config.dt, control_period, control_rate_hz_);
  }
  if (config.wheel_base_x <= 0.0 || config.wheel_base_y <= 0.0) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】轮心半轴偏置 wheel_base_x=%.3f m、"
                 "wheel_base_y=%.3f m 必须为正值（实车约 0.270 m）；"
                 "当前配置下四舵轮模块级约束退化，仅车体平移限幅生效。",
                 config.wheel_base_x, config.wheel_base_y);
  }
  if (config.max_wheel_speed <= 1e-6) {
    RCLCPP_ERROR(get_logger(),
                 "【动力学参数越界】单轮最大线速度 max_wheel_speed=%.3f m/s 未配置，"
                 "轮速上限约束完全失效，实车存在超速风险。",
                 config.max_wheel_speed);
  } else {
    // 全向底盘同时平移与自转时，单轮速度上界为 hypot(v, wz*R)，
    // 其中 R=hypot(wheel_base_x, wheel_base_y) 为轮心到底盘中心的距离。
    const double corner_radius =
        std::hypot(std::max(0.0, config.wheel_base_x),
                   std::max(0.0, config.wheel_base_y));
    const double worst_case =
        std::hypot(std::hypot(config.max_vx, config.max_vy),
                   config.max_wz * corner_radius);
    if (worst_case > config.max_wheel_speed * 1.5) {
      RCLCPP_WARN(get_logger(),
                  "【动力学参数不一致】车体速度上限组合出的最坏单轮速度 %.3f m/s "
                  "已达单轮上限 %.3f m/s 的 %.2f 倍，MPC 会长期工作在轮速饱和区，"
                  "建议下调 max_vx/max_vy/max_wz。",
                  worst_case, config.max_wheel_speed,
                  worst_case / config.max_wheel_speed);
    }
  }
  if (config.max_steer_rate <= 1e-6) {
    RCLCPP_WARN(get_logger(),
                "【动力学参数越界】舵轮最大转向角速度 max_steer_rate=%.3f rad/s "
                "未配置，舵角速率约束失效，急转向时可能要求舵轮瞬间掉头。",
                config.max_steer_rate);
  }
  if (config.max_wheel_acceleration <= 1e-6) {
    RCLCPP_WARN(get_logger(),
                "【动力学参数越界】单轮最大线加速度 max_wheel_acceleration=%.3f "
                "m/s^2 未配置，轮端加速度约束失效。",
                config.max_wheel_acceleration);
  }
}

/**
 * @brief 输出求解饱和与实时性诊断日志。
 * @param result 本周期 MPC 求解结果。
 * @note 每个成功周期调用一次，全部使用 THROTTLE 避免刷屏。
 */
void AtsSwerveMpcNode::reportSolverDiagnostics(const Se2MpcResult &result) {
  if (!result.module_limits_active) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "【动力学参数越界】四舵轮模块级约束未生效"
                          "（wheel_base_* 或 max_wheel_speed 缺失），"
                          "当前仅车体级限幅在保护实车，请立即检查参数文件。");
  }
  if (result.wheel_limit_saturated) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【控制饱和】单轮速度触及上限 %.3f m/s，指令已整体等比缩放；"
                         "跟踪精度会下降，检查参考速度是否超出底盘能力。",
                         controller_->config().max_wheel_speed);
  } else if (result.body_limit_saturated) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制饱和】车体速度指令触及 vx/vy/wz 上限"
                         "（%.2f/%.2f/%.2f），已限幅输出。",
                         controller_->config().max_vx,
                         controller_->config().max_vy,
                         controller_->config().max_wz);
  }
  if (result.increment_limited) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制饱和】舵角速率/轮加速度约束触发增量回退，"
                         "本周期实际指令小于 MPC 期望值。");
  }
  const double control_period_ms = 1000.0 / std::max(1.0, control_rate_hz_);
  if (result.solve_time_ms > solve_time_warn_ratio_ * control_period_ms) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【实时性告警】MPC 求解耗时 %.2f ms，已占控制周期 %.2f ms 的 "
                         "%.0f%%，接近超时会导致控制周期抖动。",
                         result.solve_time_ms, control_period_ms,
                         100.0 * result.solve_time_ms / control_period_ms);
  }
}

/**
 * @brief 执行节点级 fail-stop。
 * @details 首次进入时清空 reference、controller warm state 和 frame/deadline；每次调用都通过
 *          同一 command publisher 重发零速度，防止下游保持最后一条非零 Twist。
 */
void AtsSwerveMpcNode::engageFailStop() {
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

/** @brief 将内部车体系 Control 映射到唯一 `geometry_msgs::msg::Twist` 发布通道。 */
void AtsSwerveMpcNode::publishCommand(const Control &command) {
  geometry_msgs::msg::Twist message;
  message.linear.x = command(0);
  message.linear.y = command(1);
  message.angular.z = command(2);
  command_pub_->publish(message);
}

/** @brief 发布调试状态序列为 Path；仅供观察，不是 QP/iLQR 的控制输入。 */
void AtsSwerveMpcNode::publishPath(
    const std::vector<State> &states,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher) const {
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id =
      trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
  path.poses.reserve(states.size());
  for (const auto &state : states) {
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

/** @brief 计算最短 yaw 差，供终点判定和定位跳变检测复用。 */
double AtsSwerveMpcNode::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace ats_swerve_mpc
