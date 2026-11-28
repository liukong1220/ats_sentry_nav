#include "ats_swerve_mpc_node_helpers.hpp"

namespace ats_swerve_mpc {

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
  planner_status_topic_ = declare_parameter<std::string>(
      "planner_status_topic", "/minco/planning_status");
  map_ready_topic_ = declare_parameter<std::string>(
      "map_ready_topic", "/rog_map_adapter/ready");
  map_ready_timeout_sec_ = std::max(0.1, declare_parameter<double>(
      "map_ready_timeout_sec", map_ready_timeout_sec_));
  planning_snapshot_topic_ = declare_parameter<std::string>(
      "planning_snapshot_topic", planning_snapshot_topic_);
  planning_snapshot_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("planning_snapshot_timeout_sec",
                                     map_ready_timeout_sec_));
  footprint_length_ = std::max(
      0.0, declare_parameter<double>("footprint_length", footprint_length_));
  footprint_width_ = std::max(
      0.0, declare_parameter<double>("footprint_width", footprint_width_));
  footprint_safety_margin_ = std::max(
      0.0, declare_parameter<double>("footprint_safety_margin",
                                     footprint_safety_margin_));
  qp_esdf_config_.enabled =
      declare_parameter<bool>("qp_esdf_corridor_enabled", true);
  qp_esdf_config_.corridor_clearance_m = std::max(
      0.0, declare_parameter<double>("qp_esdf_corridor_clearance", 0.44));
  qp_esdf_config_.reference_margin_m = std::max(
      0.0, declare_parameter<double>("qp_esdf_reference_margin", 0.015));
  qp_esdf_config_.min_corridor_clearance_m = std::max(
      0.0, declare_parameter<double>(
               "qp_esdf_min_corridor_clearance",
               0.5 * std::min(footprint_length_, footprint_width_) +
                   footprint_safety_margin_));
  qp_esdf_config_.slack_linear_weight = std::max(
      0.0, declare_parameter<double>("qp_esdf_slack_linear_weight", 400.0));
  qp_esdf_config_.slack_quadratic_weight = std::max(
      1.0, declare_parameter<double>("qp_esdf_slack_quadratic_weight", 4000.0));
  rollout_safety_enabled_ =
      declare_parameter<bool>("rollout_safety_enabled", true);
  rollout_immediate_stop_horizon_sec_ = std::max(
      0.0, declare_parameter<double>("rollout_immediate_stop_horizon_sec",
                                     rollout_immediate_stop_horizon_sec_));
  rollout_brake_horizon_sec_ = std::max(
      rollout_immediate_stop_horizon_sec_,
      declare_parameter<double>("rollout_brake_horizon_sec",
                                rollout_brake_horizon_sec_));
  rollout_slowdown_horizon_sec_ = std::max(
      rollout_brake_horizon_sec_,
      declare_parameter<double>("rollout_slowdown_horizon_sec",
                                rollout_slowdown_horizon_sec_));
  rollout_slowdown_clearance_m_ = std::max(
      0.0, declare_parameter<double>("rollout_slowdown_clearance_m",
                                     rollout_slowdown_clearance_m_));
  rollout_min_slowdown_scale_ = std::clamp(
      declare_parameter<double>("rollout_min_slowdown_scale",
                                rollout_min_slowdown_scale_),
      0.1, 1.0);
  rollout_hard_stop_penetration_m_ = std::max(
      0.0, declare_parameter<double>("rollout_hard_stop_penetration_m",
                                     rollout_hard_stop_penetration_m_));
  command_topic_ = declare_parameter<std::string>("command_topic",
                                                  "/cmd_vel/autonomy_raw");
  emergency_stop_topic_ = declare_parameter<std::string>(
      "emergency_stop_topic", "/planner/emergency_stop");
  localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");
  gimbal_status_topic_ = declare_parameter<std::string>(
      "gimbal_status_topic", "/gimbal/yaw_status");
  require_gimbal_status_ = declare_parameter<bool>("require_gimbal_status", false);
  enforce_gimbal_lock_on_execution_ = declare_parameter<bool>(
      "enforce_gimbal_lock_on_execution", enforce_gimbal_lock_on_execution_);
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
  execution_command_future_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("execution_command_future_tolerance_sec",
                                     execution_command_future_tolerance_sec_));
  execution_command_enabled_ = !execution_command_topic_.empty();
  require_planning_snapshot_ = declare_parameter<bool>(
      "require_planning_snapshot", execution_command_enabled_);
  supersede_grace_sec_ = std::max(
      0.0, declare_parameter<double>("supersede_grace_sec", supersede_grace_sec_));
  goal_position_tolerance_ = declare_parameter<double>(
      "goal_position_tolerance", goal_position_tolerance_);
  goal_yaw_tolerance_ =
      declare_parameter<double>("goal_yaw_tolerance", goal_yaw_tolerance_);
  terminal_stop_speed_threshold_ = std::max(
      0.0, declare_parameter<double>("terminal_stop_speed_threshold",
                                     terminal_stop_speed_threshold_));
  terminal_speed_window_sec_ = std::max(
      1e-3, declare_parameter<double>("terminal_speed_window_sec",
                                      terminal_speed_window_sec_));
  terminal_approach_time_constant_ = std::max(
      0.0, declare_parameter<double>("terminal_approach_time_constant",
                                     terminal_approach_time_constant_));
  terminal_approach_radius_ = std::max(
      0.0, declare_parameter<double>("terminal_approach_radius", terminal_approach_radius_));
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
  solver_mode_ = declare_parameter<std::string>("solver_mode", "qp");
  if (solver_mode_ != "qp") {
    throw std::invalid_argument(
        "solver_mode must be 'qp' (OSQP v1.0.0 SQP-RTI is the sole MPC solver)");
  }
  qp_solver_settings_.max_iterations = declare_parameter<int>(
      "qp_max_iterations", 400);
  qp_solver_settings_.time_limit_ms = declare_parameter<double>(
      "qp_time_limit_ms", 12.0);
  qp_solver_settings_.max_primal_residual = declare_parameter<double>(
      "qp_max_primal_residual", 2.0e-3);
  qp_solver_settings_.max_dual_residual = declare_parameter<double>(
      "qp_max_dual_residual", 5.0e-3);
  qp_solver_settings_.max_tracking_slack = declare_parameter<double>(
      "qp_max_tracking_slack", 0.35);
  qp_solver_settings_.max_hard_constraint_violation = declare_parameter<double>(
      "qp_max_hard_constraint_violation", 2e-3);
  qp_esdf_config_.max_slack_m = qp_solver_settings_.max_tracking_slack;
  qp_rti_iterations_ = std::clamp(
      static_cast<int>(declare_parameter<int>("qp_rti_iterations", 1)), 1, 2);
  qp_rti_second_pass_delta_threshold_ = std::max(
      0.01, declare_parameter<double>("qp_rti_second_pass_delta_threshold", 0.15));
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
  qp_problem_buffer_ = LtvQpBuilder::allocate(ltv_dimensions.horizon);
  if (!qp_problem_buffer_.hasExpectedLayout()) {
    throw std::invalid_argument(
        "LTV-QP sparse buffer allocation did not produce the checked layout");
  }
  qp_solver_ = std::make_unique<LtvQpOsqpSolver>(
      ltv_dimensions, qp_solver_settings_);
  if (!qp_solver_->initialized()) {
    RCLCPP_ERROR(get_logger(),
                 "OSQP v1.0.0 后端初始化失败；控制周期将保持确定性零速度 fail-stop。");
    qp_solver_.reset();
  }
  trajectory_tracker_.setConfig(loadTrackerConfig());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AtsSwerveMpcNode::onOdometry, this, std::placeholders::_1));
  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      trajectory_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&AtsSwerveMpcNode::onPath, this, std::placeholders::_1));
  if (execution_command_enabled_) {
    if (planner_status_topic_.empty() || map_ready_topic_.empty()) {
      throw std::invalid_argument(
          "ExecutionCommand requires planner_status_topic and map_ready_topic");
    }
    planner_status_sub_ = create_subscription<
        ats_navigation_interfaces::msg::PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsSwerveMpcNode::onPlannerStatus, this, std::placeholders::_1));
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsSwerveMpcNode::onMapReady, this, std::placeholders::_1));
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
    RCLCPP_WARN(get_logger(), "【安全配置】急停输入被显式关闭。");
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
  if (require_planning_snapshot_ && planning_snapshot_topic_.empty()) {
    throw std::invalid_argument(
        "require_planning_snapshot=true requires a non-empty planning_snapshot_topic");
  }
  if (!planning_snapshot_topic_.empty()) {
    planning_snapshot_sub_ = create_subscription<
        ats_navigation_interfaces::msg::PlanningMapSnapshot>(
        planning_snapshot_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&AtsSwerveMpcNode::onPlanningSnapshot, this,
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
              "【求解器配置】模式=%s（OSQP v1.0.0 零分配稀疏 SQP-RTI 唯一主求解器，未通过准入即确定性零速度 fail-stop）",
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
  config.lag_weight =
      declare_parameter<double>("lag_weight", config.lag_weight);
  config.contour_weight =
      declare_parameter<double>("contour_weight", config.contour_weight);
  config.actuator_lag_tau_sec = std::max(
      0.0, declare_parameter<double>("actuator_lag_tau_sec",
                                     config.actuator_lag_tau_sec));
  config.actuator_delay_sec = std::clamp(
      declare_parameter<double>("actuator_delay_sec",
                                config.actuator_delay_sec),
      0.0, 0.25);
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
  config.clamp_bisection_iterations = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("clamp_bisection_iterations",
                                config.clamp_bisection_iterations)));
  config.warm_start_blend_ratio = std::clamp(
      declare_parameter<double>("warm_start_blend_ratio",
                                config.warm_start_blend_ratio),
      0.0, 1.0);
  return config;
}

/** @brief 加载路径投影、横向偏差降速和时延补偿参数。 */
TrajectoryTrackerConfig AtsSwerveMpcNode::loadTrackerConfig() {
  TrajectoryTrackerConfig config;
  config.terminal_position_tolerance = std::max(
      goal_position_tolerance_,
      declare_parameter<double>("terminal_yaw_handoff_tolerance", 0.045));
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
  config.stationary_yaw_tolerance = declare_parameter<double>(
      "stationary_yaw_tolerance", config.stationary_yaw_tolerance);
  config.intermediate_rotation_handoff_tolerance = declare_parameter<double>(
      "intermediate_rotation_handoff_tolerance",
      config.intermediate_rotation_handoff_tolerance);
  config.intermediate_rotation_stop_wz_threshold = declare_parameter<double>(
      "intermediate_rotation_stop_wz_threshold",
      config.intermediate_rotation_stop_wz_threshold);
  config.min_rotation_span_rad = declare_parameter<double>(
      "min_rotation_span_rad", config.min_rotation_span_rad);
  config.use_derivative_feedforward = declare_parameter<bool>(
      "use_derivative_feedforward", config.use_derivative_feedforward);
  const double default_lead_sec =
      controller_ ? controller_->config().actuator_lag_tau_sec : 0.0;
  config.feedforward_accel_lead_sec = declare_parameter<double>(
      "feedforward_accel_lead_sec", default_lead_sec);
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
  Control incoming_vel;
  incoming_vel(0) = message->twist.twist.linear.x;
  incoming_vel(1) = message->twist.twist.linear.y;
  incoming_vel(2) = message->twist.twist.angular.z;
  current_velocity_ = incoming_vel.allFinite() ? incoming_vel : Control::Zero();
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
bool AtsSwerveMpcNode::odometryUsable(State &current, Control *measured_velocity) {
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
  if (measured_velocity != nullptr) {
    *measured_velocity = current_velocity_;
  }
  return true;
}

/**
 * @brief 处理 legacy Path 兼容入口。
 * @details 一旦启用 ExecutionCommand，该回调只记录警告且不修改 tracker，防止独立 DDS
 *          topic 绕开原子执行授权；legacy 模式下才委托 installPath。
 */

} // namespace ats_swerve_mpc
