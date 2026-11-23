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
                "【参数错误】参数 '%s' 必须包含 3 个值。", name.c_str());
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
 * @brief 从预分配稀疏 CSC LTV-QP 中以 O(nnz) 提取有限尺度诊断指标。
 */
void recordQpProblemMetrics(const LtvQpProblem &problem,
                            ControlCycleTelemetrySample &telemetry) {
  const auto dims = checkedLtvQpDimensions(problem.horizon);
  if (!problem.valid || !dims.valid || !problem.hasExpectedLayout() ||
      !problem.hasFiniteNumerics()) {
    return;
  }
  telemetry.hessian_diagonal_minimum = std::numeric_limits<double>::infinity();
  telemetry.hessian_diagonal_maximum =
      -std::numeric_limits<double>::infinity();
  for (int col = 0; col < problem.decisionSize(); ++col) {
    const int end = problem.hessian_structure
                        .column_offsets[static_cast<std::size_t>(col + 1)];
    if (end > 0) {
      const double diag_val =
          problem.hessian_values[static_cast<std::size_t>(end - 1)];
      telemetry.hessian_diagonal_minimum =
          std::min(telemetry.hessian_diagonal_minimum, diag_val);
      telemetry.hessian_diagonal_maximum =
          std::max(telemetry.hessian_diagonal_maximum, diag_val);
    }
  }
  const int m = problem.constraintRows();
  std::vector<double> row_norm_sq(static_cast<std::size_t>(m), 0.0);
  for (std::size_t idx = 0; idx < problem.constraint_values.size(); ++idx) {
    const int row = problem.constraint_structure.row_indices[idx];
    const double val = problem.constraint_values[idx];
    row_norm_sq[static_cast<std::size_t>(row)] += val * val;
  }
  telemetry.constraint_row_l2_minimum = std::numeric_limits<double>::infinity();
  telemetry.constraint_row_l2_maximum = 0.0;
  const int check_rows = dims.equality_rows + dims.inequality_rows;
  for (int row = 0; row < check_rows; ++row) {
    const double norm = std::sqrt(row_norm_sq[static_cast<std::size_t>(row)]);
    telemetry.constraint_row_l2_minimum =
        std::min(telemetry.constraint_row_l2_minimum, norm);
    telemetry.constraint_row_l2_maximum =
        std::max(telemetry.constraint_row_l2_maximum, norm);
  }
  telemetry.nonzero_bound_abs_minimum = std::numeric_limits<double>::infinity();
  telemetry.bound_abs_maximum = 0.0;
  const auto update_bound_range = [&telemetry](const Eigen::VectorXd &bounds) {
    for (int index = 0; index < bounds.size(); ++index) {
      const double magnitude = std::abs(bounds(index));
      if (!std::isfinite(magnitude)) {
        continue;
      }
      telemetry.bound_abs_maximum =
          std::max(telemetry.bound_abs_maximum, magnitude);
      if (magnitude > 1e-12) {
        telemetry.nonzero_bound_abs_minimum =
            std::min(telemetry.nonzero_bound_abs_minimum, magnitude);
      }
    }
  };
  update_bound_range(problem.lower);
  update_bound_range(problem.upper);
  if (!std::isfinite(telemetry.nonzero_bound_abs_minimum)) {
    telemetry.nonzero_bound_abs_minimum = 0.0;
  }
  telemetry.zero_delta_dynamic_equality_residual =
      dims.equality_rows > 3
          ? problem.lower.segment(3, dims.equality_rows - 3)
                .cwiseAbs()
                .maxCoeff()
          : 0.0;
  telemetry.qp_problem_metrics_recorded =
      std::isfinite(telemetry.hessian_diagonal_minimum) &&
      std::isfinite(telemetry.hessian_diagonal_maximum) &&
      std::isfinite(telemetry.constraint_row_l2_minimum) &&
      std::isfinite(telemetry.constraint_row_l2_maximum) &&
      std::isfinite(telemetry.nonzero_bound_abs_minimum) &&
      std::isfinite(telemetry.bound_abs_maximum) &&
      std::isfinite(telemetry.zero_delta_dynamic_equality_residual);
}

bool validPlanningSnapshotPayload(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot) {
  const auto count =
      static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height;
  if (!snapshot.ready || snapshot.header.frame_id.empty() ||
      snapshot.source_generation == 0U ||
      snapshot.publication_sequence == 0U ||
      snapshot.occupied_value_threshold <= 0 ||
      snapshot.occupied_value_threshold > 100 ||
      !std::isfinite(snapshot.info.resolution) ||
      snapshot.info.resolution <= 0.0F || count == 0U ||
      snapshot.occupancy.size() != count ||
      snapshot.signed_distance_m.size() != count ||
      snapshot.gradient_x.size() != count ||
      snapshot.gradient_y.size() != count) {
    return false;
  }
  const auto &q = snapshot.info.origin.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto occ = snapshot.occupancy[i];
    const auto dist = snapshot.signed_distance_m[i];
    const auto gx = snapshot.gradient_x[i];
    const auto gy = snapshot.gradient_y[i];
    if (occ < -1 || occ > 100 || !std::isfinite(dist) || !std::isfinite(gx) ||
        !std::isfinite(gy)) {
      return false;
    }
    const bool blocked =
        occ >= snapshot.occupied_value_threshold ||
        (occ == -1 && snapshot.unknown_is_obstacle);
    if (blocked ? dist > 0.0F : dist < 0.0F) {
      return false;
    }
  }
  return true;
}

bool sampleSnapshotEsdfBilinear(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    double snap_x, double snap_y, double &out_distance_m,
    Eigen::Vector2d *out_gradient_snap) {
  const double res = static_cast<double>(snapshot.info.resolution);
  const auto &origin = snapshot.info.origin;
  const double oyaw = tf2::getYaw(origin.orientation);
  const double co = std::cos(oyaw);
  const double so = std::sin(oyaw);
  const double dx = snap_x - origin.position.x;
  const double dy = snap_y - origin.position.y;
  const double gx = (co * dx + so * dy) / res - 0.5;
  const double gy = (-so * dx + co * dy) / res - 0.5;
  const int w = static_cast<int>(snapshot.info.width);
  const int h = static_cast<int>(snapshot.info.height);
  if (gx < -0.5 || gy < -0.5 || gx > static_cast<double>(w) - 0.5 ||
      gy > static_cast<double>(h) - 0.5) {
    out_distance_m = -res;
    if (out_gradient_snap != nullptr) {
      out_gradient_snap->setZero();
    }
    return false;
  }
  const double clamped_x = std::clamp(gx, 0.0, static_cast<double>(w - 1));
  const double clamped_y = std::clamp(gy, 0.0, static_cast<double>(h - 1));
  const int x0 = std::min(w - 1, static_cast<int>(std::floor(clamped_x)));
  const int y0 = std::min(h - 1, static_cast<int>(std::floor(clamped_y)));
  const int x1 = std::min(w - 1, x0 + 1);
  const int y1 = std::min(h - 1, y0 + 1);
  const double tx = clamped_x - static_cast<double>(x0);
  const double ty = clamped_y - static_cast<double>(y0);

  const auto idx = [w](int cx, int cy) {
    return static_cast<std::size_t>(cy) * static_cast<std::size_t>(w) +
           static_cast<std::size_t>(cx);
  };
  const std::size_t i00 = idx(x0, y0);
  const std::size_t i10 = idx(x1, y0);
  const std::size_t i01 = idx(x0, y1);
  const std::size_t i11 = idx(x1, y1);
  const double d00 = snapshot.signed_distance_m[i00];
  const double d10 = snapshot.signed_distance_m[i10];
  const double d01 = snapshot.signed_distance_m[i01];
  const double d11 = snapshot.signed_distance_m[i11];
  out_distance_m = (1.0 - ty) * ((1.0 - tx) * d00 + tx * d10) +
                   ty * ((1.0 - tx) * d01 + tx * d11);
  if (out_gradient_snap != nullptr) {
    const double gx_val =
        (1.0 - ty) * ((1.0 - tx) * snapshot.gradient_x[i00] +
                      tx * snapshot.gradient_x[i10]) +
        ty * ((1.0 - tx) * snapshot.gradient_x[i01] +
              tx * snapshot.gradient_x[i11]);
    const double gy_val =
        (1.0 - ty) * ((1.0 - tx) * snapshot.gradient_y[i00] +
                      tx * snapshot.gradient_y[i10]) +
        ty * ((1.0 - tx) * snapshot.gradient_y[i01] +
              tx * snapshot.gradient_y[i11]);
    *out_gradient_snap = Eigen::Vector2d(gx_val, gy_val);
  }
  return true;
}

bool snapshotCenterCellFree(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    double snap_x, double snap_y) {
  const double res = static_cast<double>(snapshot.info.resolution);
  const auto &origin = snapshot.info.origin;
  const double oyaw = tf2::getYaw(origin.orientation);
  const double co = std::cos(oyaw);
  const double so = std::sin(oyaw);
  const double dx = snap_x - origin.position.x;
  const double dy = snap_y - origin.position.y;
  const double cx = (co * dx + so * dy) / res;
  const double cy = (-so * dx + co * dy) / res;
  if (cx < 0.0 || cy < 0.0 || cx >= snapshot.info.width ||
      cy >= snapshot.info.height) {
    return false;
  }
  const int ix = static_cast<int>(std::floor(cx));
  const int iy = static_cast<int>(std::floor(cy));
  const std::size_t index =
      static_cast<std::size_t>(iy) * snapshot.info.width +
      static_cast<std::size_t>(ix);
  const int occ = snapshot.occupancy[index];
  if (occ >= snapshot.occupied_value_threshold) {
    return false;
  }
  if (occ < 0 && snapshot.unknown_is_obstacle) {
    return false;
  }
  return true;
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
  config.terminal_position_tolerance = goal_position_tolerance_;
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
      "【参考丢弃】ExecutionCommand 已拥有 MPC 授权，忽略旧版 Path。");
    return;
  }
  installPath(*message);
}

/**
 * @brief 校验 frame/时间并把 Path 转为 tracker 的相对时间 reference。
 * @details 成功时设置 reference deadline；默认同时重置 iLQR warm start 与 last control，
 *          仅 keep_control_state=true（同一 goal/epoch/generation 内换轨）时保留二者。
 *          失败不覆盖原有安全状态，调用方负责进入 fail-stop。
 */
bool AtsSwerveMpcNode::installPath(
    const nav_msgs::msg::Path & message,
    bool keep_control_state,
    std::optional<CommandOrder> command,
    const ats_navigation_interfaces::msg::ExecutionCommand * execution_command) {
  if (require_localization_status_ && !localization_tracking_.load()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "【参考丢弃】定位未处于 TRACKING，忽略参考轨迹。");
    return false;
  }
  if (message.poses.size() < 2) {
    RCLCPP_WARN(get_logger(), "【参考丢弃】参考轨迹点数少于 2，已忽略。");
    return false;
  }
  if (!frame_id_.empty() && !message.header.frame_id.empty() &&
      message.header.frame_id != frame_id_) {
    RCLCPP_ERROR(get_logger(),
                 "【参考丢弃】轨迹坐标系为 '%s'，MPC 期望 '%s'。",
                 message.header.frame_id.c_str(), frame_id_.c_str());
    return false;
  }
  const std::size_t pose_count = message.poses.size();
  bool has_derivatives = false;
  if (execution_command != nullptr) {
    const bool any_derivatives =
      !execution_command->reference_vx.empty() ||
      !execution_command->reference_vy.empty() ||
      !execution_command->reference_ax.empty() ||
      !execution_command->reference_ay.empty() ||
      !execution_command->reference_yaw_rate.empty();
    if (any_derivatives) {
      if (execution_command->reference_vx.size() != pose_count ||
          execution_command->reference_vy.size() != pose_count ||
          execution_command->reference_ax.size() != pose_count ||
          execution_command->reference_ay.size() != pose_count ||
          execution_command->reference_yaw_rate.size() != pose_count) {
        RCLCPP_ERROR(get_logger(),
                     "【参考丢弃】ExecutionCommand 导数数组长度与 poses=%zu 不一致。",
                     pose_count);
        return false;
      }
      has_derivatives = true;
    }
  }
  std::vector<TimedState> parsed;
  parsed.reserve(pose_count);
  const double header_time = rclcpp::Time(message.header.stamp).seconds();
  bool monotonic = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < pose_count; ++i) {
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
    if (has_derivatives) {
      timed.world_velocity << execution_command->reference_vx[i],
          execution_command->reference_vy[i];
      timed.world_acceleration << execution_command->reference_ax[i],
          execution_command->reference_ay[i];
      timed.yaw_rate = execution_command->reference_yaw_rate[i];
      if (!timed.world_velocity.allFinite() ||
          !timed.world_acceleration.allFinite() ||
          !std::isfinite(timed.yaw_rate)) {
        RCLCPP_ERROR(get_logger(),
                     "【参考丢弃】ExecutionCommand 导数包含非有限值（i=%zu）。", i);
        return false;
      }
      timed.has_derivatives = true;
    }
    parsed.push_back(timed);
    previous_time = pose_time;
  }
  if (!monotonic) {
    const double start_time = header_time > 0.0 ? header_time : now().seconds();
    for (std::size_t i = 0; i < parsed.size(); ++i) {
      parsed[i].time = start_time + fallback_path_dt_ * static_cast<double>(i);
      parsed[i].has_derivatives = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool missing_stamp =
        message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0;
    // 序号在停车 STOP 之后的 EXECUTE 由 Goal Manager 在停车之后提交，其 reference 戳可与
    // 停车戳落在同一仿真时钟格（domain 213）。其它来源的停车没有序号可比，相等仍拒绝。
    const bool ordered_after_stop = command && last_stop_command_ &&
      command->incarnation == last_stop_command_->incarnation &&
      command->sequence > last_stop_command_->sequence;
    const rclcpp::Time reference_stamp(message.header.stamp);
    if (last_stop_stamp_.nanoseconds() > 0 &&
        (missing_stamp ||
         reference_stamp < last_stop_stamp_ ||
         (reference_stamp == last_stop_stamp_ && !ordered_after_stop))) {
      RCLCPP_WARN(
          get_logger(),
          "【参考丢弃】轨迹时间早于最近一次急停，拒绝旧轨迹。");
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
  // 任何新轨迹切换均重置 OSQP 对偶热启动，避免旧参考的 active-set 乘子污染新轨迹求解。
  qp_warm_start_valid_ = false;
  if (qp_solver_) {
    qp_solver_->resetWarmStart();
  }
  if (!keep_control_state) {
    controller_->reset();
    {
      std::lock_guard<std::mutex> history_lock(command_history_mutex_);
      command_delay_history_.clear();
    }
    last_control_.setZero();
    last_control_update_stamp_.reset();
    last_control_update_steady_.reset();
  }
  // 保留时不调用 reset()：iLQR 的 warm_controls_ 留在原处，下一拍 initializeControls 照常
  // 左移一格并与新 reference 的前馈 0.7/0.3 混合；last_control_ 保持上一拍已发布的指令。
  return true;
}

/**
 * @brief 接收唯一的执行授权和 reference 原子快照。
 * @details 严格拒绝 incarnation/sequence 倒退、gimbal 或 localization epoch 不匹配；STOP
 *          与任何非法 EXECUTE 都走既有急停，而不是只停止 QP shadow。
 */
void AtsSwerveMpcNode::onExecutionCommand(
    const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr message) {
  const rclcpp::Time receipt = now();
  // rclcpp::Time throws on a negative sec; a malformed stamp must take the same
  // fail-closed rejection path as an expired one instead of aborting the node.
  const bool stamp_valid = message && message->header.stamp.sec >= 0 &&
    message->header.stamp.nanosec < 1000000000U;
  const rclcpp::Time command_stamp = stamp_valid ?
    rclcpp::Time(message->header.stamp, receipt.get_clock_type()) : receipt;
  if (!message || message->manager_incarnation == 0 ||
      message->command_sequence == 0) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=invalid receipt_ns=%lld",
        static_cast<long long>(receipt.nanoseconds()));
    engageFailStop();
    return;
  }
  // The command lease is measured from the producer timestamp as well as DDS
  // receipt time.  A transient-local sample can be delivered immediately
  // after startup even though it was emitted by an older process; accepting it
  // here would let a stale/future authorization install a reference before the
  // arbiter sees it.  Keep the MPC gate identical to the arbiter gate so either
  // consumer fails closed on the same clock contract.
  const rclcpp::Duration command_age = receipt - command_stamp;
  const std::int64_t min_allowed_age_ns =
      -rclcpp::Duration::from_seconds(execution_command_future_tolerance_sec_)
           .nanoseconds();
  if (!stamp_valid || command_age.nanoseconds() < min_allowed_age_ns ||
      command_age > rclcpp::Duration::from_seconds(execution_command_timeout_)) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=timestamp mode=%u incarnation=%llu "
        "sequence=%llu stamp_ns=%lld receipt_ns=%lld age_ns=%lld limit_ns=%lld",
        static_cast<unsigned>(message->mode),
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<long long>(command_age.nanoseconds()),
        static_cast<long long>(
            rclcpp::Duration::from_seconds(execution_command_timeout_).nanoseconds()));
    engageFailStop();
    return;
  }
  const std::int64_t effective_age_ns =
      std::max<std::int64_t>(0, command_age.nanoseconds());
  bool install_reference = false;
  bool keep_control_state = false;
  bool stop = message->mode ==
    ats_navigation_interfaces::msg::ExecutionCommand::MODE_STOP;
  {
    std::unique_lock<std::mutex> lock(trajectory_mutex_);
    if (message->manager_incarnation < last_execution_command_incarnation_) {
      RCLCPP_WARN(
          get_logger(),
          "【执行授权拒绝】TRACE execution_command rejected=old_incarnation incarnation=%llu sequence=%llu "
          "stamp_ns=%lld receipt_ns=%lld active_incarnation=%llu",
          static_cast<unsigned long long>(message->manager_incarnation),
          static_cast<unsigned long long>(message->command_sequence),
          static_cast<long long>(command_stamp.nanoseconds()),
          static_cast<long long>(receipt.nanoseconds()),
          static_cast<unsigned long long>(last_execution_command_incarnation_));
      active_execution_command_.reset();
      lock.unlock();
      engageFailStop();
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
      RCLCPP_WARN(
          get_logger(),
          "【执行授权拒绝】TRACE execution_command rejected=replay incarnation=%llu sequence=%llu "
          "stamp_ns=%lld receipt_ns=%lld last_sequence=%llu",
          static_cast<unsigned long long>(message->manager_incarnation),
          static_cast<unsigned long long>(message->command_sequence),
          static_cast<long long>(command_stamp.nanoseconds()),
          static_cast<long long>(receipt.nanoseconds()),
          static_cast<unsigned long long>(last_execution_command_sequence_));
      active_execution_command_.reset();
      lock.unlock();
      engageFailStop();
      return;
    } else {
      last_execution_command_sequence_ = message->command_sequence;
    }
    last_execution_command_signal_ = std::chrono::steady_clock::now() -
      std::chrono::nanoseconds(effective_age_ns);
    const bool grace_active = supersedeGraceActiveLocked(std::chrono::steady_clock::now());
    if (!stop && grace_active &&
        message->mode == ats_navigation_interfaces::msg::ExecutionCommand::MODE_EXECUTE &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        message->map_generation == supersede_old_generation_) {
      // 换代宽限内 Goal Manager 仍按旧 generation 心跳：不安装、不停车，旧 reference
      // 继续执行到宽限截止或新 generation 的 EXECUTE 到达（报告 5.1.9 D）。
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "【换代宽限】旧 generation %llu 的心跳不安装，等待 generation %llu 的执行授权。",
          static_cast<unsigned long long>(message->map_generation),
          static_cast<unsigned long long>(planner_generation_));
      return;
    }
    if (stop) {
      active_execution_command_.reset();
    } else if (message->mode !=
        ats_navigation_interfaces::msg::ExecutionCommand::MODE_EXECUTE ||
      message->goal_id == 0 || message->reference.poses.size() < 2 ||
      (require_gimbal_status_ && message->gimbal_request_sequence == 0) ||
      (message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_GIMBAL_COMPENSATED &&
       message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_BODY_YAW_FOLLOW &&
       message->yaw_authority !=
        ats_navigation_interfaces::msg::ExecutionCommand::YAW_AUTHORITY_SPIN) ||
      !gimbalExecutionValidLocked(*message) ||
      !mapExecutionValidLocked(*message) ||
      (require_localization_status_ &&
       (!localization_tracking_.load() || !localization_epoch_ ||
        *localization_epoch_ != message->localization_epoch))) {
      stop = true;
      active_execution_command_.reset();
    } else {
      const bool same_goal_epoch_generation = active_execution_command_ &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        active_execution_command_->map_generation == message->map_generation;
      // 换代宽限内到达的新 generation：同一 goal 与 epoch，车还在旧 reference 上按巡航
      // 速度跑，和同 generation 换轨一样沿用 last_control_ 与 warm start。
      const bool successor_generation = grace_active &&
        active_execution_command_->goal_id == message->goal_id &&
        active_execution_command_->localization_epoch == message->localization_epoch &&
        message->map_generation > active_execution_command_->map_generation;
      const bool same_reference = same_goal_epoch_generation &&
        active_execution_command_->map_publication_sequence ==
          message->map_publication_sequence &&
        active_execution_command_->reference.header.stamp.sec ==
          message->reference.header.stamp.sec &&
        active_execution_command_->reference.header.stamp.nanosec ==
          message->reference.header.stamp.nanosec;
      if (!same_reference) {
        install_reference = true;
        // 同一 goal、同一定位 epoch、同一 planner generation 内的换轨保留 last_control_ 与
        // iLQR warm start（报告 9.4 第 3 项）：reference 变了，但车的速度没变，从零起步会让
        // max_a*dt 增量限幅把换轨第一拍压到约 0.1 m/s，等同一次急刹。其余情形照旧清零：
        // 首次安装（无 active command）、STOP/急停/FAILED/MAP_UNREADY 之后（这些路径都已清掉
        // active command 或置 fail_stop_engaged_）、generation/epoch/goal 变化。
        keep_control_state = (same_goal_epoch_generation || successor_generation) &&
          !fail_stop_engaged_.load();
      }
      active_execution_command_ = *message;
      supersede_deadline_.reset();
    }
  }
  if (stop) {
    RCLCPP_WARN(
        get_logger(),
        "【执行授权停止】TRACE execution_command accepted=0 stop=1 mode=%u incarnation=%llu sequence=%llu "
        "stamp_ns=%lld receipt_ns=%lld goal=%llu map_generation=%llu map_sequence=%llu",
        static_cast<unsigned>(message->mode),
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<unsigned long long>(message->goal_id),
        static_cast<unsigned long long>(message->map_generation),
        static_cast<unsigned long long>(message->map_publication_sequence));
    std::optional<CommandOrder> stop_command;
    if (message->mode == ats_navigation_interfaces::msg::ExecutionCommand::MODE_STOP) {
      stop_command = CommandOrder{message->manager_incarnation, message->command_sequence};
    }
    engageFailStop(true, stop_command);
    return;
  }
  if (install_reference &&
      !installPath(message->reference, keep_control_state,
                   CommandOrder{message->manager_incarnation, message->command_sequence},
                   message.get())) {
    RCLCPP_ERROR(
        get_logger(),
        "【执行授权拒绝】TRACE execution_command rejected=reference_install incarnation=%llu sequence=%llu "
        "stamp_ns=%lld receipt_ns=%lld goal=%llu poses=%zu",
        static_cast<unsigned long long>(message->manager_incarnation),
        static_cast<unsigned long long>(message->command_sequence),
        static_cast<long long>(command_stamp.nanoseconds()),
        static_cast<long long>(receipt.nanoseconds()),
        static_cast<unsigned long long>(message->goal_id), message->reference.poses.size());
    engageFailStop();
    return;
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "【执行授权接受】TRACE execution_command accepted=1 incarnation=%llu sequence=%llu "
      "stamp_ns=%lld receipt_ns=%lld goal=%llu epoch=%llu map_generation=%llu "
      "map_sequence=%llu local_generation_authorized=1 content_lineage_pending=1 poses=%zu",
      static_cast<unsigned long long>(message->manager_incarnation),
      static_cast<unsigned long long>(message->command_sequence),
      static_cast<long long>(command_stamp.nanoseconds()),
      static_cast<long long>(receipt.nanoseconds()),
      static_cast<unsigned long long>(message->goal_id),
      static_cast<unsigned long long>(message->localization_epoch),
      static_cast<unsigned long long>(message->map_generation),
      static_cast<unsigned long long>(message->map_publication_sequence),
      message->reference.poses.size());
  fail_stop_engaged_.store(false);
}

// PlannerStatus proves only MINCO's local immutable map identity. It is not
// adapter source-generation or atomic publication/content lineage evidence.
void AtsSwerveMpcNode::onPlannerStatus(
    const ats_navigation_interfaces::msg::PlannerStatus::SharedPtr message) {
  using Status = ats_navigation_interfaces::msg::PlannerStatus;
  if (!message || message->state == Status::STATE_ACCEPTED ||
      message->localization_epoch == 0 || message->map_generation == 0) {
    return;  // Scheduling telemetry cannot establish or erase map authority.
  }
  if (message->header.frame_id.empty() || message->header.stamp.sec < 0 ||
      message->header.stamp.nanosec >= 1000000000u) {
    return;
  }
  const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
  if (stamp_ns <= 0) {
    return;
  }
  bool stop = false;
  bool superseded_only = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (message->localization_epoch < planner_epoch_ ||
        (message->localization_epoch == planner_epoch_ &&
         message->map_generation < planner_generation_)) {
      return;
    }
    const bool changed_epoch = message->localization_epoch > planner_epoch_;
    const bool changed_identity = changed_epoch ||
      message->map_generation > planner_generation_;
    if (!changed_identity && stamp_ns <= planner_status_stamp_ns_) {
      return;
    }
    if (changed_epoch) {
      retired_planner_generation_ = 0;
    }
    if (changed_identity) {
      planner_generation_usable_ = false;
      stop = true;
    }
    planner_epoch_ = message->localization_epoch;
    planner_generation_ = message->map_generation;
    planner_status_stamp_ns_ = stamp_ns;
    if (message->state == Status::STATE_FAILED ||
        message->failure_reason != Status::FAILURE_NONE) {
      planner_generation_usable_ = false;
      stop = true;
      if (message->failure_reason == Status::FAILURE_MAP_UNREADY) {
        retired_planner_generation_ = std::max(
          retired_planner_generation_, planner_generation_);
      }
    } else if (message->state == Status::STATE_REFERENCE_READY) {
      planner_generation_usable_ = planner_generation_ > retired_planner_generation_;
      if (planner_generation_usable_ && mapReadyFreshLocked()) {
        ready_lease_generation_ = planner_generation_;
      }
      // 同一 epoch 内更新的 generation 就绪：旧命令照常作废并停车，但这不是危险停车，
      // 不推进 last_stop_stamp_。旧 reference 不可能复活：EXECUTE 必须带新 generation
      // （mapExecutionValidLocked），而新 generation 的命令只能在本状态之后由 Goal
      // Manager 提交。domain 112：替换 EXECUTE 的 reference 戳 44.4225 s 早于这里的
      // 停车戳，被 installPath 判为"早于最近一次急停"，无停车重规划因此每次都停住。
      // legacy Path 模式没有 generation 绑定，仍按原规则推进停车戳。
      superseded_only = execution_command_enabled_ && changed_identity && !changed_epoch;
      // 换代宽限（报告 5.1.9 D）：旧命令不立即作废，最多再执行 supersede_grace_sec，
      // 等新 generation 的 EXECUTE。新 generation 已标为可用，旧 generation 的 EXECUTE
      // 仍不能安装；宽限截止时控制周期按 superseded 停车。宽限已开始时不延长。
      if (superseded_only && supersede_grace_sec_ > 0.0 && active_execution_command_ &&
          !fail_stop_engaged_.load() &&
          active_execution_command_->localization_epoch == message->localization_epoch) {
        if (!supersede_deadline_) {
          supersede_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(supersede_grace_sec_));
          supersede_old_generation_ = active_execution_command_->map_generation;
        }
        stop = false;
      }
    } else {
      planner_generation_usable_ = false;
      stop = true;
    }
    if (stop) {
      active_execution_command_.reset();
      supersede_deadline_.reset();
    }
  }
  if (stop) {
    engageFailStop(!superseded_only);
  }
}

bool AtsSwerveMpcNode::mapReadyFreshLocked() const {
  if (!map_ready_ || !last_map_ready_signal_) {
    return false;
  }
  const auto age = std::chrono::steady_clock::now() - *last_map_ready_signal_;
  return age >= std::chrono::steady_clock::duration::zero() &&
    std::chrono::duration<double>(age).count() <= map_ready_timeout_sec_;
}

bool AtsSwerveMpcNode::supersedeGraceActiveLocked(
    std::chrono::steady_clock::time_point now) const {
  return supersede_deadline_ && now <= *supersede_deadline_ &&
    active_execution_command_ && !fail_stop_engaged_.load();
}

bool AtsSwerveMpcNode::activeExecutionValidLocked(
    std::chrono::steady_clock::time_point now) const {
  if (!active_execution_command_) {
    return false;
  }
  const auto & command = *active_execution_command_;
  if (mapExecutionValidLocked(command)) {
    return true;
  }
  // 宽限内旧命令仍可执行：地图心跳新鲜、新 generation 已就绪，旧 generation 没被退役。
  return supersedeGraceActiveLocked(now) && mapReadyFreshLocked() &&
    planner_generation_usable_ && command.localization_epoch == planner_epoch_ &&
    command.map_generation == supersede_old_generation_ &&
    command.map_generation > retired_planner_generation_;
}

bool AtsSwerveMpcNode::mapExecutionValidLocked(
    const ats_navigation_interfaces::msg::ExecutionCommand & command) const {
  return mapReadyFreshLocked() && planner_generation_usable_ &&
    command.localization_epoch != 0 && command.localization_epoch == planner_epoch_ &&
    command.map_generation != 0 && command.map_generation == planner_generation_ &&
    command.map_generation > retired_planner_generation_;
}

void AtsSwerveMpcNode::onMapReady(const std_msgs::msg::Bool::SharedPtr message) {
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool expired =
      map_ready_ && last_map_ready_signal_ && !mapReadyFreshLocked();
    stop = !message->data || expired;
    if (stop) {
      // Retire only the identity covered by the expired/false lease. A newer
      // generation published while unready must remain eligible after a fresh
      // REFERENCE_READY + EXECUTE; the recovery heartbeat must not raise the
      // floor past it.
      const std::uint64_t retire_floor =
        (expired && ready_lease_generation_ != 0)
          ? ready_lease_generation_
          : planner_generation_;
      retired_planner_generation_ = std::max(
        retired_planner_generation_, retire_floor);
      planner_generation_usable_ = false;
      active_execution_command_.reset();
      supersede_deadline_.reset();
    }
    map_ready_ = message->data;
    last_map_ready_signal_ = std::chrono::steady_clock::now();
    if (message->data) {
      ready_lease_generation_ = planner_generation_;
    }
  }
  if (stop) {
    engageFailStop();
  }
}

void AtsSwerveMpcNode::onPlanningSnapshot(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot::SharedPtr
        message) {
  const bool payload_valid =
      message != nullptr && validPlanningSnapshotPayload(*message);
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!payload_valid) {
      planning_snapshot_.reset();
      planning_snapshot_ready_ = false;
      stop = require_planning_snapshot_ && active_execution_command_.has_value();
    } else {
      if (planning_snapshot_ &&
          planning_snapshot_->localization_epoch == message->localization_epoch &&
          message->publication_sequence <
              planning_snapshot_->publication_sequence) {
        return;
      }
      planning_snapshot_ = message;
      planning_snapshot_ready_ = true;
      last_planning_snapshot_signal_ = std::chrono::steady_clock::now();
    }
  }
  if (stop) {
    engageFailStop();
  }
}

bool AtsSwerveMpcNode::planningSnapshotUsableLocked(
    std::chrono::steady_clock::time_point now_steady,
    std::shared_ptr<const ats_navigation_interfaces::msg::PlanningMapSnapshot>
        *out_snapshot,
    Eigen::Vector3d *out_snapshot_from_odom) const {
  if (out_snapshot != nullptr) {
    out_snapshot->reset();
  }
  if (out_snapshot_from_odom != nullptr) {
    out_snapshot_from_odom->setZero();
  }
  if (!planning_snapshot_ || !planning_snapshot_ready_ ||
      !last_planning_snapshot_signal_ ||
      now_steady < *last_planning_snapshot_signal_) {
    return !require_planning_snapshot_;
  }
  const double age = std::chrono::duration<double>(
                         now_steady - *last_planning_snapshot_signal_)
                         .count();
  if (age > planning_snapshot_timeout_sec_) {
    return !require_planning_snapshot_;
  }
  if (active_execution_command_) {
    if (planning_snapshot_->localization_epoch !=
        active_execution_command_->localization_epoch) {
      return !require_planning_snapshot_;
    }
  } else if (require_localization_status_ && localization_epoch_) {
    if (planning_snapshot_->localization_epoch != *localization_epoch_) {
      return !require_planning_snapshot_;
    }
  }

  Eigen::Vector3d snap_from_odom = Eigen::Vector3d::Zero();
  const std::string &snap_frame = planning_snapshot_->header.frame_id;
  const std::string &ref_frame =
      trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
  if (snap_frame != ref_frame) {
    if (!active_execution_command_) {
      return !require_planning_snapshot_;
    }
    const auto &tf_msg = active_execution_command_->validation_from_reference;
    if (tf_msg.header.frame_id != snap_frame ||
        (!tf_msg.child_frame_id.empty() && tf_msg.child_frame_id != ref_frame)) {
      return !require_planning_snapshot_;
    }
    const auto &q = tf_msg.transform.rotation;
    const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(q_norm) || std::abs(q_norm - 1.0) > 1e-2 ||
        !std::isfinite(tf_msg.transform.translation.x) ||
        !std::isfinite(tf_msg.transform.translation.y)) {
      return !require_planning_snapshot_;
    }
    snap_from_odom(0) = tf_msg.transform.translation.x;
    snap_from_odom(1) = tf_msg.transform.translation.y;
    snap_from_odom(2) = tf2::getYaw(q);
  }

  if (out_snapshot != nullptr) {
    *out_snapshot = planning_snapshot_;
  }
  if (out_snapshot_from_odom != nullptr) {
    *out_snapshot_from_odom = snap_from_odom;
  }
  return true;
}

std::vector<EsdfCorridorSample> AtsSwerveMpcNode::extractEsdfCorridorSamples(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    const std::vector<State> &nominal_states,
    const std::vector<Se2Reference> &references) const {
  const int horizon = controller_->config().horizon;
  std::vector<EsdfCorridorSample> samples(static_cast<std::size_t>(horizon));
  if (!qp_esdf_config_.enabled ||
      nominal_states.size() < static_cast<std::size_t>(horizon + 1) ||
      references.size() < static_cast<std::size_t>(horizon + 1)) {
    return samples;
  }
  const double ct = std::cos(snapshot_from_odom(2));
  const double st = std::sin(snapshot_from_odom(2));
  for (int step = 0; step < horizon; ++step) {
    const std::size_t k = static_cast<std::size_t>(step + 1);
    const State &nom = nominal_states[k];
    const State &ref = references[k].state;
    const double nom_sx = ct * nom(0) - st * nom(1) + snapshot_from_odom(0);
    const double nom_sy = st * nom(0) + ct * nom(1) + snapshot_from_odom(1);
    double d_nom = 0.0;
    Eigen::Vector2d grad_snap = Eigen::Vector2d::Zero();
    if (!sampleSnapshotEsdfBilinear(snapshot, nom_sx, nom_sy, d_nom,
                                    &grad_snap)) {
      continue;
    }
    const double ref_sx = ct * ref(0) - st * ref(1) + snapshot_from_odom(0);
    const double ref_sy = st * ref(0) + ct * ref(1) + snapshot_from_odom(1);
    double d_ref = qp_esdf_config_.corridor_clearance_m;
    sampleSnapshotEsdfBilinear(snapshot, ref_sx, ref_sy, d_ref, nullptr);
    const double target = std::clamp(
        std::min(qp_esdf_config_.corridor_clearance_m,
                 d_ref - qp_esdf_config_.reference_margin_m),
        qp_esdf_config_.min_corridor_clearance_m,
        qp_esdf_config_.corridor_clearance_m);
    if (d_nom > target + 0.60 || grad_snap.norm() <= 1e-3) {
      continue;
    }
    auto &sample = samples[static_cast<std::size_t>(step)];
    sample.active = true;
    sample.signed_distance_m = d_nom;
    sample.target_clearance_m = target;
    sample.gradient_odom(0) = ct * grad_snap(0) + st * grad_snap(1);
    sample.gradient_odom(1) = -st * grad_snap(0) + ct * grad_snap(1);
  }
  return samples;
}

AtsSwerveMpcNode::RolloutSafetyEvaluation
AtsSwerveMpcNode::evaluateRolloutFootprintSafety(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    const std::vector<State> &rollout_states) const {
  RolloutSafetyEvaluation eval;
  if (!rollout_safety_enabled_ || rollout_states.size() < 2U) {
    return eval;
  }
  const double res = static_cast<double>(snapshot.info.resolution);
  const double hx = 0.5 * footprint_length_ + footprint_safety_margin_;
  const double hy = 0.5 * footprint_width_ + footprint_safety_margin_;
  const double r_circum = std::hypot(hx, hy);
  const double ct = std::cos(snapshot_from_odom(2));
  const double st = std::sin(snapshot_from_odom(2));
  const double dt = controller_->config().dt;

  const auto evalPoseClearance = [&](const State &state_odom,
                                     bool &center_free) -> double {
    const double sx =
        ct * state_odom(0) - st * state_odom(1) + snapshot_from_odom(0);
    const double sy =
        st * state_odom(0) + ct * state_odom(1) + snapshot_from_odom(1);
    const double syaw = state_odom(2) + snapshot_from_odom(2);
    center_free = snapshotCenterCellFree(snapshot, sx, sy);
    double d_center = 0.0;
    sampleSnapshotEsdfBilinear(snapshot, sx, sy, d_center, nullptr);
    const double lb = d_center - r_circum - 0.5 * res;
    if (center_free && lb >= rollout_slowdown_clearance_m_ + 0.01) {
      return lb;
    }
    const double cy = std::cos(syaw);
    const double sy_s = std::sin(syaw);
    double min_d = std::numeric_limits<double>::infinity();
    constexpr int kEdgeSamples = 3;
    for (int side = -1; side <= 1; side += 2) {
      for (int i = 0; i <= kEdgeSamples; ++i) {
        const double u = -1.0 + 2.0 * static_cast<double>(i) /
                                    static_cast<double>(kEdgeSamples);
        const std::array<Eigen::Vector2d, 2> pts{{
            Eigen::Vector2d(side * hx, u * hy),
            Eigen::Vector2d(u * hx, side * hy)}};
        for (const auto &local_p : pts) {
          const double px = sx + cy * local_p(0) - sy_s * local_p(1);
          const double py = sy + sy_s * local_p(0) + cy * local_p(1);
          double dp = 0.0;
          sampleSnapshotEsdfBilinear(snapshot, px, py, dp, nullptr);
          min_d = std::min(min_d, dp - 0.5 * res);
        }
      }
    }
    return min_d;
  };

  bool init_center_free = true;
  const double init_clearance =
      evalPoseClearance(rollout_states.front(), init_center_free);
  const int max_steps = std::min(
      static_cast<int>(rollout_states.size()) - 1,
      std::max(
          1, static_cast<int>(std::ceil(rollout_slowdown_horizon_sec_ /
                                        std::max(1e-3, dt)))));

  for (int step = 1; step <= max_steps; ++step) {
    const double t_sec = static_cast<double>(step) * dt;
    bool center_free = true;
    const double clearance = evalPoseClearance(
        rollout_states[static_cast<std::size_t>(step)], center_free);
    eval.min_footprint_clearance_m =
        std::min(eval.min_footprint_clearance_m, clearance);
    const bool worsening = clearance < init_clearance - 0.005;

    if (t_sec <= rollout_immediate_stop_horizon_sec_ + 1e-6) {
      if (!center_free ||
          (clearance < -rollout_hard_stop_penetration_m_ && worsening)) {
        eval.level = RolloutSafetyLevel::kUnsafeStop;
        eval.speed_scale = 0.0;
        return eval;
      }
    }
    if (t_sec <= rollout_brake_horizon_sec_ + 1e-6 && clearance < -0.005 &&
        (init_clearance >= -0.005 || worsening)) {
      eval.level = RolloutSafetyLevel::kBrake;
      const double ratio = std::clamp(
          t_sec / std::max(0.1, rollout_brake_horizon_sec_), 0.20, 0.60);
      eval.speed_scale = std::min(eval.speed_scale, ratio);
    } else if (clearance < rollout_slowdown_clearance_m_ &&
               eval.level != RolloutSafetyLevel::kBrake) {
      eval.level = RolloutSafetyLevel::kSlowdown;
      const double ratio =
          rollout_slowdown_clearance_m_ > 1e-6
              ? std::clamp(clearance / rollout_slowdown_clearance_m_, 0.0, 1.0)
              : 1.0;
      const double scale =
          rollout_min_slowdown_scale_ +
          (1.0 - rollout_min_slowdown_scale_) * ratio;
      eval.speed_scale = std::min(eval.speed_scale, scale);
    }
  }
  return eval;
}

State AtsSwerveMpcNode::advanceStateThroughCommandDelay(
    const State &measured_state, double now_sec, double rotation_delay) const {
  const double delay_sec = controller_->config().actuator_delay_sec;
  if (!std::isfinite(delay_sec) || delay_sec <= 1e-4) {
    return rotationPhaseCompensatedState(measured_state, last_control_(2),
                                         rotation_delay);
  }
  std::deque<CommandHistorySample> history_copy;
  {
    std::lock_guard<std::mutex> lock(command_history_mutex_);
    history_copy = command_delay_history_;
  }
  if (history_copy.empty()) {
    return Se2Model::dynamics(measured_state, last_control_, delay_sec);
  }
  const double window_start = now_sec - delay_sec;
  State predicted = measured_state;
  double cursor = window_start;
  Control active_u = history_copy.front().control;
  for (const auto &entry : history_copy) {
    if (entry.stamp_sec <= window_start) {
      active_u = entry.control;
      continue;
    }
    if (entry.stamp_sec > now_sec) {
      break;
    }
    const double seg_dt = entry.stamp_sec - cursor;
    if (seg_dt > 1e-6) {
      predicted = Se2Model::dynamics(predicted, active_u, seg_dt);
      cursor = entry.stamp_sec;
    }
    active_u = entry.control;
  }
  if (now_sec > cursor + 1e-6) {
    predicted = Se2Model::dynamics(predicted, active_u, now_sec - cursor);
  }
  return predicted;
}

bool AtsSwerveMpcNode::solveSqpRti(
    const ControlCycleSnapshot &snapshot,
    const ats_navigation_interfaces::msg::PlanningMapSnapshot *map_snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    ControlCycleTelemetrySample &telemetry, Se2MpcResult &out_result,
    RolloutSafetyEvaluation &out_rollout_safety) {
  const auto sqp_start = std::chrono::steady_clock::now();
  if (!qp_solver_ || !qp_solver_->initialized()) {
    telemetry.status = LtvQpSolverStatus::kBackendUnavailable;
    std::snprintf(telemetry.rejection_reason.data(),
                  telemetry.rejection_reason.size(), "%s",
                  "backend_unavailable");
    return false;
  }

  bool first_body_sat = false;
  bool first_wheel_sat = false;
  bool first_inc_lim = false;
  std::vector<Control> nominal_controls = controller_->buildNominalControls(
      snapshot.references, snapshot.last_control_before_solve,
      snapshot.first_step_dt, true, &first_body_sat, &first_wheel_sat,
      &first_inc_lim);
  std::vector<State> nominal_states = controller_->rolloutTrajectory(
      snapshot.current_state, nominal_controls,
      snapshot.last_control_before_solve, snapshot.first_step_dt);

  LtvQpSolveResult qp_res;
  LtvQpPrimalCandidate candidate;
  LtvQpCandidateAudit audit;
  RolloutSafetyEvaluation rollout_safety;
  double total_build_ms = 0.0;
  double total_update_ms = 0.0;
  double total_backend_ms = 0.0;
  double total_complete_ms = 0.0;
  double total_solve_ms = 0.0;
  double total_recon_ms = 0.0;
  double total_audit_ms = 0.0;
  int executed_rti = 0;

  for (int rti_pass = 0; rti_pass < qp_rti_iterations_; ++rti_pass) {
    const auto build_start = std::chrono::steady_clock::now();
    std::vector<EsdfCorridorSample> esdf_samples;
    if (map_snapshot != nullptr && qp_esdf_config_.enabled) {
      esdf_samples = extractEsdfCorridorSamples(
          *map_snapshot, snapshot_from_odom, nominal_states,
          snapshot.references);
    }
    const bool built = LtvQpBuilder::build(
        snapshot.current_state, nominal_states, nominal_controls,
        snapshot.references, snapshot.last_control_before_solve,
        controller_->config(), qp_problem_buffer_, ZeroSpeedGuardConfig(),
        snapshot.first_step_dt, qp_esdf_config_,
        esdf_samples.empty() ? nullptr : &esdf_samples);
    total_build_ms += elapsedMilliseconds(build_start,
                                          std::chrono::steady_clock::now());
    if (!built) {
      qp_res.status = LtvQpSolverStatus::kInvalidProblem;
      break;
    }
    if (rti_pass == 0) {
      recordQpProblemMetrics(qp_problem_buffer_, telemetry);
    }

    LtvQpWarmStart rti_warm;
    const LtvQpWarmStart *warm_ptr = nullptr;
    if (qp_warm_start_valid_ &&
        qp_warm_start_.dual.size() == qp_problem_buffer_.constraintRows()) {
      const int horizon = qp_problem_buffer_.horizon;
      const auto dims = checkedLtvQpDimensions(horizon);
      rti_warm.primal = Eigen::VectorXd::Zero(qp_problem_buffer_.decisionSize());
      const int esdf_base = dims.equality_rows + dims.body_accel_rows +
                            dims.wheel_octagon_rows + dims.wheel_rate_rows;
      for (int s = 0; s < horizon; ++s) {
        rti_warm.primal(qp_problem_buffer_.slackOffset(s)) =
            std::max(0.0, qp_problem_buffer_.lower(esdf_base + s));
      }
      if (rti_pass == 0 && horizon > 1) {
        // Fractionally shift dual blocks forward by first_step_dt / dt to match
        // the receding horizon window at 50 Hz (consistent with initializeControls).
        const double shift = std::clamp(
            snapshot.first_step_dt / std::max(1e-6, controller_->config().dt),
            0.0, 1.0);
        rti_warm.dual = Eigen::VectorXd::Zero(dims.constraint_rows);
        const auto shiftBlock = [&](int base, int stride, int count) {
          for (int k = 0; k + 1 < count; ++k) {
            rti_warm.dual.segment(base + k * stride, stride) =
                (1.0 - shift) * qp_warm_start_.dual.segment(base + k * stride, stride) +
                shift * qp_warm_start_.dual.segment(base + (k + 1) * stride, stride);
          }
          rti_warm.dual.segment(base + (count - 1) * stride, stride) =
              qp_warm_start_.dual.segment(base + (count - 1) * stride, stride);
        };
        rti_warm.dual.segment<3>(0) = qp_warm_start_.dual.segment<3>(0);
        shiftBlock(3, 3, horizon);
        int cursor = dims.equality_rows;
        shiftBlock(cursor, 3, horizon);
        cursor += dims.body_accel_rows;
        shiftBlock(cursor, 8, horizon);
        cursor += dims.wheel_octagon_rows + dims.wheel_rate_rows;
        shiftBlock(cursor, 1, horizon);
        cursor += dims.esdf_corridor_rows;
        shiftBlock(cursor, 3, horizon);
        cursor += 3 * horizon;
        shiftBlock(cursor, 1, horizon);
      } else {
        rti_warm.dual = qp_warm_start_.dual;
      }
      for (int s = 0; s < horizon; ++s) {
        if (qp_problem_buffer_.lower(esdf_base + s) < -0.5) {
          rti_warm.dual(esdf_base + s) = 0.0;
        }
      }
      warm_ptr = &rti_warm;
    }

    qp_res = qp_solver_->solveLtvProblem(qp_problem_buffer_,
                                         qp_solver_settings_, warm_ptr);
    ++executed_rti;
    total_update_ms += qp_res.wall_update_time_ms;
    total_backend_ms += qp_res.wall_qp_phase_time_ms;
    total_complete_ms += qp_res.wall_complete_qp_phase_time_ms;
    total_solve_ms += qp_res.wall_solve_time_ms;

    if ((qp_res.status != LtvQpSolverStatus::kSolved &&
         qp_res.status != LtvQpSolverStatus::kSolvedInaccurate) ||
        qp_res.primal_solution.size() != qp_problem_buffer_.decisionSize() ||
        qp_res.dual_solution.size() != qp_problem_buffer_.constraintRows()) {
      break;
    }
    qp_warm_start_.primal = qp_res.primal_solution;
    qp_warm_start_.dual = qp_res.dual_solution;
    qp_warm_start_valid_ = true;

    const auto recon_start = std::chrono::steady_clock::now();
    candidate = LtvQpCandidateReconstructor::reconstruct(
        snapshot.current_state, qp_problem_buffer_, nominal_controls,
        controller_->config(), qp_res, snapshot.last_control_before_solve,
        snapshot.first_step_dt);
    total_recon_ms += elapsedMilliseconds(recon_start,
                                          std::chrono::steady_clock::now());
    if (!candidate.valid) {
      break;
    }

    double max_delta_u = 0.0;
    for (int step = 0; step < qp_problem_buffer_.horizon; ++step) {
      max_delta_u = std::max(
          max_delta_u,
          qp_res.primal_solution
              .segment<3>(qp_problem_buffer_.controlOffset(step))
              .lpNorm<Eigen::Infinity>());
    }
    const double elapsed_sqp_ms =
        elapsedMilliseconds(sqp_start, std::chrono::steady_clock::now());
    if (rti_pass + 1 < qp_rti_iterations_ &&
        max_delta_u > qp_rti_second_pass_delta_threshold_ &&
        elapsed_sqp_ms < 0.45 * qp_solver_settings_.time_limit_ms) {
      Control prev_u = snapshot.last_control_before_solve;
      for (int step = 0; step < qp_problem_buffer_.horizon; ++step) {
        const std::size_t idx = static_cast<std::size_t>(step);
        const double sdt =
            step == 0 ? snapshot.first_step_dt : controller_->config().dt;
        nominal_controls[idx] = controller_->clampStepIncrement(
            candidate.controls[idx], prev_u, sdt);
        prev_u = nominal_controls[idx];
      }
      nominal_states = controller_->rolloutTrajectory(
          snapshot.current_state, nominal_controls,
          snapshot.last_control_before_solve, snapshot.first_step_dt);
      continue;
    }
    break;
  }

  recordTiming(telemetry, ControlCycleTimingStage::kQpProblemBuild,
               total_build_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kOsqpNumericUpdate,
               total_update_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kQpBackendPhase,
               total_backend_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kQpCompletePhase,
               total_complete_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kOsqpSolve, total_solve_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kPrimalReconstructionRollout,
               total_recon_ms);

  if (candidate.valid && map_snapshot != nullptr) {
    rollout_safety = evaluateRolloutFootprintSafety(
        *map_snapshot, snapshot_from_odom, candidate.states);
  }

  LtvQpCandidateSafety safety;
  safety.inputs_healthy = snapshot.localization_fresh &&
                          snapshot.reference_fresh &&
                          snapshot.execution_lease_valid &&
                          snapshot.gimbal_valid;
  safety.emergency_stop_active = snapshot.emergency_stop_active;
  safety.collision_free =
      rollout_safety.level != RolloutSafetyLevel::kUnsafeStop;
  safety.localization_fresh = snapshot.localization_fresh;
  safety.reference_fresh = snapshot.reference_fresh;
  safety.execution_lease_valid = snapshot.execution_lease_valid;
  safety.gimbal_valid = snapshot.gimbal_valid;
  safety.map_fresh = snapshot.map_fresh;

  LtvQpSolverSettings audit_settings = qp_solver_settings_;
  audit_settings.time_limit_ms =
      qp_solver_settings_.time_limit_ms * std::max(1, executed_rti);
  const auto audit_start = std::chrono::steady_clock::now();
  if (candidate.valid) {
    audit = LtvQpCandidateValidator::validate(
        qp_problem_buffer_, nominal_controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), audit_settings, safety, qp_res, candidate,
        snapshot.first_step_dt);
  } else {
    audit = LtvQpCandidateValidator::validate(
        qp_problem_buffer_, nominal_controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), audit_settings, safety, qp_res,
        snapshot.first_step_dt);
  }
  total_audit_ms =
      elapsedMilliseconds(audit_start, std::chrono::steady_clock::now());
  recordTiming(telemetry, ControlCycleTimingStage::kCandidateHardCheck,
               total_audit_ms);

  telemetry.qp_attempted = true;
  telemetry.rti_iterations = executed_rti;
  telemetry.same_snapshot_identity =
      snapshot.identity_digest_valid &&
      snapshot.identity_digest == controlCycleSnapshotIdentityDigest(snapshot);
  telemetry.status = qp_res.status;
  telemetry.iterations = qp_res.iterations;
  telemetry.warm_start_used = qp_res.warm_start_used;
  telemetry.osqp_reported_update_ms = qp_res.update_time_ms;
  telemetry.osqp_reported_solve_ms = qp_res.solve_time_ms;
  telemetry.osqp_wall_update_ms = total_update_ms;
  telemetry.osqp_wall_solve_ms = total_solve_ms;
  telemetry.qp_complete_phase_ms = total_complete_ms;
  telemetry.osqp_wall_qp_phase_ms = total_backend_ms;
  telemetry.primal_residual = qp_res.primal_residual;
  telemetry.dual_residual = qp_res.dual_residual;
  telemetry.hard_constraint_margin =
      qp_solver_settings_.max_hard_constraint_violation -
      std::max(qp_res.hard_constraint_maximum_violation,
               audit.actual_hard_constraint_maximum_violation);
  telemetry.slack_maximum =
      std::max(qp_res.slack_maximum, audit.actual_slack_maximum);
  telemetry.candidate_feasible = audit.feasible;
  const char *reason =
      audit.rejection_reason.empty() ? "none" : audit.rejection_reason.c_str();
  std::snprintf(telemetry.rejection_reason.data(),
                telemetry.rejection_reason.size(), "%s", reason);
  telemetry.collision_gate = safety.collision_free;
  telemetry.map_gate = safety.map_fresh;

  if (!audit.feasible || !candidate.valid || candidate.controls.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "【QP诊断】status=%s iter=%d prim=%.2e dual=%.2e rho=%.3e rho_upd=%d "
        "slack=%.3f hard_viol=%.2e wall_solve=%.2fms reason=%s",
        ltvQpSolverStatusName(qp_res.status), qp_res.iterations,
        qp_res.primal_residual, qp_res.dual_residual, qp_res.rho,
        qp_res.rho_updates, telemetry.slack_maximum,
        audit.actual_hard_constraint_maximum_violation, total_solve_ms, reason);
    const bool keep_warm_start =
        (qp_res.status == LtvQpSolverStatus::kSolved ||
         qp_res.status == LtvQpSolverStatus::kSolvedInaccurate ||
         qp_res.status == LtvQpSolverStatus::kMaxIterations ||
         qp_res.status == LtvQpSolverStatus::kTimeLimit) &&
        qp_res.primal_solution.size() == qp_problem_buffer_.decisionSize() &&
        qp_res.dual_solution.size() == qp_problem_buffer_.constraintRows() &&
        qp_res.primal_solution.allFinite() && qp_res.dual_solution.allFinite();
    if (keep_warm_start) {
      qp_warm_start_.primal = qp_res.primal_solution;
      qp_warm_start_.dual = qp_res.dual_solution;
      qp_warm_start_valid_ = true;
    } else {
      qp_warm_start_valid_ = false;
      qp_solver_->resetWarmStart();
    }
    out_rollout_safety = rollout_safety;
    return false;
  }

  bool post_body_sat = false;
  bool post_wheel_sat = false;
  bool post_inc_lim = false;
  candidate.controls.front() = controller_->clampStepIncrement(
      candidate.controls.front(), snapshot.last_control_before_solve,
      snapshot.first_step_dt, &post_body_sat, &post_wheel_sat, &post_inc_lim);
  controller_->commitWarmControls(candidate.controls);

  out_result.success = true;
  out_result.controls = std::move(candidate.controls);
  out_result.states = std::move(candidate.states);
  out_result.iterations = qp_res.iterations;
  out_result.accepted_iterations = executed_rti;
  out_result.cost = 0.0;
  out_result.solve_time_ms =
      elapsedMilliseconds(sqp_start, std::chrono::steady_clock::now());
  out_result.body_limit_saturated = first_body_sat || post_body_sat;
  out_result.wheel_limit_saturated = first_wheel_sat || post_wheel_sat;
  out_result.increment_limited = first_inc_lim || post_inc_lim;
  out_result.module_limits_active = controller_->moduleLimitsActive();
  out_rollout_safety = rollout_safety;
  for (int axis = 0; axis < 3; ++axis) {
    telemetry.qp_first_control[static_cast<std::size_t>(axis)] =
        out_result.controls.front()(axis);
  }
  return true;
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

/** @brief Align with adapter/goal_manager planning-healthy states.
 *  CONFIRMED is the post-accept hold; rejecting it made GM commit while MPC
 *  fail-stopped (recovery182: mode=1 DualMap-ok on arbiter, MPC accepted=0).
 *  DEGRADED remains usable for control; LOST/BOOTSTRAP/RELOCALIZING/UNINITIALIZED
 *  stay fail-closed. */
void AtsSwerveMpcNode::onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
        message) {
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  const bool allows_control = message->state == LS::STATE_TRACKING ||
    message->state == LS::STATE_CONFIRMED ||
    message->state == LS::STATE_DEGRADED;
  if (!allows_control) {
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
  return !enforce_gimbal_lock_on_execution_ || !command.requires_gimbal_lock ||
         gimbal_status_->locked;
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
    bool supersede_grace_expired = false;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (supersede_deadline_ && lease_check_time > *supersede_deadline_) {
        // 宽限截止仍没有新 generation 的 EXECUTE：与无宽限时 superseded 的停车相同。
        supersede_deadline_.reset();
        if (active_execution_command_ &&
            active_execution_command_->map_generation == supersede_old_generation_) {
          active_execution_command_.reset();
          supersede_grace_expired = true;
        }
      }
      if (map_ready_ && last_map_ready_signal_ && !mapReadyFreshLocked()) {
        const std::uint64_t retire_floor = ready_lease_generation_ != 0
          ? ready_lease_generation_
          : planner_generation_;
        retired_planner_generation_ = std::max(
          retired_planner_generation_, retire_floor);
        planner_generation_usable_ = false;
        active_execution_command_.reset();
        map_ready_ = false;
      }
      execution_lease_valid = last_execution_command_signal_ &&
        lease_check_time >= *last_execution_command_signal_ &&
        std::chrono::duration<double>(
          lease_check_time - *last_execution_command_signal_).count() <=
          execution_command_timeout_ && active_execution_command_ &&
        activeExecutionValidLocked(lease_check_time) &&
        gimbalExecutionValidLocked(*active_execution_command_);
      gimbal_valid = active_execution_command_ &&
                     gimbalExecutionValidLocked(*active_execution_command_);
    }
    if (supersede_grace_expired) {
      engageFailStop(false);
      return;
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
  double rotation_delay = 0.0;
  double reference_terminal_speed = 0.0;
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
    rotation_delay = trajectory_tracker_.commandLatencyCompensation();
    reference_terminal_speed =
        trajectory_tracker_.terminalSpeed(terminal_speed_window_sec_);
    projection = trajectory_tracker_.project(current, last_control_(2));
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
  if (trajectory_expired) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "【轨迹过期】参考轨迹已超过有效期，停止跟踪并输出零速度；"
                         "请检查规划器发布频率与时间戳同步。");
    publishZeroCommandForFailure(nullptr);
    return;
  }
  const double nominal_control_period = 1.0 / std::max(1.0, control_rate_hz_);
  const double max_step_dt = std::max(
      controller_->config().dt, 2.0 * nominal_control_period);
  double actual_step_dt = nominal_control_period;
  if (get_clock()->ros_time_is_active()) {
    const auto update_stamp = now();
    if (last_control_update_stamp_) {
      const auto elapsed_ns = (update_stamp - *last_control_update_stamp_).nanoseconds();
      if (elapsed_ns < 0) {
        // A backward clock must not keep driving with a held command.
        const auto previous_update_stamp = *last_control_update_stamp_;
        engageFailStop();
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (last_stop_stamp_ < previous_update_stamp) {
          last_stop_stamp_ = previous_update_stamp;
        }
        // This stop is not authorized by a sequenced Goal Manager STOP.
        // Keep the pre-rewind reference ineligible even with a newer heartbeat.
        last_stop_command_.reset();
        return;
      }
      const auto period_ns = static_cast<std::int64_t>(
        std::llround(1e9 * nominal_control_period));
      if (elapsed_ns < period_ns) {
        // Slow simulation clocks must not advance dynamics/warm-start on every
        // wall tick. All safety/odometry/expiry checks above still run; renew
        // the checked command heartbeat until one ROS control period elapses.
        publishCommand(last_control_);
        return;
      }
      actual_step_dt = std::clamp(
          static_cast<double>(elapsed_ns) * 1e-9, 1e-3, max_step_dt);
    }
    last_control_update_stamp_ = update_stamp;
  } else {
    if (last_control_update_steady_) {
      const double elapsed_s = std::chrono::duration<double>(
          cycle_start - *last_control_update_steady_).count();
      if (std::isfinite(elapsed_s) && elapsed_s > 0.0) {
        actual_step_dt = std::clamp(elapsed_s, 1e-3, max_step_dt);
      }
    }
    last_control_update_steady_ = cycle_start;
  }
  if (goal_position_error <= goal_position_tolerance_ &&
      goal_yaw_error <= goal_yaw_tolerance_) {
    publishTerminalDecelerationCommand(actual_step_dt);
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
  // 6. 校验规划地图快照健康与 epoch 绑定，并调用 MPC（SQP-RTI 或 iLQR）求解
  std::shared_ptr<const ats_navigation_interfaces::msg::PlanningMapSnapshot>
      active_map_snapshot;
  Eigen::Vector3d snapshot_from_odom = Eigen::Vector3d::Zero();
  bool map_snapshot_fresh = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    map_snapshot_fresh = planningSnapshotUsableLocked(
        cycle_start, &active_map_snapshot, &snapshot_from_odom);
  }
  if (require_planning_snapshot_ && !map_snapshot_fresh) {
    publishZeroCommandForFailure("规划地图快照过期、未就绪或 epoch 不一致");
    return;
  }

  const auto snapshot_start = cycle_start;
  ControlCycleSnapshot snapshot;
  snapshot.cycle_sequence = cycle_sequence;
  snapshot.steady_start = cycle_start;
  const double now_sec_for_delay =
      get_clock()->ros_time_is_active()
          ? now().seconds()
          : std::chrono::duration<double>(cycle_start.time_since_epoch()).count();
  snapshot.current_state = advanceStateThroughCommandDelay(
      current, now_sec_for_delay, rotation_delay);
  snapshot.references = references;
  snapshot.last_control_before_solve = last_control_;
  snapshot.first_step_dt = actual_step_dt;
  snapshot.reference_fresh = projection.valid && !trajectory_expired &&
                             references.size() >= static_cast<std::size_t>(
                                 controller_->config().horizon + 1);
  snapshot.localization_fresh = localization_fresh;
  snapshot.execution_lease_valid = execution_lease_valid;
  snapshot.gimbal_valid = gimbal_valid;
  snapshot.emergency_stop_active = fail_stop_engaged_.load() ||
      (emergency_stop_watchdog_enabled_ &&
       emergency_stop_watchdog_.stopRequired());
  snapshot.map_fresh = map_snapshot_fresh;
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

  Se2MpcResult result;
  RolloutSafetyEvaluation rollout_safety;
  const auto active_solve_start = std::chrono::steady_clock::now();
  const bool qp_ok = solveSqpRti(
      snapshot, active_map_snapshot.get(), snapshot_from_odom, telemetry,
      result, rollout_safety);
  recordTiming(telemetry, ControlCycleTimingStage::kActiveSolve,
               elapsedMilliseconds(active_solve_start,
                                   std::chrono::steady_clock::now()));
  telemetry.rollout_safety_level = static_cast<int>(rollout_safety.level);
  telemetry.cross_track_error_m = projection.cross_track_error;

  if (!qp_ok || !result.success || result.controls.empty()) {
    if (rollout_safety.level == RolloutSafetyLevel::kUnsafeStop) {
      publishZeroCommandForFailure("预测滚动轨迹触发近端硬碰撞门禁");
    } else {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "【MPC求解失败】OSQP SQP-RTI 未获得可信解（status=%s reason=%s iter=%d "
          "耗时=%.2f ms 横向误差=%.3f m）；已触发确定性零速度 fail-stop。",
          ltvQpSolverStatusName(telemetry.status),
          telemetry.rejection_reason.data(), telemetry.iterations,
          telemetry.osqp_wall_solve_ms, projection.cross_track_error);
      publishZeroCommandForFailure(nullptr);
    }
    finalizeControlTelemetry(telemetry, cycle_start);
    return;
  }
  if (rollout_safety.level == RolloutSafetyLevel::kUnsafeStop) {
    publishZeroCommandForFailure("预测滚动轨迹触发近端硬碰撞门禁");
    finalizeControlTelemetry(telemetry, cycle_start);
    return;
  }
  // 7. 应用第一个控制量（含健康预测期 SLOWDOWN/BRAKE 平滑减速与终点区限速）并发布
  last_control_ = result.controls.front();
  if ((rollout_safety.level == RolloutSafetyLevel::kSlowdown ||
       rollout_safety.level == RolloutSafetyLevel::kBrake) &&
      rollout_safety.speed_scale < 1.0 - 1e-6) {
    Control scaled_target = last_control_;
    if (rollout_safety.level == RolloutSafetyLevel::kSlowdown) {
      // kSlowdown 表示预测无碰撞但靠近墙面（如 0.80 m 窄隧道）；仅限制高速接近，
      // 保留 0.35 m/s 慢速蠕行下限，避免在无碰撞窄通道重启时把起步速度乘衰至零。
      constexpr double kSlowdownCrawlFloorMps = 0.35;
      const double linear_speed = scaled_target.head<2>().norm();
      if (linear_speed > kSlowdownCrawlFloorMps) {
        const double target_speed =
            std::max(kSlowdownCrawlFloorMps,
                     linear_speed * rollout_safety.speed_scale);
        scaled_target.head<2>() *= (target_speed / linear_speed);
      }
    } else {
      // kBrake 表示预测在制动窗内进入负净空，严格无下限减速。
      scaled_target.head<2>() *= rollout_safety.speed_scale;
    }
    last_control_ = controller_->clampStepIncrement(
        scaled_target, snapshot.last_control_before_solve,
        snapshot.first_step_dt);
    if (!result.controls.empty()) {
      result.controls.front() = last_control_;
      controller_->commitWarmControls(result.controls);
    }
  }
  // 终点区限速（报告 5.1.10 第 1 项）：MPC 的模型里没有底盘约 0.4 s 的响应滞后，离终点
  // 0.15-0.3 m 时仍给 0.9-1.0 m/s，冲过终点再拉回。终点区内按 |v| <= d / tau 限速，相当于
  // 一阶趋近，tau 大于 e 倍滞后时不过冲。只对终点速度为零的 reference 生效。
  if (terminal_approach_time_constant_ > 0.0 &&
      goal_position_error <= terminal_approach_radius_ &&
      reference_terminal_speed <= terminal_stop_speed_threshold_) {
    last_control_ = limitTerminalApproachSpeed(
        last_control_, goal_position_error, terminal_approach_time_constant_);
    if (goal_position_error <= goal_position_tolerance_) {
      // Once translation has reached the endpoint, the physical chassis still
      // needs braking distance for its stationary yaw tail. Waiting until the
      // yaw tolerance is reached can overshoot a footprint-checked terminal
      // orientation (d26). Reuse the configured approach time constant and
      // preserve the existing angular acceleration limit during deceleration.
      const double deceleration_floor = std::max(
          0.0, std::abs(snapshot.last_control_before_solve(2)) -
                   controller_->config().max_awz * snapshot.first_step_dt);
      const double yaw_rate_limit = std::max(
          goal_yaw_error / terminal_approach_time_constant_, deceleration_floor);
      last_control_(2) = std::clamp(last_control_(2), -yaw_rate_limit, yaw_rate_limit);
    }
    if (!result.controls.empty()) {
      result.controls.front() = last_control_;
      controller_->commitWarmControls(result.controls);
    }
  }
  const auto command_publish_start = std::chrono::steady_clock::now();
  publishCommand(last_control_);
  recordTiming(telemetry, ControlCycleTimingStage::kCommandPublish,
               elapsedMilliseconds(command_publish_start,
                                   std::chrono::steady_clock::now()));
  for (int axis = 0; axis < 3; ++axis) {
    telemetry.published_first_control[static_cast<std::size_t>(axis)] =
        last_control_(axis);
  }
  const double max_pub_wheel = controller_->computeMaxModuleSpeed(last_control_);
  telemetry.published_wheel_speed_max_mps = max_pub_wheel;
  telemetry.wheel_speed_violation_mps =
      controller_->config().max_wheel_speed > 1e-6
          ? std::max(0.0, max_pub_wheel - controller_->config().max_wheel_speed)
          : 0.0;
  const auto diagnostics_log_start = std::chrono::steady_clock::now();
  reportSolverDiagnostics(result);
  double logging_publish_ms = elapsedMilliseconds(
      diagnostics_log_start, std::chrono::steady_clock::now());
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
               "【MPC调试】vx=%.3f vy=%.3f wz=%.3f 横向误差=%.3f "
               "进度比例=%.2f 求解=%.2fms",
               last_control_(0), last_control_(1), last_control_(2),
               projection.cross_track_error,
               trajectory_tracker_.progressScale(projection.cross_track_error),
               result.solve_time_ms);
  logging_publish_ms += elapsedMilliseconds(debug_log_start,
                                             std::chrono::steady_clock::now());
  recordTiming(telemetry, ControlCycleTimingStage::kLoggingPublish,
               logging_publish_ms);
  finalizeControlTelemetry(telemetry, cycle_start);
}

/**
 * @brief 完成本控制周期的性能记录并按明确预算累积根因计数。
 * @details OSQP update/solve 各自以 `qp_time_limit_ms` 复核；LTV 构造、candidate
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
        "控制周期遥测 序号=%llu 模式=%s 频率=%.1fHz 周期=%.3fms "
        "qp_status=%s iter=%d prim=%.2e dual=%.2e slack=%.3f "
        "qp_wall(update/solve/backend/complete)=%.3f/%.3f/%.3f/%.3fms "
        "callback_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_backend_p50/p95/p99=%.3f/%.3f/%.3fms "
        "qp_complete_p50/p95/p99=%.3f/%.3f/%.3fms "
        "same_snapshot=%s feasible=%s reject=%s",
        static_cast<unsigned long long>(telemetry.cycle_sequence), solver_mode_.c_str(),
        control_rate_hz_, control_period_ms, ltvQpSolverStatusName(telemetry.status),
        telemetry.iterations, telemetry.primal_residual, telemetry.dual_residual,
        telemetry.slack_maximum, telemetry.osqp_wall_update_ms,
        telemetry.osqp_wall_solve_ms,
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
  if (telemetry.qp_attempted &&
      telemetry.status == LtvQpSolverStatus::kTimeLimit) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpTimeLimitStatus);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kOsqpSolve, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kOsqpSolveBudgetOverrun);
  }
  if (exceeded(ControlCycleTimingStage::kFullCallback, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kCallbackPeriodOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpProblemBuild, control_period_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpProblemBuildBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kOsqpNumericUpdate, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpUpdateBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpBackendPhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpPhaseBudgetOverrun);
  }
  if (telemetry.qp_attempted && exceeded(
          ControlCycleTimingStage::kQpCompletePhase, qp_solver_settings_.time_limit_ms)) {
    control_telemetry_.incrementDeadlineCause(
        ControlCycleDeadlineCause::kQpCompletePhaseBudgetOverrun);
  }
  if (telemetry.qp_attempted && candidate_audit_overrun) {
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
  last_control_update_stamp_.reset();
  last_control_update_steady_.reset();
  qp_warm_start_valid_ = false;
  if (qp_solver_) {
    qp_solver_->resetWarmStart();
  }
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    command_delay_history_.clear();
  }
  publishCommand(last_control_);
  controller_->reset();
  if (reason_zh != nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "【控制兜底】%s，已输出确定性零速度。", reason_zh);
  }
}

Control AtsSwerveMpcNode::limitTerminalApproachSpeed(
    const Control & command, double goal_distance, double time_constant) {
  const double limit = std::max(0.0, goal_distance) / time_constant;
  const double speed = command.head<2>().norm();
  if (speed <= limit || speed <= 0.0) {
    return command;
  }
  Control limited = command;
  limited.head<2>() *= limit / speed;
  return limited;
}

/**
 * @brief 终点容差圆内的减速出口：上一拍指令每轴按 max_a*step_dt 降向零，降到零后才清零。
 * @details 报告 5.1.9 B：容差圆内直接发零时指令从约 0.5 m/s 阶跃到 0，底盘带着加速度
 *          限幅与约 0.4 s 滞后滑出 0.15-0.25 m，MPC 再从零起步拉回，形成终点极限环。
 *          这里每拍按实际经过时间步长 step_dt 衰减，与 MPC 自身的增量约束同一物理口径；
 *          车在衰减途中滑出容差圆时，MPC 从这条衰减后的指令接着求解，而不是从零。
 *          warm start 每拍都清掉：衰减期间不求解，旧 warm controls 没有随时间左移。
 *          轨迹过期、急停与其它失败路径不经过这里，仍直接发零。
 */
void AtsSwerveMpcNode::publishTerminalDecelerationCommand(double step_dt) {
  const Se2MpcConfig &config = controller_->config();
  const double effective_dt =
      (std::isfinite(step_dt) && step_dt > 0.0) ? step_dt : config.dt;
  const Control step(config.max_ax * effective_dt, config.max_ay * effective_dt,
                     config.max_awz * effective_dt);
  Control next = Control::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double magnitude = std::max(0.0, std::abs(last_control_(axis)) - step(axis));
    next(axis) = std::copysign(magnitude, last_control_(axis));
  }
  if (next.isZero(0.0)) {
    publishZeroCommandForFailure(nullptr);
    return;
  }
  last_control_ = next;
  publishCommand(last_control_);
  controller_->reset();
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
  if (control_period > 1.2 * config.dt) {
    RCLCPP_WARN(
        get_logger(),
        "【动力学参数不匹配】控制周期 %.4f s（%.1f Hz）慢于预测步长 dt=%.4f s 超过 20%%，"
        "建议提高 control_rate_hz 或调大预测步长 dt。",
        control_period, control_rate_hz_, config.dt);
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
void AtsSwerveMpcNode::engageFailStop(
    bool record_stop_stamp, std::optional<CommandOrder> stop_command) {
  const bool was_engaged = fail_stop_engaged_.exchange(true);
  if (!was_engaged) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_tracker_.clear();
    trajectory_frame_.clear();
    trajectory_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (record_stop_stamp) {
      last_stop_stamp_ = now();
      last_stop_command_ = stop_command;
    }
    controller_->reset();
    qp_warm_start_valid_ = false;
    if (qp_solver_) {
      qp_solver_->resetWarmStart();
    }
  }
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    command_delay_history_.clear();
  }
  last_control_.setZero();
  last_control_update_stamp_.reset();
  last_control_update_steady_.reset();
  publishCommand(last_control_);
}

/** @brief 将内部车体系 Control 映射到唯一 `geometry_msgs::msg::Twist` 发布通道。 */
void AtsSwerveMpcNode::publishCommand(const Control &command) {
  const double stamp_sec =
      get_clock()->ros_time_is_active()
          ? now().seconds()
          : std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
  {
    std::lock_guard<std::mutex> history_lock(command_history_mutex_);
    if (!command_delay_history_.empty() &&
        stamp_sec < command_delay_history_.back().stamp_sec) {
      command_delay_history_.clear();
    }
    command_delay_history_.push_back(CommandHistorySample{stamp_sec, command});
    while (command_delay_history_.size() > 64U ||
           (command_delay_history_.size() > 2U &&
            stamp_sec - command_delay_history_.front().stamp_sec > 0.5)) {
      command_delay_history_.pop_front();
    }
  }
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
