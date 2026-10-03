// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include "minco_planner/planning/graph_search_failure.hpp"
#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/nodes/planning_digest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "minco_planner/trajectory/reference_path_timing.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{

namespace
{

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
    std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
    std::isfinite(pose.orientation.w);
}

std::string formatCollisionIndices(const std::vector<CollisionSample> & collisions)
{
  std::ostringstream out;
  const std::size_t cap = std::min<std::size_t>(16U, collisions.size());
  for (std::size_t i = 0; i < cap; ++i) {
    if (i != 0U) {
      out << ',';
    }
    out << collisions[i].trajectory_index;
  }
  if (collisions.size() > cap) {
    out << ",...";
  }
  return out.str();
}

// 把拒绝点周围的规划栅格原样打成一行,用于区分
// "车自身格子被占"与"外侧存在成片障碍"。只在 footprint
// 拒绝分支调用,窗口为 ±radius_m,行序从北到南。
std::string describeGridWindow(
  const nav_msgs::msg::OccupancyGrid & grid, double center_x, double center_y,
  double radius_m, int obstacle_value_threshold, bool unknown_is_obstacle)
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0U || grid.info.height == 0U) {
    return std::string("<invalid grid>");
  }
  const double resolution = grid.info.resolution;
  const int span = std::max(1, static_cast<int>(std::lround(radius_m / resolution)));
  const int center_mx = static_cast<int>(
    std::floor((center_x - grid.info.origin.position.x) / resolution));
  const int center_my = static_cast<int>(
    std::floor((center_y - grid.info.origin.position.y) / resolution));
  std::ostringstream stream;
  stream << "res=" << resolution << " center_cell=(" << center_mx << ", " << center_my
         << ") span=" << span << " rows_north_to_south=[";
  for (int dy = span; dy >= -span; --dy) {
    const int my = center_my + dy;
    for (int dx = -span; dx <= span; ++dx) {
      const int mx = center_mx + dx;
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) ||
        my >= static_cast<int>(grid.info.height))
      {
        // 越界在 footprint 判据里等价于占据,这里用独立
      // 符号标出来,避免与真实障碍混淆。
        stream << 'X';
        continue;
      }
      const int value = grid.data[
        static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
        static_cast<std::size_t>(mx)];
      if (value < 0) {
        stream << (unknown_is_obstacle ? 'U' : 'u');
      } else if (value >= obstacle_value_threshold) {
        stream << '#';
      } else if (value == 0) {
        stream << '.';
      } else {
        // 连续风险 1..threshold-1 保留量级,便于判断是否贴着 terrain 风险带走。
        stream << static_cast<char>('0' + std::min(9, value / 10));
      }
    }
    if (dy != -span) {
      stream << '/';
    }
  }
  stream << ']';
  return stream.str();
}

}  // namespace

MincoPlannerNode::MincoPlannerNode(const rclcpp::NodeOptions & options)
: Node("minco_planner", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  declareAndLoadParams();

  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions map_options;
  map_options.callback_group = map_callback_group_;
  rclcpp::SubscriptionOptions health_options;
  health_options.callback_group = health_callback_group_;

  // Match adapter TRANSIENT_LOCAL publication. VOLATILE missed the first
  // /rc_esdf/planning_grid under DualMap-era ready races (recovery231:
  // ready=1 + snapshot log, but MINCO kept MAP_UNREADY / zero plans).
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&MincoPlannerNode::onGrid, this, std::placeholders::_1), map_options);
  if (!goal_topic_.empty()) {
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&MincoPlannerNode::onGoal, this, std::placeholders::_1), planning_options);
  }
  if (!goal_request_topic_.empty()) {
    goal_request_sub_ = create_subscription<ats_navigation_interfaces::msg::PlannerGoal>(
      goal_request_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&MincoPlannerNode::onPlannerGoal, this, std::placeholders::_1), planning_options);
  }
  if (!raw_path_topic_.empty()) {
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, rclcpp::QoS(1));
  }
  if (!preprocessed_guide_topic_.empty()) {
    preprocessed_guide_pub_ = create_publisher<nav_msgs::msg::Path>(
      preprocessed_guide_topic_, rclcpp::QoS(1).reliable());
  }
  if (!esdf_refined_guide_topic_.empty()) {
    esdf_refined_guide_pub_ = create_publisher<nav_msgs::msg::Path>(
      esdf_refined_guide_topic_, rclcpp::QoS(1).reliable());
  }
  if (planner_manages_emergency_stop_) {
    reference_path_pub_ =
      create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(1));
  }
  if (!candidate_reference_path_topic_.empty()) {
    candidate_reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      candidate_reference_path_topic_, rclcpp::QoS(1).reliable());
  }
  if (!debug_marker_topic_.empty()) {
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic_, rclcpp::QoS(1));
  }
  if (planner_manages_emergency_stop_) {
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());
  }
  if (!planner_status_topic_.empty()) {
    planner_status_pub_ = create_publisher<ats_navigation_interfaces::msg::PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(1).reliable().transient_local());
  }
  safety_state_.map_ready = map_ready_topic_.empty();
  if (!map_ready_topic_.empty()) {
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MincoPlannerNode::onMapReady, this, std::placeholders::_1), health_options);
  }
  safety_watchdog_timer_ = create_wall_timer(
    std::chrono::duration<double>(emergency_stop_heartbeat_period_sec_),
    std::bind(&MincoPlannerNode::onMapReadyWatchdog, this), health_callback_group_);
  runtime_safety_recheck_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(0.1, runtime_safety_recheck_hz_)),
    std::bind(&MincoPlannerNode::onRuntimeSafetyRecheck, this), health_callback_group_);
  publishEmergencyStop(true);

  RCLCPP_INFO(
    get_logger(), "【规划器就绪】栅格='%s' 目标='%s' 请求='%s' 原始路径='%s' 参考路径='%s'",
    grid_topic_.c_str(), goal_topic_.c_str(), goal_request_topic_.c_str(), raw_path_topic_.c_str(),
    reference_path_topic_.c_str());
}

void MincoPlannerNode::declareAndLoadParams()
{
  declare_parameter<std::string>("grid_topic", grid_topic_);
  declare_parameter<std::string>("goal_topic", goal_topic_);
  declare_parameter<std::string>("goal_request_topic", goal_request_topic_);
  declare_parameter<std::string>("planner_status_topic", planner_status_topic_);
  declare_parameter<std::string>("raw_path_topic", raw_path_topic_);
  declare_parameter<std::string>("reference_path_topic", reference_path_topic_);
  declare_parameter<std::string>("candidate_reference_path_topic", candidate_reference_path_topic_);
  declare_parameter<std::string>("preprocessed_guide_topic", preprocessed_guide_topic_);
  declare_parameter<std::string>("esdf_refined_guide_topic", esdf_refined_guide_topic_);
  declare_parameter<std::string>("debug_marker_topic", debug_marker_topic_);
  declare_parameter<std::string>("map_ready_topic", map_ready_topic_);
  declare_parameter<std::string>("emergency_stop_topic", emergency_stop_topic_);
  declare_parameter<double>("map_ready_timeout_sec", map_ready_timeout_sec_);
  declare_parameter<std::int64_t>("log_throttle_ms", static_cast<std::int64_t>(log_throttle_ms_));
  declare_parameter<double>(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  declare_parameter<double>("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  declare_parameter<double>("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  declare_parameter<bool>(
    "retain_safe_reference_on_snapshot_change", retain_safe_reference_on_snapshot_change_);
  declare_parameter<double>("retain_reference_horizon_sec", retain_reference_horizon_sec_);
  declare_parameter<bool>(
    "reference_window_from_robot_progress", reference_window_from_robot_progress_);
  declare_parameter<double>("reference_window_backward_sec", reference_window_backward_sec_);
  declare_parameter<bool>("replan_advisory_enabled", replan_advisory_enabled_);
  declare_parameter<double>("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  declare_parameter<bool>("force_body_yaw_follow", force_body_yaw_follow_);
  declare_parameter<std::string>("global_frame", global_frame_);
  declare_parameter<std::string>("robot_frame", robot_frame_);
  declare_parameter<std::string>("search_algorithm", search_algorithm_);
  declare_parameter<bool>("astar_fallback", astar_fallback_);
  declare_parameter<bool>("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  declare_parameter<bool>(
    "escape_from_contact_enabled", escape_prefix_params_.enabled);
  declare_parameter<double>(
    "escape_from_contact_max_head_offset", escape_prefix_params_.max_head_offset_m);
  declare_parameter<double>(
    "escape_from_contact_max_prefix_length", escape_prefix_params_.max_prefix_length_m);
  declare_parameter<double>(
    "escape_from_contact_max_prefix_yaw_sweep",
    escape_prefix_params_.max_prefix_yaw_sweep_rad);
  declare_parameter<int>(
    "escape_from_contact_max_prefix_points",
    static_cast<int>(escape_prefix_params_.max_prefix_points));
  declare_parameter<double>(
    "escape_from_contact_max_contact_depth", escape_prefix_params_.max_contact_depth_m);
  declare_parameter<bool>(
    "goal_pose_admission_enabled", goal_pose_admission_params_.enabled);
  declare_parameter<double>(
    "goal_admission_position_tolerance", goal_pose_admission_params_.position_tolerance_m);
  declare_parameter<double>(
    "goal_admission_yaw_tolerance", goal_pose_admission_params_.yaw_tolerance_rad);
  declare_parameter<double>(
    "goal_admission_position_shrink", goal_pose_admission_params_.position_shrink);
  declare_parameter<double>(
    "goal_admission_yaw_shrink", goal_pose_admission_params_.yaw_shrink);
  declare_parameter<double>(
    "goal_admission_extra_margin", goal_pose_admission_params_.preferred_extra_margin_m);
  declare_parameter<double>(
    "goal_admission_position_step", goal_pose_admission_params_.position_step_m);
  declare_parameter<int>(
    "goal_admission_yaw_samples",
    static_cast<int>(goal_pose_admission_params_.yaw_samples));
  // <=0 disables. Gazebo red_box enables to block north mega-detours (d21).
  declare_parameter<double>(
    "commit_max_length_ratio", commit_geometry_limits_.max_length_ratio);
  declare_parameter<double>(
    "commit_max_lateral_deviation_m", commit_geometry_limits_.max_lateral_deviation_m);
  declare_parameter<bool>("planner_manages_emergency_stop", planner_manages_emergency_stop_);

  GridAstarParams astar_params;
  declare_parameter<int>("obstacle_value_threshold", astar_params.obstacle_value_threshold);
  declare_parameter<bool>("unknown_is_obstacle", astar_params.unknown_is_obstacle);
  declare_parameter<bool>("allow_diagonal", astar_params.allow_diagonal);
  declare_parameter<double>("search_clearance_cost_weight", astar_params.clearance_cost_weight);
  declare_parameter<double>(
    "search_clearance_cost_distance", astar_params.clearance_cost_distance);
  GridJpsParams jps_params;
  declare_parameter<int>("jps_max_expanded_nodes", jps_params.max_expanded_nodes);
  declare_parameter<double>("jps_safe_distance", jps_params.safe_distance);
  declare_parameter<bool>("clearance_relaxation_enabled", clearance_relaxation_enabled_);
  declare_parameter<bool>(
    "endpoint_clearance_relaxation_enabled", endpoint_clearance_relaxation_enabled_);
  declare_parameter<double>("search_clearance_floor", -1.0);

  MincoTrajectoryOptimizerParams optimizer_params;
  declare_parameter<double>("reference_speed", optimizer_params.reference_speed);
  declare_parameter<double>("min_segment_time", optimizer_params.min_segment_time);
  declare_parameter<double>("sample_spacing", optimizer_params.sample_spacing);
  declare_parameter<double>("max_velocity", optimizer_params.max_velocity);
  declare_parameter<double>("max_acceleration", optimizer_params.max_acceleration);
  declare_parameter<double>("max_jerk", optimizer_params.max_jerk);
  declare_parameter<double>("max_lateral_acceleration", optimizer_params.max_lateral_acceleration);
  declare_parameter<int>(
    "max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  declare_parameter<double>("time_scaling_factor", optimizer_params.time_scaling_factor);
  declare_parameter<bool>(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  declare_parameter<double>("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  declare_parameter<double>(
    "esdf_obstacle_trigger_clearance", optimizer_params.esdf_obstacle_trigger_clearance);
  declare_parameter<double>(
    "esdf_obstacle_target_clearance", optimizer_params.esdf_obstacle_target_clearance);
  declare_parameter<int>(
    "esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  declare_parameter<double>(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  declare_parameter<double>(
    "guide_control_point_spacing", optimizer_params.guide_control_point_spacing);
  declare_parameter<int>(
    "guide_smoothing_iterations", optimizer_params.guide_smoothing_iterations);
  declare_parameter<double>("guide_smoothing_alpha", optimizer_params.guide_smoothing_alpha);
  declare_parameter<double>(
    "guide_smoothing_fidelity", optimizer_params.guide_smoothing_fidelity);
  declare_parameter<double>(
    "guide_smoothing_max_deviation", optimizer_params.guide_smoothing_max_deviation);
  declare_parameter<double>(
    "guide_smoothing_min_clearance", optimizer_params.guide_smoothing_min_clearance);
  declare_parameter<double>("joint_waypoint_spacing", optimizer_params.joint_waypoint_spacing);
  declare_parameter<int>("joint_samples_per_piece", optimizer_params.joint_samples_per_piece);
  declare_parameter<double>("joint_max_piece_time", optimizer_params.joint_max_piece_time);
  declare_parameter<double>("joint_energy_weight", optimizer_params.joint_energy_weight);
  declare_parameter<double>("joint_time_weight", optimizer_params.joint_time_weight);
  declare_parameter<double>("joint_obstacle_weight", optimizer_params.joint_obstacle_weight);
  declare_parameter<double>("joint_velocity_weight", optimizer_params.joint_velocity_weight);
  declare_parameter<double>(
    "joint_acceleration_weight", optimizer_params.joint_acceleration_weight);
  declare_parameter<double>("joint_lateral_weight", optimizer_params.joint_lateral_weight);
  declare_parameter<double>("joint_guide_weight", optimizer_params.joint_guide_weight);
  declare_parameter<double>(
    "joint_guide_max_deviation", optimizer_params.joint_guide_max_deviation);
  declare_parameter<double>("joint_center_clearance", optimizer_params.joint_center_clearance);
  declare_parameter<double>(
    "joint_footprint_clearance", optimizer_params.joint_footprint_clearance);
  declare_parameter<int>(
    "joint_footprint_edge_samples", optimizer_params.joint_footprint_edge_samples);
  declare_parameter<double>("joint_g_epsilon", optimizer_params.joint_g_epsilon);
  declare_parameter<int>("joint_max_iterations", optimizer_params.joint_max_iterations);
  declare_parameter<double>("joint_time_budget_ms", optimizer_params.joint_time_budget_ms);
  declare_parameter<double>("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  declare_parameter<double>(
    "esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  declare_parameter<double>(
    "esdf_obstacle_trust_region", optimizer_params.esdf_obstacle_trust_region);
  declare_parameter<int>(
    "esdf_obstacle_backtracking_steps", optimizer_params.esdf_obstacle_backtracking_steps);
  declare_parameter<double>(
    "esdf_obstacle_smoothing_weight", optimizer_params.esdf_obstacle_smoothing_weight);
  declare_parameter<bool>(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  declare_parameter<double>("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  declare_parameter<double>(
    "esdf_footprint_trigger_clearance", optimizer_params.esdf_footprint_trigger_clearance);
  declare_parameter<double>(
    "esdf_footprint_target_clearance", optimizer_params.esdf_footprint_target_clearance);
  declare_parameter<double>(
    "esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);
  declare_parameter<double>(
    "path_duplicate_epsilon", optimizer_params.geometry_preprocessor.duplicate_epsilon);
  declare_parameter<double>(
    "path_collinear_lateral_tolerance",
    optimizer_params.geometry_preprocessor.collinear_lateral_tolerance);
  declare_parameter<double>(
    "path_short_segment_length", optimizer_params.geometry_preprocessor.short_segment_length);
  declare_parameter<double>(
    "path_corner_angle_threshold_rad",
    optimizer_params.geometry_preprocessor.corner_angle_threshold_rad);
  declare_parameter<bool>(
    "path_footprint_aware_shortcut_enabled",
    optimizer_params.geometry_preprocessor.footprint_aware_shortcut_enabled);
  declare_parameter<double>(
    "path_fillet_radius", optimizer_params.geometry_preprocessor.fillet_radius);
  declare_parameter<int>(
    "path_fillet_arc_samples", optimizer_params.geometry_preprocessor.fillet_arc_samples);
  declare_parameter<double>(
    "path_shortcut_min_clearance", optimizer_params.geometry_preprocessor.shortcut_min_clearance);

  YawSplinePlannerParams yaw_params;
  declare_parameter<std::string>("yaw_mode", yaw_params.mode);
  declare_parameter<double>("yaw_rate_limit", yaw_params.yaw_rate_limit);
  declare_parameter<double>("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  declare_parameter<double>("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  declare_parameter<int>("yaw_tangent_symmetry_order", yaw_params.tangent_symmetry_order);
  declare_parameter<double>("yaw_narrow_gap_bridge_time", yaw_params.narrow_gap_bridge_time);
  declare_parameter<double>("yaw_acceleration_limit", yaw_params.yaw_acceleration_limit);
  declare_parameter<std::string>("yaw_open_area_mode", yaw_params.open_area_yaw_mode);
  declare_parameter<double>(
    "wheel_speed_time_scaling_limit", wheel_speed_time_scaling_params_.wheel_speed_limit);
  declare_parameter<double>(
    "wheel_speed_time_scaling_offset_x", wheel_speed_time_scaling_params_.wheel_offset_x);
  declare_parameter<double>(
    "wheel_speed_time_scaling_offset_y", wheel_speed_time_scaling_params_.wheel_offset_y);
  declare_parameter<double>(
    "wheel_speed_time_scaling_change_rate", wheel_speed_time_scaling_params_.scale_change_rate);
  declare_parameter<double>(
    "narrow_turn_speed_limit", wheel_speed_time_scaling_params_.narrow_turn_speed_limit);
  declare_parameter<double>(
    "narrow_turn_yaw_rate_threshold",
    wheel_speed_time_scaling_params_.narrow_turn_yaw_rate_threshold);
  declare_parameter<double>(
    "clearance_speed_min", wheel_speed_time_scaling_params_.clearance_speed_min);
  declare_parameter<double>(
    "clearance_speed_low", wheel_speed_time_scaling_params_.clearance_speed_low);
  declare_parameter<double>(
    "clearance_speed_high", wheel_speed_time_scaling_params_.clearance_speed_high);
  declare_parameter<double>(
    "terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);
  declare_parameter<bool>(
    "terminal_yaw_relocation_enabled", terminal_yaw_relocation_enabled_);
  declare_parameter<int>(
    "footprint_yaw_refinement_rounds", footprint_yaw_refinement_rounds_);
  declare_parameter<int>(
    "terminal_yaw_relocation_max_candidates", terminal_yaw_relocation_max_candidates_);
  declare_parameter<double>(
    "terminal_yaw_relocation_window_length",
    terminal_yaw_relocation_params_.terminal_window_length);

  FootprintSafetyParams footprint_params;
  declare_parameter<double>("footprint_length", footprint_params.length);
  declare_parameter<double>("footprint_width", footprint_params.width);
  declare_parameter<double>("footprint_safety_margin", footprint_params.safety_margin);
  declare_parameter<double>(
    "swept_max_corner_step_cells", footprint_params.swept_max_corner_step_cells);

  LocalCollisionRepairParams repair_params;
  declare_parameter<bool>("local_repair_enabled", repair_params.enabled);
  declare_parameter<int>("local_repair_max_iterations", repair_params.max_iterations);
  declare_parameter<double>("local_repair_search_radius", repair_params.search_radius);

  get_parameter("grid_topic", grid_topic_);
  get_parameter("goal_topic", goal_topic_);
  get_parameter("goal_request_topic", goal_request_topic_);
  get_parameter("planner_status_topic", planner_status_topic_);
  get_parameter("raw_path_topic", raw_path_topic_);
  get_parameter("reference_path_topic", reference_path_topic_);
  get_parameter("candidate_reference_path_topic", candidate_reference_path_topic_);
  get_parameter("preprocessed_guide_topic", preprocessed_guide_topic_);
  get_parameter("esdf_refined_guide_topic", esdf_refined_guide_topic_);
  get_parameter("debug_marker_topic", debug_marker_topic_);
  get_parameter("map_ready_topic", map_ready_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("map_ready_timeout_sec", map_ready_timeout_sec_);
  map_ready_timeout_sec_ = std::max(0.1, map_ready_timeout_sec_);
  std::int64_t configured_log_throttle_ms = static_cast<std::int64_t>(log_throttle_ms_);
  get_parameter("log_throttle_ms", configured_log_throttle_ms);
  log_throttle_ms_ = static_cast<std::uint64_t>(
    std::clamp<std::int64_t>(configured_log_throttle_ms, 250, 60000));
  get_parameter(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  emergency_stop_heartbeat_period_sec_ = std::max(0.02, emergency_stop_heartbeat_period_sec_);
  get_parameter("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  runtime_safety_recheck_hz_ = std::max(0.1, runtime_safety_recheck_hz_);
  get_parameter("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  runtime_safety_horizon_sec_ = std::max(0.0, runtime_safety_horizon_sec_);
  get_parameter(
    "retain_safe_reference_on_snapshot_change", retain_safe_reference_on_snapshot_change_);
  get_parameter("retain_reference_horizon_sec", retain_reference_horizon_sec_);
  get_parameter("reference_window_from_robot_progress", reference_window_from_robot_progress_);
  get_parameter("replan_advisory_enabled", replan_advisory_enabled_);
  get_parameter("reference_window_backward_sec", reference_window_backward_sec_);
  reference_window_backward_sec_ = std::max(0.0, reference_window_backward_sec_);
  get_parameter("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  body_yaw_follow_clearance_ = std::max(0.0, body_yaw_follow_clearance_);
  get_parameter("force_body_yaw_follow", force_body_yaw_follow_);
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("search_algorithm", search_algorithm_);
  get_parameter("astar_fallback", astar_fallback_);
  get_parameter("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  get_parameter("escape_from_contact_enabled", escape_prefix_params_.enabled);
  get_parameter("escape_from_contact_max_head_offset", escape_prefix_params_.max_head_offset_m);
  get_parameter(
    "escape_from_contact_max_prefix_length", escape_prefix_params_.max_prefix_length_m);
  get_parameter(
    "escape_from_contact_max_prefix_yaw_sweep",
    escape_prefix_params_.max_prefix_yaw_sweep_rad);
  {
    int escape_max_prefix_points =
      static_cast<int>(escape_prefix_params_.max_prefix_points);
    get_parameter("escape_from_contact_max_prefix_points", escape_max_prefix_points);
    escape_prefix_params_.max_prefix_points =
      static_cast<std::size_t>(std::max(0, escape_max_prefix_points));
  }
  get_parameter(
    "escape_from_contact_max_contact_depth", escape_prefix_params_.max_contact_depth_m);
  get_parameter("goal_pose_admission_enabled", goal_pose_admission_params_.enabled);
  get_parameter(
    "goal_admission_position_tolerance", goal_pose_admission_params_.position_tolerance_m);
  get_parameter("goal_admission_yaw_tolerance", goal_pose_admission_params_.yaw_tolerance_rad);
  get_parameter("goal_admission_position_shrink", goal_pose_admission_params_.position_shrink);
  get_parameter("goal_admission_yaw_shrink", goal_pose_admission_params_.yaw_shrink);
  get_parameter("goal_admission_extra_margin",
    goal_pose_admission_params_.preferred_extra_margin_m);
  get_parameter("goal_admission_position_step", goal_pose_admission_params_.position_step_m);
  {
    int goal_admission_yaw_samples =
      static_cast<int>(goal_pose_admission_params_.yaw_samples);
    get_parameter("goal_admission_yaw_samples", goal_admission_yaw_samples);
    goal_pose_admission_params_.yaw_samples =
      static_cast<std::size_t>(std::max(1, goal_admission_yaw_samples));
  }
  get_parameter("commit_max_length_ratio", commit_geometry_limits_.max_length_ratio);
  get_parameter(
    "commit_max_lateral_deviation_m", commit_geometry_limits_.max_lateral_deviation_m);
  commit_geometry_limits_.max_length_ratio =
    std::max(0.0, commit_geometry_limits_.max_length_ratio);
  commit_geometry_limits_.max_lateral_deviation_m =
    std::max(0.0, commit_geometry_limits_.max_lateral_deviation_m);
  get_parameter("planner_manages_emergency_stop", planner_manages_emergency_stop_);
  get_parameter("obstacle_value_threshold", obstacle_value_threshold_);
  get_parameter("unknown_is_obstacle", unknown_is_obstacle_);
  astar_params.obstacle_value_threshold = obstacle_value_threshold_;
  astar_params.unknown_is_obstacle = unknown_is_obstacle_;
  get_parameter("allow_diagonal", astar_params.allow_diagonal);
  get_parameter("search_clearance_cost_weight", astar_params.clearance_cost_weight);
  get_parameter("search_clearance_cost_distance", astar_params.clearance_cost_distance);
  if (astar_params.clearance_cost_weight > 0.0 && search_algorithm_ == "jps") {
    // JPS 的跳点剪枝只对均匀代价成立，无法表达离墙代价；开启离墙代价时改用 A*。
    RCLCPP_WARN(
      get_logger(),
      "【规划参数修正】search_clearance_cost_weight=%.3f 需要加权搜索，search_algorithm 由 jps 改为 astar。",
      astar_params.clearance_cost_weight);
    search_algorithm_ = "astar";
  }
  get_parameter("jps_max_expanded_nodes", jps_params.max_expanded_nodes);
  get_parameter("jps_safe_distance", jps_params.safe_distance);
  get_parameter("reference_speed", optimizer_params.reference_speed);
  get_parameter("min_segment_time", optimizer_params.min_segment_time);
  get_parameter("sample_spacing", optimizer_params.sample_spacing);
  get_parameter("max_velocity", optimizer_params.max_velocity);
  get_parameter("max_acceleration", optimizer_params.max_acceleration);
  get_parameter("max_jerk", optimizer_params.max_jerk);
  get_parameter("max_lateral_acceleration", optimizer_params.max_lateral_acceleration);
  get_parameter("max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  get_parameter("time_scaling_factor", optimizer_params.time_scaling_factor);
  get_parameter(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  get_parameter("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  get_parameter(
    "esdf_obstacle_trigger_clearance", optimizer_params.esdf_obstacle_trigger_clearance);
  get_parameter(
    "esdf_obstacle_target_clearance", optimizer_params.esdf_obstacle_target_clearance);
  get_parameter("esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  get_parameter(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  get_parameter(
    "guide_control_point_spacing", optimizer_params.guide_control_point_spacing);
  get_parameter("guide_smoothing_iterations", optimizer_params.guide_smoothing_iterations);
  get_parameter("guide_smoothing_alpha", optimizer_params.guide_smoothing_alpha);
  get_parameter("guide_smoothing_fidelity", optimizer_params.guide_smoothing_fidelity);
  get_parameter(
    "guide_smoothing_max_deviation", optimizer_params.guide_smoothing_max_deviation);
  get_parameter(
    "guide_smoothing_min_clearance", optimizer_params.guide_smoothing_min_clearance);
  get_parameter("joint_waypoint_spacing", optimizer_params.joint_waypoint_spacing);
  get_parameter("joint_samples_per_piece", optimizer_params.joint_samples_per_piece);
  get_parameter("joint_max_piece_time", optimizer_params.joint_max_piece_time);
  get_parameter("joint_energy_weight", optimizer_params.joint_energy_weight);
  get_parameter("joint_time_weight", optimizer_params.joint_time_weight);
  get_parameter("joint_obstacle_weight", optimizer_params.joint_obstacle_weight);
  get_parameter("joint_velocity_weight", optimizer_params.joint_velocity_weight);
  get_parameter("joint_acceleration_weight", optimizer_params.joint_acceleration_weight);
  get_parameter("joint_lateral_weight", optimizer_params.joint_lateral_weight);
  get_parameter("joint_guide_weight", optimizer_params.joint_guide_weight);
  get_parameter("joint_guide_max_deviation", optimizer_params.joint_guide_max_deviation);
  get_parameter("joint_center_clearance", optimizer_params.joint_center_clearance);
  get_parameter("joint_footprint_clearance", optimizer_params.joint_footprint_clearance);
  get_parameter("joint_footprint_edge_samples", optimizer_params.joint_footprint_edge_samples);
  get_parameter("joint_g_epsilon", optimizer_params.joint_g_epsilon);
  get_parameter("joint_max_iterations", optimizer_params.joint_max_iterations);
  get_parameter("joint_time_budget_ms", optimizer_params.joint_time_budget_ms);
  get_parameter("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  get_parameter("esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  get_parameter("esdf_obstacle_trust_region", optimizer_params.esdf_obstacle_trust_region);
  get_parameter(
    "esdf_obstacle_backtracking_steps", optimizer_params.esdf_obstacle_backtracking_steps);
  get_parameter(
    "esdf_obstacle_smoothing_weight", optimizer_params.esdf_obstacle_smoothing_weight);
  get_parameter(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  get_parameter("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  get_parameter(
    "esdf_footprint_trigger_clearance", optimizer_params.esdf_footprint_trigger_clearance);
  get_parameter(
    "esdf_footprint_target_clearance", optimizer_params.esdf_footprint_target_clearance);
  get_parameter("esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);
  get_parameter("path_duplicate_epsilon", optimizer_params.geometry_preprocessor.duplicate_epsilon);
  get_parameter(
    "path_collinear_lateral_tolerance",
    optimizer_params.geometry_preprocessor.collinear_lateral_tolerance);
  get_parameter(
    "path_short_segment_length", optimizer_params.geometry_preprocessor.short_segment_length);
  get_parameter(
    "path_corner_angle_threshold_rad",
    optimizer_params.geometry_preprocessor.corner_angle_threshold_rad);
  get_parameter(
    "path_footprint_aware_shortcut_enabled",
    optimizer_params.geometry_preprocessor.footprint_aware_shortcut_enabled);
  get_parameter(
    "path_fillet_radius", optimizer_params.geometry_preprocessor.fillet_radius);
  get_parameter(
    "path_fillet_arc_samples", optimizer_params.geometry_preprocessor.fillet_arc_samples);
  get_parameter(
    "path_shortcut_min_clearance", optimizer_params.geometry_preprocessor.shortcut_min_clearance);
  get_parameter("yaw_mode", yaw_params.mode);
  get_parameter("yaw_rate_limit", yaw_params.yaw_rate_limit);
  get_parameter("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  get_parameter("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  get_parameter("yaw_tangent_symmetry_order", yaw_params.tangent_symmetry_order);
  get_parameter("yaw_narrow_gap_bridge_time", yaw_params.narrow_gap_bridge_time);
  get_parameter("yaw_acceleration_limit", yaw_params.yaw_acceleration_limit);
  get_parameter("yaw_open_area_mode", yaw_params.open_area_yaw_mode);
  if (yaw_params.open_area_yaw_mode != "goal_heading" &&
    yaw_params.open_area_yaw_mode != "minimal_rotation")
  {
    RCLCPP_WARN(
      get_logger(), "【航向参数告警】yaw_open_area_mode=%s 无效，已回退为 goal_heading。",
      yaw_params.open_area_yaw_mode.c_str());
    yaw_params.open_area_yaw_mode = "goal_heading";
  }
  get_parameter("wheel_speed_time_scaling_limit", wheel_speed_time_scaling_params_.wheel_speed_limit);
  get_parameter(
    "wheel_speed_time_scaling_offset_x", wheel_speed_time_scaling_params_.wheel_offset_x);
  get_parameter(
    "wheel_speed_time_scaling_offset_y", wheel_speed_time_scaling_params_.wheel_offset_y);
  get_parameter(
    "wheel_speed_time_scaling_change_rate", wheel_speed_time_scaling_params_.scale_change_rate);
  get_parameter(
    "narrow_turn_speed_limit", wheel_speed_time_scaling_params_.narrow_turn_speed_limit);
  get_parameter(
    "narrow_turn_yaw_rate_threshold",
    wheel_speed_time_scaling_params_.narrow_turn_yaw_rate_threshold);
  get_parameter("clearance_speed_min", wheel_speed_time_scaling_params_.clearance_speed_min);
  get_parameter("clearance_speed_low", wheel_speed_time_scaling_params_.clearance_speed_low);
  get_parameter("clearance_speed_high", wheel_speed_time_scaling_params_.clearance_speed_high);
  // 窄通道判据与 yaw 规划共用进入阈值，两者对"窄"的定义一致。
  wheel_speed_time_scaling_params_.narrow_turn_clearance = yaw_params.narrow_clearance_enter;
  get_parameter("terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);
  get_parameter("terminal_yaw_relocation_enabled", terminal_yaw_relocation_enabled_);
  get_parameter("footprint_yaw_refinement_rounds", footprint_yaw_refinement_rounds_);
  footprint_yaw_refinement_rounds_ = std::max(0, footprint_yaw_refinement_rounds_);
  get_parameter(
    "terminal_yaw_relocation_max_candidates", terminal_yaw_relocation_max_candidates_);
  terminal_yaw_relocation_max_candidates_ = std::max(
    1, terminal_yaw_relocation_max_candidates_);
  // 重定位必须与 yaw 规划用同一组转向参数,否则两条路径生成的原地转向时长
  // 不一致,MPC 侧会看到两种不同的终端时间预算。
  terminal_yaw_relocation_params_.yaw_rate_limit = yaw_params.yaw_rate_limit;
  terminal_yaw_relocation_params_.sample_period = yaw_params.terminal_yaw_sample_period;
  get_parameter(
    "terminal_yaw_relocation_window_length",
    terminal_yaw_relocation_params_.terminal_window_length);
  get_parameter("footprint_length", footprint_params.length);
  get_parameter("footprint_width", footprint_params.width);
  get_parameter("footprint_safety_margin", footprint_params.safety_margin);
  get_parameter(
    "swept_max_corner_step_cells", footprint_params.swept_max_corner_step_cells);
  footprint_params.swept_max_corner_step_cells = std::max(
    1e-3, footprint_params.swept_max_corner_step_cells);
  footprint_length_ = footprint_params.length;
  footprint_width_ = footprint_params.width;
  footprint_safety_margin_ = footprint_params.safety_margin;
  optimizer_params.footprint_length = footprint_length_;
  optimizer_params.footprint_width = footprint_width_;
  optimizer_params.footprint_safety_margin = footprint_safety_margin_;
  const double all_yaw_footprint_radius = std::hypot(
    0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_,
    0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_);
  if (jps_params.safe_distance + 1e-6 < all_yaw_footprint_radius) {
    RCLCPP_WARN(
      get_logger(),
      "【规划参数修正】图搜索优选净空由 %.3f m 提高到矩形全航向足迹半径 %.3f m。",
      jps_params.safe_distance, all_yaw_footprint_radius);
    jps_params.safe_distance = all_yaw_footprint_radius;
  }
  // The all-yaw radius is the circumscribed disc: it demands room to spin in
  // place at every cell. A rectangular body aligned with a corridor only needs
  // the inscribed half-width, so refusing to search below the circumscribed
  // radius makes tight-but-passable corridors permanently unreachable. The
  // ladder keeps the circumscribed radius as the preferred level and only falls
  // back to the inscribed one when nothing is reachable, leaving the yaw-aware
  // footprint gate and local collision repair as the authoritative safety check.
  const double inscribed_footprint_radius =
    0.5 * std::max(0.0, std::min(footprint_length_, footprint_width_)) +
    footprint_safety_margin_;
  preferred_search_clearance_ = jps_params.safe_distance;
  get_parameter("clearance_relaxation_enabled", clearance_relaxation_enabled_);
  get_parameter(
    "endpoint_clearance_relaxation_enabled", endpoint_clearance_relaxation_enabled_);
  double configured_floor = -1.0;
  get_parameter("search_clearance_floor", configured_floor);
  inscribed_footprint_radius_ = inscribed_footprint_radius;
  search_clearance_floor_configured_ = configured_floor >= 0.0;
  // 这里还不知道规划栅格分辨率,所以自动下限先记成内切半宽;真正与 footprint gate
  // 一致的下限在 runGraphSearch 里按当次 snapshot 的分辨率补上半个格对角线。
  search_clearance_floor_ = configured_floor < 0.0
    ? std::min(inscribed_footprint_radius, preferred_search_clearance_)
    : std::min(std::max(0.0, configured_floor), preferred_search_clearance_);
  // 自动下限依赖当次 snapshot 的栅格分辨率,启动时还取不到,所以这里明确标注下限是
  // "按 snapshot 解析"的,不要让这行 INFO 被当成实际生效值。
  RCLCPP_INFO(
    get_logger(),
    "【图搜索净空梯度】优选=%.3f m 下限=%.3f m（%s）放宽=%s",
    preferred_search_clearance_, search_clearance_floor_,
    search_clearance_floor_configured_
      ? "显式配置" : "内切半径；每个快照另加栅格量化余量",
    clearance_relaxation_enabled_ ? "开启" : "关闭");
  astar_params.safe_distance = jps_params.safe_distance;
  astar_params.min_safe_distance = search_clearance_floor_;
  get_parameter("local_repair_enabled", repair_params.enabled);
  get_parameter("local_repair_max_iterations", repair_params.max_iterations);
  get_parameter("local_repair_search_radius", repair_params.search_radius);

  footprint_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  footprint_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  repair_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  repair_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  // 拒绝轨迹的是 yaw 相关矩形足迹门禁,所以修复必须按同一
  // 套几何挑格子:候选格中心周围一个外接圆半径内全空,矩形
  // 在任意 yaw 下都放得下。只按"该格空闲"会挑回原地,domain
  // 193 目标 8 就是这样每 2 s 拒同一条轨迹。找不到合格候选时
  // 返回 false,行为与今天完全一致(拒绝),方向是 fail-closed。
  repair_params.required_clearance_m = all_yaw_footprint_radius;
  // 严格档在实机栅格上常是空集(domain 169 目标 9 连续 89 次拒绝、domain 171
  // 目标 8 同样,一次都没挑出候选格)。第二档按内切半宽推 footprint 一致下限,
  // 与图搜索梯子共用几何;最终安全性仍由重解 MINCO 后的矩形足迹门禁裁定。
  repair_params.inscribed_radius_m = inscribed_footprint_radius_;
  if (repair_params.enabled &&
    repair_params.search_radius + 1e-6 < repair_params.inscribed_radius_m)
  {
    // 连第二档都够不到时修复必然是空集。这里显式告警而不是悄悄放宽搜索窗口:
    // search_radius 是用户对引导点位移的上界,由配置负责,不由代码覆盖。
    RCLCPP_WARN(
      get_logger(),
      "【局部修复参数告警】搜索半径 %.3f m 小于足迹一致净空下限（内切半径=%.3f m）；"
      "无法选出候选栅格，拒绝轨迹不会被修复。",
      repair_params.search_radius, repair_params.inscribed_radius_m);
  }

  astar_.setParams(astar_params);
  static_cast<GridAstarParams &>(jps_params) = astar_params;
  jps_.setParams(jps_params);
  // 终点净空放宽档在 const 方法里构造临时搜索器,需要一份参数副本。
  astar_params_cache_ = astar_params;
  jps_params_cache_ = jps_params;
  optimizer_.setParams(optimizer_params);
  // 4 阶对称（切线 ±pi/2 等价）只对正方形足迹成立；非正方形时 ±pi/2 会把长边横进
  // 通道，这里回退到 2 阶，fail-closed。
  const bool square_footprint = std::abs(footprint_length_ - footprint_width_) <= 1e-3;
  if (yaw_params.tangent_symmetry_order != 2 &&
    !(yaw_params.tangent_symmetry_order == 4 && square_footprint))
  {
    RCLCPP_WARN(
      get_logger(),
      "【航向参数告警】yaw_tangent_symmetry_order=%d 不适用于足迹 %.3f x %.3f m，已回退为 2。",
      yaw_params.tangent_symmetry_order, footprint_length_, footprint_width_);
    yaw_params.tangent_symmetry_order = 2;
  }
  yaw_planner_.setParams(yaw_params);
  // 缓存一份给目标位姿准入用，保证两者的矩形几何完全一致。
  footprint_params_ = footprint_params;
  safety_checker_.setParams(footprint_params);
  collision_repair_.setParams(repair_params);
}

GridAstarResult MincoPlannerNode::runGraphSearch(
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool goal_pose_footprint_verified,
  double & used_clearance)
{
  // 显式配置的下限按用户意图使用;自动下限必须补上栅格量化余量,否则梯子会稳定产出
  // footprint gate 必然拒绝的路径,把"窄通道不可通行"表现成目标超时。
  const double effective_floor = search_clearance_floor_configured_
    ? search_clearance_floor_
    : footprintConsistentClearanceFloor(
      inscribed_footprint_radius_, planning_grid.info.resolution,
      preferred_search_clearance_);
  RCLCPP_INFO_ONCE(
    get_logger(),
    "【图搜索净空生效】下限=%.3f m（内切半径=%.3f m 栅格分辨率=%.3f m 量化余量=%.3f m）；"
    "低于此值的路径无法通过矩形足迹门禁。",
    effective_floor, inscribed_footprint_radius_, planning_grid.info.resolution,
    gridQuantizationAllowance(planning_grid.info.resolution));
  std::vector<double> ladder {preferred_search_clearance_};
  if (clearance_relaxation_enabled_ && effective_floor + 1e-6 < preferred_search_clearance_) {
    ladder.push_back(effective_floor);
  }

  GridAstarResult last_result;
  for (const double clearance : ladder) {
    GridAstarResult attempt;
    if (search_algorithm_ == "jps") {
      attempt = jps_.planWithClearance(planning_grid, start, goal, clearance);
      if (!attempt.success && astar_fallback_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【规划回退】JPS 失败（jps failed: clearance=%.3f m reason=%s），改用 A*。",
          clearance, attempt.reason.c_str());
        attempt = astar_.planWithClearance(planning_grid, start, goal, clearance);
      }
    } else {
      attempt = astar_.planWithClearance(planning_grid, start, goal, clearance);
    }
    if (attempt.success) {
      used_clearance = clearance;
      if (clearance + 1e-6 < preferred_search_clearance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【规划净空放宽】图搜索从 %.3f m 放宽到 %.3f m 后成功；最终仍以航向感知足迹门禁为准。",
          preferred_search_clearance_, clearance);
      }
      return attempt;
    }
    last_result = attempt;
  }

  // 最后一档:目标格净空低于 footprint 一致下限,但目标位姿本身已通过同一套矩形
  // 足迹门禁。目标格净空是各向同性代理量,矩形门禁是精确判定;此时继续用代理量
  // 否决精确判定,结果是一条路径都产不出来(domain 187 目标 9:目标格净空约 0.26 m
  // 对下限 0.341 m,图搜索报 goal occupied,7 个规划周期零可执行计划直到看门狗耗尽)。
  // 放宽只作用于"目标格是否可作为终点",搜索器随后按实测目标净空重规划整条路径,
  // 安全性仍由重解 MINCO 之后的矩形足迹门禁与局部修复裁定,不是靠放宽判据换成功。
  const double relaxed_endpoint_min = gridQuantizationAllowance(planning_grid.info.resolution);
  if (goal_pose_footprint_verified && endpoint_clearance_relaxation_enabled_ &&
    !last_result.success && relaxed_endpoint_min + 1e-6 < effective_floor)
  {
    GridAstarResult attempt;
    if (search_algorithm_ == "jps") {
      GridJpsParams relaxed = jps_params_cache_;
      relaxed.min_safe_distance = relaxed_endpoint_min;
      relaxed.relax_endpoint_clearance = true;
      attempt = GridJps(relaxed).planWithClearance(planning_grid, start, goal, effective_floor);
      if (!attempt.success && astar_fallback_) {
        GridAstarParams relaxed_astar = astar_params_cache_;
        relaxed_astar.min_safe_distance = relaxed_endpoint_min;
        relaxed_astar.relax_endpoint_clearance = true;
        attempt =
          GridAstar(relaxed_astar).planWithClearance(planning_grid, start, goal, effective_floor);
      }
    } else {
      GridAstarParams relaxed_astar = astar_params_cache_;
      relaxed_astar.min_safe_distance = relaxed_endpoint_min;
      relaxed_astar.relax_endpoint_clearance = true;
      attempt =
        GridAstar(relaxed_astar).planWithClearance(planning_grid, start, goal, effective_floor);
    }
    if (attempt.success) {
      used_clearance = effective_floor;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终点净空放宽】终点下限从 %.3f m 放宽到 %.3f m 后成功；目标已通过矩形足迹门禁，"
        "轨迹安全仍由该门禁最终裁定。",
        effective_floor, relaxed_endpoint_min);
      return attempt;
    }
    last_result = attempt;
  }

  used_clearance = ladder.back();
  return last_result;
}

void MincoPlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::uint64_t candidate_generation = 0;
  std::uint64_t candidate_health_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    candidate_generation = next_map_generation_ + 1;
    candidate_health_epoch = map_health_epoch_;
  }
  const auto snapshot = PlanningMapSnapshot::create(
    candidate_generation, *msg, obstacle_value_threshold_, unknown_is_obstacle_);
  if (!snapshot) {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      invalidateMapLocked();
      publishEmergencyStop(true);
    }
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【规划栅格错误】收到无效规划栅格，已拒绝并急停。");
    return;
  }
  // 可选：安全语义变化的新 snapshot 若仍让已提交参考的完整剩余段通过同一套矩形足迹
  // 门禁（含提交门同源的擦边逃逸规则），就把参考改绑到新 generation 继续执行，而不是
  // 每次地图刷新都作废参考、急停、等 GoalManager 重派。判定在锁外对不可变 snapshot
  // 完成，锁内再核对参考身份未变；任何不一致都回到原先的作废路径（fail-closed）。
  std::optional<ActiveSafetyReference> retain_candidate;
  if (retain_safe_reference_on_snapshot_change_) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (active_safety_reference_ && latest_map_snapshot_ && safety_state_.plan_safe &&
      !snapshot->hasSameSafetyContent(*latest_map_snapshot_))
    {
      retain_candidate = active_safety_reference_;
    }
  }
  std::string retain_reason;
  const bool retain_verdict = retain_candidate &&
    retain_candidate->trajectory.header.frame_id == snapshot->grid.header.frame_id &&
    remainingReferenceSafeOn(retain_candidate->trajectory, snapshot->grid, &retain_reason);
  // 远端冲突、近端安全：继续执行并建议重规划，交给 10 Hz 运行期复检兜底近端。
  const bool advise_replan = replan_advisory_enabled_ && retain_candidate && !retain_verdict &&
    runtimeWindowSafeOn(retain_candidate->trajectory, snapshot->grid);
  std::optional<ActiveSafetyReference> advised_reference;
  bool retained_reference = false;
  std::uint64_t retained_reference_goal = 0;
  bool retained_identical_snapshot = false;
  std::uint64_t retained_generation = 0;
  std::string retained_digest;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (
      candidate_health_epoch != map_health_epoch_ ||
      candidate_generation != next_map_generation_ + 1)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【规划栅格丢弃】地图健康状态在构建期间变化，已丢弃本次栅格。");
      return;
    }
    const bool replaces_existing_snapshot = latest_map_snapshot_ != nullptr;
    retained_identical_snapshot =
      replaces_existing_snapshot && snapshot->hasSameSafetyContent(*latest_map_snapshot_);
    if (retained_identical_snapshot) {
      retained_generation = latest_map_snapshot_->generation;
      retained_digest = latest_map_snapshot_->safety_content_digest;
    } else {
      retained_reference = (retain_verdict || advise_replan) && replaces_existing_snapshot &&
        active_safety_reference_ && safety_state_.plan_safe &&
        safety_state_.mapSnapshotUsable(snapshot->generation) &&
        active_safety_reference_->goal_id == retain_candidate->goal_id &&
        active_safety_reference_->localization_epoch == retain_candidate->localization_epoch &&
        active_safety_reference_->plan_request_sequence ==
          retain_candidate->plan_request_sequence &&
        active_safety_reference_->map_generation == retain_candidate->map_generation &&
        active_safety_reference_->trajectory.header.stamp ==
          retain_candidate->trajectory.header.stamp;
      if (retained_reference) {
        active_safety_reference_->map_generation = snapshot->generation;
        retained_reference_goal = active_safety_reference_->goal_id;
        if (!retain_verdict && (!replan_advised_reference_stamp_ ||
          *replan_advised_reference_stamp_ != active_safety_reference_->trajectory.header.stamp))
        {
          replan_advised_reference_stamp_ = active_safety_reference_->trajectory.header.stamp;
          advised_reference = active_safety_reference_;
        }
      } else {
        active_safety_reference_.reset();
      }
      next_map_generation_ = snapshot->generation;
      latest_map_snapshot_ = snapshot;
      if (replaces_existing_snapshot && !retained_reference) {
        // A safety-semantic change creates a new immutable map identity.  The
        // old reference cannot execute even though the source heartbeat stays
        // healthy; a fresh plan must bind this exact local generation.
        safety_state_.invalidatePlanForNewMap(snapshot->generation);
        invalidatePlannerStatusLocked(
          ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
          snapshot->generation);
      }
      if (map_ready_topic_.empty()) {
        safety_state_.map_ready = true;
      }
      publishEmergencyStop(safety_state_.emergencyStopRequired());
    }
  }
  if (retain_candidate && !retained_reference && !retained_identical_snapshot) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【参考轨迹失效】地图快照变化后无法保留目标=%llu 的已提交参考（判定=%d 前视窗=%.2f s 原因=%s），"
      "将在 generation=%llu 上重新规划。",
      static_cast<unsigned long long>(retain_candidate->goal_id),
      static_cast<int>(retain_verdict), retain_reference_horizon_sec_,
      retain_reason.empty() ? "身份或帧不一致" : retain_reason.c_str(),
      static_cast<unsigned long long>(snapshot->generation));
  }
  if (advised_reference) {
    // 直接发布、不进入 planner_status_state_：它不是该请求的终态，不能被后续作废复用。
    // STATE_ACCEPTED 不会改变 MPC/arbiter 的地图授权。
    if (planner_status_pub_) {
      ats_navigation_interfaces::msg::PlannerStatus status;
      status.header.stamp = now();
      status.header.frame_id = global_frame_;
      status.goal_id = advised_reference->goal_id;
      status.localization_epoch = advised_reference->localization_epoch;
      status.plan_request_sequence = advised_reference->plan_request_sequence;
      status.map_generation = snapshot->generation;
      status.map_publication_sequence = advised_reference->map_publication_sequence;
      status.state = ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED;
      status.failure_reason = ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REPLAN_ADVISED;
      planner_status_pub_->publish(status);
    }
    RCLCPP_WARN(
      get_logger(),
      "【建议重规划】目标=%llu 的参考在 generation=%llu 上远端冲突（%s），运行期窗口 %.2f s 内仍安全，"
      "继续执行并请求无停车重规划。",
      static_cast<unsigned long long>(advised_reference->goal_id),
      static_cast<unsigned long long>(snapshot->generation),
      retain_reason.empty() ? "未知" : retain_reason.c_str(), runtime_safety_horizon_sec_);
  }
  if (retained_reference) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "【参考轨迹保留】目标=%llu 跨地图快照变化继续执行，剩余轨迹通过 generation=%llu 的足迹门禁。",
      static_cast<unsigned long long>(retained_reference_goal),
      static_cast<unsigned long long>(snapshot->generation));
    return;
  }
  if (retained_identical_snapshot) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "【快照复用】安全内容未变，继续使用不可变规划快照 generation=%llu digest=%s；"
      "adapter/source generation 仍仅作为心跳证据。",
      static_cast<unsigned long long>(retained_generation), retained_digest.c_str());
    return;
  }
}

void MincoPlannerNode::onMapReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!msg->data) {
      invalidateMapLocked();
    } else {
      safety_state_.map_ready = true;
    }
    last_map_ready_signal_ = std::chrono::steady_clock::now();
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
}

void MincoPlannerNode::onMapReadyWatchdog()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (
      !map_ready_topic_.empty() && safety_state_.map_ready &&
      !steadyHeartbeatLeaseValid(
        last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_))
    {
      invalidateMapLocked();
      timed_out = true;
    }
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
  if (timed_out) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【地图租约超时】规划地图 ready 心跳超时，已进入急停。");
  }
}

bool MincoPlannerNode::remainingReferenceSafeOn(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
  std::string * reason)
{
  if (trajectory.points.size() < 2) {
    if (reason) {
      *reason = "参考点不足";
    }
    return false;
  }
  const rclcpp::Time reference_stamp(trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  // 保留窗口必须覆盖运行期复检窗口,否则改绑后下一拍复检就可能作废。
  const double progress = referenceProgress(trajectory, grid, elapsed);
  const double horizon_end = retain_reference_horizon_sec_ > 0.0 ?
    progress + std::max(retain_reference_horizon_sec_, runtime_safety_horizon_sec_) :
    std::numeric_limits<double>::infinity();
  const double window_start = std::max(0.0, progress - reference_window_backward_sec_);
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    trajectory, window_start, horizon_end);
  if (remaining.points.size() < 2) {
    if (reason) {
      std::ostringstream text;
      text << "剩余参考不足两点 墙钟=" << elapsed << "s 起点=" << window_start <<
        "s 参考时长=" << trajectory.points.back().t << "s";
      *reason = text.str();
    }
    return false;
  }
  const FootprintSafetyResult safety = safety_checker_.check(remaining, grid);
  if (safety.safe) {
    return true;
  }
  const EscapePrefixDecision escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  const bool shallow = escapePrefixContactShallow(
    remaining, escape, footprint_params_, grid, escape_prefix_params_);
  if (!shallow && reason) {
    std::ostringstream text;
    const CollisionSample & first = safety.collisions.front();
    const ReferencePoint & center =
      remaining.points[std::min(first.trajectory_index, remaining.points.size() - 1U)];
    text << "窗口足迹冲突=" << safety.collisions.size() << " 墙钟=" << elapsed << "s 起点=" <<
      window_start << "s 首冲突序号=" << first.trajectory_index << " 中心=(" << center.x << "," <<
      center.y << ") 起点位置=(" << remaining.points.front().x << "," <<
      remaining.points.front().y << ")";
    *reason = text.str();
  }
  return shallow;
}

bool MincoPlannerNode::runtimeWindowSafeOn(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid)
{
  if (trajectory.points.size() < 2 || trajectory.header.frame_id != grid.header.frame_id) {
    return false;
  }
  const rclcpp::Time reference_stamp(trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  const double progress = referenceProgress(trajectory, grid, elapsed);
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    trajectory, std::max(0.0, progress - reference_window_backward_sec_),
    progress + runtime_safety_horizon_sec_);
  if (remaining.points.size() < 2) {
    return false;
  }
  const FootprintSafetyResult safety = safety_checker_.check(remaining, grid);
  if (safety.safe) {
    return true;
  }
  const EscapePrefixDecision escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  return escapePrefixContactShallow(
    remaining, escape, footprint_params_, grid, escape_prefix_params_);
}

double MincoPlannerNode::referenceProgress(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
  double elapsed)
{
  if (!reference_window_from_robot_progress_ ||
    trajectory.header.frame_id != grid.header.frame_id)
  {
    return elapsed;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!lookupStartPose(grid, pose)) {
    return elapsed;
  }
  // 车沿参考的实际推进（不超过墙钟）。MPC 按几何投影跟踪，拐角减速后车落后于墙钟；
  // 墙钟窗口会跳过车前方还没走的那段参考（漏检），又把车身后很远的冲突算进"前视窗"。
  // 保留检查与运行期复检都以它为锚点，窗口语义是"车前方 horizon 秒的参考"。
  return std::min(elapsed, projectedReferenceTime(
    trajectory, pose.pose.position.x, pose.pose.position.y, elapsed));
}

void MincoPlannerNode::onRuntimeSafetyRecheck()
{
  std::optional<ActiveSafetyReference> active_reference;
  std::shared_ptr<const PlanningMapSnapshot> snapshot;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ || !latest_map_snapshot_ ||
      !safety_state_.mapSnapshotUsable(latest_map_snapshot_->generation))
    {
      return;
    }
    active_reference = active_safety_reference_;
    snapshot = latest_map_snapshot_;
  }

  if (active_reference->trajectory.header.frame_id != snapshot->grid.header.frame_id ||
    active_reference->trajectory.points.size() < 2)
  {
    return;
  }
  const rclcpp::Time reference_stamp(active_reference->trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  const double progress = referenceProgress(active_reference->trajectory, snapshot->grid, elapsed);
  const double horizon_end = progress + runtime_safety_horizon_sec_;
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    active_reference->trajectory, std::max(0.0, progress - reference_window_backward_sec_),
    horizon_end);
  if (remaining.points.size() < 2) {
    return;
  }

  const FootprintSafetyResult safety = safety_checker_.check(remaining, snapshot->grid);
  if (safety.safe) {
    return;
  }
  // remaining.points[0] 是车当前跟踪到的位姿，所以这道运行期门和提交门面对同一个边界：
  // 因"车现在所在的位姿被占据"而撤销轨迹，会立刻把正在执行的逃逸动作掐掉，车留在接触
  // 里不动。判据与提交门同源——只有冲突紧贴 remaining 起点、且有界前缀之后不再有冲突时
  // 才放行；车一旦沿轨迹驶离，head_offset 增大，这个放行自然停止生效。
  const EscapePrefixDecision runtime_escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  if (escapePrefixContactShallow(
      remaining, runtime_escape, footprint_params_, snapshot->grid, escape_prefix_params_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "【接触脱困】运行期扫掠门禁下保留脱困参考：冲突=%zu 起始偏移=%.3f m 前缀末端=%zu 长度=%.3f m。",
      runtime_escape.collision_count, runtime_escape.head_offset_m,
      runtime_escape.prefix_end, runtime_escape.prefix_length_m);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ ||
      active_safety_reference_->goal_id != active_reference->goal_id ||
      active_safety_reference_->localization_epoch != active_reference->localization_epoch ||
      active_safety_reference_->plan_request_sequence != active_reference->plan_request_sequence ||
      active_safety_reference_->map_generation != active_reference->map_generation ||
      active_safety_reference_->trajectory.header.stamp != active_reference->trajectory.header.stamp)
    {
      return;
    }
    active_safety_reference_.reset();
    safety_state_.plan_safe = false;
    publishEmergencyStop(true);
    publishPlannerStatusLocked(
      active_reference->goal_id, active_reference->localization_epoch,
      active_reference->plan_request_sequence, snapshot->generation,
      active_reference->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_RUNTIME_UNSAFE);
  }
  const bool has_collision = !safety.collisions.empty();
  const std::size_t collision_index = has_collision ?
    std::min(safety.collisions.front().trajectory_index, remaining.points.size() - 1U) : 0U;
  const ReferencePoint & collision_reference = remaining.points[collision_index];
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), log_throttle_ms_,
    "【运行期安全拒绝】扫掠足迹拒绝（goal=%llu candidate_generation=%llu "
    "current_generation=%llu collisions=%zu discrete_samples=%zu swept_samples=%zu "
    "first_index=%zu first_swept=%d first_collision=(%.3f,%.3f) "
    "remaining_start=(%.3f,%.3f,%.3f) reference_center=(%.3f,%.3f,%.3f)）。",
    static_cast<unsigned long long>(active_reference->goal_id),
    static_cast<unsigned long long>(active_reference->map_generation),
    static_cast<unsigned long long>(snapshot->generation), safety.collisions.size(),
    safety.discrete_samples_checked, safety.swept_samples_checked, collision_index,
    has_collision && safety.collisions.front().swept ? 1 : 0,
    has_collision ? safety.collisions.front().x : 0.0,
    has_collision ? safety.collisions.front().y : 0.0,
    remaining.points.front().x, remaining.points.front().y, remaining.points.front().yaw,
    collision_reference.x, collision_reference.y, collision_reference.yaw);
}

void MincoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // P2/P3 过渡兼容入口：P2 保持直接 PoseStamped，不携带任务生命周期编号。
  planGoal(*msg, 0, 0, 0, 0, false);
}

void MincoPlannerNode::onPlannerGoal(
  const ats_navigation_interfaces::msg::PlannerGoal::SharedPtr msg)
{
  if (msg->goal_id == 0 || msg->plan_request_sequence == 0) {
    publishPlannerStatus(
        0, msg->localization_epoch, msg->plan_request_sequence, 0,
        msg->map_publication_sequence,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_INVALID_GOAL);
    return;
  }
  InitialKinematicState frozen_seed;
  const geometry_msgs::msg::PoseStamped * frozen_start = nullptr;
  if (!msg->start_pose.header.frame_id.empty()) {
    if (!finitePose(msg->start_pose.pose) ||
      !std::isfinite(msg->start_twist.linear.x) ||
      !std::isfinite(msg->start_twist.linear.y) ||
      !std::isfinite(msg->start_twist.linear.z) ||
      !std::isfinite(msg->start_twist.angular.x) ||
      !std::isfinite(msg->start_twist.angular.y) ||
      !std::isfinite(msg->start_twist.angular.z))
    {
      publishPlannerStatus(
          msg->goal_id, msg->localization_epoch, msg->plan_request_sequence, 0,
          msg->map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
          ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER);
      return;
    }
    frozen_start = &msg->start_pose;
    frozen_seed.valid = true;
    frozen_seed.velocity = Eigen::Vector2d(
      msg->start_twist.linear.x, msg->start_twist.linear.y);
  }
  publishPlannerStatus(
      msg->goal_id, msg->localization_epoch, msg->plan_request_sequence, 0,
      msg->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE);
  planGoal(
    msg->goal_pose, msg->goal_id, msg->localization_epoch,
    msg->plan_request_sequence,
    msg->map_publication_sequence, true, frozen_start,
    frozen_seed.valid ? &frozen_seed : nullptr);
}

void MincoPlannerNode::planGoal(
    const geometry_msgs::msg::PoseStamped &input_goal, std::uint64_t goal_id,
    std::uint64_t localization_epoch, std::uint64_t plan_request_sequence,
    std::uint64_t map_publication_sequence,
    bool report_status,
    const geometry_msgs::msg::PoseStamped * frozen_start,
    const InitialKinematicState * initial_state) {
  bool keep_active_reference = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    // 由【建议重规划】触发的同目标新请求：旧参考仍在执行，保留它的运行期复检守护与
    // plan_safe，直到新参考提交（publishReferenceIfCurrent 替换）或本次规划失败。
    keep_active_reference = replan_advised_reference_stamp_ && active_safety_reference_ &&
      safety_state_.plan_safe && active_safety_reference_->goal_id == goal_id &&
      active_safety_reference_->localization_epoch == localization_epoch &&
      active_safety_reference_->plan_request_sequence < plan_request_sequence &&
      active_safety_reference_->trajectory.header.stamp == *replan_advised_reference_stamp_;
    if (!keep_active_reference) {
      active_safety_reference_.reset();
    }
  }
  if (!keep_active_reference) {
    setPlanSafe(false);
  }
  const auto fail = [this, goal_id, localization_epoch, plan_request_sequence,
                     map_publication_sequence, report_status, keep_active_reference](
                        std::uint8_t failure_reason, std::uint64_t generation) {
    if (keep_active_reference) {
      // 无停车重规划失败：旧参考仍在执行且受 10 Hz 运行期复检守护，不撤销它。失败以
      // STATE_ACCEPTED 发布（MPC/arbiter 忽略，不停车），GoalManager 清除替换标记；下一次
      // 地图变化若仍远端冲突会再次建议重规划。旧参考近端失效时运行期复检照常急停。
      bool still_guarded = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        still_guarded = active_safety_reference_.has_value() && safety_state_.plan_safe;
        replan_advised_reference_stamp_.reset();
        if (still_guarded && planner_status_pub_ && report_status) {
          ats_navigation_interfaces::msg::PlannerStatus status;
          status.header.stamp = now();
          status.header.frame_id = global_frame_;
          status.goal_id = goal_id;
          status.localization_epoch = localization_epoch;
          status.plan_request_sequence = plan_request_sequence;
          status.map_generation = generation != 0 ? generation : next_map_generation_;
          status.map_publication_sequence = map_publication_sequence;
          status.state = ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED;
          status.failure_reason = failure_reason;
          planner_status_pub_->publish(status);
        }
      }
      if (still_guarded) {
        RCLCPP_WARN(
          get_logger(),
          "【无停车重规划失败】目标=%llu 请求=%llu 原因=%u，继续执行受运行期复检守护的旧参考。",
          static_cast<unsigned long long>(goal_id),
          static_cast<unsigned long long>(plan_request_sequence),
          static_cast<unsigned>(failure_reason));
        return;
      }
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      active_safety_reference_.reset();
    }
    setPlanSafe(false);
    if (report_status) {
      publishPlannerStatus(
          goal_id, localization_epoch, plan_request_sequence, generation,
          map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED, failure_reason);
    }
  };
  std::shared_ptr<const PlanningMapSnapshot> map_snapshot;
  std::uint64_t map_health_epoch = 0;
  bool map_ready = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_snapshot = latest_map_snapshot_;
    map_health_epoch = map_health_epoch_;
    map_ready = map_snapshot && safety_state_.mapSnapshotUsable(map_snapshot->generation);
  }
  if (!map_ready || !map_snapshot) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【等待地图】尚未收到可通行栅格，规划保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY, 0);
    return;
  }
  const auto & planning_grid = map_snapshot->grid;
  const auto & clearance_esdf = map_snapshot->clearance_esdf;
  InitialKinematicState frozen_seed;
  const InitialKinematicState * seed_ptr = nullptr;
  if (initial_state && initial_state->valid) {
    frozen_seed = *initial_state;
    seed_ptr = &frozen_seed;
  }

  geometry_msgs::msg::PoseStamped start;
  if (!resolveStartPose(planning_grid, frozen_start, start)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】无法获取起点位姿，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_START_TF,
      map_snapshot->generation);
    return;
  }

  geometry_msgs::msg::PoseStamped goal = input_goal;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id =
      planning_grid.header.frame_id.empty() ? global_frame_ : planning_grid.header.frame_id;
  }
  geometry_msgs::msg::PoseStamped goal_in_grid;
  if (!transformGoalToGrid(planning_grid, goal, goal_in_grid)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】目标位姿转换失败，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_GOAL_TF,
      map_snapshot->generation);
    return;
  }
  goal = goal_in_grid;

  // 目标位姿准入。action 的成功判据是一个容差域，而不是一个点：目标点足迹与栅格墙面
  // 重叠几毫米时，容差域内往往仍有大量完全可行的位姿。此前规划器只认那个点，于是提交门
  // 正确拒绝每一条轨迹，goal manager 反复重试直到预算耗尽（domain 175 目标 8 实测：
  // 车已停在目标 0.16 m 处，只差一个可行的终点标注）。这里在容差域内挑一个可行终点，
  // 候选过的是与提交门完全相同的足迹判定，偏移严格小于容差，所以 SUCCEEDED 依然由原目标
  // 的判据给出，不是靠放宽判据换来的。找不到可行位姿时保持原目标不变，走既有失败路径。
  // 终点净空放宽档只有在目标位姿确实过了矩形足迹门禁时才允许启用;准入关闭时
  // 保持 false,行为与今天完全一致。
  bool goal_pose_footprint_verified = false;
  if (goal_pose_admission_params_.enabled) {
    const double commanded_goal_yaw = tf2::getYaw(goal.pose.orientation);
    const GoalPoseAdmissionResult admission = admitGoalPose(
      goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
      planning_grid, footprint_params_, goal_pose_admission_params_);
    if (!admission.feasible) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "【目标准入】成功容差内没有可行位姿：目标=(%.3f, %.3f, yaw=%.3f) "
        "候选数=%zu；保留原目标。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.candidates_checked);
    }
    goal_pose_footprint_verified = admission.feasible;
    if (admission.feasible && admission.relocated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【目标准入】在成功容差内找到可行位姿：原目标=(%.3f, %.3f, yaw=%.3f) "
        "采用=(%.3f, %.3f, yaw=%.3f) 位置偏差=%.3f m 航向偏差=%.3f rad "
        "额外净空=%d 候选数=%zu。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.x, admission.y, admission.yaw, admission.position_deviation_m,
        admission.yaw_deviation_rad, admission.used_preferred_margin ? 1 : 0,
        admission.candidates_checked);
      goal.pose.position.x = admission.x;
      goal.pose.position.y = admission.y;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = std::sin(admission.yaw * 0.5);
      goal.pose.orientation.w = std::cos(admission.yaw * 0.5);
    }
  }

  // 起点来自 TF、终点来自 goal_topic，因此此分支不依赖 Smac 的路径几何。
  double used_clearance = preferred_search_clearance_;
  const GridAstarResult search_result =
    runGraphSearch(
    planning_grid, start, goal, goal_pose_footprint_verified, used_clearance);
  if (!search_result.success) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划失败】%s 失败（failed: %s expanded=%d clearance=%.3f m）",
      search_algorithm_.c_str(), search_result.reason.c_str(),
      search_result.expanded_nodes, used_clearance);
    const std::uint8_t failure_reason =
      classifyGraphSearchFailure(search_result.reason, search_result.expanded_nodes);
    fail(failure_reason, map_snapshot->generation);
    return;
  }

  // The raw JPS route, guide stages and final candidate all come from this
  // immutable snapshot.  The guide shortcut delegates collision semantics to
  // the exact oriented footprint/swept checker used by the final gate.
  // These are diagnostic-only products. They deliberately publish before the
  // candidate gate so a fail-closed rejection can still be attributed to a
  // geometry, ESDF, time-allocation, or dynamic-limit stage.
  if (raw_path_pub_) {
    raw_path_pub_->publish(search_result.path);
  }
  MincoOptimizationTrace selected_trace;
  ReferenceTrajectory center_reference = optimizer_.optimize(
    search_result.path, clearance_esdf.get(), nullptr, seed_ptr, &planning_grid,
    &safety_checker_, &selected_trace);
  if (preprocessed_guide_pub_ && !selected_trace.preprocessed_guide.poses.empty()) {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && !selected_trace.esdf_refined_guide.poses.empty()) {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!center_reference.valid()) {
    std::ostringstream durations;
    durations.setf(std::ios::fixed);
    durations.precision(3);
    for (std::size_t index = 0; index < selected_trace.segment_durations.size(); ++index) {
      if (index > 0U) {
        durations << ',';
      }
      durations << selected_trace.segment_durations[index];
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】候选被拒绝（candidate rejected generation=%llu snapshot_publication=%llu stage=%s "
      "raw_points=%zu preprocessed_points=%zu esdf_refined_points=%zu peak_v=%.3f "
      "peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f joint_termination=%s joint_iterations=%d "
      "joint_wall_ms=%.3f segment_durations=[%s]）",
      static_cast<unsigned long long>(map_snapshot->generation),
      static_cast<unsigned long long>(map_publication_sequence),
      selected_trace.failure_reason.empty() ? "unknown" : selected_trace.failure_reason.c_str(),
      search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
      selected_trace.esdf_refined_guide.poses.size(), selected_trace.peak_velocity,
      selected_trace.peak_acceleration, selected_trace.peak_jerk,
      selected_trace.solver_wall_time_ms,
      selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
      selected_trace.joint_iterations, selected_trace.joint_wall_time_ms, durations.str().c_str());
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  center_reference.header.stamp = now();
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  planYaw(center_reference, *map_snapshot, start_yaw, goal_yaw);
  FootprintSafetyResult center_safety = safety_checker_.check(center_reference, planning_grid);
  ReferenceTrajectory reference = center_reference;
  FootprintSafetyResult safety = center_safety;
  // 足迹罚项用的是上一条轨迹的 yaw，而 planYaw 会按新轨迹重算 yaw：两者不一致时，
  // 角点可落进薄墙尖（优化器看不到、门禁拒绝）。用刚算出的 yaw 作参考再解，
  // 直到门禁通过或轮数用尽；每轮结果都过同一矩形门禁，不放宽任何判据，
  // 碰撞数变多即停并保留上一轮。
  const auto refineFootprintYaw = [&](
    const nav_msgs::msg::Path & guide, ReferenceTrajectory & candidate,
    FootprintSafetyResult & candidate_safety, MincoOptimizationTrace & candidate_trace) {
      for (int round = 0; !candidate_safety.safe && round < footprint_yaw_refinement_rounds_;
        ++round)
      {
        MincoOptimizationTrace refined_trace;
        ReferenceTrajectory refined = optimizer_.optimize(
          guide, clearance_esdf.get(), &candidate, seed_ptr, &planning_grid, &safety_checker_,
          &refined_trace);
        if (!refined.valid()) {
          return;
        }
        refined.header.stamp = now();
        planYaw(refined, *map_snapshot, start_yaw, goal_yaw);
        FootprintSafetyResult refined_safety = safety_checker_.check(refined, planning_grid);
        if (refined_safety.collisions.size() > candidate_safety.collisions.size()) {
          return;
        }
        candidate = std::move(refined);
        candidate_safety = std::move(refined_safety);
        candidate_trace = std::move(refined_trace);
      }
    };
  if (optimizer_.esdfFootprintOptimizationEnabled()) {
    MincoOptimizationTrace footprint_trace;
    ReferenceTrajectory footprint_reference = optimizer_.optimize(
      search_result.path, clearance_esdf.get(), &center_reference, seed_ptr, &planning_grid,
      &safety_checker_, &footprint_trace);
    if (footprint_reference.valid()) {
      footprint_reference.header.stamp = now();
      planYaw(footprint_reference, *map_snapshot, start_yaw, goal_yaw);
      FootprintSafetyResult footprint_safety = safety_checker_.check(
        footprint_reference, planning_grid);
      refineFootprintYaw(
        search_result.path, footprint_reference, footprint_safety, footprint_trace);
      if (footprint_safety.safe || !center_safety.safe) {
        reference = std::move(footprint_reference);
        safety = std::move(footprint_safety);
        selected_trace = std::move(footprint_trace);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【足迹候选回退】RC-ESDF 足迹候选有 %zu 个冲突，改用安全的中心 ESDF 候选。",
          footprint_safety.collisions.size());
      }
    }
  }
  bool esdf_fallback_used = false;
  if (!safety.safe && optimizer_.esdfObstacleOptimizationEnabled()) {
    // 外推候选仍碰撞时回到不做 ESDF 位移的 JPS-MINCO，避免“修正越修越差”。
    MincoOptimizationTrace fallback_trace;
    ReferenceTrajectory fallback_reference = optimizer_.optimize(
      search_result.path, nullptr, nullptr, seed_ptr, &planning_grid, &safety_checker_,
      &fallback_trace);
    if (fallback_reference.valid()) {
      fallback_reference.header.stamp = now();
      planYaw(fallback_reference, *map_snapshot, start_yaw, goal_yaw);
      const FootprintSafetyResult fallback_safety = safety_checker_.check(
        fallback_reference, planning_grid);
      if (fallback_safety.safe) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【足迹候选回退】RC-ESDF 外层候选有 %zu 个足迹冲突，改用安全的 JPS-MINCO 基线。",
          safety.collisions.size());
        esdf_fallback_used = true;
        reference = std::move(fallback_reference);
        safety = fallback_safety;
        selected_trace = std::move(fallback_trace);
      }
    }
  }
  LocalCollisionRepairStats repair_stats;
  // repair() 就地改引导点,所以要先留一份修复前的轨迹:重解 MINCO 失败时
  // 必须能退回原状。以前这条路走不到(严格档恒为空集),现在会真的走到,
  // 而 FAILURE_REPAIR 在 goal manager 里不是瞬时故障,会立刻判死目标。
  // 保证"尝试修复"永远不比"不尝试"更差。
  ReferenceTrajectory pre_repair_reference;
  if (!safety.safe) {
    pre_repair_reference = reference;
  }
  const bool repair_changed = !safety.safe &&
    collision_repair_.repair(reference, safety, planning_grid, &repair_stats);
  if (!safety.safe) {
    // repair() 自身没有 logger。没有这一行时无法区分"没尝试""挑不出候选格"
    // 和"挑出来但位移可忽略",domain 169 的 89 次连续拒绝就完全没有痕迹。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【局部碰撞修复】changed=%d points=%zu strict=%zu fallback=%zu "
      "no_candidate=%zu negligible=%zu endpoint_protected=%zu strict_clearance=%.3f "
      "fallback_clearance=%.3f search_radius=%.3f m.",
      repair_changed ? 1 : 0, repair_stats.collision_points,
      repair_stats.strict_repaired, repair_stats.fallback_repaired,
      repair_stats.no_candidate, repair_stats.negligible_shift,
      repair_stats.endpoint_protected,
      repair_stats.strict_clearance_m, repair_stats.fallback_clearance_m,
      repair_stats.effective_search_radius_m);
  }
  bool local_repair_used = false;
  if (repair_changed) {
    // 局部修复只改变几何引导线，必须重新求 MINCO、yaw 和最终矩形足迹安全性。
    MincoOptimizationTrace repair_trace;
    ReferenceTrajectory repaired = optimizer_.optimize(
      toPath(reference), clearance_esdf.get(), &reference, seed_ptr, &planning_grid,
      &safety_checker_, &repair_trace);
    if (!repaired.valid()) {
      // 退回修复前轨迹,交给下面既有的 footprint 拒绝路径。那条路径报
      // FAILURE_FOOTPRINT(瞬时,等下一次 snapshot 重规划),而 FAILURE_REPAIR
      // 会让目标直接失败——修复只是一次尝试,失败不该比没尝试更严重。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【局部碰撞修复失败】修复后 MINCO 轨迹无效，保留修复前轨迹并进入既有足迹拒绝路径。");
      reference = std::move(pre_repair_reference);
    } else {
      local_repair_used = true;
      const nav_msgs::msg::Path repair_guide = toPath(reference);
      reference = std::move(repaired);
      reference.header.stamp = now();
      planYaw(reference, *map_snapshot, start_yaw, goal_yaw);
      safety = safety_checker_.check(reference, planning_grid);
      refineFootprintYaw(repair_guide, reference, safety, repair_trace);
      selected_trace = std::move(repair_trace);
    }
  }

  // 终端原地转向重定位。冲突全部落在终点原地转向段时,换一个更早的转向位置:
  // 先在路径上某点把 yaw 拧到 goal_yaw,再保持 goal_yaw 平移进入目标。
  // 候选族的最晚一个(尾段起点)与现状完全等价,所以这是现状的超集;每个候选都
  // 要过同一套矩形足迹门禁,不安全就继续走既有拒绝路径。因此既不会放过不安全
  // 轨迹,也不会比不做更差。目标位姿本身不被修改,只改到达目标的 yaw 时序。
  // 触发条件用"终端近域窗口"而不是"与末点严格重合的原地转向段":domain 189
  // 目标 9 的冲突落在最后 6 个采样点,其中前几个仍在以通道切线 yaw 平移,坐标
  // 与末点差 0.014 m,旧判据因此返回 false,重定位一次都没试过。窗口之外只要
  // 有一个冲突仍然返回 false,中途不可行照旧交回既有拒绝路径。
  if (!safety.safe && terminal_yaw_relocation_enabled_ &&
    collisionsConfinedToTerminalApproach(
      reference, safety, terminal_yaw_relocation_params_.terminal_window_length))
  {
    const std::size_t tail_start = terminalCoincidentTailStart(reference);
    const std::size_t window_start = std::min(
      tail_start,
      terminalApproachWindowStart(
        reference, terminal_yaw_relocation_params_.terminal_window_length));
    const std::size_t original_collisions = safety.collisions.size();
    const std::vector<std::size_t> candidates = terminalYawRelocationCandidates(
      reference, static_cast<std::size_t>(terminal_yaw_relocation_max_candidates_));
    std::ostringstream candidate_list;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (i != 0U) {
        candidate_list << ',';
      }
      candidate_list << candidates[i];
    }
    std::size_t first_conflict = 0;
    std::size_t last_conflict = 0;
    if (!safety.collisions.empty()) {
      first_conflict = safety.collisions.front().trajectory_index;
      last_conflict = first_conflict;
      for (const CollisionSample & collision : safety.collisions) {
        first_conflict = std::min(first_conflict, collision.trajectory_index);
        last_conflict = std::max(last_conflict, collision.trajectory_index);
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【终端航向重定位】尝试：tail_start=%zu window_start=%zu candidates=[%s] "
      "conflict_span=[%zu,%zu] collisions=%zu.",
      tail_start, window_start, candidate_list.str().c_str(), first_conflict, last_conflict,
      original_collisions);
    std::size_t built = 0;
    std::ostringstream rejected;
    for (const std::size_t rotation_index : candidates) {
      ReferenceTrajectory candidate;
      if (!relocateTerminalYawRotation(
          reference, goal_yaw, rotation_index, terminal_yaw_relocation_params_, &candidate))
      {
        rejected << rotation_index << ":build_failed;";
        continue;
      }
      ++built;
      candidate.header.stamp = now();
      // 只补净空标注,绝不调用 planYaw:planYaw 会在末点再追加一段原地转向,
      // 把刚刚移开的问题原样搬回来。
      annotateClearance(candidate, *map_snapshot);
      const FootprintSafetyResult candidate_safety = safety_checker_.check(
        candidate, planning_grid);
      if (!candidate_safety.safe) {
        rejected << rotation_index << ":footprint;";
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终端航向重定位成功】rotation_index=%zu tail_start=%zu "
        "window_start=%zu candidates=%zu built=%zu cleared_collisions=%zu points=%zu。",
        rotation_index, tail_start, window_start, candidates.size(), built, original_collisions,
        candidate.points.size());
      reference = std::move(candidate);
      safety = candidate_safety;
      break;
    }
    if (!safety.safe) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终端航向重定位失败】tail_start=%zu window_start=%zu candidates=[%s] "
        "built=%zu collisions=%zu rejected=%s；保留现有足迹拒绝路径。",
        tail_start, window_start, candidate_list.str().c_str(), built, original_collisions,
        rejected.str().c_str());
    }
  }

  if (preprocessed_guide_pub_ && !selected_trace.preprocessed_guide.poses.empty()) {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && !selected_trace.esdf_refined_guide.poses.empty()) {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!reference.valid()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】参考轨迹无效，保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  EscapePrefixDecision escape_decision = safety.safe ?
    EscapePrefixDecision{} :
    evaluateEscapePrefix(reference.points, safety.collisions, escape_prefix_params_);
  if (escape_decision.allowed &&
    !escapePrefixContactShallow(
      reference, escape_decision, footprint_params_, planning_grid, escape_prefix_params_))
  {
    // 前缀压进占据格超过深度上界：不是擦边接触，照旧 fail-closed。
    escape_decision.prefix_end = 0U;
    escape_decision.allowed = false;
  }
  // 贴障起步时 yaw 规划在起点就转向，角点扫进障碍，逃逸前缀的深度/扫掠两道上界都拒绝它。
  // 换成"先保持起点 yaw 平移驶离再追回原 yaw"的候选，重新过同一套门禁与逃逸判据；
  // 几何路径和终端 yaw 不变。只在逃逸开关打开且冲突紧贴起点时尝试。
  if (!safety.safe && !escape_decision.allowed && escape_prefix_params_.enabled &&
    !safety.collisions.empty() &&
    std::min_element(
      safety.collisions.begin(), safety.collisions.end(),
      [](const CollisionSample & a, const CollisionSample & b) {
        return a.trajectory_index < b.trajectory_index;
      })->trajectory_index == 0U)
  {
    std::ostringstream tried;
    for (const double hold_length : {0.20, escape_prefix_params_.max_prefix_length_m}) {
      ReferenceTrajectory candidate;
      if (!holdStartYawForEscape(
          reference, hold_length, terminal_yaw_relocation_params_.yaw_rate_limit, &candidate))
      {
        tried << hold_length << ":build_failed;";
        continue;
      }
      annotateClearance(candidate, *map_snapshot);
      const FootprintSafetyResult candidate_safety =
        safety_checker_.check(candidate, planning_grid);
      EscapePrefixDecision candidate_escape = candidate_safety.safe ?
        EscapePrefixDecision{} :
        evaluateEscapePrefix(candidate.points, candidate_safety.collisions, escape_prefix_params_);
      if (candidate_escape.allowed &&
        !escapePrefixContactShallow(
          candidate, candidate_escape, footprint_params_, planning_grid, escape_prefix_params_))
      {
        candidate_escape.prefix_end = 0U;
        candidate_escape.allowed = false;
      }
      if (!candidate_safety.safe && !candidate_escape.allowed) {
        tried << hold_length << ":collisions=" << candidate_safety.collisions.size() <<
          ",prefix_end=" << candidate_escape.candidate_prefix_end <<
          ",yaw_sweep=" << candidate_escape.prefix_yaw_sweep_rad << ';';
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【脱困航向保持】接受：长度=%.2f m safe=%d 冲突=%zu 前缀末端=%zu。",
        hold_length, candidate_safety.safe ? 1 : 0, candidate_safety.collisions.size(),
        candidate_escape.prefix_end);
      reference = std::move(candidate);
      safety = candidate_safety;
      escape_decision = candidate_escape;
      break;
    }
    if (!safety.safe && !escape_decision.allowed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【脱困航向保持失败】已尝试：%s", tried.str().c_str());
    }
  }
  if (!safety.safe && !publish_unsafe_trajectory_ && escape_decision.allowed) {
    // 车此刻就压在冲突区里，冲突集证明这条轨迹在有界前缀内驶出障碍并且不再驶回。
    // 继续拒绝只会让唯一能挪走车的执行器拿不到轨迹，参见 escape_prefix.hpp 的说明。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【提交脱困轨迹】MINCO 轨迹驶离接触区：冲突=%zu 起始偏移=%.3f m "
      "prefix_end=%zu prefix_length=%.3f m prefix_yaw_sweep=%.3f rad.",
      escape_decision.collision_count, escape_decision.head_offset_m,
      escape_decision.prefix_end, escape_decision.prefix_length_m,
      escape_decision.prefix_yaw_sweep_rad);
  } else if (!safety.safe && !publish_unsafe_trajectory_) {
    if (safety.collisions.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】安全校验失败，未发布不安全轨迹。");
    } else {
      const CollisionSample & first_collision = safety.collisions.front();
      // 只报冲突数和一个 footprint 采样坐标无法区分三种完全不同的失败:车此刻就压在
      // 障碍上、轨迹先走一段再撞墙、以及轨迹整条贴着障碍蹭行。这三种的修复位置分别在
      // 逃逸放行、图搜索净空和栅格量化一致性上,所以这里把判据本身打全:冲突的离散/
      // 扫掠构成、首末冲突下标、首个冲突点的轨迹中心位姿(注意 first_collision 的坐标
      // 是矩形上的采样点,不是中心),以及逃逸判据为什么没有放行。
      std::size_t discrete_collisions = 0;
      std::size_t swept_collisions = 0;
      std::size_t last_collision_index = 0;
      for (const CollisionSample & collision : safety.collisions) {
        if (collision.swept) {
          ++swept_collisions;
        } else {
          ++discrete_collisions;
        }
        last_collision_index = std::max(last_collision_index, collision.trajectory_index);
      }
      const ReferencePoint & first_center =
        reference.points[std::min(first_collision.trajectory_index, reference.points.size() - 1U)];
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【MINCO拒绝】足迹冲突=%zu，未发布不安全轨迹 "
        "（discrete=%zu swept=%zu）first_index=%zu last_index=%zu points=%zu "
        "first_sample=(%.3f, %.3f) first_center=(%.3f, %.3f, yaw=%.3f) "
        "start=(%.3f, %.3f, yaw=%.3f) raw_points=%zu preprocessed_points=%zu "
        "esdf_refined_points=%zu escape_allowed=%d "
        "escape_head_offset=%.3f m escape_prefix_end=%zu escape_candidate_prefix_end=%zu "
        "escape_prefix_length=%.3f m escape_prefix_yaw_sweep=%.3f rad）",
        safety.collisions.size(), discrete_collisions, swept_collisions,
        first_collision.trajectory_index, last_collision_index, reference.points.size(),
        first_collision.x, first_collision.y,
        first_center.x, first_center.y, first_center.yaw,
        reference.points.front().x, reference.points.front().y, reference.points.front().yaw,
        search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
        selected_trace.esdf_refined_guide.poses.size(), static_cast<int>(escape_decision.allowed),
        escape_decision.head_offset_m, escape_decision.prefix_end,
        escape_decision.candidate_prefix_end,
        escape_decision.prefix_length_m, escape_decision.prefix_yaw_sweep_rad);
      // 上面的数字只说明整条轨迹都不安全,不说明是哪
      // 一层把格子打成了障碍。静态层已离线排除:停车位
      // 姿到最近静态占据格心约 0.61 m,大于 0.4187 m 的
      // 全 yaw 半径。所以必须把实际栅格打出来,否则无法
      // 区分 ROG 投影、terrain 硬障碍与 slope 障碍。两个
      // 窗口分别覆盖轨迹起点和首个冲突点。
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】起点：%s",
        describeGridWindow(
          planning_grid, reference.points.front().x, reference.points.front().y, 0.6,
          obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      if (first_collision.trajectory_index != 0U) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】首个冲突点：%s",
          describeGridWindow(
            planning_grid, first_center.x, first_center.y, 0.6,
            obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      }
    }
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  TrajectoryQualityMetrics quality = quality_evaluator_.evaluate(
    reference, clearance_esdf.get(), footprintSamples(planning_grid),
    selected_trace.segment_durations);
  for (const CollisionSample & collision : safety.collisions) {
    if (collision.swept) {
      ++quality.swept_collision_count;
    } else {
      ++quality.footprint_collision_count;
    }
  }
  if (!quality.finite || !quality.strictly_monotonic_time) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】导数包含非有限值或时间不单调。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  // Nominal commits only: escape-from-contact may need a short lateral shove.
  // d21 gen173 committed length_ratio=5.496 lateral=7.424 and drove north of spawn.
  if (safety.safe && !admitsCommitGeometry(quality, commit_geometry_limits_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】绕行轨迹提交前未通过几何约束（length_ratio=%.3f max=%.3f "
      "lateral=%.3f m max=%.3f m path_length=%.2f direct=%.2f）。",
      quality.length_ratio, commit_geometry_limits_.max_length_ratio,
      quality.max_lateral_deviation, commit_geometry_limits_.max_lateral_deviation_m,
      quality.path_length, quality.direct_length);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NO_PATH,
      map_snapshot->generation);
    return;
  }
  nav_msgs::msg::Path control_reference;
  if (!transformPathToGlobal(toPath(reference), control_reference)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考发布失败】控制坐标系转换失败，未发布 MINCO 参考。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REFERENCE_TF,
      map_snapshot->generation);
    return;
  }
  const std::uint8_t yaw_authority = selectYawAuthority(
    reference, force_body_yaw_follow_, body_yaw_follow_clearance_);
  PlannerCommitTelemetry telemetry;
  telemetry.grid_topic = grid_topic_;
  telemetry.validation_frame = planning_grid.header.frame_id.empty() ?
    global_frame_ : planning_grid.header.frame_id;
  telemetry.gate_collision_count =
    static_cast<std::uint32_t>(safety.collisions.size());
  telemetry.gate_collision_indices = formatCollisionIndices(safety.collisions);
  telemetry.occupancy_digest = planning_digest::occupancyDigest(planning_grid);
  telemetry.content_digest = planning_digest::pathContentDigest(control_reference);
  telemetry.reference_point_count =
    static_cast<std::uint32_t>(reference.points.size());
  if (!publishReferenceIfCurrent(map_snapshot, map_health_epoch,
                                 control_reference, reference, goal_id, localization_epoch,
                                 plan_request_sequence, map_publication_sequence, yaw_authority,
                                 report_status, telemetry)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考提交丢弃】generation=%llu 因规划地图变化或过期而丢弃，等待重规划。",
      static_cast<unsigned long long>(map_snapshot->generation));
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
      map_snapshot->generation);
    return;
  }
  if (marker_pub_) {
    // Marker publication follows the immutable-snapshot commit, so the red
    // curve is exactly the safety-validated reference handed to MPC.
    marker_pub_->publish(visualizer_.buildMarkers(
      selected_trace.preprocessed_guide, reference, safety));
  }

  std::ostringstream durations;
  durations.setf(std::ios::fixed);
  durations.precision(3);
  for (std::size_t index = 0; index < quality.segment_durations.size(); ++index) {
    if (index > 0U) {
      durations << ',';
    }
    durations << quality.segment_durations[index];
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), log_throttle_ms_,
    "【规划完成】planned generation=%llu snapshot_publication=%llu "
    "raw_points=%zu preprocessed_points=%zu "
    "esdf_refined_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu "
    "footprint_collisions=%zu swept_collisions=%zu "
    "collision_indices=%s occupancy_digest=%s content_digest=%s grid_topic=%s validation_frame=%s "
    "fallback=%d local_repair=%d optimization_failure_reason=%s escape_prefix_end=%zu "
    "expanded=%d yaw_authority=%u center_clearance=%.3f footprint_clearance=%.3f "
    "length_ratio=%.3f lateral=%.3f curvature_max=%.3f curvature_p95=%.3f turn=%.3f "
    "curvature_tv=%.3f curvature_sign_changes=%zu local_scaled=%d uniform_scaled=%d "
    "peak_v=%.3f peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f "
    "joint_pieces=%d joint_iterations=%d joint_evaluations=%d joint_termination=%s "
    "joint_cost=%.3f->%.3f joint_wall_ms=%.3f joint_satisfied=%d joint_center_violation=%.3f "
    "joint_footprint_violation=%.3f joint_guide_excess=%.3f joint_v_excess=%.3f "
    "joint_a_excess=%.3f "
    "joint_lat_excess=%.3f segment_durations=[%s]",
    static_cast<unsigned long long>(map_snapshot->generation),
    static_cast<unsigned long long>(map_publication_sequence), search_result.path.poses.size(),
    selected_trace.preprocessed_guide.poses.size(), selected_trace.esdf_refined_guide.poses.size(),
    reference.points.size(), reference.totalLength(), reference.totalTime(), safety.collisions.size(),
    quality.footprint_collision_count, quality.swept_collision_count,
    telemetry.gate_collision_indices.c_str(),
    telemetry.occupancy_digest.c_str(), telemetry.content_digest.c_str(), telemetry.grid_topic.c_str(),
    telemetry.validation_frame.c_str(),
    esdf_fallback_used ? 1 : 0, local_repair_used ? 1 : 0,
    selected_trace.failure_reason.empty() ? "none" : selected_trace.failure_reason.c_str(),
    escape_decision.prefix_end,
    search_result.expanded_nodes, static_cast<unsigned int>(yaw_authority),
    quality.minimum_center_clearance, quality.minimum_footprint_clearance, quality.length_ratio,
    quality.max_lateral_deviation, quality.max_geometric_curvature,
    quality.p95_geometric_curvature, quality.total_turning_angle,
    quality.curvature_total_variation, quality.curvature_sign_changes,
    selected_trace.local_time_scaled ? 1 : 0, selected_trace.uniform_time_scaled ? 1 : 0,
    quality.peak_velocity, quality.peak_acceleration, quality.peak_jerk,
    selected_trace.solver_wall_time_ms, selected_trace.joint_piece_count,
    selected_trace.joint_iterations, selected_trace.joint_evaluations,
    selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
    selected_trace.joint_initial_cost, selected_trace.joint_final_cost,
    selected_trace.joint_wall_time_ms, selected_trace.joint_constraints_satisfied ? 1 : 0,
    selected_trace.joint_max_center_violation, selected_trace.joint_max_footprint_violation,
    selected_trace.joint_max_guide_excess, selected_trace.joint_max_velocity_excess,
    selected_trace.joint_max_acceleration_excess,
    selected_trace.joint_max_lateral_excess, durations.str().c_str());
}

void MincoPlannerNode::annotatePositionClearance(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const
{
  const bool available = snapshot.clearance_esdf && snapshot.clearance_esdf->available();
  for (auto & point : trajectory.points) {
    point.clearance = available ?
      snapshot.clearance_esdf->getDistance(point.x, point.y) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

void MincoPlannerNode::planYaw(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot,
  double start_yaw, double goal_yaw) const
{
  // 这三步的顺序就是这段逻辑的全部内容,所以收进一个函数,避免四个调用点各自写错。
  // 窄通道判据必须用与 yaw 无关的位置净空:yaw 正是这一步要决定的量,拿 yaw 相关的
  // footprint 净空当输入构成循环依赖,而 annotateClearance 恰好是 yaw 相关的。
  annotatePositionClearance(trajectory, snapshot);
  yaw_planner_.apply(trajectory, start_yaw, goal_yaw);
  // yaw 定下后才知道平移与转向叠加的轮速；超出底盘能力的局部放慢时间，位置和 yaw 不变。
  applyWheelSpeedTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  // 窄通道边平移边转向时再放慢，压低 MPC 横向/yaw 跟踪误差；同样只改时间。
  applyNarrowTurnTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  // 离墙越近走得越慢（按中心净空），同样只改时间。
  applyClearanceSpeedTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  // 下游(yaw authority、轨迹质量评估、planned 记录)读的是 yaw 相关的 footprint 净空,
  // 所以 yaw 定下来之后再覆盖回 footprint 净空,对外语义不变。
  annotateClearance(trajectory, snapshot);
}

void MincoPlannerNode::annotateClearance(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const
{
  const bool available = snapshot.clearance_esdf && snapshot.clearance_esdf->available();
  const std::vector<Eigen::Vector2d> samples = footprintSamples(snapshot.grid);
  for (auto & point : trajectory.points) {
    point.clearance = available ? snapshot.clearance_esdf->getFootprintClearance(
      Eigen::Vector2d(point.x, point.y), point.yaw, samples) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

std::vector<Eigen::Vector2d> MincoPlannerNode::footprintSamples(
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const double half_length = 0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_;
  const double half_width = 0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_;
  const double resolution = std::max(0.02, static_cast<double>(grid.info.resolution));
  const int samples_x = std::max(2, static_cast<int>(std::ceil((2.0 * half_length) / resolution)));
  const int samples_y = std::max(2, static_cast<int>(std::ceil((2.0 * half_width) / resolution)));
  std::vector<Eigen::Vector2d> samples;
  samples.reserve(static_cast<std::size_t>((samples_x + 1) * (samples_y + 1)));
  for (int ix = 0; ix <= samples_x; ++ix) {
    const double x = -half_length + 2.0 * half_length * ix / static_cast<double>(samples_x);
    for (int iy = 0; iy <= samples_y; ++iy) {
      const double y = -half_width + 2.0 * half_width * iy / static_cast<double>(samples_y);
      samples.emplace_back(x, y);
    }
  }
  return samples;
}

bool MincoPlannerNode::lookupStartPose(
  const nav_msgs::msg::OccupancyGrid & grid, geometry_msgs::msg::PoseStamped & start)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      !grid.header.frame_id.empty() ? grid.header.frame_id : global_frame_,
      robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    start.header = transform.header;
    start.pose.position.x = transform.transform.translation.x;
    start.pose.position.y = transform.transform.translation.y;
    // The planning grid is a 2D ground plane: the robot_frame height (gimbal /
    // lidar mount) must not lift the JPS/MINCO start above the grid.
    start.pose.position.z = grid.info.origin.position.z;
    start.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【TF失败】查询 %s -> %s 失败：%s",
      global_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

bool MincoPlannerNode::resolveStartPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped * frozen_start,
  geometry_msgs::msg::PoseStamped & start)
{
  if (frozen_start == nullptr || frozen_start->header.frame_id.empty()) {
    return lookupStartPose(grid, start);
  }
  if (!finitePose(frozen_start->pose)) {
    return false;
  }
  return transformGoalToGrid(grid, *frozen_start, start);
}

bool MincoPlannerNode::transformGoalToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output)
{
  const std::string target_frame = grid.header.frame_id.empty() ? global_frame_ : grid.header.frame_id;
  if (input.header.frame_id.empty() || input.header.frame_id == target_frame) {
    output = input;
    output.header.frame_id = target_frame;
    output.pose.position.z = grid.info.origin.position.z;
    return true;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::doTransform(input, output, transform);
    // Start/goal poses arrive at the localization frame height; project them
    // onto the 2D planning grid plane so every path vertex stays on the ground.
    output.pose.position.z = grid.info.origin.position.z;
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【TF失败】查询 %s -> %s 失败：%s", input.header.frame_id.c_str(),
      target_frame.c_str(), exception.what());
    return false;
  }
}

bool MincoPlannerNode::transformPathToGlobal(
  const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output)
{
  const std::string source_frame =
    input.header.frame_id.empty() ? global_frame_ : input.header.frame_id;
  if (source_frame == global_frame_) {
    output = input;
    output.header.frame_id = global_frame_;
    for (auto & pose : output.poses) {
      pose.header.frame_id = global_frame_;
    }
    return true;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      global_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    output.header = input.header;
    output.header.frame_id = global_frame_;
    output.poses.clear();
    output.poses.reserve(input.poses.size());
    for (const auto & pose : input.poses) {
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(pose, transformed, transform);
      transformed.header = pose.header;
      transformed.header.frame_id = global_frame_;
      output.poses.push_back(std::move(transformed));
    }
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【TF失败】MINCO 参考查询 %s -> %s 失败：%s",
      source_frame.c_str(), global_frame_.c_str(), exception.what());
    return false;
  }
}

void MincoPlannerNode::publishEmergencyStop(bool stop)
{
  if (!planner_manages_emergency_stop_ || !emergency_stop_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = stop;
  emergency_stop_pub_->publish(message);
}

void MincoPlannerNode::publishPlannerStatus(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_generation, std::uint64_t map_publication_sequence,
    std::uint8_t state, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time &reference_stamp,
    std::uint8_t yaw_authority, bool requires_gimbal_lock,
    const PlannerCommitTelemetry & telemetry) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  publishPlannerStatusLocked(
    goal_id, localization_epoch, plan_request_sequence, map_generation,
    map_publication_sequence, state, failure_reason, reference_stamp,
    yaw_authority, requires_gimbal_lock, telemetry);
}

void MincoPlannerNode::publishPlannerStatusLocked(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_generation, std::uint64_t map_publication_sequence,
    std::uint8_t state, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time & reference_stamp,
    std::uint8_t yaw_authority, bool requires_gimbal_lock,
    const PlannerCommitTelemetry & telemetry) {
  if (!planner_status_pub_) {
    return;
  }
  ats_navigation_interfaces::msg::PlannerStatus status;
  status.header.stamp = now();
  status.header.frame_id = global_frame_;
  status.goal_id = goal_id;
  status.localization_epoch = localization_epoch;
  status.plan_request_sequence = plan_request_sequence;
  status.map_generation = map_generation;
  if (status.map_generation == 0) {
    status.map_generation = next_map_generation_;
  }
  status.map_publication_sequence = map_publication_sequence;
  status.state = state;
  status.reference_stamp = reference_stamp;
  status.failure_reason = failure_reason;
  status.yaw_authority = yaw_authority;
  status.requires_gimbal_lock = requires_gimbal_lock;
  status.grid_topic = telemetry.grid_topic;
  status.validation_frame = telemetry.validation_frame;
  status.gate_collision_count = telemetry.gate_collision_count;
  status.gate_collision_indices = telemetry.gate_collision_indices;
  status.occupancy_digest = telemetry.occupancy_digest;
  status.content_digest = telemetry.content_digest;
  status.reference_point_count = telemetry.reference_point_count;
  if (!planner_status_state_.update(status)) {
    return;
  }
  if (state == ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED) {
    // Acceptance is not a map-health/reference grant.  Retain the local
    // identity for invalidation, but preserve the unbound wire status until
    // the commit gate publishes REFERENCE_READY.
    status.map_generation = map_generation;
  }
  planner_status_pub_->publish(status);
}

void MincoPlannerNode::invalidatePlannerStatusLocked(
    std::uint8_t failure_reason, std::uint64_t generation) {
  if (planner_status_pub_ && planner_status_state_.invalidate(generation, failure_reason, now())) {
    planner_status_pub_->publish(*planner_status_state_.latest);
  }
}

void MincoPlannerNode::invalidateMapLocked() {
  ++map_health_epoch_;
  latest_map_snapshot_.reset();
  active_safety_reference_.reset();
  safety_state_.invalidateMap(next_map_generation_);
  invalidatePlannerStatusLocked(
    ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY,
    next_map_generation_);
}

void MincoPlannerNode::setPlanSafe(bool safe)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    safety_state_.plan_safe = safe;
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
}

bool MincoPlannerNode::publishReferenceIfCurrent(
    const std::shared_ptr<const PlanningMapSnapshot> &snapshot,
    std::uint64_t map_health_epoch, const nav_msgs::msg::Path &reference_path,
    const ReferenceTrajectory &safety_reference,
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_publication_sequence,
    std::uint8_t yaw_authority,
    bool report_status,
    const PlannerCommitTelemetry & telemetry) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  const bool heartbeat_fresh = map_ready_topic_.empty() || steadyHeartbeatLeaseValid(
    last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_);
  if (!heartbeat_fresh && safety_state_.map_ready) {
    invalidateMapLocked();
  }
  const bool publish = heartbeat_fresh && snapshot && latest_map_snapshot_ &&
    safety_state_.mapSnapshotUsable(snapshot->generation) &&
    map_health_epoch == map_health_epoch_ &&
    snapshot->generation == latest_map_snapshot_->generation;
  if (!publish) {
    // 无停车重规划的替换候选过期：正在执行的旧参考仍受运行期复检守护，只在地图
    // 健康且快照可用时保留它；其余情况照旧撤销。
    const bool keep_guarded_reference = heartbeat_fresh && latest_map_snapshot_ &&
      replan_advised_reference_stamp_ && active_safety_reference_ &&
      active_safety_reference_->goal_id == goal_id &&
      active_safety_reference_->localization_epoch == localization_epoch &&
      active_safety_reference_->plan_request_sequence < plan_request_sequence &&
      safety_state_.mapSnapshotUsable(latest_map_snapshot_->generation);
    if (!keep_guarded_reference) {
      safety_state_.plan_safe = false;
      publishEmergencyStop(true);
    }
    return false;
  }
  if (!planner_manages_emergency_stop_ && !candidate_reference_path_pub_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【规划器配置错误】P3 模式没有候选参考发布器。");
    return false;
  }
  nav_msgs::msg::Path committed_reference = reference_path;
  rebasePathTimestamps(committed_reference, now());
  safety_state_.plan_safe = true;
  ActiveSafetyReference active_reference;
  active_reference.trajectory = safety_reference;
  active_reference.trajectory.header.stamp = committed_reference.header.stamp;
  active_reference.goal_id = goal_id;
  active_reference.localization_epoch = localization_epoch;
  active_reference.plan_request_sequence = plan_request_sequence;
  active_reference.map_generation = snapshot->generation;
  active_reference.map_publication_sequence = map_publication_sequence;
  active_safety_reference_ = std::move(active_reference);
  if (planner_manages_emergency_stop_) {
    // Preserve the P2 stop-clear/reference ordering inside the same map lock.
    publishEmergencyStop(false);
    reference_path_pub_->publish(committed_reference);
  } else {
    // Commit candidate and READY before any map/health invalidation can publish.
    // GoalManager remains the sole P3 final-reference/emergency-stop authority.
    candidate_reference_path_pub_->publish(committed_reference);
    if (report_status) {
      publishPlannerStatusLocked(
        goal_id, localization_epoch, plan_request_sequence,
        snapshot->generation, map_publication_sequence,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_REFERENCE_READY,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE,
        committed_reference.header.stamp, yaw_authority,
        yaw_authority == ats_navigation_interfaces::msg::PlannerStatus::
          YAW_AUTHORITY_BODY_YAW_FOLLOW,
        telemetry);
    }
  }
  return true;
}

nav_msgs::msg::Path MincoPlannerNode::toPath(const ReferenceTrajectory & trajectory) const
{
  nav_msgs::msg::Path path;
  path.header = trajectory.header;
  path.poses.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory.header;
    const rclcpp::Time point_stamp =
      rclcpp::Time(trajectory.header.stamp) + rclcpp::Duration::from_seconds(point.t);
    pose.header.stamp.sec = static_cast<int32_t>(point_stamp.nanoseconds() / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<uint32_t>(point_stamp.nanoseconds() % 1000000000LL);
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * point.yaw);
    pose.pose.orientation.w = std::cos(0.5 * point.yaw);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace minco_planner

RCLCPP_COMPONENTS_REGISTER_NODE(minco_planner::MincoPlannerNode)
