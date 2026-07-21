// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "minco_planner/trajectory/reference_path_timing.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{

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

  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).reliable(),
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
  // 设为空可彻底断开 Nav2 /plan；此时 goal_topic 仍可独立触发 JPS 与 MINCO。
  if (!global_plan_topic_.empty()) {
    global_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(1),
      std::bind(&MincoPlannerNode::onGlobalPlan, this, std::placeholders::_1), planning_options);
  }
  raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, rclcpp::QoS(1));
  if (planner_manages_emergency_stop_) {
    reference_path_pub_ =
      create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(1));
  }
  if (!candidate_reference_path_topic_.empty()) {
    candidate_reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      candidate_reference_path_topic_, rclcpp::QoS(1).reliable());
  }
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic_, rclcpp::QoS(1));
  if (planner_manages_emergency_stop_) {
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());
  }
  if (!planner_status_topic_.empty()) {
    planner_status_pub_ = create_publisher<ats_navigation_interfaces::msg::PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(10).reliable());
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
    get_logger(), "minco_planner ready: grid='%s' goal='%s' request='%s' raw='%s' reference='%s'",
    grid_topic_.c_str(), goal_topic_.c_str(), goal_request_topic_.c_str(), raw_path_topic_.c_str(),
    reference_path_topic_.c_str());
}

void MincoPlannerNode::declareAndLoadParams()
{
  declare_parameter<std::string>("grid_topic", grid_topic_);
  declare_parameter<std::string>("goal_topic", goal_topic_);
  declare_parameter<std::string>("global_plan_topic", global_plan_topic_);
  declare_parameter<std::string>("goal_request_topic", goal_request_topic_);
  declare_parameter<std::string>("planner_status_topic", planner_status_topic_);
  declare_parameter<std::string>("raw_path_topic", raw_path_topic_);
  declare_parameter<std::string>("reference_path_topic", reference_path_topic_);
  declare_parameter<std::string>("candidate_reference_path_topic", candidate_reference_path_topic_);
  declare_parameter<std::string>("debug_marker_topic", debug_marker_topic_);
  declare_parameter<std::string>("map_ready_topic", map_ready_topic_);
  declare_parameter<std::string>("emergency_stop_topic", emergency_stop_topic_);
  declare_parameter<double>("map_ready_timeout_sec", map_ready_timeout_sec_);
  declare_parameter<double>(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  declare_parameter<double>("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  declare_parameter<double>("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  declare_parameter<double>("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  declare_parameter<bool>("force_body_yaw_follow", force_body_yaw_follow_);
  declare_parameter<std::string>("global_frame", global_frame_);
  declare_parameter<std::string>("robot_frame", robot_frame_);
  declare_parameter<std::string>("search_algorithm", search_algorithm_);
  declare_parameter<bool>("astar_fallback", astar_fallback_);
  declare_parameter<bool>("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  declare_parameter<bool>("planner_manages_emergency_stop", planner_manages_emergency_stop_);

  GridAstarParams astar_params;
  declare_parameter<int>("obstacle_value_threshold", astar_params.obstacle_value_threshold);
  declare_parameter<bool>("unknown_is_obstacle", astar_params.unknown_is_obstacle);
  declare_parameter<bool>("allow_diagonal", astar_params.allow_diagonal);
  GridJpsParams jps_params;
  declare_parameter<int>("jps_max_expanded_nodes", jps_params.max_expanded_nodes);
  declare_parameter<double>("jps_safe_distance", jps_params.safe_distance);

  MincoTrajectoryOptimizerParams optimizer_params;
  declare_parameter<double>("reference_speed", optimizer_params.reference_speed);
  declare_parameter<double>("min_segment_time", optimizer_params.min_segment_time);
  declare_parameter<double>("sample_spacing", optimizer_params.sample_spacing);
  declare_parameter<double>("max_velocity", optimizer_params.max_velocity);
  declare_parameter<double>("max_acceleration", optimizer_params.max_acceleration);
  declare_parameter<int>(
    "max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  declare_parameter<double>("time_scaling_factor", optimizer_params.time_scaling_factor);
  declare_parameter<bool>(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  declare_parameter<double>("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  declare_parameter<int>(
    "esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  declare_parameter<double>(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  declare_parameter<double>("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  declare_parameter<double>(
    "esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  declare_parameter<bool>(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  declare_parameter<double>("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  declare_parameter<double>(
    "esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);

  YawSplinePlannerParams yaw_params;
  declare_parameter<std::string>("yaw_mode", yaw_params.mode);
  declare_parameter<double>("yaw_rate_limit", yaw_params.yaw_rate_limit);
  declare_parameter<double>("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  declare_parameter<double>("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  declare_parameter<double>(
    "terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);

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
  get_parameter("global_plan_topic", global_plan_topic_);
  get_parameter("goal_request_topic", goal_request_topic_);
  get_parameter("planner_status_topic", planner_status_topic_);
  get_parameter("raw_path_topic", raw_path_topic_);
  get_parameter("reference_path_topic", reference_path_topic_);
  get_parameter("candidate_reference_path_topic", candidate_reference_path_topic_);
  get_parameter("debug_marker_topic", debug_marker_topic_);
  get_parameter("map_ready_topic", map_ready_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("map_ready_timeout_sec", map_ready_timeout_sec_);
  map_ready_timeout_sec_ = std::max(0.1, map_ready_timeout_sec_);
  get_parameter(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  emergency_stop_heartbeat_period_sec_ = std::max(0.02, emergency_stop_heartbeat_period_sec_);
  get_parameter("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  runtime_safety_recheck_hz_ = std::max(0.1, runtime_safety_recheck_hz_);
  get_parameter("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  runtime_safety_horizon_sec_ = std::max(0.0, runtime_safety_horizon_sec_);
  get_parameter("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  body_yaw_follow_clearance_ = std::max(0.0, body_yaw_follow_clearance_);
  get_parameter("force_body_yaw_follow", force_body_yaw_follow_);
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("search_algorithm", search_algorithm_);
  get_parameter("astar_fallback", astar_fallback_);
  get_parameter("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  get_parameter("planner_manages_emergency_stop", planner_manages_emergency_stop_);
  get_parameter("obstacle_value_threshold", obstacle_value_threshold_);
  get_parameter("unknown_is_obstacle", unknown_is_obstacle_);
  astar_params.obstacle_value_threshold = obstacle_value_threshold_;
  astar_params.unknown_is_obstacle = unknown_is_obstacle_;
  get_parameter("allow_diagonal", astar_params.allow_diagonal);
  get_parameter("jps_max_expanded_nodes", jps_params.max_expanded_nodes);
  get_parameter("jps_safe_distance", jps_params.safe_distance);
  get_parameter("reference_speed", optimizer_params.reference_speed);
  get_parameter("min_segment_time", optimizer_params.min_segment_time);
  get_parameter("sample_spacing", optimizer_params.sample_spacing);
  get_parameter("max_velocity", optimizer_params.max_velocity);
  get_parameter("max_acceleration", optimizer_params.max_acceleration);
  get_parameter("max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  get_parameter("time_scaling_factor", optimizer_params.time_scaling_factor);
  get_parameter(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  get_parameter("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  get_parameter("esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  get_parameter(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  get_parameter("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  get_parameter("esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  get_parameter(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  get_parameter("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  get_parameter("esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);
  get_parameter("yaw_mode", yaw_params.mode);
  get_parameter("yaw_rate_limit", yaw_params.yaw_rate_limit);
  get_parameter("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  get_parameter("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  get_parameter("terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);
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
      "Raising JPS clearance from %.3f m to rectangular all-yaw footprint radius %.3f m.",
      jps_params.safe_distance, all_yaw_footprint_radius);
    jps_params.safe_distance = all_yaw_footprint_radius;
  }
  get_parameter("local_repair_enabled", repair_params.enabled);
  get_parameter("local_repair_max_iterations", repair_params.max_iterations);
  get_parameter("local_repair_search_radius", repair_params.search_radius);

  footprint_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  footprint_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  repair_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  repair_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;

  astar_.setParams(astar_params);
  static_cast<GridAstarParams &>(jps_params) = astar_params;
  jps_.setParams(jps_params);
  optimizer_.setParams(optimizer_params);
  yaw_planner_.setParams(yaw_params);
  safety_checker_.setParams(footprint_params);
  collision_repair_.setParams(repair_params);
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
      latest_map_snapshot_.reset();
      ++map_health_epoch_;
      safety_state_.invalidateMap(next_map_generation_);
      publishEmergencyStop(true);
    }
    RCLCPP_ERROR(get_logger(), "Rejected an invalid planning grid.");
    return;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (
    candidate_health_epoch != map_health_epoch_ ||
    candidate_generation != next_map_generation_ + 1)
  {
    RCLCPP_WARN(get_logger(), "Discarded a planning grid built across a map-health change.");
    return;
  }
  next_map_generation_ = snapshot->generation;
  latest_map_snapshot_ = snapshot;
  if (map_ready_topic_.empty()) {
    safety_state_.map_ready = true;
  }
}

void MincoPlannerNode::onMapReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!msg->data) {
      ++map_health_epoch_;
      latest_map_snapshot_.reset();
      active_safety_reference_.reset();
      safety_state_.invalidateMap(next_map_generation_);
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
      ++map_health_epoch_;
      latest_map_snapshot_.reset();
      active_safety_reference_.reset();
      safety_state_.invalidateMap(next_map_generation_);
      timed_out = true;
    }
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
  if (timed_out) {
    RCLCPP_ERROR(get_logger(), "Planning-map ready heartbeat timed out.");
  }
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
  const double horizon_end = elapsed + runtime_safety_horizon_sec_;
  std::size_t first_index = active_reference->trajectory.points.size();
  for (std::size_t i = 0; i < active_reference->trajectory.points.size(); ++i) {
    if (active_reference->trajectory.points[i].t >= elapsed) {
      first_index = i == 0 ? 0 : i - 1;
      break;
    }
  }
  if (first_index >= active_reference->trajectory.points.size() - 1) {
    return;
  }

  ReferenceTrajectory remaining;
  remaining.header = active_reference->trajectory.header;
  for (std::size_t i = first_index; i < active_reference->trajectory.points.size(); ++i) {
    remaining.points.push_back(active_reference->trajectory.points[i]);
    if (active_reference->trajectory.points[i].t >= horizon_end &&
      remaining.points.size() >= 2)
    {
      break;
    }
  }
  const FootprintSafetyResult safety = safety_checker_.check(remaining, snapshot->grid);
  if (safety.safe) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ ||
      active_safety_reference_->goal_id != active_reference->goal_id ||
      active_safety_reference_->localization_epoch != active_reference->localization_epoch)
    {
      return;
    }
    active_safety_reference_.reset();
    safety_state_.plan_safe = false;
  }
  RCLCPP_ERROR(
    get_logger(),
    "Runtime swept footprint rejected goal=%llu current_generation=%llu collisions=%zu "
    "discrete_samples=%zu swept_samples=%zu.",
    static_cast<unsigned long long>(active_reference->goal_id),
    static_cast<unsigned long long>(snapshot->generation), safety.collisions.size(),
    safety.discrete_samples_checked, safety.swept_samples_checked);
  publishPlannerStatus(
    active_reference->goal_id, active_reference->localization_epoch, snapshot->generation,
    active_reference->map_publication_sequence,
    ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
    ats_navigation_interfaces::msg::PlannerStatus::FAILURE_RUNTIME_UNSAFE);
}

void MincoPlannerNode::onGlobalPlan(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    setPlanSafe(false);
    return;
  }
  // 兼容 Nav2 时只取全局路径终点；真正的离散搜索仍由本节点在 RC-ESDF 上完成。
  onGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(msg->poses.back()));
}

void MincoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // P2/P3 过渡兼容入口：P2 保持直接 PoseStamped，不携带任务生命周期编号。
  planGoal(*msg, 0, 0, 0, false);
}

void MincoPlannerNode::onPlannerGoal(
  const ats_navigation_interfaces::msg::PlannerGoal::SharedPtr msg)
{
  if (msg->goal_id == 0) {
    publishPlannerStatus(
        0, msg->localization_epoch, 0, msg->map_publication_sequence,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_INVALID_GOAL);
    return;
  }
  publishPlannerStatus(
      msg->goal_id, msg->localization_epoch, 0, msg->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE);
  planGoal(
    msg->goal_pose, msg->goal_id, msg->localization_epoch,
    msg->map_publication_sequence, true);
}

void MincoPlannerNode::planGoal(
    const geometry_msgs::msg::PoseStamped &input_goal, std::uint64_t goal_id,
    std::uint64_t localization_epoch, std::uint64_t map_publication_sequence,
    bool report_status) {
  setPlanSafe(false);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    active_safety_reference_.reset();
  }
  const auto fail = [this, goal_id, localization_epoch, map_publication_sequence, report_status](
                        std::uint8_t failure_reason, std::uint64_t generation) {
    setPlanSafe(false);
    if (report_status) {
      publishPlannerStatus(
          goal_id, localization_epoch, generation, map_publication_sequence,
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No traversability grid received yet.");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY, 0);
    return;
  }
  const auto & planning_grid = map_snapshot->grid;
  const auto & clearance_esdf = map_snapshot->clearance_esdf;

  geometry_msgs::msg::PoseStamped start;
  if (!lookupStartPose(planning_grid, start)) {
    RCLCPP_WARN(get_logger(), "Cannot plan because start pose lookup failed.");
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
    RCLCPP_WARN(get_logger(), "Cannot plan because goal transform failed.");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_GOAL_TF,
      map_snapshot->generation);
    return;
  }
  goal = goal_in_grid;

  // 起点来自 TF、终点来自 goal_topic，因此此分支不依赖 Smac 的路径几何。
  GridAstarResult search_result;
  if (search_algorithm_ == "jps") {
    search_result = jps_.plan(planning_grid, start, goal);
    if (!search_result.success && astar_fallback_) {
      RCLCPP_WARN(
        get_logger(), "JPS failed (%s); falling back to A*.", search_result.reason.c_str());
      search_result = astar_.plan(planning_grid, start, goal);
    }
  } else {
    search_result = astar_.plan(planning_grid, start, goal);
  }
  if (!search_result.success) {
    RCLCPP_WARN(
      get_logger(), "%s failed: %s expanded=%d", search_algorithm_.c_str(),
      search_result.reason.c_str(), search_result.expanded_nodes);
    const std::uint8_t failure_reason =
      search_result.reason.find("occupied") != std::string::npos
      ? ats_navigation_interfaces::msg::PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED
      : ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NO_PATH;
    fail(failure_reason, map_snapshot->generation);
    return;
  }

  // 先生成质心 ESDF 候选，再以独立 yaw 的矩形足迹进行第二阶段内点修正。
  ReferenceTrajectory center_reference = optimizer_.optimize(search_result.path, clearance_esdf.get());
  if (!center_reference.valid()) {
    RCLCPP_ERROR(get_logger(), "MINCO returned an invalid center trajectory.");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  center_reference.header.stamp = now();
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  yaw_planner_.apply(center_reference, start_yaw, goal_yaw);
  annotateClearance(center_reference, *map_snapshot);
  FootprintSafetyResult center_safety = safety_checker_.check(center_reference, planning_grid);
  ReferenceTrajectory reference = center_reference;
  FootprintSafetyResult safety = center_safety;
  if (optimizer_.esdfFootprintOptimizationEnabled()) {
    ReferenceTrajectory footprint_reference = optimizer_.optimize(
      search_result.path, clearance_esdf.get(), &center_reference);
    if (footprint_reference.valid()) {
      footprint_reference.header.stamp = now();
      yaw_planner_.apply(footprint_reference, start_yaw, goal_yaw);
      annotateClearance(footprint_reference, *map_snapshot);
      FootprintSafetyResult footprint_safety = safety_checker_.check(
        footprint_reference, planning_grid);
      if (footprint_safety.safe || !center_safety.safe) {
        reference = std::move(footprint_reference);
        safety = std::move(footprint_safety);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Footprint-aware RC-ESDF candidate had %zu collisions; keeping safe center ESDF candidate.",
          footprint_safety.collisions.size());
      }
    }
  }
  if (!safety.safe && optimizer_.esdfObstacleOptimizationEnabled()) {
    // 外推候选仍碰撞时回到不做 ESDF 位移的 JPS-MINCO，避免“修正越修越差”。
    ReferenceTrajectory fallback_reference = optimizer_.optimize(search_result.path);
    if (fallback_reference.valid()) {
      fallback_reference.header.stamp = now();
      yaw_planner_.apply(fallback_reference, start_yaw, goal_yaw);
      annotateClearance(fallback_reference, *map_snapshot);
      const FootprintSafetyResult fallback_safety = safety_checker_.check(
        fallback_reference, planning_grid);
      if (fallback_safety.safe) {
        RCLCPP_WARN(
          get_logger(),
          "RC-ESDF outer candidate had %zu footprint collisions; using the safe JPS-MINCO baseline.",
          safety.collisions.size());
        reference = std::move(fallback_reference);
        safety = fallback_safety;
      }
    }
  }
  if (!safety.safe && collision_repair_.repair(reference, safety, planning_grid)) {
    // 局部修复只改变几何引导线，必须重新求 MINCO、yaw 和最终矩形足迹安全性。
    reference = optimizer_.optimize(toPath(reference), clearance_esdf.get(), &reference);
    if (!reference.valid()) {
      RCLCPP_ERROR(get_logger(), "Local collision repair produced an invalid MINCO trajectory.");
      fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REPAIR,
        map_snapshot->generation);
      return;
    }
    reference.header.stamp = now();
    yaw_planner_.apply(reference, start_yaw, goal_yaw);
    annotateClearance(reference, *map_snapshot);
    safety = safety_checker_.check(reference, planning_grid);
  }

  raw_path_pub_->publish(search_result.path);
  marker_pub_->publish(visualizer_.buildMarkers(search_result.path, reference, safety));
  if (!reference.valid()) {
    RCLCPP_ERROR(get_logger(), "Rejecting an invalid MINCO reference trajectory.");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  if (!safety.safe && !publish_unsafe_trajectory_) {
    if (safety.collisions.empty()) {
      RCLCPP_ERROR(get_logger(), "Rejecting MINCO trajectory because safety validation failed.");
    } else {
      const CollisionSample & first_collision = safety.collisions.front();
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting unsafe MINCO trajectory with %zu footprint collisions; first index=%zu at (%.3f, %.3f).",
        safety.collisions.size(), first_collision.trajectory_index,
        first_collision.x, first_collision.y);
    }
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  nav_msgs::msg::Path control_reference;
  if (!transformPathToGlobal(toPath(reference), control_reference)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot publish MINCO reference because the control-frame transform failed.");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REFERENCE_TF,
      map_snapshot->generation);
    return;
  }
  const std::uint8_t yaw_authority = selectYawAuthority(
    reference, force_body_yaw_follow_, body_yaw_follow_clearance_);
  if (!publishReferenceIfCurrent(map_snapshot, map_health_epoch,
                                 control_reference, reference, goal_id, localization_epoch,
                                 map_publication_sequence, yaw_authority,
                                 report_status)) {
    RCLCPP_WARN(
      get_logger(), "Discarded generation %llu because the planning map changed or became stale.",
      static_cast<unsigned long long>(map_snapshot->generation));
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
      map_snapshot->generation);
    return;
  }

  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (const auto & point : reference.points) {
    if (std::isfinite(point.clearance)) {
      minimum_clearance = std::min(minimum_clearance, point.clearance);
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "planned generation=%llu raw_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu "
    "expanded=%d yaw_authority=%u minimum_clearance=%.3f",
    static_cast<unsigned long long>(map_snapshot->generation),
    search_result.path.poses.size(), reference.points.size(), reference.totalLength(),
    reference.totalTime(), safety.collisions.size(), search_result.expanded_nodes,
    static_cast<unsigned int>(yaw_authority), minimum_clearance);
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
  const nav_msgs::msg::OccupancyGrid & grid, geometry_msgs::msg::PoseStamped & start) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      !grid.header.frame_id.empty() ? grid.header.frame_id : global_frame_,
      robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    start.header = transform.header;
    start.pose.position.x = transform.transform.translation.x;
    start.pose.position.y = transform.transform.translation.y;
    start.pose.position.z = transform.transform.translation.z;
    start.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "TF lookup %s -> %s failed: %s", global_frame_.c_str(), robot_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool MincoPlannerNode::transformGoalToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const
{
  const std::string target_frame = grid.header.frame_id.empty() ? global_frame_ : grid.header.frame_id;
  if (input.header.frame_id.empty() || input.header.frame_id == target_frame) {
    output = input;
    output.header.frame_id = target_frame;
    return true;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::doTransform(input, output, transform);
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(
      get_logger(), "TF lookup %s -> %s failed: %s", input.header.frame_id.c_str(),
      target_frame.c_str(), exception.what());
    return false;
  }
}

bool MincoPlannerNode::transformPathToGlobal(
  const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output) const
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
    RCLCPP_WARN(
      get_logger(), "TF lookup %s -> %s failed for MINCO reference: %s",
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
    std::uint64_t map_generation, std::uint64_t map_publication_sequence,
    std::uint8_t state, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time &reference_stamp,
    std::uint8_t yaw_authority, bool requires_gimbal_lock) {
  if (!planner_status_pub_) {
    return;
  }
  ats_navigation_interfaces::msg::PlannerStatus status;
  status.header.stamp = now();
  status.header.frame_id = global_frame_;
  status.goal_id = goal_id;
  status.localization_epoch = localization_epoch;
  status.map_generation = map_generation;
  status.map_publication_sequence = map_publication_sequence;
  status.state = state;
  status.reference_stamp = reference_stamp;
  status.failure_reason = failure_reason;
  status.yaw_authority = yaw_authority;
  status.requires_gimbal_lock = requires_gimbal_lock;
  planner_status_pub_->publish(status);
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
    std::uint64_t map_publication_sequence,
    std::uint8_t yaw_authority,
    bool report_status) {
  bool publish = false;
  nav_msgs::msg::Path candidate_reference;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const bool heartbeat_fresh = map_ready_topic_.empty() || steadyHeartbeatLeaseValid(
      last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_);
    if (!heartbeat_fresh && safety_state_.map_ready) {
      ++map_health_epoch_;
      latest_map_snapshot_.reset();
      safety_state_.invalidateMap(next_map_generation_);
    }
    publish = heartbeat_fresh && snapshot && latest_map_snapshot_ &&
      safety_state_.mapSnapshotUsable(snapshot->generation) &&
      map_health_epoch == map_health_epoch_ &&
      snapshot->generation == latest_map_snapshot_->generation;
    if (publish) {
      nav_msgs::msg::Path committed_reference = reference_path;
      rebasePathTimestamps(committed_reference, now());
      safety_state_.plan_safe = true;
      ActiveSafetyReference active_reference;
      active_reference.trajectory = safety_reference;
      active_reference.trajectory.header.stamp = committed_reference.header.stamp;
      active_reference.goal_id = goal_id;
      active_reference.localization_epoch = localization_epoch;
      active_reference.map_generation = snapshot->generation;
      active_reference.map_publication_sequence = map_publication_sequence;
      active_safety_reference_ = std::move(active_reference);
      if (planner_manages_emergency_stop_) {
        // P2 保留同一互斥区内的先解除急停、再发布正式 reference 契约。
        publishEmergencyStop(false);
        reference_path_pub_->publish(committed_reference);
      } else {
        // P3 不让规划器越权发布正式 reference；目标管理器会再次核对 heartbeat 后提交。
        candidate_reference = std::move(committed_reference);
      }
    } else {
      safety_state_.plan_safe = false;
      publishEmergencyStop(true);
    }
  }
  if (publish && !planner_manages_emergency_stop_) {
    if (!candidate_reference_path_pub_) {
      RCLCPP_ERROR(get_logger(), "P3 planner has no candidate reference publisher.");
      return false;
    }
    candidate_reference_path_pub_->publish(candidate_reference);
    if (report_status) {
      publishPlannerStatus(
          goal_id, localization_epoch, snapshot->generation, map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_REFERENCE_READY,
          ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE,
          candidate_reference.header.stamp, yaw_authority,
          yaw_authority == ats_navigation_interfaces::msg::PlannerStatus::
            YAW_AUTHORITY_BODY_YAW_FOLLOW);
    }
  }
  return publish;
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
