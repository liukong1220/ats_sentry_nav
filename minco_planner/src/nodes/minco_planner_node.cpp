// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{

MincoPlannerNode::MincoPlannerNode(const rclcpp::NodeOptions & options)
: Node("minco_planner", options),
  clearance_esdf_(std::make_shared<trajectory_optimizer::RcTraversabilityEsdfProvider>()),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  declareAndLoadParams();

  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&MincoPlannerNode::onGrid, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&MincoPlannerNode::onGoal, this, std::placeholders::_1));
  if (!global_plan_topic_.empty()) {
    global_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(1),
      std::bind(&MincoPlannerNode::onGlobalPlan, this, std::placeholders::_1));
  }
  raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, rclcpp::QoS(1));
  reference_path_pub_ =
    create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(1));
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic_, rclcpp::QoS(1));

  RCLCPP_INFO(
    get_logger(), "minco_planner ready: grid='%s' goal='%s' raw='%s' reference='%s'",
    grid_topic_.c_str(), goal_topic_.c_str(), raw_path_topic_.c_str(),
    reference_path_topic_.c_str());
}

void MincoPlannerNode::declareAndLoadParams()
{
  declare_parameter<std::string>("grid_topic", grid_topic_);
  declare_parameter<std::string>("goal_topic", goal_topic_);
  declare_parameter<std::string>("global_plan_topic", global_plan_topic_);
  declare_parameter<std::string>("raw_path_topic", raw_path_topic_);
  declare_parameter<std::string>("reference_path_topic", reference_path_topic_);
  declare_parameter<std::string>("debug_marker_topic", debug_marker_topic_);
  declare_parameter<std::string>("global_frame", global_frame_);
  declare_parameter<std::string>("robot_frame", robot_frame_);
  declare_parameter<std::string>("search_algorithm", search_algorithm_);
  declare_parameter<bool>("astar_fallback", astar_fallback_);
  declare_parameter<bool>("publish_unsafe_trajectory", publish_unsafe_trajectory_);

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

  YawSplinePlannerParams yaw_params;
  declare_parameter<std::string>("yaw_mode", yaw_params.mode);
  declare_parameter<double>("yaw_rate_limit", yaw_params.yaw_rate_limit);
  declare_parameter<double>("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  declare_parameter<double>("narrow_clearance_exit", yaw_params.narrow_clearance_exit);

  FootprintSafetyParams footprint_params;
  declare_parameter<double>("footprint_length", footprint_params.length);
  declare_parameter<double>("footprint_width", footprint_params.width);
  declare_parameter<double>("footprint_safety_margin", footprint_params.safety_margin);

  LocalCollisionRepairParams repair_params;
  declare_parameter<bool>("local_repair_enabled", repair_params.enabled);
  declare_parameter<int>("local_repair_max_iterations", repair_params.max_iterations);
  declare_parameter<double>("local_repair_search_radius", repair_params.search_radius);

  get_parameter("grid_topic", grid_topic_);
  get_parameter("goal_topic", goal_topic_);
  get_parameter("global_plan_topic", global_plan_topic_);
  get_parameter("raw_path_topic", raw_path_topic_);
  get_parameter("reference_path_topic", reference_path_topic_);
  get_parameter("debug_marker_topic", debug_marker_topic_);
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("search_algorithm", search_algorithm_);
  get_parameter("astar_fallback", astar_fallback_);
  get_parameter("publish_unsafe_trajectory", publish_unsafe_trajectory_);
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
  get_parameter("yaw_mode", yaw_params.mode);
  get_parameter("yaw_rate_limit", yaw_params.yaw_rate_limit);
  get_parameter("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  get_parameter("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  get_parameter("footprint_length", footprint_params.length);
  get_parameter("footprint_width", footprint_params.width);
  get_parameter("footprint_safety_margin", footprint_params.safety_margin);
  footprint_length_ = footprint_params.length;
  footprint_width_ = footprint_params.width;
  footprint_safety_margin_ = footprint_params.safety_margin;
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
  latest_grid_ = msg;
  clearance_esdf_->updateGrid(*msg, obstacle_value_threshold_, unknown_is_obstacle_);
}

void MincoPlannerNode::onGlobalPlan(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    return;
  }
  onGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(msg->poses.back()));
}

void MincoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!latest_grid_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No traversability grid received yet.");
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  if (!lookupStartPose(start)) {
    RCLCPP_WARN(get_logger(), "Cannot plan because start pose lookup failed.");
    return;
  }

  geometry_msgs::msg::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id =
      latest_grid_->header.frame_id.empty() ? global_frame_ : latest_grid_->header.frame_id;
  }
  geometry_msgs::msg::PoseStamped goal_in_grid;
  if (!transformGoalToGrid(goal, goal_in_grid)) {
    RCLCPP_WARN(get_logger(), "Cannot plan because goal transform failed.");
    return;
  }
  goal = goal_in_grid;

  GridAstarResult search_result;
  if (search_algorithm_ == "jps") {
    search_result = jps_.plan(*latest_grid_, start, goal);
    if (!search_result.success && astar_fallback_) {
      RCLCPP_WARN(
        get_logger(), "JPS failed (%s); falling back to A*.", search_result.reason.c_str());
      search_result = astar_.plan(*latest_grid_, start, goal);
    }
  } else {
    search_result = astar_.plan(*latest_grid_, start, goal);
  }
  if (!search_result.success) {
    RCLCPP_WARN(
      get_logger(), "%s failed: %s expanded=%d", search_algorithm_.c_str(),
      search_result.reason.c_str(), search_result.expanded_nodes);
    return;
  }

  ReferenceTrajectory reference = optimizer_.optimize(search_result.path, clearance_esdf_.get());
  reference.header.stamp = now();
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  yaw_planner_.apply(reference, start_yaw, goal_yaw);
  annotateClearance(reference);
  FootprintSafetyResult safety = safety_checker_.check(reference, *latest_grid_);
  if (!safety.safe && optimizer_.esdfObstacleOptimizationEnabled()) {
    ReferenceTrajectory fallback_reference = optimizer_.optimize(search_result.path);
    fallback_reference.header.stamp = now();
    yaw_planner_.apply(fallback_reference, start_yaw, goal_yaw);
    annotateClearance(fallback_reference);
    const FootprintSafetyResult fallback_safety = safety_checker_.check(
      fallback_reference, *latest_grid_);
    if (fallback_safety.safe) {
      RCLCPP_WARN(
        get_logger(),
        "RC-ESDF outer candidate had %zu footprint collisions; using the safe JPS-MINCO baseline.",
        safety.collisions.size());
      reference = std::move(fallback_reference);
      safety = fallback_safety;
    }
  }
  if (!safety.safe && collision_repair_.repair(reference, safety, *latest_grid_)) {
    reference = optimizer_.optimize(toPath(reference), clearance_esdf_.get());
    reference.header.stamp = now();
    yaw_planner_.apply(reference, start_yaw, goal_yaw);
    annotateClearance(reference);
    safety = safety_checker_.check(reference, *latest_grid_);
  }

  raw_path_pub_->publish(search_result.path);
  marker_pub_->publish(visualizer_.buildMarkers(search_result.path, reference, safety));
  if (!safety.safe && !publish_unsafe_trajectory_) {
    const CollisionSample & first_collision = safety.collisions.front();
    RCLCPP_ERROR(
      get_logger(),
      "Rejecting unsafe MINCO trajectory with %zu footprint collisions; first index=%zu at (%.3f, %.3f).",
      safety.collisions.size(), first_collision.trajectory_index,
      first_collision.x, first_collision.y);
    return;
  }
  reference_path_pub_->publish(toPath(reference));

  RCLCPP_INFO(
    get_logger(),
    "planned raw_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu expanded=%d",
    search_result.path.poses.size(), reference.points.size(), reference.totalLength(),
    reference.totalTime(), safety.collisions.size(), search_result.expanded_nodes);
}

void MincoPlannerNode::annotateClearance(ReferenceTrajectory & trajectory) const
{
  const bool available = clearance_esdf_ && clearance_esdf_->available();
  const std::vector<Eigen::Vector2d> samples = footprintSamples();
  for (auto & point : trajectory.points) {
    point.clearance = available ? clearance_esdf_->getFootprintClearance(
      Eigen::Vector2d(point.x, point.y), point.yaw, samples) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

std::vector<Eigen::Vector2d> MincoPlannerNode::footprintSamples() const
{
  const double half_length = 0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_;
  const double half_width = 0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_;
  const double resolution = latest_grid_ ?
    std::max(0.02, static_cast<double>(latest_grid_->info.resolution)) : 0.05;
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

bool MincoPlannerNode::lookupStartPose(geometry_msgs::msg::PoseStamped & start) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      latest_grid_ && !latest_grid_->header.frame_id.empty() ? latest_grid_->header.frame_id
                                                             : global_frame_,
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
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const
{
  const std::string target_frame = latest_grid_ && !latest_grid_->header.frame_id.empty()
                                     ? latest_grid_->header.frame_id
                                     : global_frame_;
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
