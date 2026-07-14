// Copyright 2026

#include "trajectory_optimizer/nodes/rc_esdf_map_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rclcpp_components/register_node_macro.hpp"

namespace trajectory_optimizer
{

RcEsdfMapNode::RcEsdfMapNode(const rclcpp::NodeOptions & options)
: Node("rc_esdf_map", options),
  esdf_provider_(std::make_shared<RcTraversabilityEsdfProvider>()),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  declare_parameter<std::string>("static_map_topic", static_map_topic_);
  declare_parameter<std::string>("traversability_grid_topic", traversability_grid_topic_);
  declare_parameter<std::string>("planning_grid_topic", planning_grid_topic_);
  declare_parameter<std::string>("signed_distance_grid_topic", signed_distance_grid_topic_);
  declare_parameter<std::string>(
    "footprint_clearance_grid_topic", footprint_clearance_grid_topic_);
  declare_parameter<std::string>("static_map_frame", static_map_frame_);
  declare_parameter<int>(
    "static_obstacle_value_threshold", fusion_params_.static_obstacle_value_threshold);
  declare_parameter<int>(
    "local_obstacle_value_threshold", fusion_params_.local_obstacle_value_threshold);
  declare_parameter<double>("planning_grid_resolution", planning_grid_resolution_);
  declare_parameter<double>("signed_distance_max_m", signed_distance_max_m_);
  declare_parameter<double>("footprint_length", footprint_length_);
  declare_parameter<double>("footprint_width", footprint_width_);
  declare_parameter<double>("footprint_safety_margin", footprint_safety_margin_);

  get_parameter("static_map_topic", static_map_topic_);
  get_parameter("traversability_grid_topic", traversability_grid_topic_);
  get_parameter("planning_grid_topic", planning_grid_topic_);
  get_parameter("signed_distance_grid_topic", signed_distance_grid_topic_);
  get_parameter("footprint_clearance_grid_topic", footprint_clearance_grid_topic_);
  get_parameter("static_map_frame", static_map_frame_);
  get_parameter(
    "static_obstacle_value_threshold", fusion_params_.static_obstacle_value_threshold);
  get_parameter(
    "local_obstacle_value_threshold", fusion_params_.local_obstacle_value_threshold);
  get_parameter("planning_grid_resolution", planning_grid_resolution_);
  get_parameter("signed_distance_max_m", signed_distance_max_m_);
  get_parameter("footprint_length", footprint_length_);
  get_parameter("footprint_width", footprint_width_);
  get_parameter("footprint_safety_margin", footprint_safety_margin_);
  signed_distance_max_m_ = std::max(signed_distance_max_m_, 1e-3);
  planning_grid_resolution_ = std::max(0.0, planning_grid_resolution_);
  footprint_length_ = std::max(footprint_length_, 0.0);
  footprint_width_ = std::max(footprint_width_, 0.0);
  footprint_safety_margin_ = std::max(footprint_safety_margin_, 0.0);

  static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    static_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&RcEsdfMapNode::onStaticMap, this, std::placeholders::_1));
  traversability_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_grid_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&RcEsdfMapNode::onTraversabilityGrid, this, std::placeholders::_1));
  const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
  planning_grid_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>(planning_grid_topic_, output_qos);
  signed_distance_grid_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>(signed_distance_grid_topic_, output_qos);
  footprint_clearance_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    footprint_clearance_grid_topic_, output_qos);

  RCLCPP_INFO(
    get_logger(), "RC-ESDF map ready: static='%s' terrain='%s' planning='%s'",
    static_map_topic_.c_str(), traversability_grid_topic_.c_str(), planning_grid_topic_.c_str());
}

void RcEsdfMapNode::onStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  static_map_ = msg;
  rebuild();
}

void RcEsdfMapNode::onTraversabilityGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_grid_ = msg;
  rebuild();
}

void RcEsdfMapNode::rebuild()
{
  if (!static_map_ || !traversability_grid_) {
    return;
  }

  const std::string static_frame = static_map_->header.frame_id.empty() ?
    static_map_frame_ : static_map_->header.frame_id;
  const std::string local_frame = traversability_grid_->header.frame_id;
  if (local_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "RC-ESDF map ignored a terrain grid without frame_id.");
    return;
  }

  geometry_msgs::msg::TransformStamped static_from_local;
  if (static_frame == local_frame) {
    static_from_local.header.frame_id = static_frame;
    static_from_local.child_frame_id = local_frame;
    static_from_local.transform.rotation.w = 1.0;
  } else {
    try {
      static_from_local = tf_buffer_->lookupTransform(
        static_frame, local_frame, rclcpp::Time(traversability_grid_->header.stamp),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "RC-ESDF map waits for TF %s -> %s: %s",
        local_frame.c_str(), static_frame.c_str(), exception.what());
      return;
    }
  }

  const nav_msgs::msg::OccupancyGrid terrain_grid =
    resampleTraversabilityGrid(*traversability_grid_);
  nav_msgs::msg::OccupancyGrid planning_grid;
  if (!StaticMapFusion::buildPlanningGrid(
      terrain_grid, *static_map_, static_from_local, fusion_params_, planning_grid))
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "RC-ESDF map rejected malformed static or terrain grid.");
    return;
  }

  planning_grid_pub_->publish(planning_grid);
  // Unknown cells retain -1 in the published planning grid, but are deliberately
  // treated as obstacles by the distance field and all path consumers.
  esdf_provider_->updateGrid(
    planning_grid, fusion_params_.local_obstacle_value_threshold, true,
    fusion_params_.local_obstacle_value_threshold);
  std::vector<double> distance_field;
  if (!esdf_provider_->copySignedDistanceField(distance_field)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "RC-ESDF map has no valid signed distance field yet.");
    return;
  }

  signed_distance_grid_pub_->publish(encodeDistanceGrid(planning_grid, distance_field, 0.0));
  const double all_yaw_footprint_radius = 0.5 * std::hypot(footprint_length_, footprint_width_) +
    footprint_safety_margin_;
  footprint_clearance_grid_pub_->publish(
    encodeDistanceGrid(planning_grid, distance_field, all_yaw_footprint_radius));
}

nav_msgs::msg::OccupancyGrid RcEsdfMapNode::resampleTraversabilityGrid(
  const nav_msgs::msg::OccupancyGrid & input) const
{
  const std::size_t input_size =
    static_cast<std::size_t>(input.info.width) * input.info.height;
  if (input.info.resolution <= 0.0F || input.data.size() < input_size) {
    return input;
  }
  if (planning_grid_resolution_ <= 0.0 ||
    planning_grid_resolution_ >= input.info.resolution - 1e-6)
  {
    return input;
  }

  nav_msgs::msg::OccupancyGrid output = input;
  const double extent_x = static_cast<double>(input.info.width) * input.info.resolution;
  const double extent_y = static_cast<double>(input.info.height) * input.info.resolution;
  output.info.resolution = planning_grid_resolution_;
  output.info.width = static_cast<uint32_t>(std::ceil(extent_x / planning_grid_resolution_));
  output.info.height = static_cast<uint32_t>(std::ceil(extent_y / planning_grid_resolution_));
  output.data.assign(
    static_cast<std::size_t>(output.info.width) * output.info.height, -1);

  for (uint32_t y = 0; y < output.info.height; ++y) {
    for (uint32_t x = 0; x < output.info.width; ++x) {
      const double local_x = (static_cast<double>(x) + 0.5) * output.info.resolution;
      const double local_y = (static_cast<double>(y) + 0.5) * output.info.resolution;
      const int source_x = static_cast<int>(std::floor(local_x / input.info.resolution));
      const int source_y = static_cast<int>(std::floor(local_y / input.info.resolution));
      if (source_x < 0 || source_y < 0 ||
        source_x >= static_cast<int>(input.info.width) ||
        source_y >= static_cast<int>(input.info.height))
      {
        continue;
      }
      const std::size_t source_index =
        static_cast<std::size_t>(source_y) * input.info.width + source_x;
      const std::size_t output_index = static_cast<std::size_t>(y) * output.info.width + x;
      output.data[output_index] = input.data[source_index];
    }
  }
  return output;
}

nav_msgs::msg::OccupancyGrid RcEsdfMapNode::encodeDistanceGrid(
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const std::vector<double> & distance_field,
  double clearance_offset) const
{
  nav_msgs::msg::OccupancyGrid encoded = planning_grid;
  encoded.data.assign(planning_grid.data.size(), -1);
  const std::size_t size = std::min(planning_grid.data.size(), distance_field.size());
  for (std::size_t index = 0; index < size; ++index) {
    if (planning_grid.data[index] < 0 || !std::isfinite(distance_field[index])) {
      continue;
    }
    const double normalized = std::max(
      -1.0, std::min(1.0, (distance_field[index] - clearance_offset) / signed_distance_max_m_));
    encoded.data[index] = static_cast<int8_t>(std::lround(50.0 + 50.0 * normalized));
  }
  return encoded;
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::RcEsdfMapNode)
