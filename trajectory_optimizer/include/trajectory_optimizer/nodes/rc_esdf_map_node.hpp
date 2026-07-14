// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__NODES__RC_ESDF_MAP_NODE_HPP_
#define TRAJECTORY_OPTIMIZER__NODES__RC_ESDF_MAP_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"
#include "trajectory_optimizer/esdf/static_map_fusion.hpp"

namespace trajectory_optimizer
{

class RcEsdfMapNode : public rclcpp::Node
{
public:
  explicit RcEsdfMapNode(const rclcpp::NodeOptions & options);

private:
  void onStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onTraversabilityGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void rebuild();
  nav_msgs::msg::OccupancyGrid resampleTraversabilityGrid(
    const nav_msgs::msg::OccupancyGrid & input) const;
  nav_msgs::msg::OccupancyGrid encodeDistanceGrid(
    const nav_msgs::msg::OccupancyGrid & planning_grid,
    const std::vector<double> & distance_field,
    double clearance_offset) const;

  std::string static_map_topic_ = "/map";
  std::string traversability_grid_topic_ = "traversability_grid";
  std::string planning_grid_topic_ = "rc_esdf/planning_grid";
  std::string signed_distance_grid_topic_ = "rc_esdf/signed_distance_grid";
  std::string footprint_clearance_grid_topic_ = "rc_esdf/footprint_clearance_grid";
  std::string static_map_frame_ = "map";
  StaticMapFusionParams fusion_params_;
  // Zero preserves the terrain grid resolution. A finer value lets static walls
  // retain their geometry without turning every coarse terrain cell into a wall.
  double planning_grid_resolution_ = 0.0;
  double signed_distance_max_m_ = 2.0;
  double footprint_length_ = 0.60;
  double footprint_width_ = 0.50;
  double footprint_safety_margin_ = 0.02;

  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_grid_;
  std::shared_ptr<RcTraversabilityEsdfProvider> esdf_provider_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_grid_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr signed_distance_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr footprint_clearance_grid_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__NODES__RC_ESDF_MAP_NODE_HPP_
