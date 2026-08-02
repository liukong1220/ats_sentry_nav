// Copyright 2026

#ifndef ATS_RC_ESDF__ESDF__STATIC_MAP_FUSION_HPP_
#define ATS_RC_ESDF__ESDF__STATIC_MAP_FUSION_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ats_rc_esdf
{

struct StaticMapFusionParams
{
  int static_obstacle_value_threshold = 50;
  int local_obstacle_value_threshold = 50;
};

// Fuses a static map into a rolling traversability grid without assuming that
// their frames are identical. The returned grid stays in the local grid frame.
class StaticMapFusion
{
public:
  static bool buildPlanningGrid(
    const nav_msgs::msg::OccupancyGrid & local_grid,
    const nav_msgs::msg::OccupancyGrid & static_map,
    const geometry_msgs::msg::TransformStamped & static_from_local,
    const StaticMapFusionParams & params,
    nav_msgs::msg::OccupancyGrid & planning_grid);
};

}  // namespace ats_rc_esdf
#endif  // ATS_RC_ESDF__ESDF__STATIC_MAP_FUSION_HPP_
