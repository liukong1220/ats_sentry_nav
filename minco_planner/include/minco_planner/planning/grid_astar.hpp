// Copyright 2026

#ifndef MINCO_PLANNER__GRID_ASTAR_HPP_
#define MINCO_PLANNER__GRID_ASTAR_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "minco_planner/planning/grid_clearance.hpp"

namespace minco_planner
{

struct GridAstarParams
{
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  bool allow_diagonal = true;
  double diagonal_cost = 1.41421356237;
  /// Omnidirectional clearance every expanded cell must have, in metres.
  ///
  /// This lives in the base params on purpose: when it only existed on
  /// GridJpsParams, the JPS -> A* fallback planned with zero clearance and
  /// produced wall-hugging seeds that the footprint gate had to reject, which
  /// the goal manager then retried as a transient failure until goal timeout.
  double safe_distance = 0.0;
  /// Admit the start cell even when it fails the clearance ring.
  ///
  /// The robot already occupies that pose; refusing to plan cannot move it, so a
  /// fail-closed rejection here is a livelock rather than a safety measure. The
  /// yaw-aware footprint gate and local collision repair stay authoritative.
  bool assume_start_traversable = true;
  /// Fall back to the clearance a start/goal cell actually has, capped by
  /// `safe_distance`, instead of rejecting the endpoint outright.
  bool relax_endpoint_clearance = true;
  /// Hard lower bound the endpoint relaxation may not go below, in metres.
  ///
  /// Without it the relaxation would re-create the zero-clearance fallback it
  /// exists to remove: a goal sitting in a 0.05 m nook would drag the whole
  /// search down to 0.05 m. Set this to the inscribed footprint half-width, the
  /// clearance below which no body yaw fits at all.
  double min_safe_distance = 0.0;
};

struct GridAstarResult
{
  bool success = false;
  std::string reason;
  nav_msgs::msg::Path path;
  double length = 0.0;
  int expanded_nodes = 0;
};

class GridAstar
{
public:
  explicit GridAstar(GridAstarParams params = GridAstarParams());

  void setParams(const GridAstarParams & params);
  GridAstarResult plan(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Plan with a one-off clearance override; negative keeps the configured value.
  GridAstarResult planWithClearance(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double clearance_override) const;

private:
  struct GridIndex
  {
    int x = 0;
    int y = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx,
    double wy,
    GridIndex & out) const;
  geometry_msgs::msg::PoseStamped gridToPose(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridIndex & index) const;
  bool isTraversable(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  GridOccupancyPolicy occupancyPolicy() const;
  std::size_t linearIndex(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  double heuristic(const GridIndex & from, const GridIndex & to) const;

  GridAstarParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GRID_ASTAR_HPP_
