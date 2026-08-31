// Copyright 2026

#ifndef MINCO_PLANNER__PLANNING__GRID_JPS_HPP_
#define MINCO_PLANNER__PLANNING__GRID_JPS_HPP_

#include <vector>

#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_clearance.hpp"

namespace minco_planner
{

struct GridJpsParams : public GridAstarParams
{
  int max_expanded_nodes = -1;
};

class GridJps
{
public:
  explicit GridJps(GridJpsParams params = GridJpsParams());

  void setParams(const GridJpsParams & params);
  GridAstarResult plan(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Plan with a one-off clearance override; negative keeps the configured value.
  ///
  /// Const and non-mutating so the node can walk a graduated clearance ladder
  /// without touching shared search state from another callback group.
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

  struct Direction
  {
    int dx = 0;
    int dy = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx,
    double wy,
    GridIndex & out) const;
  geometry_msgs::msg::PoseStamped gridToPose(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridIndex & index) const;
  bool isTraversable(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const;
  bool isCellFree(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const;
  GridOccupancyPolicy occupancyPolicy() const;
  std::size_t linearIndex(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const;
  bool hasForcedNeighbor(
    const nav_msgs::msg::OccupancyGrid & grid,
    int x,
    int y,
    int dx,
    int dy) const;
  bool jump(
    const nav_msgs::msg::OccupancyGrid & grid,
    int x,
    int y,
    int dx,
    int dy,
    const GridIndex & goal,
    GridIndex & jump_point) const;
  std::vector<Direction> prunedDirections(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridIndex & current,
    const Direction & incoming) const;

  GridJpsParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__GRID_JPS_HPP_
