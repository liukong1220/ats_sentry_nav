// Copyright 2026

#ifndef MINCO_PLANNER__GRID_SEARCH_HPP_
#define MINCO_PLANNER__GRID_SEARCH_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "minco_planner/planning/grid_clearance.hpp"

namespace minco_planner
{

struct GridSearchParams
{
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  bool allow_diagonal = true;
  double diagonal_cost = 1.41421356237;
  // Omnidirectional clearance every expanded cell must have, in metres.
  double safe_distance = 0.0;
  // The robot already occupies the start pose; rejecting it cannot move it.
  bool assume_start_traversable = true;
  // Use the clearance an endpoint actually has, capped by safe_distance.
  bool relax_endpoint_clearance = true;
  // Lower bound for endpoint relaxation (inscribed footprint half-width).
  double min_safe_distance = 0.0;
  // Step cost multiplier 1 + w * ((D - d) / D)^2 for d < D; <= 0 disables.
  double clearance_cost_weight = 0.0;
  double clearance_cost_distance = 0.0;
  // Cells the start may climb along non-decreasing clearance; 0 disables.
  int start_escape_max_steps = 0;
  bool jump_enabled = true;
  int max_expanded_nodes = -1;
};

struct GridSearchResult
{
  bool success = false;
  std::string reason;
  nav_msgs::msg::Path path;
  double length = 0.0;
  int expanded_nodes = 0;
  double cost = 0.0;  // Weighted graph cost in cell units.
  double clearance = 0.0;
  bool partial = false;  // A stoppable segment. Only success denotes the requested goal.
};

struct GridSearchBounds
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

struct GridSearchOptions
{
  double clearance = -1.0;
  double endpoint_minimum = -1.0;
  std::optional<GridSearchBounds> bounds;
  bool strict_endpoints = false;
  // Opt in on a known-free grid and validate the directed terminal footprint.
  std::function<bool(const geometry_msgs::msg::PoseStamped &)> can_stop;
};

// One workspace per caller. Searches using the same workspace run serially.
class GridSearch
{
public:
  explicit GridSearch(GridSearchParams params = {});

  void setParams(const GridSearchParams & params) {params_ = params;}

  // blocked_squared must describe this grid and the configured occupancy policy.
  // Standalone callers can omit it and have the search compute the field.
  GridSearchResult plan(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<double> * blocked_squared = nullptr,
    const GridSearchOptions & options = {});

private:
  struct Cell
  {
    double g = 0.0;
    int parent = -1;
    int escape_steps = 0;
    std::uint32_t visited = 0;
    std::uint32_t closed = 0;
    std::uint32_t neighborhood_checked = 0;
    bool uniform_neighborhood = false;
  };

  GridSearchParams params_;
  std::vector<Cell> cells_;
  std::uint32_t generation_ = 0;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GRID_SEARCH_HPP_
