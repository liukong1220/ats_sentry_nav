// Copyright 2026

#ifndef MINCO_PLANNER__GRID_SEARCH_HPP_
#define MINCO_PLANNER__GRID_SEARCH_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "minco_planner/planning/grid_astar.hpp"

namespace minco_planner
{

struct GridSearchParams : GridAstarParams
{
  bool jump_enabled = true;
  int max_expanded_nodes = -1;
};

struct GridSearchResult : GridAstarResult
{
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
