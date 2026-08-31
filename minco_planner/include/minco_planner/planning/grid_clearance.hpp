// Copyright 2026

#ifndef MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_
#define MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

/// Occupancy interpretation shared by every grid search layer.
///
/// Keeping this in one place is deliberate: JPS and A* used to disagree about
/// what "traversable" means, which let the JPS -> A* fallback silently drop the
/// clearance requirement to zero and hand MINCO a wall-hugging seed path.
struct GridOccupancyPolicy
{
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
};

/// Out-of-bounds counts as blocked; the planner may not route outside the map.
inline bool isGridCellFree(
  const nav_msgs::msg::OccupancyGrid & grid, int x, int y, const GridOccupancyPolicy & policy)
{
  if (
    x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
    y >= static_cast<int>(grid.info.height)) {
    return false;
  }
  const std::size_t index =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(x);
  const int8_t value = grid.data[index];
  return value < 0 ? !policy.unknown_is_obstacle : value < policy.obstacle_value_threshold;
}

/// True when the disc of `radius_m` around the cell contains no blocked cell.
inline bool hasGridClearance(
  const nav_msgs::msg::OccupancyGrid & grid, int x, int y, double radius_m,
  const GridOccupancyPolicy & policy)
{
  if (!isGridCellFree(grid, x, y, policy)) {
    return false;
  }
  if (!(radius_m > 0.0) || grid.info.resolution <= 0.0) {
    return true;
  }
  const double radius_cells = radius_m / grid.info.resolution;
  const int radius = static_cast<int>(std::ceil(radius_cells));
  const double radius_squared = radius_cells * radius_cells;
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      if (offset_x * offset_x + offset_y * offset_y > radius_squared) {
        continue;
      }
      if (!isGridCellFree(grid, x + offset_x, y + offset_y, policy)) {
        return false;
      }
    }
  }
  return true;
}

/// Largest radius (metres, capped at `cap_m`) for which `hasGridClearance` holds.
///
/// Returns 0.0 when the cell itself is blocked. Used to relax endpoint admission
/// to the clearance a pose actually has instead of rejecting it outright.
inline double measureGridClearance(
  const nav_msgs::msg::OccupancyGrid & grid, int x, int y, double cap_m,
  const GridOccupancyPolicy & policy)
{
  if (!isGridCellFree(grid, x, y, policy)) {
    return 0.0;
  }
  const double cap = std::max(0.0, cap_m);
  if (cap <= 0.0 || grid.info.resolution <= 0.0) {
    return 0.0;
  }
  const int radius = static_cast<int>(std::ceil(cap / grid.info.resolution));
  double nearest_blocked_squared = std::numeric_limits<double>::infinity();
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      if (offset_x == 0 && offset_y == 0) {
        continue;
      }
      const double distance_squared =
        static_cast<double>(offset_x * offset_x + offset_y * offset_y);
      if (distance_squared >= nearest_blocked_squared) {
        continue;
      }
      if (!isGridCellFree(grid, x + offset_x, y + offset_y, policy)) {
        nearest_blocked_squared = distance_squared;
      }
    }
  }
  if (!std::isfinite(nearest_blocked_squared)) {
    return cap;
  }
  // hasGridClearance(r) requires every blocked cell to satisfy
  // off^2 > (r/res)^2, so the admissible radius is strictly below the nearest
  // blocked distance. Step back one epsilon so the returned value round-trips.
  const double nearest = std::sqrt(nearest_blocked_squared) * grid.info.resolution;
  return std::max(0.0, std::min(cap, std::nextafter(nearest, 0.0)));
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_
