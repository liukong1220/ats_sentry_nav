// Copyright 2026

#ifndef MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_
#define MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

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

/// 一维精确平方距离变换（Felzenszwalb & Huttenlocher）。f 为输入代价，d 为输出。
inline void squaredDistanceTransform1d(
  const std::vector<double> & f, std::vector<double> & d,
  std::vector<int> & v, std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  d.assign(f.size(), std::numeric_limits<double>::infinity());
  v.assign(f.size(), 0);
  z.assign(f.size() + 1U, 0.0);
  const double inf = std::numeric_limits<double>::infinity();
  int k = -1;
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(f[q])) {
      continue;
    }
    if (k < 0) {
      k = 0;
      v[0] = q;
      z[0] = -inf;
      z[1] = inf;
      continue;
    }
    double s = 0.0;
    while (true) {
      const int p = v[k];
      s = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) /
        (2.0 * (q - p));
      if (s <= z[k] && k > 0) {
        --k;
        continue;
      }
      break;
    }
    if (s <= z[k]) {
      // k == 0 且新抛物线完全占优：替换。
      v[0] = q;
      z[0] = -inf;
      z[1] = inf;
      continue;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = inf;
  }
  if (k < 0) {
    return;
  }
  int j = 0;
  for (int q = 0; q < n; ++q) {
    while (z[j + 1] < q) {
      ++j;
    }
    const double offset = static_cast<double>(q - v[j]);
    d[q] = offset * offset + f[v[j]];
  }
}

/// 每个格心到最近"阻塞格"格心的平方距离（单位：格²）。阻塞格与 isGridCellFree 一致，
/// 栅格外一圈视为阻塞，所以结果与 hasGridClearance 的判据完全等价：
/// hasGridClearance(x, y, r) <=> isGridCellFree(x, y) && squared[x, y] > (r / res)²。
inline std::vector<double> computeBlockedSquaredDistanceCells(
  const nav_msgs::msg::OccupancyGrid & grid, const GridOccupancyPolicy & policy)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int padded_width = width + 2;
  const int padded_height = height + 2;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> field(
    static_cast<std::size_t>(padded_width) * static_cast<std::size_t>(padded_height), inf);
  for (int y = -1; y <= height; ++y) {
    for (int x = -1; x <= width; ++x) {
      if (!isGridCellFree(grid, x, y, policy)) {
        field[static_cast<std::size_t>(y + 1) * padded_width + static_cast<std::size_t>(x + 1)] =
          0.0;
      }
    }
  }
  std::vector<double> line;
  std::vector<double> out;
  std::vector<int> v;
  std::vector<double> z;
  // 先按列再按行做两遍一维变换，得到精确欧氏平方距离。
  line.resize(static_cast<std::size_t>(padded_height));
  for (int x = 0; x < padded_width; ++x) {
    for (int y = 0; y < padded_height; ++y) {
      line[static_cast<std::size_t>(y)] =
        field[static_cast<std::size_t>(y) * padded_width + static_cast<std::size_t>(x)];
    }
    squaredDistanceTransform1d(line, out, v, z);
    for (int y = 0; y < padded_height; ++y) {
      field[static_cast<std::size_t>(y) * padded_width + static_cast<std::size_t>(x)] =
        out[static_cast<std::size_t>(y)];
    }
  }
  line.resize(static_cast<std::size_t>(padded_width));
  for (int y = 0; y < padded_height; ++y) {
    for (int x = 0; x < padded_width; ++x) {
      line[static_cast<std::size_t>(x)] =
        field[static_cast<std::size_t>(y) * padded_width + static_cast<std::size_t>(x)];
    }
    squaredDistanceTransform1d(line, out, v, z);
    for (int x = 0; x < padded_width; ++x) {
      field[static_cast<std::size_t>(y) * padded_width + static_cast<std::size_t>(x)] =
        out[static_cast<std::size_t>(x)];
    }
  }
  std::vector<double> result(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), inf);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      result[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] =
        field[static_cast<std::size_t>(y + 1) * padded_width + static_cast<std::size_t>(x + 1)];
    }
  }
  return result;
}

/// 世界坐标点所在格到最近阻塞格的格心距离（米）；点在栅格外返回 0。
inline double blockedDistanceAt(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & squared_cells,
  double wx, double wy)
{
  const double resolution = grid.info.resolution;
  if (!(resolution > 0.0) || squared_cells.empty()) {
    return 0.0;
  }
  const double gx = (wx - grid.info.origin.position.x) / resolution;
  const double gy = (wy - grid.info.origin.position.y) / resolution;
  if (gx < 0.0 || gy < 0.0 || gx >= static_cast<double>(grid.info.width) ||
    gy >= static_cast<double>(grid.info.height))
  {
    return 0.0;
  }
  const std::size_t index =
    static_cast<std::size_t>(std::floor(gy)) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(std::floor(gx));
  return std::sqrt(squared_cells[index]) * resolution;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__GRID_CLEARANCE_HPP_
