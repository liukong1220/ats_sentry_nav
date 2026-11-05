// Copyright 2026

#ifndef ATS_ROG_MAP__PLANNING__PLANNING_MAP_SNAPSHOT_HPP_
#define ATS_ROG_MAP__PLANNING__PLANNING_MAP_SNAPSHOT_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ats_rog_map::planning
{

inline ats_navigation_interfaces::msg::PlanningMapSnapshot makeUnavailablePlanningMapSnapshot(
  const nav_msgs::msg::OccupancyGrid & grid,
  const builtin_interfaces::msg::Time & publication_stamp,
  std::uint64_t localization_epoch, std::uint64_t source_generation,
  std::uint64_t publication_sequence, bool unknown_is_obstacle,
  int occupied_value_threshold)
{
  ats_navigation_interfaces::msg::PlanningMapSnapshot snapshot;
  snapshot.header.stamp = publication_stamp;
  snapshot.header.frame_id = grid.header.frame_id;
  snapshot.source_stamp = grid.header.stamp;
  snapshot.ready = false;
  snapshot.unknown_is_obstacle = unknown_is_obstacle;
  snapshot.occupied_value_threshold = static_cast<std::int8_t>(occupied_value_threshold);
  snapshot.localization_epoch = localization_epoch;
  snapshot.source_generation = source_generation;
  snapshot.publication_sequence = publication_sequence;
  snapshot.info = grid.info;
  return snapshot;
}

// An unavailable snapshot normally carries only identity and health metadata.
// The blocked variant additionally preserves a numerically inspectable grid
// for fault auditing while remaining unconditionally non-executable.
inline ats_navigation_interfaces::msg::PlanningMapSnapshot makeBlockedUnavailablePlanningMapSnapshot(
  const nav_msgs::msg::OccupancyGrid & grid,
  const builtin_interfaces::msg::Time & publication_stamp,
  std::uint64_t localization_epoch, std::uint64_t source_generation,
  std::uint64_t publication_sequence, bool unknown_is_obstacle,
  int occupied_value_threshold)
{
  auto snapshot = makeUnavailablePlanningMapSnapshot(
    grid, publication_stamp, localization_epoch, source_generation,
    publication_sequence, unknown_is_obstacle, occupied_value_threshold);
  const auto expected_cells =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (expected_cells == 0U || grid.data.size() != expected_cells) {
    return snapshot;
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  snapshot.occupancy = grid.data;
  snapshot.signed_distance_m.assign(expected_cells, nan);
  snapshot.gradient_x.assign(expected_cells, nan);
  snapshot.gradient_y.assign(expected_cells, nan);
  return snapshot;
}

inline ats_navigation_interfaces::msg::PlanningMapSnapshot makePlanningMapSnapshot(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<double> & signed_distance,
  const builtin_interfaces::msg::Time & publication_stamp,
  std::uint64_t localization_epoch, std::uint64_t source_generation,
  std::uint64_t publication_sequence, bool unknown_is_obstacle,
  int occupied_value_threshold)
{
  auto snapshot = makeUnavailablePlanningMapSnapshot(
    grid, publication_stamp, localization_epoch, source_generation,
    publication_sequence, unknown_is_obstacle, occupied_value_threshold);
  const auto expected_cells =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.header.frame_id.empty() || grid.info.resolution <= 0.0F ||
    expected_cells == 0U || grid.data.size() != expected_cells ||
    signed_distance.size() != expected_cells || source_generation == 0U ||
    publication_sequence == 0U)
  {
    return snapshot;
  }

  // Every cell carries a finite signed distance. A cell is blocked when it is
  // occupied, or unknown while unknown_is_obstacle holds; blocked cells are
  // non-positive and the rest non-negative. Consumers interpolate across the
  // unknown boundary, so unknown cells cannot be NaN. Any violation means the
  // producer is broken and the snapshot is never marked ready.
  const auto blocked = [&grid, unknown_is_obstacle, occupied_value_threshold](std::size_t index) {
      const auto value = grid.data[index];
      return value >= occupied_value_threshold || (value < 0 && unknown_is_obstacle);
    };
  for (std::size_t index = 0; index < expected_cells; ++index) {
    const double value = signed_distance[index];
    if (!std::isfinite(value) || (blocked(index) ? value > 0.0 : value < 0.0)) {
      return snapshot;
    }
  }

  snapshot.ready = true;
  snapshot.occupancy = grid.data;
  snapshot.signed_distance_m.resize(expected_cells);
  snapshot.gradient_x.resize(expected_cells);
  snapshot.gradient_y.resize(expected_cells);
  const double resolution = grid.info.resolution;
  const double origin_yaw = std::atan2(
    2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
      grid.info.origin.orientation.x * grid.info.origin.orientation.y),
    1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
      grid.info.origin.orientation.z * grid.info.origin.orientation.z));
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const auto at = [&signed_distance, width](int x, int y) {
      return signed_distance[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
               static_cast<std::size_t>(x)];
    };
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      const double center = signed_distance[index];
      snapshot.signed_distance_m[index] = static_cast<float>(center);
      const double gradient_local_x = width < 2 ? 0.0 :
        (x == 0 ? (at(1, y) - center) / resolution :
        (x == width - 1 ? (center - at(x - 1, y)) / resolution :
        (at(x + 1, y) - at(x - 1, y)) / (2.0 * resolution)));
      const double gradient_local_y = height < 2 ? 0.0 :
        (y == 0 ? (at(x, 1) - center) / resolution :
        (y == height - 1 ? (center - at(x, y - 1)) / resolution :
        (at(x, y + 1) - at(x, y - 1)) / (2.0 * resolution)));
      snapshot.gradient_x[index] = static_cast<float>(
        std::cos(origin_yaw) * gradient_local_x - std::sin(origin_yaw) * gradient_local_y);
      snapshot.gradient_y[index] = static_cast<float>(
        std::sin(origin_yaw) * gradient_local_x + std::cos(origin_yaw) * gradient_local_y);
    }
  }
  return snapshot;
}

}  // namespace ats_rog_map::planning

#endif  // ATS_ROG_MAP__PLANNING__PLANNING_MAP_SNAPSHOT_HPP_
