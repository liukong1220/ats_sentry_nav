// Copyright 2026

#ifndef ATS_ROG_MAP_ADAPTER__PLANNING_MAP_SNAPSHOT_HPP_
#define ATS_ROG_MAP_ADAPTER__PLANNING_MAP_SNAPSHOT_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ats_rog_map_adapter
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

  snapshot.ready = true;
  snapshot.occupancy = grid.data;
  snapshot.signed_distance_m.resize(expected_cells);
  snapshot.gradient_x.resize(expected_cells);
  snapshot.gradient_y.resize(expected_cells);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const double resolution = grid.info.resolution;
  const double origin_yaw = std::atan2(
    2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
      grid.info.origin.orientation.x * grid.info.origin.orientation.y),
    1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
      grid.info.origin.orientation.z * grid.info.origin.orientation.z));
  const auto finiteKnownDistance = [&grid, &signed_distance, expected_cells,
      occupied_value_threshold](int x, int y, double & value) {
      if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
        y >= static_cast<int>(grid.info.height))
      {
        return false;
      }
      const auto index = static_cast<std::size_t>(y) * grid.info.width +
        static_cast<std::size_t>(x);
      if (index >= expected_cells || grid.data[index] < 0 ||
        !std::isfinite(signed_distance[index]))
      {
        return false;
      }
      value = signed_distance[index];
      // Retain the raw signed-distance contract even if an upstream producer
      // attempted to hand us an inconsistent sign.
      return (grid.data[index] >= occupied_value_threshold && value <= 0.0) ||
        (grid.data[index] < occupied_value_threshold && value >= 0.0);
    };
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x < grid.info.width; ++x) {
      const auto index = static_cast<std::size_t>(y) * grid.info.width + x;
      double center = 0.0;
      if (!finiteKnownDistance(static_cast<int>(x), static_cast<int>(y), center)) {
        snapshot.occupancy[index] = -1;
        snapshot.signed_distance_m[index] = nan;
        snapshot.gradient_x[index] = nan;
        snapshot.gradient_y[index] = nan;
        continue;
      }
      snapshot.signed_distance_m[index] = static_cast<float>(center);
      double left = 0.0;
      double right = 0.0;
      double down = 0.0;
      double up = 0.0;
      const bool has_left = finiteKnownDistance(static_cast<int>(x) - 1, static_cast<int>(y), left);
      const bool has_right = finiteKnownDistance(static_cast<int>(x) + 1, static_cast<int>(y), right);
      const bool has_down = finiteKnownDistance(static_cast<int>(x), static_cast<int>(y) - 1, down);
      const bool has_up = finiteKnownDistance(static_cast<int>(x), static_cast<int>(y) + 1, up);
      const double gradient_local_x = has_left && has_right ? (right - left) / (2.0 * resolution) :
        (has_right ? (right - center) / resolution :
        (has_left ? (center - left) / resolution : 0.0));
      const double gradient_local_y = has_down && has_up ? (up - down) / (2.0 * resolution) :
        (has_up ? (up - center) / resolution :
        (has_down ? (center - down) / resolution : 0.0));
      snapshot.gradient_x[index] = static_cast<float>(
        std::cos(origin_yaw) * gradient_local_x - std::sin(origin_yaw) * gradient_local_y);
      snapshot.gradient_y[index] = static_cast<float>(
        std::sin(origin_yaw) * gradient_local_x + std::cos(origin_yaw) * gradient_local_y);
    }
  }
  return snapshot;
}

}  // namespace ats_rog_map_adapter

#endif  // ATS_ROG_MAP_ADAPTER__PLANNING_MAP_SNAPSHOT_HPP_
