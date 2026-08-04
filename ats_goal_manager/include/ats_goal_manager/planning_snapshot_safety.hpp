// Copyright 2026

#ifndef ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_
#define ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace ats_goal_manager
{

struct PlanningSnapshotSafetyParams
{
  double footprint_length{0.70};
  double footprint_width{0.55};
  double footprint_safety_margin{0.05};
};

struct PlanningSnapshotSafetyResult
{
  bool valid_snapshot{false};
  bool inside_map{false};
  bool robot_cell_free{false};
  bool footprint_safe{false};
};

inline bool validPlanningSnapshot(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot)
{
  const auto count = static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height;
  if (!snapshot.ready || snapshot.header.frame_id.empty() ||
    snapshot.source_generation == 0U || snapshot.publication_sequence == 0U ||
    snapshot.occupied_value_threshold <= 0 || snapshot.occupied_value_threshold > 100 ||
    !std::isfinite(snapshot.info.resolution) || snapshot.info.resolution <= 0.0F ||
    count == 0U || snapshot.occupancy.size() != count ||
    snapshot.signed_distance_m.size() != count ||
    snapshot.gradient_x.size() != count || snapshot.gradient_y.size() != count)
  {
    return false;
  }
  const auto & orientation = snapshot.info.origin.orientation;
  const double norm = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const auto occupancy = snapshot.occupancy[index];
    const auto distance = snapshot.signed_distance_m[index];
    const auto gradient_x = snapshot.gradient_x[index];
    const auto gradient_y = snapshot.gradient_y[index];
    if (occupancy < -1 || occupancy > 100) {
      return false;
    }
    if (occupancy == -1) {
      if (std::isfinite(distance) || std::isfinite(gradient_x) ||
        std::isfinite(gradient_y))
      {
        return false;
      }
      continue;
    }
    if (!std::isfinite(distance) || !std::isfinite(gradient_x) ||
      !std::isfinite(gradient_y) ||
      (occupancy >= snapshot.occupied_value_threshold && distance > 0.0F) ||
      (occupancy < snapshot.occupied_value_threshold && distance < 0.0F))
    {
      return false;
    }
  }
  return true;
}

inline bool worldToSnapshotCell(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  double world_x, double world_y, int & cell_x, int & cell_y)
{
  const auto & origin = snapshot.info.origin;
  const double yaw = std::atan2(
    2.0 * (origin.orientation.w * origin.orientation.z +
      origin.orientation.x * origin.orientation.y),
    1.0 - 2.0 * (origin.orientation.y * origin.orientation.y +
      origin.orientation.z * origin.orientation.z));
  const double dx = world_x - origin.position.x;
  const double dy = world_y - origin.position.y;
  const double grid_x =
    (std::cos(yaw) * dx + std::sin(yaw) * dy) / snapshot.info.resolution;
  const double grid_y =
    (-std::sin(yaw) * dx + std::cos(yaw) * dy) / snapshot.info.resolution;
  if (grid_x < 0.0 || grid_y < 0.0 ||
    grid_x >= static_cast<double>(snapshot.info.width) ||
    grid_y >= static_cast<double>(snapshot.info.height))
  {
    return false;
  }
  cell_x = static_cast<int>(std::floor(grid_x));
  cell_y = static_cast<int>(std::floor(grid_y));
  return true;
}

inline bool snapshotCellFree(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  int cell_x, int cell_y)
{
  if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(snapshot.info.width) ||
    cell_y >= static_cast<int>(snapshot.info.height))
  {
    return false;
  }
  const auto index = static_cast<std::size_t>(cell_y) * snapshot.info.width +
    static_cast<std::size_t>(cell_x);
  const int value = snapshot.occupancy[index];
  // P2 always treats unknown as an obstacle at the task-level restart gate.
  return value >= 0 && value < snapshot.occupied_value_threshold;
}

inline PlanningSnapshotSafetyResult checkPlanningSnapshotFootprint(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  const geometry_msgs::msg::Pose & pose,
  const PlanningSnapshotSafetyParams & params = PlanningSnapshotSafetyParams())
{
  PlanningSnapshotSafetyResult result;
  result.valid_snapshot = validPlanningSnapshot(snapshot);
  if (!result.valid_snapshot || !std::isfinite(pose.position.x) ||
    !std::isfinite(pose.position.y))
  {
    return result;
  }
  int center_x = 0;
  int center_y = 0;
  result.inside_map = worldToSnapshotCell(
    snapshot, pose.position.x, pose.position.y, center_x, center_y);
  result.robot_cell_free = result.inside_map && snapshotCellFree(snapshot, center_x, center_y);
  if (!result.robot_cell_free) {
    return result;
  }

  const double half_length = 0.5 * std::max(0.0, params.footprint_length) +
    std::max(0.0, params.footprint_safety_margin);
  const double half_width = 0.5 * std::max(0.0, params.footprint_width) +
    std::max(0.0, params.footprint_safety_margin);
  const double spacing = std::max(0.02, static_cast<double>(snapshot.info.resolution));
  const int samples_x = std::max(2, static_cast<int>(std::ceil(2.0 * half_length / spacing)));
  const int samples_y = std::max(2, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
  const double yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int ix = 0; ix <= samples_x; ++ix) {
    const double local_x = -half_length + 2.0 * half_length * ix / samples_x;
    for (int iy = 0; iy <= samples_y; ++iy) {
      const double local_y = -half_width + 2.0 * half_width * iy / samples_y;
      int cell_x = 0;
      int cell_y = 0;
      const double world_x = pose.position.x + cosine * local_x - sine * local_y;
      const double world_y = pose.position.y + sine * local_x + cosine * local_y;
      if (!worldToSnapshotCell(snapshot, world_x, world_y, cell_x, cell_y) ||
        !snapshotCellFree(snapshot, cell_x, cell_y))
      {
        return result;
      }
    }
  }
  result.footprint_safe = true;
  return result;
}

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_
