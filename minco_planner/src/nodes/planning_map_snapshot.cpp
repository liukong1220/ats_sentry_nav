// Copyright 2026

#include "minco_planner/nodes/planning_map_snapshot.hpp"

#include <cmath>
#include <utility>

#include "minco_planner/nodes/atomic_planning_contract.hpp"
#include "minco_planner/nodes/planning_digest.hpp"
#include "minco_planner/planning/grid_clearance.hpp"

namespace minco_planner
{

bool steadyHeartbeatLeaseValid(
  const std::optional<std::chrono::steady_clock::time_point> & last_signal,
  std::chrono::steady_clock::time_point current_time,
  double timeout_sec)
{
  if (!last_signal || current_time < *last_signal) {
    return false;
  }
  return std::chrono::duration<double>(current_time - *last_signal).count() <= timeout_sec;
}

std::shared_ptr<const PlanningMapSnapshot> PlanningMapSnapshot::create(
  std::uint64_t generation,
  const nav_msgs::msg::OccupancyGrid & grid,
  int obstacle_value_threshold,
  bool unknown_is_obstacle)
{
  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.header.frame_id.empty() || !std::isfinite(grid.info.resolution) ||
    grid.info.resolution <= 0.0 || cell_count == 0 ||
    grid.data.size() != cell_count)
  {
    return nullptr;
  }
  auto snapshot = std::make_shared<PlanningMapSnapshot>();
  snapshot->generation = generation;
  snapshot->grid = grid;
  snapshot->safety_content_digest = planning_digest::gridSafetyDigest(
    grid, obstacle_value_threshold, unknown_is_obstacle);
  auto esdf = std::make_shared<ats_rc_esdf::RcTraversabilityEsdfProvider>();
  esdf->updateGrid(grid, obstacle_value_threshold, unknown_is_obstacle);
  snapshot->clearance_esdf = std::move(esdf);
  GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = obstacle_value_threshold;
  policy.unknown_is_obstacle = unknown_is_obstacle;
  snapshot->blocked_squared_cells = std::make_shared<const std::vector<double>>(
    computeBlockedSquaredDistanceCells(grid, policy));
  return snapshot;
}

std::shared_ptr<const PlanningMapSnapshot> PlanningMapSnapshot::fromMessage(
  std::uint64_t generation,
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & message,
  int obstacle_value_threshold,
  bool unknown_is_obstacle)
{
  if (!message.ready || !validPlanningMapSnapshot(message) ||
    message.occupied_value_threshold != obstacle_value_threshold ||
    message.unknown_is_obstacle != unknown_is_obstacle)
  {
    return nullptr;
  }
  auto snapshot = std::make_shared<PlanningMapSnapshot>();
  snapshot->generation = generation;
  snapshot->grid.header.frame_id = message.header.frame_id;
  snapshot->grid.header.stamp = message.source_stamp;
  snapshot->grid.info = message.info;
  snapshot->grid.data = message.occupancy;
  snapshot->safety_content_digest = planning_digest::gridSafetyDigest(
    snapshot->grid, obstacle_value_threshold, unknown_is_obstacle);

  const std::size_t cell_count = message.occupancy.size();
  const double resolution = message.info.resolution;
  std::vector<double> signed_distance(cell_count);
  auto blocked_squared = std::make_shared<std::vector<double>>(cell_count, 0.0);
  for (std::size_t index = 0; index < cell_count; ++index) {
    const double distance = message.signed_distance_m[index];
    signed_distance[index] = distance;
    if (distance > 0.0) {
      // Free distances are exact cell-centre distances, so the squared cell count is an
      // integer; rounding removes the float32 transport error.
      const double cells = distance / resolution;
      (*blocked_squared)[index] = std::round(cells * cells);
    }
  }
  auto esdf = std::make_shared<ats_rc_esdf::RcTraversabilityEsdfProvider>();
  if (!esdf->loadSignedDistanceField(snapshot->grid, signed_distance)) {
    return nullptr;
  }
  snapshot->clearance_esdf = std::move(esdf);
  snapshot->blocked_squared_cells = std::move(blocked_squared);
  return snapshot;
}

}  // namespace minco_planner
