// Copyright 2026

#include "minco_planner/nodes/planning_map_snapshot.hpp"

#include <cmath>

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
  auto esdf = std::make_shared<ats_rc_esdf::RcTraversabilityEsdfProvider>();
  esdf->updateGrid(grid, obstacle_value_threshold, unknown_is_obstacle);
  snapshot->clearance_esdf = std::move(esdf);
  return snapshot;
}

}  // namespace minco_planner
