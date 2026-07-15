// Copyright 2026

#ifndef MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_
#define MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"

namespace minco_planner
{

bool steadyHeartbeatLeaseValid(
  const std::optional<std::chrono::steady_clock::time_point> & last_signal,
  std::chrono::steady_clock::time_point current_time,
  double timeout_sec);

struct PlannerSafetyState
{
  bool map_ready{false};
  bool plan_safe{false};
  std::uint64_t minimum_map_generation{0};

  void invalidateMap(std::uint64_t current_generation)
  {
    map_ready = false;
    plan_safe = false;
    minimum_map_generation = current_generation + 1;
  }

  bool mapSnapshotUsable(std::uint64_t generation) const
  {
    return map_ready && generation >= minimum_map_generation;
  }

  bool emergencyStopRequired() const
  {
    return !map_ready || !plan_safe;
  }
};

struct PlanningMapSnapshot
{
  std::uint64_t generation{0};
  nav_msgs::msg::OccupancyGrid grid;
  std::shared_ptr<const trajectory_optimizer::RcTraversabilityEsdfProvider> clearance_esdf;

  static std::shared_ptr<const PlanningMapSnapshot> create(
    std::uint64_t generation,
    const nav_msgs::msg::OccupancyGrid & grid,
    int obstacle_value_threshold,
    bool unknown_is_obstacle);
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_
