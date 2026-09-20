// Copyright 2026

#ifndef MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_
#define MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"

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

  // A fresh grid is still a healthy map source, but every reference committed
  // against the previous immutable snapshot is no longer executable.  Keep
  // the heartbeat lease alive while requiring a new plan for this generation.
  void invalidatePlanForNewMap(std::uint64_t generation)
  {
    plan_safe = false;
    if (generation > minimum_map_generation) {
      minimum_map_generation = generation;
    }
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
  // A local safety identity, not a ROGMap source-generation or adapter
  // publication sequence.  It binds the exact inputs from which RC-ESDF and
  // swept-footprint checks are derived.
  std::string safety_content_digest;
  nav_msgs::msg::OccupancyGrid grid;
  std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> clearance_esdf;

  bool hasSameSafetyContent(const PlanningMapSnapshot & other) const
  {
    return !safety_content_digest.empty() &&
           safety_content_digest == other.safety_content_digest;
  }

  static std::shared_ptr<const PlanningMapSnapshot> create(
    std::uint64_t generation,
    const nav_msgs::msg::OccupancyGrid & grid,
    int obstacle_value_threshold,
    bool unknown_is_obstacle);
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_
