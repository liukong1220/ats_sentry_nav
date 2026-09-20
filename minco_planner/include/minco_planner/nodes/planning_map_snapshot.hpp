// Copyright 2026

#ifndef MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_
#define MINCO_PLANNER__NODES__PLANNING_MAP_SNAPSHOT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "ats_navigation_interfaces/msg/planner_status.hpp"
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

// Accessed under the node's map mutex, including publication.  Unlike the
// active trajectory, this identity survives replanning and map-health loss.
struct PlannerStatusState
{
  using Status = ats_navigation_interfaces::msg::PlannerStatus;
  std::optional<Status> latest;

  bool update(Status & status)
  {
    if (latest) {
      if (status.localization_epoch < latest->localization_epoch) {
        return false;
      }
      if (status.localization_epoch == latest->localization_epoch) {
        if (status.map_generation < latest->map_generation ||
          status.plan_request_sequence < latest->plan_request_sequence)
        {
          return false;
        }
        if (status.map_generation == latest->map_generation &&
          latest->state == Status::STATE_FAILED &&
          latest->failure_reason == Status::FAILURE_MAP_UNREADY &&
          status.failure_reason != Status::FAILURE_MAP_UNREADY)
        {
          return false;
        }
      }
      // ROS time may stand still in simulation.  Give serialized transitions
      // distinct stamps so a queued healthy sample cannot undo invalidation.
      if (status.header.stamp.sec < latest->header.stamp.sec ||
        (status.header.stamp.sec == latest->header.stamp.sec &&
        status.header.stamp.nanosec <= latest->header.stamp.nanosec))
      {
        status.header.stamp = latest->header.stamp;
        if (++status.header.stamp.nanosec == 1000000000U) {
          status.header.stamp.nanosec = 0;
          ++status.header.stamp.sec;
        }
      }
    }
    latest = status;
    return true;
  }

  bool invalidate(
    std::uint64_t generation, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!latest) {
      return false;
    }
    auto status = *latest;
    status.header.stamp = stamp;
    status.map_generation = generation;
    status.state = Status::STATE_FAILED;
    status.failure_reason = failure_reason;
    return update(status);
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
