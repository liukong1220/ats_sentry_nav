// Copyright 2026

#ifndef MINCO_PLANNER__NODES__ATOMIC_PLANNING_CONTRACT_HPP_
#define MINCO_PLANNER__NODES__ATOMIC_PLANNING_CONTRACT_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ats_navigation_interfaces/msg/planner_candidate.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"

namespace minco_planner {

inline bool finite(double value) { return std::isfinite(value); }

inline bool unitQuaternion(const geometry_msgs::msg::Quaternion &orientation) {
  if (!finite(orientation.x) || !finite(orientation.y) ||
      !finite(orientation.z) || !finite(orientation.w)) {
    return false;
  }
  const double squared_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  return std::abs(squared_norm - 1.0) <= 1e-3;
}

inline bool nonZeroDigest(const std::array<std::uint8_t, 32> &digest) {
  for (const auto byte : digest) {
    if (byte != 0U) {
      return true;
    }
  }
  return false;
}

inline bool validPlanningMapSnapshot(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot) {
  const auto expected_cells =
      static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height;
  if (!snapshot.ready) {
    return snapshot.occupancy.empty() && snapshot.signed_distance_m.empty() &&
           snapshot.gradient_x.empty() && snapshot.gradient_y.empty();
  }
  if (snapshot.header.frame_id.empty() || snapshot.source_generation == 0U ||
      snapshot.publication_sequence == 0U ||
      snapshot.occupied_value_threshold <= 0 ||
      snapshot.occupied_value_threshold > 100 ||
      !finite(snapshot.info.resolution) || snapshot.info.resolution <= 0.0F ||
      !unitQuaternion(snapshot.info.origin.orientation) ||
      expected_cells == 0U || snapshot.occupancy.size() != expected_cells ||
      snapshot.signed_distance_m.size() != expected_cells ||
      snapshot.gradient_x.size() != expected_cells ||
      snapshot.gradient_y.size() != expected_cells) {
    return false;
  }

  for (std::size_t index = 0; index < expected_cells; ++index) {
    const auto occupancy = snapshot.occupancy[index];
    const auto distance = snapshot.signed_distance_m[index];
    const auto gradient_x = snapshot.gradient_x[index];
    const auto gradient_y = snapshot.gradient_y[index];
    if (occupancy < -1 || occupancy > 100) {
      return false;
    }
    if (occupancy == -1) {
      if (finite(distance) || finite(gradient_x) || finite(gradient_y)) {
        return false;
      }
      continue;
    }
    if (!finite(distance) || !finite(gradient_x) || !finite(gradient_y)) {
      return false;
    }
    if ((occupancy >= snapshot.occupied_value_threshold && distance > 0.0F) ||
        (occupancy < snapshot.occupied_value_threshold && distance < 0.0F)) {
      return false;
    }
  }
  return true;
}

inline bool monotonicPathStamps(const nav_msgs::msg::Path &path) {
  std::int64_t previous_nanoseconds = -1;
  for (const auto &pose : path.poses) {
    if (pose.header.frame_id != path.header.frame_id ||
        !finite(pose.pose.position.x) || !finite(pose.pose.position.y) ||
        !finite(pose.pose.position.z) ||
        !unitQuaternion(pose.pose.orientation)) {
      return false;
    }
    const auto stamp =
        static_cast<std::int64_t>(pose.header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(pose.header.stamp.nanosec);
    if (stamp < previous_nanoseconds) {
      return false;
    }
    previous_nanoseconds = stamp;
  }
  return true;
}

inline bool validPlannerCandidate(
    const ats_navigation_interfaces::msg::PlannerCandidate &candidate) {
  if (candidate.state ==
      ats_navigation_interfaces::msg::PlannerCandidate::STATE_REJECTED) {
    return candidate.failure_reason !=
           ats_navigation_interfaces::msg::PlannerCandidate::FAILURE_NONE;
  }
  if (candidate.state !=
          ats_navigation_interfaces::msg::PlannerCandidate::STATE_READY ||
      candidate.header.frame_id.empty() ||
      candidate.planner_incarnation == 0U ||
      candidate.candidate_sequence == 0U || candidate.goal_id == 0U ||
      candidate.map_snapshot_generation == 0U ||
      candidate.map_source_generation == 0U ||
      candidate.map_publication_sequence == 0U ||
      candidate.failure_reason !=
          ats_navigation_interfaces::msg::PlannerCandidate::FAILURE_NONE ||
      !candidate.footprint_safe || !candidate.dynamics_feasible ||
      candidate.footprint_collision_samples != 0U ||
      !finite(candidate.minimum_clearance_m) ||
      candidate.minimum_clearance_m < 0.0F ||
      candidate.lease_duration.sec < 0 ||
      (candidate.lease_duration.sec == 0 &&
       candidate.lease_duration.nanosec == 0U) ||
      !nonZeroDigest(candidate.content_digest) ||
      candidate.raw_path.poses.empty() || candidate.reference.poses.empty() ||
      candidate.raw_path.header.frame_id != candidate.header.frame_id ||
      candidate.reference.header.frame_id != candidate.header.frame_id) {
    return false;
  }
  return monotonicPathStamps(candidate.raw_path) &&
         monotonicPathStamps(candidate.reference);
}

} // namespace minco_planner

#endif // MINCO_PLANNER__NODES__ATOMIC_PLANNING_CONTRACT_HPP_
