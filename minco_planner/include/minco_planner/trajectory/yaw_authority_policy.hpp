// Copyright 2026

#ifndef MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_
#define MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_

#include <cmath>
#include <cstdint>

#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

inline std::uint8_t selectYawAuthority(
  const ReferenceTrajectory & trajectory,
  bool force_body_yaw_follow,
  double body_yaw_follow_clearance)
{
  using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
  if (force_body_yaw_follow) {
    return PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
  }
  for (const auto & point : trajectory.points) {
    if (std::isfinite(point.clearance) &&
      point.clearance <= body_yaw_follow_clearance)
    {
      return PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
    }
  }
  return PlannerStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_
