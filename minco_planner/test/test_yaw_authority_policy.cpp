// Copyright 2026

#include <limits>

#include <gtest/gtest.h>

#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "minco_planner/trajectory/yaw_authority_policy.hpp"

namespace minco_planner
{

TEST(YawAuthorityPolicy, OpenReferenceUsesGimbalCompensation)
{
  ReferenceTrajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points[0].clearance = 0.90;
  trajectory.points[1].clearance = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    selectYawAuthority(trajectory, false, 0.55),
    ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED);
}

TEST(YawAuthorityPolicy, NarrowReferenceRequiresBodyYawAndGimbalLock)
{
  ReferenceTrajectory trajectory;
  trajectory.points.resize(3);
  trajectory.points[0].clearance = 0.80;
  trajectory.points[1].clearance = 0.55;
  trajectory.points[2].clearance = 0.70;

  EXPECT_EQ(
    selectYawAuthority(trajectory, false, 0.55),
    ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW);
}

TEST(YawAuthorityPolicy, RouteProfileCanForceConservativeBodyYaw)
{
  ReferenceTrajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points[0].clearance = 2.0;
  trajectory.points[1].clearance = 2.0;

  EXPECT_EQ(
    selectYawAuthority(trajectory, true, 0.55),
    ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW);
}

}  // namespace minco_planner
