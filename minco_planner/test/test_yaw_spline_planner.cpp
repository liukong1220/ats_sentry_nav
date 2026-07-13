// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>

#include "minco_planner/trajectory/yaw_spline_planner.hpp"

TEST(YawSplinePlanner, GoalHeadingIsIndependentFromTranslationTangent)
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i <= 20; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = 0.1 * i;
    point.y = 0.0;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "goal_heading";
  params.yaw_rate_limit = 2.5;
  minco_planner::YawSplinePlanner planner(params);
  planner.apply(trajectory, 0.0, M_PI_2);

  EXPECT_NEAR(trajectory.points.front().yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().yaw, M_PI_2, 1e-8);
  EXPECT_NEAR(trajectory.points.front().yaw_rate, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().yaw_rate, 0.0, 1e-8);
  EXPECT_GT(trajectory.points[10].yaw, 0.1);
}

TEST(YawSplinePlanner, HoldModeDoesNotFollowPath)
{
  minco_planner::ReferenceTrajectory trajectory;
  trajectory.points.resize(3);
  trajectory.points[1].x = 0.0;
  trajectory.points[1].y = 1.0;
  minco_planner::YawSplinePlannerParams params;
  params.mode = "hold";
  minco_planner::YawSplinePlanner planner(params);
  planner.apply(trajectory, -0.4, 1.0);
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.yaw, -0.4, 1e-9);
  }
}

TEST(YawSplinePlanner, ClearanceAwareKeepsIndependentYawInOpenArea)
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i <= 10; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = 0.1 * i;
    point.clearance = 2.0;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, M_PI_2);

  EXPECT_NEAR(trajectory.points.front().yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().yaw, M_PI_2, 1e-8);
}

TEST(YawSplinePlanner, ClearanceAwareUsesEnterExitHysteresis)
{
  minco_planner::ReferenceTrajectory trajectory;
  const std::vector<double> clearances{1.0, 0.4, 0.6, 0.8};
  for (std::size_t i = 0; i < clearances.size(); ++i) {
    minco_planner::ReferencePoint point;
    point.t = static_cast<double>(i);
    point.x = static_cast<double>(i);
    point.clearance = clearances[i];
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.narrow_clearance_enter = 0.5;
  params.narrow_clearance_exit = 0.7;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, M_PI_2, M_PI_2);

  EXPECT_NEAR(trajectory.points[1].yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points[2].yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points[3].yaw, M_PI_2, 1e-9);
}

TEST(YawSplinePlanner, ClearanceAwareSelectsReverseTangentWhenCloser)
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 3; ++i) {
    minco_planner::ReferencePoint point;
    point.t = static_cast<double>(i);
    point.x = static_cast<double>(i);
    point.clearance = 0.2;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, M_PI - 0.1, M_PI - 0.1);

  EXPECT_NEAR(std::abs(trajectory.points[1].yaw), M_PI, 1e-9);
  EXPECT_NEAR(std::abs(trajectory.points[2].yaw), M_PI, 1e-9);
}
