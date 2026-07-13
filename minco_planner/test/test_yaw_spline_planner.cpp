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
