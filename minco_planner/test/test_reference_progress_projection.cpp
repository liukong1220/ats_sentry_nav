// Copyright 2026

#include <gtest/gtest.h>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace
{
// 东行 1 m/s，每 0.1 s 一个点，共 50 点（0..4.9 m）。
minco_planner::ReferenceTrajectory makeEastbound()
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 50; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = 0.1 * i;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(ReferenceProgressProjection, LaggingRobotProjectsBehindWallClock)
{
  const auto trajectory = makeEastbound();
  // 墙钟已到 3.0 s，车只走到 1.25 m（横向偏 0.1 m）。
  EXPECT_NEAR(minco_planner::projectedReferenceTime(trajectory, 1.25, 0.1, 3.0), 1.25, 1e-9);
}

TEST(ReferenceProgressProjection, ProjectionNeverExceedsWallClock)
{
  const auto trajectory = makeEastbound();
  // 车跑在墙钟前面时不越过墙钟时间，保持原窗口起点。
  EXPECT_NEAR(minco_planner::projectedReferenceTime(trajectory, 4.0, 0.0, 2.0), 2.0, 1e-9);
}

TEST(ReferenceProgressProjection, LoopingPathDoesNotJumpToLaterPass)
{
  // 去程东行 0..2 m，回程西行 2..0 m；车在去程 0.5 m 处、墙钟 1.0 s，
  // 回程同一位置（t≈3.5 s）不得被选中。
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i <= 40; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = i <= 20 ? 0.1 * i : 0.1 * (40 - i);
    trajectory.points.push_back(point);
  }
  EXPECT_NEAR(minco_planner::projectedReferenceTime(trajectory, 0.5, 0.0, 1.0), 0.5, 1e-9);
}

TEST(ReferenceProgressProjection, LaggingWindowStillCoversRobotSegment)
{
  const auto trajectory = makeEastbound();
  const double start = minco_planner::projectedReferenceTime(trajectory, 1.25, 0.0, 3.0);
  const auto window = minco_planner::remainingReferenceWindow(trajectory, start, 3.0 + 1.0);
  ASSERT_GE(window.points.size(), 2U);
  EXPECT_LE(window.points.front().x, 1.25);
  EXPECT_GE(window.points.back().t, 4.0 - 1e-9);
}
