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

TEST(YawSplinePlanner, ClearanceAwareRestoresGoalHeadingAfterNarrowTerminalSegment)
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 3; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.4 * static_cast<double>(i);
    point.s = 0.5 * static_cast<double>(i);
    point.x = 0.5 * static_cast<double>(i);
    point.clearance = 0.2;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 2.5;
  params.terminal_yaw_sample_period = 0.1;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, M_PI_2);

  ASSERT_GT(trajectory.points.size(), 3U);
  EXPECT_NEAR(trajectory.points[2].yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().yaw, M_PI_2, 1e-8);
  EXPECT_NEAR(trajectory.points.back().yaw_rate, 0.0, 1e-8);
  for (std::size_t i = 3; i < trajectory.points.size(); ++i) {
    EXPECT_NEAR(trajectory.points[i].x, 1.0, 1e-9);
    EXPECT_NEAR(trajectory.points[i].y, 0.0, 1e-9);
    EXPECT_NEAR(trajectory.points[i].v, 0.0, 1e-9);
    EXPECT_LE(std::abs(trajectory.points[i].yaw_rate), params.yaw_rate_limit + 1e-9);
  }
  EXPECT_TRUE(trajectory.valid());
}

TEST(YawSplinePlanner, UnannotatedClearanceIsNotTreatedAsAZeroClearanceCorridor)
{
  // 回归:ReferencePoint::clearance 曾默认 0.0。0.0 是有限值,会让第一个点就满足
  // `clearance <= narrow_clearance_enter` 并因迟滞一直锁在窄通道分支,于是净空阈值
  // 变成死参数,而调用方是否标注过净空完全看不出来。未知净空必须是 NaN。
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i <= 10; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = 0.1 * i;
    trajectory.points.push_back(point);
    EXPECT_FALSE(std::isfinite(point.clearance));
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, M_PI_2);

  // 没有净空信息时保持独立 yaw,不能被当成贴墙而强制对齐切线。
  EXPECT_NEAR(trajectory.points.back().yaw, M_PI_2, 1e-8);
  EXPECT_GT(trajectory.points[5].yaw, 0.0);
}

TEST(YawSplinePlanner, NarrowCorridorAlignsYawWithTheCorridorTangent)
{
  // 走廊沿 -x 方向,位置净空 0.30 m 低于 0.4187 m 全 yaw 半径:必须对齐走廊轴线,
  // 否则 0.60 x 0.50 m 足迹会扫进两侧墙。这一段锁住 red_box 目标 4 的通行前提。
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 6; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.2 * static_cast<double>(i);
    point.s = 0.2 * static_cast<double>(i);
    point.x = -0.2 * static_cast<double>(i);
    point.y = 0.0;
    point.clearance = 0.30;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.narrow_clearance_enter = 0.55;
  params.narrow_clearance_exit = 0.70;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, M_PI, M_PI_2);

  for (std::size_t i = 1; i < 6U; ++i) {
    EXPECT_NEAR(std::abs(trajectory.points[i].yaw), M_PI, 1e-9)
      << "narrow index " << i << " left the corridor axis";
  }
}
