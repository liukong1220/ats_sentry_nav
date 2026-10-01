// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
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

namespace
{
// 沿 -y 方向的窄通道（切线 yaw = -pi/2），与南侧墙尖处 JPS 的竖直段相同。
minco_planner::ReferenceTrajectory makeSouthboundNarrowTrajectory()
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 4; ++i) {
    minco_planner::ReferencePoint point;
    point.t = static_cast<double>(i);
    point.y = -static_cast<double>(i);
    point.clearance = 0.2;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(YawSplinePlanner, SquareSymmetryKeepsAlreadyAlignedYawInNarrowSection)
{
  auto trajectory = makeSouthboundNarrowTrajectory();
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.tangent_symmetry_order = 4;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, 0.0);

  // yaw 0 与切线 -pi/2 相差 pi/2，正方形足迹已对齐，不应再转向。
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.yaw, 0.0, 1e-9);
    EXPECT_NEAR(point.yaw_rate, 0.0, 1e-9);
  }
}

TEST(YawSplinePlanner, SquareSymmetryCapsNarrowAlignmentTurnAtQuarterPi)
{
  auto trajectory = makeSouthboundNarrowTrajectory();
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.tangent_symmetry_order = 4;
  minco_planner::YawSplinePlanner planner(params);

  // 起始 yaw 0.6 离 0（= -pi/2 + pi/2）最近，只需转 0.6 rad 而不是转到 -pi/2。
  planner.apply(trajectory, 0.6, 0.0);

  EXPECT_NEAR(trajectory.points[1].yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points[2].yaw, 0.0, 1e-9);
}

TEST(YawSplinePlanner, RectangleSymmetryStillTurnsToTangent)
{
  auto trajectory = makeSouthboundNarrowTrajectory();
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, 0.0);

  // 默认 2 阶：yaw 0 到 ±pi/2 等距，按正向优先转到 -pi/2。
  EXPECT_NEAR(trajectory.points[1].yaw, -M_PI_2, 1e-9);
  EXPECT_NEAR(trajectory.points[2].yaw, -M_PI_2, 1e-9);
}

TEST(YawSplinePlanner, UnsupportedSymmetryOrderFallsBackToRectangle)
{
  auto trajectory = makeSouthboundNarrowTrajectory();
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.tangent_symmetry_order = 3;
  minco_planner::YawSplinePlanner planner(params);

  planner.apply(trajectory, 0.0, 0.0);

  EXPECT_NEAR(trajectory.points[1].yaw, -M_PI_2, 1e-9);
}

namespace
{
// 东行 8 点：1-2 窄、3-4 开阔（间隙 t=2..5 共 3 s）、5-6 窄、7 开阔。
minco_planner::ReferenceTrajectory makeNarrowGapNarrowTrajectory()
{
  const double clearances[] = {0.2, 0.2, 0.2, 2.0, 2.0, 0.2, 0.2, 2.0};
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    minco_planner::ReferencePoint point;
    point.t = static_cast<double>(i);
    point.x = static_cast<double>(i);
    point.clearance = clearances[i];
    trajectory.points.push_back(point);
  }
  return trajectory;
}

minco_planner::YawSplinePlannerParams makeGapBridgeParams(double bridge_time)
{
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 10.0;
  params.narrow_gap_bridge_time = bridge_time;
  return params;
}
}  // namespace

TEST(YawSplinePlanner, ShortOpenGapBetweenNarrowSectionsKeepsTangentYaw)
{
  auto trajectory = makeNarrowGapNarrowTrajectory();
  minco_planner::YawSplinePlanner planner(makeGapBridgeParams(3.0));

  // 目标朝向 1.0 与切线 0 不同；间隙 3 s 不超过桥接时长，间隙内不应回摆到目标朝向。
  planner.apply(trajectory, 0.0, 1.0);

  for (std::size_t i = 1; i <= 6; ++i) {
    EXPECT_NEAR(trajectory.points[i].yaw, 0.0, 1e-9) << "index " << i;
  }
  // 末端开阔段不桥接，仍转向目标朝向。
  EXPECT_GT(trajectory.points[7].yaw, 0.1);
}

TEST(YawSplinePlanner, LongOpenGapBetweenNarrowSectionsRestoresGoalHeading)
{
  auto trajectory = makeNarrowGapNarrowTrajectory();
  minco_planner::YawSplinePlanner planner(makeGapBridgeParams(2.9));

  planner.apply(trajectory, 0.0, 1.0);

  // 间隙 3 s 超过桥接时长，保持原有行为：间隙内回到开阔区域的目标朝向。
  EXPECT_GT(trajectory.points[3].yaw, 0.1);
  EXPECT_GT(trajectory.points[4].yaw, 0.1);
}

TEST(YawSplinePlanner, GapBridgeDisabledByDefault)
{
  auto bridged = makeNarrowGapNarrowTrajectory();
  auto baseline = makeNarrowGapNarrowTrajectory();
  minco_planner::YawSplinePlanner default_planner(makeGapBridgeParams(0.0));
  minco_planner::YawSplinePlannerParams defaults;
  defaults.mode = "clearance_aware";
  defaults.yaw_rate_limit = 10.0;
  minco_planner::YawSplinePlanner reference_planner(defaults);

  default_planner.apply(bridged, 0.0, 1.0);
  reference_planner.apply(baseline, 0.0, 1.0);

  ASSERT_EQ(bridged.points.size(), baseline.points.size());
  for (std::size_t i = 0; i < bridged.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(bridged.points[i].yaw, baseline.points[i].yaw);
  }
  EXPECT_GT(bridged.points[3].yaw, 0.1);
}

namespace
{
// 窄通道内 L 形路径：先东行 20 点再北行 40 点，每 0.05 s、0.05 m 一个点。
minco_planner::ReferenceTrajectory makeNarrowCornerTrajectory()
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 60; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.05 * i;
    point.x = 0.05 * std::min(i, 20);
    point.y = 0.05 * std::max(0, i - 20);
    point.clearance = 0.2;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

minco_planner::YawSplinePlannerParams makeYawAccelerationParams(double acceleration_limit)
{
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.yaw_rate_limit = 2.0;
  params.yaw_acceleration_limit = acceleration_limit;
  return params;
}
}  // namespace

TEST(YawSplinePlanner, YawAccelerationLimitBoundsRateChangeAtNarrowCorner)
{
  auto trajectory = makeNarrowCornerTrajectory();
  minco_planner::YawSplinePlanner planner(makeYawAccelerationParams(3.0));

  // 目标朝向与终点切线一致，避免末端追加原地转向。
  planner.apply(trajectory, 0.0, M_PI_2);

  for (std::size_t i = 1; i < 60; ++i) {
    const auto & previous = trajectory.points[i - 1];
    const auto & current = trajectory.points[i];
    const double dt = current.t - previous.t;
    EXPECT_LE(std::abs(current.yaw_rate), 2.0 + 1e-9) << "index " << i;
    EXPECT_LE(
      std::abs(current.yaw_rate - previous.yaw_rate), 3.0 * dt + 1e-9) << "index " << i;
  }
  // 可刹停跟踪：最终收敛到北向切线，且不越过它。
  double max_yaw = -M_PI;
  for (std::size_t i = 0; i < 60; ++i) {
    max_yaw = std::max(max_yaw, trajectory.points[i].yaw);
  }
  EXPECT_NEAR(trajectory.points[59].yaw, M_PI_2, 1e-3);
  EXPECT_LE(max_yaw, M_PI_2 + 1e-6);
}

TEST(YawSplinePlanner, YawAccelerationLimitDisabledByDefault)
{
  auto trajectory = makeNarrowCornerTrajectory();
  minco_planner::YawSplinePlanner planner(makeYawAccelerationParams(0.0));

  planner.apply(trajectory, 0.0, M_PI_2);

  // 关闭时保持原行为：拐角处角速度直接跳到 yaw_rate_limit。
  double max_rate_change = 0.0;
  for (std::size_t i = 1; i < 60; ++i) {
    max_rate_change = std::max(
      max_rate_change,
      std::abs(trajectory.points[i].yaw_rate - trajectory.points[i - 1].yaw_rate));
  }
  EXPECT_NEAR(max_rate_change, 2.0, 1e-9);
}

TEST(YawSplinePlanner, YawAccelerationLimitStretchesSmallTerminalTurn)
{
  // 开阔直线、目标朝向 0.3 rad：只按角速度定时长时终端原地转向仅约 0.28 s。
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 3; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.05 * i;
    point.x = 0.05 * i;
    point.clearance = 0.2;
    trajectory.points.push_back(point);
  }
  minco_planner::YawSplinePlanner planner(makeYawAccelerationParams(3.0));

  planner.apply(trajectory, 0.0, 0.3);

  EXPECT_NEAR(trajectory.points.back().yaw, 0.3, 1e-9);
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double dt = trajectory.points[i].t - trajectory.points[i - 1].t;
    ASSERT_GT(dt, 0.0);
    // 终端五次曲线按采样点差分，允许离散误差。
    EXPECT_LE(
      std::abs(trajectory.points[i].yaw_rate - trajectory.points[i - 1].yaw_rate) / dt,
      3.0 * 1.05) << "index " << i;
  }
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
