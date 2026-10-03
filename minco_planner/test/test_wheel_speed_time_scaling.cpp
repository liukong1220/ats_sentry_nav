// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "minco_planner/trajectory/wheel_speed_time_scaling.hpp"

namespace
{
// 东行匀速 1.5 m/s，每 0.1 s 一个点；[turn_begin, turn_end) 内叠加 2.5 rad/s 转向。
minco_planner::ReferenceTrajectory makeTurningTrajectory(
  int count, int turn_begin, int turn_end)
{
  minco_planner::ReferenceTrajectory trajectory;
  double yaw = 0.0;
  for (int i = 0; i < count; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.x = 0.15 * i;
    point.v = 1.5;
    point.vx = 1.5;
    point.ax = 0.2;
    point.yaw = yaw;
    point.yaw_rate = (i >= turn_begin && i < turn_end) ? 2.5 : 0.0;
    yaw += 0.1 * point.yaw_rate;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

minco_planner::WheelSpeedTimeScalingParams makeParams()
{
  minco_planner::WheelSpeedTimeScalingParams params;
  params.wheel_speed_limit = 1.55;
  params.wheel_offset_x = 0.27;
  params.wheel_offset_y = 0.27;
  params.scale_change_rate = 2.0;
  return params;
}
}  // namespace

TEST(WheelSpeedTimeScaling, DisabledByDefault)
{
  auto trajectory = makeTurningTrajectory(20, 5, 10);
  const auto original = trajectory;

  EXPECT_FALSE(minco_planner::applyWheelSpeedTimeScaling(
    trajectory, minco_planner::WheelSpeedTimeScalingParams()));

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].t, original.points[i].t);
    EXPECT_DOUBLE_EQ(trajectory.points[i].vx, original.points[i].vx);
  }
}

TEST(WheelSpeedTimeScaling, FeasibleTrajectoryIsUnchanged)
{
  auto trajectory = makeTurningTrajectory(20, 0, 0);
  const auto original = trajectory;

  EXPECT_FALSE(minco_planner::applyWheelSpeedTimeScaling(trajectory, makeParams()));

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].t, original.points[i].t);
  }
}

TEST(WheelSpeedTimeScaling, TurningSegmentIsSlowedBelowWheelLimit)
{
  auto trajectory = makeTurningTrajectory(40, 15, 25);
  const auto original = trajectory;
  const auto params = makeParams();
  ASSERT_GT(
    minco_planner::maxWheelSpeed(original.points[20], params.wheel_offset_x,
    params.wheel_offset_y), params.wheel_speed_limit);

  EXPECT_TRUE(minco_planner::applyWheelSpeedTimeScaling(trajectory, params));

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    EXPECT_LE(
      minco_planner::maxWheelSpeed(point, params.wheel_offset_x, params.wheel_offset_y),
      params.wheel_speed_limit + 1e-9) << "index " << i;
    // 几何与 yaw 序列不变，只改时间参数化。
    EXPECT_DOUBLE_EQ(point.x, original.points[i].x);
    EXPECT_DOUBLE_EQ(point.yaw, original.points[i].yaw);
    if (i > 0) {
      EXPECT_GT(point.t, trajectory.points[i - 1].t);
    }
  }
  EXPECT_GT(trajectory.totalTime(), original.totalTime());
  // 远离转向段的首尾保持原速。
  EXPECT_DOUBLE_EQ(trajectory.points.front().vx, 1.5);
  EXPECT_DOUBLE_EQ(trajectory.points.back().vx, 1.5);
  // 速度按系数 f、加速度按 f^2 缩放。
  const double f = trajectory.points[20].vx / 1.5;
  EXPECT_LT(f, 1.0);
  EXPECT_NEAR(trajectory.points[20].yaw_rate, 2.5 * f, 1e-9);
  EXPECT_NEAR(trajectory.points[20].ax, 0.2 * f * f, 1e-9);
}

TEST(WheelSpeedTimeScaling, ScaleFactorChangeIsRateLimited)
{
  auto trajectory = makeTurningTrajectory(40, 20, 21);
  const auto params = makeParams();

  ASSERT_TRUE(minco_planner::applyWheelSpeedTimeScaling(trajectory, params));

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double f_previous = trajectory.points[i - 1].vx / 1.5;
    const double f_current = trajectory.points[i].vx / 1.5;
    // 原时间步 0.1 s，系数每步最多变化 rate * 0.1。
    EXPECT_LE(std::abs(f_current - f_previous), params.scale_change_rate * 0.1 + 1e-9);
  }
  // 减速要在转向点之前开始。
  EXPECT_LT(trajectory.points[19].vx, 1.5);
}

TEST(WheelSpeedTimeScaling, NarrowTurnIsSlowedOnlyWhereNarrowAndTurning)
{
  // 转向段 [15, 25)；其中只有 [18, 22) 在窄通道内。
  auto trajectory = makeTurningTrajectory(40, 15, 25);
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].clearance = (i >= 18 && i < 22) ? 0.3 : 2.0;
  }
  const auto original = trajectory;
  minco_planner::WheelSpeedTimeScalingParams params;
  params.narrow_turn_speed_limit = 0.8;
  params.narrow_turn_clearance = 0.55;
  params.scale_change_rate = 2.0;

  EXPECT_TRUE(minco_planner::applyNarrowTurnTimeScaling(trajectory, params));

  for (std::size_t i = 18; i < 22; ++i) {
    EXPECT_LE(std::hypot(trajectory.points[i].vx, trajectory.points[i].vy), 0.8 + 1e-9);
    EXPECT_DOUBLE_EQ(trajectory.points[i].yaw, original.points[i].yaw);
  }
  EXPECT_DOUBLE_EQ(trajectory.points.front().vx, 1.5);
  EXPECT_DOUBLE_EQ(trajectory.points.back().vx, 1.5);
  EXPECT_GT(trajectory.totalTime(), original.totalTime());
}

TEST(WheelSpeedTimeScaling, NarrowTurnIgnoresStraightAndUnannotatedPoints)
{
  minco_planner::WheelSpeedTimeScalingParams params;
  params.narrow_turn_speed_limit = 0.8;
  params.narrow_turn_clearance = 0.55;

  // 窄但不转向：不放慢。
  auto straight = makeTurningTrajectory(20, 0, 0);
  for (auto & point : straight.points) {
    point.clearance = 0.3;
  }
  EXPECT_FALSE(minco_planner::applyNarrowTurnTimeScaling(straight, params));

  // 转向但净空未标注：不放慢。
  auto unannotated = makeTurningTrajectory(20, 5, 10);
  EXPECT_FALSE(minco_planner::applyNarrowTurnTimeScaling(unannotated, params));

  // 默认关闭。
  auto turning = makeTurningTrajectory(20, 5, 10);
  for (auto & point : turning.points) {
    point.clearance = 0.3;
  }
  EXPECT_FALSE(minco_planner::applyNarrowTurnTimeScaling(
    turning, minco_planner::WheelSpeedTimeScalingParams()));
}

TEST(WheelSpeedTimeScaling, ClearanceSpeedLimitIsMonotonicAndContinuous)
{
  minco_planner::WheelSpeedTimeScalingParams params;
  params.clearance_speed_min = 0.6;
  params.clearance_speed_low = 0.45;
  params.clearance_speed_high = 0.9;
  EXPECT_DOUBLE_EQ(minco_planner::clearanceSpeedLimit(0.30, params), 0.6);
  EXPECT_DOUBLE_EQ(minco_planner::clearanceSpeedLimit(0.45, params), 0.6);
  double previous = 0.6;
  for (double c = 0.46; c < 0.9; c += 0.01) {
    const double limit = minco_planner::clearanceSpeedLimit(c, params);
    EXPECT_GE(limit, previous);
    previous = limit;
  }
  EXPECT_TRUE(std::isinf(minco_planner::clearanceSpeedLimit(0.9, params)));
  EXPECT_TRUE(std::isinf(minco_planner::clearanceSpeedLimit(
    std::numeric_limits<double>::quiet_NaN(), params)));
}

TEST(WheelSpeedTimeScaling, ClearanceSpeedSlowsOnlyLowClearancePoints)
{
  auto trajectory = makeTurningTrajectory(40, 0, 0);
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].clearance = (i >= 18 && i < 22) ? 0.40 : 2.0;
  }
  const auto original = trajectory;
  minco_planner::WheelSpeedTimeScalingParams params;
  params.clearance_speed_min = 0.6;
  params.clearance_speed_low = 0.45;
  params.clearance_speed_high = 0.9;
  params.scale_change_rate = 2.0;

  EXPECT_TRUE(minco_planner::applyClearanceSpeedTimeScaling(trajectory, params));
  for (std::size_t i = 18; i < 22; ++i) {
    EXPECT_LE(std::hypot(trajectory.points[i].vx, trajectory.points[i].vy), 0.6 + 1e-9);
    EXPECT_DOUBLE_EQ(trajectory.points[i].x, original.points[i].x);
  }
  EXPECT_DOUBLE_EQ(trajectory.points.front().vx, 1.5);
  EXPECT_DOUBLE_EQ(trajectory.points.back().vx, 1.5);

  auto untouched = makeTurningTrajectory(40, 0, 0);
  EXPECT_FALSE(minco_planner::applyClearanceSpeedTimeScaling(
    untouched, minco_planner::WheelSpeedTimeScalingParams()));
}
