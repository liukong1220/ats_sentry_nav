// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "minco_planner/trajectory/replan_start.hpp"

namespace
{
using minco_planner::MeasuredStart;
using minco_planner::ReferenceTrajectory;
using minco_planner::ReplanStart;
using minco_planner::ReplanStartParams;
using minco_planner::ReplanStartReason;
using minco_planner::ReplanStartSource;

constexpr std::uint64_t kEpoch = 7;

// 沿 +x 的匀加速参考：v0 + accel * t，每 0.1 s 一个点。accel=0 即 v0 的匀速直线。
ReferenceTrajectory makeAlongX(double v0, double accel, double duration)
{
  ReferenceTrajectory trajectory;
  const int count = static_cast<int>(std::lround(duration / 0.1)) + 1;
  for (int i = 0; i < count; ++i) {
    const double t = 0.1 * i;
    minco_planner::ReferencePoint point;
    point.t = t;
    point.x = v0 * t + 0.5 * accel * t * t;
    point.s = point.x;
    point.v = v0 + accel * t;
    point.vx = point.v;
    point.ax = accel;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// 车在参考上 elapsed 时刻的位置，再横向偏 lateral。速度取该点的标称值。
MeasuredStart carAt(const ReferenceTrajectory & trajectory, double elapsed, double lateral = 0.0)
{
  const auto on_track = minco_planner::interpolateReferencePoint(trajectory, elapsed);
  MeasuredStart measured;
  measured.x = on_track.x;
  measured.y = on_track.y + lateral;
  measured.vx = on_track.vx;
  return measured;
}

ReplanStartParams defaultParams()
{
  return ReplanStartParams{};  // plan_lead 0.15 s，deviation_max 0.5 m
}
}  // namespace

TEST(ReplanStart, StartsAtCommittedTrajectoryOnePlanLeadAhead)
{
  // 车以 1 m/s 沿已提交直线执行，t_now 距轨迹起点 2.0 s，车在 (2.0, 0.05)。
  const auto trajectory = makeAlongX(1.0, 0.0, 10.0);
  const double elapsed = 2.0;
  const MeasuredStart measured = carAt(trajectory, elapsed, 0.05);
  const ReplanStart start =
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, elapsed, measured, defaultParams());

  ASSERT_EQ(start.source, ReplanStartSource::kCommitted);
  EXPECT_EQ(start.reason, ReplanStartReason::kCommitted);
  // 起点 == traj(t_now + plan_lead)，不是实测位姿。
  const auto expected = minco_planner::interpolateReferencePoint(trajectory, elapsed + 0.15);
  EXPECT_NEAR(start.x, expected.x, 1e-9);
  EXPECT_NEAR(start.x, 2.15, 1e-9);
  EXPECT_NEAR(start.y, 0.0, 1e-9);
  EXPECT_NEAR(start.vx, 1.0, 1e-9);
  EXPECT_NEAR(start.t_ref, 2.15, 1e-9);
  EXPECT_NEAR(start.start_delay_sec, 0.15, 1e-9);
  EXPECT_GT(std::hypot(start.x - measured.x, start.y - measured.y), 0.14);
}

TEST(ReplanStart, SeedsVelocityAndAccelerationFromCommittedTrajectory)
{
  const auto trajectory = makeAlongX(0.5, 0.5, 10.0);
  const MeasuredStart measured = carAt(trajectory, 2.0);
  const ReplanStart start =
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, 2.0, measured, defaultParams());

  ASSERT_EQ(start.source, ReplanStartSource::kCommitted);
  EXPECT_NEAR(start.vx, 0.5 + 0.5 * 2.15, 1e-9);
  EXPECT_NEAR(start.ax, 0.5, 1e-9);
  // 位置在 0.1 s 采样点之间线性插值，二次曲线的插值误差约 6e-4 m。
  EXPECT_NEAR(start.x, 0.5 * 2.15 + 0.25 * 2.15 * 2.15, 2e-3);
}

TEST(ReplanStart, AnchorsOnRobotProgressWhenCarLagsWallClock)
{
  // 车比墙钟落后 0.3 s（拐角减速后常见）。起点跟车的推进走，不跟墙钟，
  // 否则新起点会落在车前 0.45 m，而不是 plan_lead 对应的 0.15 m。
  const auto trajectory = makeAlongX(1.0, 0.0, 10.0);
  const MeasuredStart car = carAt(trajectory, 1.7);
  const ReplanStart start =
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, 2.0, car, defaultParams());

  ASSERT_EQ(start.source, ReplanStartSource::kCommitted);
  EXPECT_NEAR(start.progress, 1.7, 1e-9);
  EXPECT_NEAR(start.t_ref, 1.85, 1e-9);
  EXPECT_NEAR(start.x, 1.85, 1e-9);
  EXPECT_NEAR(start.start_delay_sec, 0.15, 1e-9);
}
TEST(ReplanStart, FallsBackToMeasuredWhenDeviationReachesMax)
{
  const auto trajectory = makeAlongX(1.0, 0.0, 10.0);
  const MeasuredStart far = carAt(trajectory, 2.0, 0.6);
  const ReplanStart start =
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, 2.0, far, defaultParams());

  EXPECT_EQ(start.source, ReplanStartSource::kMeasured);
  EXPECT_EQ(start.reason, ReplanStartReason::kDeviation);
  EXPECT_NEAR(start.deviation, 0.6, 1e-9);
  EXPECT_DOUBLE_EQ(start.x, far.x);
  EXPECT_DOUBLE_EQ(start.y, far.y);
  EXPECT_DOUBLE_EQ(start.vx, far.vx);
  EXPECT_DOUBLE_EQ(start.ax, 0.0);
  EXPECT_DOUBLE_EQ(start.start_delay_sec, 0.0);

  // 阈值以内仍取轨迹。
  const MeasuredStart close = carAt(trajectory, 2.0, 0.49);
  EXPECT_EQ(
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, 2.0, close, defaultParams())
    .source,
    ReplanStartSource::kCommitted);
}

TEST(ReplanStart, ClampsReferenceTimeAtTrajectoryDuration)
{
  const auto trajectory = makeAlongX(1.0, 0.0, 10.0);
  const double duration = trajectory.points.back().t;

  // 距末端 0.05 s，elapsed + plan_lead 越界。
  const double elapsed = duration - 0.05;
  const ReplanStart near_end = minco_planner::selectReplanStart(
    &trajectory, kEpoch, kEpoch, elapsed, carAt(trajectory, elapsed), defaultParams());
  ASSERT_EQ(near_end.source, ReplanStartSource::kCommitted);
  EXPECT_DOUBLE_EQ(near_end.t_ref, duration);
  EXPECT_NEAR(near_end.x, trajectory.points.back().x, 1e-9);
  EXPECT_NEAR(near_end.start_delay_sec, 0.05, 1e-9);

  // t_now 已超过轨迹时长（车停在终点附近）：t_ref 仍夹在 duration。
  const ReplanStart past = minco_planner::selectReplanStart(
    &trajectory, kEpoch, kEpoch, duration + 3.0, carAt(trajectory, duration), defaultParams());
  ASSERT_EQ(past.source, ReplanStartSource::kCommitted);
  EXPECT_DOUBLE_EQ(past.t_ref, duration);
  EXPECT_NEAR(past.x, trajectory.points.back().x, 1e-9);
}

TEST(ReplanStart, UsesMeasuredWithoutUsableCommittedTrajectory)
{
  const auto trajectory = makeAlongX(1.0, 0.0, 10.0);
  const MeasuredStart car = carAt(trajectory, 2.0);

  // 定位 epoch 不同：旧参考在另一个定位基准下，不能用。
  const ReplanStart other_epoch = minco_planner::selectReplanStart(
    &trajectory, kEpoch, kEpoch + 1, 2.0, car, defaultParams());
  EXPECT_EQ(other_epoch.source, ReplanStartSource::kMeasured);
  EXPECT_EQ(other_epoch.reason, ReplanStartReason::kEpochMismatch);
  EXPECT_DOUBLE_EQ(other_epoch.x, car.x);

  // 没有正在执行的轨迹（含急停撤销）。
  const ReplanStart none = minco_planner::selectReplanStart(
    nullptr, kEpoch, kEpoch, 2.0, car, defaultParams());
  EXPECT_EQ(none.source, ReplanStartSource::kMeasured);
  EXPECT_EQ(none.reason, ReplanStartReason::kNoTrajectory);
  EXPECT_DOUBLE_EQ(none.x, car.x);
  EXPECT_DOUBLE_EQ(none.ax, 0.0);

  // deviation_max <= 0 关闭该功能。
  ReplanStartParams off = defaultParams();
  off.deviation_max_m = 0.0;
  const ReplanStart disabled =
    minco_planner::selectReplanStart(&trajectory, kEpoch, kEpoch, 2.0, car, off);
  EXPECT_EQ(disabled.source, ReplanStartSource::kMeasured);
  EXPECT_EQ(disabled.reason, ReplanStartReason::kDisabled);
}

TEST(ReplanStart, InterpolatesYawAlongShortestArc)
{
  ReferenceTrajectory trajectory;
  for (int i = 0; i < 2; ++i) {
    minco_planner::ReferencePoint point;
    point.t = static_cast<double>(i);
    point.x = static_cast<double>(i);
    point.s = static_cast<double>(i);
    point.yaw = i == 0 ? 3.1 : -3.1;
    trajectory.points.push_back(point);
  }
  const auto middle = minco_planner::interpolateReferencePoint(trajectory, 0.5);
  EXPECT_NEAR(std::abs(middle.yaw), M_PI, 1e-6);
}
