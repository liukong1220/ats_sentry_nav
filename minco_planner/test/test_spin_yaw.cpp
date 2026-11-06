// Copyright 2026

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "minco_planner/trajectory/spin_yaw.hpp"

namespace minco_planner
{
namespace
{

ReferenceTrajectory line(std::size_t count, double dt, double clearance)
{
  ReferenceTrajectory trajectory;
  trajectory.points.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto & p = trajectory.points[i];
    p.t = dt * static_cast<double>(i);
    p.s = 0.1 * static_cast<double>(i);
    p.x = p.s;
    p.yaw = 0.2;
    p.clearance = clearance;
  }
  return trajectory;
}

}  // namespace

TEST(SpinYaw, OpenReferenceAdvancesByRateTimesDt)
{
  auto trajectory = line(11, 0.1, 1.0);
  SpinYawParams params;
  params.rate = 3.0;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(trajectory.points.front().yaw, 0.2, 1e-12);
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double expected = wrapAngle(0.2 + 3.0 * 0.1 * static_cast<double>(i));
    EXPECT_NEAR(trajectory.points[i].yaw, expected, 1e-9);
    EXPECT_NEAR(trajectory.points[i].yaw_rate, 3.0, 1e-12);
  }
}

TEST(SpinYaw, YawIsWrappedAndStaysContinuousModuloTwoPi)
{
  auto trajectory = line(41, 0.1, 1.0);
  SpinYawParams params;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    EXPECT_LE(std::abs(trajectory.points[i].yaw), M_PI + 1e-12);
    const double step = wrapAngle(trajectory.points[i].yaw - trajectory.points[i - 1].yaw);
    EXPECT_NEAR(step, 0.3, 1e-9);
  }
}

TEST(SpinYaw, NarrowPointStopsSpinAndBlendsBackAtLimitedRate)
{
  auto trajectory = line(41, 0.1, 1.0);
  for (std::size_t i = 10; i < trajectory.points.size(); ++i) {
    trajectory.points[i].clearance = 0.2;
    trajectory.points[i].yaw = 0.2;
  }
  SpinYawParams params;
  params.rate = 3.0;
  params.blend_rate_limit = 2.0;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(trajectory.points[9].yaw_rate, 3.0, 1e-12);
  for (std::size_t i = 10; i < trajectory.points.size(); ++i) {
    const double step = wrapAngle(trajectory.points[i].yaw - trajectory.points[i - 1].yaw);
    EXPECT_LE(std::abs(step), 2.0 * 0.1 + 1e-9);
  }
  // 收敛后与规划 yaw 一致。
  EXPECT_NEAR(wrapAngle(trajectory.points.back().yaw - 0.2), 0.0, 1e-9);
}

TEST(SpinYaw, ResumesSpinningAfterTheNarrowSectionOnceClearAgain)
{
  auto trajectory = line(61, 0.1, 1.0);  // s = 0.1 * i
  for (std::size_t i = 20; i <= 30; ++i) {
    trajectory.points[i].clearance = 0.2;  // 窄段 s 2.0..3.0（隧道）
  }
  SpinYawParams params;
  params.stop_before_narrow_m = 1.5;
  params.resume_after_narrow_m = 0.5;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(trajectory.points[3].yaw_rate, params.rate, 1e-12);
  for (std::size_t i = 5; i <= 35; ++i) {
    EXPECT_LT(trajectory.points[i].yaw_rate, params.rate) << i;
  }
  // 窄段内 yaw 已回到规划 yaw（对齐隧道）。
  for (std::size_t i = 20; i <= 30; ++i) {
    EXPECT_NEAR(wrapAngle(trajectory.points[i].yaw - 0.2), 0.0, 1e-9) << i;
  }
  EXPECT_NEAR(trajectory.points[45].yaw_rate, params.rate, 1e-12);
}

TEST(SpinYaw, NarrowStartStillSpinsInTheOpenPartLaterOn)
{
  auto trajectory = line(41, 0.1, 1.0);
  for (std::size_t i = 0; i <= 5; ++i) {
    trajectory.points[i].clearance = 0.2;
  }
  SpinYawParams params;
  params.resume_after_narrow_m = 0.5;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_LT(trajectory.points[5].yaw_rate, params.rate);
  EXPECT_NEAR(trajectory.points[20].yaw_rate, params.rate, 1e-12);
}

TEST(SpinYaw, DecelerationFromSpinRespectsAngularAccel)
{
  auto trajectory = line(201, 0.05, 1.0);
  for (std::size_t i = 60; i < trajectory.points.size(); ++i) {
    trajectory.points[i].clearance = 0.2;
  }
  SpinYawParams params;
  params.angular_accel = 2.0;
  params.blend_rate_limit = 2.0;
  params.stop_before_narrow_m = 1.0;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(wrapAngle(trajectory.points.back().yaw - 0.2), 0.0, 1e-9);
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    EXPECT_LE(
      std::abs(trajectory.points[i].yaw_rate - trajectory.points[i - 1].yaw_rate),
      2.0 * 0.05 + 1e-9) << i;
  }
}

TEST(SpinYaw, UnknownClearanceAndNonPositiveRateDoNotSpin)
{
  auto unknown = line(5, 0.1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(applySpinYaw(unknown, SpinYawParams{}));
  auto open = line(5, 0.1, 1.0);
  SpinYawParams params;
  params.rate = 0.0;
  EXPECT_FALSE(applySpinYaw(open, params));
  for (const auto & p : open.points) {
    EXPECT_NEAR(p.yaw, 0.2, 1e-12);
  }
}

TEST(SpinYaw, RampsFromInitialRateWithLimitedAcceleration)
{
  auto trajectory = line(41, 0.1, 1.0);
  SpinYawParams params;
  params.rate = 3.0;
  params.initial_rate = 0.0;
  params.angular_accel = 2.0;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  double previous_rate = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double rate = trajectory.points[i].yaw_rate;
    EXPECT_LE(rate - previous_rate, 2.0 * 0.1 + 1e-9);
    EXPECT_LE(rate, 3.0 + 1e-12);
    previous_rate = rate;
  }
  EXPECT_NEAR(previous_rate, 3.0, 1e-9);
  EXPECT_LT(trajectory.points[1].yaw_rate, 0.5);
}

TEST(SpinYaw, ContinuingSpinStartsAtFullRate)
{
  auto trajectory = line(5, 0.1, 1.0);
  SpinYawParams params;
  params.rate = 3.0;
  params.initial_rate = 3.0;
  params.angular_accel = 2.0;
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(trajectory.points[1].yaw_rate, 3.0, 1e-12);
}

TEST(SpinYaw, StopsSpinningAheadOfTheNarrowPointByTheConfiguredDistance)
{
  auto trajectory = line(41, 0.1, 1.0);  // s = 0.1 * i
  for (std::size_t i = 30; i < trajectory.points.size(); ++i) {
    trajectory.points[i].clearance = 0.2;
  }
  SpinYawParams params;
  params.stop_before_narrow_m = 1.0;  // 窄点 s=3.0，从 s=2.0（i=20）起不再自旋
  ASSERT_TRUE(applySpinYaw(trajectory, params));
  EXPECT_NEAR(trajectory.points[19].yaw_rate, params.rate, 1e-12);
  EXPECT_LT(trajectory.points[20].yaw_rate, params.rate);
  for (std::size_t i = 20; i < trajectory.points.size(); ++i) {
    EXPECT_LT(trajectory.points[i].yaw_rate, params.rate);
  }
}

}  // namespace minco_planner
