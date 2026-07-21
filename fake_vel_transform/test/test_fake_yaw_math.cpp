// Copyright 2026

#include <gtest/gtest.h>

#include "fake_vel_transform/fake_yaw_math.hpp"

namespace
{

TEST(FakeYawMath, PreservesNonZeroInitialYawAndRoundTripsVelocity)
{
  constexpr double initial_yaw = 0.7;
  constexpr double current_yaw = 0.7 + M_PI_2;
  EXPECT_NEAR(fake_vel_transform::gimbalToFakeYaw(initial_yaw, initial_yaw), 0.0, 1e-12);
  EXPECT_NEAR(fake_vel_transform::gimbalToFakeYaw(initial_yaw, current_yaw), -M_PI_2, 1e-12);

  const fake_vel_transform::PlanarVelocity fake_velocity{1.0, 0.0};
  const auto gimbal_velocity =
    fake_vel_transform::fakeToGimbalVelocity(fake_velocity, initial_yaw, current_yaw);
  EXPECT_NEAR(gimbal_velocity.x, 0.0, 1e-12);
  EXPECT_NEAR(gimbal_velocity.y, 1.0, 1e-12);
}

TEST(FakeYawMath, WrapsAcrossPiWithoutChangingTheRotationDirection)
{
  const double initial_yaw = M_PI - 0.1;
  const double current_yaw = -M_PI + 0.1;
  EXPECT_NEAR(fake_vel_transform::gimbalToFakeYaw(initial_yaw, current_yaw), -0.2, 1e-12);
  const auto velocity =
    fake_vel_transform::fakeToGimbalVelocity({1.0, 0.0}, initial_yaw, current_yaw);
  EXPECT_NEAR(velocity.x, std::cos(0.2), 1e-12);
  EXPECT_NEAR(velocity.y, std::sin(0.2), 1e-12);
}

}  // namespace
