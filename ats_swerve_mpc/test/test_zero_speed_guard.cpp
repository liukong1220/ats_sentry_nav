// Copyright 2026

#include <array>

#include <gtest/gtest.h>

#include "ats_swerve_mpc/zero_speed_guard.hpp"

namespace {

TEST(ZeroSpeedGuard, UsesHysteresisAroundTheZeroSpeedDeadband) {
  ats_swerve_mpc::ZeroSpeedGuard guard;
  EXPECT_TRUE(guard.update({0.0, 0.0, 0.0, 0.0}));
  EXPECT_TRUE(guard.active());
  EXPECT_TRUE(guard.update({0.015, 0.015, 0.015, 0.015}));
  EXPECT_TRUE(guard.active());
  EXPECT_FALSE(guard.update({0.025, 0.025, 0.025, 0.025}));
  EXPECT_FALSE(guard.active());
}

TEST(ZeroSpeedGuard, DoesNotLinearizeUndefinedSteeringDirection) {
  ats_swerve_mpc::ZeroSpeedGuard guard;
  guard.update({0.0, 0.0, 0.0, 0.0});
  EXPECT_FALSE(guard.angleConstraintDefined(0.0, 0.5));
  EXPECT_FALSE(guard.angleConstraintDefined(0.5, 0.0));
  guard.update({0.05, 0.05, 0.05, 0.05});
  EXPECT_TRUE(guard.angleConstraintDefined(0.05, 0.06));
}

}  // namespace
