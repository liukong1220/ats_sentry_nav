// Copyright 2026
#include "ats_goal_manager/stop_reference_barrier.hpp"

#include <limits>
#include <gtest/gtest.h>

using ats_goal_manager::StopReferenceBarrier;

TEST(StopReferenceBarrier, DefaultAndParameterBounds) {
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(), 0.01);
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(0.0), 0.002);
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(1.0), 0.1);
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(0.002), 0.002);
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(0.1), 0.1);
  EXPECT_DOUBLE_EQ(StopReferenceBarrier::boundedGuardSeconds(
    std::numeric_limits<double>::quiet_NaN()), 0.01);
}

TEST(StopReferenceBarrier, FirstStopAtZeroStillArmsTheBarrier) {
  StopReferenceBarrier gate;
  gate.recordStop(0, 0.01);
  EXPECT_FALSE(gate.permits(0));
  EXPECT_FALSE(gate.permits(9999999));
  EXPECT_TRUE(gate.permits(10000000));
}

TEST(StopReferenceBarrier, FrozenOrRegressedRosClockCannotReleaseTheBarrier) {
  StopReferenceBarrier gate;
  gate.recordStop(1000000000, 0.01);
  // Repeated retries have no wall-clock escape, regardless of retry count.
  for (int i = 0; i < 10000; ++i) {
    EXPECT_FALSE(gate.permits(1000000000));
  }
  EXPECT_FALSE(gate.permits(900000000));
  EXPECT_FALSE(gate.permits(1009999999));
  EXPECT_TRUE(gate.permits(1010000000));
}

TEST(StopReferenceBarrier, EveryNewStopRearmsAndReadinessDoesNotConsumeTheGate) {
  StopReferenceBarrier gate;
  gate.recordStop(1000000000, 0.01);
  EXPECT_TRUE(gate.permits(1010000000));
  gate.recordStop(1010000000, 0.01);
  EXPECT_FALSE(gate.permits(1010000000));
  EXPECT_TRUE(gate.permits(1020000000));
  EXPECT_TRUE(gate.permits(1020000000));
  EXPECT_FALSE(gate.permits(1019999999));
}
