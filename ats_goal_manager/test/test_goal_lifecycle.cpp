// Copyright 2026

#include "ats_goal_manager/goal_lifecycle.hpp"

#include <gtest/gtest.h>

namespace ats_goal_manager
{

TEST(GoalLifecycle, RefusesReferenceUntilMapAndMatchingGoalAreReady)
{
  GoalLifecycle lifecycle;
  lifecycle.start(7, false);
  EXPECT_EQ(lifecycle.state(), GoalLifecycleState::kWaitingForMap);
  EXPECT_TRUE(lifecycle.emergencyStopRequired());
  EXPECT_FALSE(lifecycle.referenceReady(7, true));
  EXPECT_TRUE(lifecycle.mapReady(7));

  lifecycle.start(7, true);
  EXPECT_FALSE(lifecycle.referenceReady(8, true));
  EXPECT_TRUE(lifecycle.referenceReady(7, true));
  EXPECT_EQ(lifecycle.state(), GoalLifecycleState::kTracking);
  EXPECT_FALSE(lifecycle.emergencyStopRequired());
}

TEST(GoalLifecycle, TerminalTransitionsAlwaysRequireEmergencyStop)
{
  GoalLifecycle lifecycle;
  lifecycle.start(9, true);
  ASSERT_TRUE(lifecycle.referenceReady(9, true));
  lifecycle.cancel();
  EXPECT_FALSE(lifecycle.active());
  EXPECT_EQ(lifecycle.state(), GoalLifecycleState::kCanceled);
  EXPECT_TRUE(lifecycle.emergencyStopRequired());

  lifecycle.start(10, true);
  lifecycle.preempt();
  EXPECT_EQ(lifecycle.state(), GoalLifecycleState::kPreempted);
  EXPECT_TRUE(lifecycle.emergencyStopRequired());

  lifecycle.start(11, true);
  lifecycle.timeout();
  EXPECT_EQ(lifecycle.state(), GoalLifecycleState::kTimedOut);
  EXPECT_TRUE(lifecycle.emergencyStopRequired());
}

}  // namespace ats_goal_manager
