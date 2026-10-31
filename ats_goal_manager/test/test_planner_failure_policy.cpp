// Copyright 2026

#include "ats_goal_manager/planner_failure_policy.hpp"

#include <gtest/gtest.h>

namespace ats_goal_manager
{

using Status = ats_navigation_interfaces::msg::PlannerStatus;

TEST(PlannerFailurePolicy, OptimizerAndRepairFailuresAreRetriedLikeNoPath)
{
  EXPECT_TRUE(plannerFailureIsTransient(Status::FAILURE_OPTIMIZER));
  EXPECT_TRUE(plannerFailureIsTransient(Status::FAILURE_REPAIR));
  EXPECT_TRUE(plannerFailureIsTransient(Status::FAILURE_NO_PATH));
}

TEST(PlannerFailurePolicy, ExistingTransientFailuresStayTransient)
{
  for (const auto reason : {
      Status::FAILURE_MAP_UNREADY, Status::FAILURE_RUNTIME_UNSAFE, Status::FAILURE_FOOTPRINT,
      Status::FAILURE_SNAPSHOT_CHANGED, Status::FAILURE_START_ENCLOSED,
      Status::FAILURE_START_OR_GOAL_OCCUPIED, Status::FAILURE_START_TF, Status::FAILURE_GOAL_TF,
      Status::FAILURE_REFERENCE_TF})
  {
    EXPECT_TRUE(plannerFailureIsTransient(reason)) << static_cast<int>(reason);
  }
}

TEST(PlannerFailurePolicy, InvalidGoalAndUnknownReasonsEndTheGoal)
{
  EXPECT_FALSE(plannerFailureIsTransient(Status::FAILURE_INVALID_GOAL));
  EXPECT_FALSE(plannerFailureIsTransient(Status::FAILURE_NONE));
  EXPECT_FALSE(plannerFailureIsTransient(200));
}

}  // namespace ats_goal_manager
