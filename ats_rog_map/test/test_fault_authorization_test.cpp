// Copyright 2026

#include <string>
#include <vector>

#include "ats_rog_map/test_fault_authorization.hpp"
#include "gtest/gtest.h"

namespace
{

const std::vector<std::string> kGuarded{"test_reset_to_unknown"};

TEST(RogMapTestFaultAuthorization, RefusesResetToUnknownWhileGateIsOff)
{
  const auto verdict =
    ats_rog_map::screenTestFaultParameters(false, {"test_reset_to_unknown"}, kGuarded);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_NE(verdict.reason.find("test_reset_to_unknown"), std::string::npos);
  EXPECT_NE(verdict.reason.find("enable_test_fault_injection=false"), std::string::npos);
}

TEST(RogMapTestFaultAuthorization, AllowsResetToUnknownOnlyWhileGateIsOn)
{
  const auto verdict =
    ats_rog_map::screenTestFaultParameters(true, {"test_reset_to_unknown"}, kGuarded);
  EXPECT_TRUE(verdict.accepted);
  EXPECT_TRUE(verdict.reason.empty());
}

TEST(RogMapTestFaultAuthorization, RefusesRuntimeChangesToTheGateItself)
{
  for (const bool gate : {false, true}) {
    const auto verdict = ats_rog_map::screenTestFaultParameters(
      gate, {"enable_test_fault_injection"}, kGuarded);
    EXPECT_FALSE(verdict.accepted);
    EXPECT_NE(verdict.reason.find("startup-only"), std::string::npos);
  }
}

TEST(RogMapTestFaultAuthorization, RefusesMixedRequestsThatCarryAGuardedParameter)
{
  const auto verdict = ats_rog_map::screenTestFaultParameters(
    false, {"cloud_timeout_sec", "test_reset_to_unknown"}, kGuarded);
  EXPECT_FALSE(verdict.accepted);
}

TEST(RogMapTestFaultAuthorization, LeavesProductionParametersUntouchedWhileGateIsOff)
{
  const auto verdict = ats_rog_map::screenTestFaultParameters(
    false, {"cloud_timeout_sec", "odom_timeout_sec"}, kGuarded);
  EXPECT_TRUE(verdict.accepted);
}

}  // namespace
