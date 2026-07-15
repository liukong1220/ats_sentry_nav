// Copyright 2026

#include <chrono>

#include "ats_swerve_mpc/emergency_stop_watchdog.hpp"
#include "gtest/gtest.h"

namespace
{

using Watchdog = ats_swerve_mpc::EmergencyStopWatchdog;

TEST(EmergencyStopWatchdog, StartsStoppedAndRequiresFreshReleaseHeartbeat)
{
  Watchdog watchdog(0.5);
  const auto start = Watchdog::Clock::time_point(std::chrono::seconds(10));

  EXPECT_TRUE(watchdog.stopRequired(start));
  watchdog.update(false, start);
  EXPECT_FALSE(watchdog.stopRequired(start + std::chrono::milliseconds(500)));
  EXPECT_TRUE(watchdog.stopRequired(start + std::chrono::milliseconds(501)));
}

TEST(EmergencyStopWatchdog, StopSignalOverridesAValidLease)
{
  Watchdog watchdog(1.0);
  const auto start = Watchdog::Clock::time_point(std::chrono::seconds(10));

  watchdog.update(false, start);
  EXPECT_FALSE(watchdog.stopRequired(start + std::chrono::milliseconds(100)));
  watchdog.update(true, start + std::chrono::milliseconds(200));
  EXPECT_TRUE(watchdog.stopRequired(start + std::chrono::milliseconds(201)));
}

}  // namespace
