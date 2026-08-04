// Copyright 2026

#include <chrono>

#include "ats_goal_manager/plan_progress_watchdog.hpp"
#include "gtest/gtest.h"

namespace
{

using Watchdog = ats_goal_manager::PlanProgressWatchdog;
using Decision = ats_goal_manager::PlanProgressDecision;
using Clock = Watchdog::Clock;

ats_goal_manager::PlanProgressGate healthyGate(double distance)
{
  ats_goal_manager::PlanProgressGate gate;
  gate.goal_active = true;
  gate.emergency_stop = false;
  gate.map_fresh = true;
  gate.localization_fresh = true;
  gate.tf_healthy = true;
  gate.robot_inside_map = true;
  gate.robot_cell_free = true;
  gate.footprint_safe = true;
  gate.has_current_reference = true;
  gate.distance_to_goal_m = distance;
  gate.identity.goal_id = 7;
  gate.identity.localization_epoch = 3;
  gate.identity.map_publication_sequence = 11;
  gate.identity.map_generation = 19;
  gate.identity.plan_request_sequence = 2;
  return gate;
}

TEST(PlanProgressWatchdog, WaitsForStallAndHonorsProgressReset)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.progress_min_delta_m = 0.20;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 0.5;
  params.max_consecutive_replans = 2;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(5.0);
  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  gate.distance_to_goal_m = 4.7;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(500)), Decision::kNone);
  gate.distance_to_goal_m = 4.4;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(900)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(1800)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(1901)), Decision::kReplan);
  EXPECT_EQ(watchdog.consecutiveReplans(), 1U);
}

TEST(PlanProgressWatchdog, ReplansOnceThenUsesMinimumInterval)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 2.0;
  params.max_consecutive_replans = 2;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  const auto gate = healthyGate(5.0);

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(1)), Decision::kReplan);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(2100)), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(3001)), Decision::kReplan);
}

TEST(PlanProgressWatchdog, RejectsUnsafeGatesWithoutReplanning)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 0.1;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto gate = healthyGate(5.0);
  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);

  for (auto member : {0, 1, 2, 3, 4, 5}) {
    auto unsafe = gate;
    switch (member) {
      case 0: unsafe.map_fresh = false; break;
      case 1: unsafe.localization_fresh = false; break;
      case 2: unsafe.tf_healthy = false; break;
      case 3: unsafe.robot_inside_map = false; break;
      case 4: unsafe.robot_cell_free = false; break;
      default: unsafe.footprint_safe = false; break;
    }
    EXPECT_EQ(
      watchdog.evaluate(unsafe, t0 + std::chrono::seconds(2)), Decision::kUnsafe);
    EXPECT_EQ(watchdog.consecutiveReplans(), 0U);
  }
}

TEST(PlanProgressWatchdog, ExhaustsBoundedReplans)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 0.0;
  params.max_consecutive_replans = 1;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  const auto gate = healthyGate(5.0);

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(1)), Decision::kReplan);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(2)), Decision::kExhausted);
  EXPECT_EQ(watchdog.consecutiveReplans(), 1U);
}

}  // namespace
