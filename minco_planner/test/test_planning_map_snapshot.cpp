// Copyright 2026

#include <limits>

#include "gtest/gtest.h"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.info.resolution = 1.0;
  grid.info.width = 5;
  grid.info.height = 5;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(25, 0);
  return grid;
}

TEST(PlanningMapSnapshot, KeepsGridAndEsdfImmutableAcrossGenerations)
{
  auto first_grid = makeGrid();
  first_grid.data[12] = 100;
  const auto first = minco_planner::PlanningMapSnapshot::create(7, first_grid, 50, true);
  const double first_distance = first->clearance_esdf->getDistance(2.5, 2.5);

  auto second_grid = makeGrid();
  second_grid.data[0] = 100;
  const auto second = minco_planner::PlanningMapSnapshot::create(8, second_grid, 50, true);

  EXPECT_EQ(first->generation, 7U);
  EXPECT_EQ(second->generation, 8U);
  EXPECT_EQ(first->grid.data[12], 100);
  EXPECT_EQ(second->grid.data[12], 0);
  EXPECT_DOUBLE_EQ(first->clearance_esdf->getDistance(2.5, 2.5), first_distance);
  EXPECT_LT(first_distance, 0.0);
  EXPECT_GT(second->clearance_esdf->getDistance(2.5, 2.5), 0.0);
}

TEST(PlanningMapSnapshot, SafetyIdentityIgnoresHeartbeatButRejectsSemanticChanges)
{
  auto base_grid = makeGrid();
  base_grid.header.stamp.sec = 10;
  const auto base = minco_planner::PlanningMapSnapshot::create(7, base_grid, 50, true);
  ASSERT_NE(base, nullptr);

  auto heartbeat_grid = base_grid;
  heartbeat_grid.header.stamp.sec = 11;
  const auto heartbeat =
    minco_planner::PlanningMapSnapshot::create(8, heartbeat_grid, 50, true);
  ASSERT_NE(heartbeat, nullptr);
  EXPECT_EQ(base->safety_content_digest, heartbeat->safety_content_digest);
  EXPECT_TRUE(base->hasSameSafetyContent(*heartbeat));

  auto unknown_grid = base_grid;
  unknown_grid.data[0] = -1;
  const auto unknown = minco_planner::PlanningMapSnapshot::create(9, unknown_grid, 50, true);
  ASSERT_NE(unknown, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*unknown));

  auto occupied_grid = base_grid;
  occupied_grid.data[0] = 100;
  const auto occupied = minco_planner::PlanningMapSnapshot::create(10, occupied_grid, 50, true);
  ASSERT_NE(occupied, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*occupied));

  auto shifted_grid = base_grid;
  shifted_grid.info.origin.position.x = 0.1;
  const auto shifted = minco_planner::PlanningMapSnapshot::create(11, shifted_grid, 50, true);
  ASSERT_NE(shifted, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*shifted));

  const auto changed_unknown_policy =
    minco_planner::PlanningMapSnapshot::create(12, base_grid, 50, false);
  ASSERT_NE(changed_unknown_policy, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*changed_unknown_policy));
}

TEST(PlanningMapSnapshot, RejectsMalformedGrid)
{
  auto grid = makeGrid();
  grid.data.pop_back();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);

  grid = makeGrid();
  grid.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);

  grid.info.resolution = std::numeric_limits<float>::infinity();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);
}

TEST(PlannerSafetyState, RequiresBothReadyMapAndSafePlan)
{
  minco_planner::PlannerSafetyState state;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.map_ready = true;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.plan_safe = true;
  EXPECT_FALSE(state.emergencyStopRequired());
  state.map_ready = false;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.map_ready = true;
  state.plan_safe = false;
  EXPECT_TRUE(state.emergencyStopRequired());
}

TEST(PlannerSafetyState, HealthLossRequiresANewerMapSnapshot)
{
  minco_planner::PlannerSafetyState state;
  state.map_ready = true;
  EXPECT_TRUE(state.mapSnapshotUsable(7));

  state.invalidateMap(7);
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  state.map_ready = true;
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  EXPECT_TRUE(state.mapSnapshotUsable(8));
}

TEST(PlannerSafetyState, NewMapInvalidatesPlanButKeepsHealthyMapLease)
{
  minco_planner::PlannerSafetyState state;
  state.map_ready = true;
  state.plan_safe = true;
  state.minimum_map_generation = 7;

  state.invalidatePlanForNewMap(8);

  EXPECT_TRUE(state.map_ready);
  EXPECT_FALSE(state.plan_safe);
  EXPECT_TRUE(state.emergencyStopRequired());
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  EXPECT_TRUE(state.mapSnapshotUsable(8));
}

ats_navigation_interfaces::msg::PlannerStatus makeReadyStatus()
{
  ats_navigation_interfaces::msg::PlannerStatus status;
  status.header.stamp.sec = 10;
  status.goal_id = 12;
  status.localization_epoch = 3;
  status.plan_request_sequence = 7;
  status.map_generation = 5;
  status.map_publication_sequence = 19;
  status.reference_stamp.sec = 9;
  status.state = status.STATE_REFERENCE_READY;
  status.failure_reason = status.FAILURE_NONE;
  return status;
}

TEST(PlannerStatusState, MapLossRetainsIdentityWithoutAnActiveReference)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  EXPECT_FALSE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  ASSERT_TRUE(state.latest.has_value());
  EXPECT_EQ(state.latest->state, ready.STATE_FAILED);
  EXPECT_EQ(state.latest->failure_reason, ready.FAILURE_MAP_UNREADY);
  EXPECT_EQ(state.latest->goal_id, ready.goal_id);
  EXPECT_EQ(state.latest->localization_epoch, ready.localization_epoch);
  EXPECT_EQ(state.latest->plan_request_sequence, ready.plan_request_sequence);
  EXPECT_EQ(state.latest->map_generation, ready.map_generation);
  EXPECT_EQ(state.latest->map_publication_sequence, ready.map_publication_sequence);
  EXPECT_EQ(state.latest->reference_stamp, ready.reference_stamp);
  EXPECT_EQ(state.latest->header.stamp.nanosec, 1U);

  // Repeated false-ready/watchdog reports retain the same invalidated identity.
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  EXPECT_EQ(state.latest->header.stamp.nanosec, 2U);
  EXPECT_FALSE(state.update(ready));
  ++ready.plan_request_sequence;
  EXPECT_FALSE(state.update(ready));
  ++ready.map_generation;
  EXPECT_TRUE(state.update(ready));
}

TEST(PlannerStatusState, PlanFailureAndMapReplacementPermitFreshReference)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_RUNTIME_UNSAFE, ready.header.stamp));
  ++ready.plan_request_sequence;
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(6, ready.FAILURE_SNAPSHOT_CHANGED, ready.header.stamp));
  EXPECT_FALSE(state.update(ready));
  ++ready.map_generation;
  ++ready.plan_request_sequence;
  EXPECT_TRUE(state.update(ready));
  EXPECT_EQ(state.latest->state, ready.STATE_REFERENCE_READY);
}

TEST(PlannerStatusState, RejectsObsoleteEpochGenerationAndRequest)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ASSERT_TRUE(state.update(ready));
  auto stale = ready;
  --stale.localization_epoch;
  ++stale.map_generation;
  EXPECT_FALSE(state.update(stale));
  stale = ready;
  --stale.map_generation;
  EXPECT_FALSE(state.update(stale));
  stale = ready;
  --stale.plan_request_sequence;
  EXPECT_FALSE(state.update(stale));
  EXPECT_EQ(state.latest->goal_id, ready.goal_id);
  EXPECT_EQ(state.latest->map_generation, ready.map_generation);

  ++ready.localization_epoch;
  ready.map_generation = 1;
  ready.plan_request_sequence = 1;
  EXPECT_TRUE(state.update(ready));
}

TEST(PlannerStatusState, SerializesEqualAndBackwardClockStamps)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ready.header.stamp.nanosec = 999999999U;
  ASSERT_TRUE(state.update(ready));
  auto heartbeat = ready;
  ASSERT_TRUE(state.update(heartbeat));
  EXPECT_EQ(heartbeat.header.stamp.sec, 11);
  EXPECT_EQ(heartbeat.header.stamp.nanosec, 0U);
  heartbeat.header.stamp.sec = 1;
  ASSERT_TRUE(state.update(heartbeat));
  EXPECT_EQ(heartbeat.header.stamp.sec, 11);
  EXPECT_EQ(heartbeat.header.stamp.nanosec, 1U);
  EXPECT_EQ(heartbeat.map_generation, ready.map_generation);
  EXPECT_EQ(heartbeat.state, ready.STATE_REFERENCE_READY);
}

TEST(HeartbeatLease, RejectsMissingFutureAndExpiredSignals)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::time_point(std::chrono::seconds(10));

  EXPECT_FALSE(minco_planner::steadyHeartbeatLeaseValid(std::nullopt, start, 1.0));
  EXPECT_FALSE(
    minco_planner::steadyHeartbeatLeaseValid(
      start, start - std::chrono::milliseconds(1), 1.0));
  EXPECT_TRUE(
    minco_planner::steadyHeartbeatLeaseValid(start, start + std::chrono::seconds(1), 1.0));
  EXPECT_FALSE(
    minco_planner::steadyHeartbeatLeaseValid(
      start, start + std::chrono::milliseconds(1001), 1.0));
}

TEST(ReferenceTrajectory, RequiresFiniteMonotonicSamples)
{
  minco_planner::ReferenceTrajectory trajectory;
  EXPECT_FALSE(trajectory.valid());
  minco_planner::ReferencePoint first;
  minco_planner::ReferencePoint second;
  second.t = 1.0;
  second.s = 1.0;
  second.x = 1.0;
  trajectory.points = {first, second};
  EXPECT_TRUE(trajectory.valid());
  trajectory.points.back().t = 0.0;
  EXPECT_FALSE(trajectory.valid());
}

TEST(FootprintSafetyChecker, RejectsEmptyTrajectory)
{
  minco_planner::FootprintSafetyChecker checker;
  minco_planner::ReferenceTrajectory trajectory;
  EXPECT_FALSE(checker.check(trajectory, makeGrid()).safe);
}

}  // namespace
