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
