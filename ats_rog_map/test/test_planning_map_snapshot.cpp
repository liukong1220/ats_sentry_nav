// Copyright 2026

#include <cmath>
#include <limits>

#include "ats_rog_map/planning/planning_map_snapshot.hpp"
#include "gtest/gtest.h"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.info.resolution = 0.5F;
  grid.info.width = 2U;
  grid.info.height = 2U;
  grid.info.origin.orientation.w = 1.0;
  grid.data = {0, 100, -1, 0};
  return grid;
}

builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time value;
  value.sec = 12;
  value.nanosec = 34U;
  return value;
}

TEST(PlanningMapSnapshot, PreservesIdentityAndCarriesUnknownAsBlockedDistance)
{
  const auto grid = makeGrid();
  const std::vector<double> signed_distance = {1.0, -0.5, -0.5, 2.0};
  const auto snapshot = ats_rog_map::planning::makePlanningMapSnapshot(
    grid, signed_distance, stamp(), 9U, 17U, 23U, true, 50);

  EXPECT_TRUE(snapshot.ready);
  EXPECT_EQ(snapshot.source_generation, 17U);
  EXPECT_EQ(snapshot.localization_epoch, 9U);
  EXPECT_EQ(snapshot.publication_sequence, 23U);
  EXPECT_EQ(snapshot.source_stamp.sec, 0);
  ASSERT_EQ(snapshot.occupancy.size(), 4U);
  EXPECT_EQ(snapshot.occupancy[0], 0);
  EXPECT_EQ(snapshot.occupancy[1], 100);
  EXPECT_EQ(snapshot.occupancy[2], -1);
  EXPECT_FLOAT_EQ(snapshot.signed_distance_m[2], -0.5F);
  EXPECT_TRUE(std::isfinite(snapshot.gradient_x[2]));
  EXPECT_EQ(snapshot.header.stamp.sec, 12);
  EXPECT_EQ(snapshot.header.stamp.nanosec, 34U);
}

TEST(PlanningMapSnapshot, RejectsInconsistentOrNonFiniteSignedDistance)
{
  const auto grid = makeGrid();
  // occupied cell with positive distance
  EXPECT_FALSE(ats_rog_map::planning::makePlanningMapSnapshot(
      grid, std::vector<double>{1.0, 0.5, -0.5, 2.0}, stamp(), 1U, 2U, 3U, true, 50).ready);
  // unknown cell treated as obstacle but given positive distance
  EXPECT_FALSE(ats_rog_map::planning::makePlanningMapSnapshot(
      grid, std::vector<double>{1.0, -0.5, 0.5, 2.0}, stamp(), 1U, 2U, 3U, true, 50).ready);
  // NaN anywhere
  EXPECT_FALSE(ats_rog_map::planning::makePlanningMapSnapshot(
      grid, std::vector<double>{1.0, -0.5, std::numeric_limits<double>::quiet_NaN(), 2.0},
      stamp(), 1U, 2U, 3U, true, 50).ready);
  // unknown is free when unknown_is_obstacle is false
  EXPECT_TRUE(ats_rog_map::planning::makePlanningMapSnapshot(
      grid, std::vector<double>{1.0, -0.5, 0.5, 2.0}, stamp(), 1U, 2U, 3U, false, 50).ready);
}

TEST(PlanningMapSnapshot, RotatesLocalGradientIntoWorldFrame)
{
  auto grid = makeGrid();
  constexpr double pi = 3.14159265358979323846;
  const double yaw = 0.5 * pi;
  grid.info.origin.orientation.z = std::sin(0.5 * yaw);
  grid.info.origin.orientation.w = std::cos(0.5 * yaw);
  const auto snapshot = ats_rog_map::planning::makePlanningMapSnapshot(
    grid, std::vector<double>{0.0, -1.0, 0.0, 1.0}, stamp(), 1U, 2U, 3U, true, 50);

  ASSERT_TRUE(snapshot.ready);
  EXPECT_NEAR(snapshot.gradient_x[0], 0.0, 1e-6);
  EXPECT_NEAR(snapshot.gradient_y[0], -2.0, 1e-6);
}

TEST(PlanningMapSnapshot, UnavailableSnapshotNeverClaimsReady)
{
  const auto snapshot = ats_rog_map::planning::makeUnavailablePlanningMapSnapshot(
    makeGrid(), stamp(), 4U, 5U, 6U, true, 50);
  EXPECT_FALSE(snapshot.ready);
  EXPECT_TRUE(snapshot.occupancy.empty());
  EXPECT_EQ(snapshot.source_generation, 5U);
  EXPECT_EQ(snapshot.publication_sequence, 6U);
}

TEST(PlanningMapSnapshot, BlockedUnavailableSnapshotRetainsUnknownAuditPayload)
{
  auto grid = makeGrid();
  grid.data.assign(grid.data.size(), -1);
  const auto snapshot = ats_rog_map::planning::makeBlockedUnavailablePlanningMapSnapshot(
    grid, stamp(), 4U, 5U, 6U, true, 50);

  EXPECT_FALSE(snapshot.ready);
  EXPECT_EQ(snapshot.occupancy, grid.data);
  ASSERT_EQ(snapshot.signed_distance_m.size(), grid.data.size());
  EXPECT_TRUE(std::isnan(snapshot.signed_distance_m.front()));
  EXPECT_TRUE(std::isnan(snapshot.gradient_x.front()));
  EXPECT_TRUE(std::isnan(snapshot.gradient_y.front()));
}

}  // namespace
