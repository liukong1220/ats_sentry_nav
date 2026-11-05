// Copyright 2026

#include <limits>
#include <vector>
#include "ats_rog_map/ground_column.hpp"
#include "gtest/gtest.h"

using ats_rog_map::ColumnState;
using ats_rog_map::GroundColumnConfig;
using ats_rog_map::classifyGroundColumn;

TEST(GroundColumn, RampIsFreeRegardlessOfAbsoluteElevation)
{
  GroundColumnConfig config;
  for (const double ground : {-0.6, 0.0, 0.4, 1.5}) {
    EXPECT_EQ(classifyGroundColumn({ground, ground + 0.04}, 4, ground, config),
      ColumnState::kFree);
  }
}

TEST(GroundColumn, GroundRelativeLowObstacleAndWallRemainOccupied)
{
  GroundColumnConfig config;
  for (const double ground : {-0.4, 0.0, 0.5}) {
    EXPECT_EQ(classifyGroundColumn({ground, ground + 0.15}, 4, ground, config),
      ColumnState::kOccupied);
    EXPECT_EQ(classifyGroundColumn(
      {ground, ground + 0.1, ground + 0.2, ground + 0.3, ground + 0.4},
      5, ground, config), ColumnState::kOccupied);
  }
}

TEST(GroundColumn, CeilingMustClearRobotAndTopMargin)
{
  GroundColumnConfig config;
  config.robot_height = 0.255;
  config.top_margin = 0.05;
  EXPECT_EQ(classifyGroundColumn({0.4, 0.7}, 4, 0.4, config), ColumnState::kOccupied);
  EXPECT_EQ(classifyGroundColumn({0.4, 0.75}, 4, 0.4, config), ColumnState::kFree);
  // A hanging obstacle inside an otherwise high tunnel remains occupied.
  EXPECT_EQ(classifyGroundColumn({0.4, 0.6, 0.8}, 4, 0.4, config),
    ColumnState::kOccupied);
}

TEST(GroundColumn, InsufficientObservationAndMissingGroundFailClosed)
{
  GroundColumnConfig config;
  EXPECT_EQ(classifyGroundColumn({}, 0, 0.0, config), ColumnState::kUnknown);
  EXPECT_EQ(classifyGroundColumn({0.0}, 0, 0.0, config), ColumnState::kUnknown);
  EXPECT_EQ(classifyGroundColumn({0.5}, 4, std::nullopt, config), ColumnState::kUnknown);
  EXPECT_EQ(classifyGroundColumn({std::numeric_limits<double>::quiet_NaN()},
    4, 0.0, config), ColumnState::kUnknown);
  EXPECT_EQ(classifyGroundColumn({}, 4, 0.0, config), ColumnState::kFree);
}

TEST(GroundColumn, DenseWallCannotBecomeOverheadFree)
{
  GroundColumnConfig config;
  EXPECT_EQ(classifyGroundColumn({0.0, 0.1, 0.2, 0.3, 0.4, 0.5},
    6, 0.0, config), ColumnState::kOccupied);
}

TEST(GroundColumn, ConnectedGroundFollowsRampButDoesNotClimbFloatingPlate)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(30, 8, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 8; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, x * 0.03});
    }
  }
  grid.observe(points, {0.05, 0.35, 0.0});
  EXPECT_EQ(grid.state(25, 3), ColumnState::kFree);
  grid.clear();
  for (auto & point : points) {point[2] = point[0] > 1.0 ? 0.2 : 0.0;}
  grid.observe(points, {0.05, 0.35, 0.0});
  EXPECT_NE(grid.state(15, 3), ColumnState::kFree);
}

TEST(GroundColumn, NearbyFloatingPlateCannotSeedItsOwnGround)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 6, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, x < 3 ? 0.0 : 0.20});
    }
  }
  grid.observe(points, {0.05, 0.25, 0.0});
  EXPECT_EQ(grid.state(2, 2), ColumnState::kFree);
  EXPECT_EQ(grid.state(3, 2), ColumnState::kOccupied);
  EXPECT_NE(grid.state(5, 2), ColumnState::kFree);
}

namespace
{
// Flat ground (x < 0.3 m) joined to a 0.2 m platform: column (3, y) is the
// platform's upper edge, a step the robot cannot drive onto.
ats_rog_map::GroundColumnGrid makePlatformGrid()
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 6, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, x < 3 ? 0.0 : 0.20});
    }
  }
  grid.observe(points, {0.05, 0.25, 0.0});
  return grid;
}
}  // namespace

TEST(GroundColumn, RayClearingSupportingGroundVoxelDoesNotOpenAStep)
{
  auto grid = makePlatformGrid();
  ASSERT_EQ(grid.state(3, 2), ColumnState::kOccupied);
  ASSERT_TRUE(grid.ground(3, 2).has_value());
  // A grazing ray inside the platform surface voxel passes above the stored
  // return at z = 0.20 m: it does not show the surface is gone.
  grid.clearVoxel({0.35, 0.25, 0.225}, 0.22);
  grid.observeRayCell(3, 2);
  EXPECT_EQ(grid.state(3, 2), ColumnState::kOccupied);
  EXPECT_TRUE(grid.ground(3, 2).has_value());
}

TEST(GroundColumn, RayAtOrBelowStoredReturnClearsIt)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 6, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.0});
    }
  }
  points.push_back({0.55, 0.25, 0.24});
  grid.observe(points, {0.05, 0.25, 0.0});
  ASSERT_EQ(grid.state(5, 2), ColumnState::kOccupied);
  grid.clearVoxel({0.55, 0.25, 0.225}, 0.21);
  grid.observeRayCell(5, 2);
  EXPECT_EQ(grid.state(5, 2), ColumnState::kFree);
}

TEST(GroundColumn, RayClearingDynamicObstacleAboveGroundStillFrees)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 6, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.0});
    }
  }
  for (const double z : {0.15, 0.20, 0.25}) {points.push_back({0.55, 0.25, z});}
  grid.observe(points, {0.05, 0.25, 0.0});
  ASSERT_EQ(grid.state(5, 2), ColumnState::kOccupied);
  for (const double z : {0.15, 0.20, 0.25}) {grid.clearVoxel({0.55, 0.25, z}, z - 0.01);}
  grid.observeRayCell(5, 2);
  EXPECT_EQ(grid.state(5, 2), ColumnState::kFree);
}

TEST(GroundColumn, RayJustAboveLowObstacleReturnStillClearsIt)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 6, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 6; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.0});
    }
  }
  // Lowest face return of a removed box, 0.20 m above the ground of its column.
  points.push_back({0.55, 0.25, 0.20});
  grid.observe(points, {0.05, 0.25, 0.0});
  ASSERT_EQ(grid.state(5, 2), ColumnState::kOccupied);
  // Mid360 rays that reach this height are rare and nearly all pass a few
  // centimetres above the stored return; that must still remove an obstacle.
  grid.clearVoxel({0.55, 0.25, 0.225}, 0.22);
  grid.observeRayCell(5, 2);
  EXPECT_EQ(grid.state(5, 2), ColumnState::kFree);
}

TEST(GroundColumn, RayClearingRampGroundVoxelKeepsRampFree)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(30, 8, 0.1, 0.0, 0.0, config);
  std::vector<std::array<double, 3>> points;
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 8; ++y) {
      points.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, x * 0.03});
    }
  }
  grid.observe(points, {0.05, 0.35, 0.0});
  ASSERT_EQ(grid.state(25, 3), ColumnState::kFree);
  grid.clearVoxel({2.55, 0.35, 25 * 0.03 + 0.01}, 25 * 0.03 + 0.012);
  grid.observeRayCell(25, 3);
  EXPECT_EQ(grid.state(25, 3), ColumnState::kFree);
}

TEST(GroundColumn, ClearedGroundVoxelDoesNotLetFloatingPlateSeedGround)
{
  auto grid = makePlatformGrid();
  grid.clearVoxel({0.55, 0.25, 0.225}, 0.21);
  grid.observeRayCell(5, 2);
  // The platform interior still has no ground of its own below it.
  EXPECT_NE(grid.state(5, 2), ColumnState::kFree);
}

TEST(GroundColumn, InvalidGeometryIsRejectedBeforeAllocation)
{
  GroundColumnConfig config;
  EXPECT_THROW(ats_rog_map::GroundColumnGrid(-1, 10, 0.1, 0.0, 0.0, config),
    std::invalid_argument);
  EXPECT_THROW(ats_rog_map::GroundColumnGrid(10, 10, 0.0, 0.0, 0.0, config),
    std::invalid_argument);
}
