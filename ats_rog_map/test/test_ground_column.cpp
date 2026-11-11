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

TEST(GroundColumn, SlopeHeadroomUsesCeilingXYNotLowestNeighbor)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  for (const double clearance : {0.239, 0.247, 0.257}) {
    ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
    std::vector<std::array<double, 3>> points;
    for (const double x : {-0.09, 0.01, 0.11, 0.21}) {
      for (const double y : {-0.09, 0.01, 0.11, 0.21}) {
        points.push_back({x, y, 0.3 * x});
      }
    }
    points.push_back({0.09, 0.05, 0.3 * 0.09 + clearance});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), clearance < 0.247 ? ColumnState::kOccupied : ColumnState::kFree)
      << clearance;
  }
}

TEST(GroundColumn, LowerReturnInSameVoxelCannotBeOverwrittenByHighReturn)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  for (const bool low_first : {true, false}) {
    ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
    std::vector<std::array<double, 3>> floor;
    for (const double x : {-0.09, 0.01, 0.11}) {
      for (const double y : {-0.09, 0.01, 0.11}) {floor.push_back({x, y, 0.0});}
    }
    grid.observe(floor, {0.0, 0.0, 0.0});
    grid.observe({{0.05, 0.05, low_first ? 0.251 : 0.29}}, {0.0, 0.0, 0.0});
    grid.observe({{0.05, 0.05, low_first ? 0.29 : 0.251}}, {0.0, 0.0, 0.0});
    // Both are safe until a low obstacle in this voxel is observed.
    grid.observe({{0.05, 0.05, 0.25}}, {0.0, 0.0, 0.0});
    // Another voxel contains a genuinely low point; a later high point in it
    // must not erase that obstacle without ray evidence.
    grid.observe({{0.05, 0.05, 0.239}, {0.05, 0.05, 0.249}}, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), ColumnState::kOccupied);
  }
}

TEST(GroundColumn, SameVoxelInteriorReturnCannotHideBelowSlopedCeilingExtrema)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  std::vector<std::array<double, 3>> points;
  for (const double x : {-0.09, 0.01, 0.11}) {
    for (const double y : {-0.09, 0.01, 0.11}) {points.push_back({x, y, 0.3 * x});}
  }
  points.push_back({0.01, 0.05, 0.255});
  points.push_back({0.09, 0.05, 0.290});
  // Neither the absolute minimum nor maximum, but only 0.239 m above its floor.
  points.push_back({0.08, 0.05, 0.263});
  grid.observe(points, {0.0, 0.0, 0.0});
  EXPECT_EQ(grid.state(5, 5), ColumnState::kOccupied);
}

TEST(GroundColumn, TwoAxisSlopeHeadroomAndWorldHeightAreInvariant)
{
  for (const double datum : {-0.6, 0.0, 1.5}) {
    for (const double clearance : {0.237, 0.247, 0.257}) {
      GroundColumnConfig config;
      config.robot_height = 0.242;
      config.top_margin = 0.005;
      ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
      std::vector<std::array<double, 3>> points;
      for (const double x : {-0.09, 0.01, 0.11}) {
        for (const double y : {-0.09, 0.01, 0.11}) {
          points.push_back({x, y, datum + 0.3 * x + 0.4 * y});
        }
      }
      points.push_back({0.09, 0.08, datum + 0.3 * 0.09 + 0.4 * 0.08 + clearance});
      grid.observe(points, {0.0, 0.0, datum});
      EXPECT_EQ(grid.state(5, 5), clearance < 0.247 ? ColumnState::kOccupied : ColumnState::kFree);
    }
  }
}

TEST(GroundColumn, RankDeficientOrOneSidedSupportCannotInventSafeHeadroom)
{
  for (const bool one_sided : {false, true}) {
    GroundColumnConfig config;
    config.robot_height = 0.242;
    config.top_margin = 0.005;
    ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
    std::vector<std::array<double, 3>> points;
    for (const double x : {-0.19, -0.09, 0.01}) {
      for (const double y : {-0.09, 0.01, 0.11}) {
        if (one_sided || y == 0.01) {points.push_back({x, y, 0.3 * x});}
      }
    }
    points.push_back({0.09, 0.05, 0.3 * 0.09 + 0.239});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_NE(grid.state(5, 5), ColumnState::kFree);
  }
}

TEST(GroundColumn, LowHangingReturnPersistsAcrossHighFramesUntilRayClear)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  std::vector<std::array<double, 3>> floor;
  for (const double x : {-0.09, 0.01, 0.11}) {
    for (const double y : {-0.09, 0.01, 0.11}) {floor.push_back({x, y, 0.0});}
  }
  grid.observe(floor, {0.0, 0.0, 0.0});
  grid.observe({{0.05, 0.05, 0.237}}, {0.0, 0.0, 0.0});
  for (int frame = 0; frame < 8; ++frame) {
    grid.observe({{0.05, 0.05, 0.249}}, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), ColumnState::kOccupied);
  }
  grid.clearVoxel({0.05, 0.05, 0.225}, 0.21);
  grid.observeRayCell(5, 5);
  EXPECT_EQ(grid.state(5, 5), ColumnState::kFree);
  grid.clear();
  EXPECT_EQ(grid.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, ExtremaDoNotDoubleTheObservedVoxelCount)
{
  GroundColumnConfig config;
  config.min_observed = 2;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  grid.observe({{0.01, 0.01, 0.0}, {0.01, 0.01, 0.01}}, {0.0, 0.0, 0.0});
  EXPECT_EQ(grid.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, ContradictedSupportingReturnCannotLeaveAnEmptyFreeColumn)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  grid.observe({{0.05, 0.05, 0.01}}, {0.0, 0.0, 0.0});
  ASSERT_EQ(grid.state(5, 5), ColumnState::kFree);
  EXPECT_TRUE(grid.clearVoxel({0.05, 0.05, 0.025}, 0.0));
  grid.observeRayCell(5, 5);
  EXPECT_EQ(grid.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, SparseFloorColumnUsesNeighborSupportWithoutCeilingReturns)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  std::vector<std::array<double, 3>> floor;
  for (const double x : {-0.09, 0.01, 0.11}) {
    for (const double y : {-0.09, 0.01, 0.11}) {
      if (x != 0.01 || y != 0.01) {floor.push_back({x, y, 0.3 * x});}
    }
  }
  grid.observe(floor, {0.0, 0.0, 0.0});
  // A ray crossed the column but no endpoint landed in it.
  grid.observeRayCell(5, 5);
  EXPECT_EQ(grid.state(5, 5), ColumnState::kFree);
  // A ceiling above the gate needs the column's own floor; neighbours cannot
  // establish support under a hanging plate.
  ats_rog_map::GroundColumnGrid plate(10, 10, 0.1, -0.5, -0.5, config);
  plate.observe(floor, {0.0, 0.0, 0.0});
  plate.observe({{0.05, 0.05, 0.40}}, {5.0, 5.0, 0.0});
  EXPECT_EQ(plate.state(5, 5), ColumnState::kUnknown);
  // Without any connected neighbour there is no support at all.
  ats_rog_map::GroundColumnGrid empty(10, 10, 0.1, -0.5, -0.5, config);
  empty.observeRayCell(5, 5);
  EXPECT_EQ(empty.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, NearbyFloorReturnsAcrossCellBoundaryStayConnected)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  // 2 mm apart in XY, 3 mm apart in z: LIO noise, not a step. Only the first
  // return is a seed; the second must join by connectivity. The high ceiling
  // in that column is free only when the column has its own connected floor.
  grid.observe({{0.099, 0.05, 0.0}, {0.101, 0.05, 0.003}, {0.15, 0.05, 0.40}},
    {0.05, 0.05, -0.048});
  EXPECT_EQ(grid.state(6, 5), ColumnState::kFree);
  // A real step between adjacent columns is still not connected.
  ats_rog_map::GroundColumnGrid step(10, 10, 0.1, -0.5, -0.5, config);
  step.observe({{0.05, 0.05, 0.0}, {0.15, 0.05, 0.12}, {0.15, 0.05, 0.50}},
    {0.05, 0.05, 0.0});
  EXPECT_NE(step.state(6, 5), ColumnState::kFree);
  // 2 cm apart in z over 2 mm XY is beyond slope plus noise: not certified.
  ats_rog_map::GroundColumnGrid torn(10, 10, 0.1, -0.5, -0.5, config);
  torn.observe({{0.099, 0.05, 0.0}, {0.101, 0.05, 0.02}, {0.15, 0.05, 0.40}},
    {0.05, 0.05, -0.048});
  EXPECT_EQ(torn.state(6, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, ReturnsBelowNeighborSupportWithoutOwnFloorStayUnknown)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  std::vector<std::array<double, 3>> floor;
  for (const double x : {-0.09, 0.01, 0.11}) {
    for (const double y : {-0.09, 0.01, 0.11}) {
      if (x != 0.01 || y != 0.01) {floor.push_back({x, y, 0.0});}
    }
  }
  grid.observe(floor, {0.0, 0.0, 0.0});
  // An unconnected lower surface is a drop, not floor.
  grid.observe({{0.05, 0.05, -0.20}}, {5.0, 5.0, 0.0});
  EXPECT_EQ(grid.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, RayAboveSlopedSurfaceAtItsOwnXYDoesNotRemoveSupport)
{
  GroundColumnConfig config;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  std::vector<std::array<double, 3>> points;
  for (const double x : {-0.09, 0.09, 0.19}) {
    for (const double y : {-0.075, 0.025, 0.125}) {points.push_back({x, y, 0.3 * x});}
  }
  grid.observe(points, {0.0, 0.0, 0.0});
  ASSERT_TRUE(grid.ground(5, 5).has_value());
  // 7 mm above the plane at its own XY, but below the stored support at x=0.09.
  EXPECT_FALSE(grid.clearVoxel({0.05, 0.05, 0.025}, {0.01, 0.025, 0.010}));
  EXPECT_TRUE(grid.ground(5, 5).has_value());
  // Clearly below any surface of bounded slope through the stored return.
  EXPECT_TRUE(grid.clearVoxel({0.05, 0.05, 0.025}, {0.01, 0.025, -0.08}));
}

TEST(GroundColumn, LaterLowFloorReturnCannotErasePreviouslyMeasuredSupportHeight)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
  for (const double height : {0.0, -0.02}) {
    std::vector<std::array<double, 3>> points;
    for (const double x : {-0.09, 0.01, 0.11}) {
      for (const double y : {-0.09, 0.01, 0.11}) {points.push_back({x, y, height});}
    }
    points.push_back({0.05, 0.05, 0.239});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_NE(grid.state(5, 5), ColumnState::kFree);
  }
}
