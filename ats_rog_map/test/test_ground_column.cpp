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

TEST(GroundColumn, CellCentreDistanceCannotConnectAStepAndSeedTheFarPlate)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.007;
  ats_rog_map::GroundColumnGrid grid(12, 4, 0.1, 0.0, 0.0, config);
  // The 6 cm rise occurs across 2 mm at a cell boundary, not over the 10 cm
  // centre spacing. Treating it as a ramp seeds the whole elevated surface:
  // far from the real floor its false support then passes all local bounds.
  std::vector<std::array<double, 3>> points{{0.099, 0.15, 0.0}};
  for (int x = 1; x < 10; ++x) {
    points.push_back({x * 0.1 + 0.001, 0.15, 0.06});
    points.push_back({x * 0.1 + 0.001, 0.15, 0.36});
  }
  // Only the real ground can seed. The raised returns are outside the seed's
  // height tolerance even before they are outside its spatial radius.
  grid.observe(points, {0.05, 0.15, -0.048});
  EXPECT_EQ(grid.state(0, 1), ColumnState::kFree);
  EXPECT_EQ(grid.state(8, 1), ColumnState::kUnknown);
  EXPECT_FALSE(grid.ground(8, 1).has_value());
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

TEST(GroundColumn, ColumnsWithoutEndpointOrRayStayUnknownDespitePlanarNeighbors)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
  std::vector<std::array<double, 3>> floor;
  // A planar tilted floor with a 3 x 3 hole of columns that never had an endpoint.
  for (int ix = 0; ix < 12; ++ix) {
    for (int iy = 0; iy < 12; ++iy) {
      if (ix >= 4 && ix <= 6 && iy >= 4 && iy <= 6) {continue;}
      const double x = -0.6 + (ix + 0.5) * 0.1;
      const double y = -0.6 + (iy + 0.5) * 0.1;
      floor.push_back({x, y, 0.1 * x});
    }
  }
  grid.observe(floor, {0.0, 0.0, 0.0});
  for (int ix = 4; ix <= 6; ++ix) {
    for (int iy = 4; iy <= 6; ++iy) {
      // Neighbour fit must not manufacture free for a never-observed column.
      EXPECT_EQ(grid.state(ix, iy), ColumnState::kUnknown);
    }
  }
  EXPECT_EQ(grid.state(2, 2), ColumnState::kFree);
  // Only an explicit ray observation may turn such a column free.
  grid.observeRayCell(4, 5);
  EXPECT_EQ(grid.state(4, 5), ColumnState::kFree);
  EXPECT_EQ(grid.state(4, 6), ColumnState::kUnknown);
  // The hole's centre has no connected support within its neighbourhood at all.
  grid.observeRayCell(5, 5);
  EXPECT_EQ(grid.state(5, 5), ColumnState::kUnknown);
}

TEST(GroundColumn, ElevatedSupportCandidateNeverMakesAHangingReturnFreeFloor)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.007;
  for (const double lift : {0.04, 0.098, 0.12, 0.278, 0.28}) {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    std::vector<std::array<double, 3>> floor;
    for (int ix = 0; ix < 12; ++ix) {
      for (int iy = 0; iy < 12; ++iy) {
        if (ix == 5 && iy == 5) {continue;}
        floor.push_back({-0.6 + (ix + 0.5) * 0.1, -0.6 + (iy + 0.5) * 0.1, 0.0});
      }
    }
    // The centre column holds only a return lift above the true floor (a hanging
    // underside seen from outside) and a plate 0.30 m above that return.
    // Centre of the omitted (5, 5) cell; (0, 0) belongs to (6, 6) and would
    // leave the asserted column unobserved, making this regression vacuous.
    floor.push_back({-0.05, -0.05, lift});
    floor.push_back({-0.05, -0.05, lift + 0.30});
    grid.observe(floor, {0.0, 0.0, 0.0});
    // Whatever the support identity, a column whose lowest return is above the
    // measured floor and whose headroom to the true floor is unproven is not free.
    const auto state = grid.state(5, 5);
    const auto ground = grid.ground(5, 5);
    if (lift > 0.06 + 1e-9) {
      EXPECT_NE(state, ColumnState::kFree) << "lift " << lift;
    }
    if (ground) {
      // The reported support is never above the elevated candidate.
      EXPECT_LE(*ground, lift + 1e-9);
    }
  }
}

TEST(GroundColumn, PlanarSupportIntervalDoesNotWidenByTheSlopeConeOverTheCeilingVoxel)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // A plane is linear, so its extrema over the ceiling voxel sit at the corners:
  // headroom 0.250 clears the 0.247 gate and 0.238 does not, with no allowance
  // for slope between measured planes that were fitted exactly.
  for (const auto & [ceiling, expected] : {
      std::pair<double, ColumnState>{0.250, ColumnState::kFree},
      std::pair<double, ColumnState>{0.238, ColumnState::kOccupied}})
  {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    std::vector<std::array<double, 3>> points;
    for (int ix = 0; ix < 12; ++ix) {
      for (int iy = 0; iy < 12; ++iy) {
        points.push_back({-0.6 + (ix + 0.5) * 0.1, -0.6 + (iy + 0.5) * 0.1, 0.0});
      }
    }
    // Ceiling returns spread over the whole centre cell (x in [-0.1, 0]).
    points.push_back({-0.095, -0.05, ceiling});
    points.push_back({-0.005, -0.05, ceiling});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), expected) << "ceiling " << ceiling;
  }
}

TEST(GroundColumn, JitteredFloorGatesOnTheFittedPlaneNotItsResidualBand)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // Floor returns scatter +-0.012 m around a plane. The gate compares the lowest
  // ceiling return with the fitted plane height, so a 0.250 m headroom clears the
  // 0.247 m gate and 0.240 m does not; the +-0.012 m band decides only that a
  // plane exists. The previous upper-bound rule rejected 0.250 m.
  for (const auto & [ceiling, expected] : {
      std::pair<double, ColumnState>{0.262, ColumnState::kFree},
      std::pair<double, ColumnState>{0.252, ColumnState::kFree},
      std::pair<double, ColumnState>{0.240, ColumnState::kOccupied},
      std::pair<double, ColumnState>{0.230, ColumnState::kOccupied}})
  {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    std::vector<std::array<double, 3>> points;
    for (int ix = 0; ix < 12; ++ix) {
      for (int iy = 0; iy < 12; ++iy) {
        const double jitter = ((ix + 2 * iy) % 3 - 1) * 0.012;
        points.push_back({-0.6 + (ix + 0.5) * 0.1, -0.6 + (iy + 0.5) * 0.1, jitter});
      }
    }
    points.push_back({-0.05, -0.05, ceiling});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), expected) << "ceiling " << ceiling;
    const auto headroom = grid.ceilingHeadroom(5, 5);
    ASSERT_TRUE(headroom.has_value());
    EXPECT_NEAR(headroom->headroom, ceiling - headroom->support_estimate, 1e-9);
    EXPECT_LE(headroom->support_estimate, headroom->support_upper + 1e-12);
  }
}

TEST(GroundColumn, ScatterBeyondPlaneResidualKeepsTheConservativeUpperBound)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // Scatter of +-0.03 m exceeds plane_residual_max, so no plane is trusted. The
  // gate then uses the slope-bound upper bound (tightest Lipschitz bound over the
  // measured highs), which equals the support interval's upper end; the fitted
  // height is never substituted. Here the nearest return is -0.03 m.
  for (const auto & [ceiling, expected] : {
      std::pair<double, ColumnState>{0.230, ColumnState::kFree},
      std::pair<double, ColumnState>{0.210, ColumnState::kOccupied}})
  {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    std::vector<std::array<double, 3>> points;
    for (int ix = 0; ix < 12; ++ix) {
      for (int iy = 0; iy < 12; ++iy) {
        const double jitter = ((ix + 2 * iy) % 3 - 1) * 0.03;
        points.push_back({-0.6 + (ix + 0.5) * 0.1, -0.6 + (iy + 0.5) * 0.1, jitter});
      }
    }
    points.push_back({-0.05, -0.05, ceiling});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), expected) << "ceiling " << ceiling;
    const auto headroom = grid.ceilingHeadroom(5, 5);
    ASSERT_TRUE(headroom.has_value());
    EXPECT_DOUBLE_EQ(headroom->support_estimate, headroom->support_upper);
  }
}

TEST(GroundColumn, StepBeyondPlaneResidualNeverBecomesAFloorPlane)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // A 0.06 m step across the neighbourhood must not be fitted away: the low side
  // keeps its own interval, so a 0.26 m ceiling over it stays occupied.
  ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
  std::vector<std::array<double, 3>> points;
  for (int ix = 0; ix < 12; ++ix) {
    for (int iy = 0; iy < 12; ++iy) {
      points.push_back({-0.6 + (ix + 0.5) * 0.1, -0.6 + (iy + 0.5) * 0.1, ix >= 6 ? 0.06 : 0.0});
    }
  }
  points.push_back({-0.05, -0.05, 0.26});
  grid.observe(points, {0.0, 0.0, 0.0});
  EXPECT_NE(grid.state(5, 5), ColumnState::kFree);
}

namespace
{
// Flat floor plus a flat board of nominal height `board` whose returns scatter
// +-0.006 m, on a 12 x 12 grid of 0.1 m columns around the origin.
std::vector<std::array<double, 3>> boardScene(double board, bool full_board = true)
{
  std::vector<std::array<double, 3>> points;
  for (int ix = 0; ix < 12; ++ix) {
    for (int iy = 0; iy < 12; ++iy) {
      const double cx = -0.6 + (ix + 0.5) * 0.1, cy = -0.6 + (iy + 0.5) * 0.1;
      points.push_back({cx, cy, ((ix * 3 + iy) % 3 - 1) * 0.002});
      if (!full_board && !(ix == 5 && iy == 5)) {continue;}
      points.push_back({cx - 0.02, cy, board + ((ix * 7 + iy * 3) % 5 - 2) * 0.003});
      points.push_back({cx + 0.02, cy + 0.02, board + ((ix * 5 + iy * 11) % 5 - 2) * 0.003});
    }
  }
  return points;
}
}  // namespace

TEST(GroundColumn, BoardPlaneClearsTheGateWhereTheLowestNoisyReturnWouldNot)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  GroundColumnConfig legacy = config;
  legacy.ceiling_plane_enabled = false;
  legacy.floor_plane_enabled = false;
  // Nominal 0.252 m board, lowest noisy return about 0.246 m: below the 0.247 m gate.
  for (const auto & [board, expected, legacy_expected] : {
      std::tuple<double, ColumnState, ColumnState>{0.252, ColumnState::kFree, ColumnState::kOccupied},
      std::tuple<double, ColumnState, ColumnState>{0.240, ColumnState::kOccupied, ColumnState::kOccupied}})
  {
    ats_rog_map::GroundColumnGrid with_plane(12, 12, 0.1, -0.6, -0.6, config);
    ats_rog_map::GroundColumnGrid without(12, 12, 0.1, -0.6, -0.6, legacy);
    with_plane.observe(boardScene(board), {0.0, 0.0, 0.0});
    without.observe(boardScene(board), {0.0, 0.0, 0.0});
    EXPECT_EQ(with_plane.state(5, 5), expected) << "board " << board;
    EXPECT_EQ(without.state(5, 5), legacy_expected) << "board " << board;
  }
}

TEST(GroundColumn, ReturnBelowTheBoardBandStaysAnObstacle)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // A board 0.30 m up is free, but a protrusion hanging below it, in the same
  // column, is a real obstacle and must not be averaged into the board.
  for (const double hanging : {0.20, 0.228}) {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    auto points = boardScene(0.30);
    points.push_back({-0.05, -0.05, hanging});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(5, 5), ColumnState::kOccupied) << "hanging " << hanging;
  }
  ats_rog_map::GroundColumnGrid clean(12, 12, 0.1, -0.6, -0.6, config);
  clean.observe(boardScene(0.30), {0.0, 0.0, 0.0});
  EXPECT_EQ(clean.state(5, 5), ColumnState::kFree);
}

TEST(GroundColumn, SmallCeilingPatchIsNotTreatedAsABoard)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // Only one column has ceiling returns: too few points and too small an extent
  // for a plane, so the lowest return decides (0.246 m < 0.247 m gate).
  ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
  grid.observe(boardScene(0.252, false), {0.0, 0.0, 0.0});
  EXPECT_NE(grid.state(5, 5), ColumnState::kFree);
}

TEST(GroundColumn, TiltedBoardUsesItsOwnHeightPerColumn)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.005;
  // The board descends along x (slope -0.05 from 0.30 at x = 0): the plane is
  // read at each column's own XY.
  for (const auto & [shift, expected] : {
      std::pair<double, ColumnState>{0.0, ColumnState::kFree},
      std::pair<double, ColumnState>{-0.06, ColumnState::kOccupied}})
  {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    auto points = boardScene(0.30);
    for (auto & p : points) {
      if (p[2] > 0.1) {p[2] += -0.05 * p[0] + shift;}
    }
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(8, 5), expected) << "shift " << shift;  // x 0.25: board 0.2875 or 0.2275
  }
}

TEST(GroundColumn, DenseSlopedSupportKeepsHeightPairedWithMeasuredXY)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.007;
  for (const double clearance : {0.249, 0.239}) {
    ats_rog_map::GroundColumnGrid grid(12, 12, 0.1, -0.6, -0.6, config);
    std::vector<std::array<double, 3>> points;
    for (int ix = 0; ix < 12; ++ix) {
      for (int iy = 0; iy < 12; ++iy) {
        for (const double dx : {0.01, 0.04, 0.07, 0.09}) {
          for (const double dy : {0.01, 0.04, 0.07, 0.09}) {
            const double x = -0.6 + ix * 0.1 + dx;
            const double y = -0.6 + iy * 0.1 + dy;
            points.push_back({x, y, 0.3 * x});
          }
        }
      }
    }
    // Same plane, more observed XY within each cell: density must not invent
    // off-plane combinations of the lowest height and highest-X position.
    points.push_back({0.05, 0.05, 0.3 * 0.05 + clearance});
    grid.observe(points, {0.0, 0.0, 0.0});
    EXPECT_EQ(grid.state(6, 6), clearance == 0.249 ?
      ColumnState::kFree : ColumnState::kOccupied) << clearance;
  }
}

TEST(GroundColumn, SupportFitIsInvalidatedByNewHigherFloorAndRayRemoval)
{
  GroundColumnConfig config;
  config.robot_height = 0.242;
  config.top_margin = 0.007;
  for (const bool remove_support : {false, true}) {
    ats_rog_map::GroundColumnGrid grid(10, 10, 0.1, -0.5, -0.5, config);
    std::vector<std::array<double, 3>> floor;
    for (const double x : {-0.09, 0.01, 0.11}) {
      for (const double y : {-0.09, 0.01, 0.11}) {floor.push_back({x, y, 0.0});}
    }
    grid.observe(floor, {0.0, 0.0, 0.0});
    grid.observe({{0.09, 0.05, 0.260}}, {0.0, 0.0, 0.0});
    ASSERT_EQ(grid.state(5, 5), ColumnState::kFree);
    if (remove_support) {
      for (const auto & p : floor) {
        if (p[0] != 0.01 || p[1] != 0.01) {EXPECT_TRUE(grid.clearVoxel(p, -0.10));}
      }
    } else {
      for (auto & p : floor) {p[2] = 0.02;}
      grid.observe(floor, {0.0, 0.0, 0.0});
    }
    // A formerly valid plane cannot survive changed support evidence.
    EXPECT_NE(grid.state(5, 5), ColumnState::kFree);
  }
}
