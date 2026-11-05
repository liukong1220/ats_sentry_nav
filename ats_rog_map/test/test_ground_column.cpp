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

TEST(GroundColumn, InvalidGeometryIsRejectedBeforeAllocation)
{
  GroundColumnConfig config;
  EXPECT_THROW(ats_rog_map::GroundColumnGrid(-1, 10, 0.1, 0.0, 0.0, config),
    std::invalid_argument);
  EXPECT_THROW(ats_rog_map::GroundColumnGrid(10, 10, 0.0, 0.0, 0.0, config),
    std::invalid_argument);
}
