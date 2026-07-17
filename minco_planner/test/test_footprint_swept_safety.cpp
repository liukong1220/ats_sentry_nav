// Copyright 2026

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"

namespace
{

using minco_planner::FootprintSafetyChecker;
using minco_planner::FootprintSafetyParams;
using minco_planner::LocalCollisionRepair;
using minco_planner::LocalCollisionRepairParams;
using minco_planner::ReferencePoint;
using minco_planner::ReferenceTrajectory;

nav_msgs::msg::OccupancyGrid makeGrid(
  double resolution = 0.05, std::uint32_t width = 30, std::uint32_t height = 30,
  double origin_x = 0.0, double origin_y = 0.0, double origin_yaw = 0.0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.z = std::sin(0.5 * origin_yaw);
  grid.info.origin.orientation.w = std::cos(0.5 * origin_yaw);
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

void setCell(nav_msgs::msg::OccupancyGrid & grid, int x, int y, int8_t value)
{
  ASSERT_GE(x, 0);
  ASSERT_GE(y, 0);
  ASSERT_LT(x, static_cast<int>(grid.info.width));
  ASSERT_LT(y, static_cast<int>(grid.info.height));
  grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = value;
}

ReferenceTrajectory makeTrajectory(
  double start_x, double start_y, double start_yaw,
  double end_x, double end_y, double end_yaw)
{
  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  ReferencePoint start;
  start.x = start_x;
  start.y = start_y;
  start.yaw = start_yaw;
  ReferencePoint end = start;
  end.t = 1.0;
  end.s = std::hypot(end_x - start_x, end_y - start_y);
  end.x = end_x;
  end.y = end_y;
  end.yaw = end_yaw;
  trajectory.points = {start, end};
  return trajectory;
}

FootprintSafetyChecker makeChecker(bool unknown_is_obstacle = true)
{
  FootprintSafetyParams params;
  params.length = 0.01;
  params.width = 0.01;
  params.safety_margin = 0.0;
  params.unknown_is_obstacle = unknown_is_obstacle;
  params.swept_max_corner_step_cells = 0.5;
  return FootprintSafetyChecker(params);
}

bool hasSweptCollision(const minco_planner::FootprintSafetyResult & result)
{
  return std::any_of(
    result.collisions.begin(), result.collisions.end(),
    [](const minco_planner::CollisionSample & sample) {return sample.swept;});
}

std::pair<double, double> localToWorld(
  const nav_msgs::msg::OccupancyGrid & grid, double local_x, double local_y)
{
  const double yaw = 2.0 * std::atan2(
    grid.info.origin.orientation.z, grid.info.origin.orientation.w);
  return std::make_pair(
    grid.info.origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y,
    grid.info.origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y);
}

TEST(FootprintSweptSafety, DetectsFineWallBetweenSafeEndpointSamples)
{
  auto grid = makeGrid();
  setCell(grid, 8, 2, 100);
  const auto result =
    makeChecker().check(makeTrajectory(0.125, 0.125, 0.0, 0.625, 0.125, 0.0), grid);

  EXPECT_FALSE(result.safe);
  EXPECT_TRUE(hasSweptCollision(result));
  EXPECT_GT(result.swept_samples_checked, 0U);
}

TEST(FootprintSweptSafety, DetectsPureLateralAndDiagonalCornerMotion)
{
  auto lateral_grid = makeGrid();
  setCell(lateral_grid, 2, 8, 100);
  const auto lateral = makeChecker().check(
    makeTrajectory(0.125, 0.125, 0.0, 0.125, 0.625, 0.0), lateral_grid);
  EXPECT_FALSE(lateral.safe);
  EXPECT_TRUE(hasSweptCollision(lateral));

  auto diagonal_grid = makeGrid();
  setCell(diagonal_grid, 8, 8, 100);
  const auto diagonal = makeChecker().check(
    makeTrajectory(0.125, 0.125, 0.0, 0.625, 0.625, 0.0), diagonal_grid);
  EXPECT_FALSE(diagonal.safe);
  EXPECT_TRUE(hasSweptCollision(diagonal));
}

TEST(FootprintSweptSafety, DetectsPureRotationCornerSweep)
{
  auto grid = makeGrid(0.025, 80, 80);
  setCell(grid, 38, 34, 100);
  FootprintSafetyParams params;
  params.length = 0.40;
  params.width = 0.10;
  params.unknown_is_obstacle = true;
  params.swept_max_corner_step_cells = 0.5;
  FootprintSafetyChecker checker(params);

  const auto result = checker.check(
    makeTrajectory(
      0.75, 0.75, -0.7853981633974483,
      0.75, 0.75, 0.7853981633974483), grid);
  EXPECT_FALSE(result.safe);
  EXPECT_TRUE(hasSweptCollision(result));
}

TEST(FootprintSweptSafety, UsesShortestYawAcrossPiBoundary)
{
  FootprintSafetyParams params;
  params.length = 0.40;
  params.width = 0.10;
  params.unknown_is_obstacle = false;
  params.swept_max_corner_step_cells = 0.5;
  FootprintSafetyChecker checker(params);
  const auto result = checker.check(
    makeTrajectory(0.30, 0.30, 3.0, 0.30, 0.30, -3.0), makeGrid());

  EXPECT_TRUE(result.safe);
  EXPECT_GT(result.swept_samples_checked, 0U);
  EXPECT_LT(result.swept_samples_checked, 10U);
}

TEST(FootprintSweptSafety, RejectsUnknownAndOutsideMap)
{
  auto unknown_grid = makeGrid();
  setCell(unknown_grid, 8, 2, -1);
  EXPECT_FALSE(
    makeChecker(true).check(
      makeTrajectory(0.125, 0.125, 0.0, 0.625, 0.125, 0.0), unknown_grid).safe);

  EXPECT_FALSE(
    makeChecker(true).check(
      makeTrajectory(0.125, 0.125, 0.0, -0.125, 0.125, 0.0), makeGrid()).safe);
}

TEST(FootprintSweptSafety, HonorsTranslatedRotatedNonIntegralGridGeometry)
{
  auto grid = makeGrid(0.07, 20, 20, 1.3, -0.7, 0.31);
  setCell(grid, 4, 5, 100);
  const auto start = localToWorld(grid, 0.105, 0.385);
  const auto end = localToWorld(grid, 0.525, 0.385);
  const auto result = makeChecker().check(
    makeTrajectory(start.first, start.second, 0.0, end.first, end.second, 0.0), grid);

  EXPECT_FALSE(result.safe);
  EXPECT_TRUE(hasSweptCollision(result));
}

TEST(FootprintSweptSafety, RechecksRepairOutputWithSweptGeometry)
{
  auto grid = makeGrid();
  setCell(grid, 8, 2, 100);
  const auto checker = makeChecker();
  auto trajectory = makeTrajectory(0.425, 0.125, 0.0, 0.625, 0.125, 0.0);
  const auto initial = checker.check(trajectory, grid);
  ASSERT_FALSE(initial.safe);

  LocalCollisionRepairParams repair_params;
  repair_params.search_radius = 0.10;
  repair_params.unknown_is_obstacle = true;
  LocalCollisionRepair repair(repair_params);
  ASSERT_TRUE(repair.repair(trajectory, initial, grid));

  const auto final = checker.check(trajectory, grid);
  EXPECT_FALSE(final.safe);
  EXPECT_TRUE(hasSweptCollision(final));
}

}  // namespace
