// Copyright 2026

#include <cmath>

#include "gtest/gtest.h"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/path_geometry_preprocessor.hpp"

namespace
{

nav_msgs::msg::Path makePath(std::initializer_list<std::pair<double, double>> coordinates)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & coordinate : coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = coordinate.first;
    pose.pose.position.y = coordinate.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::OccupancyGrid makeGrid(int8_t value = 0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 40;
  grid.info.height = 40;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, value);
  return grid;
}

void setOccupied(nav_msgs::msg::OccupancyGrid & grid, double x, double y)
{
  const int gx = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
  const int gy = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
  ASSERT_GE(gx, 0);
  ASSERT_GE(gy, 0);
  ASSERT_LT(gx, static_cast<int>(grid.info.width));
  ASSERT_LT(gy, static_cast<int>(grid.info.height));
  grid.data[static_cast<std::size_t>(gy) * grid.info.width + gx] = 100;
}

minco_planner::FootprintSafetyChecker makeChecker()
{
  minco_planner::FootprintSafetyParams params;
  params.length = 0.05;
  params.width = 0.05;
  params.safety_margin = 0.0;
  params.unknown_is_obstacle = true;
  return minco_planner::FootprintSafetyChecker(params);
}

TEST(PathGeometryPreprocessor, StraightPathRemainsStraightWithRedundantCollinearPoints)
{
  minco_planner::PathGeometryPreprocessor preprocessor;
  const auto result = preprocessor.preprocess(makePath({
      {0.0, 0.0}, {0.0, 0.0}, {0.2, 0.001}, {0.4, -0.001}, {1.0, 0.0}}));

  ASSERT_EQ(result.waypoints.size(), 2U);
  EXPECT_NEAR(result.waypoints.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(result.waypoints.back().x(), 1.0, 1e-9);
  EXPECT_NEAR(result.waypoints.front().y(), 0.0, 1e-9);
  EXPECT_NEAR(result.waypoints.back().y(), 0.0, 1e-9);
  EXPECT_EQ(result.duplicate_points_removed, 1U);
  EXPECT_EQ(result.corner_waypoints.size(), 2U);
  EXPECT_FALSE(result.corner_waypoints.front());
  EXPECT_FALSE(result.corner_waypoints.back());
}

TEST(PathGeometryPreprocessor, BlockedShortcutPreservesCollisionFreeCorner)
{
  auto grid = makeGrid();
  setOccupied(grid, 0.5, 0.5);
  const auto checker = makeChecker();
  minco_planner::PathGeometryPreprocessor preprocessor;
  const auto result = preprocessor.preprocess(
    makePath({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}}), &grid, &checker);

  ASSERT_EQ(result.waypoints.size(), 3U);
  EXPECT_TRUE(result.corner_waypoints[1]);
  EXPECT_NEAR(result.waypoints[1].x(), 0.0, 1e-9);
  EXPECT_NEAR(result.waypoints[1].y(), 1.0, 1e-9);
}

TEST(PathGeometryPreprocessor, ShortcutRejectsUnknownOutsideAndSweptFootprintCollision)
{
  const auto checker = makeChecker();
  minco_planner::PathGeometryPreprocessor preprocessor;
  const auto corner_path = makePath({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}});

  auto unknown_grid = makeGrid(-1);
  const auto unknown = preprocessor.preprocess(corner_path, &unknown_grid, &checker);
  EXPECT_EQ(unknown.waypoints.size(), 3U);

  auto occupied = makeGrid();
  setOccupied(occupied, 0.5, 0.5);
  const auto blocked = preprocessor.preprocess(corner_path, &occupied, &checker);
  EXPECT_EQ(blocked.waypoints.size(), 3U);
}

}  // namespace
