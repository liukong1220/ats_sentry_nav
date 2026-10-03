// Copyright 2026

#include <algorithm>
#include <cmath>

#include "gtest/gtest.h"
#include "minco_planner/planning/grid_clearance.hpp"
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

TEST(PathGeometryPreprocessor, InnerFilletReplacesSharpCornerWithoutOutsidePoints)
{
  minco_planner::PathGeometryPreprocessorParams params;
  params.fillet_radius = 0.55;
  params.fillet_arc_samples = 1;
  params.footprint_aware_shortcut_enabled = false;
  minco_planner::PathGeometryPreprocessor preprocessor(params);
  const auto result = preprocessor.preprocess(makePath({
      {0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}}));

  ASSERT_GT(result.waypoints.size(), 3U);
  bool kept_sharp_corner = false;
  for (const auto & waypoint : result.waypoints) {
    EXPECT_GE(waypoint.x(), -1e-6);
    EXPECT_LE(waypoint.y(), 2.0 + 1e-6);
    EXPECT_LE(waypoint.x(), 2.0 + 1e-6);
    EXPECT_GE(waypoint.y(), -1e-6);
    if (std::abs(waypoint.x()) <= 1e-6 && std::abs(waypoint.y() - 2.0) <= 1e-6) {
      kept_sharp_corner = true;
    }
  }
  EXPECT_FALSE(kept_sharp_corner);
}

TEST(PathGeometryPreprocessor, InnerFilletKeepsCornerWhenInnerArcHitsOccupied)
{
  auto grid = makeGrid();
  for (double x = 0.10; x <= 0.50; x += 0.10) {
    for (double y = 1.50; y <= 1.90; y += 0.10) {
      setOccupied(grid, x, y);
    }
  }
  const auto checker = makeChecker();
  minco_planner::PathGeometryPreprocessorParams params;
  params.fillet_radius = 0.55;
  params.fillet_arc_samples = 3;
  params.footprint_aware_shortcut_enabled = false;
  minco_planner::PathGeometryPreprocessor preprocessor(params);
  const auto result = preprocessor.preprocess(
    makePath({{0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}}), &grid, &checker);

  ASSERT_EQ(result.waypoints.size(), 3U);
  EXPECT_NEAR(result.waypoints[1].x(), 0.0, 1e-9);
  EXPECT_NEAR(result.waypoints[1].y(), 2.0, 1e-9);
}

}  // namespace

namespace
{
// 3 m x 3 m 栅格中一个从北墙伸出的凸角（x=0.5, y 0.1..1.9 m 外的列被占据），
// 居中路径从左侧 (0.05, 0.25) 绕到右侧 (1.05, 0.25)，在 y=-0.35 处绕过凸角底端。
nav_msgs::msg::OccupancyGrid makeConvexCornerGrid()
{
  auto grid = makeGrid();
  for (double y = 0.15; y <= 2.85; y += 0.1) {
    setOccupied(grid, 0.55, y);
  }
  return grid;
}

nav_msgs::msg::Path makeDetourPath()
{
  return makePath({
      {0.05, 0.25}, {0.05, 0.05}, {0.05, -0.15}, {0.15, -0.35}, {0.35, -0.45}, {0.55, -0.45},
      {0.75, -0.45}, {0.95, -0.35}, {1.05, -0.15}, {1.05, 0.05}, {1.05, 0.25}});
}

double minimumGridClearance(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<Eigen::Vector2d> & points)
{
  minco_planner::GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = 100;
  const auto squared = minco_planner::computeBlockedSquaredDistanceCells(grid, policy);
  double minimum = 1e9;
  for (std::size_t index = 1; index < points.size(); ++index) {
    for (int sample = 0; sample <= 20; ++sample) {
      const Eigen::Vector2d point =
        points[index - 1] + (points[index] - points[index - 1]) * (sample / 20.0);
      minimum = std::min(
        minimum, minco_planner::blockedDistanceAt(grid, squared, point.x(), point.y()));
    }
  }
  return minimum;
}

minco_planner::FootprintSafetyChecker makeSmallChecker()
{
  minco_planner::FootprintSafetyParams params;
  params.length = 0.2;
  params.width = 0.2;
  params.safety_margin = 0.0;
  params.obstacle_value_threshold = 100;
  params.unknown_is_obstacle = true;
  return minco_planner::FootprintSafetyChecker(params);
}
}  // namespace

TEST(PathGeometryPreprocessor, ShortcutClearancePreservationKeepsDetourOffCorner)
{
  const auto grid = makeConvexCornerGrid();
  const auto checker = makeSmallChecker();
  const auto raw = makeDetourPath();

  minco_planner::PathGeometryPreprocessorParams params;
  const auto taut = minco_planner::PathGeometryPreprocessor(params).preprocess(raw, &grid, &checker);
  params.shortcut_min_clearance = 0.5;
  const auto kept = minco_planner::PathGeometryPreprocessor(params).preprocess(raw, &grid, &checker);

  const double taut_clearance = minimumGridClearance(grid, taut.waypoints);
  const double kept_clearance = minimumGridClearance(grid, kept.waypoints);
  const double raw_clearance = minimumGridClearance(
    grid, minco_planner::PathGeometryPreprocessor(
      minco_planner::PathGeometryPreprocessorParams()).preprocess(raw).waypoints);
  // 不保持净空时，最远可行捷径把路径拉向凸角；保持后不低于原路径净空（容差 1/4 格）。
  EXPECT_LT(taut_clearance, raw_clearance - 0.05);
  EXPECT_GE(kept_clearance, raw_clearance - 0.025 - 1e-9);
  // 仍然做了化简，没有退化成原始点列。
  EXPECT_LT(kept.waypoints.size(), raw.poses.size());
}

TEST(PathGeometryPreprocessor, ShortcutClearanceDisabledByDefault)
{
  const auto grid = makeConvexCornerGrid();
  const auto checker = makeSmallChecker();
  const auto raw = makeDetourPath();
  minco_planner::PathGeometryPreprocessorParams defaults;
  minco_planner::PathGeometryPreprocessorParams explicit_off;
  explicit_off.shortcut_min_clearance = 0.0;
  const auto a = minco_planner::PathGeometryPreprocessor(defaults).preprocess(raw, &grid, &checker);
  const auto b = minco_planner::PathGeometryPreprocessor(explicit_off).preprocess(raw, &grid, &checker);
  ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
  for (std::size_t index = 0; index < a.waypoints.size(); ++index) {
    EXPECT_NEAR((a.waypoints[index] - b.waypoints[index]).norm(), 0.0, 1e-12);
  }
}
