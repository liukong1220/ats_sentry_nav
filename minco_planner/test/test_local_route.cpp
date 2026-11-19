// Copyright 2026

#include <gtest/gtest.h>

#include "minco_planner/planning/local_route.hpp"

namespace
{

using minco_planner::GridSearch;
using minco_planner::GridSearchParams;
using minco_planner::FootprintSafetyChecker;
using minco_planner::FootprintSafetyParams;

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1F;
  grid.info.width = 80;
  grid.info.height = 52;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  return grid;
}

nav_msgs::msg::Path makeRoute(const nav_msgs::msg::OccupancyGrid & grid)
{
  nav_msgs::msg::Path path;
  path.header = grid.header;
  for (int x = 7; x <= 72; ++x) {
    auto & pose = path.poses.emplace_back();
    pose.header = grid.header;
    pose.pose.position.x = (x + 0.5) * grid.info.resolution;
    pose.pose.position.y = 25.5 * grid.info.resolution;
    pose.pose.orientation.w = 1.0;
  }
  return path;
}

FootprintSafetyChecker checker()
{
  FootprintSafetyParams params;
  params.length = 0.40;
  params.width = 0.38;
  params.safety_margin = 0.02;
  params.unknown_is_obstacle = true;
  return FootprintSafetyChecker(params);
}

GridSearch searcher()
{
  GridSearchParams params;
  params.unknown_is_obstacle = true;
  params.safe_distance = 0.35;
  params.min_safe_distance = 0.35;
  return GridSearch(params);
}

}  // namespace

TEST(LocalRoute, RejoinsAfterTheObstructionAndKeepsTheUnaffectedSuffix)
{
  auto grid = makeGrid();
  const auto original = makeRoute(grid);
  std::fill_n(grid.data.begin() + 23 * grid.info.width + 37, 6, -1);
  for (int y = 24; y <= 28; ++y) {
    std::fill_n(grid.data.begin() + y * grid.info.width + 37, 6, 100);
  }
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  auto search = searcher();
  const auto result = minco_planner::rejoinGlobalRoute(
    original, grid, field, 0.35, checker(), search);
  ASSERT_TRUE(result.route.success) << result.route.reason;
  ASSERT_TRUE(result.obstruction);
  ASSERT_TRUE(result.rejoin);
  EXPECT_LT(result.obstruction->first, result.obstruction->last);
  EXPECT_GE(result.rejoin->x - original.poses[result.obstruction->last].pose.position.x, 1.0);
  EXPECT_EQ(result.route.path.poses.front(), original.poses.front());
  EXPECT_EQ(result.route.path.poses.back(), original.poses.back());
  EXPECT_GT(result.route.length, minco_planner::traceGuide(original).totalLength());
  const auto suffix = [&](const nav_msgs::msg::Path & path) {
      const auto first = std::find_if(path.poses.begin(), path.poses.end(), [&](const auto & p) {
          return p.pose.position == *result.rejoin;
        });
      return std::vector<geometry_msgs::msg::PoseStamped>(first, path.poses.end());
    };
  EXPECT_EQ(suffix(original), suffix(result.route.path));
  const auto trace = minco_planner::traceGuide(result.route.path);
  EXPECT_FALSE(minco_planner::obstructedGuide(trace, grid, checker()));
  for (std::size_t i = 1; i < trace.points.size(); ++i) {
    EXPECT_LE(trace.points[i].s - trace.points[i - 1].s, 0.142);
  }
}

TEST(LocalRoute, UsesOnlyTheRemainingRouteWhenTheRobotHasAlreadyPassedAnObstacle)
{
  auto grid = makeGrid();
  const auto original = makeRoute(grid);
  grid.data[25 * grid.info.width + 14] = 100;
  const auto remaining = minco_planner::remainingGuide(original, original.poses[25], 0.5);
  ASSERT_LT(remaining.poses.size(), original.poses.size());
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  auto search = searcher();
  const auto result = minco_planner::rejoinGlobalRoute(
    remaining, grid, field, 0.35, checker(), search);
  ASSERT_TRUE(result.route.success);
  EXPECT_FALSE(result.obstruction);
  EXPECT_EQ(result.route.path, remaining);
  EXPECT_EQ(result.route.expanded_nodes, 0);
}

TEST(LocalRoute, OnlyNewBlockersOnTheGuideTriggerABypass)
{
  auto accepted = makeGrid();
  const auto original = makeRoute(accepted);
  accepted.data[27 * accepted.info.width + 22] = 100;
  auto current = accepted;
  current.data[8 * current.info.width + 35] = 100;
  const auto trace = minco_planner::traceGuide(original);
  const auto safety = checker();
  ASSERT_TRUE(minco_planner::obstructedGuide(trace, current, safety));
  EXPECT_FALSE(minco_planner::obstructedGuide(trace, current, safety, &accepted));

  current.data[25 * current.info.width + 45] = 100;
  const auto newly_blocked = minco_planner::obstructedGuide(trace, current, safety, &accepted);
  ASSERT_TRUE(newly_blocked);
  EXPECT_GT(newly_blocked->first, 28U);
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(current, {50, true});
  auto search = searcher();
  const auto result = minco_planner::rejoinGlobalRoute(
    original, current, field, 0.35, safety, search, std::nullopt, &accepted);
  ASSERT_TRUE(result.route.success) << result.route.reason;
  EXPECT_TRUE(result.rejoin);
  // The independent reference gate still sees the old obstacle on this geometric trace.
  EXPECT_TRUE(minco_planner::obstructedGuide(
    minco_planner::traceGuide(result.route.path), current, safety));
}

TEST(LocalRoute, GivesTheRotatedFootprintRoomOnTheNewBends)
{
  auto grid = makeGrid();
  const auto original = makeRoute(grid);
  for (int y = 23; y <= 28; ++y) {
    std::fill_n(grid.data.begin() + y * grid.info.width + 37, 6, 100);
  }
  auto footprint = checker().params();
  footprint.length = 0.581;
  footprint.width = 0.571;
  const FootprintSafetyChecker safety(footprint);
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  auto search = searcher();
  const auto result = minco_planner::rejoinGlobalRoute(
    original, grid, field, 0.35, safety, search);
  ASSERT_TRUE(result.route.success) << result.route.reason;
  ASSERT_TRUE(result.rejoin);
  EXPECT_EQ(result.route.clearance, 0.35);
  EXPECT_TRUE(safety.check(minco_planner::traceGuide(result.route.path), grid).safe);
}

TEST(LocalRoute, BlockedBarrierFailsWithinTheLocalBoxAndFullSearchAlsoFails)
{
  auto grid = makeGrid();
  const auto original = makeRoute(grid);
  for (std::uint32_t y = 0; y < grid.info.height; ++y) {
    grid.data[y * grid.info.width + 40] = 100;
  }
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  auto search = searcher();
  const auto result = minco_planner::rejoinGlobalRoute(
    original, grid, field, 0.35, checker(), search);
  ASSERT_TRUE(result.obstruction);
  EXPECT_FALSE(result.route.success);
  EXPECT_FALSE(result.rejoin);
  EXPECT_TRUE(result.route.path.poses.empty());
  EXPECT_LT(result.route.expanded_nodes, 45 * 26);
  const auto full = search.plan(grid, original.poses.front(), original.poses.back(), &field);
  EXPECT_FALSE(full.success);
  EXPECT_EQ(full.reason, "no path");
}
