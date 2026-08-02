// Copyright 2026

#include <gtest/gtest.h>

#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_jps.hpp"
#include "ats_rc_esdf/esdf/static_map_fusion.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 30;
  grid.info.height = 20;
  grid.info.resolution = 0.1F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    if (y >= 8 && y <= 11) {
      continue;
    }
    grid.data[y * grid.info.width + 14] = 100;
  }
  return grid;
}

geometry_msgs::msg::PoseStamped poseAt(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(GridJps, FindsPathThroughWallOpening)
{
  minco_planner::GridJps planner;
  const auto result = planner.plan(makeGrid(), poseAt(0.25, 0.25), poseAt(2.75, 1.75));
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GE(result.path.poses.size(), 2U);
  EXPECT_GT(result.length, 0.0);
  EXPECT_LT(result.expanded_nodes, 300);
}

TEST(GridJps, PreservesContinuousStartAndGoalPoses)
{
  const auto start = poseAt(0.21, 0.24);
  const auto goal = poseAt(2.79, 1.76);
  minco_planner::GridJps planner;
  const auto result = planner.plan(makeGrid(), start, goal);
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GE(result.path.poses.size(), 2U);
  EXPECT_NEAR(result.path.poses.front().pose.position.x, start.pose.position.x, 1e-9);
  EXPECT_NEAR(result.path.poses.front().pose.position.y, start.pose.position.y, 1e-9);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, goal.pose.position.x, 1e-9);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, goal.pose.position.y, 1e-9);
}

TEST(GridAstar, PreservesContinuousStartAndGoalPoses)
{
  const auto start = poseAt(0.21, 0.24);
  const auto goal = poseAt(2.79, 1.76);
  minco_planner::GridAstar planner;
  const auto result = planner.plan(makeGrid(), start, goal);
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GE(result.path.poses.size(), 2U);
  EXPECT_NEAR(result.path.poses.front().pose.position.x, start.pose.position.x, 1e-9);
  EXPECT_NEAR(result.path.poses.front().pose.position.y, start.pose.position.y, 1e-9);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, goal.pose.position.x, 1e-9);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, goal.pose.position.y, 1e-9);
}

TEST(GridJps, RejectsOccupiedGoal)
{
  minco_planner::GridJps planner;
  const auto result = planner.plan(makeGrid(), poseAt(0.25, 0.25), poseAt(1.45, 0.55));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal occupied");
}

TEST(GridJps, AppliesConfiguredSafeDistance)
{
  minco_planner::GridJpsParams params;
  params.safe_distance = 0.21;
  minco_planner::GridJps planner(params);
  const auto result = planner.plan(makeGrid(), poseAt(0.25, 0.25), poseAt(2.75, 1.75));
  EXPECT_FALSE(result.success);
}

TEST(GridJps, RejectsAPathBlockedByStaticMapAfterFusion)
{
  auto local_grid = makeGrid();
  local_grid.header.frame_id = "odom";
  local_grid.data.assign(local_grid.info.width * local_grid.info.height, 0);
  auto static_map = local_grid;
  static_map.header.frame_id = "map";
  for (unsigned int y = 0; y < static_map.info.height; ++y) {
    static_map.data[y * static_map.info.width + 14U] = 100;
  }
  geometry_msgs::msg::TransformStamped map_from_odom;
  map_from_odom.header.frame_id = "map";
  map_from_odom.child_frame_id = "odom";
  map_from_odom.transform.rotation.w = 1.0;

  nav_msgs::msg::OccupancyGrid planning_grid;
  ASSERT_TRUE(ats_rc_esdf::StaticMapFusion::buildPlanningGrid(
    local_grid, static_map, map_from_odom, ats_rc_esdf::StaticMapFusionParams {},
    planning_grid));
  EXPECT_EQ(planning_grid.data[10U * planning_grid.info.width + 14U], 100);

  minco_planner::GridJps planner;
  const auto result = planner.plan(
    planning_grid, poseAt(0.25, 0.25), poseAt(2.75, 1.75));
  EXPECT_FALSE(result.success);
}

}  // namespace
