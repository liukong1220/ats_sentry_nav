// Copyright 2026

#include <gtest/gtest.h>

#include "minco_planner/planning/grid_jps.hpp"

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

}  // namespace
