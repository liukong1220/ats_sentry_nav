// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"

namespace
{

nav_msgs::msg::Path makePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & xy :
    {std::pair<double, double> {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path makeObstacleSkimmingPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & xy :
    {std::pair<double, double> {0.0, 0.0}, {2.0, 0.0}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

void populateOffsetObstacleEsdf(trajectory_optimizer::RcTraversabilityEsdfProvider & esdf)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = 80;
  grid.info.height = 80;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.data.assign(
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height), 0);
  for (unsigned int y = 23; y <= 32; ++y) {
    for (unsigned int x = 34; x <= 45; ++x) {
      grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
    }
  }

  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
}

double minimumDistance(
  const minco_planner::ReferenceTrajectory & trajectory,
  const trajectory_optimizer::RcTraversabilityEsdfProvider & esdf)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & point : trajectory.points) {
    minimum = std::min(minimum, esdf.getDistance(point.x, point.y));
  }
  return minimum;
}

TEST(MincoTrajectoryOptimizer, InterpolatesEndpointsWithZeroBoundaryVelocity)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.0;
  params.sample_spacing = 0.05;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  const auto trajectory = optimizer.optimize(makePath());

  ASSERT_GT(trajectory.points.size(), 10U);
  EXPECT_NEAR(trajectory.points.front().x, 0.0, 1e-8);
  EXPECT_NEAR(trajectory.points.front().y, 0.0, 1e-8);
  EXPECT_NEAR(trajectory.points.front().v, 0.0, 1e-8);
  EXPECT_NEAR(trajectory.points.back().x, 1.0, 1e-8);
  EXPECT_NEAR(trajectory.points.back().y, 1.0, 1e-8);
  EXPECT_NEAR(trajectory.points.back().v, 0.0, 1e-7);
  EXPECT_GT(trajectory.totalTime(), 0.0);
  EXPECT_GT(trajectory.totalLength(), 1.5);
}

TEST(MincoTrajectoryOptimizer, ProducesFiniteOmnidirectionalDerivatives)
{
  minco_planner::MincoTrajectoryOptimizer optimizer;
  const auto trajectory = optimizer.optimize(makePath());
  ASSERT_FALSE(trajectory.empty());
  for (const auto & point : trajectory.points) {
    EXPECT_TRUE(std::isfinite(point.vx));
    EXPECT_TRUE(std::isfinite(point.vy));
    EXPECT_TRUE(std::isfinite(point.ax));
    EXPECT_TRUE(std::isfinite(point.ay));
  }
}

TEST(MincoTrajectoryOptimizer, UsesEsdfGradientToIncreaseObstacleClearance)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.0;
  params.sample_spacing = 0.04;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  params.esdf_obstacle_clearance = 0.40;
  params.esdf_obstacle_max_iterations = 8;
  params.esdf_obstacle_control_point_spacing = 0.20;
  params.esdf_obstacle_max_step = 0.10;
  params.esdf_obstacle_max_deviation = 0.60;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  trajectory_optimizer::RcTraversabilityEsdfProvider esdf;
  populateOffsetObstacleEsdf(esdf);
  const auto baseline = optimizer.optimize(makeObstacleSkimmingPath());
  const auto optimized = optimizer.optimize(makeObstacleSkimmingPath(), &esdf);

  ASSERT_FALSE(baseline.empty());
  ASSERT_FALSE(optimized.empty());
  EXPECT_NEAR(optimized.points.front().x, 0.0, 1e-8);
  EXPECT_NEAR(optimized.points.front().y, 0.0, 1e-8);
  EXPECT_NEAR(optimized.points.back().x, 2.0, 1e-8);
  EXPECT_NEAR(optimized.points.back().y, 0.0, 1e-8);
  EXPECT_GT(minimumDistance(optimized, esdf), minimumDistance(baseline, esdf) + 0.08);
}

}  // namespace
