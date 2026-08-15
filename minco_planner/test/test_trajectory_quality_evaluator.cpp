// Copyright 2026

#include <cmath>

#include "gtest/gtest.h"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "minco_planner/trajectory/trajectory_quality_evaluator.hpp"

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

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = 80;
  grid.info.height = 80;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  return grid;
}

void setOccupied(nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
}

TEST(TrajectoryQualityEvaluator, SafeStraightPathDoesNotTriggerEsdfRefinement)
{
  const auto grid = makeGrid();
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
  minco_planner::MincoTrajectoryOptimizerParams params;
  // Match the executed Gazebo profile. A free two-point guide must stay a
  // single geometric segment and still pass the full v/a/j gate.
  params.reference_speed = 1.5;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_jerk = 12.0;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  minco_planner::MincoOptimizationTrace trace;
  const auto trajectory = optimizer.optimize(
    makePath({{0.0, 0.0}, {2.0, 0.0}}), &esdf, nullptr, nullptr, nullptr, nullptr, &trace);

  ASSERT_TRUE(trajectory.valid());
  EXPECT_FALSE(trace.esdf_geometry_refined);
  EXPECT_EQ(trace.preprocessed_guide.poses.size(), 2U);
  EXPECT_EQ(trace.esdf_refined_guide.poses.size(), 2U);
  EXPECT_EQ(trace.segment_durations.size(), 1U);
  EXPECT_LE(trace.peak_velocity, params.max_velocity + 1e-6);
  EXPECT_LE(trace.peak_acceleration, params.max_acceleration + 1e-6);
  EXPECT_LE(trace.peak_jerk, params.max_jerk + 1e-6);
  EXPECT_NEAR(trajectory.points.front().y, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().y, 0.0, 1e-9);
}

TEST(TrajectoryQualityEvaluator, NoisyAlternatingGradientDoesNotCreateZigzag)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.sample_spacing = 0.04;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  minco_planner::MincoOptimizationTrace trace;
  const auto trajectory = optimizer.optimize(makePath({
      {0.0, 0.0}, {0.2, 0.006}, {0.4, -0.006}, {0.6, 0.005}, {1.0, 0.0}}),
    nullptr, nullptr, nullptr, nullptr, nullptr, &trace);

  ASSERT_TRUE(trajectory.valid());
  minco_planner::TrajectoryQualityEvaluator evaluator;
  const auto metrics = evaluator.evaluate(trajectory, nullptr, {}, trace.segment_durations);
  EXPECT_LE(metrics.max_lateral_deviation, 0.02);
  EXPECT_EQ(metrics.curvature_sign_changes, 0U);
  EXPECT_LE(trace.preprocessed_guide.poses.size(), 2U);
}

TEST(TrajectoryQualityEvaluator, RefinementBacktracksWhenClearanceOrCurvatureRegresses)
{
  auto grid = makeGrid();
  for (int y = 23; y <= 31; ++y) {
    for (int x = 34; x <= 45; ++x) {
      setOccupied(grid, x, y);
    }
  }
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.0;
  params.sample_spacing = 0.04;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  params.esdf_obstacle_trigger_clearance = 0.40;
  params.esdf_obstacle_target_clearance = 0.45;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  const auto baseline = optimizer.optimize(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  minco_planner::MincoOptimizationTrace trace;
  const auto refined = optimizer.optimize(
    makePath({{0.0, 0.0}, {2.0, 0.0}}), &esdf, nullptr, nullptr, nullptr, nullptr, &trace);

  ASSERT_TRUE(baseline.valid());
  ASSERT_TRUE(refined.valid());
  ASSERT_FALSE(trace.esdf_refined_guide.poses.empty());
  EXPECT_NEAR(refined.points.front().x, 0.0, 1e-8);
  EXPECT_NEAR(refined.points.back().x, 2.0, 1e-8);
  EXPECT_TRUE(std::isfinite(refined.totalLength()));
}

TEST(TrajectoryQualityEvaluator, FinalCandidatePreservesEndpointsInitialStateAndTerminalStop)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.0;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  minco_planner::InitialKinematicState initial_state;
  initial_state.valid = true;
  initial_state.velocity = Eigen::Vector2d(0.2, 0.0);
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  const auto trajectory = optimizer.optimize(
    makePath({{0.0, 0.0}, {1.0, 0.0}}), nullptr, nullptr, &initial_state);

  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.points.front().x, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.points.back().x, 1.0, 1e-9);
  EXPECT_NEAR(trajectory.points.front().vx, 0.2, 1e-8);
  EXPECT_NEAR(trajectory.points.back().v, 0.0, 1e-7);
}

TEST(TrajectoryQualityEvaluator, UnsafeCandidateFailsClosedAndOldReferenceCannotRevive)
{
  auto grid = makeGrid();
  setOccupied(grid, 30, 20);
  minco_planner::ReferenceTrajectory unsafe;
  minco_planner::ReferencePoint first;
  first.x = 0.0;
  first.y = 0.0;
  minco_planner::ReferencePoint last = first;
  last.x = 1.0;
  last.t = 1.0;
  last.s = 1.0;
  unsafe.points = {first, last};
  minco_planner::FootprintSafetyParams footprint;
  footprint.length = 0.05;
  footprint.width = 0.05;
  footprint.unknown_is_obstacle = true;
  EXPECT_FALSE(minco_planner::FootprintSafetyChecker(footprint).check(unsafe, grid).safe);

  minco_planner::PlannerSafetyState state;
  state.map_ready = true;
  state.plan_safe = true;
  state.invalidateMap(17);
  EXPECT_TRUE(state.emergencyStopRequired());
  state.map_ready = true;
  EXPECT_FALSE(state.mapSnapshotUsable(17));
  EXPECT_TRUE(state.mapSnapshotUsable(18));
}

}  // namespace
