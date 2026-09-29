// Copyright 2026

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "minco_planner/safety/footprint_samples.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

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

nav_msgs::msg::Path makeCoupledLongPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & xy :
    {std::pair<double, double> {0.0, 0.0}, {5.0, -5.0}, {6.5, -6.0},
      {6.7, -5.8}, {6.9, -5.9}, {7.1, -5.6}, {7.4, -5.7},
      {7.6, -5.3}, {7.9, -5.2}, {8.2, -4.7}, {8.4, -4.8},
      {8.8, -4.1}, {9.0, -4.0}, {9.3, -3.2}, {9.5, -3.1},
      {9.8, -2.1}, {10.0, -2.0}, {10.5, 0.3}})
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

void populateOffsetObstacleEsdf(ats_rc_esdf::RcTraversabilityEsdfProvider & esdf)
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

void populateFootprintEdgeObstacleEsdf(ats_rc_esdf::RcTraversabilityEsdfProvider & esdf)
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
  // The centre line at y=0 retains more than 0.40 m clearance, while a body
  // with +0.20 m lateral extent approaches this obstacle below that threshold.
  for (unsigned int y = 30; y <= 38; ++y) {
    for (unsigned int x = 34; x <= 45; ++x) {
      grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
    }
  }

  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
}

double minimumDistance(
  const minco_planner::ReferenceTrajectory & trajectory,
  const ats_rc_esdf::RcTraversabilityEsdfProvider & esdf)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & point : trajectory.points) {
    minimum = std::min(minimum, esdf.getDistance(point.x, point.y));
  }
  return minimum;
}

double minimumFootprintDistance(
  const minco_planner::ReferenceTrajectory & trajectory,
  const ats_rc_esdf::RcTraversabilityEsdfProvider & esdf,
  double length,
  double width,
  double safety_margin,
  double sample_spacing)
{
  const auto samples = minco_planner::makeRectangularFootprintSamples(
    length, width, safety_margin, sample_spacing);
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & point : trajectory.points) {
    const double cos_yaw = std::cos(point.yaw);
    const double sin_yaw = std::sin(point.yaw);
    for (const auto & sample : samples) {
      const double x = point.x + cos_yaw * sample.x() - sin_yaw * sample.y();
      const double y = point.y + sin_yaw * sample.x() + cos_yaw * sample.y();
      minimum = std::min(minimum, esdf.getDistance(x, y));
    }
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

// 联合优化带软速度/加速度罚，是否还需要整体时间缩放取决于残差；这里只验证
// 最终轨迹闭合耦合的动力学上限，不再要求必然走整体缩放分支。
TEST(MincoTrajectoryOptimizer, CoupledLongPathClosesDynamicLimits)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.5;
  params.sample_spacing = 0.02;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_jerk = 12.0;
  params.max_time_scaling_iterations = 0;
  params.time_scaling_factor = 1.25;
  params.esdf_obstacle_optimization_enabled = false;
  params.geometry_preprocessor.footprint_aware_shortcut_enabled = false;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  minco_planner::MincoOptimizationTrace trace;

  const auto trajectory = optimizer.optimize(
    makeCoupledLongPath(), nullptr, nullptr, nullptr, nullptr, nullptr, &trace);

  ASSERT_FALSE(trajectory.empty()) << trace.failure_reason;
  EXPECT_FALSE(trace.local_time_scaled);
  EXPECT_FALSE(trace.joint_termination.empty());
  EXPECT_LE(trace.peak_velocity, params.max_velocity + 1e-6);
  EXPECT_LE(trace.peak_acceleration, params.max_acceleration + 1e-6);
  EXPECT_LE(trace.peak_jerk, params.max_jerk + 1e-6);
}

TEST(MincoTrajectoryOptimizer, MovingStraightReplanPreservesBoundaryAndClosesJerk)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.max_jerk = 12.0;
  params.guide_control_point_spacing = 0.30;
  params.esdf_obstacle_optimization_enabled = false;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  auto path = makeObstacleSkimmingPath();
  path.poses.back().pose.position.x = 0.9;
  minco_planner::InitialKinematicState initial_state;
  initial_state.valid = true;
  initial_state.velocity = Eigen::Vector2d(0.7, 0.0);
  minco_planner::MincoOptimizationTrace trace;
  const auto trajectory = optimizer.optimize(
    path, nullptr, nullptr, &initial_state, nullptr, nullptr, &trace);
  ASSERT_FALSE(trajectory.empty()) << trace.failure_reason << ": jerk=" << trace.peak_jerk;
  EXPECT_NEAR(trajectory.points.front().vx, 0.7, 1e-8);
  EXPECT_NEAR(trajectory.points.front().vy, 0.0, 1e-8);
  EXPECT_NEAR(trajectory.points.back().x, 0.9, 1e-8);
  EXPECT_NEAR(trajectory.points.back().v, 0.0, 1e-8);
  EXPECT_LE(trace.peak_velocity, params.max_velocity + 1e-6);
  EXPECT_LE(trace.peak_acceleration, params.max_acceleration + 1e-6);
  EXPECT_LE(trace.peak_jerk, params.max_jerk + 1e-6);
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
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
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

TEST(MincoTrajectoryOptimizer, UsesYawAwareFootprintToIncreaseEdgeClearance)
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
  params.esdf_footprint_optimization_enabled = true;
  params.esdf_footprint_clearance = 0.40;
  params.esdf_footprint_sample_spacing = 0.05;
  params.footprint_length = 0.60;
  params.footprint_width = 0.40;
  params.footprint_safety_margin = 0.0;
  // 联合优化的足迹净空目标与上面迭代修正一致；中心目标低于足迹半宽以免中心项主导。
  params.joint_center_clearance = 0.40;
  params.joint_footprint_clearance = 0.40;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  populateFootprintEdgeObstacleEsdf(esdf);

  const auto center_reference = optimizer.optimize(makeObstacleSkimmingPath(), &esdf);
  ASSERT_FALSE(center_reference.empty());
  auto yaw_reference = center_reference;
  for (auto & point : yaw_reference.points) {
    point.yaw = 0.0;
  }
  const auto footprint_optimized = optimizer.optimize(
    makeObstacleSkimmingPath(), &esdf, &yaw_reference);

  ASSERT_FALSE(footprint_optimized.empty());
  EXPECT_NEAR(footprint_optimized.points.front().x, 0.0, 1e-8);
  EXPECT_NEAR(footprint_optimized.points.front().y, 0.0, 1e-8);
  EXPECT_NEAR(footprint_optimized.points.back().x, 2.0, 1e-8);
  EXPECT_NEAR(footprint_optimized.points.back().y, 0.0, 1e-8);
  EXPECT_GT(
    minimumFootprintDistance(
      footprint_optimized, esdf, params.footprint_length, params.footprint_width,
      params.footprint_safety_margin, params.esdf_footprint_sample_spacing),
    0.39);
  EXPECT_GT(
    minimumFootprintDistance(
      footprint_optimized, esdf, params.footprint_length, params.footprint_width,
      params.footprint_safety_margin, params.esdf_footprint_sample_spacing),
    minimumFootprintDistance(
      center_reference, esdf, params.footprint_length, params.footprint_width,
       params.footprint_safety_margin, params.esdf_footprint_sample_spacing) + 0.05);
}

TEST(MincoTrajectoryOptimizer, NonFiniteInitialStateFailsClosed)
{
  minco_planner::MincoTrajectoryOptimizer optimizer;
  minco_planner::InitialKinematicState initial_state;
  initial_state.valid = true;
  initial_state.velocity = Eigen::Vector2d(
    std::numeric_limits<double>::quiet_NaN(), 0.0);
  minco_planner::MincoOptimizationTrace trace;
  const auto trajectory = optimizer.optimize(
    makePath(), nullptr, nullptr, &initial_state, nullptr, nullptr, &trace);
  EXPECT_TRUE(trajectory.empty());
  EXPECT_EQ(trace.failure_reason, "non_finite_initial_state");
}

TEST(MincoTrajectoryOptimizer, SharedFrozenInitialStateSeedsEveryPass)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.0;
  params.max_velocity = 3.0;
  params.max_acceleration = 10.0;
  params.esdf_obstacle_optimization_enabled = false;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  minco_planner::InitialKinematicState initial_state;
  initial_state.valid = true;
  initial_state.velocity = Eigen::Vector2d(0.25, 0.05);
  const auto center = optimizer.optimize(makePath(), nullptr, nullptr, &initial_state);
  const auto fallback = optimizer.optimize(makePath(), nullptr, nullptr, &initial_state);
  ASSERT_FALSE(center.empty());
  ASSERT_FALSE(fallback.empty());
  EXPECT_NEAR(center.points.front().vx, 0.25, 1e-6);
  EXPECT_NEAR(center.points.front().vy, 0.05, 1e-6);
  EXPECT_NEAR(fallback.points.front().vx, 0.25, 1e-6);
  EXPECT_NEAR(fallback.points.front().vy, 0.05, 1e-6);
}


TEST(MincoTrajectoryOptimizer, TinySeedOnTwoWaypointsKeepsBoundedDuration)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.5;
  params.sample_spacing = 0.12;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.esdf_obstacle_optimization_enabled = false;
  minco_planner::MincoTrajectoryOptimizer optimizer(params);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & xy : {std::pair<double, double>{0.0, 0.0}, {2.5, 0.0}}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  minco_planner::InitialKinematicState initial_state;
  initial_state.valid = true;
  initial_state.velocity = Eigen::Vector2d(0.008, 0.0);
  const auto trajectory = optimizer.optimize(path, nullptr, nullptr, &initial_state);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_LT(trajectory.totalTime(), 20.0);
  EXPECT_LT(trajectory.points.size(), 400U);
}

nav_msgs::msg::Path makeLPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & xy :
    {std::pair<double, double> {0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}})
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

double distanceToSegment(
  double x, double y, const Eigen::Vector2d & start, const Eigen::Vector2d & end)
{
  const Eigen::Vector2d delta = end - start;
  const Eigen::Vector2d offset(x - start.x(), y - start.y());
  const double length2 = delta.squaredNorm();
  const double blend = length2 > 1e-12 ?
    std::max(0.0, std::min(1.0, offset.dot(delta) / length2)) : 0.0;
  const Eigen::Vector2d projection = start + blend * delta;
  return (Eigen::Vector2d(x, y) - projection).norm();
}

double maxPolylineDeviation(const minco_planner::ReferenceTrajectory & trajectory)
{
  const Eigen::Vector2d start(0.0, 0.0);
  const Eigen::Vector2d corner(3.0, 0.0);
  const Eigen::Vector2d goal(3.0, 3.0);
  double peak = 0.0;
  for (const auto & point : trajectory.points) {
    peak = std::max(
      peak,
      std::min(
        distanceToSegment(point.x, point.y, start, corner),
        distanceToSegment(point.x, point.y, corner, goal)));
  }
  return peak;
}

// 没有 ESDF 时联合优化没有障碍信息，引导管道把拐角内切限制在
// joint_guide_max_deviation 附近（三次铰链是软约束，允许少量超出）。
TEST(MincoTrajectoryOptimizer, GuideTubeBoundsLCornerCutWithoutEsdf)
{
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.5;
  params.sample_spacing = 0.05;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_jerk = 12.0;
  params.esdf_obstacle_optimization_enabled = false;
  params.geometry_preprocessor.footprint_aware_shortcut_enabled = false;
  params.geometry_preprocessor.fillet_radius = 0.0;

  minco_planner::MincoTrajectoryOptimizer sparse(params);
  const auto sparse_trajectory = sparse.optimize(makeLPath());
  params.guide_control_point_spacing = 0.30;
  minco_planner::MincoTrajectoryOptimizer dense(params);
  minco_planner::MincoOptimizationTrace trace;
  const auto dense_trajectory = dense.optimize(
    makeLPath(), nullptr, nullptr, nullptr, nullptr, nullptr, &trace);

  ASSERT_FALSE(sparse_trajectory.empty());
  ASSERT_FALSE(dense_trajectory.empty());
  // 联合优化只看 K=16 的离散采样点，时间缩放后的密采样允许略大一点的超出。
  EXPECT_LT(maxPolylineDeviation(sparse_trajectory), params.joint_guide_max_deviation + 0.08);
  EXPECT_LT(maxPolylineDeviation(dense_trajectory), params.joint_guide_max_deviation + 0.08);
  EXPECT_LT(trace.joint_max_guide_excess, 0.05);
}

// 按弧长重采样后的最大 Menger 曲率；时间采样在起停处点距趋零，会放大数值噪声。
double maxArcCurvature(const minco_planner::ReferenceTrajectory & trajectory, double spacing)
{
  std::vector<Eigen::Vector2d> points;
  for (const auto & point : trajectory.points) {
    const Eigen::Vector2d current(point.x, point.y);
    if (points.empty() || (current - points.back()).norm() >= spacing) {
      points.push_back(current);
    }
  }
  double maximum = 0.0;
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    const Eigen::Vector2d first = points[index] - points[index - 1U];
    const Eigen::Vector2d second = points[index + 1U] - points[index];
    const double denominator =
      first.norm() * second.norm() * (points[index + 1U] - points[index - 1U]).norm();
    if (denominator > 1e-9) {
      maximum = std::max(
        maximum, std::abs(2.0 * (first.x() * second.y() - first.y() * second.x()) / denominator));
    }
  }
  return maximum;
}

TEST(MincoTrajectoryOptimizer, GuideSmoothingRoundsDenseLCornerAndKeepsEndpoints)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = 120;
  grid.info.height = 120;
  grid.info.origin.position.x = -2.0;
  grid.info.origin.position.y = -2.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width * grid.info.height), 0);
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);

  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.5;
  params.sample_spacing = 0.05;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_jerk = 12.0;
  params.esdf_obstacle_optimization_enabled = false;
  params.geometry_preprocessor.footprint_aware_shortcut_enabled = false;
  params.geometry_preprocessor.fillet_radius = 0.0;
  params.guide_control_point_spacing = 0.30;
  minco_planner::MincoTrajectoryOptimizer polyline(params);
  const auto polyline_trajectory = polyline.optimize(makeLPath(), &esdf);
  params.guide_smoothing_iterations = 80;
  minco_planner::MincoTrajectoryOptimizer smoothed(params);
  const auto smoothed_trajectory = smoothed.optimize(makeLPath(), &esdf);

  ASSERT_FALSE(polyline_trajectory.empty());
  ASSERT_FALSE(smoothed_trajectory.empty());
  const auto & first = makeLPath().poses.front().pose.position;
  const auto & last = makeLPath().poses.back().pose.position;
  EXPECT_NEAR(smoothed_trajectory.points.front().x, first.x, 1e-8);
  EXPECT_NEAR(smoothed_trajectory.points.front().y, first.y, 1e-8);
  EXPECT_NEAR(smoothed_trajectory.points.back().x, last.x, 1e-8);
  EXPECT_NEAR(smoothed_trajectory.points.back().y, last.y, 1e-8);
  const double polyline_curvature = maxArcCurvature(polyline_trajectory, 0.05);
  const double smoothed_curvature = maxArcCurvature(smoothed_trajectory, 0.05);
  EXPECT_LT(smoothed_curvature, 0.5 * polyline_curvature);
  // 偏离受 max_deviation 约束，不会把拐角抄近路抄穿。
  EXPECT_LT(maxPolylineDeviation(smoothed_trajectory), params.guide_smoothing_max_deviation + 0.10);
}

}  // namespace
