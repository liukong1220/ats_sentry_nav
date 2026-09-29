// Copyright 2026

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "minco_planner/trajectory/minco_joint_optimizer.hpp"
#include "minco_planner/trajectory/minco_s3.hpp"
#include "minco_planner/trajectory/minco_time_allocator.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

namespace
{

using Point = Eigen::Vector2d;
using minco_planner::JointDistanceQuery;
using minco_planner::JointYawQuery;
using minco_planner::MincoJointOptimizer;
using minco_planner::MincoJointOptimizerParams;
using minco_planner::MincoJointProblem;
using minco_planner::MincoJointResult;

// 圆盘障碍的解析 ESDF：d = |p - c| - r，∇d = (p - c)/|p - c|。
JointDistanceQuery makeDiskDistance(const Point & center, double radius)
{
  return [center, radius](const Point & position, double & distance, Point & gradient) {
           const Point offset = position - center;
           const double norm = std::max(1e-9, offset.norm());
           distance = norm - radius;
           gradient = offset / norm;
           return true;
         };
}

// y = ±half_gap 两面墙的走廊：d = half_gap - |y|。
JointDistanceQuery makeCorridorDistance(double half_gap)
{
  return [half_gap](const Point & position, double & distance, Point & gradient) {
           distance = half_gap - std::abs(position.y());
           gradient = Point(0.0, position.y() >= 0.0 ? -1.0 : 1.0);
           return true;
         };
}

std::vector<Point> lPolyline()
{
  return {Point(0.0, 0.0), Point(3.0, 0.0), Point(3.0, 3.0)};
}

Eigen::VectorXd allocate(const std::vector<Point> & waypoints, double initial_speed = 0.0)
{
  minco_planner::MincoTimeAllocatorParams params;
  params.reference_speed = 1.5;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_lateral_acceleration = 1.5;
  const auto allocation = minco_planner::MincoTimeAllocator(params).allocate(
    waypoints, initial_speed);
  EXPECT_TRUE(allocation.valid);
  return allocation.durations;
}

bool solveMinco(
  const Eigen::Matrix<double, 2, 3> & head,
  const std::vector<Point> & waypoints,
  const Eigen::VectorXd & durations,
  minco_planner::MincoS3 & minco)
{
  Eigen::Matrix<double, 2, 3> head_state = head;
  Eigen::Matrix<double, 2, 3> tail = Eigen::Matrix<double, 2, 3>::Zero();
  head_state.col(0) = waypoints.front();
  tail.col(0) = waypoints.back();
  Eigen::MatrixXd inner(2, static_cast<int>(waypoints.size()) - 2);
  for (int i = 0; i < inner.cols(); ++i) {
    inner.col(i) = waypoints[static_cast<std::size_t>(i + 1)];
  }
  return minco.solve(head_state, tail, inner, durations);
}

std::vector<minco_planner::MincoSample> sampleMinco(
  const minco_planner::MincoS3 & minco, int per_piece = 40)
{
  std::vector<minco_planner::MincoSample> samples;
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    for (int step = piece == 0 ? 0 : 1; step <= per_piece; ++step) {
      samples.push_back(minco.sample(piece, minco.pieceDuration(piece) * step / per_piece));
    }
  }
  return samples;
}

// 按弧长等距取点后的 Menger 曲率序列（避开起停处时间采样点距趋零的数值噪声）。
std::vector<double> arcCurvatures(const std::vector<Point> & positions, double spacing)
{
  std::vector<Point> points;
  for (const Point & position : positions) {
    if (points.empty() || (position - points.back()).norm() >= spacing) {
      points.push_back(position);
    }
  }
  std::vector<double> curvatures;
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    const Point first = points[index] - points[index - 1U];
    const Point second = points[index + 1U] - points[index];
    const double denominator =
      first.norm() * second.norm() * (points[index + 1U] - points[index - 1U]).norm();
    if (denominator > 1e-12) {
      curvatures.push_back(
        std::abs(2.0 * (first.x() * second.y() - first.y() * second.x()) / denominator));
    }
  }
  return curvatures;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index = std::min(
    values.size() - 1U, static_cast<std::size_t>(std::floor(fraction * (values.size() - 1U))));
  return values[index];
}

std::vector<Point> positionsOf(const std::vector<minco_planner::MincoSample> & samples)
{
  std::vector<Point> positions;
  for (const auto & sample : samples) {
    positions.push_back(sample.position);
  }
  return positions;
}

MincoJointProblem makeProblem(
  const std::vector<Point> & waypoints, const Eigen::VectorXd & durations,
  const Eigen::Matrix<double, 2, 3> & head,
  const JointDistanceQuery & distance, const JointYawQuery & yaw)
{
  MincoJointProblem problem;
  problem.head_state = head;
  problem.head_state.col(0) = waypoints.front();
  problem.tail_position = waypoints.back();
  problem.piece_count = static_cast<int>(waypoints.size()) - 1;
  problem.distance = distance;
  problem.yaw = yaw;
  // 与 optimize() 一致：yaw 进度按固定时长计算，与决策变量无关。
  problem.yaw_progress_durations = durations;
  return problem;
}

Eigen::VectorXd packVariables(
  const std::vector<Point> & waypoints, const Eigen::VectorXd & durations)
{
  const int pieces = static_cast<int>(durations.size());
  Eigen::VectorXd x(3 * pieces - 2);
  for (int i = 0; i + 1 < pieces; ++i) {
    x.segment<2>(2 * i) = waypoints[static_cast<std::size_t>(i + 1)];
  }
  x.tail(pieces) = durations.array().log().matrix();
  return x;
}

// 相对误差 ||g_fd - g|| / ||g||，中心差分步长 h。
double gradientRelativeError(
  const MincoJointOptimizer & optimizer, const MincoJointProblem & problem,
  const Eigen::VectorXd & x, double * cost_out = nullptr)
{
  Eigen::VectorXd analytic;
  const double cost = optimizer.evaluate(problem, x, analytic);
  EXPECT_TRUE(std::isfinite(cost));
  if (cost_out) {
    *cost_out = cost;
  }
  Eigen::VectorXd numeric(x.size());
  Eigen::VectorXd scratch;
  const double h = 1e-6;
  for (int k = 0; k < x.size(); ++k) {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(k) += h;
    minus(k) -= h;
    numeric(k) = (optimizer.evaluate(problem, plus, scratch) -
      optimizer.evaluate(problem, minus, scratch)) / (2.0 * h);
  }
  return (numeric - analytic).norm() / std::max(1e-12, analytic.norm());
}

MincoJointOptimizerParams isolatedTerm()
{
  MincoJointOptimizerParams params;
  params.energy_weight = 0.0;
  params.time_weight = 0.0;
  params.time_bound_weight = 0.0;
  params.obstacle_weight = 0.0;
  params.velocity_weight = 0.0;
  params.acceleration_weight = 0.0;
  params.lateral_weight = 0.0;
  params.guide_weight = 0.0;
  params.footprint_length = 0.58;
  params.footprint_width = 0.58;
  params.footprint_safety_margin = 0.02;
  return params;
}

TEST(MincoJointOptimizer, EveryCostTermMatchesCentralDifference)
{
  // 非对称航点 + 非零首端速度/加速度，保证每项都在铰链激活区。
  const std::vector<Point> waypoints = {
    Point(0.0, 0.0), Point(0.9, 0.2), Point(1.6, 0.9), Point(1.9, 1.8), Point(2.8, 2.1)};
  Eigen::VectorXd durations(4);
  durations << 0.45, 0.55, 0.50, 0.60;
  Eigen::Matrix<double, 2, 3> head = Eigen::Matrix<double, 2, 3>::Zero();
  head.col(1) = Point(0.8, 0.3);
  head.col(2) = Point(0.2, -0.4);
  const Eigen::VectorXd x = packVariables(waypoints, durations);
  const JointDistanceQuery disk = makeDiskDistance(Point(1.1, 1.0), 0.15);
  const JointYawQuery yaw = [](double progress) {return 0.3 + 0.8 * progress;};

  struct Case
  {
    const char * name;
    MincoJointOptimizerParams params;
    bool use_distance;
    bool use_yaw;
    bool use_guide = false;
  };
  std::vector<Case> cases;
  {
    Case energy{"energy", isolatedTerm(), false, false};
    energy.params.energy_weight = 1.0;
    cases.push_back(energy);
    Case time{"time", isolatedTerm(), false, false};
    time.params.time_weight = 20.0;
    time.params.time_bound_weight = 1e4;
    time.params.max_piece_time = 0.5;
    cases.push_back(time);
    Case center{"center_obstacle", isolatedTerm(), true, false};
    center.params.obstacle_weight = 1e4;
    center.params.center_clearance = 0.60;
    cases.push_back(center);
    Case footprint{"footprint_obstacle", isolatedTerm(), true, true};
    footprint.params.obstacle_weight = 1e4;
    footprint.params.center_clearance = 0.0;
    footprint.params.footprint_clearance = 0.10;
    footprint.params.footprint_edge_samples = 2;
    cases.push_back(footprint);
    Case guide{"guide_tube", isolatedTerm(), false, false};
    guide.params.guide_weight = 1e5;
    guide.params.max_guide_deviation = 0.10;
    guide.use_guide = true;
    cases.push_back(guide);
    Case velocity{"velocity", isolatedTerm(), false, false};
    velocity.params.velocity_weight = 1e3;
    velocity.params.max_velocity = 1.0;
    cases.push_back(velocity);
    Case acceleration{"acceleration", isolatedTerm(), false, false};
    acceleration.params.acceleration_weight = 1e3;
    acceleration.params.max_acceleration = 0.8;
    cases.push_back(acceleration);
    Case lateral{"lateral", isolatedTerm(), false, false};
    lateral.params.lateral_weight = 1e3;
    lateral.params.max_lateral_acceleration = 0.5;
    cases.push_back(lateral);
  }

  for (const Case & test_case : cases) {
    const MincoJointOptimizer optimizer(test_case.params);
    MincoJointProblem problem = makeProblem(
      waypoints, durations, head, test_case.use_distance ? disk : JointDistanceQuery(),
      test_case.use_yaw ? yaw : JointYawQuery());
    if (test_case.use_guide) {
      // 与航点不重合的折线，保证各采样点的最近点落在折线段内部。
      problem.guide = {Point(0.0, 0.0), Point(1.4, -0.1), Point(1.5, 2.2), Point(2.8, 2.1)};
    }
    double cost = 0.0;
    const double error = gradientRelativeError(optimizer, problem, x, &cost);
    EXPECT_GT(cost, 1e-6) << test_case.name << " term is inactive";
    EXPECT_LT(error, 1e-5) << test_case.name;
  }
}

TEST(MincoJointOptimizer, SolveAdjMatchesDenseTransposeSolve)
{
  const int size = 24;
  const int lower = 3;
  const int upper = 4;
  std::mt19937 generator(7);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  minco_planner::BandedSystem banded;
  banded.create(size, lower, upper);
  banded.reset();
  Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(size, size);
  for (int row = 0; row < size; ++row) {
    for (int column = std::max(0, row - lower); column <= std::min(size - 1, row + upper);
      ++column)
    {
      const double value = row == column ? 8.0 + uniform(generator) : uniform(generator);
      banded(row, column) = value;
      dense(row, column) = value;
    }
  }
  Eigen::MatrixX2d rhs(size, 2);
  for (int row = 0; row < size; ++row) {
    rhs(row, 0) = uniform(generator);
    rhs(row, 1) = uniform(generator);
  }
  const Eigen::MatrixX2d expected = dense.transpose().fullPivLu().solve(rhs);
  ASSERT_TRUE(banded.factorizeLu());
  Eigen::MatrixX2d actual = rhs;
  ASSERT_TRUE(banded.solveAdj(actual));
  EXPECT_LT((actual - expected).norm() / expected.norm(), 1e-12);
}

TEST(MincoJointOptimizer, StraightLineStaysOnLineWithExactEndpoints)
{
  const std::vector<Point> guide = {Point(0.0, 0.0), Point(5.0, 0.0)};
  const std::vector<Point> waypoints = MincoJointOptimizer::resampleByArcLength(guide, 1.0);
  ASSERT_EQ(waypoints.size(), 6U);
  const MincoJointOptimizer optimizer;
  const MincoJointResult result = optimizer.optimize(
    Eigen::Matrix<double, 2, 3>::Zero(), waypoints, allocate(waypoints),
    JointDistanceQuery(), JointYawQuery());
  ASSERT_TRUE(result.solved) << result.termination;
  minco_planner::MincoS3 minco;
  ASSERT_TRUE(
    solveMinco(
      Eigen::Matrix<double, 2, 3>::Zero(), result.waypoints, result.durations, minco));
  double lateral = 0.0;
  for (const auto & sample : sampleMinco(minco)) {
    lateral = std::max(lateral, std::abs(sample.position.y()));
  }
  EXPECT_LT(lateral, 1e-6);
  const int last_piece = minco.pieceCount() - 1;
  const auto last = minco.sample(last_piece, minco.pieceDuration(last_piece));
  EXPECT_NEAR((minco.sample(0, 0.0).position - guide.front()).norm(), 0.0, 1e-12);
  EXPECT_NEAR((last.position - guide.back()).norm(), 0.0, 1e-12);
  EXPECT_LT(last.velocity.norm(), 1e-9);
}

TEST(MincoJointOptimizer, LPathIsSmootherThanDenseGuideMincoAndRespectsLimits)
{
  // 基线：历史做法，0.30 m 加密折线全部作为 MINCO 硬插值点。
  std::vector<Point> dense;
  const std::vector<Point> corners = lPolyline();
  dense.push_back(corners.front());
  for (std::size_t index = 0; index + 1U < corners.size(); ++index) {
    const int steps = static_cast<int>(
      std::ceil((corners[index + 1U] - corners[index]).norm() / 0.30));
    for (int step = 1; step <= steps; ++step) {
      dense.push_back(corners[index] + (corners[index + 1U] - corners[index]) * step / steps);
    }
  }
  minco_planner::MincoS3 baseline;
  ASSERT_TRUE(solveMinco(Eigen::Matrix<double, 2, 3>::Zero(), dense, allocate(dense), baseline));
  const std::vector<double> baseline_curvature = arcCurvatures(
    positionsOf(sampleMinco(baseline)), 0.05);

  // 联合优化走完整 MincoTrajectoryOptimizer 流水线（含时间缩放），检验上限闭合。
  minco_planner::MincoTrajectoryOptimizerParams params;
  params.reference_speed = 1.5;
  params.sample_spacing = 0.02;
  params.max_velocity = 2.0;
  params.max_acceleration = 2.5;
  params.max_jerk = 12.0;
  params.max_lateral_acceleration = 1.5;
  params.geometry_preprocessor.footprint_aware_shortcut_enabled = false;
  params.geometry_preprocessor.fillet_radius = 0.0;
  params.guide_control_point_spacing = 0.30;
  // 部署配置下联合优化总有 ESDF，此时不加引导管道，由障碍项约束拐角。
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = 120;
  grid.info.height = 120;
  grid.info.origin.position.x = -2.0;
  grid.info.origin.position.y = -2.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  // 内角障碍块 x∈[-2, 1.8]、y∈[1.2, 4]：
  // 中心净空 0.42 迫使轨迹沿 L 转弯，而非走对角线。
  for (unsigned int y = 64; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x <= 76; ++x) {
      grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
    }
  }
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const Point & corner : corners) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = corner.x();
    pose.pose.position.y = corner.y();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  minco_planner::MincoOptimizationTrace trace;
  const auto trajectory = minco_planner::MincoTrajectoryOptimizer(params).optimize(
    path, &esdf, nullptr, nullptr, nullptr, nullptr, &trace);
  ASSERT_FALSE(trajectory.empty()) << trace.failure_reason;
  EXPECT_FALSE(trace.joint_termination.empty());
  EXPECT_GT(trace.joint_iterations, 0);

  std::vector<Point> positions;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_lateral = 0.0;
  for (const auto & point : trajectory.points) {
    positions.emplace_back(point.x, point.y);
    const Point velocity(point.vx, point.vy);
    const Point acceleration(point.ax, point.ay);
    peak_velocity = std::max(peak_velocity, velocity.norm());
    peak_acceleration = std::max(peak_acceleration, acceleration.norm());
    if (velocity.norm() > 0.1) {
      peak_lateral = std::max(
        peak_lateral,
        std::abs(velocity.x() * acceleration.y() - velocity.y() * acceleration.x()) /
        velocity.norm());
    }
  }
  const std::vector<double> joint_curvature = arcCurvatures(positions, 0.05);
  const double baseline_kmax = percentile(baseline_curvature, 1.0);
  const double baseline_k95 = percentile(baseline_curvature, 0.95);
  const double joint_kmax = percentile(joint_curvature, 1.0);
  const double joint_k95 = percentile(joint_curvature, 0.95);
  RecordProperty("baseline_kmax", std::to_string(baseline_kmax));
  RecordProperty("joint_kmax", std::to_string(joint_kmax));
  RecordProperty("baseline_k95", std::to_string(baseline_k95));
  RecordProperty("joint_k95", std::to_string(joint_k95));
  EXPECT_LT(joint_kmax, baseline_kmax) << "baseline " << baseline_kmax << " joint " << joint_kmax;
  EXPECT_LT(joint_k95, baseline_k95) << "baseline " << baseline_k95 << " joint " << joint_k95;
  EXPECT_NEAR(trajectory.points.front().x, corners.front().x(), 1e-8);
  EXPECT_NEAR(trajectory.points.front().y, corners.front().y(), 1e-8);
  EXPECT_NEAR(trajectory.points.back().x, corners.back().x(), 1e-8);
  EXPECT_NEAR(trajectory.points.back().y, corners.back().y(), 1e-8);
  EXPECT_LE(peak_velocity, params.max_velocity + 1e-3);
  EXPECT_LE(peak_acceleration, params.max_acceleration + 1e-3);
  EXPECT_LE(peak_lateral, params.max_lateral_acceleration + 1e-3);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (const Point & position : positions) {
    minimum_clearance = std::min(minimum_clearance, esdf.getDistance(position.x(), position.y()));
  }
  RecordProperty("l_path_min_center_clearance", std::to_string(minimum_clearance));
  EXPECT_GE(minimum_clearance, params.joint_center_clearance - 0.05);
}

TEST(MincoJointOptimizer, InnerCornerObstacleKeepsFootprintClearance)
{
  const std::vector<Point> waypoints =
    MincoJointOptimizer::resampleByArcLength(lPolyline(), 1.0);
  // 自由优化会把 L 角抹成近似对角线，圆盘放在对角线附近的内角区域。
  const Point disk_center(1.8, 1.2);
  const double disk_radius = 0.12;
  MincoJointOptimizerParams params;
  params.footprint_length = 0.58;
  params.footprint_width = 0.58;
  params.footprint_safety_margin = 0.02;
  params.center_clearance = 0.44;
  params.footprint_clearance = 0.03;
  params.solver.time_budget_ms = 0.0;
  const JointYawQuery yaw = [](double) {return 0.0;};
  const MincoJointOptimizer optimizer(params);

  // 对照：不给障碍时联合优化会把拐角抹圆到压进圆盘，证明本测试确实在检验障碍项。
  const MincoJointResult free = optimizer.optimize(
    Eigen::Matrix<double, 2, 3>::Zero(), waypoints, allocate(waypoints),
    JointDistanceQuery(), JointYawQuery());
  const MincoJointResult guarded = optimizer.optimize(
    Eigen::Matrix<double, 2, 3>::Zero(), waypoints, allocate(waypoints),
    makeDiskDistance(disk_center, disk_radius), yaw);
  ASSERT_TRUE(free.solved);
  ASSERT_TRUE(guarded.solved) << guarded.termination;

  const auto footprintClearance = [&](const MincoJointResult & result) {
      minco_planner::MincoS3 minco;
      EXPECT_TRUE(
        solveMinco(
          Eigen::Matrix<double, 2, 3>::Zero(), result.waypoints, result.durations, minco));
      const double half = 0.5 * params.footprint_length + params.footprint_safety_margin;
      double minimum = std::numeric_limits<double>::infinity();
      for (const auto & sample : sampleMinco(minco, 64)) {
        // 轴对齐方形足迹到圆盘的精确距离。
        const Point offset = disk_center - sample.position;
        const Point nearest(
          std::clamp(offset.x(), -half, half), std::clamp(offset.y(), -half, half));
        minimum = std::min(minimum, (offset - nearest).norm() - disk_radius);
      }
      return minimum;
    };
  const double free_clearance = footprintClearance(free);
  const double guarded_clearance = footprintClearance(guarded);
  EXPECT_LT(free_clearance, params.footprint_clearance - params.obstacle_tolerance);
  RecordProperty("free_clearance", std::to_string(free_clearance));
  RecordProperty("guarded_clearance", std::to_string(guarded_clearance));
  EXPECT_GE(guarded_clearance, params.footprint_clearance - params.obstacle_tolerance)
    << "free " << free_clearance << " guarded " << guarded_clearance;
  EXPECT_LE(guarded.stats.max_footprint_violation, params.obstacle_tolerance);
}

TEST(MincoJointOptimizer, CorridorNarrowerThanFootprintReportsResidual)
{
  const std::vector<Point> waypoints = MincoJointOptimizer::resampleByArcLength(
    {Point(0.0, 0.0), Point(4.0, 0.0)}, 1.0);
  MincoJointOptimizerParams params;
  params.footprint_length = 0.58;
  params.footprint_width = 0.58;
  params.footprint_safety_margin = 0.02;
  params.center_clearance = 0.0;
  const MincoJointOptimizer optimizer(params);
  // 墙距中心 0.25 m，足迹半宽 0.31 m：任何平移都不可行。
  const MincoJointResult result = optimizer.optimize(
    Eigen::Matrix<double, 2, 3>::Zero(), waypoints, allocate(waypoints),
    makeCorridorDistance(0.25), [](double) {return 0.0;});
  ASSERT_TRUE(result.solved) << result.termination;
  EXPECT_FALSE(result.constraints_satisfied);
  EXPECT_GT(result.stats.max_footprint_violation, 0.05);
  ASSERT_EQ(result.waypoints.size(), waypoints.size());
  EXPECT_NEAR((result.waypoints.front() - waypoints.front()).norm(), 0.0, 1e-12);
  EXPECT_NEAR((result.waypoints.back() - waypoints.back()).norm(), 0.0, 1e-12);
  for (const Point & point : result.waypoints) {
    EXPECT_TRUE(point.allFinite());
  }
  EXPECT_TRUE(result.durations.allFinite());
}

TEST(MincoJointOptimizer, TwentySixMeterPathFinishesWithinBudget)
{
  // RMUC 规模：26 m 折线穿过一排立柱，使用真实 RC-ESDF 查询（含互斥锁开销）。
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = 640;
  grid.info.height = 240;
  grid.info.origin.position.x = -2.0;
  grid.info.origin.position.y = -6.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  for (int pillar = 0; pillar < 6; ++pillar) {
    const int cx = 80 + pillar * 80;
    const int cy = pillar % 2 == 0 ? 100 : 140;
    for (int y = cy - 6; y <= cy + 6; ++y) {
      for (int x = cx - 6; x <= cx + 6; ++x) {
        grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
      }
    }
  }
  ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
  esdf.configureRollingWindow(false, 0.0, 0.0);
  esdf.updateGrid(grid, 50, true);
  ASSERT_TRUE(esdf.available());

  const std::vector<Point> guide = {
    Point(0.0, 0.0), Point(6.0, 0.0), Point(9.0, 2.0), Point(15.0, 2.0),
    Point(18.0, -1.0), Point(24.0, -1.0), Point(26.0, 0.0)};
  const std::vector<Point> waypoints = MincoJointOptimizer::resampleByArcLength(guide, 1.0);
  ASSERT_GE(waypoints.size(), 25U);

  MincoJointOptimizerParams params;
  params.footprint_length = 0.58;
  params.footprint_width = 0.58;
  params.footprint_safety_margin = 0.02;
  params.center_clearance = 0.44;
  const MincoJointOptimizer optimizer(params);
  const JointDistanceQuery distance = [&esdf](const Point & p, double & d, Point & g) {
      ats_rc_esdf::EsdfQueryResult query;
      if (!esdf.query(p.x(), p.y(), query)) {
        return false;
      }
      d = query.distance;
      g = query.gradient;
      return true;
    };
  const MincoJointResult result = optimizer.optimize(
    Eigen::Matrix<double, 2, 3>::Zero(), waypoints, allocate(waypoints), distance,
    [](double progress) {return 0.2 * progress;});
  ASSERT_TRUE(result.solved) << result.termination;
  EXPECT_GT(result.iterations, 0);
  EXPECT_LE(result.final_cost, result.initial_cost);
  RecordProperty("wall_time_ms", std::to_string(result.wall_time_ms));
  RecordProperty("iterations", result.iterations);
  RecordProperty("termination", result.termination);
  // 预算只在每次迭代结束后检查，允许超出最后一次线搜索的耗时。
  EXPECT_LT(result.wall_time_ms, 2.0 * params.solver.time_budget_ms)
    << "iterations " << result.iterations << " evaluations " << result.evaluations;
}

}  // namespace
