// Copyright 2026

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/trajectory/minco_s3.hpp"

namespace minco_planner
{

namespace
{

using Point = Eigen::Vector2d;

std::vector<Point> extractWaypoints(const nav_msgs::msg::Path & path)
{
  std::vector<Point> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const Point point(pose.pose.position.x, pose.pose.position.y);
    if (points.empty() || (point - points.back()).norm() > 1e-6) {
      points.push_back(point);
    }
  }
  if (points.size() <= 2) {
    return points;
  }

  std::vector<Point> simplified;
  simplified.reserve(points.size());
  simplified.push_back(points.front());
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Point incoming = points[i] - simplified.back();
    const Point outgoing = points[i + 1] - points[i];
    const double cross = incoming.x() * outgoing.y() - incoming.y() * outgoing.x();
    const double scale = std::max(1e-9, incoming.norm() * outgoing.norm());
    if (std::abs(cross) / scale > 1e-3 || incoming.dot(outgoing) <= 0.0) {
      simplified.push_back(points[i]);
    }
  }
  simplified.push_back(points.back());
  return simplified;
}

Eigen::VectorXd allocateDurations(
  const std::vector<Point> & waypoints,
  double reference_speed,
  double min_segment_time)
{
  Eigen::VectorXd durations(static_cast<int>(waypoints.size()) - 1);
  for (int i = 0; i < durations.size(); ++i) {
    durations(i) = std::max(
      min_segment_time,
      (waypoints[static_cast<std::size_t>(i + 1)] -
      waypoints[static_cast<std::size_t>(i)]).norm() / reference_speed);
  }
  return durations;
}

bool solveMinco(
  const std::vector<Point> & waypoints,
  const Eigen::VectorXd & durations,
  MincoS3 & minco)
{
  Eigen::Matrix<double, 2, 3> head = Eigen::Matrix<double, 2, 3>::Zero();
  Eigen::Matrix<double, 2, 3> tail = Eigen::Matrix<double, 2, 3>::Zero();
  head.col(0) = waypoints.front();
  tail.col(0) = waypoints.back();
  Eigen::MatrixXd inner_points(2, static_cast<int>(waypoints.size()) - 2);
  for (int i = 0; i < inner_points.cols(); ++i) {
    inner_points.col(i) = waypoints[static_cast<std::size_t>(i + 1)];
  }
  return minco.solve(head, tail, inner_points, durations);
}

void findDynamicExtrema(
  const MincoS3 & minco,
  double sample_spacing,
  double reference_speed,
  double & max_velocity,
  double & max_acceleration)
{
  max_velocity = 0.0;
  max_acceleration = 0.0;
  const double sample_dt = sample_spacing / std::max(0.05, reference_speed);
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const int steps = std::max(
      2, static_cast<int>(std::ceil(minco.pieceDuration(piece) / sample_dt)));
    for (int step = 0; step <= steps; ++step) {
      const auto sample = minco.sample(
        piece, minco.pieceDuration(piece) * static_cast<double>(step) / steps);
      max_velocity = std::max(max_velocity, sample.velocity.norm());
      max_acceleration = std::max(max_acceleration, sample.acceleration.norm());
    }
  }
}

}  // namespace

MincoTrajectoryOptimizer::MincoTrajectoryOptimizer(MincoTrajectoryOptimizerParams params)
: params_(params)
{
}

void MincoTrajectoryOptimizer::setParams(const MincoTrajectoryOptimizerParams & params)
{
  params_ = params;
}

ReferenceTrajectory MincoTrajectoryOptimizer::optimize(const nav_msgs::msg::Path & raw_path) const
{
  ReferenceTrajectory trajectory;
  trajectory.header = raw_path.header;
  const std::vector<Point> waypoints = extractWaypoints(raw_path);
  if (waypoints.empty()) {
    return trajectory;
  }
  if (waypoints.size() == 1) {
    ReferencePoint point;
    point.x = waypoints.front().x();
    point.y = waypoints.front().y();
    trajectory.points.push_back(point);
    return trajectory;
  }

  const double reference_speed = std::max(0.05, params_.reference_speed);
  const double sample_spacing = std::max(0.02, params_.sample_spacing);
  Eigen::VectorXd durations = allocateDurations(
    waypoints, reference_speed, std::max(0.01, params_.min_segment_time));
  MincoS3 minco;
  if (!solveMinco(waypoints, durations, minco)) {
    return trajectory;
  }

  for (int iteration = 0; iteration < std::max(0, params_.max_time_scaling_iterations);
    ++iteration)
  {
    double peak_velocity = 0.0;
    double peak_acceleration = 0.0;
    findDynamicExtrema(
      minco, sample_spacing, reference_speed, peak_velocity, peak_acceleration);
    const bool velocity_ok = params_.max_velocity <= 0.0 ||
      peak_velocity <= params_.max_velocity + 1e-6;
    const bool acceleration_ok = params_.max_acceleration <= 0.0 ||
      peak_acceleration <= params_.max_acceleration + 1e-6;
    if (velocity_ok && acceleration_ok) {
      break;
    }
    double scale = std::max(1.01, params_.time_scaling_factor);
    if (!velocity_ok) {
      scale = std::max(scale, peak_velocity / params_.max_velocity);
    }
    if (!acceleration_ok) {
      scale = std::max(scale, std::sqrt(peak_acceleration / params_.max_acceleration));
    }
    durations *= scale;
    if (!solveMinco(waypoints, durations, minco)) {
      trajectory.points.clear();
      return trajectory;
    }
  }

  double accumulated_time = 0.0;
  double accumulated_distance = 0.0;
  Point previous_position = waypoints.front();
  for (int piece = 0; piece < minco.pieceCount(); ++piece) {
    const double duration = minco.pieceDuration(piece);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(duration * reference_speed / sample_spacing)));
    const int first_step = piece == 0 ? 0 : 1;
    for (int step = first_step; step <= steps; ++step) {
      const double local_time = duration * static_cast<double>(step) / steps;
      const MincoSample sample = minco.sample(piece, local_time);
      if (!trajectory.points.empty()) {
        accumulated_distance += (sample.position - previous_position).norm();
      }
      ReferencePoint point;
      point.t = accumulated_time + local_time;
      point.s = accumulated_distance;
      point.x = sample.position.x();
      point.y = sample.position.y();
      point.vx = sample.velocity.x();
      point.vy = sample.velocity.y();
      point.ax = sample.acceleration.x();
      point.ay = sample.acceleration.y();
      point.v = sample.velocity.norm();
      trajectory.points.push_back(point);
      previous_position = sample.position;
    }
    accumulated_time += duration;
  }
  return trajectory;
}

}  // namespace minco_planner
