// Copyright 2026

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/safety/footprint_samples.hpp"
#include "minco_planner/trajectory/minco_s3.hpp"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"

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

std::vector<Point> densifyWaypoints(const std::vector<Point> & waypoints, double spacing)
{
  if (waypoints.size() <= 1 || spacing <= 1e-6) {
    return waypoints;
  }

  std::vector<Point> dense;
  dense.reserve(waypoints.size());
  dense.push_back(waypoints.front());
  for (std::size_t index = 0; index + 1 < waypoints.size(); ++index) {
    const Point & start = waypoints[index];
    const Point & end = waypoints[index + 1];
    const int steps = std::max(1, static_cast<int>(std::ceil((end - start).norm() / spacing)));
    for (int step = 1; step <= steps; ++step) {
      dense.push_back(start + (end - start) * static_cast<double>(step) / steps);
    }
  }
  return dense;
}

Point limitNorm(const Point & value, double maximum_norm)
{
  const double norm = value.norm();
  if (maximum_norm > 0.0 && norm > maximum_norm) {
    return value * (maximum_norm / norm);
  }
  return value;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double interpolateReferenceYaw(const ReferenceTrajectory & reference, double progress)
{
  if (reference.points.empty()) {
    return 0.0;
  }
  if (reference.points.size() == 1U || reference.totalTime() <= 1e-6) {
    return reference.points.front().yaw;
  }

  const double target_time = std::max(0.0, std::min(1.0, progress)) * reference.totalTime();
  const auto upper = std::lower_bound(
    reference.points.begin(), reference.points.end(), target_time,
    [](const ReferencePoint & point, double time) {return point.t < time;});
  if (upper == reference.points.begin()) {
    return upper->yaw;
  }
  if (upper == reference.points.end()) {
    return reference.points.back().yaw;
  }

  const ReferencePoint & before = *(upper - 1);
  const double duration = std::max(1e-6, upper->t - before.t);
  const double alpha = std::max(0.0, std::min(1.0, (target_time - before.t) / duration));
  return normalizeAngle(before.yaw + alpha * normalizeAngle(upper->yaw - before.yaw));
}

bool queryFootprintEsdf(
  const trajectory_optimizer::RcTraversabilityEsdfProvider & esdf,
  const Point & position,
  double yaw,
  const std::vector<Point> & samples,
  trajectory_optimizer::EsdfQueryResult & result)
{
  result = trajectory_optimizer::EsdfQueryResult {};
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  bool found = false;
  for (const Point & sample : samples) {
    trajectory_optimizer::EsdfQueryResult query;
    const Point world = position + Point(
      cos_yaw * sample.x() - sin_yaw * sample.y(),
      sin_yaw * sample.x() + cos_yaw * sample.y());
    if (!esdf.query(world.x(), world.y(), query) || !std::isfinite(query.distance)) {
      continue;
    }
    if (!found || query.distance < result.distance) {
      result = query;
      found = true;
    }
  }
  return found;
}

std::vector<Point> refineWaypointsWithEsdf(
  const std::vector<Point> & input_waypoints,
  const MincoTrajectoryOptimizerParams & params,
  const trajectory_optimizer::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation)
{
  if (!params.esdf_obstacle_optimization_enabled || !esdf || !esdf->available()) {
    return input_waypoints;
  }

  const bool footprint_aware = params.esdf_footprint_optimization_enabled &&
    footprint_orientation && !footprint_orientation->empty();
  const double minimum_clearance = std::max(0.0, params.esdf_obstacle_clearance);
  const double footprint_clearance = std::max(0.0, params.esdf_footprint_clearance);
  const double maximum_step = std::max(0.0, params.esdf_obstacle_max_step);
  const double maximum_deviation = std::max(0.0, params.esdf_obstacle_max_deviation);
  const double required_clearance = footprint_aware ? footprint_clearance : minimum_clearance;
  if (input_waypoints.size() < 2 || required_clearance <= 0.0 || maximum_step <= 0.0) {
    return input_waypoints;
  }

  const std::vector<Point> original_waypoints = densifyWaypoints(
    input_waypoints, std::max(0.05, params.esdf_obstacle_control_point_spacing));
  std::vector<Point> waypoints = original_waypoints;
  const double reference_speed = std::max(0.05, params.reference_speed);
  const double sample_dt = std::max(0.02, params.sample_spacing * 0.5) / reference_speed;
  const std::vector<Point> footprint_samples = footprint_aware ?
    makeRectangularFootprintSamples(
    params.footprint_length, params.footprint_width, params.footprint_safety_margin,
    params.esdf_footprint_sample_spacing) : std::vector<Point> {};

  for (int iteration = 0; iteration < std::max(0, params.esdf_obstacle_max_iterations);
    ++iteration)
  {
    const Eigen::VectorXd durations = allocateDurations(
      waypoints, reference_speed, std::max(0.01, params.min_segment_time));
    MincoS3 minco;
    if (!solveMinco(waypoints, durations, minco)) {
      break;
    }
    const double total_duration = std::max(1e-6, durations.sum());
    double elapsed_duration = 0.0;

    std::vector<Point> corrections(waypoints.size(), Point::Zero());
    std::vector<double> weights(waypoints.size(), 0.0);
    bool needs_correction = false;
    for (int piece = 0; piece < minco.pieceCount(); ++piece) {
      const double duration = minco.pieceDuration(piece);
      const int steps = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
      for (int step = 1; step < steps; ++step) {
        const double fraction = static_cast<double>(step) / steps;
        const MincoSample sample = minco.sample(piece, duration * fraction);
        trajectory_optimizer::EsdfQueryResult query;
        const bool query_ok = footprint_aware ? queryFootprintEsdf(
          *esdf, sample.position,
          interpolateReferenceYaw(*footprint_orientation,
          (elapsed_duration + duration * fraction) / total_duration),
          footprint_samples, query) :
          esdf->query(sample.position.x(), sample.position.y(), query);
        if (!query_ok || query.distance >= required_clearance ||
          query.gradient.squaredNorm() < 1e-10)
        {
          continue;
        }

        const Point correction = query.gradient.normalized() * std::min(
          maximum_step, 0.5 * (required_clearance - query.distance));
        const std::size_t start_index = static_cast<std::size_t>(piece);
        const std::size_t end_index = start_index + 1U;
        corrections[start_index] += (1.0 - fraction) * correction;
        corrections[end_index] += fraction * correction;
        weights[start_index] += 1.0 - fraction;
        weights[end_index] += fraction;
        needs_correction = true;
      }
      elapsed_duration += duration;
    }
    if (!needs_correction) {
      break;
    }

    bool changed = false;
    for (std::size_t index = 1; index + 1 < waypoints.size(); ++index) {
      if (weights[index] <= 1e-9) {
        continue;
      }
      const Point step = limitNorm(corrections[index] / weights[index], maximum_step);
      Point candidate = waypoints[index] + step;
      candidate = original_waypoints[index] + limitNorm(
        candidate - original_waypoints[index], maximum_deviation);
      if ((candidate - waypoints[index]).norm() > 1e-6) {
        waypoints[index] = candidate;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
  return waypoints;
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

bool MincoTrajectoryOptimizer::esdfObstacleOptimizationEnabled() const
{
  return params_.esdf_obstacle_optimization_enabled;
}

bool MincoTrajectoryOptimizer::esdfFootprintOptimizationEnabled() const
{
  return params_.esdf_obstacle_optimization_enabled && params_.esdf_footprint_optimization_enabled;
}

ReferenceTrajectory MincoTrajectoryOptimizer::optimize(
  const nav_msgs::msg::Path & raw_path,
  const trajectory_optimizer::RcTraversabilityEsdfProvider * esdf,
  const ReferenceTrajectory * footprint_orientation) const
{
  ReferenceTrajectory trajectory;
  trajectory.header = raw_path.header;
  std::vector<Point> waypoints = extractWaypoints(raw_path);
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
  waypoints = refineWaypointsWithEsdf(waypoints, params_, esdf, footprint_orientation);
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
