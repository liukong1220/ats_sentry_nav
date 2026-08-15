// Copyright 2026

#include "minco_planner/trajectory/trajectory_quality_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"

namespace minco_planner
{

namespace
{

double percentile95(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = 0.95 * static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  return values[lower] + (values[upper] - values[lower]) * (position - lower);
}

double signedCurvature(const ReferencePoint & before, const ReferencePoint & current,
  const ReferencePoint & after)
{
  const Eigen::Vector2d first(current.x - before.x, current.y - before.y);
  const Eigen::Vector2d second(after.x - current.x, after.y - current.y);
  const Eigen::Vector2d chord(after.x - before.x, after.y - before.y);
  const double denominator = first.norm() * second.norm() * chord.norm();
  if (denominator <= 1e-9) {
    return 0.0;
  }
  return 2.0 * (first.x() * second.y() - first.y() * second.x()) / denominator;
}

}  // namespace

TrajectoryQualityMetrics TrajectoryQualityEvaluator::evaluate(
  const ReferenceTrajectory & trajectory,
  const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf,
  const std::vector<Eigen::Vector2d> & footprint_samples,
  const std::vector<double> & segment_durations) const
{
  TrajectoryQualityMetrics metrics;
  metrics.segment_durations = segment_durations;
  if (trajectory.points.size() < 2U) {
    return metrics;
  }
  metrics.minimum_center_clearance = std::numeric_limits<double>::infinity();
  metrics.minimum_footprint_clearance = std::numeric_limits<double>::infinity();
  const ReferencePoint & start = trajectory.points.front();
  const ReferencePoint & goal = trajectory.points.back();
  const Eigen::Vector2d direct(goal.x - start.x, goal.y - start.y);
  metrics.direct_length = direct.norm();
  double previous_time = -std::numeric_limits<double>::infinity();
  metrics.finite = true;
  metrics.strictly_monotonic_time = true;
  std::vector<double> absolute_curvatures;
  std::vector<double> signed_curvatures;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const ReferencePoint & point = trajectory.points[index];
    const bool point_finite = std::isfinite(point.t) && std::isfinite(point.s) &&
      std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.vx) &&
      std::isfinite(point.vy) && std::isfinite(point.ax) && std::isfinite(point.ay);
    metrics.finite = metrics.finite && point_finite;
    if (index > 0U && point.t <= previous_time) {
      metrics.strictly_monotonic_time = false;
    }
    previous_time = point.t;
    metrics.peak_velocity = std::max(metrics.peak_velocity, std::hypot(point.vx, point.vy));
    metrics.peak_acceleration = std::max(metrics.peak_acceleration, std::hypot(point.ax, point.ay));
    if (index > 0U) {
      const ReferencePoint & previous = trajectory.points[index - 1U];
      metrics.path_length += std::hypot(point.x - previous.x, point.y - previous.y);
      const double dt = point.t - previous.t;
      if (dt > 1e-9) {
        metrics.peak_jerk = std::max(metrics.peak_jerk,
          std::hypot(point.ax - previous.ax, point.ay - previous.ay) / dt);
      }
    }
    if (metrics.direct_length > 1e-9) {
      const Eigen::Vector2d relative(point.x - start.x, point.y - start.y);
      metrics.max_lateral_deviation = std::max(metrics.max_lateral_deviation,
        std::abs(direct.x() * relative.y() - direct.y() * relative.x()) / metrics.direct_length);
    }
    if (esdf && esdf->available()) {
      metrics.minimum_center_clearance = std::min(
        metrics.minimum_center_clearance, esdf->getDistance(point.x, point.y));
      if (!footprint_samples.empty()) {
        metrics.minimum_footprint_clearance = std::min(metrics.minimum_footprint_clearance,
          esdf->getFootprintClearance(Eigen::Vector2d(point.x, point.y), point.yaw, footprint_samples));
      }
    }
    if (std::isfinite(point.clearance)) {
      metrics.minimum_footprint_clearance = std::min(metrics.minimum_footprint_clearance, point.clearance);
    }
  }
  for (std::size_t index = 1; index + 1U < trajectory.points.size(); ++index) {
    const double curvature = signedCurvature(
      trajectory.points[index - 1U], trajectory.points[index], trajectory.points[index + 1U]);
    signed_curvatures.push_back(curvature);
    absolute_curvatures.push_back(std::abs(curvature));
    metrics.max_geometric_curvature = std::max(metrics.max_geometric_curvature, std::abs(curvature));
    const Eigen::Vector2d first(
      trajectory.points[index].x - trajectory.points[index - 1U].x,
      trajectory.points[index].y - trajectory.points[index - 1U].y);
    const Eigen::Vector2d second(
      trajectory.points[index + 1U].x - trajectory.points[index].x,
      trajectory.points[index + 1U].y - trajectory.points[index].y);
    if (first.norm() > 1e-9 && second.norm() > 1e-9) {
      metrics.total_turning_angle += std::abs(std::atan2(
        first.x() * second.y() - first.y() * second.x(), first.dot(second)));
    }
  }
  metrics.p95_geometric_curvature = percentile95(absolute_curvatures);
  int previous_sign = 0;
  for (const double curvature : signed_curvatures) {
    if (std::abs(curvature) <= 1e-6) {
      continue;
    }
    const int sign = curvature > 0.0 ? 1 : -1;
    if (previous_sign != 0 && sign != previous_sign) {
      ++metrics.curvature_sign_changes;
    }
    previous_sign = sign;
  }
  for (std::size_t index = 1; index < signed_curvatures.size(); ++index) {
    metrics.curvature_total_variation += std::abs(
      signed_curvatures[index] - signed_curvatures[index - 1U]);
  }
  metrics.length_ratio = metrics.direct_length > 1e-9 ?
    metrics.path_length / metrics.direct_length : 1.0;
  if (!std::isfinite(metrics.minimum_center_clearance)) {
    metrics.minimum_center_clearance = std::numeric_limits<double>::quiet_NaN();
  }
  if (!std::isfinite(metrics.minimum_footprint_clearance)) {
    metrics.minimum_footprint_clearance = std::numeric_limits<double>::quiet_NaN();
  }
  metrics.finite = metrics.finite && std::isfinite(metrics.path_length) &&
    std::isfinite(metrics.peak_velocity) && std::isfinite(metrics.peak_acceleration) &&
    std::isfinite(metrics.peak_jerk);
  return metrics;
}

}  // namespace minco_planner
