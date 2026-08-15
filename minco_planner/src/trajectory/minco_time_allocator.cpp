// Copyright 2026

#include "minco_planner/trajectory/minco_time_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

namespace
{

double signedCurvature(
  const Eigen::Vector2d & before, const Eigen::Vector2d & current,
  const Eigen::Vector2d & after)
{
  const Eigen::Vector2d first = current - before;
  const Eigen::Vector2d second = after - current;
  const Eigen::Vector2d chord = after - before;
  const double denominator = first.norm() * second.norm() * chord.norm();
  if (denominator <= 1e-9) {
    return 0.0;
  }
  const double cross = first.x() * second.y() - first.y() * second.x();
  return 2.0 * cross / denominator;
}

bool finitePositive(const Eigen::VectorXd & values)
{
  return values.allFinite() && (values.array() > 0.0).all();
}

}  // namespace

MincoTimeAllocator::MincoTimeAllocator(MincoTimeAllocatorParams params)
: params_(params)
{
}

void MincoTimeAllocator::setParams(const MincoTimeAllocatorParams & params)
{
  params_ = params;
}

MincoTimeAllocation MincoTimeAllocator::allocate(
  const std::vector<Eigen::Vector2d> & waypoints, double initial_speed) const
{
  MincoTimeAllocation result;
  if (waypoints.size() < 2U) {
    return result;
  }
  const std::size_t count = waypoints.size();
  const double reference_speed = std::max(0.05, params_.reference_speed);
  const double max_velocity = params_.max_velocity > 0.0 ?
    params_.max_velocity : reference_speed;
  const double acceleration = std::max(1e-6, params_.max_acceleration);
  const double lateral_acceleration = std::max(1e-6, params_.max_lateral_acceleration);
  const double min_segment_time = std::max(1e-4, params_.min_segment_time);

  std::vector<double> lengths(count - 1U, 0.0);
  for (std::size_t index = 0; index + 1U < count; ++index) {
    lengths[index] = (waypoints[index + 1U] - waypoints[index]).norm();
    if (!std::isfinite(lengths[index])) {
      return result;
    }
  }
  result.signed_curvatures.assign(count, 0.0);
  for (std::size_t index = 1; index + 1U < count; ++index) {
    result.signed_curvatures[index] = signedCurvature(
      waypoints[index - 1U], waypoints[index], waypoints[index + 1U]);
  }
  result.waypoint_speeds.assign(count, std::min(reference_speed, max_velocity));
  for (std::size_t index = 1; index + 1U < count; ++index) {
    const double curvature = std::abs(result.signed_curvatures[index]);
    if (curvature > 1e-9) {
      result.waypoint_speeds[index] = std::min(
        result.waypoint_speeds[index], std::sqrt(lateral_acceleration / curvature));
    }
  }
  const double bounded_initial_speed = std::isfinite(initial_speed) ?
    std::max(0.0, initial_speed) : 0.0;
  result.waypoint_speeds.front() = std::min(result.waypoint_speeds.front(), bounded_initial_speed);
  result.waypoint_speeds.back() = 0.0;

  for (std::size_t index = 0; index + 1U < count; ++index) {
    const double reachable = std::sqrt(std::max(
      0.0, result.waypoint_speeds[index] * result.waypoint_speeds[index] +
      2.0 * acceleration * lengths[index]));
    result.waypoint_speeds[index + 1U] = std::min(result.waypoint_speeds[index + 1U], reachable);
  }
  for (std::size_t index = count - 1U; index > 0U; --index) {
    const double reachable = std::sqrt(std::max(
      0.0, result.waypoint_speeds[index] * result.waypoint_speeds[index] +
      2.0 * acceleration * lengths[index - 1U]));
    result.waypoint_speeds[index - 1U] = std::min(result.waypoint_speeds[index - 1U], reachable);
  }

  result.durations.resize(static_cast<int>(count - 1U));
  for (std::size_t index = 0; index + 1U < count; ++index) {
    const double speed_sum = result.waypoint_speeds[index] + result.waypoint_speeds[index + 1U];
    result.durations(static_cast<int>(index)) = std::max(
      min_segment_time, speed_sum > 1e-6 ? 2.0 * lengths[index] / speed_sum : min_segment_time);
  }
  result.valid = finitePositive(result.durations);
  return result;
}

bool MincoTimeAllocator::applyLocalDynamicScaling(
  Eigen::VectorXd & durations,
  const std::vector<double> & segment_peak_velocities,
  const std::vector<double> & segment_peak_accelerations,
  const std::vector<double> & segment_peak_jerks,
  double max_velocity,
  double max_acceleration,
  double max_jerk) const
{
  if (durations.size() <= 0 || !finitePositive(durations) ||
    static_cast<std::size_t>(durations.size()) != segment_peak_velocities.size() ||
    segment_peak_velocities.size() != segment_peak_accelerations.size() ||
    segment_peak_velocities.size() != segment_peak_jerks.size())
  {
    return false;
  }
  std::vector<double> requested_scales(static_cast<std::size_t>(durations.size()), 1.0);
  const int duration_count = static_cast<int>(durations.size());
  for (int index = 0; index < durations.size(); ++index) {
    double scale = 1.0;
    if (max_velocity > 0.0 && segment_peak_velocities[static_cast<std::size_t>(index)] > max_velocity) {
      scale = std::max(scale, segment_peak_velocities[static_cast<std::size_t>(index)] / max_velocity);
    }
    if (max_acceleration > 0.0 &&
      segment_peak_accelerations[static_cast<std::size_t>(index)] > max_acceleration)
    {
      scale = std::max(scale, std::sqrt(
        segment_peak_accelerations[static_cast<std::size_t>(index)] / max_acceleration));
    }
    if (max_jerk > 0.0 && segment_peak_jerks[static_cast<std::size_t>(index)] > max_jerk) {
      scale = std::max(scale, std::cbrt(segment_peak_jerks[static_cast<std::size_t>(index)] / max_jerk));
    }
    if (scale > 1.0 + 1e-6) {
      const double bounded_scale = std::max(1.01, scale);
      // S3 continuity couples a violating segment to the segment on either
      // side. Scale that compact neighbourhood together; remote straight
      // segments remain untouched.
      const int first_affected = std::max(0, index - 1);
      const int last_affected = std::min(duration_count - 1, index + 1);
      for (int affected = first_affected; affected <= last_affected; ++affected) {
        requested_scales[static_cast<std::size_t>(affected)] = std::max(
          requested_scales[static_cast<std::size_t>(affected)], bounded_scale);
      }
    }
  }
  bool changed = false;
  for (int index = 0; index < durations.size(); ++index) {
    const double scale = requested_scales[static_cast<std::size_t>(index)];
    if (scale > 1.0 + 1e-6) {
      durations(index) *= scale;
      changed = true;
    }
  }
  return changed && finitePositive(durations);
}

}  // namespace minco_planner
