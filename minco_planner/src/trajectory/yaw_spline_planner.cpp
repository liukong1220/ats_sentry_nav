// Copyright 2026

#include "minco_planner/trajectory/yaw_spline_planner.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

YawSplinePlanner::YawSplinePlanner(YawSplinePlannerParams params) : params_(params) {}

void YawSplinePlanner::setParams(const YawSplinePlannerParams & params) { params_ = params; }

void YawSplinePlanner::apply(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }
  if (params_.mode == "path_tangent") {
    applyPathTangent(trajectory, initial_yaw);
    return;
  }
  if (params_.mode == "clearance_aware") {
    applyClearanceAware(trajectory, initial_yaw, goal_yaw);
    return;
  }
  if (params_.mode == "hold") {
    for (auto & point : trajectory.points) {
      point.yaw = normalizeAngle(initial_yaw);
      point.yaw_rate = 0.0;
    }
    return;
  }
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);
}

void YawSplinePlanner::applyClearanceAware(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);
  if (trajectory.points.size() < 2) {
    return;
  }

  std::vector<double> open_area_yaws;
  open_area_yaws.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    open_area_yaws.push_back(point.yaw);
  }

  const double enter_clearance = std::max(0.0, params_.narrow_clearance_enter);
  const double exit_clearance = std::max(enter_clearance, params_.narrow_clearance_exit);
  bool narrow = false;
  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    auto & current = trajectory.points[i];
    if (std::isfinite(current.clearance)) {
      narrow = narrow ? current.clearance < exit_clearance : current.clearance <= enter_clearance;
    }

    double desired_yaw = open_area_yaws[i];
    if (narrow) {
      const std::size_t previous_index = i - 1;
      const std::size_t next_index = std::min(i + 1, trajectory.points.size() - 1);
      const double tangent_x =
        trajectory.points[next_index].x - trajectory.points[previous_index].x;
      const double tangent_y =
        trajectory.points[next_index].y - trajectory.points[previous_index].y;
      if (std::hypot(tangent_x, tangent_y) > 1e-6) {
        const double forward_yaw = std::atan2(tangent_y, tangent_x);
        const double reverse_yaw = normalizeAngle(forward_yaw + M_PI);
        desired_yaw = std::abs(shortestAngularDistance(previous_yaw, forward_yaw)) <=
                          std::abs(shortestAngularDistance(previous_yaw, reverse_yaw))
                        ? forward_yaw
                        : reverse_yaw;
      }
    }

    const double dt = std::max(1e-3, current.t - previous_t);
    const double max_delta = std::max(0.0, params_.yaw_rate_limit) * dt;
    const double desired_delta = shortestAngularDistance(previous_yaw, desired_yaw);
    const double clamped_delta = std::max(-max_delta, std::min(max_delta, desired_delta));
    current.yaw = normalizeAngle(previous_yaw + clamped_delta);
    current.yaw_rate = clamped_delta / dt;
    previous_yaw = current.yaw;
    previous_t = current.t;
  }
}

void YawSplinePlanner::applyGoalHeading(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  const double start = normalizeAngle(initial_yaw);
  const double delta = shortestAngularDistance(start, goal_yaw);
  double available_duration = std::max(1e-3, trajectory.totalTime());
  const double rate_limited_duration = params_.yaw_rate_limit > 1e-6
                                         ? 1.875 * std::abs(delta) / params_.yaw_rate_limit
                                         : available_duration;
  if (rate_limited_duration > available_duration && trajectory.totalTime() > 1e-6) {
    const double time_scale = rate_limited_duration / available_duration;
    for (auto & point : trajectory.points) {
      point.t *= time_scale;
      point.vx /= time_scale;
      point.vy /= time_scale;
      point.v /= time_scale;
      point.ax /= time_scale * time_scale;
      point.ay /= time_scale * time_scale;
    }
    available_duration = rate_limited_duration;
  }
  const double duration = std::max(1e-3, std::min(available_duration, rate_limited_duration));

  for (auto & point : trajectory.points) {
    const double u = std::max(0.0, std::min(1.0, point.t / duration));
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
    point.yaw = normalizeAngle(start + delta * blend);
    point.yaw_rate = delta * blend_rate;
  }
}

void YawSplinePlanner::applyPathTangent(ReferenceTrajectory & trajectory, double initial_yaw) const
{
  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & prev = trajectory.points[i - 1];
    auto & current = trajectory.points[i];
    const double tangent_yaw = std::atan2(current.y - prev.y, current.x - prev.x);
    const double dt = std::max(1e-3, current.t - previous_t);
    const double max_delta = std::max(0.0, params_.yaw_rate_limit) * dt;
    const double desired_delta = shortestAngularDistance(previous_yaw, tangent_yaw);
    const double clamped_delta = std::max(-max_delta, std::min(max_delta, desired_delta));

    current.yaw = normalizeAngle(previous_yaw + clamped_delta);
    current.yaw_rate = clamped_delta / dt;
    previous_yaw = current.yaw;
    previous_t = current.t;
  }
}

double YawSplinePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double YawSplinePlanner::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

}  // namespace minco_planner
