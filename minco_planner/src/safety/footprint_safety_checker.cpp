// Copyright 2026

#include "minco_planner/safety/footprint_safety_checker.hpp"

#include <algorithm>
#include <cmath>

#include "minco_planner/safety/footprint_samples.hpp"

namespace minco_planner
{

FootprintSafetyChecker::FootprintSafetyChecker(FootprintSafetyParams params)
: params_(params)
{
}

void FootprintSafetyChecker::setParams(const FootprintSafetyParams & params)
{
  params_ = params;
}

FootprintSafetyResult FootprintSafetyChecker::check(
  const ReferenceTrajectory & trajectory,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  FootprintSafetyResult result;
  if (!trajectory.valid() || grid.data.empty()) {
    result.safe = false;
    return result;
  }

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    ++result.discrete_samples_checked;
    double collision_x = 0.0;
    double collision_y = 0.0;
    if (!sampleFootprintOccupied(trajectory.points[i], grid, collision_x, collision_y)) {
      continue;
    }
    result.safe = false;
    CollisionSample sample;
    sample.trajectory_index = i;
    sample.segment_index = i;
    sample.x = collision_x;
    sample.y = collision_y;
    result.collisions.push_back(sample);
  }

  // A reference point check alone can skip a wall between two safe poses.  The
  // same oriented rectangular rasterization is therefore evaluated at
  // adaptively subdivided SE(2) poses.  The bound uses actual corner motion,
  // so pure rotation, lateral translation, and diagonal motion share one rule.
  for (std::size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const std::size_t subdivisions = sweptSubdivisions(
      trajectory.points[i], trajectory.points[i + 1], grid);
    ++result.swept_segments_checked;
    for (std::size_t step = 1; step < subdivisions; ++step) {
      const double fraction = static_cast<double>(step) /
        static_cast<double>(subdivisions);
      const ReferencePoint point = interpolate(
        trajectory.points[i], trajectory.points[i + 1], fraction);
      ++result.swept_samples_checked;
      double collision_x = 0.0;
      double collision_y = 0.0;
      if (!sampleFootprintOccupied(point, grid, collision_x, collision_y)) {
        continue;
      }
      result.safe = false;
      CollisionSample sample;
      sample.trajectory_index = i;
      sample.segment_index = i;
      sample.segment_fraction = fraction;
      sample.swept = true;
      sample.x = collision_x;
      sample.y = collision_y;
      result.collisions.push_back(sample);
    }
  }
  return result;
}

bool FootprintSafetyChecker::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double wx,
  double wy,
  GridIndex & out) const
{
  if (grid.info.resolution <= 0.0) {
    return false;
  }
  const double yaw = std::atan2(
    2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
      grid.info.origin.orientation.x * grid.info.origin.orientation.y),
    1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
      grid.info.origin.orientation.z * grid.info.origin.orientation.z));
  const double dx = wx - grid.info.origin.position.x;
  const double dy = wy - grid.info.origin.position.y;
  // OccupancyGrid cells live in the origin pose's local axes.  Rotate the
  // query into those axes before converting to an index.
  const double gx = (std::cos(yaw) * dx + std::sin(yaw) * dy) / grid.info.resolution;
  const double gy = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / grid.info.resolution;
  if (gx < 0.0 || gy < 0.0 ||
    gx >= static_cast<double>(grid.info.width) ||
    gy >= static_cast<double>(grid.info.height))
  {
    return false;
  }
  out.x = static_cast<int>(std::floor(gx));
  out.y = static_cast<int>(std::floor(gy));
  return true;
}

bool FootprintSafetyChecker::isOccupied(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  if (index.x < 0 || index.y < 0 ||
    index.x >= static_cast<int>(grid.info.width) ||
    index.y >= static_cast<int>(grid.info.height))
  {
    return true;
  }
  const std::size_t linear =
    static_cast<std::size_t>(index.y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(index.x);
  const int8_t value = grid.data[linear];
  if (value < 0) {
    return params_.unknown_is_obstacle;
  }
  return value >= params_.obstacle_value_threshold;
}

bool FootprintSafetyChecker::sampleFootprintOccupied(
  const ReferencePoint & point,
  const nav_msgs::msg::OccupancyGrid & grid,
  double & collision_x,
  double & collision_y) const
{
  const std::vector<Eigen::Vector2d> samples = makeRectangularFootprintSamples(
    params_.length, params_.width, params_.safety_margin, grid.info.resolution);
  const double cos_yaw = std::cos(point.yaw);
  const double sin_yaw = std::sin(point.yaw);

  for (const auto & sample : samples) {
    const double wx = point.x + cos_yaw * sample.x() - sin_yaw * sample.y();
    const double wy = point.y + sin_yaw * sample.x() + cos_yaw * sample.y();
    GridIndex index;
    if (!worldToGrid(grid, wx, wy, index) || isOccupied(grid, index)) {
      collision_x = wx;
      collision_y = wy;
      return true;
    }
  }
  return false;
}

std::size_t FootprintSafetyChecker::sweptSubdivisions(
  const ReferencePoint & start, const ReferencePoint & end,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const double half_length = 0.5 * std::max(0.0, params_.length) +
    std::max(0.0, params_.safety_margin);
  const double half_width = 0.5 * std::max(0.0, params_.width) +
    std::max(0.0, params_.safety_margin);
  const std::vector<Eigen::Vector2d> corners = {
    Eigen::Vector2d(-half_length, -half_width),
    Eigen::Vector2d(-half_length, half_width),
    Eigen::Vector2d(half_length, -half_width),
    Eigen::Vector2d(half_length, half_width)};
  const double start_cos = std::cos(start.yaw);
  const double start_sin = std::sin(start.yaw);
  const double end_cos = std::cos(end.yaw);
  const double end_sin = std::sin(end.yaw);
  double maximum_corner_displacement = 0.0;
  for (const auto & corner : corners) {
    const Eigen::Vector2d start_corner(
      start.x + start_cos * corner.x() - start_sin * corner.y(),
      start.y + start_sin * corner.x() + start_cos * corner.y());
    const Eigen::Vector2d end_corner(
      end.x + end_cos * corner.x() - end_sin * corner.y(),
      end.y + end_sin * corner.x() + end_cos * corner.y());
    maximum_corner_displacement = std::max(
      maximum_corner_displacement, (end_corner - start_corner).norm());
  }
  const double maximum_step = std::max(
    1e-6, static_cast<double>(grid.info.resolution) *
    std::max(1e-3, params_.swept_max_corner_step_cells));
  return std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(maximum_corner_displacement / maximum_step)));
}

ReferencePoint FootprintSafetyChecker::interpolate(
  const ReferencePoint & start, const ReferencePoint & end, double fraction)
{
  const double clamped_fraction = std::max(0.0, std::min(1.0, fraction));
  ReferencePoint result = start;
  result.x = start.x + (end.x - start.x) * clamped_fraction;
  result.y = start.y + (end.y - start.y) * clamped_fraction;
  result.yaw = normalizeAngle(
    start.yaw + normalizeAngle(end.yaw - start.yaw) * clamped_fraction);
  result.t = start.t + (end.t - start.t) * clamped_fraction;
  result.s = start.s + (end.s - start.s) * clamped_fraction;
  return result;
}

double FootprintSafetyChecker::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace minco_planner
