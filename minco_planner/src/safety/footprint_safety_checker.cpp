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
  if (trajectory.points.empty() || grid.data.empty()) {
    return result;
  }

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    double collision_x = 0.0;
    double collision_y = 0.0;
    if (!sampleFootprintOccupied(trajectory.points[i], grid, collision_x, collision_y)) {
      continue;
    }
    result.safe = false;
    CollisionSample sample;
    sample.trajectory_index = i;
    sample.x = collision_x;
    sample.y = collision_y;
    result.collisions.push_back(sample);
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
  const double gx = (wx - grid.info.origin.position.x) / grid.info.resolution;
  const double gy = (wy - grid.info.origin.position.y) / grid.info.resolution;
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

}  // namespace minco_planner
