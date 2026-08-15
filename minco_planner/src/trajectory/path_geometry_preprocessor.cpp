// Copyright 2026

#include "minco_planner/trajectory/path_geometry_preprocessor.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

namespace
{

double normalizedTurn(const Eigen::Vector2d & incoming, const Eigen::Vector2d & outgoing)
{
  const double scale = incoming.norm() * outgoing.norm();
  if (scale <= 1e-9) {
    return 0.0;
  }
  return std::atan2(
    incoming.x() * outgoing.y() - incoming.y() * outgoing.x(), incoming.dot(outgoing));
}

}  // namespace

PathGeometryPreprocessor::PathGeometryPreprocessor(PathGeometryPreprocessorParams params)
: params_(params)
{
}

void PathGeometryPreprocessor::setParams(const PathGeometryPreprocessorParams & params)
{
  params_ = params;
}

PathGeometryResult PathGeometryPreprocessor::preprocess(
  const nav_msgs::msg::Path & raw_path,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker) const
{
  PathGeometryResult result;
  std::vector<Eigen::Vector2d> unique;
  unique.reserve(raw_path.poses.size());
  const double duplicate_epsilon = std::max(1e-9, params_.duplicate_epsilon);
  for (const auto & pose : raw_path.poses) {
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (unique.empty() || (point - unique.back()).norm() > duplicate_epsilon) {
      unique.push_back(point);
    } else {
      ++result.duplicate_points_removed;
    }
  }
  if (unique.size() <= 2U) {
    result.waypoints = unique;
    result.corner_waypoints.assign(unique.size(), false);
    result.guide_path = makePath(raw_path.header, unique);
    return result;
  }

  std::vector<Eigen::Vector2d> simplified;
  simplified.reserve(unique.size());
  simplified.push_back(unique.front());
  for (std::size_t index = 1; index + 1U < unique.size(); ++index) {
    const Eigen::Vector2d & previous = simplified.back();
    const Eigen::Vector2d & current = unique[index];
    const Eigen::Vector2d & next = unique[index + 1U];
    const Eigen::Vector2d direct = next - previous;
    const Eigen::Vector2d incoming = current - previous;
    const Eigen::Vector2d outgoing = next - current;
    const double direct_length = direct.norm();
    const double lateral = direct_length > 1e-9 ?
      std::abs(direct.x() * incoming.y() - direct.y() * incoming.x()) / direct_length : 0.0;
    const bool forward_collinear = incoming.dot(outgoing) > 0.0 &&
      lateral <= std::max(0.0, params_.collinear_lateral_tolerance);
    const bool short_segment = std::min(incoming.norm(), outgoing.norm()) <=
      std::max(0.0, params_.short_segment_length);
    if (forward_collinear) {
      ++result.collinear_points_removed;
      continue;
    }
    // A short corner may be merged only if the exact directed rectangular
    // shortcut is safe.  Without the immutable grid it is intentionally kept.
    if (short_segment && shortcutSafe(previous, next, planning_grid, safety_checker)) {
      ++result.short_segments_merged;
      continue;
    }
    simplified.push_back(current);
  }
  simplified.push_back(unique.back());

  std::vector<Eigen::Vector2d> shortcut;
  shortcut.reserve(simplified.size());
  std::size_t anchor = 0;
  shortcut.push_back(simplified.front());
  while (anchor + 1U < simplified.size()) {
    std::size_t next = anchor + 1U;
    if (params_.footprint_aware_shortcut_enabled && planning_grid && safety_checker) {
      for (std::size_t candidate = simplified.size() - 1U; candidate > anchor + 1U; --candidate) {
        if (shortcutSafe(simplified[anchor], simplified[candidate], planning_grid, safety_checker)) {
          next = candidate;
          break;
        }
      }
    }
    result.shortcut_waypoints_removed += next - anchor - 1U;
    shortcut.push_back(simplified[next]);
    anchor = next;
  }

  result.waypoints = std::move(shortcut);
  result.corner_waypoints.assign(result.waypoints.size(), false);
  for (std::size_t index = 1; index + 1U < result.waypoints.size(); ++index) {
    const double turn = normalizedTurn(
      result.waypoints[index] - result.waypoints[index - 1U],
      result.waypoints[index + 1U] - result.waypoints[index]);
    result.corner_waypoints[index] = std::abs(turn) >=
      std::max(0.0, params_.corner_angle_threshold_rad);
  }
  result.guide_path = makePath(raw_path.header, result.waypoints);
  return result;
}

bool PathGeometryPreprocessor::shortcutSafe(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker) const
{
  if (!planning_grid || !safety_checker || planning_grid->data.empty()) {
    return false;
  }
  const double yaw = std::atan2(end.y() - start.y(), end.x() - start.x());
  ReferenceTrajectory shortcut;
  shortcut.header = planning_grid->header;
  ReferencePoint first;
  first.x = start.x();
  first.y = start.y();
  first.yaw = yaw;
  first.t = 0.0;
  ReferencePoint last = first;
  last.x = end.x();
  last.y = end.y();
  last.t = std::max(1e-3, (end - start).norm());
  shortcut.points = {first, last};
  return safety_checker->check(shortcut, *planning_grid).safe;
}

nav_msgs::msg::Path PathGeometryPreprocessor::makePath(
  const std_msgs::msg::Header & header, const std::vector<Eigen::Vector2d> & points)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(points.size());
  for (const Eigen::Vector2d & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace minco_planner
