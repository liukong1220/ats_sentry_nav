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
  result.waypoints = insertInnerCornerFillets(
    result.waypoints, planning_grid, safety_checker);
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

std::vector<Eigen::Vector2d> PathGeometryPreprocessor::insertInnerCornerFillets(
  const std::vector<Eigen::Vector2d> & waypoints,
  const nav_msgs::msg::OccupancyGrid * planning_grid,
  const FootprintSafetyChecker * safety_checker) const
{
  const double requested_radius = std::max(0.0, params_.fillet_radius);
  if (requested_radius <= 1e-6 || waypoints.size() < 3U) {
    return waypoints;
  }
  const double angle_threshold = std::max(0.0, params_.corner_angle_threshold_rad);
  const int arc_samples = std::max(1, params_.fillet_arc_samples);
  const double duplicate_epsilon = std::max(1e-9, params_.duplicate_epsilon);
  std::vector<Eigen::Vector2d> filleted;
  filleted.reserve(waypoints.size() + static_cast<std::size_t>(arc_samples) * waypoints.size());
  filleted.push_back(waypoints.front());
  for (std::size_t index = 1; index + 1U < waypoints.size(); ++index) {
    const Eigen::Vector2d & previous = waypoints[index - 1U];
    const Eigen::Vector2d & corner = waypoints[index];
    const Eigen::Vector2d & next = waypoints[index + 1U];
    const Eigen::Vector2d incoming = corner - previous;
    const Eigen::Vector2d outgoing = next - corner;
    const double incoming_length = incoming.norm();
    const double outgoing_length = outgoing.norm();
    const double turn = normalizedTurn(incoming, outgoing);
    if (std::abs(turn) < angle_threshold || incoming_length <= 1e-9 || outgoing_length <= 1e-9) {
      filleted.push_back(corner);
      continue;
    }
    const double half_angle = 0.5 * std::abs(turn);
    const double tan_half = std::tan(half_angle);
    if (tan_half <= 1e-9) {
      filleted.push_back(corner);
      continue;
    }
    bool inserted = false;
    double radius = requested_radius;
    for (int attempt = 0; attempt < 3 && !inserted; ++attempt) {
      double inset = radius * tan_half;
      const double max_inset = 0.45 * std::min(incoming_length, outgoing_length);
      inset = std::min(inset, max_inset);
      if (inset < 0.04) {
        radius *= 0.5;
        continue;
      }
      const double actual_radius = inset / tan_half;
      const Eigen::Vector2d incoming_unit = incoming / incoming_length;
      const Eigen::Vector2d outgoing_unit = outgoing / outgoing_length;
      const Eigen::Vector2d tangent_in = corner - incoming_unit * inset;
      const Eigen::Vector2d tangent_out = corner + outgoing_unit * inset;
      const double sign = turn > 0.0 ? 1.0 : -1.0;
      const Eigen::Vector2d inward_normal(-incoming_unit.y() * sign, incoming_unit.x() * sign);
      const Eigen::Vector2d center = tangent_in + inward_normal * actual_radius;
      const Eigen::Vector2d first_radius = tangent_in - center;
      const Eigen::Vector2d second_radius = tangent_out - center;
      const double first_angle = std::atan2(first_radius.y(), first_radius.x());
      const double second_angle = std::atan2(second_radius.y(), second_radius.x());
      const double pi = std::acos(-1.0);
      double sweep = second_angle - first_angle;
      while (sweep > pi) {
        sweep -= 2.0 * pi;
      }
      while (sweep < -pi) {
        sweep += 2.0 * pi;
      }
      std::vector<Eigen::Vector2d> arc;
      arc.push_back(tangent_in);
      for (int sample = 1; sample <= arc_samples; ++sample) {
        const double fraction = static_cast<double>(sample) / static_cast<double>(arc_samples + 1);
        const double angle = first_angle + sweep * fraction;
        arc.push_back(
          center + actual_radius * Eigen::Vector2d(std::cos(angle), std::sin(angle)));
      }
      arc.push_back(tangent_out);
      bool safe = true;
      Eigen::Vector2d previous_point = filleted.back();
      for (std::size_t arc_index = 0; arc_index < arc.size(); ++arc_index) {
        if (planning_grid && safety_checker &&
          !shortcutSafe(previous_point, arc[arc_index], planning_grid, safety_checker))
        {
          safe = false;
          break;
        }
        previous_point = arc[arc_index];
      }
      if (!safe) {
        radius *= 0.5;
        continue;
      }
      for (std::size_t arc_index = 0; arc_index < arc.size(); ++arc_index) {
        if ((arc[arc_index] - filleted.back()).norm() > duplicate_epsilon) {
          filleted.push_back(arc[arc_index]);
        }
      }
      inserted = true;
    }
    if (!inserted) {
      filleted.push_back(corner);
    }
  }
  if ((waypoints.back() - filleted.back()).norm() > duplicate_epsilon) {
    filleted.push_back(waypoints.back());
  }
  return filleted;
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
