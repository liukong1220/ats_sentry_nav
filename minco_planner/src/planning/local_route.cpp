// Copyright 2026

#include "minco_planner/planning/local_route.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/safety/goal_pose_admission.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{
namespace
{

std::size_t nearestPose(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & point)
{
  std::size_t nearest = 0U;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double squared = (p.x - point.x) * (p.x - point.x) + (p.y - point.y) * (p.y - point.y);
    if (squared <= distance) {
      nearest = i;
      distance = squared;
    }
  }
  return nearest;
}

}  // namespace

ReferenceTrajectory traceGuide(const nav_msgs::msg::Path & path)
{
  ReferenceTrajectory trace;
  trace.header = path.header;
  trace.points.reserve(path.poses.size());
  double length = 0.0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    const auto & previous = path.poses[i == 0 ? i : i - 1].pose.position;
    const auto & next = path.poses[std::min(i + 1U, path.poses.size() - 1U)].pose.position;
    length += std::hypot(pose.position.x - previous.x, pose.position.y - previous.y);
    ReferencePoint point;
    point.t = static_cast<double>(i);
    point.s = length;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.yaw = (i == 0 || i + 1U == path.poses.size()) ? tf2::getYaw(pose.orientation) :
      std::atan2(next.y - previous.y, next.x - previous.x);
    trace.points.push_back(point);
  }
  return trace;
}

nav_msgs::msg::Path remainingGuide(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & start,
  double max_attachment_distance)
{
  nav_msgs::msg::Path remaining;
  remaining.header = path.header;
  if (path.poses.empty() || start.header.frame_id != path.header.frame_id) {
    return remaining;
  }
  const auto & current = start.pose.position;
  const auto resume = nearestPose(path, current);
  const auto & anchor = path.poses[resume].pose.position;
  const double distance = std::hypot(current.x - anchor.x, current.y - anchor.y);
  if (distance > max_attachment_distance) {
    return remaining;
  }
  remaining.poses.push_back(start);
  const auto first = resume + (distance <= 1e-6 ? 1U : 0U);
  remaining.poses.insert(remaining.poses.end(), path.poses.begin() + first, path.poses.end());
  return remaining;
}

std::optional<RouteObstruction> obstructedGuide(
  const ReferenceTrajectory & guide, const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyChecker & checker, const nav_msgs::msg::OccupancyGrid * accepted_grid)
{
  if (guide.points.size() < 2U) {
    return std::nullopt;
  }
  nav_msgs::msg::OccupancyGrid newly_blocked;
  const auto * obstruction_grid = &grid;
  if (accepted_grid && accepted_grid->header.frame_id == grid.header.frame_id &&
    accepted_grid->info.resolution > 0.0 && !accepted_grid->data.empty())
  {
    // A graph guide uses cell centers and its tangent yaw can intersect an
    // old wall at a corner. Only newly blocked cells trigger a guide detour.
    // The bypass and the executable reference are checked on the full grid.
    newly_blocked = grid;
    const auto & params = checker.params();
    const bool exact_layout =
      accepted_grid->info.origin == grid.info.origin &&
      accepted_grid->info.resolution == grid.info.resolution &&
      accepted_grid->info.width == grid.info.width &&
      accepted_grid->info.height == grid.info.height &&
      accepted_grid->data.size() == grid.data.size();
    if (exact_layout) {
      for (std::size_t i = 0; i < grid.data.size(); ++i) {
        const auto previous = accepted_grid->data[i];
        if (previous < 0 ? params.unknown_is_obstacle : previous >= params.obstacle_value_threshold) {
          newly_blocked.data[i] = 0;
        }
      }
    } else {
      const double grid_yaw = tf2::getYaw(grid.info.origin.orientation);
      const double cg = std::cos(grid_yaw);
      const double sg = std::sin(grid_yaw);
      const double acc_yaw = tf2::getYaw(accepted_grid->info.origin.orientation);
      const double ca = std::cos(acc_yaw);
      const double sa = std::sin(acc_yaw);
      const double grid_res = grid.info.resolution;
      const double acc_res = accepted_grid->info.resolution;
      for (std::uint32_t gy = 0; gy < grid.info.height; ++gy) {
        for (std::uint32_t gx = 0; gx < grid.info.width; ++gx) {
          const std::size_t idx =
            static_cast<std::size_t>(gy) * grid.info.width + gx;
          const double lx = (static_cast<double>(gx) + 0.5) * grid_res;
          const double ly = (static_cast<double>(gy) + 0.5) * grid_res;
          const double wx = grid.info.origin.position.x + cg * lx - sg * ly;
          const double wy = grid.info.origin.position.y + sg * lx + cg * ly;
          const double dx = wx - accepted_grid->info.origin.position.x;
          const double dy = wy - accepted_grid->info.origin.position.y;
          const int ax = static_cast<int>(std::floor((ca * dx + sa * dy) / acc_res));
          const int ay = static_cast<int>(std::floor((-sa * dx + ca * dy) / acc_res));
          if (ax < 0 || ay < 0 ||
            ax >= static_cast<int>(accepted_grid->info.width) ||
            ay >= static_cast<int>(accepted_grid->info.height))
          {
            continue;
          }
          const auto previous = accepted_grid->data[
            static_cast<std::size_t>(ay) * accepted_grid->info.width +
            static_cast<std::size_t>(ax)];
          if (previous < 0 ? params.unknown_is_obstacle : previous >= params.obstacle_value_threshold) {
            newly_blocked.data[idx] = 0;
          }
        }
      }
    }
    obstruction_grid = &newly_blocked;
  }
  const auto safety = checker.check(guide, *obstruction_grid);
  std::optional<RouteObstruction> obstruction;
  for (const auto & sample : safety.collisions) {
    const auto first = sample.swept ? sample.segment_index : sample.trajectory_index;
    const auto last = sample.swept ? first + 1U : first;
    if (!obstruction) {
      obstruction = RouteObstruction{first, last};
    } else {
      obstruction->first = std::min(obstruction->first, first);
      obstruction->last = std::max(obstruction->last, last);
    }
  }
  return obstruction;
}

LocalRouteResult rejoinGlobalRoute(
  const nav_msgs::msg::Path & remaining, const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<double> & blocked_squared, double clearance,
  const FootprintSafetyChecker & checker, GridSearch & search,
  const std::optional<geometry_msgs::msg::Point> & reference_conflict,
  const nav_msgs::msg::OccupancyGrid * accepted_grid,
  const LocalRouteParams & params)
{
  const double anchor_offset = std::max(0.0, params.anchor_offset);
  const double bypass_padding = std::max(0.0, params.bypass_padding);
  const double max_local_length = std::max(0.1, params.max_local_length);

  LocalRouteResult result;
  result.route.clearance = clearance;
  if (remaining.poses.size() < 2U || remaining.header.frame_id != grid.header.frame_id) {
    result.route.reason = "cannot attach to the saved route";
    return result;
  }
  const auto guide = traceGuide(remaining);
  result.obstruction = obstructedGuide(guide, grid, checker, accepted_grid);
  if (reference_conflict) {
    const auto blocked = nearestPose(remaining, *reference_conflict);
    if (!result.obstruction) {
      result.obstruction = RouteObstruction{blocked, blocked};
    } else {
      result.obstruction->first = std::min(result.obstruction->first, blocked);
      result.obstruction->last = std::max(result.obstruction->last, blocked);
    }
  }
  if (!result.obstruction) {
    result.route.success = true;
    result.route.reason = "cached route free";
    result.route.path = remaining;
    result.route.length = guide.totalLength();
    return result;
  }

  const auto & points = guide.points;
  std::size_t before = result.obstruction->first;
  std::size_t after = result.obstruction->last;
  const double first_blocked_s = points[before].s;
  const double last_blocked_s = points[after].s;
  while (before > 0 && first_blocked_s - points[before].s < anchor_offset) {
    --before;
  }
  while (after + 1U < points.size() && points[after].s - last_blocked_s < anchor_offset) {
    ++after;
  }
  if (before >= result.obstruction->first || after <= result.obstruction->last ||
    points[after].s - points[before].s > max_local_length ||
    !goalPoseFootprintFree(points[before].x, points[before].y, points[before].yaw, grid,
    checker.params()) ||
    !goalPoseFootprintFree(points[after].x, points[after].y, points[after].yaw, grid,
    checker.params()))
  {
    result.route.reason = "no traversable local rejoin anchors";
    return result;
  }

  GridSearchBounds bounds{points[before].x, points[before].y, points[before].x, points[before].y};
  for (std::size_t i = before + 1U; i <= after; ++i) {
    bounds.min_x = std::min(bounds.min_x, points[i].x);
    bounds.min_y = std::min(bounds.min_y, points[i].y);
    bounds.max_x = std::max(bounds.max_x, points[i].x);
    bounds.max_y = std::max(bounds.max_y, points[i].y);
  }
  bounds.min_x -= bypass_padding;
  bounds.min_y -= bypass_padding;
  bounds.max_x += bypass_padding;
  bounds.max_y += bypass_padding;
  const auto & saved = remaining.poses;

  // A bypass introduces new bends. Net clearance for the corners of the
  // rotated rectangle helps the grid path pass the swept check at those bends.
  const auto & footprint = checker.params();
  const double corner_clearance = std::hypot(
    0.5 * footprint.length + footprint.safety_margin,
    0.5 * footprint.width + footprint.safety_margin) +
    gridQuantizationAllowance(grid.info.resolution);
  std::vector<double> clearances{clearance};
  if (corner_clearance > clearance + 1e-9) {
    clearances.push_back(corner_clearance);
  }

  for (const double search_clearance : clearances) {
    GridSearchOptions options;
    options.clearance = search_clearance;
    options.bounds = bounds;
    options.strict_endpoints = true;
    const auto bypass = search.plan(grid, saved[before], saved[after], &blocked_squared, options);
    result.route.expanded_nodes += bypass.expanded_nodes;
    if (!bypass.success || bypass.path.poses.size() < 2U) {
      if (result.route.reason.empty()) {
        result.route.reason = "local bypass failed: " + bypass.reason;
      }
      break;
    }

    nav_msgs::msg::Path joined;
    joined.header = grid.header;
    joined.poses.insert(joined.poses.end(), saved.begin(), saved.begin() + before + 1U);
    const auto & detour = bypass.path.poses;
    joined.poses.insert(joined.poses.end(), std::next(detour.begin()), std::prev(detour.end()));
    joined.poses.insert(joined.poses.end(), saved.begin() + after, saved.end());
    const auto joined_trace = traceGuide(joined);
    ReferenceTrajectory joined_segment;
    joined_segment.header = grid.header;
    const auto & joined_points = joined_trace.points;
    joined_segment.points.assign(
      joined_points.begin() + before, joined_points.begin() + before + detour.size());
    if (obstructedGuide(joined_segment, grid, checker) ||
      obstructedGuide(joined_trace, grid, checker, accepted_grid))
    {
      result.route.reason = "rejoined route fails the swept footprint check";
      continue;
    }
    result.route.success = true;
    result.route.reason = "local bypass rejoined";
    result.route.length = joined_trace.totalLength();
    result.route.path = std::move(joined);
    result.bypass_length = bypass.length;
    result.rejoin = saved[after].pose.position;
    break;
  }
  return result;
}

}  // namespace minco_planner
