// Copyright 2026

#ifndef ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_
#define ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ats_goal_manager
{

struct PlanningSnapshotSafetyParams
{
  double footprint_length{0.70};
  double footprint_width{0.55};
  double footprint_safety_margin{0.05};
};

struct PlanningSnapshotSafetyResult
{
  bool valid_snapshot{false};
  bool inside_map{false};
  bool robot_cell_free{false};
  bool footprint_safe{false};
};

inline bool validPlanningSnapshot(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot)
{
  const auto count = static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height;
  if (!snapshot.ready || snapshot.header.frame_id.empty() ||
    snapshot.source_generation == 0U || snapshot.publication_sequence == 0U ||
    snapshot.occupied_value_threshold <= 0 || snapshot.occupied_value_threshold > 100 ||
    !std::isfinite(snapshot.info.resolution) || snapshot.info.resolution <= 0.0F ||
    count == 0U || snapshot.occupancy.size() != count ||
    snapshot.signed_distance_m.size() != count ||
    snapshot.gradient_x.size() != count || snapshot.gradient_y.size() != count)
  {
    return false;
  }
  const auto & orientation = snapshot.info.origin.orientation;
  const double norm = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const auto occupancy = snapshot.occupancy[index];
    const auto distance = snapshot.signed_distance_m[index];
    const auto gradient_x = snapshot.gradient_x[index];
    const auto gradient_y = snapshot.gradient_y[index];
    if (occupancy < -1 || occupancy > 100) {
      return false;
    }
    if (occupancy == -1) {
      if (std::isfinite(distance) || std::isfinite(gradient_x) ||
        std::isfinite(gradient_y))
      {
        return false;
      }
      continue;
    }
    if (!std::isfinite(distance) || !std::isfinite(gradient_x) ||
      !std::isfinite(gradient_y) ||
      (occupancy >= snapshot.occupied_value_threshold && distance > 0.0F) ||
      (occupancy < snapshot.occupied_value_threshold && distance < 0.0F))
    {
      return false;
    }
  }
  return true;
}

inline bool worldToSnapshotCell(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  double world_x, double world_y, int & cell_x, int & cell_y)
{
  const auto & origin = snapshot.info.origin;
  const double yaw = std::atan2(
    2.0 * (origin.orientation.w * origin.orientation.z +
      origin.orientation.x * origin.orientation.y),
    1.0 - 2.0 * (origin.orientation.y * origin.orientation.y +
      origin.orientation.z * origin.orientation.z));
  const double dx = world_x - origin.position.x;
  const double dy = world_y - origin.position.y;
  const double grid_x =
    (std::cos(yaw) * dx + std::sin(yaw) * dy) / snapshot.info.resolution;
  const double grid_y =
    (-std::sin(yaw) * dx + std::cos(yaw) * dy) / snapshot.info.resolution;
  if (grid_x < 0.0 || grid_y < 0.0 ||
    grid_x >= static_cast<double>(snapshot.info.width) ||
    grid_y >= static_cast<double>(snapshot.info.height))
  {
    return false;
  }
  cell_x = static_cast<int>(std::floor(grid_x));
  cell_y = static_cast<int>(std::floor(grid_y));
  return true;
}

inline bool snapshotCellFree(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  int cell_x, int cell_y)
{
  if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(snapshot.info.width) ||
    cell_y >= static_cast<int>(snapshot.info.height))
  {
    return false;
  }
  const auto index = static_cast<std::size_t>(cell_y) * snapshot.info.width +
    static_cast<std::size_t>(cell_x);
  const int value = snapshot.occupancy[index];
  // P2 always treats unknown as an obstacle at the task-level restart gate.
  return value >= 0 && value < snapshot.occupied_value_threshold;
}

inline PlanningSnapshotSafetyResult checkPlanningSnapshotFootprint(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  const geometry_msgs::msg::Pose & pose,
  const PlanningSnapshotSafetyParams & params = PlanningSnapshotSafetyParams())
{
  PlanningSnapshotSafetyResult result;
  result.valid_snapshot = validPlanningSnapshot(snapshot);
  if (!result.valid_snapshot || !std::isfinite(pose.position.x) ||
    !std::isfinite(pose.position.y))
  {
    return result;
  }
  int center_x = 0;
  int center_y = 0;
  result.inside_map = worldToSnapshotCell(
    snapshot, pose.position.x, pose.position.y, center_x, center_y);
  result.robot_cell_free = result.inside_map && snapshotCellFree(snapshot, center_x, center_y);
  if (!result.robot_cell_free) {
    return result;
  }

  const double half_length = 0.5 * std::max(0.0, params.footprint_length) +
    std::max(0.0, params.footprint_safety_margin);
  const double half_width = 0.5 * std::max(0.0, params.footprint_width) +
    std::max(0.0, params.footprint_safety_margin);
  const double spacing = std::max(0.02, static_cast<double>(snapshot.info.resolution));
  const int samples_x = std::max(2, static_cast<int>(std::ceil(2.0 * half_length / spacing)));
  const int samples_y = std::max(2, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
  const double yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int ix = 0; ix <= samples_x; ++ix) {
    const double local_x = -half_length + 2.0 * half_length * ix / samples_x;
    for (int iy = 0; iy <= samples_y; ++iy) {
      const double local_y = -half_width + 2.0 * half_width * iy / samples_y;
      int cell_x = 0;
      int cell_y = 0;
      const double world_x = pose.position.x + cosine * local_x - sine * local_y;
      const double world_y = pose.position.y + sine * local_x + cosine * local_y;
      if (!worldToSnapshotCell(snapshot, world_x, world_y, cell_x, cell_y) ||
        !snapshotCellFree(snapshot, cell_x, cell_y))
      {
        return result;
      }
    }
  }
  result.footprint_safe = true;
  return result;
}

// Commit-time revalidation on a newer immutable map. Check the whole rectangle
// against every overlapping cell, then interpolate sparse path samples so a
// new obstacle between candidate poses cannot escape the check. Unknown and
// outside-map remain forbidden even in mapping mode. This is an extra gate;
// it does not replace MINCO's original clearance/footprint validation.
inline bool planningSnapshotReferenceSafe(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  const nav_msgs::msg::Path & path,
  const PlanningSnapshotSafetyParams & params)
{
  if (!validPlanningSnapshot(snapshot) || path.poses.size() < 2U ||
    path.header.frame_id != snapshot.header.frame_id ||
    !std::isfinite(params.footprint_length) || !std::isfinite(params.footprint_width) ||
    !std::isfinite(params.footprint_safety_margin) || params.footprint_length < 0.0 ||
    params.footprint_width < 0.0 || params.footprint_safety_margin < 0.0)
  {
    return false;
  }
  const auto yawOf = [](const auto & q) {
      return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };
  const double resolution = snapshot.info.resolution;
  const double half_x = (0.5 * params.footprint_length + params.footprint_safety_margin) /
    resolution;
  const double half_y = (0.5 * params.footprint_width + params.footprint_safety_margin) /
    resolution;
  const auto & origin = snapshot.info.origin;
  const double origin_yaw = yawOf(origin.orientation);
  const double co = std::cos(origin_yaw);
  const double so = std::sin(origin_yaw);
  const auto freeRectangle = [&](double x, double y, double yaw) {
      const double dx = x - origin.position.x;
      const double dy = y - origin.position.y;
      const double cx = (co * dx + so * dy) / resolution;
      const double cy = (-so * dx + co * dy) / resolution;
      const double c = std::cos(yaw - origin_yaw);
      const double s = std::sin(yaw - origin_yaw);
      const double ac = std::abs(c);
      const double as = std::abs(s);
      const double extent_x = ac * half_x + as * half_y;
      const double extent_y = as * half_x + ac * half_y;
      if (cx - extent_x < 0.0 || cy - extent_y < 0.0 ||
        cx + extent_x >= snapshot.info.width || cy + extent_y >= snapshot.info.height)
      {
        return false;
      }
      for (int iy = static_cast<int>(std::floor(cy - extent_y));
        iy <= static_cast<int>(std::floor(cy + extent_y)); ++iy)
      {
        for (int ix = static_cast<int>(std::floor(cx - extent_x));
          ix <= static_cast<int>(std::floor(cx + extent_x)); ++ix)
        {
          const double cell_dx = ix + 0.5 - cx;
          const double cell_dy = iy + 0.5 - cy;
          // Separating axes of the oriented footprint and the axis-aligned cell.
          if (std::abs(c * cell_dx + s * cell_dy) > half_x + 0.5 * (ac + as) ||
            std::abs(-s * cell_dx + c * cell_dy) > half_y + 0.5 * (ac + as))
          {
            continue;
          }
          if (!snapshotCellFree(snapshot, ix, iy)) {
            return false;
          }
        }
      }
      return true;
    };
  double previous_x = 0.0;
  double previous_y = 0.0;
  double previous_yaw = 0.0;
  bool first = true;
  for (const auto & stamped : path.poses) {
    const auto & p = stamped.pose;
    const auto & q = p.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if ((!stamped.header.frame_id.empty() && stamped.header.frame_id != path.header.frame_id) ||
      !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    {
      return false;
    }
    const double yaw = yawOf(q);
    if (!freeRectangle(p.position.x, p.position.y, yaw)) {
      return false;
    }
    if (!first) {
      const double yaw_delta = std::atan2(
        std::sin(yaw - previous_yaw), std::cos(yaw - previous_yaw));
      const double travel = std::hypot(p.position.x - previous_x, p.position.y - previous_y) +
        resolution * std::hypot(half_x, half_y) * std::abs(yaw_delta);
      const double intervals = std::ceil(travel / (0.5 * resolution));
      if (!std::isfinite(intervals) || intervals > 1000000.0) {
        return false;
      }
      const int steps = std::max(1, static_cast<int>(intervals));
      for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        if (!freeRectangle(previous_x + t * (p.position.x - previous_x),
          previous_y + t * (p.position.y - previous_y), previous_yaw + t * yaw_delta))
        {
          return false;
        }
      }
    }
    previous_x = p.position.x;
    previous_y = p.position.y;
    previous_yaw = yaw;
    first = false;
  }
  return true;
}

// 擦边接触判定，与 minco_planner escapePrefixContactShallow 同一口径：足迹每边内缩
// max_contact_depth_m 后完全无冲突，即车体压进占据格的深度不超过该值。规划器只在这一
// 条件下提交逃逸轨迹，watchdog 若仍按原足迹急停，就会把唯一能驱动执行器的轨迹掐掉，
// 车永远停在原位（RMUC 多目标第 2 轮 (4.847,-0.800) 每 2 s 提交一次逃逸、每次被急停）。
// 深度上界 <= 0 时恒为 false，保持原 fail-closed 行为。
inline bool planningSnapshotContactShallow(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  const geometry_msgs::msg::Pose & pose,
  const PlanningSnapshotSafetyParams & params, double max_contact_depth_m)
{
  if (!(max_contact_depth_m > 0.0) || !std::isfinite(max_contact_depth_m)) {
    return false;
  }
  PlanningSnapshotSafetyParams shrunk = params;
  shrunk.footprint_length -= 2.0 * max_contact_depth_m;
  shrunk.footprint_width -= 2.0 * max_contact_depth_m;
  if (shrunk.footprint_length <= 0.0 || shrunk.footprint_width <= 0.0) {
    return false;
  }
  return checkPlanningSnapshotFootprint(snapshot, pose, shrunk).footprint_safe;
}

// 当前位姿对足迹检查的"接触深度"：足迹每边内缩多少之后才能完整通过 checkPlanningSnapshotFootprint。
// 0 = 本来就通过；+inf = 机器人格被占/越界，或内缩到 max_search_depth_m 仍不通过。
// 与 planningSnapshotContactShallow 同一口径(depth <= d 等价于 shallow(d))，二分到 resolution_m。
// 只用于观测(日志里每次挂起的深度)与有界放行，不改变 checkPlanningSnapshotFootprint 的判定。
inline double planningSnapshotContactDepth(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot,
  const geometry_msgs::msg::Pose & pose,
  const PlanningSnapshotSafetyParams & params, double max_search_depth_m = 0.30,
  double resolution_m = 0.0025)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const auto base = checkPlanningSnapshotFootprint(snapshot, pose, params);
  if (base.footprint_safe) {
    return 0.0;
  }
  if (!base.robot_cell_free) {
    return kInf;
  }
  // 内缩后的足迹必须保持正的长宽。
  const double limit = std::min(
    max_search_depth_m, 0.5 * std::min(params.footprint_length, params.footprint_width) - 1e-3);
  if (!(limit > 0.0) || !std::isfinite(limit)) {
    return kInf;
  }
  const auto passes = [&](double depth) {
      PlanningSnapshotSafetyParams shrunk = params;
      shrunk.footprint_length -= 2.0 * depth;
      shrunk.footprint_width -= 2.0 * depth;
      return checkPlanningSnapshotFootprint(snapshot, pose, shrunk).footprint_safe;
    };
  if (!passes(limit)) {
    return kInf;
  }
  double low = 0.0;
  double high = limit;
  const double step = std::max(resolution_m, 1e-4);
  while (high - low > step) {
    const double mid = 0.5 * (low + high);
    if (passes(mid)) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return high;
}

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__PLANNING_SNAPSHOT_SAFETY_HPP_
