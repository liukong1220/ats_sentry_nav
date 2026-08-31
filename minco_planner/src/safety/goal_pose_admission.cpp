// Copyright 2026

#include "minco_planner/safety/goal_pose_admission.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

// 用两点等位姿轨迹调用同一个 checker：扫掠段长度为零，等价于纯离散足迹判定，
// 但占据阈值、unknown 语义和栅格越界处理都与提交门共用同一份实现。
ReferenceTrajectory singlePoseTrajectory(double x, double y, double yaw)
{
  ReferencePoint point;
  point.x = x;
  point.y = y;
  point.yaw = yaw;
  point.t = 0.0;
  point.s = 0.0;
  ReferenceTrajectory trajectory;
  trajectory.points.push_back(point);
  point.t = 0.1;
  trajectory.points.push_back(point);
  return trajectory;
}

FootprintSafetyParams inflated(const FootprintSafetyParams & base, double extra)
{
  FootprintSafetyParams params = base;
  params.length = base.length + 2.0 * std::max(0.0, extra);
  params.width = base.width + 2.0 * std::max(0.0, extra);
  return params;
}

// 偏航候选按 |偏移| 升序：0, +d, -d, +2d, -2d ...，保证原始 goal_yaw 永远优先。
std::vector<double> yawOffsets(const GoalPoseAdmissionParams & params)
{
  std::vector<double> offsets{0.0};
  const std::size_t samples = std::max<std::size_t>(1U, params.yaw_samples);
  if (samples <= 1U) {
    return offsets;
  }
  const double limit =
    std::max(0.0, params.yaw_tolerance_rad) * std::clamp(params.yaw_shrink, 0.0, 1.0);
  if (limit <= 0.0) {
    return offsets;
  }
  const std::size_t pairs = (samples - 1U + 1U) / 2U;
  const double step = limit / static_cast<double>(std::max<std::size_t>(1U, pairs));
  for (std::size_t index = 1U; index <= pairs; ++index) {
    const double offset = step * static_cast<double>(index);
    offsets.push_back(offset);
    if (offsets.size() >= samples) {break;}
    offsets.push_back(-offset);
    if (offsets.size() >= samples) {break;}
  }
  return offsets;
}

}  // namespace

bool goalPoseFootprintFree(
  double x, double y, double yaw,
  const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyParams & footprint)
{
  FootprintSafetyChecker checker(footprint);
  return checker.check(singlePoseTrajectory(x, y, yaw), grid).safe;
}

GoalPoseAdmissionResult admitGoalPose(
  double goal_x, double goal_y, double goal_yaw,
  const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyParams & footprint,
  const GoalPoseAdmissionParams & params)
{
  GoalPoseAdmissionResult result;
  result.x = goal_x;
  result.y = goal_y;
  result.yaw = normalizeAngle(goal_yaw);
  if (!params.enabled) {
    return result;
  }

  const double radius_limit =
    std::max(0.0, params.position_tolerance_m) * std::clamp(params.position_shrink, 0.0, 1.0);
  const double step = std::max(1e-3, params.position_step_m);
  const std::size_t rings = static_cast<std::size_t>(std::floor(radius_limit / step));
  const std::size_t directions = std::max<std::size_t>(1U, params.position_directions);
  const std::vector<double> yaw_offsets = yawOffsets(params);

  // 两档：先要求额外膨胀余量，整轮无解才退到裸足迹。绝不放宽到比提交门更松。
  const double extras[2] = {std::max(0.0, params.preferred_extra_margin_m), 0.0};
  for (std::size_t tier = 0; tier < 2U; ++tier) {
    if (tier == 1U && extras[1] >= extras[0]) {
      break;
    }
    const FootprintSafetyParams tier_footprint = inflated(footprint, extras[tier]);
    for (std::size_t ring = 0; ring <= rings; ++ring) {
      const double radius = step * static_cast<double>(ring);
      const std::size_t ring_directions = (ring == 0U) ? 1U : directions;
      for (std::size_t direction = 0; direction < ring_directions; ++direction) {
        const double bearing = (2.0 * M_PI * static_cast<double>(direction)) /
          static_cast<double>(directions);
        const double x = goal_x + radius * std::cos(bearing);
        const double y = goal_y + radius * std::sin(bearing);
        for (const double yaw_offset : yaw_offsets) {
          const double yaw = normalizeAngle(goal_yaw + yaw_offset);
          ++result.candidates_checked;
          if (!goalPoseFootprintFree(x, y, yaw, grid, tier_footprint)) {
            continue;
          }
          result.feasible = true;
          result.used_preferred_margin = extras[tier] > 0.0;
          result.x = x;
          result.y = y;
          result.yaw = yaw;
          result.position_deviation_m = radius;
          result.yaw_deviation_rad = std::abs(yaw_offset);
          result.relocated = radius > 0.0 || std::abs(yaw_offset) > 0.0;
          return result;
        }
      }
    }
  }
  return result;
}

}  // namespace minco_planner
