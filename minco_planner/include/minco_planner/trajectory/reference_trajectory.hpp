// Copyright 2026

#ifndef MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
#define MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "std_msgs/msg/header.hpp"

namespace minco_planner
{

struct ReferencePoint
{
  double t = 0.0;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double yaw_rate = 0.0;
  // 未标注的净空是"未知"，不是"实测为 0"。默认 0.0 是有限值，会让窄通道判据
  // 把没标注过的轨迹当成贴着障碍，从而无条件进入窄通道分支。所有读取方都已
  // 用 isfinite 守卫，因此这里用 NaN 表达未知。
  double clearance = std::numeric_limits<double>::quiet_NaN();
  double slope = 0.0;
};

struct ReferenceTrajectory
{
  std_msgs::msg::Header header;
  std::vector<ReferencePoint> points;

  bool empty() const
  {
    return points.empty();
  }

  bool valid() const
  {
    if (points.size() < 2) {
      return false;
    }
    double previous_time = -std::numeric_limits<double>::infinity();
    double previous_length = -std::numeric_limits<double>::infinity();
    for (const auto & point : points) {
      if (!std::isfinite(point.t) || !std::isfinite(point.s) || !std::isfinite(point.x) ||
        !std::isfinite(point.y) || !std::isfinite(point.yaw) || !std::isfinite(point.v) ||
        !std::isfinite(point.vx) || !std::isfinite(point.vy) || !std::isfinite(point.ax) ||
        !std::isfinite(point.ay) || !std::isfinite(point.yaw_rate) ||
        point.t <= previous_time || point.s < previous_length)
      {
        return false;
      }
      previous_time = point.t;
      previous_length = point.s;
    }
    return points.back().t > points.front().t;
  }

  double totalLength() const
  {
    return points.empty() ? 0.0 : points.back().s;
  }

  double totalTime() const
  {
    return points.empty() ? 0.0 : points.back().t;
  }
};

// 从已提交参考中截出车当前跟踪位置之后的剩余段：起点取 t>=elapsed 的前一个点，
// 终点取第一个 t>=horizon_end 的点（horizon_end 为 +inf 时保留到末尾）。
// 剩余不足两个点（已跑完）时返回空轨迹。
inline ReferenceTrajectory remainingReferenceWindow(
  const ReferenceTrajectory & trajectory, double elapsed, double horizon_end)
{
  ReferenceTrajectory remaining;
  remaining.header = trajectory.header;
  const std::size_t count = trajectory.points.size();
  std::size_t first_index = count;
  for (std::size_t i = 0; i < count; ++i) {
    if (trajectory.points[i].t >= elapsed) {
      first_index = i == 0 ? 0 : i - 1;
      break;
    }
  }
  if (count < 2 || first_index >= count - 1) {
    return remaining;
  }
  for (std::size_t i = first_index; i < count; ++i) {
    remaining.points.push_back(trajectory.points[i]);
    if (trajectory.points[i].t >= horizon_end && remaining.points.size() >= 2) {
      break;
    }
  }
  return remaining;
}

// 车沿参考实际推进到的时间：在 t <= time_upper_bound 的折线段上找离 (x, y) 最近的点，返回其
// 插值时间。MPC 按几何投影跟踪，车常落后于墙钟时间（拐角减速、进度缩放），所以墙钟
// elapsed 会越过车的真实位置；只在不超过墙钟的部分里搜索，避免回环路径投影到后段。
// 参考少于两个点时返回 time_upper_bound。
inline double projectedReferenceTime(
  const ReferenceTrajectory & trajectory, double x, double y, double time_upper_bound)
{
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return time_upper_bound;
  }
  double best_distance = std::numeric_limits<double>::infinity();
  double best_time = std::min(time_upper_bound, points.front().t);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const ReferencePoint & a = points[i - 1];
    const ReferencePoint & b = points[i];
    if (a.t > time_upper_bound) {
      break;
    }
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    double u = length_squared > 1e-12 ? ((x - a.x) * dx + (y - a.y) * dy) / length_squared : 0.0;
    u = std::max(0.0, std::min(1.0, u));
    const double t = a.t + u * (b.t - a.t);
    if (t > time_upper_bound) {
      u = b.t > a.t ? (time_upper_bound - a.t) / (b.t - a.t) : 0.0;
    }
    const double px = a.x + u * dx;
    const double py = a.y + u * dy;
    const double distance = std::hypot(px - x, py - y);
    if (distance < best_distance) {
      best_distance = distance;
      best_time = std::min(time_upper_bound, a.t + u * (b.t - a.t));
    }
  }
  return best_time;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
