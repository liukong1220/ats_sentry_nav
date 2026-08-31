// Copyright 2026

#ifndef MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
#define MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_

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

}  // namespace minco_planner

#endif  // MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
