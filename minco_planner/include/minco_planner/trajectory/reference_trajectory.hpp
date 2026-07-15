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
  double clearance = 0.0;
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
