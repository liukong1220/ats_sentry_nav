// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__LOCAL_TIME_SCALING_HPP_
#define MINCO_PLANNER__TRAJECTORY__LOCAL_TIME_SCALING_HPP_

#include <algorithm>
#include <vector>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

// 按逐点速度系数 factors（(0, 1]，1 表示原速）对轨迹做局部放慢：位置、yaw 序列不变，
// 速度、角速度按系数 f 缩放，加速度按 f^2 缩放（忽略 df/dt 项，f 变化率已受限）。
// 系数先经前向/反向两遍变成斜率受限（change_rate，1/s）的下包络：提前减速、之后逐步恢复。
// factors 长度必须与轨迹点数一致，否则不修改。返回是否修改了轨迹。
inline bool applyLocalTimeScaling(
  ReferenceTrajectory & trajectory, std::vector<double> factors, double change_rate)
{
  const std::size_t count = trajectory.points.size();
  if (count < 2 || factors.size() != count) {
    return false;
  }
  bool limited = false;
  for (double & factor : factors) {
    factor = std::max(1e-3, std::min(1.0, factor));
    limited = limited || factor < 1.0;
  }
  if (!limited) {
    return false;
  }

  const double rate = std::max(1e-3, change_rate);
  for (std::size_t i = 1; i < count; ++i) {
    const double dt = std::max(0.0, trajectory.points[i].t - trajectory.points[i - 1].t);
    factors[i] = std::min(factors[i], factors[i - 1] + rate * dt);
  }
  for (std::size_t i = count - 1; i > 0; --i) {
    const double dt = std::max(0.0, trajectory.points[i].t - trajectory.points[i - 1].t);
    factors[i - 1] = std::min(factors[i - 1], factors[i] + rate * dt);
  }

  double previous_old_t = trajectory.points.front().t;
  double previous_new_t = previous_old_t;
  for (std::size_t i = 0; i < count; ++i) {
    auto & point = trajectory.points[i];
    const double old_t = point.t;
    if (i > 0) {
      const double mean_factor = 0.5 * (factors[i] + factors[i - 1]);
      previous_new_t += (old_t - previous_old_t) / mean_factor;
    }
    previous_old_t = old_t;
    const double f = factors[i];
    point.t = previous_new_t;
    point.v *= f;
    point.vx *= f;
    point.vy *= f;
    point.yaw_rate *= f;
    point.ax *= f * f;
    point.ay *= f * f;
  }
  return true;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__LOCAL_TIME_SCALING_HPP_
