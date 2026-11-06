// Copyright 2026

#ifndef MINCO_PLANNER__SPIN_YAW_HPP_
#define MINCO_PLANNER__SPIN_YAW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

struct SpinYawParams
{
  double rate = 3.0;           // 自旋角速度 [rad/s]，正为逆时针。
  double min_clearance = 0.30; // 低于该足迹净空的点不自旋。
  double blend_rate_limit = 2.5;  // 退出自旋回到规划 yaw 时的最大角速度 [rad/s]。
  double initial_rate = 0.0;      // 起点已有的自旋角速度 [rad/s]；连续自旋的重规划传 rate。
  double angular_accel = 1.0e3;   // 自旋加速度上限 [rad/s^2]；默认近似瞬时。
  double stop_before_narrow_m = 0.0;   // 非净空点前这么远（弧长）停止自旋并回到规划 yaw。
  double resume_after_narrow_m = 0.0;  // 离开非净空点这么远（弧长）后才重新自旋。
};

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// 小陀螺 yaw 序列。足迹净空 > min_clearance 的点为"净空点"；某点可自旋当且仅当它是净空点、
/// 弧长前方 stop_before_narrow_m 内没有非净空点、后方 resume_after_narrow_m 内也没有。
/// 可自旋段内 yaw 以不超过 angular_accel 的角加速度加到 rate（逆时针）；其余点以不超过
/// blend_rate_limit 的角速度、angular_accel 的角加速度收敛回规划 yaw，收敛后严格跟随规划 yaw，
/// 所以窄段（隧道）内的 yaw 就是规划 yaw（含隧道轴线对齐），出窄段后再次自旋。
/// 位置、时间不变；净空 NaN 按非净空处理。返回是否有点处于自旋。
inline bool applySpinYaw(ReferenceTrajectory & trajectory, const SpinYawParams & params)
{
  auto & points = trajectory.points;
  const std::size_t n = points.size();
  if (n < 2 || !(params.rate > 0.0)) {
    return false;
  }
  std::vector<char> spin_ok(n, 0);
  double last_narrow_s = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const bool clear =
      std::isfinite(points[i].clearance) && points[i].clearance > params.min_clearance;
    if (!clear) {
      last_narrow_s = points[i].s;
    }
    spin_ok[i] = clear && points[i].s - last_narrow_s > std::max(0.0, params.resume_after_narrow_m);
  }
  double next_narrow_s = std::numeric_limits<double>::infinity();
  for (std::size_t k = n; k-- > 0;) {
    const bool clear =
      std::isfinite(points[k].clearance) && points[k].clearance > params.min_clearance;
    if (!clear) {
      next_narrow_s = points[k].s;
    }
    if (next_narrow_s - points[k].s <= std::max(0.0, params.stop_before_narrow_m)) {
      spin_ok[k] = 0;
    }
  }

  const double accel = std::max(0.0, params.angular_accel);
  const double blend = std::max(0.0, params.blend_rate_limit);
  const double initial = std::clamp(params.initial_rate, 0.0, params.rate);
  bool following = !(initial > 0.0);
  double yaw = wrapAngle(points.front().yaw);
  double rate = following ? points.front().yaw_rate : initial;
  points.front().yaw = yaw;
  if (!following) {
    points.front().yaw_rate = initial;
  }
  bool any_spin = false;
  for (std::size_t i = 1; i < n; ++i) {
    auto & point = points[i];
    const double dt = std::max(0.0, point.t - points[i - 1].t);
    const double dv = accel * dt;
    const double planned = point.yaw;
    if (spin_ok[i]) {
      following = false;
      rate = std::min(params.rate, rate + dv);
      yaw += rate * dt;
      any_spin = true;
    } else if (following) {
      yaw = planned;
      rate = point.yaw_rate;
    } else {
      const double diff = wrapAngle(planned - yaw);
      const double desired = std::copysign(
        std::min(blend, std::sqrt(2.0 * accel * std::abs(diff))), diff);
      const double command = std::clamp(desired, rate - dv, rate + dv);
      const double step = command * dt;
      // 只有角度与角速度都能在一个加速度步长内接上规划 yaw 时才并回，否则允许小幅越过再折返，
      // 保证全程角加速度不超过 angular_accel。
      const bool reaches = std::abs(diff) < 1e-9 ||
        (step * diff >= 0.0 && std::abs(step) >= std::abs(diff));
      if (reaches && std::abs(command - point.yaw_rate) <= dv + 1e-12) {
        rate = point.yaw_rate;
        yaw = planned;
        following = true;
      } else {
        yaw += step;
        rate = command;
      }
    }
    yaw = wrapAngle(yaw);
    point.yaw = yaw;
    point.yaw_rate = rate;
  }
  return any_spin;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__SPIN_YAW_HPP_
