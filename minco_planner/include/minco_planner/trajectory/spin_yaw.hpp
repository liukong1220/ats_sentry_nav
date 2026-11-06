// Copyright 2026

#ifndef MINCO_PLANNER__SPIN_YAW_HPP_
#define MINCO_PLANNER__SPIN_YAW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
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

// 在已停止平移的终点先将自旋角速度连续降至零，再以五次 smoothstep 对齐请求 yaw。
// 整段仍由调用者重做轮速缩放与 swept footprint 门禁，不绕过窄区旋转安全检查。
inline bool appendSpinTerminalGoal(
  ReferenceTrajectory & trajectory, double goal_yaw, double original_end_time,
  const SpinYawParams & params)
{
  constexpr double sample_period = 0.05;
  const auto append_phase = [&](double duration, const auto & evaluate) {
      if (duration <= 1e-12) {
        return true;
      }
      const double count = std::max(2.0, std::ceil(duration / sample_period));
      if (!std::isfinite(count) || count > 10000.0) {
        return false;  // 非合理参数不允许产生无界尾段；调用者回退到原规划。
      }
      const ReferencePoint start = trajectory.points.back();
      for (std::size_t i = 1; i <= static_cast<std::size_t>(count); ++i) {
        const double u = static_cast<double>(i) / count;
        ReferencePoint point = start;
        point.t = start.t + duration * u;
        point.v = point.vx = point.vy = point.ax = point.ay = 0.0;
        evaluate(point, start, duration, u);
        trajectory.points.push_back(point);
      }
      return true;
    };
  const double braking_time = std::abs(trajectory.points.back().yaw_rate) / params.angular_accel;
  if (!append_phase(braking_time, [](auto & point, const auto & start, double duration, double u) {
      point.yaw = wrapAngle(start.yaw + start.yaw_rate * duration * (u - 0.5 * u * u));
      point.yaw_rate = start.yaw_rate * (1.0 - u);
    }))
  {
    return false;
  }
  const double delta = wrapAngle(goal_yaw - trajectory.points.back().yaw);
  const double duration = std::max(
    1.875 * std::abs(delta) / params.blend_rate_limit,
    std::sqrt((10.0 / std::sqrt(3.0)) * std::abs(delta) / params.angular_accel));
  if (!append_phase(duration, [delta](auto & point, const auto & start, double span, double u) {
      const double blend = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
      point.yaw = wrapAngle(start.yaw + delta * blend);
      point.yaw_rate = delta * 30.0 * u * u * (1.0 - u) * (1.0 - u) / span;
    }))
  {
    return false;
  }
  trajectory.points.back().yaw = wrapAngle(goal_yaw);
  trajectory.points.back().yaw_rate = 0.0;
  if (trajectory.points.back().t < original_end_time) {
    ReferencePoint hold = trajectory.points.back();
    hold.t = original_end_time;
    trajectory.points.push_back(hold);
  }
  return trajectory.valid();
}

/// 小陀螺 yaw 序列。足迹净空 > min_clearance 的点为"净空点"；某点可自旋当且仅当它是净空点、
/// 弧长前方 stop_before_narrow_m 内没有非净空点、后方 resume_after_narrow_m 内也没有。
/// 可自旋段内 yaw 以不超过 angular_accel 的角加速度加到 rate（逆时针）；其余点以不超过
/// blend_rate_limit 的角速度、angular_accel 的角加速度收敛回规划 yaw，收敛后严格跟随规划 yaw，
/// 所以窄段（隧道）内的 yaw 就是规划 yaw（含隧道轴线对齐），出窄段后再次自旋。
/// 平移部分的位置、时间不变；停止终点的原地尾段替换为减速与目标 yaw 收敛段，必要时延长。
/// 纯原地 yaw 参考不自旋。净空 NaN 按非净空处理。失败时原参考不变。
inline bool applySpinYaw(ReferenceTrajectory & trajectory, const SpinYawParams & params)
{
  if (!trajectory.valid() || !std::isfinite(params.rate) || !(params.rate > 0.0) ||
    !std::isfinite(params.angular_accel) || !std::isfinite(params.blend_rate_limit))
  {
    return false;
  }
  ReferenceTrajectory candidate = trajectory;
  auto & points = candidate.points;
  const std::size_t n = points.size();
  const auto stopped = [](const ReferencePoint & p) {
      return std::abs(p.v) <= 1e-6 && std::hypot(p.vx, p.vy) <= 1e-6;
    };
  const ReferencePoint terminal = points.back();
  const bool terminal_stop = stopped(terminal);
  std::size_t terminal_begin = n - 1;
  if (terminal_stop) {
    while (terminal_begin > 0 && stopped(points[terminal_begin - 1]) &&
      std::hypot(points[terminal_begin - 1].x - terminal.x,
      points[terminal_begin - 1].y - terminal.y) <= 1e-6 &&
      std::abs(points[terminal_begin - 1].s - terminal.s) <= 1e-6)
    {
      --terminal_begin;
    }
  }
  if (terminal_stop && (terminal_begin == 0 || !(params.angular_accel > 0.0) ||
    !(params.blend_rate_limit > 0.0)))
  {
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
  const std::size_t spin_end = terminal_stop ? terminal_begin : n - 1;
  for (std::size_t i = 1; i <= spin_end; ++i) {
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
  if (!any_spin) {
    return false;
  }
  if (terminal_stop) {
    points.resize(terminal_begin + 1);
    if (!appendSpinTerminalGoal(candidate, terminal.yaw, terminal.t, params)) {
      return false;
    }
  }
  trajectory = std::move(candidate);
  return true;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__SPIN_YAW_HPP_
