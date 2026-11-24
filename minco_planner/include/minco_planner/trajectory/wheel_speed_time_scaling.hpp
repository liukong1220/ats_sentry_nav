// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_
#define MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "minco_planner/trajectory/local_time_scaling.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

struct WheelSpeedTimeScalingParams
{
  // 单轮线速度上限（m/s）；<= 0 关闭。应低于底盘/MPC 的硬上限，给跟踪修正留余量。
  double wheel_speed_limit = 0.0;
  // 底盘中心到轮心的 x / y 半轴偏置（m），与 ats_swerve_mpc 的 wheel_base_x/y 同义。
  double wheel_offset_x = 0.0;
  double wheel_offset_y = 0.0;
  // 速度缩放系数每秒允许的最大变化量（1/s），避免局部减速在参考里产生速度阶跃。
  double scale_change_rate = 1.0;
  // 窄通道转向限速（m/s）；<= 0 关闭。净空不大于 narrow_turn_clearance 且 |yaw_rate| 超过
  // narrow_turn_yaw_rate_threshold 的点，平移速度不超过该值。墙尖拐角处车体一边平移一边
  // 转向，MPC 的横向与 yaw 跟踪误差随速度增大，足迹角点贴墙余量只有几厘米。
  double narrow_turn_speed_limit = 0.0;
  double narrow_turn_clearance = 0.0;
  double narrow_turn_yaw_rate_threshold = 0.2;
  // 按中心净空限速（m/s）；clearance_speed_min <= 0 关闭。中心净空 <= clearance_speed_low
  // 时平移速度不超过 clearance_speed_min，在 clearance_speed_low..clearance_speed_high
  // 之间光滑放开，>= clearance_speed_high 不限（或按 clearance_speed_max 上限）。
  double clearance_speed_min = 0.0;
  double clearance_speed_low = 0.0;
  double clearance_speed_high = 0.0;
  // > 0 时使用与 MINCO 上游一致的 C1 光滑曲线衔接至 clearance_speed_max；<= 0 保持原 min/(1-u)。
  double clearance_speed_max = 0.0;
};

// 四个舵轮中最大的轮速：轮心速度 = 车体平移速度 + wz x r。vx/vy 为世界系速度。
inline double maxWheelSpeed(
  const ReferencePoint & point, double wheel_offset_x, double wheel_offset_y)
{
  const double cos_yaw = std::cos(point.yaw);
  const double sin_yaw = std::sin(point.yaw);
  const double body_vx = cos_yaw * point.vx + sin_yaw * point.vy;
  const double body_vy = -sin_yaw * point.vx + cos_yaw * point.vy;
  double result = 0.0;
  for (const double sign_x : {1.0, -1.0}) {
    for (const double sign_y : {1.0, -1.0}) {
      const double rx = sign_x * wheel_offset_x;
      const double ry = sign_y * wheel_offset_y;
      result = std::max(
        result, std::hypot(body_vx - point.yaw_rate * ry, body_vy + point.yaw_rate * rx));
    }
  }
  return result;
}

// 平移与转向同时进行时，参考可能要求超过底盘能力的轮速（如拐角处 1.5 m/s 平移叠加
// 2.5 rad/s 转向）。MPC 饱和后按比例缩小指令，yaw 跟踪滞后再超调，足迹角点就会蹭墙。
// 这里只对超限的局部放慢时间：位置、yaw 序列都不变，速度、角速度按系数 f 缩放
// （轮速与 f 成正比），加速度按 f^2 缩放（忽略 df/dt 项，f 变化率已受限）。
// 返回是否修改了轨迹。
inline bool applyWheelSpeedTimeScaling(
  ReferenceTrajectory & trajectory, const WheelSpeedTimeScalingParams & params)
{
  const std::size_t count = trajectory.points.size();
  if (params.wheel_speed_limit <= 0.0 || count < 2) {
    return false;
  }

  std::vector<double> factors(count, 1.0);
  bool limited = false;
  for (std::size_t i = 0; i < count; ++i) {
    const double wheel_speed = maxWheelSpeed(
      trajectory.points[i], params.wheel_offset_x, params.wheel_offset_y);
    if (wheel_speed > params.wheel_speed_limit) {
      factors[i] = params.wheel_speed_limit / wheel_speed;
      limited = true;
    }
  }
  if (!limited) {
    return false;
  }

  return applyLocalTimeScaling(trajectory, factors, params.scale_change_rate);
}

// 窄通道内边平移边转向的局部放慢：位置与 yaw 序列不变，只改时间参数化。
// 依赖 ReferencePoint::clearance 已标注；未标注（NaN）的点不受影响。返回是否修改了轨迹。
inline bool applyNarrowTurnTimeScaling(
  ReferenceTrajectory & trajectory, const WheelSpeedTimeScalingParams & params)
{
  const std::size_t count = trajectory.points.size();
  if (params.narrow_turn_speed_limit <= 0.0 || count < 2) {
    return false;
  }
  std::vector<double> factors(count, 1.0);
  bool limited = false;
  for (std::size_t i = 0; i < count; ++i) {
    const auto & point = trajectory.points[i];
    const double speed = std::hypot(point.vx, point.vy);
    const double curvature = speed > 0.1 ?
      std::abs(point.vx * point.ay - point.vy * point.ax) / (speed * speed * speed) : 0.0;
    const bool tight_or_curving = point.clearance <= 0.45 || curvature >= 0.20;
    if (std::isfinite(point.clearance) && point.clearance <= params.narrow_turn_clearance &&
      tight_or_curving &&
      std::abs(point.yaw_rate) > params.narrow_turn_yaw_rate_threshold &&
      speed > params.narrow_turn_speed_limit)
    {
      factors[i] = params.narrow_turn_speed_limit / speed;
      limited = true;
    }
  }
  if (!limited) {
    return false;
  }
  return applyLocalTimeScaling(trajectory, factors, params.scale_change_rate);
}

// C1 光滑净空限速及其对净空的解析导数（供 MINCO 上游联合优化与后置限速同源使用）。
// p(s) = s^3 (4 - 3 s)，在 s=0 与 s=1 处一阶导均为 0，单调递增且全局 C1。
inline double clearanceSpeedLimitAndDerivative(
  double clearance, double speed_min, double clearance_low, double clearance_high,
  double speed_max, double * dlimit_dclearance = nullptr)
{
  if (dlimit_dclearance != nullptr) {
    *dlimit_dclearance = 0.0;
  }
  if (speed_min <= 0.0 || !std::isfinite(clearance) || clearance_high <= clearance_low) {
    return speed_max > 0.0 ? speed_max : std::numeric_limits<double>::infinity();
  }
  if (speed_max > 0.0) {
    const double eff_min = std::min(speed_min, speed_max);
    if (clearance <= clearance_low) {
      return eff_min;
    }
    if (clearance >= clearance_high) {
      return speed_max;
    }
    const double span = clearance_high - clearance_low;
    const double s = (clearance - clearance_low) / span;
    const double p = s * s * s * (4.0 - 3.0 * s);
    if (dlimit_dclearance != nullptr) {
      *dlimit_dclearance = (speed_max - eff_min) * (12.0 * s * s * (1.0 - s)) / span;
    }
    return eff_min + (speed_max - eff_min) * p;
  }
  if (clearance >= clearance_high) {
    return std::numeric_limits<double>::infinity();
  }
  if (clearance <= clearance_low) {
    return speed_min;
  }
  const double u = (clearance - clearance_low) / (clearance_high - clearance_low);
  return speed_min / std::max(1e-3, 1.0 - u);
}

// 净空 c 处允许的平移速度上限；关闭或净空未知（NaN）时返回 +inf。
inline double clearanceSpeedLimit(double clearance, const WheelSpeedTimeScalingParams & params)
{
  if (params.clearance_speed_min <= 0.0 || !std::isfinite(clearance) ||
    params.clearance_speed_high <= params.clearance_speed_low)
  {
    return std::numeric_limits<double>::infinity();
  }
  if (clearance >= params.clearance_speed_high) {
    return std::numeric_limits<double>::infinity();
  }
  return clearanceSpeedLimitAndDerivative(
    clearance, params.clearance_speed_min, params.clearance_speed_low,
    params.clearance_speed_high, params.clearance_speed_max);
}

// 按中心净空的局部放慢：位置与 yaw 序列不变，只改时间参数化。返回是否修改了轨迹。
inline bool applyClearanceSpeedTimeScaling(
  ReferenceTrajectory & trajectory, const WheelSpeedTimeScalingParams & params)
{
  const std::size_t count = trajectory.points.size();
  if (params.clearance_speed_min <= 0.0 || count < 2) {
    return false;
  }
  std::vector<double> factors(count, 1.0);
  bool limited = false;
  for (std::size_t i = 0; i < count; ++i) {
    const auto & point = trajectory.points[i];
    const double speed = std::hypot(point.vx, point.vy);
    const double limit = clearanceSpeedLimit(point.clearance, params);
    if (speed > limit) {
      factors[i] = limit / speed;
      limited = true;
    }
  }
  if (!limited) {
    return false;
  }
  return applyLocalTimeScaling(trajectory, factors, params.scale_change_rate);
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_
