// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_
#define MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_

#include <algorithm>
#include <cmath>
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
    if (std::isfinite(point.clearance) && point.clearance <= params.narrow_turn_clearance &&
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

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__WHEEL_SPEED_TIME_SCALING_HPP_
