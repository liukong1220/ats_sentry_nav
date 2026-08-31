// Copyright 2026

#include "minco_planner/trajectory/terminal_yaw_relocation.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace minco_planner
{

namespace
{

/// 与末点"同坐标"的判据。yaw 规划追加的终端转向采样是对末点的整体拷贝,
/// 坐标位是逐位相同的,所以这里不需要放宽到栅格量级。
constexpr double kCoincidentEps = 1e-6;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

void freezeMotion(ReferencePoint & point)
{
  point.v = 0.0;
  point.vx = 0.0;
  point.vy = 0.0;
  point.ax = 0.0;
  point.ay = 0.0;
}

}  // namespace

std::size_t terminalCoincidentTailStart(const ReferenceTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0U;
  }
  const std::size_t last = trajectory.points.size() - 1U;
  const double terminal_x = trajectory.points[last].x;
  const double terminal_y = trajectory.points[last].y;
  std::size_t start = last;
  while (start > 0U) {
    const ReferencePoint & previous = trajectory.points[start - 1U];
    if (std::abs(previous.x - terminal_x) > kCoincidentEps ||
      std::abs(previous.y - terminal_y) > kCoincidentEps)
    {
      break;
    }
    --start;
  }
  return start;
}

std::size_t terminalApproachWindowStart(
  const ReferenceTrajectory & trajectory,
  double window_length)
{
  if (trajectory.points.empty()) {
    return 0U;
  }
  const std::size_t last = trajectory.points.size() - 1U;
  if (!(window_length > 0.0)) {
    return last;
  }
  double accumulated = 0.0;
  std::size_t start = last;
  while (start > 0U) {
    const ReferencePoint & current = trajectory.points[start];
    const ReferencePoint & previous = trajectory.points[start - 1U];
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    if (accumulated > window_length) {
      break;
    }
    --start;
  }
  return start;
}

bool collisionsConfinedToTerminalApproach(
  const ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & safety,
  double window_length)
{
  if (safety.collisions.empty() || trajectory.points.size() < 2U) {
    return false;
  }
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  // 与 collisionsConfinedToTerminalRotation 完全相同的两个守卫:整条轨迹退化成
  // 一个点,或末点后面没有追加的原地转向段,都说明目标位姿本身不可行。
  if (tail == 0U || tail + 1U >= trajectory.points.size()) {
    return false;
  }
  const std::size_t window = std::min(tail, terminalApproachWindowStart(trajectory, window_length));
  for (const CollisionSample & collision : safety.collisions) {
    if (collision.trajectory_index < window) {
      return false;
    }
  }
  return true;
}

bool collisionsConfinedToTerminalRotation(
  const ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & safety)
{
  if (safety.collisions.empty() || trajectory.points.size() < 2U) {
    return false;
  }
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  // tail == 0 表示整条轨迹退化成一个点;tail + 1 == size 表示末点后面没有
  // 追加的原地转向段,此时目标位姿本身不可行,重定位无从下手。
  if (tail == 0U || tail + 1U >= trajectory.points.size()) {
    return false;
  }
  for (const CollisionSample & collision : safety.collisions) {
    if (collision.trajectory_index < tail) {
      return false;
    }
  }
  return true;
}

bool relocateTerminalYawRotation(
  const ReferenceTrajectory & input,
  double goal_yaw,
  std::size_t rotation_index,
  const TerminalYawRelocationParams & params,
  ReferenceTrajectory * output)
{
  if (output == nullptr || input.points.size() < 2U) {
    return false;
  }
  const std::size_t tail = terminalCoincidentTailStart(input);
  if (rotation_index > tail) {
    return false;
  }

  ReferenceTrajectory result;
  result.points.reserve(input.points.size());
  for (std::size_t index = 0U; index <= rotation_index; ++index) {
    result.points.push_back(input.points[index]);
  }

  const ReferencePoint pivot = result.points.back();
  const double delta = shortestAngularDistance(pivot.yaw, goal_yaw);
  double duration = 0.0;
  if (std::abs(delta) <= 1e-8) {
    result.points.back().yaw = normalizeAngle(goal_yaw);
    result.points.back().yaw_rate = 0.0;
  } else {
    const double yaw_rate_limit = std::max(1e-3, params.yaw_rate_limit);
    duration = 1.875 * std::abs(delta) / yaw_rate_limit;
    const double sample_period = std::max(0.01, params.sample_period);
    const int sample_count = std::max(
      2, static_cast<int>(std::ceil(duration / sample_period)));
    // 转向支点本身也要静止:整段原地转向期间不应该有任何平移指令,否则参考里
    // 会出现"这一拍 0.5 m/s、下一拍 0"的阶跃,把 MPC 推向不必要的超调。
    freezeMotion(result.points.back());
    for (int step = 1; step <= sample_count; ++step) {
      const double u = static_cast<double>(step) / static_cast<double>(sample_count);
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      const double u5 = u4 * u;
      const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;

      ReferencePoint point = pivot;
      point.t = pivot.t + duration * u;
      point.s = pivot.s;
      freezeMotion(point);
      point.yaw = normalizeAngle(pivot.yaw + delta * blend);
      point.yaw_rate = delta * blend_rate;
      result.points.push_back(point);
    }
  }

  // 转向之后全程保持 goal_yaw 平移,原有的终端转向尾段整段丢弃。
  for (std::size_t index = rotation_index + 1U; index <= tail; ++index) {
    ReferencePoint point = input.points[index];
    point.t += duration;
    point.yaw = normalizeAngle(goal_yaw);
    point.yaw_rate = 0.0;
    result.points.push_back(point);
  }

  // 末点必须静止:goal manager 的成功判据同时要求位置、yaw 和速度收敛。
  freezeMotion(result.points.back());
  result.points.back().yaw_rate = 0.0;

  if (!result.valid()) {
    return false;
  }
  *output = std::move(result);
  return true;
}

std::vector<std::size_t> terminalYawRelocationCandidates(
  const ReferenceTrajectory & trajectory,
  std::size_t max_candidates)
{
  std::vector<std::size_t> candidates;
  if (trajectory.points.empty() || max_candidates == 0U) {
    return candidates;
  }
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  if (max_candidates == 1U) {
    candidates.push_back(0U);
    return candidates;
  }
  if (max_candidates >= tail + 1U) {
    for (std::size_t index = 0U; index <= tail; ++index) {
      candidates.push_back(index);
    }
    return candidates;
  }
  for (std::size_t step = 0U; step < max_candidates; ++step) {
    const std::size_t index = (tail * step) / (max_candidates - 1U);
    if (candidates.empty() || candidates.back() != index) {
      candidates.push_back(index);
    }
  }
  return candidates;
}

}  // namespace minco_planner
