// Copyright 2026

#ifndef MINCO_PLANNER__ESCAPE_PREFIX_HPP_
#define MINCO_PLANNER__ESCAPE_PREFIX_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

// 离散 footprint gate 判定的是"轨迹上的某个位姿被占据"，而轨迹的第 0 点就是车当前所在
// 位姿。当车已经贴在障碍上（或压在栅格化噪声的占据/空闲边界上）时，从当前位姿出发的
// 每一条候选轨迹都会在 index 0/1 冲突，于是 gate 无界拒绝、goal manager 把
// FAILURE_FOOTPRINT 当瞬时故障无限重试，直到目标超时。要离开该位姿只能靠执行器，而被
// 拒绝掉的正是那条唯一能驱动执行器的轨迹：无界拒绝无法把车挪走，唯一结果是 livelock。
// 这与 plan_progress_watchdog 的 ego-blocked 逃逸、图搜索起点 clearance 阶梯是同一类
// fail-closed vs livelock 边界，只是下沉到了提交门。
//
// 因此只在"确实是从接触中逃逸"时放行一段有界前缀，判据完全来自离散冲突集本身：
//   1. 冲突必须紧贴轨迹起点（车此刻就在冲突区内或半个 footprint 余量之内），
//      否则这是一条"先走一段再撞墙"的轨迹，必须照旧拒绝；
//   2. 前缀之后必须存在冲突自由的剩余段，且按构造 prefix_end 之后不再有任何冲突——
//      即轨迹确实驶出障碍集且不再驶回，而不是沿着障碍蹭行；
//   3. 前缀在 SE(2) 上的运动量有硬上界，避免把"穿过障碍"伪装成逃逸：平移由弧长封顶，
//      旋转由 yaw 扫掠角封顶。两者必须同时存在——弧长只约束平移，原地旋转的弧长近似为 0，
//      单靠弧长无法拒绝"在障碍里自转半圈"。点数上界退化为资源保护，见 max_prefix_points。
// 这里刻意不使用 ESDF clearance 作为判据：clearance 与离散 gate 的分歧本身尚未定因，
// 用一个未解释的量去守一道安全门只会把问题挪走。
struct EscapePrefixParams
{
  // This bypass is experimental. A bounded collision prefix alone does not
  // prove that the path exits a quantization artifact rather than crossing a
  // thin physical obstacle, so production profiles stay fail-closed unless a
  // stronger, snapshot-bound escape authorization is added.
  bool enabled = false;
  // 冲突允许出现的最靠前位置（从轨迹起点起算的弧长）。index 1 在 RMUC 参考轨迹上约
  // 0.04 m，仍在 footprint 的 0.02 m 安全余量与半格栅格化噪声量级内。
  double max_head_offset_m = 0.10;
  // 逃逸前缀的平移上界，取一个外接圆半径量级（footprint 0.60x0.50 的外接半径 0.419 m）。
  double max_prefix_length_m = 0.40;
  // 逃逸前缀的旋转上界（前缀内逐点 |dyaw| 之和，往复摆动同样累加）。栅格化过报造成的
  // 接触靠转向即可脱离：矩形 footprint 的前向外扩 e(yaw)=0.32|cos|+0.27|sin| 在 0.27..0.419
  // 之间，最坏相位转到 e 的极小值只需约 pi/2 之内的一次转向。取 pi/2 既覆盖这一类脱离，
  // 又拒绝"压在障碍里自转半圈以上"的轨迹。
  double max_prefix_yaw_sweep_rad = M_PI_2;
  // 点数上界。平移与旋转都已被上面两条直接封顶后，前缀在 SE(2) 上的运动量与采样点数无关，
  // 点数只影响检查粒度（点越多检查越密而不是越松），因此这里只作为异常输入的资源保护，
  // 不再承担"防止弧长上界被大量点填满"的职责——那一职责已由 yaw 扫掠上界补齐。
  // domain 195 实测：原地转向脱离栅格过报接触时，18 个点只对应 0.029 m 弧长，
  // 12 的旧上界会把这类唯一可行的脱离轨迹判负，进而 livelock 到目标超时。
  std::size_t max_prefix_points = 64;
};

struct EscapePrefixDecision
{
  bool allowed = false;
  // 第一个冲突自由点的下标；allowed 时该点及其后的所有点都不含冲突。
  // 被拒绝时一律归零，调用方即使漏判 allowed 也拿不到一段可执行前缀（fail-closed）。
  std::size_t prefix_end = 0;
  // 归零之前测得的候选前缀末点，仅用于日志。prefix_length_m / prefix_yaw_sweep_rad 都是
  // 在这个下标上测的，只打 prefix_end 会得到"末点 0 却有 0.8 m 前缀"这种自相矛盾的行。
  std::size_t candidate_prefix_end = 0;
  double head_offset_m = 0.0;
  double prefix_length_m = 0.0;
  double prefix_yaw_sweep_rad = 0.0;
  std::size_t collision_count = 0;
};

// 用几何折线长度而不是 ReferencePoint::s，这样对未填充 s 的调用方同样成立。
inline double arcLengthTo(
  const std::vector<ReferencePoint> & points, std::size_t index)
{
  double length = 0.0;
  const std::size_t last = std::min(index, points.size() - 1U);
  for (std::size_t i = 1U; i <= last; ++i) {
    const double dx = points[i].x - points[i - 1U].x;
    const double dy = points[i].y - points[i - 1U].y;
    if (!std::isfinite(dx) || !std::isfinite(dy)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    length += std::hypot(dx, dy);
  }
  return length;
}

// 逐点累加 |dyaw|（归一化到 (-pi, pi] 后取绝对值），因此往复摆动不会互相抵消。
inline double yawSweepTo(
  const std::vector<ReferencePoint> & points, std::size_t index)
{
  double sweep = 0.0;
  const std::size_t last = std::min(index, points.size() - 1U);
  for (std::size_t i = 1U; i <= last; ++i) {
    const double previous = points[i - 1U].yaw;
    const double current = points[i].yaw;
    if (!std::isfinite(previous) || !std::isfinite(current)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    sweep += std::fabs(std::remainder(current - previous, 2.0 * M_PI));
  }
  return sweep;
}

inline EscapePrefixDecision evaluateEscapePrefix(
  const std::vector<ReferencePoint> & points,
  const std::vector<CollisionSample> & collisions,
  const EscapePrefixParams & params)
{
  EscapePrefixDecision decision;
  decision.collision_count = collisions.size();
  if (!params.enabled || collisions.empty() || points.size() < 2U) {
    return decision;
  }

  std::size_t first_index = collisions.front().trajectory_index;
  std::size_t last_index = collisions.front().trajectory_index;
  for (const CollisionSample & collision : collisions) {
    // 越界下标说明冲突集与轨迹不同源，属于调用方错误，宁可拒绝。
    if (collision.trajectory_index >= points.size()) {
      return decision;
    }
    first_index = std::min(first_index, collision.trajectory_index);
    last_index = std::max(last_index, collision.trajectory_index);
  }

  // swept 冲突记录在线段起点下标上，所以"越过 last_index"要多推一格才能保证 prefix_end
  // 之后既没有离散冲突也没有扫掠冲突。
  decision.prefix_end = last_index + 2U;
  decision.candidate_prefix_end = decision.prefix_end;
  if (decision.prefix_end >= points.size()) {
    // 没有冲突自由的剩余段：这条轨迹不是逃逸，而是整条都在障碍里。
    decision.prefix_end = 0U;
    return decision;
  }

  decision.head_offset_m = arcLengthTo(points, first_index);
  decision.prefix_length_m = arcLengthTo(points, decision.prefix_end);
  decision.prefix_yaw_sweep_rad = yawSweepTo(points, decision.prefix_end);
  if (!std::isfinite(decision.head_offset_m) || !std::isfinite(decision.prefix_length_m) ||
    !std::isfinite(decision.prefix_yaw_sweep_rad))
  {
    decision.prefix_end = 0U;
    return decision;
  }

  decision.allowed =
    decision.head_offset_m <= params.max_head_offset_m &&
    decision.prefix_end <= params.max_prefix_points &&
    decision.prefix_length_m <= params.max_prefix_length_m &&
    decision.prefix_yaw_sweep_rad <= params.max_prefix_yaw_sweep_rad;
  if (!decision.allowed) {
    decision.prefix_end = 0U;
  }
  return decision;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__ESCAPE_PREFIX_HPP_
