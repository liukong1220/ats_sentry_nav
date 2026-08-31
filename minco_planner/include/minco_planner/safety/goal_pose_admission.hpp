// Copyright 2026

#ifndef MINCO_PLANNER__GOAL_POSE_ADMISSION_HPP_
#define MINCO_PLANNER__GOAL_POSE_ADMISSION_HPP_

#include <cstddef>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

// action 下发的目标位姿是一个"点"，但成功判据是一个容差域：位置 goal_position_tolerance、
// 偏航 goal_yaw_tolerance。规划器此前只把那个点当唯一终点，于是出现这样一类失败：
// 目标点的矩形足迹与栅格墙面重叠几毫米，而容差域内存在大量完全可行的位姿。gate 正确
// 拒绝每一条以该点结尾的轨迹，goal manager 把 FOOTPRINT 当瞬时故障反复重试，直到
// no-executable-plan 预算耗尽——车其实已经停在目标 0.16 m 处，只差一个可行的终点标注。
//
// domain 175 目标 8 `east_mid` 实测：目标 (8.70, -4.90)、goal_yaw=0，聚合后的墙面东边界
// 在 x=8.40，yaw 0 的足迹半长 0.30 + 安全余量 0.02 = 0.32，于是 x_min=8.38 < 8.40，
// 重叠 0.02 m（正好是安全余量本身）。把终点东移 0.06 m（仍在 0.08 m 位置容差内）即可行。
//
// 因此这里做的是"在成功容差域内挑一个可行终点"，而不是放宽任何门禁：
//   1. 候选位姿必须过与最终提交门完全相同的 FootprintSafetyChecker；
//   2. 候选偏移严格小于容差（再乘一个收缩系数），所以到达该位姿必然满足原目标的成功判据，
//      SUCCEEDED 不是靠放宽判据换来的；
//   3. 搜索按"位置偏移优先最小、其次偏航偏移最小"的字典序，原目标可行时一定原样返回；
//   4. 两档：先要求额外膨胀余量（给 MPC 跟踪误差留空间），整轮无解才退到裸足迹。
// 容差本身属于 goal manager 的成功判据，必须与其配置一致，否则会瞄到成功域之外。
struct GoalPoseAdmissionParams
{
  bool enabled = true;
  /// 与 goal manager 的 `goal_position_tolerance` 一致。
  double position_tolerance_m = 0.08;
  /// 与 goal manager 的 `goal_yaw_tolerance` 一致。
  double yaw_tolerance_rad = 0.15;
  /// 只使用容差的一部分，剩下的留给 MPC 终端跟踪误差。
  double position_shrink = 0.75;
  double yaw_shrink = 0.60;
  /// 第一档要求的额外膨胀余量：终点被跟踪误差推回几厘米也不会立刻压到墙上。
  double preferred_extra_margin_m = 0.03;
  double position_step_m = 0.02;
  std::size_t position_directions = 16;
  /// 偏航候选个数（含 0）。1 表示只试原始 goal_yaw。
  std::size_t yaw_samples = 5;
};

struct GoalPoseAdmissionResult
{
  /// 找到了可行终点（可能就是原目标本身）。
  bool feasible = false;
  /// 可行终点与原目标不同。
  bool relocated = false;
  /// 该解来自带额外膨胀余量的第一档。
  bool used_preferred_margin = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double position_deviation_m = 0.0;
  double yaw_deviation_rad = 0.0;
  std::size_t candidates_checked = 0;
};

/// 单个位姿的矩形足迹占据判定，语义与最终提交门完全一致。
bool goalPoseFootprintFree(
  double x, double y, double yaw,
  const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyParams & footprint);

GoalPoseAdmissionResult admitGoalPose(
  double goal_x, double goal_y, double goal_yaw,
  const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyParams & footprint,
  const GoalPoseAdmissionParams & params);

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GOAL_POSE_ADMISSION_HPP_
