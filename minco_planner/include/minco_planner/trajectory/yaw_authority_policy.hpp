// Copyright 2026

#ifndef MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_
#define MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_

#include <cmath>
#include <cstdint>

#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

inline std::uint8_t selectYawAuthority(
  const ReferenceTrajectory & trajectory,
  bool force_body_yaw_follow,
  double body_yaw_follow_clearance)
{
  using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
  if (force_body_yaw_follow) {
    return PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
  }
  for (const auto & point : trajectory.points) {
    if (std::isfinite(point.clearance) &&
      point.clearance <= body_yaw_follow_clearance)
    {
      return PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
    }
  }
  return PlannerStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED;
}

/// 小陀螺参数。开启后 minco_planner 对每条参考调用 applySpinYaw：足迹净空 > min_clearance 的
/// 开阔段自旋，窄段前 align_lookahead_m 弧长起回到规划 yaw（隧道对齐），参考中有自旋段且
/// 通过足迹门禁时授权为 YAW_AUTHORITY_SPIN。
struct SpinPolicy
{
  bool enabled = false;
  double min_clearance = 0.55;
  double align_lookahead_m = 3.0;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__YAW_AUTHORITY_POLICY_HPP_
