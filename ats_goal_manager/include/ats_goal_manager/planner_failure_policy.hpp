// Copyright 2026

#ifndef ATS_GOAL_MANAGER__PLANNER_FAILURE_POLICY_HPP_
#define ATS_GOAL_MANAGER__PLANNER_FAILURE_POLICY_HPP_

#include <cstdint>

#include "ats_navigation_interfaces/msg/planner_status.hpp"

namespace ats_goal_manager
{

/**
 * @brief 规划失败是否按瞬时故障挂起重规划（否则立即结束目标）。
 * @details 瞬时故障挂起后等下一次 snapshot 再派发，上界由 onTick 的 no_executable_plan 预算
 *          给出，不靠首击失败收敛。地图、TF、足迹、无路、起点被围这几类本来就是瞬时的；
 *          FAILURE_OPTIMIZER / FAILURE_REPAIR 也是：MuJoCo 2025 红框 domain 221 的一次
 *          MINCO 候选退化（joint_termination=max_iterations、peak_v 13.43 m/s）在下一张
 *          snapshot、新起点上可以重解，旧规则却在离目标 9.7 m 时直接 ABORTED（报告 5.1.10）。
 *          只有目标本身非法（FAILURE_INVALID_GOAL）仍立即结束。
 */
inline bool plannerFailureIsTransient(std::uint8_t failure_reason)
{
  using Status = ats_navigation_interfaces::msg::PlannerStatus;
  switch (failure_reason) {
    case Status::FAILURE_MAP_UNREADY:
    case Status::FAILURE_RUNTIME_UNSAFE:
    case Status::FAILURE_FOOTPRINT:
    case Status::FAILURE_SNAPSHOT_CHANGED:
    case Status::FAILURE_NO_PATH:
    case Status::FAILURE_START_ENCLOSED:
    case Status::FAILURE_START_OR_GOAL_OCCUPIED:
    case Status::FAILURE_START_TF:
    case Status::FAILURE_GOAL_TF:
    case Status::FAILURE_REFERENCE_TF:
    case Status::FAILURE_OPTIMIZER:
    case Status::FAILURE_REPAIR:
      return true;
    default:
      return false;
  }
}

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__PLANNER_FAILURE_POLICY_HPP_
