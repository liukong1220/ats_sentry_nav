// Copyright 2026

#ifndef MINCO_PLANNER__GRAPH_SEARCH_FAILURE_HPP_
#define MINCO_PLANNER__GRAPH_SEARCH_FAILURE_HPP_

#include <cstdint>
#include <string>

#include "ats_navigation_interfaces/msg/planner_status.hpp"

namespace minco_planner
{

// "起点被包围"和"目标不可达"是两种不同的失败，混成 FAILURE_NO_PATH 会让前者被立刻放弃。
//
// 图搜索每从 open set 弹出一个节点就 ++expanded_nodes，所以 `reason=="no path"` 且
// expanded_nodes<=1 表示：搜索确实跑了并耗尽了 frontier，但除起点外没有任何节点被展开——
// 车当前所在格的邻域在 clearance 阶梯底也全部不可通行。这是"车被围住"，而不是"目标不可达"：
// 地图由实时 LiDAR 构建，一次刷新或一次成功的脱离动作就可能解除，立即 abort 会把一个可恢复
// 的瞬时状态判成终局失败（domain 141 goal 4 即如此：`jps failed: no path expanded=1
// clearance=0.270` 之后直接 result_code 5）。
//
// 真正不可达的目标会先把整个可达自由区展开完才耗尽 frontier，expanded_nodes 远大于 1，
// 因此仍然归为 FAILURE_NO_PATH，P2 `unreachable` 用例依赖的 result_code 5 不受影响。
// 早期退出（空栅格、起点/目标越界）的 expanded_nodes 也是 0，但 reason 不是 "no path"，
// 所以不会被误并入这一类。
inline std::uint8_t classifyGraphSearchFailure(const std::string & reason, int expanded_nodes)
{
  using ats_navigation_interfaces::msg::PlannerStatus;
  if (reason.find("occupied") != std::string::npos) {
    return PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED;
  }
  if (reason.find("no path") != std::string::npos && expanded_nodes <= 1) {
    return PlannerStatus::FAILURE_START_ENCLOSED;
  }
  return PlannerStatus::FAILURE_NO_PATH;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GRAPH_SEARCH_FAILURE_HPP_
