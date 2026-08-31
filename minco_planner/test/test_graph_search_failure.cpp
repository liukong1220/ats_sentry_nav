// Copyright 2026

#include <gtest/gtest.h>

#include "minco_planner/planning/graph_search_failure.hpp"

using ats_navigation_interfaces::msg::PlannerStatus;

// domain 141 goal 4 的实测失败：`jps failed: no path expanded=1 clearance=0.270 m`。
// 只有起点被弹出过，说明车所在格的邻域在阶梯底也全不可通行——这是可恢复的"被围住"，
// 过去被并入 FAILURE_NO_PATH 而立即 abort 成 result_code 5。
TEST(GraphSearchFailure, FrontierExhaustedAfterOnlyTheStartIsStartEnclosed)
{
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("no path", 1),
    PlannerStatus::FAILURE_START_ENCLOSED);
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("no path", 0),
    PlannerStatus::FAILURE_START_ENCLOSED);
}

// P2 `unreachable` 用例依赖 result_code 5：真正不可达的目标要先把整个可达自由区展开完才
// 耗尽 frontier，expanded_nodes 远大于 1，必须仍然是 FAILURE_NO_PATH。
TEST(GraphSearchFailure, UnreachableGoalStaysNoPath)
{
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("no path", 2),
    PlannerStatus::FAILURE_NO_PATH);
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("no path", 4821),
    PlannerStatus::FAILURE_NO_PATH);
}

// 起点/目标被占据的既有分类不能被新分支抢走，它有自己的上下文相关瞬时语义。
TEST(GraphSearchFailure, OccupiedReasonsKeepTheirOwnCode)
{
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("start is occupied", 0),
    PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED);
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("goal occupied", 0),
    PlannerStatus::FAILURE_START_OR_GOAL_OCCUPIED);
}

// 早期退出的 expanded_nodes 同样是 0，但搜索根本没跑过，不能被当成"被围住"而无限重试。
TEST(GraphSearchFailure, EarlyExitsAreNotMisreadAsStartEnclosed)
{
  for (const char * reason :
    {"grid is empty", "invalid grid", "start is outside grid", "start outside grid",
      "goal is outside grid", "goal outside grid", "broken parent chain"})
  {
    EXPECT_EQ(
      minco_planner::classifyGraphSearchFailure(reason, 0),
      PlannerStatus::FAILURE_NO_PATH) << reason;
  }
}

// 展开上限是资源边界而不是几何包围，保持终局失败语义。
TEST(GraphSearchFailure, ExpansionLimitIsNotStartEnclosed)
{
  EXPECT_EQ(
    minco_planner::classifyGraphSearchFailure("expansion limit reached", 1),
    PlannerStatus::FAILURE_NO_PATH);
}
