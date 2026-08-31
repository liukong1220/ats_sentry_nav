// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>

#include "minco_planner/safety/goal_pose_admission.hpp"

namespace minco_planner
{
namespace
{

// MuJoCo RMUC profile 的实际足迹参数。
FootprintSafetyParams makeFootprint()
{
  FootprintSafetyParams params;
  params.length = 0.60;
  params.width = 0.50;
  params.safety_margin = 0.02;
  params.obstacle_value_threshold = 100;
  params.unknown_is_obstacle = false;
  params.swept_max_corner_step_cells = 0.5;
  return params;
}

GoalPoseAdmissionParams makeParams()
{
  GoalPoseAdmissionParams params;
  params.position_tolerance_m = 0.08;
  params.yaw_tolerance_rad = 0.15;
  params.position_shrink = 0.75;
  params.yaw_shrink = 0.60;
  params.preferred_extra_margin_m = 0.03;
  params.position_step_m = 0.02;
  params.position_directions = 16;
  params.yaw_samples = 5;
  return params;
}

// 30x30 @ 0.1 m，原点 (0,0)，全空闲。
nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.1;
  grid.info.width = 30;
  grid.info.height = 30;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  return grid;
}

void setCell(nav_msgs::msg::OccupancyGrid & grid, int cx, int cy, int8_t value)
{
  grid.data[static_cast<std::size_t>(cy) * grid.info.width + static_cast<std::size_t>(cx)] = value;
}

// domain 175 目标 8 的实测几何缩放到本网格：墙面占据 x < 0.90 且 y < 0.70 的整个象限角,
// 目标点取 (1.20, 0.80)。yaw 0 的足迹半长 0.30+0.02=0.32 -> x_min=0.88 < 0.90 重叠;
// 东移即可行。这与实测的 8.40 墙面 / (8.70,-4.90) 目标 / 0.02 m 重叠等价。
nav_msgs::msg::OccupancyGrid makeGoalEightAnalogue()
{
  auto grid = makeGrid();
  for (int cx = 0; cx < 9; ++cx) {
    for (int cy = 0; cy < 7; ++cy) {
      setCell(grid, cx, cy, 100);
    }
  }
  return grid;
}

}  // namespace

TEST(GoalPoseAdmission, FeasibleCommandedGoalIsReturnedUnchanged)
{
  const auto grid = makeGrid();
  const auto result = admitGoalPose(1.50, 1.50, 0.0, grid, makeFootprint(), makeParams());
  EXPECT_TRUE(result.feasible);
  EXPECT_FALSE(result.relocated);
  EXPECT_DOUBLE_EQ(result.x, 1.50);
  EXPECT_DOUBLE_EQ(result.y, 1.50);
  EXPECT_DOUBLE_EQ(result.yaw, 0.0);
  EXPECT_DOUBLE_EQ(result.position_deviation_m, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_deviation_rad, 0.0);
}

TEST(GoalPoseAdmission, DisabledParamsNeverRelocate)
{
  auto params = makeParams();
  params.enabled = false;
  const auto grid = makeGoalEightAnalogue();
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), params);
  EXPECT_FALSE(result.feasible);
  EXPECT_FALSE(result.relocated);
  EXPECT_EQ(result.candidates_checked, 0U);
  EXPECT_DOUBLE_EQ(result.x, 1.20);
  EXPECT_DOUBLE_EQ(result.y, 0.80);
}

TEST(GoalPoseAdmission, CommandedGoalEightPoseIsInfeasibleWithoutAdmission)
{
  const auto grid = makeGoalEightAnalogue();
  EXPECT_FALSE(goalPoseFootprintFree(1.20, 0.80, 0.0, grid, makeFootprint()));
}

TEST(GoalPoseAdmission, GoalEightIsRescuedInsideTolerance)
{
  const auto grid = makeGoalEightAnalogue();
  const auto params = makeParams();
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), params);
  ASSERT_TRUE(result.feasible);
  EXPECT_TRUE(result.relocated);
  // 偏移必须严格小于 goal manager 的成功容差，否则到达该位姿也不算成功。
  EXPECT_LT(result.position_deviation_m, params.position_tolerance_m);
  EXPECT_LT(result.yaw_deviation_rad, params.yaw_tolerance_rad);
  EXPECT_LE(
    result.position_deviation_m,
    params.position_tolerance_m * params.position_shrink + 1e-9);
  // 准入结果本身必须过同一套足迹判定。
  EXPECT_TRUE(goalPoseFootprintFree(result.x, result.y, result.yaw, grid, makeFootprint()));
  const double deviation = std::hypot(result.x - 1.20, result.y - 0.80);
  EXPECT_NEAR(deviation, result.position_deviation_m, 1e-9);
}

TEST(GoalPoseAdmission, PreferredMarginTierIsUsedWhenItFits)
{
  const auto grid = makeGoalEightAnalogue();
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), makeParams());
  ASSERT_TRUE(result.feasible);
  EXPECT_TRUE(result.used_preferred_margin);
  auto inflated = makeFootprint();
  inflated.length += 2.0 * makeParams().preferred_extra_margin_m;
  inflated.width += 2.0 * makeParams().preferred_extra_margin_m;
  EXPECT_TRUE(goalPoseFootprintFree(result.x, result.y, result.yaw, grid, inflated));
}

TEST(GoalPoseAdmission, FallbackTierRunsWhenPreferredMarginHasNoSolution)
{
  const auto grid = makeGoalEightAnalogue();
  auto params = makeParams();
  // 额外余量调到容差撑不住的程度：第一档必然无解，只有裸足迹档能给出结果。
  params.preferred_extra_margin_m = 0.30;
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), params);
  ASSERT_TRUE(result.feasible);
  EXPECT_FALSE(result.used_preferred_margin);
  EXPECT_TRUE(goalPoseFootprintFree(result.x, result.y, result.yaw, grid, makeFootprint()));
}

TEST(GoalPoseAdmission, NoFeasiblePoseInsideToleranceReportsInfeasible)
{
  // 目标被完全埋进障碍：容差域内不存在可行位姿。
  auto grid = makeGrid();
  for (int cx = 0; cx < 30; ++cx) {
    for (int cy = 0; cy < 30; ++cy) {
      setCell(grid, cx, cy, 100);
    }
  }
  const auto result = admitGoalPose(1.50, 1.50, 0.0, grid, makeFootprint(), makeParams());
  EXPECT_FALSE(result.feasible);
  EXPECT_FALSE(result.relocated);
  EXPECT_GT(result.candidates_checked, 0U);
  // 不可行时保持原目标，交给既有失败路径。
  EXPECT_DOUBLE_EQ(result.x, 1.50);
  EXPECT_DOUBLE_EQ(result.y, 1.50);
}

TEST(GoalPoseAdmission, ZeroToleranceOnlyAcceptsTheCommandedPose)
{
  const auto grid = makeGoalEightAnalogue();
  auto params = makeParams();
  params.position_tolerance_m = 0.0;
  params.yaw_tolerance_rad = 0.0;
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), params);
  EXPECT_FALSE(result.feasible);
  // 容差为零时候选集就是原目标本身，两档各试一次：带额外余量的一档和裸足迹一档。
  EXPECT_EQ(result.candidates_checked, 2U);
}

TEST(GoalPoseAdmission, YawOffsetsArePreferredOnlyAfterPositionOffsets)
{
  // 只有偏航能救的场景：目标位置本身在 yaw 0 下不可行，但位置容差为零。
  auto grid = makeGrid();
  for (int cy = 0; cy < 30; ++cy) {
    setCell(grid, 8, cy, 100);
  }
  auto params = makeParams();
  params.position_tolerance_m = 0.0;
  params.preferred_extra_margin_m = 0.0;
  // x=1.20 时 yaw 0 的 x_min=0.88 落在 x∈[0.80,0.90) 的墙里；
  // yaw=±pi/2 才能把西向外延缩到 0.27，但 pi/2 远超偏航容差，所以这里应当无解。
  const auto result = admitGoalPose(1.20, 1.50, 0.0, grid, makeFootprint(), params);
  EXPECT_FALSE(result.feasible);
}

TEST(GoalPoseAdmission, AdmissionIsDeterministic)
{
  const auto grid = makeGoalEightAnalogue();
  const auto first = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), makeParams());
  const auto second = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), makeParams());
  ASSERT_TRUE(first.feasible);
  ASSERT_TRUE(second.feasible);
  EXPECT_DOUBLE_EQ(first.x, second.x);
  EXPECT_DOUBLE_EQ(first.y, second.y);
  EXPECT_DOUBLE_EQ(first.yaw, second.yaw);
  EXPECT_EQ(first.candidates_checked, second.candidates_checked);
}

TEST(GoalPoseAdmission, SmallestFeasibleDeviationIsChosen)
{
  const auto grid = makeGoalEightAnalogue();
  auto params = makeParams();
  params.preferred_extra_margin_m = 0.0;
  const auto result = admitGoalPose(1.20, 0.80, 0.0, grid, makeFootprint(), params);
  ASSERT_TRUE(result.feasible);
  // 任何比所选偏移更小的环上都不应存在可行位姿。
  const double smaller = result.position_deviation_m - params.position_step_m;
  if (smaller >= 0.0) {
    bool any_feasible = false;
    for (std::size_t d = 0; d < params.position_directions; ++d) {
      const double bearing =
        (2.0 * M_PI * static_cast<double>(d)) / static_cast<double>(params.position_directions);
      const double x = 1.20 + smaller * std::cos(bearing);
      const double y = 0.80 + smaller * std::sin(bearing);
      if (goalPoseFootprintFree(x, y, 0.0, grid, makeFootprint())) {
        any_feasible = true;
        break;
      }
    }
    EXPECT_FALSE(any_feasible);
  }
}

}  // namespace minco_planner
