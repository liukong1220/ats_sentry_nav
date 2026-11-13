// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "minco_planner/planning/clearance_ladder.hpp"

namespace
{
// RMUC MuJoCo profile: footprint 0.60 x 0.50 m,单侧余量 0.02 m,规划栅格 0.10 m。
constexpr double kInscribed = 0.5 * 0.50 + 0.02;   // 0.27 m
constexpr double kResolution = 0.10;
constexpr double kPreferred = 0.42;                // 全 yaw 外接半径量级
}  // namespace

TEST(ClearanceLadder, AllowanceIsHalfACellDiagonal)
{
  EXPECT_NEAR(minco_planner::gridQuantizationAllowance(0.10), 0.10 * M_SQRT1_2, 1e-12);
  EXPECT_NEAR(minco_planner::gridQuantizationAllowance(0.05), 0.05 * M_SQRT1_2, 1e-12);
}

TEST(ClearanceLadder, DegenerateResolutionsContributeNoAllowance)
{
  EXPECT_EQ(minco_planner::gridQuantizationAllowance(0.0), 0.0);
  EXPECT_EQ(minco_planner::gridQuantizationAllowance(-0.10), 0.0);
  EXPECT_EQ(
    minco_planner::gridQuantizationAllowance(std::numeric_limits<double>::quiet_NaN()), 0.0);
}

TEST(ClearanceLadder, AutoFloorClearsTheFootprintGateInsteadOfGrindingAgainstIt)
{
  // 这是本轮的核心回归。旧实现的下限恰好等于内切半宽 0.270 m:格心距 0.270 m 的占据格
  // 边缘只有约 0.199 m,矩形 footprint 必然压进去,于是图搜索反复交付 footprint gate
  // 必然拒绝的路径,直到目标超时。下限必须严格大于内切半宽。
  const double floor =
    minco_planner::footprintConsistentClearanceFloor(kInscribed, kResolution, kPreferred);
  EXPECT_GT(floor, kInscribed);
  EXPECT_NEAR(floor, kInscribed + kResolution * M_SQRT1_2, 1e-12);

  // 按这个下限找到的路径,最近占据格的边缘仍在内切半宽之外。
  const double nearest_occupied_edge =
    floor - minco_planner::gridQuantizationAllowance(kResolution);
  EXPECT_GE(nearest_occupied_edge, kInscribed - 1e-12);
}

TEST(ClearanceLadder, FloorNeverExceedsThePreferredLevel)
{
  // 下限高过 preferred 会让梯子退化成一级,窄通道放宽能力整体消失。
  const double floor =
    minco_planner::footprintConsistentClearanceFloor(kInscribed, 0.50, kPreferred);
  EXPECT_NEAR(floor, kPreferred, 1e-12);
  EXPECT_LE(floor, kPreferred);
}

TEST(ClearanceLadder, FinerGridsRecoverMoreOfTheCorridor)
{
  // 0.05 m 栅格的量化损失只有 0.10 m 栅格的一半,窄通道因此更可能真的可通行。
  const double coarse =
    minco_planner::footprintConsistentClearanceFloor(kInscribed, 0.10, kPreferred);
  const double fine =
    minco_planner::footprintConsistentClearanceFloor(kInscribed, 0.05, kPreferred);
  EXPECT_LT(fine, coarse);
  EXPECT_GT(fine, kInscribed);
}

TEST(ClearanceLadder, NegativeInscribedRadiusIsClampedNotPropagated)
{
  const double floor = minco_planner::footprintConsistentClearanceFloor(-1.0, 0.10, kPreferred);
  EXPECT_NEAR(floor, 0.10 * M_SQRT1_2, 1e-12);
}

TEST(ClearanceLadder, DetourProbeTriggersOnlyForLongRoutes)
{
  // RMUL 隧道:直线 3.4 m,首选档绕行 12 m。
  EXPECT_TRUE(minco_planner::shouldProbeRelaxedClearance(12.0, 3.4, 1.5));
  EXPECT_FALSE(minco_planner::shouldProbeRelaxedClearance(4.0, 3.4, 1.5));
  // ratio <= 0 与旧行为一致:从不额外搜索。
  EXPECT_FALSE(minco_planner::shouldProbeRelaxedClearance(12.0, 3.4, 0.0));
  EXPECT_FALSE(minco_planner::shouldProbeRelaxedClearance(12.0, 0.0, 1.5));
  EXPECT_FALSE(minco_planner::shouldProbeRelaxedClearance(
      std::numeric_limits<double>::infinity(), 3.4, 1.5));
}

TEST(ClearanceLadder, RelaxedRouteReplacesOnlyAClearlyShorterOne)
{
  EXPECT_TRUE(minco_planner::shouldTakeRelaxedRoute(12.0, 3.6, 1.5));
  // 只短一点不值得换:保留净空更大的首选路径。
  EXPECT_FALSE(minco_planner::shouldTakeRelaxedRoute(4.0, 3.6, 1.5));
  EXPECT_FALSE(minco_planner::shouldTakeRelaxedRoute(12.0, 3.6, 0.0));
  EXPECT_FALSE(minco_planner::shouldTakeRelaxedRoute(12.0, 0.0, 1.5));
}
