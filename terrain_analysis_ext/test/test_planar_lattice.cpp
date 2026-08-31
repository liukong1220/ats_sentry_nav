// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "terrain_analysis_ext/planar_lattice.hpp"

namespace
{

constexpr float kSize = 0.2f;
constexpr int kHalfWidth = 100;

// 体素边界必须落在世界系 size 的整数倍上，锚点落在哪个体素内都一样。
TEST(PlanarLattice, VoxelEdgesLandOnWorldMultiplesOfTheVoxelSize)
{
  for (int step = -40; step <= 40; ++step) {
    const float vehicle = 0.0137f * static_cast<float>(step) * 7.0f;
    const float anchor = terrain_analysis_ext::snapToVoxelCenter(vehicle, kSize);
    for (int index = kHalfWidth - 3; index <= kHalfWidth + 3; ++index) {
      const float edge = terrain_analysis_ext::planarVoxelLowerEdge(
        index, anchor, kSize, kHalfWidth);
      const float quotient = edge / kSize;
      EXPECT_NEAR(quotient, std::round(quotient), 1e-3f) << "edge=" << edge;
    }
  }
}

// 车体位于中心格内，checkTerrainConn 的洪泛种子和 terrainUnderVehicle 都依赖这一点。
TEST(PlanarLattice, VehicleAlwaysFallsInTheCentreCell)
{
  for (int step = -200; step <= 200; ++step) {
    const float vehicle = 0.0093f * static_cast<float>(step) * 11.0f;
    const float anchor = terrain_analysis_ext::snapToVoxelCenter(vehicle, kSize);
    EXPECT_EQ(
      terrain_analysis_ext::planarVoxelIndex1D(vehicle, anchor, kSize, kHalfWidth),
      kHalfWidth) << "vehicle=" << vehicle;
  }
}

// 相位不随车体滑动：固定世界点所落体素的边界与车体位置无关。
// 这是 domain 187/189/191 与 d151/d193 差别的根因，回归会直接让 red_box 目标 9
// 时而通过时而以 result_code=5 失败。
TEST(PlanarLattice, AFixedWorldPointKeepsItsVoxelEdgeWhileTheVehicleMoves)
{
  // RMUC 高地坡沿的物理墙面。
  const float wall_face = 9.825f;
  float reference_edge = 0.0f;
  bool have_reference = false;
  for (int step = 0; step <= 400; ++step) {
    const float vehicle = 6.0f + 0.01f * static_cast<float>(step);
    const float anchor = terrain_analysis_ext::snapToVoxelCenter(vehicle, kSize);
    const int index = terrain_analysis_ext::planarVoxelIndex1D(
      wall_face, anchor, kSize, kHalfWidth);
    const float edge = terrain_analysis_ext::planarVoxelLowerEdge(
      index, anchor, kSize, kHalfWidth);
    if (!have_reference) {
      reference_edge = edge;
      have_reference = true;
    }
    EXPECT_NEAR(edge, reference_edge, 1e-3f) << "vehicle=" << vehicle;
  }
  ASSERT_TRUE(have_reference);
  // 0.2 m 相位吸附后，9.825 m 的墙面落在 [9.8, 10.0) 内，过报只剩 0.025 m。
  EXPECT_NEAR(reference_edge, 9.8f, 1e-3f);
  EXPECT_LE(wall_face - reference_edge, static_cast<double>(kSize) + 1e-3);
  EXPECT_GE(wall_face - reference_edge, 0.0f);
}

// 车体锚定（未吸附）时同一面墙的面值会在一个完整体素宽度内摆动——这是被修掉的行为。
TEST(PlanarLattice, VehicleAnchoredPhaseWouldSlideAcrossAFullVoxel)
{
  const float wall_face = 9.825f;
  float min_edge = 1e9f;
  float max_edge = -1e9f;
  for (int step = 0; step <= 400; ++step) {
    const float vehicle = 6.0f + 0.01f * static_cast<float>(step);
    // 旧行为：锚点就是车体位置本身。
    const int index = terrain_analysis_ext::planarVoxelIndex1D(
      wall_face, vehicle, kSize, kHalfWidth);
    const float edge = terrain_analysis_ext::planarVoxelLowerEdge(
      index, vehicle, kSize, kHalfWidth);
    min_edge = std::min(min_edge, edge);
    max_edge = std::max(max_edge, edge);
  }
  EXPECT_GT(max_edge - min_edge, 0.9f * kSize);
}

// 负坐标同样向下取整，不在原点两侧出现半格错位。
TEST(PlanarLattice, NegativeCoordinatesFloorConsistently)
{
  const float anchor = terrain_analysis_ext::snapToVoxelCenter(-4.37f, kSize);
  EXPECT_EQ(
    terrain_analysis_ext::planarVoxelIndex1D(-4.4001f, anchor, kSize, kHalfWidth) + 1,
    terrain_analysis_ext::planarVoxelIndex1D(-4.3999f, anchor, kSize, kHalfWidth));
  const float edge = terrain_analysis_ext::planarVoxelLowerEdge(
    terrain_analysis_ext::planarVoxelIndex1D(-4.3999f, anchor, kSize, kHalfWidth),
    anchor, kSize, kHalfWidth);
  EXPECT_NEAR(edge, -4.4f, 1e-3f);
}

// 非法体素尺寸不得让下标计算出现除零或 NaN。
TEST(PlanarLattice, NonPositiveVoxelSizeFallsBackToAFiniteLattice)
{
  EXPECT_TRUE(std::isfinite(terrain_analysis_ext::snapToVoxelCenter(1.0f, 0.0f)));
  EXPECT_TRUE(std::isfinite(terrain_analysis_ext::snapToVoxelCenter(1.0f, -0.2f)));
  const int index = terrain_analysis_ext::planarVoxelIndex1D(1.0f, 0.0f, 0.0f, kHalfWidth);
  EXPECT_GT(index, kHalfWidth);
}

}  // namespace
