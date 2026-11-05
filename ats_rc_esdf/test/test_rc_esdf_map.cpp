// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "ats_rc_esdf/esdf/static_map_fusion.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(unsigned int width, unsigned int height)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 1.0F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

geometry_msgs::msg::TransformStamped identityTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1.0;
  return transform;
}

TEST(RcTraversabilityEsdfProvider, ProducesExactSignedEuclideanDistances)
{
  auto grid = makeGrid(5, 5);
  grid.data[2U * grid.info.width + 2U] = 100;
  ats_rc_esdf::RcTraversabilityEsdfProvider provider;
  provider.updateGrid(grid, 50, true, 50);

  EXPECT_TRUE(provider.available());
  EXPECT_NEAR(provider.getDistance(2.5, 2.5), -1.0, 1e-9);
  EXPECT_NEAR(provider.getDistance(3.5, 2.5), 1.0, 1e-9);
  EXPECT_NEAR(provider.getDistance(3.5, 3.5), std::sqrt(2.0), 1e-9);
}

// 暴力解：自由格到最近障碍格心的欧氏距离，障碍格到最近自由格心的距离取负。
// 一维变换在站点被弹出后必须对新栈顶重算交点；用旧交点判断会让下包络出错，
// 距离偏大（偏向不安全一侧）。只有 5x5 单障碍的用例走不到弹出分支。
std::vector<double> bruteForceSignedDistance(
  const nav_msgs::msg::OccupancyGrid & grid, int threshold, bool unknown_is_obstacle)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const auto blocked = [&](int index) {
      const int value = grid.data[static_cast<std::size_t>(index)];
      return value >= threshold || (unknown_is_obstacle && value < 0);
    };
  std::vector<double> expected(static_cast<std::size_t>(width) * height, 0.0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool own = blocked(y * width + x);
      double best = std::numeric_limits<double>::infinity();
      for (int yy = 0; yy < height; ++yy) {
        for (int xx = 0; xx < width; ++xx) {
          if (blocked(yy * width + xx) != own) {
            best = std::min(best, std::hypot(double(x - xx), double(y - yy)));
          }
        }
      }
      expected[static_cast<std::size_t>(y) * width + x] =
        (own ? -best : best) * grid.info.resolution;
    }
  }
  return expected;
}

TEST(RcTraversabilityEsdfProvider, SingleObstacleBesideWallsMatchesBruteForce)
{
  auto grid = makeGrid(40, 40);
  for (unsigned int i = 0; i < 40U; ++i) {
    grid.data[i] = 100;
    grid.data[39U * 40U + i] = 100;
    grid.data[i * 40U] = 100;
    grid.data[i * 40U + 39U] = 100;
  }
  grid.data[20U * 40U + 30U] = 100;
  ats_rc_esdf::RcTraversabilityEsdfProvider provider;
  provider.updateGrid(grid, 50, true, 50);
  std::vector<double> field;
  ASSERT_TRUE(provider.copySignedDistanceField(field));
  // 行 y=20、x=20 到 x=30 的障碍是 10 格；旧实现在这里给出 19 格。
  EXPECT_NEAR(field[20U * 40U + 20U], 10.0, 1e-9);
  const auto expected = bruteForceSignedDistance(grid, 50, true);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    ASSERT_NEAR(field[i], expected[i], 1e-9) << "cell " << i;
  }
}

TEST(RcTraversabilityEsdfProvider, RandomGridsMatchBruteForce)
{
  constexpr unsigned int kSeed = 20261105U;
  std::mt19937 rng(kSeed);
  std::uniform_int_distribution<int> box(2, 6);
  for (int mode = 0; mode < 2; ++mode) {
    for (int trial = 0; trial < 12; ++trial) {
      const int width = 48;
      const int height = 36;
      auto grid = makeGrid(width, height);
      grid.info.resolution = 0.1F;
      if (mode == 1) {
        // 类 S3：中间一块已知自由区，其余 unknown。
        grid.data.assign(grid.data.size(), -1);
        for (int y = 9; y < 27; ++y) {
          for (int x = 12; x < 36; ++x) {grid.data[y * width + x] = 0;}
        }
      } else {
        for (int x = 0; x < width; ++x) {
          grid.data[x] = 100;
          grid.data[(height - 1) * width + x] = 100;
        }
      }
      for (int i = 0; i < 10; ++i) {
        const int bw = box(rng);
        const int bh = box(rng);
        const int x0 = static_cast<int>(rng() % (width - bw));
        const int y0 = static_cast<int>(rng() % (height - bh));
        for (int y = y0; y < y0 + bh; ++y) {
          for (int x = x0; x < x0 + bw; ++x) {grid.data[y * width + x] = 100;}
        }
      }
      ats_rc_esdf::RcTraversabilityEsdfProvider provider;
      provider.updateGrid(grid, 50, true, 50);
      std::vector<double> field;
      ASSERT_TRUE(provider.copySignedDistanceField(field)) << "mode " << mode << " trial " << trial;
      const auto expected = bruteForceSignedDistance(grid, 50, true);
      for (std::size_t i = 0; i < expected.size(); ++i) {
        ASSERT_NEAR(field[i], expected[i], 1e-6)
          << "seed " << kSeed << " mode " << mode << " trial " << trial << " cell " << i;
      }
    }
  }
}

// RMUC terrain 契约：`0..99` 是连续风险，只有 `100` 是硬障碍。调用方用
// obstacle_value_threshold 声明这条边界，RC-ESDF 的障碍集必须与之一致，否则同一张
// 栅格上离散 footprint gate 与 ESDF clearance 会给出互相矛盾的结论。
TEST(RcTraversabilityEsdfProvider, RiskBandBelowDeclaredThresholdIsNotAnEsdfObstacle)
{
  auto grid = makeGrid(5, 5);
  grid.data[1U * grid.info.width + 1U] = 60;   // 连续风险，不是硬障碍
  grid.data[3U * grid.info.width + 3U] = 100;  // 硬障碍

  ats_rc_esdf::RcTraversabilityEsdfProvider strict;
  strict.updateGrid(grid, 100, false, 100);
  ASSERT_TRUE(strict.available());
  // 风险格自身必须是正 clearance，且只由 (3,3) 的硬障碍决定距离。
  EXPECT_NEAR(strict.getDistance(1.5, 1.5), std::sqrt(8.0), 1e-9);
  EXPECT_NEAR(strict.getDistance(3.5, 3.5), -1.0, 1e-9);

  // threshold=50 的既有调用点行为不变：60 仍然是障碍。
  ats_rc_esdf::RcTraversabilityEsdfProvider permissive;
  permissive.updateGrid(grid, 50, false, 50);
  ASSERT_TRUE(permissive.available());
  EXPECT_NEAR(permissive.getDistance(1.5, 1.5), -1.0, 1e-9);
}

// unknown 的归属只由 unknown_is_obstacle 决定。语义分对 unknown 返回 -1.0，阈值降到
// 0 时不能让这个负分反过来把 unknown 判成障碍。
TEST(RcTraversabilityEsdfProvider, UnknownIsNotPromotedBySemanticTermAtZeroThreshold)
{
  auto grid = makeGrid(5, 5);
  grid.data.assign(grid.data.size(), -1);
  grid.data[2U * grid.info.width + 2U] = 0;

  ats_rc_esdf::RcTraversabilityEsdfProvider provider;
  provider.updateGrid(grid, 0, false, 0);
  ASSERT_TRUE(provider.available());
  EXPECT_NEAR(provider.getDistance(2.5, 2.5), -1.0, 1e-9);
  EXPECT_NEAR(provider.getDistance(3.5, 2.5), 1.0, 1e-9);
}

TEST(StaticMapFusion, StaticWallsAndUnknownCellsCannotBeErased)
{
  auto local = makeGrid(5, 3);
  local.header.frame_id = "odom";
  local.data[1U * local.info.width + 1U] = 80;
  local.data[1U * local.info.width + 4U] = -1;
  auto static_map = makeGrid(5, 3);
  static_map.data[1U * static_map.info.width + 2U] = 100;
  static_map.data[2U * static_map.info.width + 4U] = -1;

  nav_msgs::msg::OccupancyGrid planning;
  ats_rc_esdf::StaticMapFusionParams params;
  ASSERT_TRUE(ats_rc_esdf::StaticMapFusion::buildPlanningGrid(
    local, static_map, identityTransform(), params, planning));

  EXPECT_EQ(planning.data[1U * planning.info.width + 1U], 100);
  EXPECT_EQ(planning.data[1U * planning.info.width + 2U], 100);
  EXPECT_EQ(planning.data[1U * planning.info.width + 4U], 0);
  EXPECT_EQ(planning.data[2U * planning.info.width + 4U], 0);
}

TEST(StaticMapFusion, CombinesOccupiedFreeAndUnknownByEvidence)
{
  auto local = makeGrid(5, 1);
  auto static_map = makeGrid(5, 1);
  local.data = {-1, 0, 100, -1, -1};
  static_map.data = {0, -1, -1, 100, -1};

  nav_msgs::msg::OccupancyGrid planning;
  ASSERT_TRUE(ats_rc_esdf::StaticMapFusion::buildPlanningGrid(
    local, static_map, identityTransform(), ats_rc_esdf::StaticMapFusionParams {}, planning));

  // static free resolves local unknown; local free resolves static unknown;
  // either source occupied wins; only all-source unknown remains unknown.
  EXPECT_EQ(planning.data[0], 0);
  EXPECT_EQ(planning.data[1], 0);
  EXPECT_EQ(planning.data[2], 100);
  EXPECT_EQ(planning.data[3], 100);
  EXPECT_EQ(planning.data[4], -1);
}

TEST(StaticMapFusion, PreservesNarrowStaticWallBelowLocalGridResolution)
{
  auto local = makeGrid(1, 1);
  auto static_map = makeGrid(10, 10);
  static_map.info.resolution = 0.1F;
  static_map.data[5U * static_map.info.width + 1U] = 100;

  nav_msgs::msg::OccupancyGrid planning;
  ASSERT_TRUE(ats_rc_esdf::StaticMapFusion::buildPlanningGrid(
    local, static_map, identityTransform(), ats_rc_esdf::StaticMapFusionParams {}, planning));
  EXPECT_EQ(planning.data[0], 100);
}

TEST(StaticMapFusion, PreservesOverlappingWallWithNonIntegralResolutionOriginAndYaw)
{
  auto local = makeGrid(1, 1);
  local.header.frame_id = "odom";
  local.info.origin.orientation.z = std::sin(0.5 * 0.25);
  local.info.origin.orientation.w = std::cos(0.5 * 0.25);

  auto static_map = makeGrid(6, 6);
  static_map.info.resolution = 0.6F;
  static_map.info.origin.position.x = -1.0;
  static_map.info.origin.position.y = -1.0;
  static_map.info.origin.orientation.z = std::sin(0.5 * 0.30);
  static_map.info.origin.orientation.w = std::cos(0.5 * 0.30);
  // This source cell overlaps the rotated output footprint by area; a center
  // sample can miss it because 1.0 / 0.6 is non-integral.
  static_map.data[2U * static_map.info.width + 2U] = 100;

  nav_msgs::msg::OccupancyGrid planning;
  ASSERT_TRUE(ats_rc_esdf::StaticMapFusion::buildPlanningGrid(
    local, static_map, identityTransform(), ats_rc_esdf::StaticMapFusionParams {}, planning));
  EXPECT_EQ(planning.data[0], 100);
}

}  // namespace
