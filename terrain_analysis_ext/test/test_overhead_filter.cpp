// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <limits>

#include "terrain_analysis_ext/overhead_filter.hpp"

namespace
{

constexpr double kObstacleThre = 0.18;    // traversabilityObstacleHeightThre
constexpr double kVehicleHeight = 0.242;  // 含雷达整车高（用户 2026-10-17）
// MuJoCo 2027 profile：车高 + 1.3 cm 余量。2027 悬空板最小净空 0.260 m，余量只有 1.8 cm，
// 所以余量必须小于 1.8 cm。
constexpr double kOverhead = 0.255;

struct Voxel
{
  float elevation{0.0f};
  float max_rel{0.0f};
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};
  int count{0};

  bool add(float rel_to_ground, double overhead_height)
  {
    return terrain_analysis_ext::accumulateVoxelPoint(
      elevation + rel_to_ground, elevation, overhead_height, max_rel, min_z, max_z, count);
  }
  float metric() const
  {
    return terrain_analysis_ext::voxelHeightMetric(min_z, max_z, max_rel);
  }
  bool hardObstacle() const {return metric() >= kObstacleThre;}
};

void addGround(Voxel & v, double overhead_height)
{
  for (const float z : {0.0f, 0.004f, 0.008f, 0.012f}) {
    v.add(z, overhead_height);
  }
}

// 只有路面点：不是障碍，开不开头顶过滤都一样。
TEST(OverheadFilter, FlatGroundIsNotAnObstacleWithOrWithoutTheFilter)
{
  for (const double h : {0.0, kOverhead}) {
    Voxel v;
    addGround(v, h);
    EXPECT_FALSE(v.hardObstacle()) << "overhead_height=" << h;
    EXPECT_EQ(v.count, 4);
  }
}

// 2027 ramp_bridge：板下表面离地 0.26–0.35 m。关闭时整片判硬障碍（旧行为，负对照），
// 打开后下表面点不进统计，路面不再被标成障碍。
TEST(OverheadFilter, PlateUndersideAboveVehiclePlusMarginIsIgnored)
{
  for (const float clearance : {0.260f, 0.268f, 0.30f, 0.337f, 0.35f, 0.47f}) {
    Voxel off;
    addGround(off, 0.0);
    for (int i = 0; i < 6; ++i) {
      off.add(clearance + 0.001f * static_cast<float>(i), 0.0);
    }
    EXPECT_TRUE(off.hardObstacle()) << "filter off, clearance=" << clearance;

    Voxel on;
    addGround(on, kOverhead);
    int accepted = 0;
    for (int i = 0; i < 6; ++i) {
      accepted += on.add(clearance + 0.001f * static_cast<float>(i), kOverhead) ? 1 : 0;
    }
    EXPECT_EQ(accepted, 0) << "clearance=" << clearance;
    EXPECT_FALSE(on.hardObstacle()) << "filter on, clearance=" << clearance;
    EXPECT_EQ(on.count, 4);
  }
}

// 头顶面的点完全不进统计：一个体素只剩头顶面点时点数为 0（traversability 发布 -1，无证据）。
TEST(OverheadFilter, VoxelWithOnlyOverheadPointsHasNoEvidence)
{
  Voxel v;
  for (const float z : {0.27f, 0.30f, 0.33f}) {
    EXPECT_FALSE(v.add(z, kOverhead));
  }
  EXPECT_EQ(v.count, 0);
  EXPECT_FLOAT_EQ(v.metric(), 0.0f);
  EXPECT_FALSE(v.hardObstacle());
}

// 净空低于 车高 + 余量 的结构仍是障碍：车高 0.242 m，余量内的横梁（tunnel_B 顶板离口袋底
// 0.250 m）和比车矮的悬挂物不能被滤掉。
TEST(OverheadFilter, SurfaceNotAboveVehiclePlusMarginStaysAnObstacle)
{
  for (const float clearance : {0.19f, 0.22f, 0.242f, 0.250f, 0.255f}) {
    Voxel v;
    addGround(v, kOverhead);
    EXPECT_TRUE(v.add(clearance, kOverhead)) << "clearance=" << clearance;
    EXPECT_TRUE(v.hardObstacle()) << "clearance=" << clearance;
  }
}

// 边界：恰好等于 overhead_height 的点不是头顶面（严格大于才是）。
TEST(OverheadFilter, PointExactlyAtTheThresholdIsNotOverhead)
{
  EXPECT_FALSE(terrain_analysis_ext::isOverheadPoint(1.255f, 1.0f, 0.255));
  EXPECT_TRUE(terrain_analysis_ext::isOverheadPoint(1.2561f, 1.0f, 0.255));
  EXPECT_FALSE(terrain_analysis_ext::isOverheadPoint(1.0f, 1.0f, 0.255));
  EXPECT_FALSE(terrain_analysis_ext::isOverheadPoint(0.9f, 1.0f, 0.255));
}

// 默认 0 = 关闭，旧行为不变，哪怕点高得离谱。
TEST(OverheadFilter, ZeroOrNegativeThresholdDisablesTheFilter)
{
  for (const double h : {0.0, -1.0}) {
    EXPECT_FALSE(terrain_analysis_ext::isOverheadPoint(5.0f, 0.0f, h)) << "h=" << h;
  }
}

// 真正的墙：从地面一直往上都有点。0.18–0.255 m 之间的点留下，仍判硬障碍；
// 更高处的点被滤掉也不影响结论。
TEST(OverheadFilter, WallThatReachesTheGroundStaysAnObstacle)
{
  Voxel v;
  addGround(v, kOverhead);
  for (int step = 1; step <= 30; ++step) {
    v.add(0.02f * static_cast<float>(step), kOverhead);
  }
  EXPECT_TRUE(v.hardObstacle());
  EXPECT_LE(v.max_rel, static_cast<float>(kOverhead) + 1e-4f);
  EXPECT_GE(v.max_rel, 0.239f);
}

// 车高加 1.8 cm 的余量正好是板的最小净空 0.260 m：余量取 1.8 cm 会把净空 0.242+0.018 的
// 结构判为头顶面，这是用户要求余量必须小于 1.8 cm 的原因。
TEST(OverheadFilter, TheShippedMarginIsBelowTheMeasuredPlateClearanceMargin)
{
  const double plate_min_clearance = 0.260;
  const double margin = kOverhead - kVehicleHeight;
  EXPECT_GT(margin, 0.0);
  EXPECT_LT(margin, plate_min_clearance - kVehicleHeight);
  EXPECT_LT(margin, 0.018);
}

}  // namespace
