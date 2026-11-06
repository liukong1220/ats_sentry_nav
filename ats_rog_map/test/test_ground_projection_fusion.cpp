// Copyright 2026

#include <cmath>
#include <limits>

#include "ats_rog_map/planning/ground_projection_fusion.hpp"
#include "gtest/gtest.h"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(int8_t value)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.info.resolution = 1.0;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(16, value);
  return grid;
}

geometry_msgs::msg::TransformStamped identityTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1.0;
  return transform;
}

TEST(GroundProjectionFusion, ConservativelyFusesUnknownWallsTerrainAndSlope)
{
  auto rog = makeGrid(0);
  auto terrain = makeGrid(0);
  auto slope = makeGrid(0);
  auto static_map = makeGrid(0);
  rog.data[1] = -1;
  terrain.data[1] = -1;
  slope.data[1] = -1;
  static_map.data[1] = -1;
  static_map.data[2] = 100;
  slope.data[3] = 100;
  terrain.data[4] = 100;

  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[1], -1);
  EXPECT_EQ(result.planning_grid.data[2], 100);
  EXPECT_EQ(result.planning_grid.data[3], 100);
  EXPECT_EQ(result.planning_grid.data[4], 100);
  EXPECT_EQ(result.unknown_cells, 1U);
  EXPECT_EQ(result.occupied_cells, 3U);
  EXPECT_EQ(result.known_free_cells, 12U);
}

nav_msgs::msg::OccupancyGrid terrainFrame(int8_t value, int64_t stamp_ms)
{
  auto grid = makeGrid(0);
  grid.header.stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
  grid.header.stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
  grid.data[5] = value;
  return grid;
}

using ats_rog_map::planning::TerrainSpikeFilter;

TEST(TerrainSpikeFilter, SingleFrameHardCellIsPlannedAsFreeWhileFusionStaysFree)
{
  TerrainSpikeFilter filter(3, 50);
  EXPECT_EQ(filter.process(terrainFrame(0, 1000)).data[5], 0);
  const auto spike = filter.process(terrainFrame(100, 1100));
  EXPECT_LT(spike.data[5], 50);
  EXPECT_EQ(spike.data[5], 0);
  EXPECT_EQ(filter.suppressedCells(), 1U);
  EXPECT_EQ(filter.process(terrainFrame(0, 1200)).data[5], 0);

  auto rog = makeGrid(0);
  auto slope = makeGrid(0);
  auto static_map = makeGrid(0);
  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, spike, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[5], 0);
}

TEST(TerrainSpikeFilter, PersistentHardCellBecomesObstacleOnTheNthDistinctFrame)
{
  TerrainSpikeFilter filter(3, 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1000)).data[5], 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1100)).data[5], 50);
  EXPECT_EQ(filter.process(terrainFrame(100, 1200)).data[5], 100);
  EXPECT_EQ(filter.process(terrainFrame(100, 1300)).data[5], 100);
  // One soft frame drops the cell immediately.
  EXPECT_LT(filter.process(terrainFrame(0, 1400)).data[5], 50);
}

TEST(TerrainSpikeFilter, RepeatedStampCountsOnceAndGapsRestartTheRun)
{
  TerrainSpikeFilter filter(2, 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1000)).data[5], 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1000)).data[5], 50);
  EXPECT_EQ(filter.process(terrainFrame(100, 1100)).data[5], 100);
  EXPECT_LT(filter.process(terrainFrame(0, 1200)).data[5], 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1300)).data[5], 50);
  // An unknown frame in between also breaks the run.
  EXPECT_LT(filter.process(terrainFrame(-1, 1400)).data[5], 50);
  EXPECT_LT(filter.process(terrainFrame(100, 1500)).data[5], 50);
}

TEST(TerrainSpikeFilter, NeverSeenSoftValueFallsBackToNoEvidence)
{
  TerrainSpikeFilter filter(3, 50);
  EXPECT_EQ(filter.process(terrainFrame(100, 1000)).data[5], -1);
}

TEST(TerrainSpikeFilter, DisabledAtOneFramePassesHardCellsThrough)
{
  TerrainSpikeFilter filter(1, 50);
  EXPECT_EQ(filter.process(terrainFrame(100, 1000)).data[5], 100);
}

TEST(TerrainSpikeFilter, OriginShiftKeepsCountsAndNonIntegerShiftOrTimeRewindResets)
{
  TerrainSpikeFilter filter(2, 50);
  auto first = terrainFrame(100, 1000);
  first.data[5] = 100;
  EXPECT_LT(filter.process(first).data[5], 50);
  // The window moved one cell in +x: the same world cell is now index 4.
  auto moved = terrainFrame(0, 1100);
  moved.info.origin.position.x = 1.0;
  moved.data[4] = 100;
  EXPECT_EQ(filter.process(moved).data[4], 100);

  TerrainSpikeFilter fractional(2, 50);
  EXPECT_LT(fractional.process(terrainFrame(100, 1000)).data[5], 50);
  auto off = terrainFrame(100, 1100);
  off.info.origin.position.x = 0.4;
  EXPECT_LT(fractional.process(off).data[5], 50);

  TerrainSpikeFilter rewind(2, 50);
  EXPECT_LT(rewind.process(terrainFrame(100, 2000)).data[5], 50);
  EXPECT_LT(rewind.process(terrainFrame(100, 1000)).data[5], 50);
  EXPECT_EQ(rewind.resets(), 1U);
}

TEST(GroundProjectionFusion, ProjectionFrameKeepsRotatedFineStaticAndRealtimeObstacles)
{
  auto rog = makeGrid(-1);
  rog.info.resolution = 0.1;
  rog.info.origin.position.y = -0.2;
  rog.data[15] = 100;
  auto terrain = rog;
  auto slope = rog;
  terrain.data.assign(16, -1);
  slope.data.assign(16, -1);
  auto prior = makeGrid(-1);
  prior.header.frame_id = "map";
  prior.info.resolution = 0.04;
  prior.info.width = prior.info.height = 5;
  prior.info.origin.position.x = 4.9;
  prior.info.origin.position.y = -1.9;
  prior.data.assign(25, -1);
  prior.data[12] = 100;
  auto transform = identityTransform();
  transform.header.frame_id = "map";
  transform.transform.translation.x = 5.0;
  transform.transform.translation.y = -2.0;
  transform.transform.rotation.z = std::sin(M_PI / 4);
  transform.transform.rotation.w = std::cos(M_PI / 4);
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 0.1;
  params.output_in_projection_frame = true;
  ats_rog_map::planning::GroundProjectionFusionResult result;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, prior, transform, params, result));
  EXPECT_EQ(result.planning_grid.header.frame_id, "odom");
  ASSERT_EQ(result.planning_grid.info.width, 4U);
  EXPECT_EQ(result.planning_grid.data[15], 100);
  for (const auto index : {5, 6, 9, 10}) {EXPECT_EQ(result.planning_grid.data[index], 100);}
  EXPECT_EQ(result.planning_grid.data[0], -1);
}

TEST(GroundProjectionFusion, OrientedRobotRectangleClearsOnlyInsideUnknownAndKeepsOccupied)
{
  for (const double yaw : {0.0, M_PI / 4, -0.7}) {
    ats_rog_map::planning::GroundProjectionFusionResult result;
    result.planning_grid = makeGrid(-1);
    auto & grid = result.planning_grid;
    grid.info.resolution = 0.1;
    grid.info.width = grid.info.height = 40;
    grid.info.origin.position.x = grid.info.origin.position.y = -2.0;
    grid.data.assign(1600, -1);
    grid.data[20 * 40 + 20] = 100;
    result.occupied_cells = 1; result.unknown_cells = 1599;
    const auto cleared = ats_rog_map::planning::GroundProjectionFusion::clearUnknownFootprint(
      result, 0.0, 0.0, yaw, 0.70, 0.55, 0.10);
    EXPECT_GT(cleared, 0U);
    EXPECT_EQ(result.occupied_cells, 1U);
    EXPECT_EQ(result.unknown_cells + cleared, 1599U);
    for (unsigned y = 0; y < 40; ++y) {
      for (unsigned x = 0; x < 40; ++x) {
        const double wx = -2 + (x + 0.5) * grid.info.resolution;
        const double wy = -2 + (y + 0.5) * grid.info.resolution;
        const bool inside = std::abs(std::cos(yaw) * wx + std::sin(yaw) * wy) <= 0.45 + 1e-8 &&
          std::abs(-std::sin(yaw) * wx + std::cos(yaw) * wy) <= 0.375 + 1e-8;
        EXPECT_EQ(grid.data[y * 40 + x], x == 20 && y == 20 ? 100 : (inside ? 0 : -1));
      }
    }
  }
}

TEST(GroundProjectionFusion, UsesAnyKnownFreeEvidenceButKeepsAllUnknownCellsUnknown)
{
  auto rog = makeGrid(0);
  auto terrain = makeGrid(0);
  auto slope = makeGrid(0);
  auto static_map = makeGrid(0);
  rog.data[1] = -1;
  terrain.data[2] = -1;
  slope.data[3] = -1;
  static_map.data[4] = -1;
  static_map.data[5] = -1;
  rog.data[5] = 100;
  rog.data[6] = -1;
  terrain.data[6] = -1;
  slope.data[6] = -1;
  rog.data[7] = -1;
  terrain.data[7] = -1;
  slope.data[7] = -1;
  static_map.data[7] = -1;

  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[1], 0);
  EXPECT_EQ(result.planning_grid.data[2], 0);
  EXPECT_EQ(result.planning_grid.data[3], 0);
  EXPECT_EQ(result.planning_grid.data[4], 0);
  EXPECT_EQ(result.planning_grid.data[5], 100);
  EXPECT_EQ(result.planning_grid.data[6], 0);
  EXPECT_EQ(result.planning_grid.data[7], -1);
}

TEST(GroundProjectionFusion, FallsBackToStaticMapOutsideRogWindowDespiteTerrainUnknown)
{
  auto rog = makeGrid(0);
  auto terrain = makeGrid(-1);
  auto slope = makeGrid(-1);
  auto static_map = makeGrid(0);
  rog.info.width = 1;
  rog.info.height = 1;
  rog.data.assign(1, 0);

  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  params.use_static_map_obstacles = true;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[0], 0);
  EXPECT_EQ(result.planning_grid.data[15], 0);
}

TEST(GroundProjectionFusion, PreservesFinerStaticObstacleAcrossRotatedOutputFootprints)
{
  auto rog = makeGrid(-1);
  auto terrain = makeGrid(-1);
  auto slope = makeGrid(-1);
  auto static_map = makeGrid(0);
  static_map.info.resolution = 0.04;
  static_map.info.width = 5;
  static_map.info.height = 5;
  static_map.info.origin.position.x = 3.25;
  static_map.info.origin.position.y = -1.75;
  const double yaw = 0.37;
  static_map.info.origin.orientation.z = std::sin(0.5 * yaw);
  static_map.info.origin.orientation.w = std::cos(0.5 * yaw);
  static_map.data.assign(25, 0);
  static_map.data[2U * static_map.info.width + 2U] = 100;

  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 0.10;
  params.use_static_map_obstacles = true;
  ASSERT_TRUE(
    ats_rog_map::planning::GroundProjectionFusion::fuse(
      rog, terrain, slope, static_map, identityTransform(), params, result));

  ASSERT_EQ(result.planning_grid.info.width, 2U);
  ASSERT_EQ(result.planning_grid.info.height, 2U);
  EXPECT_DOUBLE_EQ(result.planning_grid.info.origin.position.x, 3.25);
  EXPECT_DOUBLE_EQ(result.planning_grid.info.origin.position.y, -1.75);
  EXPECT_DOUBLE_EQ(result.planning_grid.info.origin.orientation.z, std::sin(0.5 * yaw));
  EXPECT_DOUBLE_EQ(result.planning_grid.info.origin.orientation.w, std::cos(0.5 * yaw));
  EXPECT_EQ(result.planning_grid.data, std::vector<int8_t>({100, 100, 100, 100}));
  EXPECT_EQ(result.occupied_cells, 4U);
  EXPECT_EQ(result.known_free_cells, 0U);
  EXPECT_EQ(result.unknown_cells, 0U);
}

TEST(GroundProjectionFusion, TerrainHardCellDoesNotOverrideStaticFreeInsideCorridorRect)
{
  auto rog = makeGrid(0);
  auto terrain = makeGrid(0);
  auto slope = makeGrid(0);
  auto static_map = makeGrid(0);
  terrain.data[5] = 100;   // 走廊内，静态图 free：忽略
  slope.data[5] = 100;     // 同格坡度硬障碍：同样忽略
  terrain.data[10] = 100;  // 走廊外：保持占据
  terrain.data[6] = 100;   // 走廊内，但静态图占据：保持占据
  static_map.data[6] = 100;
  rog.data[7] = 100;       // 走廊内的 ROG 实时占据：保持占据

  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  params.terrain_free_override_rects = {1.0, 1.0, 4.0, 2.0};
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[5], 0);
  EXPECT_EQ(result.planning_grid.data[6], 100);
  EXPECT_EQ(result.planning_grid.data[7], 100);
  EXPECT_EQ(result.planning_grid.data[10], 100);
}

TEST(GroundProjectionFusion, EmptyCorridorRectsKeepLegacyTerrainBehaviour)
{
  auto rog = makeGrid(0);
  auto terrain = makeGrid(0);
  auto slope = makeGrid(0);
  auto static_map = makeGrid(0);
  terrain.data[5] = 100;
  ats_rog_map::planning::GroundProjectionFusionResult result;
  ats_rog_map::planning::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map::planning::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[5], 100);
}

TEST(GroundProjectionFusion, ClearsOnlyUnknownCellsUnderCurrentRobot)
{
  ats_rog_map::planning::GroundProjectionFusionResult result;
  result.planning_grid = makeGrid(-1);
  result.planning_grid.data[5] = 100;
  result.unknown_cells = 15;
  result.occupied_cells = 1;

  EXPECT_EQ(
    ats_rog_map::planning::GroundProjectionFusion::clearUnknownCircle(
      result, 1.5, 1.5, 0.2),
    0U);
  EXPECT_EQ(result.planning_grid.data[5], 100);
  EXPECT_EQ(
    ats_rog_map::planning::GroundProjectionFusion::clearUnknownCircle(
      result, 2.5, 1.5, 0.2),
    1U);
  EXPECT_EQ(result.planning_grid.data[6], 0);
  EXPECT_EQ(result.unknown_cells, 14U);
  EXPECT_EQ(result.known_free_cells, 1U);
}

TEST(GroundProjectionFusion, ZeroUnknownClearRadiusLeavesGridAndCountersUnchanged)
{
  ats_rog_map::planning::GroundProjectionFusionResult result;
  result.planning_grid = makeGrid(-1);
  result.planning_grid.data[5] = 100;
  result.unknown_cells = 15;
  result.occupied_cells = 1;
  const auto original_data = result.planning_grid.data;

  EXPECT_EQ(
    ats_rog_map::planning::GroundProjectionFusion::clearUnknownCircle(
      result, 1.5, 1.5, 0.0),
    0U);
  EXPECT_EQ(result.planning_grid.data, original_data);
  EXPECT_EQ(result.unknown_cells, 15U);
  EXPECT_EQ(result.known_free_cells, 0U);
  EXPECT_EQ(result.occupied_cells, 1U);
}

TEST(RogMapEsdfSnapshot, PreservesGenerationSignedDistanceUnknownAndGradient)
{
  ats_rog_map::GroundProjection response;
  response.generation = 42;
  response.ready = true;
  response.stale = false;
  response.occupancy_grid = makeGrid(0);
  response.occupancy_grid.data[1] = -1;
  response.signed_distance.assign(16, 1.0F);
  response.gradient_x.assign(16, 0.5F);
  response.gradient_y.assign(16, -0.5F);
  response.signed_distance[1] = std::numeric_limits<float>::quiet_NaN();
  response.gradient_x[1] = std::numeric_limits<float>::quiet_NaN();
  response.gradient_y[1] = std::numeric_limits<float>::quiet_NaN();

  const auto snapshot = ats_rog_map::planning::RogMapEsdfSnapshot::fromResponse(response);
  ASSERT_TRUE(snapshot.valid());
  EXPECT_TRUE(snapshot.available());
  EXPECT_EQ(snapshot.generation, 42U);
  EXPECT_EQ(snapshot.header.frame_id, "odom");
  EXPECT_FALSE(snapshot.unknown(0));
  EXPECT_TRUE(snapshot.unknown(1));
  EXPECT_FLOAT_EQ(snapshot.signed_distance[0], 1.0F);
  EXPECT_FLOAT_EQ(snapshot.gradient_x[0], 0.5F);
  EXPECT_FLOAT_EQ(snapshot.gradient_y[0], -0.5F);

  const auto known = snapshot.query(0.5, 0.5);
  EXPECT_EQ(known.status, ats_rog_map::planning::RogMapEsdfQueryStatus::kKnown);
  EXPECT_FLOAT_EQ(known.signed_distance, 1.0F);
  EXPECT_FLOAT_EQ(known.gradient_x, 0.5F);
  EXPECT_FLOAT_EQ(known.gradient_y, -0.5F);
  EXPECT_EQ(
    snapshot.query(1.5, 0.5).status,
    ats_rog_map::planning::RogMapEsdfQueryStatus::kUnknown);
  EXPECT_EQ(
    snapshot.query(5.0, 0.5).status,
    ats_rog_map::planning::RogMapEsdfQueryStatus::kOutside);
}

}  // namespace
