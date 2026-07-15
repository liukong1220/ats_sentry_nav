// Copyright 2026

#include <cmath>
#include <limits>

#include "ats_rog_map_adapter/ground_projection_fusion.hpp"
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

  ats_rog_map_adapter::GroundProjectionFusionResult result;
  ats_rog_map_adapter::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map_adapter::GroundProjectionFusion::fuse(
    rog, terrain, slope, static_map, identityTransform(), params, result));
  EXPECT_EQ(result.planning_grid.data[1], -1);
  EXPECT_EQ(result.planning_grid.data[2], 100);
  EXPECT_EQ(result.planning_grid.data[3], 100);
  EXPECT_EQ(result.planning_grid.data[4], 100);
  EXPECT_EQ(result.unknown_cells, 1U);
  EXPECT_EQ(result.occupied_cells, 3U);
  EXPECT_EQ(result.known_free_cells, 12U);
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

  ats_rog_map_adapter::GroundProjectionFusionResult result;
  ats_rog_map_adapter::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map_adapter::GroundProjectionFusion::fuse(
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

  ats_rog_map_adapter::GroundProjectionFusionResult result;
  ats_rog_map_adapter::GroundProjectionFusionParams params;
  params.planning_resolution = 1.0;
  ASSERT_TRUE(ats_rog_map_adapter::GroundProjectionFusion::fuse(
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

  ats_rog_map_adapter::GroundProjectionFusionResult result;
  ats_rog_map_adapter::GroundProjectionFusionParams params;
  params.planning_resolution = 0.10;
  ASSERT_TRUE(
    ats_rog_map_adapter::GroundProjectionFusion::fuse(
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

TEST(GroundProjectionFusion, ClearsOnlyUnknownCellsUnderCurrentRobot)
{
  ats_rog_map_adapter::GroundProjectionFusionResult result;
  result.planning_grid = makeGrid(-1);
  result.planning_grid.data[5] = 100;
  result.unknown_cells = 15;
  result.occupied_cells = 1;

  EXPECT_EQ(
    ats_rog_map_adapter::GroundProjectionFusion::clearUnknownCircle(
      result, 1.5, 1.5, 0.2),
    0U);
  EXPECT_EQ(result.planning_grid.data[5], 100);
  EXPECT_EQ(
    ats_rog_map_adapter::GroundProjectionFusion::clearUnknownCircle(
      result, 2.5, 1.5, 0.2),
    1U);
  EXPECT_EQ(result.planning_grid.data[6], 0);
  EXPECT_EQ(result.unknown_cells, 14U);
  EXPECT_EQ(result.known_free_cells, 1U);
}

TEST(GroundProjectionFusion, ZeroUnknownClearRadiusLeavesGridAndCountersUnchanged)
{
  ats_rog_map_adapter::GroundProjectionFusionResult result;
  result.planning_grid = makeGrid(-1);
  result.planning_grid.data[5] = 100;
  result.unknown_cells = 15;
  result.occupied_cells = 1;
  const auto original_data = result.planning_grid.data;

  EXPECT_EQ(
    ats_rog_map_adapter::GroundProjectionFusion::clearUnknownCircle(
      result, 1.5, 1.5, 0.0),
    0U);
  EXPECT_EQ(result.planning_grid.data, original_data);
  EXPECT_EQ(result.unknown_cells, 15U);
  EXPECT_EQ(result.known_free_cells, 0U);
  EXPECT_EQ(result.occupied_cells, 1U);
}

TEST(RogMapEsdfSnapshot, PreservesGenerationSignedDistanceUnknownAndGradient)
{
  ats_rog_map_interfaces::srv::GetRogMapProjection::Response response;
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

  const auto snapshot = ats_rog_map_adapter::RogMapEsdfSnapshot::fromResponse(response);
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
  EXPECT_EQ(known.status, ats_rog_map_adapter::RogMapEsdfQueryStatus::kKnown);
  EXPECT_FLOAT_EQ(known.signed_distance, 1.0F);
  EXPECT_FLOAT_EQ(known.gradient_x, 0.5F);
  EXPECT_FLOAT_EQ(known.gradient_y, -0.5F);
  EXPECT_EQ(
    snapshot.query(1.5, 0.5).status,
    ats_rog_map_adapter::RogMapEsdfQueryStatus::kUnknown);
  EXPECT_EQ(
    snapshot.query(5.0, 0.5).status,
    ats_rog_map_adapter::RogMapEsdfQueryStatus::kOutside);
}

}  // namespace
