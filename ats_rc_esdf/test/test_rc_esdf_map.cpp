// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>

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
