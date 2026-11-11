// Copyright 2026

#include "ats_rog_map/odom_layer_integration.hpp"
#include "gtest/gtest.h"

namespace
{
using Point = std::array<double, 3>;

ats_rog_map::OdomLayerConfig config()
{
  ats_rog_map::OdomLayerConfig c;
  c.width = 100;
  c.height = 40;
  c.resolution = 0.1;
  c.max_frame_dt_seconds = 1.0;
  return c;
}

TEST(OdomLayerIntegration, RayRemovesMovedObjectButDoesNotClearOccludedWall)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  std::vector<Point> floor;
  for (int x = 0; x < 80; ++x) {
    for (int y = 0; y < 4; ++y) {floor.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.0});}
  }
  const Point sensor{0.05, 0.25, 0.20}, ground{0.05, 0.25, 0.0};
  auto points = floor;
  points.push_back({2.05, 0.25, 0.20});
  points.push_back({4.05, 0.25, 0.20});
  for (int t = 1; t <= 3; ++t) {
    ats_rog_map::integrateOdomLayerFrame(layer, columns, points, sensor, ground, t, 1.0);
  }
  ASSERT_TRUE(layer.saveEligible(20, 2));
  ASSERT_TRUE(layer.saveEligible(40, 2));
  // The new ray ends at the old wall. Its interior clears the moved object;
  // cells behind the wall receive no visibility evidence.
  points = {{4.05, 0.25, 0.20}};
  layer.observeHit(60, 2, 3.0, 1.0);
  const auto result = ats_rog_map::integrateOdomLayerFrame(
    layer, columns, points, sensor, ground, 4.0, 1.0);
  EXPECT_GT(result.ray_cleared_cells, 0U);
  EXPECT_EQ(layer.cell(20, 2)->state, ats_rog_map::OdomCellState::kFree);
  EXPECT_FALSE(layer.saveEligible(20, 2));
  EXPECT_EQ(layer.cell(40, 2)->state, ats_rog_map::OdomCellState::kOccupied);
  EXPECT_EQ(layer.cell(60, 2)->last_visible_time, 3.0);
}

TEST(OdomLayerIntegration, UnsupportedNewReturnRevokesOldFreeButNotOccupied)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  layer.observeFree(20, 2, 1.0, 1.0);
  layer.observeHit(40, 2, 1.0, 1.0);
  // Deliberately no connected floor. New uncertain columns must not retain
  // a previous free decision, nor erase occupied evidence.
  ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{2.05, 0.25, 0.25}, {4.05, 0.25, 0.25}},
    {0.05, 0.25, 0.4}, {0.05, 0.25, 0.0}, 2.0, 1.0);
  EXPECT_EQ(layer.cell(20, 2)->state, ats_rog_map::OdomCellState::kUnknown);
  EXPECT_EQ(layer.cell(40, 2)->state, ats_rog_map::OdomCellState::kOccupied);
}

TEST(OdomLayerIntegration, SupportRecheckDoesNotManufactureNeighborObservations)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  layer.observeFree(20, 2, 1.0, 1.0);
  layer.observeHit(22, 2, 1.0, 1.0);
  ats_rog_map::integrateOdomLayerFrame(layer, columns, {{2.15, 0.25, 0.25}},
    {2.15, 0.25, 0.4}, {0.05, 0.25, 0.0}, 2.0, 1.0);
  EXPECT_EQ(layer.cell(20, 2)->state, ats_rog_map::OdomCellState::kUnknown);
  EXPECT_EQ(layer.cell(22, 2)->state, ats_rog_map::OdomCellState::kOccupied);
  EXPECT_DOUBLE_EQ(layer.cell(22, 2)->last_visible_time, 1.0);
  EXPECT_DOUBLE_EQ(layer.cell(22, 2)->hit_time_accum, 1.0);
  EXPECT_EQ(layer.cell(21, 1)->state, ats_rog_map::OdomCellState::kUnknown);
}

TEST(OdomLayerIntegration, LowCeilingOnSlopeCannotEnterGlobalLayerAsFree)
{
  const auto c = config();
  ats_rog_map::GroundColumnConfig gc;
  gc.robot_height = 0.242;
  gc.top_margin = 0.005;
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, gc);
  std::vector<Point> points;
  for (const double x : {0.01, 0.11, 0.21}) {
    for (const double y : {0.01, 0.11, 0.21}) {points.push_back({x, y, 0.3 * x});}
  }
  points.push_back({0.19, 0.15, 0.3 * 0.19 + 0.239});
  ats_rog_map::integrateOdomLayerFrame(layer, columns, points,
    {0.01, 0.01, 0.4}, {0.01, 0.01, 0.003}, 1.0, 1.0);
  EXPECT_EQ(columns.state(1, 1), ats_rog_map::ColumnState::kOccupied);
  EXPECT_EQ(layer.cell(1, 1)->state, ats_rog_map::OdomCellState::kOccupied);
}

TEST(OdomLayerIntegration, SameFrameHitsWinAndRepeatedRaysDoNotPromote)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  std::vector<Point> points;
  for (int x = 0; x < 50; ++x) {points.push_back({(x + 0.5) * 0.1, 0.25, 0.0});}
  for (int i = 0; i < 100; ++i) {
    points.push_back({2.05, 0.25, 0.20});
    points.push_back({4.05, 0.25, 0.20});
  }
  ats_rog_map::integrateOdomLayerFrame(
    layer, columns, points, {0.05, 0.25, 0.20}, {0.05, 0.25, 0}, 1.0, 1.0);
  EXPECT_EQ(layer.cell(20, 2)->state, ats_rog_map::OdomCellState::kOccupied);
  EXPECT_DOUBLE_EQ(layer.cell(20, 2)->hit_time_accum, 1.0);
  EXPECT_FALSE(layer.saveEligible(20, 2));
  EXPECT_FALSE(layer.cell(20, 2)->cleared_once);
}

TEST(OdomLayerIntegration, RayAboveRobotDoesNotClearLowObstacle)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  std::vector<Point> points;
  for (int x = 0; x < 50; ++x) {points.push_back({(x + 0.5) * 0.1, 0.25, 0.0});}
  points.push_back({2.05, 0.25, 0.15});
  ats_rog_map::integrateOdomLayerFrame(
    layer, columns, points, {0.05, 0.25, 0.20}, {0.05, 0.25, 0}, 1.0, 1.0);
  ats_rog_map::integrateOdomLayerFrame(
    layer, columns, {{4.05, 0.25, 0.8}}, {0.05, 0.25, 0.8},
    {0.05, 0.25, 0}, 2.0, 1.0);
  EXPECT_EQ(layer.cell(20, 2)->state, ats_rog_map::OdomCellState::kOccupied);
}

TEST(OdomLayerWindow, GlobalEvidenceSurvivesWindowMovementAndConservativeAggregation)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  layer.observeHit(30, 10, 1.0, 1.0);
  EXPECT_EQ(layer.sampleFootprint(3.09, 1.09, 3.21, 1.21), 100);
  EXPECT_EQ(layer.sampleFootprint(7.0, 1.0, 7.1, 1.1), -1);
  EXPECT_EQ(layer.sampleFootprint(3.0, 1.0, 3.1, 1.1), 100);
  EXPECT_EQ(layer.sampleFootprint(-2.0, 1.0, -1.0, 2.0), -1);
}

TEST(OdomLayerIntegration, PassingRayDoesNotCountAsAnObstacleHit)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  std::vector<Point> points;
  for (int x = 0; x < 50; ++x) {points.push_back({(x + 0.5) * 0.1, 0.25, 0.0});}
  points.push_back({2.05, 0.25, 0.15});
  ats_rog_map::integrateOdomLayerFrame(
    layer, columns, points, {0.05, 0.25, 0.25}, {0.05, 0.25, 0}, 1.0, 1.0);
  for (int t = 2; t <= 4; ++t) {
    ats_rog_map::integrateOdomLayerFrame(
      layer, columns, {{4.05, 0.25, 0.25}}, {0.05, 0.25, 0.25},
      {0.05, 0.25, 0}, t, 1.0);
  }
  EXPECT_DOUBLE_EQ(layer.cell(20, 2)->hit_time_accum, 1.0);
  EXPECT_FALSE(layer.saveEligible(20, 2));
}

TEST(OdomLayerIntegration, OutOfRangeEvidenceRecordsCountAndPosition)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const auto stats = ats_rog_map::integrateOdomLayerFrame(
    layer, columns, {{20.1, 0.25, 0.2}, {-1.0, 0.25, 0.2}},
    {0.05, 0.25, 0.2}, {0.05, 0.25, 0}, 1.0, 1.0);
  ASSERT_EQ(stats.out_of_range, 2U);
  ASSERT_TRUE(stats.out_of_range_sample);
  EXPECT_DOUBLE_EQ((*stats.out_of_range_sample)[0], 20.1);
}

TEST(OdomLayerIntegration, GrazingGroundRayDoesNotEraseTheSurfaceInsideACoarseVoxel)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const Point sensor{0.05, 0.25, 0.20}, ground{0.05, 0.25, 0.0};
  // Both old returns are on a stationary floor. The later, longer ray
  // traverses their coarse voxels above the actual surface, not through it.
  for (int t = 1; t <= 3; ++t) {
    ats_rog_map::integrateOdomLayerFrame(layer, columns,
      {{2.025, 0.25, 0.001}, {4.025, 0.25, 0.001}}, sensor, ground, t, 1.0);
  }
  ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{6.025, 0.25, 0.001}}, sensor, ground, 4.0, 1.0);
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(layer.save("/tmp/ats_odom_layer_test/grazing", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 2U);
}

TEST(OdomLayerIntegration, LaterIntersectingRayStillClearsAfterGrazingTheSameVoxel)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const Point stored{2.025, 0.25, 0.225}, sensor{0.05, 0.25, 0.25};
  for (int t = 1; t <= 3; ++t) {layer.observePoint(stored, t, 1.0);}
  auto grazing_only = layer;
  ats_rog_map::integrateOdomLayerFrame(grazing_only, columns, {{4.025, 0.25, 0.23}},
    sensor, {0.05, 0.25, 0}, 4.0, 1.0);
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(grazing_only.save("/tmp/ats_odom_layer_test/first_grazing", "map", {}, &stats));
  ASSERT_EQ(stats.persistent_voxels, 1U);
  ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{4.025, 0.25, 0.23}, {4.025, 0.25, 0.20}}, sensor, {0.05, 0.25, 0}, 4.0, 1.0);
  ASSERT_TRUE(layer.save("/tmp/ats_odom_layer_test/later_intersection", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 0U);
}

TEST(OdomLayerIntegration, RayDeduplicationHandlesNegativeCoordinatesAndChangingBounds)
{
  auto c = config();
  c.origin_x = -5.0;
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, -5, 0, {});
  const auto first = ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{1.025, 0.025, 0.025}, {1.025, 0.025, 0.025}},
    {0.025, 0.025, 0.025}, {0.025, 0.025, 0}, 1.0, 1.0);
  EXPECT_EQ(first.ray_voxels, 20U);
  const auto next = ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{-1.025, 0.025, 0.025}, {-1.025, 0.025, 0.025}},
    {-0.025, 0.025, 0.025}, {-0.025, 0.025, 0}, 2.0, 1.0);
  EXPECT_EQ(next.ray_voxels, 20U);
}

TEST(OdomLayerIntegration, LargeVoxelBoundsPreserveRayEvidenceWithoutDenseAllocation)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const auto stats = ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{1.025, 0.025, 1000.025}, {1.025, 1000.025, 0.025}},
    {0.025, 0.025, 0.025}, {0.025, 0.025, 0}, 1.0, 1.0);
  EXPECT_GT(stats.ray_voxels, 10000U);
  EXPECT_EQ(stats.out_of_range, 1U);
}

TEST(OdomLayerIntegration, AbsentCloudVoxelCanReceiveAnEndpointInTheNextFrame)
{
  const auto c = config();
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const Point sensor{0.05, 0.25, 0.225}, ground{0.05, 0.25, 0.0};
  ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{4.025, 0.25, 0.225}}, sensor, ground, 1.0, 1.0);
  for (int t = 2; t <= 4; ++t) {
    // The long ray traverses the new endpoint's voxel before that endpoint
    // is integrated. Its absence in the first frame must not be cached later.
    ats_rog_map::integrateOdomLayerFrame(layer, columns,
      {{4.025, 0.25, 0.225}, {2.025, 0.25, 0.225}}, sensor, ground, t, 1.0);
  }
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(layer.save("/tmp/ats_odom_layer_test/later_endpoint", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 2U);
}

TEST(OdomLayerIntegration, NonAlignedCloudVoxelsKeepExactRayIntersectionSemantics)
{
  auto c = config();
  c.cloud_resolution = 0.08;
  ats_rog_map::OdomLayerStore layer(c);
  ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution, 0, 0, {});
  const Point sensor{0.081, 0.081, 0.161}, ground{0.081, 0.081, 0.0};
  for (int t = 1; t <= 3; ++t) {layer.observePoint({2.025, 0.081, 0.161}, t, 1.0);}
  ats_rog_map::integrateOdomLayerFrame(layer, columns,
    {{4.025, 0.081, 0.161}}, sensor, ground, 4.0, 1.0);
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(layer.save("/tmp/ats_odom_layer_test/nonaligned_cloud", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 0U);
}

TEST(OdomLayerIntegration, CandidateSnapshotExcludesClearedAndCurrentFrameReturns)
{
  auto c = config();
  c.cloud_resolution = 0.05;
  ats_rog_map::OdomLayerStore layer(c);
  layer.observePoint({1.025, 0.025, 0.225}, 1.0, 1.0);
  layer.observePoint({2.025, 0.025, 0.225}, 1.0, 1.0);
  layer.observePoint({3.025, 0.025, 0.225}, 2.0, 1.0);
  layer.clearCloudVoxel({2.025, 0.025, 0.225}, 2.0);
  const auto candidates = layer.rayClearCandidates(2.0);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), (std::array<int, 3>{20, 0, 4}));
}

TEST(OdomLayerIntegration, CandidateSnapshotTracksUpdatesAndRemovalAfterCopy)
{
  auto c = config();
  c.cloud_resolution = 0.05;
  ats_rog_map::OdomLayerStore layer(c);
  for (int x = 1; x <= 3; ++x) {layer.observePoint({x + 0.025, 0.025, 0.225}, 1.0, 1.0);}
  auto copy = layer;
  copy.clearCloudVoxel({1.025, 0.025, 0.225}, 2.0);
  copy.observePoint({3.025, 0.025, 0.225}, 3.0, 1.0);
  auto candidates = copy.rayClearCandidates(3.0);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), (std::array<int, 3>{40, 0, 4}));
  copy.clearCloudVoxel({3.025, 0.025, 0.225}, 4.0);
  copy.clearCloudVoxel({2.025, 0.025, 0.225}, 4.0);
  EXPECT_TRUE(copy.rayClearCandidates(5.0).empty());
  EXPECT_EQ(layer.rayClearCandidates(5.0).size(), 3U);
  copy.clear();
  copy.observePoint({4.025, 0.025, 0.225}, 6.0, 1.0);
  ASSERT_EQ(copy.rayClearCandidates(7.0).size(), 1U);
}

TEST(OdomLayerIntegration, NegativeColumnBoundariesRetainBothVisibilityAndEndpointProtection)
{
  for (const double shift : {0.0, 0.013}) {
    auto c = config();
    c.origin_x = -5.0 + shift;
    c.origin_y = -2.0 + shift;
    ats_rog_map::OdomLayerStore layer(c);
    ats_rog_map::GroundColumnGrid columns(c.width, c.height, c.resolution,
      c.origin_x, c.origin_y, {});
    std::vector<Point> points;
    for (int x = 0; x < 50; ++x) {points.push_back({-4.975 + x * .1 + shift, -.975 + shift, 0.0});}
    points.push_back({-2.975 + shift, -.975 + shift, .20});
    points.push_back({-.975 + shift, -.975 + shift, .20});
    for (int t = 1; t <= 3; ++t) {
      ats_rog_map::integrateOdomLayerFrame(layer, columns, points,
        {-4.975 + shift, -.975 + shift, .20}, {-4.975 + shift, -.975 + shift, 0.0}, t, 1.0);
    }
    ASSERT_TRUE(layer.saveEligible(20, 10));
    ats_rog_map::integrateOdomLayerFrame(layer, columns,
      {{-.975 + shift, -.975 + shift, .20}}, {-4.975 + shift, -.975 + shift, .20},
      {-4.975 + shift, -.975 + shift, 0.0}, 4.0, 1.0);
    EXPECT_EQ(layer.cell(20, 10)->state, ats_rog_map::OdomCellState::kFree);
    EXPECT_EQ(layer.cell(40, 10)->state, ats_rog_map::OdomCellState::kOccupied);
  }
}
}  // namespace
