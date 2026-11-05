// Copyright 2026

#include "ats_rog_map/odom_layer.hpp"

#include <fstream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cmath>

#include "gtest/gtest.h"

namespace
{

using ats_rog_map::OdomCellLayer;
using ats_rog_map::OdomCellState;
using ats_rog_map::OdomLayerConfig;
using ats_rog_map::OdomLayerStore;

OdomLayerStore makeStore()
{
  OdomLayerConfig config;
  config.width = 10;
  config.height = 10;
  config.resolution = 1.0;
  config.static_promotion_seconds = 3.0;
  config.dynamic_hold_seconds = 3.0;
  config.max_frame_dt_seconds = 1.0;
  config.save_persistence_ratio = 0.5;
  return OdomLayerStore(config);
}

TEST(OdomLayer, PromotesByAccumulatedHitTime)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ASSERT_TRUE(store.observeHit(2, 3, 2.0, 1.0));
  EXPECT_EQ(store.cell(2, 3)->layer, OdomCellLayer::kDynamic);
  ASSERT_TRUE(store.observeHit(2, 3, 3.0, 1.0));
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
  EXPECT_EQ(store.cell(2, 3)->layer, OdomCellLayer::kStatic);
}

TEST(OdomLayer, RepeatedReturnsInOneFrameDoNotPromote)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(store.observeHit(2, 3, 1.0, 1.0));
  }
  EXPECT_EQ(store.cell(2, 3)->layer, OdomCellLayer::kDynamic);
  EXPECT_FALSE(store.saveEligible(2, 3));
  EXPECT_DOUBLE_EQ(store.cell(2, 3)->hit_time_accum, 1.0);
}

TEST(OdomLayer, OutOfViewDoesNotClearAndHoldExpiresToUnknown)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  store.tick(3.5);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
  store.tick(4.1);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kUnknown);
}

TEST(OdomLayer, RayClearRequiresVisibilityAndMarksCellForSaveExclusion)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ASSERT_TRUE(store.observeHit(2, 3, 2.0, 1.0));
  ASSERT_TRUE(store.observeHit(2, 3, 3.0, 1.0));
  EXPECT_FALSE(store.observeRayClear(2, 3, 4.0, false));
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
  EXPECT_TRUE(store.observeRayClear(2, 3, 4.0, true));
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kUnknown);
  EXPECT_TRUE(store.cell(2, 3)->cleared_once);
  EXPECT_FALSE(store.saveEligible(2, 3));
}

TEST(OdomLayer, VisibleDynamicCellDoesNotExpireWithoutRayClear)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ASSERT_TRUE(store.observeVisible(2, 3, 5.0));
  store.tick(5.0);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
  store.tick(8.1);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kUnknown);
}

TEST(OdomLayer, OldOrInvalidVisibilityCannotExtendHold)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 2.0, 1.0));
  EXPECT_FALSE(store.observeVisible(2, 3, 1.0));
  EXPECT_FALSE(store.observeVisible(2, 3, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(store.observeVisible(-1, 3, 4.0));
  store.tick(5.1);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kUnknown);
}

TEST(OdomLayer, InvalidEpochTransformClearsLayer)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ats_rog_map::OdomLayerTransform invalid;
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(store.rebase(invalid));
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kUnknown);
}

TEST(OdomLayer, ValidEpochTransformPreservesCells)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ats_rog_map::OdomLayerTransform identity;
  EXPECT_TRUE(store.rebase(identity));
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
}

TEST(OdomLayer, SavesPersistentOccupiedCells)
{
  auto store = makeStore();
  ASSERT_TRUE(store.observeHit(2, 3, 1.0, 1.0));
  ASSERT_TRUE(store.observeHit(2, 3, 2.0, 1.0));
  ASSERT_TRUE(store.observeHit(2, 3, 3.0, 1.0));
  const auto prefix = std::string("/tmp/ats_odom_layer_test/map");
  EXPECT_TRUE(store.save(prefix, "odom"));
  std::ifstream yaml(prefix + ".yaml");
  std::ifstream pcd(prefix + ".pcd");
  EXPECT_TRUE(yaml.good());
  EXPECT_TRUE(pcd.good());
}

TEST(OdomLayer, GlobalLayerDefaultDimensions)
{
  const OdomLayerConfig config;
  EXPECT_EQ(config.width, 400U);
  EXPECT_EQ(config.height, 300U);
  EXPECT_DOUBLE_EQ(config.resolution, 0.10);
  EXPECT_EQ(static_cast<std::size_t>(config.width) * config.height, 120000U);
}

TEST(OdomLayerSave, TransformsGridAndPersistentThreeDimensionalCloudOnce)
{
  auto store = makeStore();
  const std::array<double, 3> wall{2.25, 3.25, 0.4};
  const std::array<double, 3> floor{1.25, 1.25, -0.1};
  for (int t = 1; t <= 3; ++t) {
    store.observeHit(2, 3, t, 1.0);
    store.observePoint(wall, t, 1.0);
    store.observePoint(floor, t, 1.0);
  }
  store.observePoint({4.25, 4.25, 0.3}, 3.0, 1.0);  // transient, not saved
  ats_rog_map::OdomLayerTransform map_from_odom;
  map_from_odom.x = 10.0;
  map_from_odom.y = -2.0;
  map_from_odom.yaw = std::acos(-1.0) / 2.0;
  map_from_odom.z = 0.5;
  ats_rog_map::OdomLayerSaveStats stats;
  const std::string prefix = "/tmp/ats_odom_layer_test/transformed";
  ASSERT_TRUE(store.save(prefix, "map", map_from_odom, &stats));
  EXPECT_EQ(stats.occupied_cells, 1U);
  EXPECT_EQ(stats.persistent_cells, 1U);
  EXPECT_EQ(stats.persistent_voxels, 2U);
  std::ifstream yaml(prefix + ".yaml");
  const std::string metadata((std::istreambuf_iterator<char>(yaml)), {});
  EXPECT_NE(metadata.find("origin: [10.000000, -2.000000, 1.570796]"), std::string::npos);
  EXPECT_NE(metadata.find("map_from_odom: [10.000000, -2.000000, 0.500000, 1.570796]"),
    std::string::npos);
  std::ifstream pcd(prefix + ".pcd");
  std::string line;
  while (std::getline(pcd, line) && line != "DATA ascii") {}
  std::vector<std::array<double, 3>> points;
  double x, y, z;
  while (pcd >> x >> y >> z) {points.push_back({x, y, z});}
  ASSERT_EQ(points.size(), 2U);
  EXPECT_NEAR(points[0][0], 8.75, 1e-6);
  EXPECT_NEAR(points[0][1], -0.75, 1e-6);
  EXPECT_NEAR(points[0][2], 0.4, 1e-6);
  EXPECT_NEAR(points[1][0], 6.75, 1e-6);
  EXPECT_NEAR(points[1][1], 0.25, 1e-6);
  EXPECT_NEAR(points[1][2], 0.9, 1e-6);
  EXPECT_EQ(store.cell(2, 3)->state, OdomCellState::kOccupied);
}

TEST(OdomLayerSave, RejectsNonFiniteTransformBeforeWriting)
{
  auto store = makeStore();
  ats_rog_map::OdomLayerTransform transform;
  transform.yaw = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(store.save("/tmp/ats_odom_layer_test/invalid", "map", transform));
}

TEST(OdomLayerSave, SparseScanReturnsAccumulateTheirOwnBoundedObservationInterval)
{
  OdomLayerConfig config;
  config.width = 10;
  config.height = 10;
  config.max_frame_dt_seconds = 0.25;
  OdomLayerStore store(config);
  // The frontend publishes at 20 Hz, but a moving scan revisits this voxel
  // only every fourth frame. A.2 accumulates t - last_hit, capped per return.
  for (int i = 0; i < 17; ++i) {
    store.observePoint({0.25, 0.25, 0.001}, 1.0 + i * 0.2, 0.05);
  }
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(store.save("/tmp/ats_odom_layer_test/sparse", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 1U);
}

TEST(OdomLayerSave, ThreeDimensionalTransformPreservesPitchAndConservativeRaster)
{
  auto store = makeStore();
  for (int t = 1; t <= 3; ++t) {
    store.observeHit(2, 3, t, 1.0);
    store.observePoint({2.25, 3.25, 0.4}, t, 1.0);
  }
  ats_rog_map::OdomLayerTransform transform;
  transform.pitch = std::acos(-1.0) / 6.0;
  ats_rog_map::OdomLayerSaveStats stats;
  const std::string prefix = "/tmp/ats_odom_layer_test/pitched";
  ASSERT_TRUE(store.save(prefix, "map", transform, &stats));
  std::ifstream pcd(prefix + ".pcd");
  std::string line;
  while (std::getline(pcd, line) && line != "DATA ascii") {}
  double x, y, z;
  ASSERT_TRUE(static_cast<bool>(pcd >> x >> y >> z));
  EXPECT_NEAR(x, std::cos(transform.pitch) * 2.25 + std::sin(transform.pitch) * 0.4, 1e-6);
  EXPECT_NEAR(y, 3.25, 1e-6);
  EXPECT_NEAR(z, -std::sin(transform.pitch) * 2.25 + std::cos(transform.pitch) * 0.4, 1e-6);
  EXPECT_EQ(stats.persistent_cells, 1U);
  EXPECT_EQ(stats.raster_occupied_cells, 2U);
}

TEST(OdomLayerSave, ClearingDynamicObjectExcludesItsThreeDimensionalReturns)
{
  auto store = makeStore();
  for (int t = 1; t <= 3; ++t) {
    store.observeHit(2, 3, t, 1.0);
    store.observePoint({2.25, 3.25, 0.25}, t, 1.0);
  }
  ASSERT_TRUE(store.observeRayClear(2, 3, 4.0, true));
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(store.save("/tmp/ats_odom_layer_test/cleared", "map", {}, &stats));
  EXPECT_EQ(stats.persistent_voxels, 0U);
  EXPECT_EQ(stats.persistent_cells, 0U);
}

TEST(OdomLayerSave, OccupiedCountAndSaveEligibleCountAreExplicitlyDifferent)
{
  OdomLayerConfig config;
  config.width = 1000;
  config.height = 1;
  config.resolution = 0.1;
  config.max_frame_dt_seconds = 1.0;
  OdomLayerStore store(config);
  for (int x = 0; x < 942; ++x) {
    store.observeHit(x, 0, 1.0, 1.0);
    if (x < 306) {
      store.observeHit(x, 0, 2.0, 1.0);
      store.observeHit(x, 0, 3.0, 1.0);
    }
  }
  ats_rog_map::OdomLayerSaveStats stats;
  ASSERT_TRUE(store.save("/tmp/ats_odom_layer_test/count_ledger", "map", {}, &stats));
  EXPECT_EQ(stats.occupied_cells, 942U);
  EXPECT_EQ(stats.persistent_cells, 306U);
  std::ifstream pgm("/tmp/ats_odom_layer_test/count_ledger.pgm", std::ios::binary);
  std::string header;
  std::getline(pgm, header);
  std::getline(pgm, header);
  std::getline(pgm, header);
  std::vector<unsigned char> pixels((std::istreambuf_iterator<char>(pgm)), {});
  EXPECT_EQ(std::count(pixels.begin(), pixels.end(), 0U), 306);
  std::ifstream pcd("/tmp/ats_odom_layer_test/count_ledger.cells.pcd");
  const std::string data((std::istreambuf_iterator<char>(pcd)), {});
  EXPECT_NE(data.find("POINTS 306\n"), std::string::npos);
}

}  // namespace
