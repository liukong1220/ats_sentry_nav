// Copyright 2026

#include "ats_rog_map/odom_layer.hpp"

#include <fstream>
#include <limits>

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

}  // namespace
