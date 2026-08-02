// Copyright 2026

#include <gtest/gtest.h>

#include <memory>

#include "ats_rog_map/rog_map_core_parameters.hpp"
#include "ats_rog_map/rog_map_engine.hpp"

namespace
{

rog_map::PointCloud makeCloud(float x)
{
  rog_map::PointCloud cloud;
  pcl::PointXYZI point;
  point.x = x;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cloud.push_back(point);
  return cloud;
}

rog_map::Pose makePose(float x, float z = 0.0F)
{
  return {rog_map::Vec3f(x, 0.0F, z), super_utils::Quatf::Identity()};
}

TEST(RogMapEngineSnapshot, InvalidatesEsdfForMapUpdatesAndSlidingResets)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ats_rog_map::RogMapEngine map(clock, ATS_ROG_MAP_TEST_CONFIG);

  EXPECT_EQ(map.generation(), 0U);
  EXPECT_TRUE(map.update(makeCloud(1.0F), makePose(0.0F), makePose(0.0F)));
  EXPECT_EQ(map.generation(), 1U);

  rog_map::Vec3f esdf_min;
  rog_map::Vec3f esdf_max;
  EXPECT_FALSE(map.getCurrentEsdfBounds(esdf_min, esdf_max));
  ASSERT_TRUE(map.ensureCurrentEsdf());
  ASSERT_TRUE(map.getCurrentEsdfBounds(esdf_min, esdf_max));
  EXPECT_LE(esdf_min.x(), 0.0);
  EXPECT_GE(esdf_max.x(), 0.0);

  EXPECT_TRUE(map.update(makeCloud(1.0F), makePose(0.0F), makePose(0.0F)));
  EXPECT_EQ(map.generation(), 2U);
  EXPECT_FALSE(map.getCurrentEsdfBounds(esdf_min, esdf_max));

  ASSERT_TRUE(map.ensureCurrentEsdf());
  EXPECT_TRUE(map.update(makeCloud(11.0F), makePose(10.0F), makePose(10.0F)));
  EXPECT_EQ(map.generation(), 3U);
  EXPECT_FALSE(map.getCurrentEsdfBounds(esdf_min, esdf_max));
  ASSERT_TRUE(map.ensureCurrentEsdf());
  ASSERT_TRUE(map.getCurrentEsdfBounds(esdf_min, esdf_max));
  EXPECT_LE(esdf_min.x(), 10.0);
  EXPECT_GE(esdf_max.x(), 10.0);

  const auto generation_before_rejected_update = map.generation();
  EXPECT_FALSE(map.update(makeCloud(11.0F), makePose(10.0F), makePose(10.0F, 2.0F)));
  EXPECT_EQ(map.generation(), generation_before_rejected_update);
  EXPECT_TRUE(map.getCurrentEsdfBounds(esdf_min, esdf_max));
}

TEST(RogMapCoreParameters, PreservesLegacyConfigDerivations)
{
  const rog_map::Config legacy(ATS_ROG_MAP_TEST_CONFIG);
  ats_rog_map::RogMapCoreParameters parameters;
  parameters.esdf_local_update_box = {4.0, 4.0, 1.0};
  parameters.map_sliding_threshold = 0.5;
  parameters.fix_map_origin = {0.0, 0.0, 0.0};
  parameters.inflation_step = 1;
  parameters.map_size = {6.0, 6.0, 1.0};
  parameters.raycasting_range = {0.10, 5.0};
  parameters.raycasting_local_update_box = {4.0, 4.0, 1.0};
  parameters.stale_decay_soft_ttl_updates = 2;
  parameters.stale_decay_hard_ttl_updates = 3;
  parameters.virtual_ground_height = -0.40;
  parameters.virtual_ceil_height = 0.60;

  const rog_map::Config explicit_config = ats_rog_map::makeRogMapConfig(parameters);
  EXPECT_FLOAT_EQ(explicit_config.l_hit, legacy.l_hit);
  EXPECT_FLOAT_EQ(explicit_config.l_miss, legacy.l_miss);
  EXPECT_FLOAT_EQ(explicit_config.l_occ, legacy.l_occ);
  EXPECT_FLOAT_EQ(explicit_config.l_free, legacy.l_free);
  EXPECT_EQ(explicit_config.half_map_size_i.x(), legacy.half_map_size_i.x());
  EXPECT_EQ(explicit_config.half_map_size_i.y(), legacy.half_map_size_i.y());
  EXPECT_EQ(explicit_config.half_map_size_i.z(), legacy.half_map_size_i.z());
  EXPECT_EQ(explicit_config.local_update_box_i.x(), legacy.local_update_box_i.x());
  EXPECT_EQ(explicit_config.local_update_box_i.y(), legacy.local_update_box_i.y());
  EXPECT_EQ(explicit_config.local_update_box_i.z(), legacy.local_update_box_i.z());
  EXPECT_EQ(explicit_config.inf_spherical_neighbor.size(), legacy.inf_spherical_neighbor.size());
  EXPECT_EQ(explicit_config.spherical_neighbor.size(), legacy.spherical_neighbor.size());
}

}  // namespace
