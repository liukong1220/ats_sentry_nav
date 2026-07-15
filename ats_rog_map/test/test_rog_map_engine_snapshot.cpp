// Copyright 2026

#include <gtest/gtest.h>

#include <memory>

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

}  // namespace
