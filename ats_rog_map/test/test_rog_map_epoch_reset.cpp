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

TEST(RogMapEngineSnapshot, FrontendEpochResetClearsAllRepresentations)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ats_rog_map::RogMapEngine map(clock, ATS_ROG_MAP_TEST_CONFIG);
  ASSERT_TRUE(map.update(makeCloud(1.0F), makePose(0.0F), makePose(0.0F)));
  const auto generation = map.generation();
  map.resetForOdomEpoch();
  EXPECT_GT(map.generation(), generation);
  EXPECT_EQ(map.getGridType(rog_map::Vec3f(1.0F, 0.0F, 0.0F)), super_utils::UNKNOWN);
  rog_map::Vec3f lower, upper;
  EXPECT_FALSE(map.getCurrentEsdfBounds(lower, upper));
  ASSERT_TRUE(map.update(makeCloud(2.0F), makePose(0.0F), makePose(0.0F)));
  EXPECT_GT(map.generation(), generation + 1);
}

}  // namespace
