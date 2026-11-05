// Copyright 2026

#include <gtest/gtest.h>
#include "small_gicp_relocalization/growing_map.hpp"

TEST(GrowingMap, SeedIsNotAnAcceptedObservationAndCurrentScanCannotSelfMatch)
{
  small_gicp_relocalization::GrowingMapStore map(0.1, 20);
  const auto transform = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(map.seed({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.5}}, transform));
  EXPECT_FALSE(map.seed({{2.0, 0.0, 0.0}}, transform));
  EXPECT_EQ(map.points().size(), 2U);
  EXPECT_FALSE(map.appendAccepted({{2.0, 0.0, 0.0}}, transform, false));
  EXPECT_EQ(map.points().size(), 2U);
  EXPECT_TRUE(map.appendAccepted({{2.0, 0.0, 0.0}}, transform, true));
  EXPECT_EQ(map.points().size(), 3U);
}

TEST(GrowingMap, TransformVoxelDeduplicationAndBoundedStorage)
{
  small_gicp_relocalization::GrowingMapStore map(0.1, 2);
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 0.3);
  ASSERT_TRUE(map.seed({{0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}, {1.0, 0.0, 0.5}}, transform));
  ASSERT_EQ(map.points().size(), 2U);
  EXPECT_NEAR(map.points().front()[2], 0.3, 1e-12);
  EXPECT_FALSE(map.appendAccepted({{3.0, 0.0, 0.0}}, transform, true));
  EXPECT_GT(map.dropped(), 0U);
  map.clear();
  EXPECT_TRUE(map.points().empty());
}

TEST(GrowingMap, BoundsFollowMappingOriginAndAreParameterized)
{
  small_gicp_relocalization::GrowingMapStore map(0.1, 20, {40.0, 30.0, 20.0});
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(25.0, 8.0, 0.0);
  transform.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ASSERT_TRUE(map.seed({{0.0, 0.0, 0.0}, {14.0, 6.0, 0.5}}, transform));
  EXPECT_EQ(map.points().size(), 2U);
  EXPECT_FALSE(map.appendAccepted({{20.1, 0.0, 0.0}}, transform, true));
  EXPECT_EQ(map.dropped(), 1U);
  EXPECT_THROW((small_gicp_relocalization::GrowingMapStore(0.1, 20, {0.0, 30.0, 20.0})),
    std::invalid_argument);
}

TEST(GrowingMap, OffsetBoundsRetainTheFarEndOfTheFullField)
{
  small_gicp_relocalization::GrowingMapStore map(0.1, 20,
    {40.0, 30.0, 20.0}, {-6.0, -15.0, -10.0});
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-0.18, 0.06, 0.201);
  ASSERT_TRUE(map.seed({{0.0, 0.0, 0.0}}, transform));
  EXPECT_TRUE(map.appendAccepted({{28.18, 0.0, 0.5}}, transform, true));
  EXPECT_EQ(map.dropped(), 0U);
  EXPECT_FALSE(map.appendAccepted({{34.0, 0.0, 0.0}}, transform, true));
  EXPECT_EQ(map.dropped(), 1U);
}
