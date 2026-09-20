// Copyright 2026

#include <cmath>

#include <gtest/gtest.h>

#include "ats_rog_map/point_cloud_input.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace
{

sensor_msgs::msg::PointCloud2 makeXyzMessage()
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.emplace_back(1.25F, -2.5F, 0.75F);
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(points, message);
  return message;
}

TEST(PointCloudInput, AcceptsXyzWhenIntensityFilterIsDisabled)
{
  const auto decoded = ats_rog_map::decodePointCloudForRogMap(makeXyzMessage(), false);

  ASSERT_EQ(decoded.status, ats_rog_map::PointCloudInputStatus::kAccepted);
  ASSERT_FALSE(decoded.has_intensity);
  ASSERT_EQ(decoded.points.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.points.front().x, 1.25F);
  EXPECT_FLOAT_EQ(decoded.points.front().y, -2.5F);
  EXPECT_FLOAT_EQ(decoded.points.front().z, 0.75F);
  EXPECT_TRUE(std::isnan(decoded.points.front().intensity));
}

TEST(PointCloudInput, RejectsXyzWhenIntensityFilterIsEnabled)
{
  const auto decoded = ats_rog_map::decodePointCloudForRogMap(makeXyzMessage(), true);

  EXPECT_EQ(decoded.status, ats_rog_map::PointCloudInputStatus::kMissingRequiredIntensity);
  EXPECT_FALSE(decoded.has_intensity);
  EXPECT_TRUE(decoded.points.empty());
}

TEST(PointCloudInput, PreservesObservedIntensity)
{
  pcl::PointCloud<pcl::PointXYZI> points;
  pcl::PointXYZI point;
  point.x = 0.25F;
  point.y = 0.5F;
  point.z = -0.75F;
  point.intensity = 42.0F;
  points.push_back(point);
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(points, message);

  const auto decoded = ats_rog_map::decodePointCloudForRogMap(message, true);

  ASSERT_EQ(decoded.status, ats_rog_map::PointCloudInputStatus::kAccepted);
  ASSERT_TRUE(decoded.has_intensity);
  ASSERT_EQ(decoded.points.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.points.front().x, point.x);
  EXPECT_FLOAT_EQ(decoded.points.front().y, point.y);
  EXPECT_FLOAT_EQ(decoded.points.front().z, point.z);
  EXPECT_FLOAT_EQ(decoded.points.front().intensity, point.intensity);
}

TEST(PointCloudInput, RejectsMissingGeometryBeforePclConversion)
{
  sensor_msgs::msg::PointCloud2 message;
  message.fields.clear();

  const auto decoded = ats_rog_map::decodePointCloudForRogMap(message, false);

  EXPECT_EQ(decoded.status, ats_rog_map::PointCloudInputStatus::kMissingGeometry);
  EXPECT_TRUE(decoded.points.empty());
}

}  // namespace
