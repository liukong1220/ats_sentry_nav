// Copyright 2026

#pragma once

#include <algorithm>
#include <limits>
#include <utility>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rog_map/rog_map_core/config.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ats_rog_map
{

enum class PointCloudInputStatus
{
  kAccepted,
  kMissingGeometry,
  kMissingRequiredIntensity,
};

struct DecodedPointCloud
{
  PointCloudInputStatus status{PointCloudInputStatus::kMissingGeometry};
  bool has_intensity{false};
  rog_map::PointCloud points;
};

inline bool hasPointField(const sensor_msgs::msg::PointCloud2 & message, const char * name)
{
  return std::any_of(
    message.fields.begin(), message.fields.end(),
    [name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
}

inline const char * pointCloudInputStatusString(const PointCloudInputStatus status)
{
  switch (status) {
    case PointCloudInputStatus::kAccepted:
      return "accepted";
    case PointCloudInputStatus::kMissingGeometry:
      return "missing x/y/z geometry fields";
    case PointCloudInputStatus::kMissingRequiredIntensity:
      return "missing required intensity field";
  }
  return "unknown point-cloud input status";
}

// Decode the geometry required by ROGMap without asking PCL to synthesize a missing
// intensity field. A configured intensity filter is a sensor-quality contract, so an
// XYZ-only message is deliberately rejected when that filter is active.
inline DecodedPointCloud decodePointCloudForRogMap(
  const sensor_msgs::msg::PointCloud2 & message, const bool require_intensity)
{
  DecodedPointCloud decoded;
  if (!hasPointField(message, "x") || !hasPointField(message, "y") ||
    !hasPointField(message, "z"))
  {
    return decoded;
  }

  decoded.has_intensity = hasPointField(message, "intensity");
  if (!decoded.has_intensity) {
    if (require_intensity) {
      decoded.status = PointCloudInputStatus::kMissingRequiredIntensity;
      return decoded;
    }

    pcl::PointCloud<pcl::PointXYZ> xyz_points;
    pcl::fromROSMsg(message, xyz_points);
    decoded.points.reserve(xyz_points.size());
    for (const auto & xyz : xyz_points) {
      pcl::PointXYZI point;
      point.x = xyz.x;
      point.y = xyz.y;
      point.z = xyz.z;
      // No physical intensity was observed. Keep this distinction explicit so a
      // future intensity consumer cannot mistake a default value for a reading.
      point.intensity = std::numeric_limits<float>::quiet_NaN();
      decoded.points.push_back(point);
    }
  } else {
    pcl::fromROSMsg(message, decoded.points);
  }

  decoded.status = PointCloudInputStatus::kAccepted;
  return decoded;
}

}  // namespace ats_rog_map
