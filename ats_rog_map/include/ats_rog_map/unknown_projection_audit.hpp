// Copyright 2026

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rog_map/rog_map_core/common_lib.hpp"

namespace ats_rog_map
{

// 将数值投影中的严格 all-unknown 状态转换为仅供审计的点中心；混合栅格、损坏 payload
// 或无效姿态一律返回空集，避免把局部 unknown 误写成全图 unknown 可视化。
inline rog_map::vec_E<rog_map::Vec3f> makeAllUnknownProjectionAudit(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  if (width == 0U || height == 0U || width > std::numeric_limits<std::size_t>::max() / height) {
    return {};
  }
  const std::size_t cell_count = width * height;
  if (grid.data.size() != cell_count ||
    !std::all_of(grid.data.begin(), grid.data.end(), [](const std::int8_t cell) {
      return cell == -1;
    }))
  {
    return {};
  }

  const auto & orientation = grid.info.origin.orientation;
  const double norm_sq =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_sq) || norm_sq <= std::numeric_limits<double>::epsilon()) {
    return {};
  }
  const double sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y) /
    norm_sq;
  const double cos_yaw =
    (orientation.w * orientation.w + orientation.x * orientation.x - orientation.y * orientation.y -
    orientation.z * orientation.z) / norm_sq;
  if (!std::isfinite(sin_yaw) || !std::isfinite(cos_yaw) ||
    !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0F)
  {
    return {};
  }

  rog_map::vec_E<rog_map::Vec3f> points;
  points.reserve(cell_count);
  const double resolution = grid.info.resolution;
  for (std::size_t y = 0U; y < height; ++y) {
    for (std::size_t x = 0U; x < width; ++x) {
      const double local_x = (static_cast<double>(x) + 0.5) * resolution;
      const double local_y = (static_cast<double>(y) + 0.5) * resolution;
      points.emplace_back(
        static_cast<float>(grid.info.origin.position.x + cos_yaw * local_x - sin_yaw * local_y),
        static_cast<float>(grid.info.origin.position.y + sin_yaw * local_x + cos_yaw * local_y),
        static_cast<float>(grid.info.origin.position.z));
    }
  }
  return points;
}

}  // namespace ats_rog_map
