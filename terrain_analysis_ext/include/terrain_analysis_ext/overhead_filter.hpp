// Copyright 2026

#ifndef TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_
#define TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace terrain_analysis_ext
{

// 体素内逐点累计的高度统计，以及硬障碍判据用到的高度量。
//
// 硬障碍判据是 max(体素高差, 体素内最大相对高度) >= traversabilityObstacleHeightThre，
// 对体素内每一个点一视同仁。悬空在车体上方的板（2027 的 ramp_bridge 下表面离地
// 0.26–0.35 m）因此把下方能通过的路面整片标成硬障碍。overhead_height > 0 时，
// 相对本体素地面达到该值的点属于头顶面，不进入任何统计。

// Match the column gate's inclusive boundary in the point cloud's float
// representation. Rounding the absolute threshold avoids cancellation from
// float subtraction; no geometric tolerance is added. Nonfinite evidence fails closed.
inline bool isOverheadPoint(float point_z, float ground_elevation, double overhead_height)
{
  return overhead_height > 0.0 && std::isfinite(overhead_height) &&
         std::isfinite(point_z) && std::isfinite(ground_elevation) &&
         point_z >= static_cast<float>(static_cast<double>(ground_elevation) + overhead_height);
}

// 把一个点并入体素统计。头顶面返回 false 且不改动任何统计。
inline bool accumulateVoxelPoint(
  float point_z, float ground_elevation, double overhead_height,
  float & max_rel_height, float & min_z, float & max_z, int & point_count)
{
  if (isOverheadPoint(point_z, ground_elevation, overhead_height)) {
    return false;
  }
  const float rel_height = point_z - ground_elevation;
  if (rel_height >= 0.0f) {
    max_rel_height = std::max(max_rel_height, rel_height);
  }
  min_z = std::min(min_z, point_z);
  max_z = std::max(max_z, point_z);
  ++point_count;
  return true;
}

// 体素高差，没有点时为 0。
inline float voxelHeightDiff(float min_z, float max_z)
{
  if (!std::isfinite(min_z) || !std::isfinite(max_z)) {
    return 0.0f;
  }
  return std::max(0.0f, max_z - min_z);
}

// 硬障碍判据用的高度量。
inline float voxelHeightMetric(float min_z, float max_z, float max_rel_height)
{
  return std::max(voxelHeightDiff(min_z, max_z), max_rel_height);
}

}  // namespace terrain_analysis_ext

#endif  // TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_
