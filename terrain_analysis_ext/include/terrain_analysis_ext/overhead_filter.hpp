// Copyright 2026

#ifndef TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_
#define TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

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

// 在已排序的 z 序列中估计地面高程分位数：overhead_height > 0 时剔除相对底部支撑簇
// 超过 overhead_height 的头顶面/悬空横梁立面点，防止横梁立面将分位数拉高到横梁表面。
inline float groundBandQuantile(
  const std::vector<float> & sorted_z, double quantile, double overhead_height)
{
  const std::size_t n = sorted_z.size();
  if (n == 0U) {
    return 0.0f;
  }
  std::size_t anchor = 0U;
  for (std::size_t k = 0U; k + 1U < n; ++k) {
    if (sorted_z[k + 1U] - sorted_z[k] <= 0.05f) {
      anchor = k;
      break;
    }
  }
  std::size_t hi = n;
  if (overhead_height > 0.0 && std::isfinite(overhead_height)) {
    const float limit = static_cast<float>(
      static_cast<double>(sorted_z[anchor]) + overhead_height);
    hi = anchor + 1U;
    while (hi < n && sorted_z[hi] < limit) {
      ++hi;
    }
  }
  const std::size_t sub_n = hi - anchor;
  const std::size_t offset = std::min(
    sub_n - 1U,
    static_cast<std::size_t>(std::max(0.0, quantile * static_cast<double>(sub_n))));
  return sorted_z[anchor + offset];
}

}  // namespace terrain_analysis_ext

#endif  // TERRAIN_ANALYSIS_EXT__OVERHEAD_FILTER_HPP_
