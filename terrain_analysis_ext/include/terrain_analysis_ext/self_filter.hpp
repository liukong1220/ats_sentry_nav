// Copyright 2026

#ifndef TERRAIN_ANALYSIS_EXT__SELF_FILTER_HPP_
#define TERRAIN_ANALYSIS_EXT__SELF_FILTER_HPP_

#include <cmath>

namespace terrain_analysis_ext
{

// 车体自身回波（轮子、舵轮支架、ToF 传感器）落在 Point-LIO 盲区球之外，
// 会被体素统计当成离地 0.05~0.16 m 的硬障碍，在车身侧面生成 0.2 m 硬格。
// radius <= 0 关闭过滤（实机 profile 默认行为不变）。
inline bool insideSelfFilter(
  float point_x, float point_y, float vehicle_x, float vehicle_y, double radius)
{
  if (!(radius > 0.0)) {
    return false;
  }
  const double dx = static_cast<double>(point_x) - static_cast<double>(vehicle_x);
  const double dy = static_cast<double>(point_y) - static_cast<double>(vehicle_y);
  return std::hypot(dx, dy) < radius;
}

}  // namespace terrain_analysis_ext

#endif  // TERRAIN_ANALYSIS_EXT__SELF_FILTER_HPP_
