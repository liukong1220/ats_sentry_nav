// Copyright 2026

#ifndef MINCO_PLANNER__PLANNING__CLEARANCE_LADDER_HPP_
#define MINCO_PLANNER__PLANNING__CLEARANCE_LADDER_HPP_

#include <algorithm>
#include <cmath>

namespace minco_planner
{

/// 栅格量化余量:半个格对角线。
///
/// `measureGridClearance` 量的是格心到格心的距离,而矩形 footprint gate 判的是格子的
/// 占据面积。一个格心距为 d 的占据格,它最近的边缘可能只有 d - res*sqrt(2)/2。
/// 所以"按某个 clearance 找到的路径"要想通过 footprint gate,这个 clearance 必须比
/// footprint 半宽多留半个格对角线,两道门用的才是同一套几何。
inline double gridQuantizationAllowance(double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 0.0;
  }
  return resolution * M_SQRT1_2;
}

/// 由内切半宽和栅格分辨率推出与 footprint gate 一致的 clearance 下限。
///
/// 不加这个余量时,下限恰好等于内切半宽:图搜索会不断在下限上找到路径,而 footprint
/// gate 必然拒绝它们——既不是可通行,也不是不可达,而是在目标超时之前一直空转。
/// 观测证据:RMUC west corridor 上 JPS 在 0.420 m 全失败、放宽到 0.270 m 全成功,
/// 随后 footprint gate 以 100~204 个冲突逐次拒绝,直到目标超时。
inline double footprintConsistentClearanceFloor(
  double inscribed_radius, double resolution, double preferred)
{
  const double consistent =
    std::max(0.0, inscribed_radius) + gridQuantizationAllowance(resolution);
  // 下限永远不高于 preferred,否则梯子会退化成一级并丢掉窄通道的放宽能力。
  return std::min(consistent, std::max(0.0, preferred));
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__CLEARANCE_LADDER_HPP_
