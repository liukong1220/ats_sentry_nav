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

/// 首选净空路径相对直线距离是否值得再用下限档搜一次。
///
/// 首选档(全 yaw 外接半径)比窄通道半宽还大时,图搜索在这一档绕远路也能成功,
/// 梯子因此永远不会降档,车会放弃几米长的隧道去绕十几米(RMUL 隧道净宽 0.80 m,
/// 首选 0.436 m 大于半宽 0.40 m)。只有首选路径明显长于直线距离时才付出第二次搜索的代价。
/// ratio <= 0 关闭,与旧行为一致。
inline bool shouldProbeRelaxedClearance(
  double preferred_length, double straight_distance, double detour_ratio)
{
  if (!(detour_ratio > 0.0) || !std::isfinite(preferred_length) ||
    !std::isfinite(straight_distance) || !(straight_distance > 0.0))
  {
    return false;
  }
  return preferred_length > detour_ratio * straight_distance;
}

/// 下限档找到的路径是否应取代首选档路径:只在它明显更短时取代。
/// 取代后安全性仍由矩形足迹门禁与局部修复裁定,这里不放宽任何安全判据。
inline bool shouldTakeRelaxedRoute(
  double preferred_length, double relaxed_length, double detour_ratio)
{
  if (!(detour_ratio > 0.0) || !std::isfinite(preferred_length) ||
    !std::isfinite(relaxed_length) || !(relaxed_length > 0.0))
  {
    return false;
  }
  return preferred_length > detour_ratio * relaxed_length;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__CLEARANCE_LADDER_HPP_
