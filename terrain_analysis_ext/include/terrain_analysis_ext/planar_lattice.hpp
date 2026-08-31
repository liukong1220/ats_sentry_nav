// Copyright 2026

#ifndef TERRAIN_ANALYSIS_EXT__PLANAR_LATTICE_HPP_
#define TERRAIN_ANALYSIS_EXT__PLANAR_LATTICE_HPP_

#include <cmath>

namespace terrain_analysis_ext
{

// 平面体素栅格的相位规则。
//
// 栅格窗口随车滚动（内存有界），但相位吸附到世界系 size 的整数倍上：体素边界恒为
// size 的整数倍，不随车体连续滑动。这样做的原因是量化过报必须是确定的——锚在车上
// 时，同一面墙被判定出的面值会在一个完整体素宽度内随车摆动（RMUC 高地坡沿在
// 0.40 m 体素下实测在 9.4~9.8 m 之间摆），同一条轨迹于是时而可行时而被 footprint
// 判负，run-to-run 不可复现。吸附相位不降低保守性（体素内量化仍朝占据一侧取整），
// 只是把随机相位噪声去掉。

// 把 value 吸附到"边界落在 size 整数倍上"的那个体素的中心。
inline float snapToVoxelCenter(float value, float size)
{
  const float pitch = (size > 0.0f) ? size : 1e-3f;
  return std::floor(value / pitch) * pitch + 0.5f * pitch;
}

// 一维体素下标。anchor 必须来自 snapToVoxelCenter，half_width 是中心格下标。
// 保留原实现的"截断再补偿"写法所对应的语义，即对负值同样向下取整。
inline int planarVoxelIndex1D(float coordinate, float anchor, float size, int half_width)
{
  const float pitch = (size > 0.0f) ? size : 1e-3f;
  const float shifted = coordinate - anchor + 0.5f * pitch;
  return static_cast<int>(std::floor(shifted / pitch)) + half_width;
}

// 体素下标对应的世界系西/南边界。断言"边界是 size 的整数倍"用得到。
inline float planarVoxelLowerEdge(int index, float anchor, float size, int half_width)
{
  const float pitch = (size > 0.0f) ? size : 1e-3f;
  return anchor + static_cast<float>(index - half_width) * pitch - 0.5f * pitch;
}

}  // namespace terrain_analysis_ext

#endif  // TERRAIN_ANALYSIS_EXT__PLANAR_LATTICE_HPP_
