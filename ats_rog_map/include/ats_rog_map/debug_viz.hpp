// Copyright 2026

#ifndef ATS_ROG_MAP__DEBUG_VIZ_HPP_
#define ATS_ROG_MAP__DEBUG_VIZ_HPP_

#include <cstddef>
#include <cstdint>

#include "rog_map/prob_map.h"

namespace ats_rog_map
{

struct VoxelDebugRgb
{
  std::uint8_t red{130U};
  std::uint8_t green{120U};
  std::uint8_t blue{150U};
};

constexpr VoxelDebugRgb voxelDebugRgb(const rog_map::GridType type)
{
  switch (type) {
    case super_utils::OCCUPIED:
      return {245U, 70U, 70U};
    case super_utils::KNOWN_FREE:
      return {92U, 220U, 235U};
    default:
      return {};
  }
}

constexpr bool shouldBuildDebugVisualization(const std::size_t subscription_count)
{
  return subscription_count > 0U;
}

}  // namespace ats_rog_map

#endif  // ATS_ROG_MAP__DEBUG_VIZ_HPP_
