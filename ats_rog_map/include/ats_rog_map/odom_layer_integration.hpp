// Copyright 2026

#ifndef ATS_ROG_MAP__ODOM_LAYER_INTEGRATION_HPP_
#define ATS_ROG_MAP__ODOM_LAYER_INTEGRATION_HPP_

#include <array>
#include <optional>
#include <vector>
#include "ats_rog_map/ground_column.hpp"
#include "ats_rog_map/odom_layer.hpp"

namespace ats_rog_map
{
struct OdomLayerFrameStats
{
  std::size_t out_of_range{0};
  std::optional<std::array<double, 3>> out_of_range_sample;
  std::size_t non_obstacle{0};
  std::size_t ray_cleared_cells{0};
  std::size_t ray_voxels{0};
  double prepare_ms{0.0};
  double ray_ms{0.0};
  double ground_ms{0.0};
  double evidence_ms{0.0};
};

OdomLayerFrameStats integrateOdomLayerFrame(
  OdomLayerStore & layer, GroundColumnGrid & columns,
  const std::vector<std::array<double, 3>> & points,
  const std::array<double, 3> & sensor, const std::array<double, 3> & robot_ground,
  double stamp, double frame_dt);
}  // namespace ats_rog_map
#endif  // ATS_ROG_MAP__ODOM_LAYER_INTEGRATION_HPP_
