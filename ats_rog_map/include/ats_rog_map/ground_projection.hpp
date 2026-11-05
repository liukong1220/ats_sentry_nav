// Copyright 2026

#ifndef ATS_ROG_MAP__GROUND_PROJECTION_HPP_
#define ATS_ROG_MAP__GROUND_PROJECTION_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ats_rog_map
{

/// In-process request for one ground projection of the ROGMap state.
struct GroundProjectionRequest
{
  // Absolute Z obstacle height band in the ROGMap frame.
  float min_height{0.0F};
  float max_height{0.0F};
  // Requested XY resolution. The map never samples finer than its occupancy map.
  float resolution{0.0F};
  // Use the ground-relative, ray-cleared global odom layer cropped to the
  // configured computation window. False keeps the raw Z projection.
  bool use_odom_layer{false};
  // Zero disables identity checking. Mapping callers set both.
  std::uint64_t expected_odom_instance_id{0};
  std::uint64_t expected_odom_epoch{0};
};

/// One immutable ground projection. Field names match the former
/// GetRogMapProjection service response so the planning publisher reads them unchanged.
struct GroundProjection
{
  // 0 free, 100 raw occupied, -1 unknown. Downstream owns metric clearance/inflation.
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  std::uint64_t generation{0};
  std::uint64_t odom_instance_id{0};
  std::uint64_t odom_epoch{0};
  bool odom_layer_included{false};
  bool ready{false};
  bool stale{true};
  // Row-major, metric signed distance. Positive free, negative occupied, NaN unknown.
  std::vector<float> signed_distance;
  // Row-major gradient in occupancy_grid.header.frame_id. Unknown cells are NaN.
  std::vector<float> gradient_x;
  std::vector<float> gradient_y;
};

/// Computes a projection synchronously. Must be safe to call from a thread other
/// than the map node's executor threads.
using GroundProjectionProvider =
  std::function<void(const GroundProjectionRequest &, GroundProjection &)>;

}  // namespace ats_rog_map

#endif  // ATS_ROG_MAP__GROUND_PROJECTION_HPP_
