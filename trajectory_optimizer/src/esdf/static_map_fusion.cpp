// Copyright 2026

#include "trajectory_optimizer/esdf/static_map_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace trajectory_optimizer
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 * (
    static_cast<double>(quaternion.w) * static_cast<double>(quaternion.z) +
    static_cast<double>(quaternion.x) * static_cast<double>(quaternion.y));
  const double cos_yaw = 1.0 - 2.0 * (
    static_cast<double>(quaternion.y) * static_cast<double>(quaternion.y) +
    static_cast<double>(quaternion.z) * static_cast<double>(quaternion.z));
  return std::atan2(sin_yaw, cos_yaw);
}

bool hasValidStorage(const nav_msgs::msg::OccupancyGrid & grid)
{
  const std::size_t size =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  return grid.info.width > 0 && grid.info.height > 0 && grid.info.resolution > 0.0F &&
         grid.data.size() >= size;
}

}  // namespace

bool StaticMapFusion::buildPlanningGrid(
  const nav_msgs::msg::OccupancyGrid & local_grid,
  const nav_msgs::msg::OccupancyGrid & static_map,
  const geometry_msgs::msg::TransformStamped & static_from_local,
  const StaticMapFusionParams & params,
  nav_msgs::msg::OccupancyGrid & planning_grid)
{
  if (!hasValidStorage(local_grid) || !hasValidStorage(static_map)) {
    return false;
  }

  planning_grid = local_grid;
  planning_grid.data.assign(
    static_cast<std::size_t>(local_grid.info.width) *
    static_cast<std::size_t>(local_grid.info.height), -1);

  const int static_threshold = std::max(0, std::min(100, params.static_obstacle_value_threshold));
  const int local_threshold = std::max(0, std::min(100, params.local_obstacle_value_threshold));
  const double local_origin_yaw = yawFromQuaternion(local_grid.info.origin.orientation);
  const double local_origin_cos = std::cos(local_origin_yaw);
  const double local_origin_sin = std::sin(local_origin_yaw);
  const double transform_yaw = yawFromQuaternion(static_from_local.transform.rotation);
  const double transform_cos = std::cos(transform_yaw);
  const double transform_sin = std::sin(transform_yaw);
  const double static_origin_yaw = yawFromQuaternion(static_map.info.origin.orientation);
  const double static_origin_cos = std::cos(static_origin_yaw);
  const double static_origin_sin = std::sin(static_origin_yaw);

  // The static map is normally much finer than the rolling terrain grid. Sample
  // every local cell at the static-map scale so a narrow wall cannot disappear
  // merely because it falls between two local-cell centres.
  const int samples_per_axis = std::max(
    1, static_cast<int>(std::ceil(local_grid.info.resolution / static_map.info.resolution)));

  for (unsigned int y = 0; y < local_grid.info.height; ++y) {
    for (unsigned int x = 0; x < local_grid.info.width; ++x) {
      const std::size_t local_index =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(local_grid.info.width) + x;
      bool static_occupied = false;
      bool static_unknown = false;
      for (int sample_y = 0; sample_y < samples_per_axis && !static_occupied; ++sample_y) {
        for (int sample_x = 0; sample_x < samples_per_axis; ++sample_x) {
          const double local_x =
            (static_cast<double>(x) +
            (static_cast<double>(sample_x) + 0.5) / samples_per_axis) *
            local_grid.info.resolution;
          const double local_y =
            (static_cast<double>(y) +
            (static_cast<double>(sample_y) + 0.5) / samples_per_axis) *
            local_grid.info.resolution;
          const double world_x = local_grid.info.origin.position.x +
            local_origin_cos * local_x - local_origin_sin * local_y;
          const double world_y = local_grid.info.origin.position.y +
            local_origin_sin * local_x + local_origin_cos * local_y;
          const double static_x = static_from_local.transform.translation.x +
            transform_cos * world_x - transform_sin * world_y;
          const double static_y = static_from_local.transform.translation.y +
            transform_sin * world_x + transform_cos * world_y;
          const double origin_dx = static_x - static_map.info.origin.position.x;
          const double origin_dy = static_y - static_map.info.origin.position.y;
          const double map_x =
            (static_origin_cos * origin_dx + static_origin_sin * origin_dy) /
            static_map.info.resolution;
          const double map_y =
            (-static_origin_sin * origin_dx + static_origin_cos * origin_dy) /
            static_map.info.resolution;
          const int map_ix = static_cast<int>(std::floor(map_x));
          const int map_iy = static_cast<int>(std::floor(map_y));
          if (map_ix < 0 || map_iy < 0 || map_ix >= static_cast<int>(static_map.info.width) ||
            map_iy >= static_cast<int>(static_map.info.height))
          {
            static_unknown = true;
            continue;
          }
          const std::size_t static_index =
            static_cast<std::size_t>(map_iy) * static_cast<std::size_t>(static_map.info.width) +
            static_cast<std::size_t>(map_ix);
          const int8_t static_value = static_map.data[static_index];
          static_unknown = static_unknown || static_value < 0;
          static_occupied = static_occupied || static_value >= static_threshold;
          if (static_occupied) {
            break;
          }
        }
      }
      if (static_occupied) {
        planning_grid.data[local_index] = 100;
        continue;
      }
      if (static_unknown) {
        continue;
      }

      const int8_t local_value = local_grid.data[local_index];
      if (local_value >= local_threshold) {
        planning_grid.data[local_index] = 100;
      } else if (local_value >= 0) {
        planning_grid.data[local_index] = local_value;
      } else {
        // Static-map free space remains free when the rolling terrain grid has
        // not observed it yet. This cannot erase a static obstacle because that
        // branch returned above.
        planning_grid.data[local_index] = 0;
      }
    }
  }

  return true;
}

}  // namespace trajectory_optimizer
