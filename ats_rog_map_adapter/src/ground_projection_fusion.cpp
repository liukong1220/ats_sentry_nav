// Copyright 2026

#include "ats_rog_map_adapter/ground_projection_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_rog_map_adapter
{
namespace
{

bool validGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  return grid.info.resolution > 0.0 && grid.info.width > 0 && grid.info.height > 0 &&
         grid.data.size() == cell_count;
}

bool sampleGrid(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y, int8_t & value)
{
  if (!validGrid(grid)) {
    return false;
  }
  tf2::Quaternion origin_rotation;
  tf2::fromMsg(grid.info.origin.orientation, origin_rotation);
  const double yaw = tf2::getYaw(origin_rotation);
  const double dx = world_x - grid.info.origin.position.x;
  const double dy = world_y - grid.info.origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const int mx = static_cast<int>(std::floor(local_x / grid.info.resolution));
  const int my = static_cast<int>(std::floor(local_y / grid.info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) ||
    my >= static_cast<int>(grid.info.height))
  {
    return false;
  }
  value = grid.data[
    static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(mx)];
  return true;
}

void projectionCellCenter(
  const nav_msgs::msg::OccupancyGrid & grid, unsigned int mx, unsigned int my,
  double & world_x, double & world_y)
{
  tf2::Quaternion origin_rotation;
  tf2::fromMsg(grid.info.origin.orientation, origin_rotation);
  const double yaw = tf2::getYaw(origin_rotation);
  const double local_x = (static_cast<double>(mx) + 0.5) * grid.info.resolution;
  const double local_y = (static_cast<double>(my) + 0.5) * grid.info.resolution;
  world_x = grid.info.origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
  world_y = grid.info.origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y;
}

void aggregateStaticFootprint(
  const nav_msgs::msg::OccupancyGrid & static_map,
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  unsigned int output_mx, unsigned int output_my, int obstacle_threshold,
  bool & occupied, bool & known_free, int & value)
{
  // planning_grid copies the static map origin and orientation, so their cell footprints are
  // axis-aligned in the same map-local coordinates even when the map is rotated in world space.
  const long double static_resolution = static_map.info.resolution;
  const long double output_resolution = planning_grid.info.resolution;
  const long double output_min_x = output_mx * output_resolution;
  const long double output_min_y = output_my * output_resolution;
  const long double output_max_x = std::min(
    (output_mx + 1U) * output_resolution,
    static_cast<long double>(static_map.info.width) * static_resolution);
  const long double output_max_y = std::min(
    (output_my + 1U) * output_resolution,
    static_cast<long double>(static_map.info.height) * static_resolution);
  if (output_min_x >= output_max_x || output_min_y >= output_max_y) {
    return;
  }

  const int first_x = std::max(
    0, static_cast<int>(std::floor(output_min_x / static_resolution)));
  const int first_y = std::max(
    0, static_cast<int>(std::floor(output_min_y / static_resolution)));
  const int last_x = std::min(
    static_cast<int>(static_map.info.width) - 1,
    static_cast<int>(std::ceil(output_max_x / static_resolution)) - 1);
  const int last_y = std::min(
    static_cast<int>(static_map.info.height) - 1,
    static_cast<int>(std::ceil(output_max_y / static_resolution)) - 1);

  for (int static_my = first_y; static_my <= last_y; ++static_my) {
    for (int static_mx = first_x; static_mx <= last_x; ++static_mx) {
      const int static_value = static_map.data[
        static_cast<std::size_t>(static_my) * static_map.info.width +
        static_cast<std::size_t>(static_mx)];
      if (static_value >= obstacle_threshold) {
        occupied = true;
        return;
      }
      if (static_value >= 0) {
        known_free = true;
        value = std::max(value, static_value);
      }
    }
  }
}

}  // namespace

bool RogMapEsdfSnapshot::valid() const
{
  const std::size_t cell_count =
    static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
  return info.resolution > 0.0 && cell_count > 0 && occupancy.size() == cell_count &&
         signed_distance.size() == cell_count && gradient_x.size() == cell_count &&
         gradient_y.size() == cell_count;
}

bool RogMapEsdfSnapshot::available() const
{
  return ready && !stale && valid();
}

bool RogMapEsdfSnapshot::unknown(std::size_t index) const
{
  return index >= occupancy.size() || occupancy[index] < 0 ||
         index >= signed_distance.size() || !std::isfinite(signed_distance[index]);
}

RogMapEsdfQuery RogMapEsdfSnapshot::query(double world_x, double world_y) const
{
  RogMapEsdfQuery result;
  if (!valid()) {
    return result;
  }
  tf2::Quaternion origin_rotation;
  tf2::fromMsg(info.origin.orientation, origin_rotation);
  const double yaw = tf2::getYaw(origin_rotation);
  const double dx = world_x - info.origin.position.x;
  const double dy = world_y - info.origin.position.y;
  const int mx = static_cast<int>(std::floor(
      (std::cos(yaw) * dx + std::sin(yaw) * dy) / info.resolution));
  const int my = static_cast<int>(std::floor(
      (-std::sin(yaw) * dx + std::cos(yaw) * dy) / info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
    my >= static_cast<int>(info.height))
  {
    return result;
  }
  const std::size_t index =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
    static_cast<std::size_t>(mx);
  if (unknown(index) || !std::isfinite(gradient_x[index]) ||
    !std::isfinite(gradient_y[index]))
  {
    result.status = RogMapEsdfQueryStatus::kUnknown;
    return result;
  }
  result.status = RogMapEsdfQueryStatus::kKnown;
  result.signed_distance = signed_distance[index];
  result.gradient_x = gradient_x[index];
  result.gradient_y = gradient_y[index];
  return result;
}

RogMapEsdfSnapshot RogMapEsdfSnapshot::fromResponse(
  const ats_rog_map_interfaces::srv::GetRogMapProjection::Response & response)
{
  RogMapEsdfSnapshot snapshot;
  snapshot.header = response.occupancy_grid.header;
  snapshot.info = response.occupancy_grid.info;
  snapshot.generation = response.generation;
  snapshot.ready = response.ready;
  snapshot.stale = response.stale;
  snapshot.occupancy = response.occupancy_grid.data;
  snapshot.signed_distance = response.signed_distance;
  snapshot.gradient_x = response.gradient_x;
  snapshot.gradient_y = response.gradient_y;
  return snapshot;
}

bool GroundProjectionFusion::fuse(
  const nav_msgs::msg::OccupancyGrid & rog_projection,
  const nav_msgs::msg::OccupancyGrid & traversability_grid,
  const nav_msgs::msg::OccupancyGrid & slope_grid,
  const nav_msgs::msg::OccupancyGrid & static_map,
  const geometry_msgs::msg::TransformStamped & static_from_projection,
  const GroundProjectionFusionParams & params,
  GroundProjectionFusionResult & result)
{
  result.planning_grid = nav_msgs::msg::OccupancyGrid();
  result.known_free_cells = 0;
  result.occupied_cells = 0;
  result.unknown_cells = 0;
  if (!validGrid(rog_projection) || !validGrid(traversability_grid) ||
    !validGrid(slope_grid) || !validGrid(static_map) ||
    rog_projection.header.frame_id != traversability_grid.header.frame_id ||
    rog_projection.header.frame_id != slope_grid.header.frame_id)
  {
    return false;
  }

  result.planning_grid = static_map;
  result.planning_grid.header = rog_projection.header;
  result.planning_grid.header.frame_id = static_map.header.frame_id;
  result.planning_grid.info.resolution = std::max(params.planning_resolution, 0.01);
  const double static_extent_x =
    static_cast<double>(static_map.info.width) * static_map.info.resolution;
  const double static_extent_y =
    static_cast<double>(static_map.info.height) * static_map.info.resolution;
  result.planning_grid.info.width = static_cast<std::uint32_t>(
    std::ceil(static_extent_x / result.planning_grid.info.resolution));
  result.planning_grid.info.height = static_cast<std::uint32_t>(
    std::ceil(static_extent_y / result.planning_grid.info.resolution));
  result.planning_grid.data.assign(
    static_cast<std::size_t>(result.planning_grid.info.width) *
    static_cast<std::size_t>(result.planning_grid.info.height), -1);
  const int static_threshold = std::clamp(params.static_obstacle_value_threshold, 0, 100);
  const int terrain_threshold = std::clamp(params.terrain_obstacle_value_threshold, 0, 100);
  const double slope_scale = std::max(params.slope_grid_max_degrees, 1e-3);
  tf2::Transform static_from_projection_transform;
  tf2::fromMsg(static_from_projection.transform, static_from_projection_transform);
  const tf2::Transform projection_from_static_transform =
    static_from_projection_transform.inverse();

  for (unsigned int my = 0; my < result.planning_grid.info.height; ++my) {
    for (unsigned int mx = 0; mx < result.planning_grid.info.width; ++mx) {
      const std::size_t index =
        static_cast<std::size_t>(my) *
        static_cast<std::size_t>(result.planning_grid.info.width) + mx;
      double static_x = 0.0;
      double static_y = 0.0;
      projectionCellCenter(result.planning_grid, mx, my, static_x, static_y);

      int value = 0;
      bool occupied = false;
      bool known_free = false;
      aggregateStaticFootprint(
        static_map, result.planning_grid, mx, my, static_threshold,
        occupied, known_free, value);

      const tf2::Vector3 projection_point =
        projection_from_static_transform * tf2::Vector3(static_x, static_y, 0.0);
      int8_t rog_value = -1;
      if (sampleGrid(
          rog_projection, projection_point.x(), projection_point.y(), rog_value))
      {
        if (rog_value >= terrain_threshold) {
          occupied = true;
        } else if (rog_value >= 0) {
          known_free = true;
          value = std::max(value, static_cast<int>(rog_value));
        }
      }

      int8_t terrain_value = -1;
      if (sampleGrid(
          traversability_grid, projection_point.x(), projection_point.y(), terrain_value))
      {
        if (terrain_value >= terrain_threshold) {
          occupied = true;
        } else if (terrain_value >= 0) {
          known_free = true;
          value = std::max(value, static_cast<int>(terrain_value));
        }
      }

      int8_t slope_value = -1;
      if (sampleGrid(slope_grid, projection_point.x(), projection_point.y(), slope_value))
      {
        if (static_cast<double>(slope_value) * slope_scale / 100.0 >=
          params.slope_obstacle_degrees)
        {
          occupied = true;
        } else if (slope_value >= 0) {
          known_free = true;
        }
      }

      if (occupied) {
        value = 100;
      } else if (params.unknown_is_obstacle && !known_free) {
        value = -1;
      } else if (value < 0) {
        value = 0;
      }
      result.planning_grid.data[index] = static_cast<int8_t>(value);
      if (value < 0) {
        ++result.unknown_cells;
      } else if (value >= terrain_threshold) {
        ++result.occupied_cells;
      } else {
        ++result.known_free_cells;
      }
    }
  }
  return true;
}

std::size_t GroundProjectionFusion::clearUnknownCircle(
  GroundProjectionFusionResult & result, double world_x, double world_y, double radius)
{
  if (!validGrid(result.planning_grid) || radius <= 0.0) {
    return 0;
  }
  std::size_t cleared = 0;
  const double radius_squared = radius * radius;
  for (unsigned int my = 0; my < result.planning_grid.info.height; ++my) {
    for (unsigned int mx = 0; mx < result.planning_grid.info.width; ++mx) {
      const std::size_t index =
        static_cast<std::size_t>(my) * result.planning_grid.info.width + mx;
      if (result.planning_grid.data[index] >= 0) {
        continue;
      }
      double cell_x = 0.0;
      double cell_y = 0.0;
      projectionCellCenter(result.planning_grid, mx, my, cell_x, cell_y);
      const double dx = cell_x - world_x;
      const double dy = cell_y - world_y;
      if (dx * dx + dy * dy > radius_squared) {
        continue;
      }
      result.planning_grid.data[index] = 0;
      ++cleared;
    }
  }
  result.unknown_cells -= std::min(result.unknown_cells, cleared);
  result.known_free_cells += cleared;
  return cleared;
}

}  // namespace ats_rog_map_adapter
