// Copyright 2026

#include "ats_rc_esdf/esdf/static_map_fusion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ats_rc_esdf
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

using Polygon = std::vector<std::array<double, 2>>;

Polygon clipAgainstAxis(
  const Polygon & input, std::size_t axis, double boundary, bool keep_greater)
{
  Polygon output;
  if (input.empty()) {
    return output;
  }

  const auto is_inside = [axis, boundary, keep_greater](const std::array<double, 2> & point) {
      return keep_greater ? point[axis] >= boundary : point[axis] <= boundary;
    };
  for (std::size_t index = 0; index < input.size(); ++index) {
    const auto & current = input[index];
    const auto & previous = input[(index + input.size() - 1U) % input.size()];
    const bool current_inside = is_inside(current);
    const bool previous_inside = is_inside(previous);
    if (current_inside != previous_inside) {
      const double delta = current[axis] - previous[axis];
      if (std::abs(delta) > std::numeric_limits<double>::epsilon()) {
        const double ratio = (boundary - previous[axis]) / delta;
        output.push_back({
          previous[0] + ratio * (current[0] - previous[0]),
          previous[1] + ratio * (current[1] - previous[1])});
      }
    }
    if (current_inside) {
      output.push_back(current);
    }
  }
  return output;
}

double polygonArea(const Polygon & polygon)
{
  if (polygon.size() < 3U) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & current = polygon[index];
    const auto & next = polygon[(index + 1U) % polygon.size()];
    twice_area += current[0] * next[1] - current[1] * next[0];
  }
  return 0.5 * std::abs(twice_area);
}

bool hasAreaOverlapWithGridCell(const Polygon & polygon, int x, int y)
{
  Polygon clipped = clipAgainstAxis(polygon, 0U, static_cast<double>(x), true);
  clipped = clipAgainstAxis(clipped, 0U, static_cast<double>(x + 1), false);
  clipped = clipAgainstAxis(clipped, 1U, static_cast<double>(y), true);
  clipped = clipAgainstAxis(clipped, 1U, static_cast<double>(y + 1), false);
  return polygonArea(clipped) > 1e-12;
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

  for (unsigned int y = 0; y < local_grid.info.height; ++y) {
    for (unsigned int x = 0; x < local_grid.info.width; ++x) {
      const std::size_t local_index =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(local_grid.info.width) + x;
      bool static_occupied = false;
      // Project the complete output-cell footprint into static-grid coordinates.
      // Center sampling loses walls whenever resolutions are non-integral or either
      // grid has a yaw. Occupied source cells with any positive-area overlap win.
      Polygon cell_polygon;
      cell_polygon.reserve(4);
      for (const auto & corner : std::array<std::array<double, 2>, 4>{{
          {{0.0, 0.0}}, {{1.0, 0.0}}, {{1.0, 1.0}}, {{0.0, 1.0}}}})
      {
        const double local_x = (static_cast<double>(x) + corner[0]) * local_grid.info.resolution;
        const double local_y = (static_cast<double>(y) + corner[1]) * local_grid.info.resolution;
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
        cell_polygon.push_back({
          (static_origin_cos * origin_dx + static_origin_sin * origin_dy) /
            static_map.info.resolution,
          (-static_origin_sin * origin_dx + static_origin_cos * origin_dy) /
            static_map.info.resolution});
      }

      double min_x = cell_polygon.front()[0];
      double max_x = min_x;
      double min_y = cell_polygon.front()[1];
      double max_y = min_y;
      for (const auto & point : cell_polygon) {
        min_x = std::min(min_x, point[0]);
        max_x = std::max(max_x, point[0]);
        min_y = std::min(min_y, point[1]);
        max_y = std::max(max_y, point[1]);
      }
      const int first_x = std::max(0, static_cast<int>(std::floor(min_x)));
      const int last_x = std::min(
        static_cast<int>(static_map.info.width) - 1,
        static_cast<int>(std::ceil(max_x)) - 1);
      const int first_y = std::max(0, static_cast<int>(std::floor(min_y)));
      const int last_y = std::min(
        static_cast<int>(static_map.info.height) - 1,
        static_cast<int>(std::ceil(max_y)) - 1);
      for (int map_y = first_y; map_y <= last_y && !static_occupied; ++map_y) {
        for (int map_x = first_x; map_x <= last_x; ++map_x) {
          if (!hasAreaOverlapWithGridCell(cell_polygon, map_x, map_y)) {
            continue;
          }
          const std::size_t static_index =
            static_cast<std::size_t>(map_y) * static_cast<std::size_t>(static_map.info.width) +
            static_cast<std::size_t>(map_x);
          static_occupied = static_map.data[static_index] >= static_threshold;
          if (static_occupied) {
            break;
          }
        }
      }
      if (static_occupied) {
        planning_grid.data[local_index] = 100;
        continue;
      }

      const int8_t local_value = local_grid.data[local_index];
      if (local_value >= local_threshold) {
        planning_grid.data[local_index] = 100;
      } else if (local_value >= 0) {
        planning_grid.data[local_index] = local_value;
      } else {
        bool static_free = false;
        for (int map_y = first_y; map_y <= last_y && !static_free; ++map_y) {
          for (int map_x = first_x; map_x <= last_x; ++map_x) {
            if (!hasAreaOverlapWithGridCell(cell_polygon, map_x, map_y)) {
              continue;
            }
            const std::size_t static_index =
              static_cast<std::size_t>(map_y) * static_cast<std::size_t>(static_map.info.width) +
              static_cast<std::size_t>(map_x);
            static_free = static_map.data[static_index] >= 0;
            if (static_free) {
              break;
            }
          }
        }
        // Explicit free from either source resolves the other's unknown. Only
        // an absence of free/occupied evidence from both sources stays unknown.
        planning_grid.data[local_index] = static_free ? 0 : -1;
      }
    }
  }

  return true;
}

}  // namespace ats_rc_esdf
