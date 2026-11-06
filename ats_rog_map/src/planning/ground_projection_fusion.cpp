// Copyright 2026

#include "ats_rog_map/planning/ground_projection_fusion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <string>
#include <utility>

#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_rog_map::planning
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

void aggregateTransformedFootprint(
  const nav_msgs::msg::OccupancyGrid & source,
  const nav_msgs::msg::OccupancyGrid & output, unsigned int mx, unsigned int my,
  const tf2::Transform & source_from_output, int threshold,
  bool & occupied, bool & known_free, int & value)
{
  tf2::Transform source_origin, output_origin;
  tf2::fromMsg(source.info.origin, source_origin);
  tf2::fromMsg(output.info.origin, output_origin);
  const auto local_from_output = source_origin.inverse() * source_from_output * output_origin;
  std::array<tf2::Vector3, 4> polygon;
  double xmin = std::numeric_limits<double>::infinity(), ymin = xmin;
  double xmax = -xmin, ymax = -xmin;
  const double r = output.info.resolution;
  for (int i = 0; i < 4; ++i) {
    polygon[i] = local_from_output * tf2::Vector3(
      (mx + (i == 1 || i == 2 ? 1.0 : 0.0)) * r,
      (my + (i >= 2 ? 1.0 : 0.0)) * r, 0.0);
    xmin = std::min(xmin, polygon[i].x()); xmax = std::max(xmax, polygon[i].x());
    ymin = std::min(ymin, polygon[i].y()); ymax = std::max(ymax, polygon[i].y());
  }
  const double sr = source.info.resolution;
  const int x0 = std::max(0, static_cast<int>(std::floor(xmin / sr)));
  const int y0 = std::max(0, static_cast<int>(std::floor(ymin / sr)));
  const int x1 = std::min(static_cast<int>(source.info.width) - 1,
      static_cast<int>(std::ceil(xmax / sr)) - 1);
  const int y1 = std::min(static_cast<int>(source.info.height) - 1,
      static_cast<int>(std::ceil(ymax / sr)) - 1);
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      bool overlaps = true;
      for (int a = 0; a < 6 && overlaps; ++a) {
        tf2::Vector3 axis(a == 0 ? 1 : 0, a == 1 ? 1 : 0, 0);
        if (a >= 2) {
          const auto edge = polygon[(a - 1) % 4] - polygon[a - 2];
          axis = tf2::Vector3(-edge.y(), edge.x(), 0);
        }
        double low = std::numeric_limits<double>::infinity(), high = -low;
        for (const auto & p : polygon) {
          low = std::min(low, p.dot(axis)); high = std::max(high, p.dot(axis));
        }
        const double center = ((x + 0.5) * axis.x() + (y + 0.5) * axis.y()) * sr;
        const double radius = 0.5 * sr * (std::abs(axis.x()) + std::abs(axis.y()));
        overlaps = std::min(high, center + radius) - std::max(low, center - radius) > 1e-10;
      }
      if (!overlaps) {continue;}
      const int sample = source.data[static_cast<std::size_t>(y) * source.info.width + x];
      if (sample >= threshold) {occupied = true; return;}
      if (sample >= 0) {known_free = true; value = std::max(value, sample);}
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
  const ats_rog_map::GroundProjection & response)
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

SourceUnknownEvidence evaluateSourceUnknownEvidence(
  const RogMapEsdfSnapshot & snapshot, const std::uint64_t baseline_generation)
{
  SourceUnknownEvidence evidence;
  evidence.generation = snapshot.generation;
  const std::size_t cell_count = static_cast<std::size_t>(snapshot.info.width) *
    static_cast<std::size_t>(snapshot.info.height);
  evidence.cell_count = cell_count;

  if (!snapshot.ready) {
    evidence.reason = "ROGMap numeric projection is not ready";
    return evidence;
  }
  if (snapshot.stale) {
    evidence.reason = "ROGMap numeric projection is stale";
    return evidence;
  }
  if (snapshot.header.frame_id.empty()) {
    evidence.reason = "ROGMap numeric projection has an empty frame_id";
    return evidence;
  }
  if (snapshot.header.stamp.sec <= 0 && snapshot.header.stamp.nanosec == 0U) {
    evidence.reason = "ROGMap numeric projection has a non-positive stamp";
    return evidence;
  }
  if (!std::isfinite(snapshot.info.resolution) || snapshot.info.resolution <= 0.0F) {
    evidence.reason = "ROGMap numeric projection resolution is not finite and positive";
    return evidence;
  }
  if (snapshot.info.width == 0 || snapshot.info.height == 0 || cell_count == 0) {
    evidence.reason = "ROGMap numeric projection grid is empty";
    return evidence;
  }
  if (snapshot.occupancy.size() != cell_count) {
    evidence.reason = "ROGMap numeric occupancy length does not equal width * height";
    return evidence;
  }
  if (snapshot.signed_distance.size() != cell_count ||
    snapshot.gradient_x.size() != cell_count || snapshot.gradient_y.size() != cell_count)
  {
    evidence.reason = "ROGMap numeric signed-distance or gradient length does not equal cells";
    return evidence;
  }
  evidence.structurally_valid = true;

  for (std::size_t index = 0; index < cell_count; ++index) {
    const int value = static_cast<int>(snapshot.occupancy[index]);
    if (value == -1) {
      ++evidence.unknown_cells;
    } else if (value == 0) {
      ++evidence.free_cells;
    } else if (value > 0 && value <= 100) {
      ++evidence.occupied_cells;
    } else {
      ++evidence.out_of_range_cells;
    }
    if (std::isfinite(snapshot.signed_distance[index]) ||
      std::isfinite(snapshot.gradient_x[index]) || std::isfinite(snapshot.gradient_y[index]))
    {
      ++evidence.finite_numeric_cells;
    }
  }

  if (evidence.unknown_cells != cell_count) {
    evidence.reason = "ROGMap numeric occupancy mixes unknown with free=" +
      std::to_string(evidence.free_cells) + " occupied=" +
      std::to_string(evidence.occupied_cells) + " out_of_range=" +
      std::to_string(evidence.out_of_range_cells) + " unknown=" +
      std::to_string(evidence.unknown_cells) + " of cells=" + std::to_string(cell_count);
    return evidence;
  }
  if (evidence.finite_numeric_cells != 0) {
    evidence.reason = "ROGMap numeric signed-distance or gradient is finite in " +
      std::to_string(evidence.finite_numeric_cells) + " unknown cells";
    return evidence;
  }
  if (snapshot.generation <= baseline_generation) {
    evidence.reason = "ROGMap source generation " + std::to_string(snapshot.generation) +
      " did not advance past the pre-fault generation " + std::to_string(baseline_generation);
    return evidence;
  }

  evidence.all_unknown = true;
  evidence.reason = "ROGMap numeric projection is strictly all-unknown across " +
    std::to_string(cell_count) + " cells at generation " + std::to_string(snapshot.generation);
  return evidence;
}

TerrainSpikeFilter::TerrainSpikeFilter(int confirm_frames, int obstacle_threshold)
: confirm_frames_(std::max(confirm_frames, 1)),
  obstacle_threshold_(std::clamp(obstacle_threshold, 0, 100))
{
}

void TerrainSpikeFilter::reset(const nav_msgs::msg::OccupancyGrid & frame)
{
  if (initialized_) {
    ++resets_;
  }
  const std::size_t cells = frame.data.size();
  counts_.assign(cells, 0);
  last_soft_.assign(cells, -1);
}

nav_msgs::msg::OccupancyGrid TerrainSpikeFilter::process(
  const nav_msgs::msg::OccupancyGrid & frame)
{
  suppressed_cells_ = 0;
  if (confirm_frames_ <= 1 || !validGrid(frame)) {
    return frame;
  }
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(frame.header.stamp.sec) * 1000000000LL + frame.header.stamp.nanosec;
  const bool same_geometry = initialized_ && frame.header.frame_id == frame_id_ &&
    frame.info.resolution == resolution_ && frame.info.width == width_ &&
    frame.info.height == height_;
  int shift_x = 0;
  int shift_y = 0;
  bool shiftable = same_geometry;
  if (same_geometry) {
    const double fx = (frame.info.origin.position.x - origin_x_) / resolution_;
    const double fy = (frame.info.origin.position.y - origin_y_) / resolution_;
    shift_x = static_cast<int>(std::lround(fx));
    shift_y = static_cast<int>(std::lround(fy));
    shiftable = std::abs(fx - shift_x) < 1e-3 && std::abs(fy - shift_y) < 1e-3;
  }
  const bool rewound = initialized_ && stamp_ns < last_stamp_ns_;
  if (!same_geometry || !shiftable || rewound) {
    reset(frame);
    shift_x = 0;
    shift_y = 0;
  } else if (shift_x != 0 || shift_y != 0) {
    // Cell (ix, iy) of the new frame is cell (ix + shift_x, iy + shift_y) of the old one.
    std::vector<std::uint16_t> counts(counts_.size(), 0);
    std::vector<int8_t> soft(last_soft_.size(), -1);
    const int w = static_cast<int>(width_);
    const int h = static_cast<int>(height_);
    for (int iy = 0; iy < h; ++iy) {
      for (int ix = 0; ix < w; ++ix) {
        const int ox = ix + shift_x;
        const int oy = iy + shift_y;
        if (ox < 0 || oy < 0 || ox >= w || oy >= h) {
          continue;
        }
        counts[static_cast<std::size_t>(iy) * w + ix] = counts_[static_cast<std::size_t>(oy) * w + ox];
        soft[static_cast<std::size_t>(iy) * w + ix] = last_soft_[static_cast<std::size_t>(oy) * w + ox];
      }
    }
    counts_ = std::move(counts);
    last_soft_ = std::move(soft);
  }
  const bool new_stamp = !initialized_ || stamp_ns != last_stamp_ns_ || rewound;
  initialized_ = true;
  frame_id_ = frame.header.frame_id;
  resolution_ = frame.info.resolution;
  origin_x_ = frame.info.origin.position.x;
  origin_y_ = frame.info.origin.position.y;
  width_ = frame.info.width;
  height_ = frame.info.height;
  last_stamp_ns_ = stamp_ns;

  auto output = frame;
  const auto confirm = static_cast<std::uint16_t>(std::min(confirm_frames_, 65535));
  for (std::size_t i = 0; i < frame.data.size(); ++i) {
    const int8_t value = frame.data[i];
    const bool hard = value >= obstacle_threshold_;
    if (new_stamp) {
      if (hard) {
        if (counts_[i] < confirm) {
          ++counts_[i];
        }
      } else {
        counts_[i] = 0;
      }
    }
    if (!hard) {
      last_soft_[i] = value;
    } else if (counts_[i] < confirm) {
      output.data[i] = last_soft_[i];
      ++suppressed_cells_;
    }
  }
  return output;
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

  const auto & geometry = params.output_in_projection_frame ? rog_projection : static_map;
  result.planning_grid = geometry;
  result.planning_grid.header = rog_projection.header;
  result.planning_grid.header.frame_id = geometry.header.frame_id;
  result.planning_grid.info.resolution = std::max(params.planning_resolution, 0.01);
  const double static_extent_x =
    static_cast<double>(geometry.info.width) * geometry.info.resolution;
  const double static_extent_y =
    static_cast<double>(geometry.info.height) * geometry.info.resolution;
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
      if (params.use_static_map_obstacles) {
        if (params.output_in_projection_frame) {
          aggregateTransformedFootprint(static_map, result.planning_grid, mx, my,
            static_from_projection_transform, static_threshold, occupied, known_free, value);
        } else {
          aggregateStaticFootprint(
            static_map, result.planning_grid, mx, my, static_threshold,
            occupied, known_free, value);
        }
      }

      const tf2::Vector3 projection_point =
        params.output_in_projection_frame ? tf2::Vector3(static_x, static_y, 0.0) :
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

std::size_t GroundProjectionFusion::clearUnknownFootprint(
  GroundProjectionFusionResult & result, double world_x, double world_y, double yaw,
  double length, double width, double margin)
{
  if (!validGrid(result.planning_grid) || !std::isfinite(world_x) ||
    !std::isfinite(world_y) || !std::isfinite(yaw) || !std::isfinite(length) ||
    !std::isfinite(width) || !std::isfinite(margin) || length <= 0 || width <= 0 || margin < 0)
  {return 0;}
  std::size_t cleared = 0;
  const double c = std::cos(yaw), s = std::sin(yaw);
  auto & grid = result.planning_grid;
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x < grid.info.width; ++x) {
      const auto index = static_cast<std::size_t>(y) * grid.info.width + x;
      if (grid.data[index] >= 0) {continue;}
      double wx, wy;
      projectionCellCenter(grid, x, y, wx, wy);
      const double dx = wx - world_x, dy = wy - world_y;
      if (std::abs(c * dx + s * dy) > 0.5 * length + margin + 1e-8 ||
        std::abs(-s * dx + c * dy) > 0.5 * width + margin + 1e-8) {continue;}
      grid.data[index] = 0; ++cleared;
    }
  }
  result.unknown_cells -= std::min(cleared, result.unknown_cells);
  result.known_free_cells += cleared;
  return cleared;
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

}  // namespace ats_rog_map::planning
