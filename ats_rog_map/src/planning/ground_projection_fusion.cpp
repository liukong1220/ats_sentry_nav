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

struct FastGridSampler
{
  const std::int8_t * data{nullptr};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  double inv_res{0.0};
  bool valid{false};

  static FastGridSampler from(const nav_msgs::msg::OccupancyGrid * grid)
  {
    FastGridSampler sampler;
    if (grid == nullptr || !validGrid(*grid)) {
      return sampler;
    }
    tf2::Quaternion origin_rotation;
    tf2::fromMsg(grid->info.origin.orientation, origin_rotation);
    const double yaw = tf2::getYaw(origin_rotation);
    sampler.data = grid->data.data();
    sampler.width = static_cast<int>(grid->info.width);
    sampler.height = static_cast<int>(grid->info.height);
    sampler.origin_x = grid->info.origin.position.x;
    sampler.origin_y = grid->info.origin.position.y;
    sampler.cos_yaw = std::cos(yaw);
    sampler.sin_yaw = std::sin(yaw);
    sampler.inv_res = 1.0 / grid->info.resolution;
    sampler.valid = true;
    return sampler;
  }

  inline bool sample(double world_x, double world_y, int8_t & value) const
  {
    if (!valid) {
      return false;
    }
    const double dx = world_x - origin_x;
    const double dy = world_y - origin_y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    const int mx = static_cast<int>(std::floor(local_x * inv_res));
    const int my = static_cast<int>(std::floor(local_y * inv_res));
    if (mx < 0 || my < 0 || mx >= width || my >= height) {
      return false;
    }
    value = data[static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(mx)];
    return true;
  }
};

struct FastAffine2D
{
  double r00{1.0}, r01{0.0}, tx{0.0};
  double r10{0.0}, r11{1.0}, ty{0.0};

  static FastAffine2D from(const tf2::Transform & tf)
  {
    const auto & b = tf.getBasis();
    const auto & o = tf.getOrigin();
    return FastAffine2D{b[0][0], b[0][1], o.x(), b[1][0], b[1][1], o.y()};
  }

  inline void apply(double x, double y, double & ox, double & oy) const
  {
    ox = r00 * x + r01 * y + tx;
    oy = r10 * x + r11 * y + ty;
  }
};

void aggregateStaticFootprint(
  const nav_msgs::msg::OccupancyGrid & static_map,
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  unsigned int output_mx, unsigned int output_my, int obstacle_threshold,
  bool & occupied, bool & known_free, int & value)
{
  // Fast path when static_map and planning_grid share the exact same resolution (e.g. 0.05 m).
  if (std::abs(static_map.info.resolution - planning_grid.info.resolution) <= 1e-9) {
    if (output_mx < static_map.info.width && output_my < static_map.info.height) {
      const int static_value = static_map.data[
        static_cast<std::size_t>(output_my) * static_map.info.width +
        static_cast<std::size_t>(output_mx)];
      if (static_value >= obstacle_threshold) {
        occupied = true;
      } else if (static_value >= 0) {
        known_free = true;
        value = std::max(value, static_value);
      }
    }
    return;
  }
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
  // MapMetaData::resolution is float32 (0.05f = 0.050000000745...), which accumulates
  // ~2e-7..1e-6 m edge drift over 20-30 m. Use a 10-micrometer overlap threshold so
  // aligned 5 cm grids copy 1:1 instead of dilating every wall into a 2x2 block, while
  // still capturing any genuine >= 0.01 mm geometric overlap.
  const double overlap_eps = std::max(1e-5, 1e-4 * std::min(r, sr));
  const int x0 = std::max(0, static_cast<int>(std::floor((xmin + overlap_eps) / sr)));
  const int y0 = std::max(0, static_cast<int>(std::floor((ymin + overlap_eps) / sr)));
  const int x1 = std::min(static_cast<int>(source.info.width) - 1,
      static_cast<int>(std::ceil((xmax - overlap_eps) / sr)) - 1);
  const int y1 = std::min(static_cast<int>(source.info.height) - 1,
      static_cast<int>(std::ceil((ymax - overlap_eps) / sr)) - 1);
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
        overlaps = std::min(high, center + radius) - std::max(low, center - radius) >
          overlap_eps * axis.length();
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
  GroundProjectionFusionResult & result,
  const geometry_msgs::msg::TransformStamped * override_from_projection,
  const nav_msgs::msg::OccupancyGrid * tunnel_free_grid)
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
  const bool override_enabled =
    (!(params.mapping_mode || !params.has_static_map) ||
    params.diagnostic_allow_terrain_free_override) &&
    override_from_projection != nullptr && !params.terrain_free_override_frame.empty() &&
    override_from_projection->header.frame_id == params.terrain_free_override_frame &&
    override_from_projection->child_frame_id == rog_projection.header.frame_id &&
    override_from_projection->header.stamp == rog_projection.header.stamp;
  tf2::Transform override_transform;
  if (override_enabled) {
    tf2::fromMsg(override_from_projection->transform, override_transform);
  }

  const FastGridSampler rog_sampler = FastGridSampler::from(&rog_projection);
  const FastGridSampler terrain_sampler = FastGridSampler::from(&traversability_grid);
  const FastGridSampler slope_sampler = FastGridSampler::from(&slope_grid);
  const FastGridSampler static_sampler = FastGridSampler::from(&static_map);
  const FastGridSampler tunnel_free_sampler = FastGridSampler::from(tunnel_free_grid);
  const FastAffine2D proj_from_static_2d = FastAffine2D::from(projection_from_static_transform);
  const FastAffine2D static_from_proj_2d = FastAffine2D::from(static_from_projection_transform);
  const FastAffine2D override_2d =
    override_enabled ? FastAffine2D::from(override_transform) : FastAffine2D{};

  tf2::Quaternion out_origin_rot;
  tf2::fromMsg(result.planning_grid.info.origin.orientation, out_origin_rot);
  const double out_yaw = tf2::getYaw(out_origin_rot);
  const double out_cos = std::cos(out_yaw);
  const double out_sin = std::sin(out_yaw);
  const double out_ox = result.planning_grid.info.origin.position.x;
  const double out_oy = result.planning_grid.info.origin.position.y;
  const double out_res = result.planning_grid.info.resolution;

  const double map_w_m = static_map.info.width * static_map.info.resolution;
  const double map_h_m = static_map.info.height * static_map.info.resolution;
  const double border = std::max(0.0, params.static_map_geofence_border_m);
  const bool check_override = override_enabled && !params.terrain_free_override_rects.empty();
  const bool check_tunnel_free = params.tunnel_free_suppresses_terrain && tunnel_free_sampler.valid;

  for (unsigned int my = 0; my < result.planning_grid.info.height; ++my) {
    const double local_y = (static_cast<double>(my) + 0.5) * out_res;
    const double row_base_x = out_ox - out_sin * local_y;
    const double row_base_y = out_oy + out_cos * local_y;
    const std::size_t row_offset =
      static_cast<std::size_t>(my) * static_cast<std::size_t>(result.planning_grid.info.width);
    for (unsigned int mx = 0; mx < result.planning_grid.info.width; ++mx) {
      const std::size_t index = row_offset + mx;
      const double local_x = (static_cast<double>(mx) + 0.5) * out_res;
      const double static_x = row_base_x + out_cos * local_x;
      const double static_y = row_base_y + out_sin * local_x;

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
      } else if (params.use_static_map_geofence_only && params.has_static_map &&
        static_sampler.valid)
      {
        double map_px = static_x;
        double map_py = static_y;
        if (params.output_in_projection_frame) {
          static_from_proj_2d.apply(static_x, static_y, map_px, map_py);
        }
        const double dx = map_px - static_sampler.origin_x;
        const double dy = map_py - static_sampler.origin_y;
        const double lx = static_sampler.cos_yaw * dx + static_sampler.sin_yaw * dy;
        const double ly = -static_sampler.sin_yaw * dx + static_sampler.cos_yaw * dy;
        int8_t static_sample = -1;
        const bool in_bounds = static_sampler.sample(map_px, map_py, static_sample);
        if (!in_bounds || static_sample < 0 ||
          lx < border || ly < border || lx > map_w_m - border || ly > map_h_m - border)
        {
          if (!in_bounds || static_sample < 0 || static_sample >= static_threshold) {
            occupied = true;
          }
        }
      }

      double proj_x = static_x;
      double proj_y = static_y;
      if (!params.output_in_projection_frame) {
        proj_from_static_2d.apply(static_x, static_y, proj_x, proj_y);
      }
      int8_t rog_value = -1;
      if (rog_sampler.sample(proj_x, proj_y, rog_value))
      {
        if (rog_value >= terrain_threshold) {
          occupied = true;
        } else if (rog_value >= 0) {
          known_free = true;
          value = std::max(value, static_cast<int>(rog_value));
        }
      }

      bool corridor_static_free = false;
      if (!occupied && check_tunnel_free) {
        int8_t tunnel_free_val = 0;
        if (tunnel_free_sampler.sample(proj_x, proj_y, tunnel_free_val) &&
          tunnel_free_val > 0)
        {
          known_free = true;
          corridor_static_free = true;
        } else if (rog_value >= 0) {
          // Suppress terrain_analysis_ext's <=10 cm overhang bleed at tunnel/overpass mouths
          // strictly on cells that odom_layer has already verified as known-free (rog_value == 0).
          const int halo_cells = std::max(
            1, std::min(2, static_cast<int>(std::lround(0.10 / out_res))));
          for (int hy = -halo_cells; hy <= halo_cells && !corridor_static_free; ++hy) {
            for (int hx = -halo_cells; hx <= halo_cells; ++hx) {
              if (hx == 0 && hy == 0) {continue;}
              int8_t nb_tf = 0;
              if (tunnel_free_sampler.sample(
                  proj_x + hx * out_res, proj_y + hy * out_res, nb_tf) && nb_tf > 0)
              {
                corridor_static_free = true;
                break;
              }
            }
          }
        }
      }
      if (!occupied && known_free && check_override) {
        double ov_x = 0.0;
        double ov_y = 0.0;
        override_2d.apply(proj_x, proj_y, ov_x, ov_y);
        for (std::size_t r = 0; r + 3 < params.terrain_free_override_rects.size(); r += 4) {
          const auto & q = params.terrain_free_override_rects;
          if (ov_x >= q[r] && ov_x <= q[r + 2] &&
            ov_y >= q[r + 1] && ov_y <= q[r + 3])
          {
            corridor_static_free = true;
            break;
          }
        }
      }

      int8_t terrain_value = -1;
      if (terrain_sampler.sample(proj_x, proj_y, terrain_value))
      {
        if (terrain_value >= terrain_threshold) {
          if (!corridor_static_free) {occupied = true;}
        } else if (terrain_value >= 0) {
          known_free = true;
          value = std::max(value, static_cast<int>(terrain_value));
        }
      }

      int8_t slope_value = -1;
      if (slope_sampler.sample(proj_x, proj_y, slope_value))
      {
        if (static_cast<double>(slope_value) * slope_scale / 100.0 >=
          params.slope_obstacle_degrees)
        {
          if (!corridor_static_free) {occupied = true;}
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
  double length, double width, double margin, int obstacle_threshold)
{
  if (!validGrid(result.planning_grid) || !std::isfinite(world_x) ||
    !std::isfinite(world_y) || !std::isfinite(yaw) || !std::isfinite(length) ||
    !std::isfinite(width) || !std::isfinite(margin) || length <= 0 || width <= 0 || margin < 0)
  {return 0;}
  std::size_t cleared = 0;
  const double c = std::cos(yaw), s = std::sin(yaw);
  auto & grid = result.planning_grid;
  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  tf2::Quaternion origin_rotation;
  tf2::fromMsg(grid.info.origin.orientation, origin_rotation);
  const double origin_yaw = tf2::getYaw(origin_rotation);
  const double cos_o = std::cos(origin_yaw);
  const double sin_o = std::sin(origin_yaw);
  const int occ_thr = std::clamp(obstacle_threshold, 1, 100);
  const auto rayOccludedByObstacle = [&](double target_x, double target_y) -> bool {
      const double rx = target_x - world_x;
      const double ry = target_y - world_y;
      const double dist = std::hypot(rx, ry);
      if (dist <= 0.5 * res) {
        return false;
      }
      const int steps = std::max(2, static_cast<int>(std::ceil(dist / (0.5 * res))));
      for (int step = 1; step < steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        const double sx = world_x + t * rx;
        const double sy = world_y + t * ry;
        const double rel_x = sx - origin_x;
        const double rel_y = sy - origin_y;
        const double gx = (cos_o * rel_x + sin_o * rel_y) / res;
        const double gy = (-sin_o * rel_x + cos_o * rel_y) / res;
        const int ix = static_cast<int>(std::floor(gx));
        const int iy = static_cast<int>(std::floor(gy));
        if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) ||
          iy >= static_cast<int>(grid.info.height))
        {
          return true;
        }
        const std::size_t sidx =
          static_cast<std::size_t>(iy) * grid.info.width + static_cast<std::size_t>(ix);
        if (grid.data[sidx] >= occ_thr) {
          return true;
        }
      }
      return false;
    };
  for (unsigned int y = 0; y < grid.info.height; ++y) {
    for (unsigned int x = 0; x < grid.info.width; ++x) {
      const auto index = static_cast<std::size_t>(y) * grid.info.width + x;
      if (grid.data[index] >= 0) {continue;}
      double wx, wy;
      projectionCellCenter(grid, x, y, wx, wy);
      const double dx = wx - world_x, dy = wy - world_y;
      if (std::abs(c * dx + s * dy) > 0.5 * length + margin + 1e-8 ||
        std::abs(-s * dx + c * dy) > 0.5 * width + margin + 1e-8) {continue;}
      if (rayOccludedByObstacle(wx, wy)) {continue;}
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
