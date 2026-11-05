// Copyright 2026

#include "ats_rog_map/odom_layer_integration.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <unordered_set>

namespace ats_rog_map
{
namespace
{
using Point = std::array<double, 3>;
using Key = std::array<int, 3>;
using Hash = OdomVoxelKeyHash;
Key key(const Point & p, double r)
{
  return {static_cast<int>(std::floor(p[0] / r)),
    static_cast<int>(std::floor(p[1] / r)), static_cast<int>(std::floor(p[2] / r))};
}

// Two bits per voxel replace hundreds of thousands of individually allocated
// hash nodes each frame. Bounds include both ray ends; the straight segment
// cannot leave their axis-aligned box. Large/outlier boxes use the exact sparse
// representation instead, bounding temporary dense storage to 32 MiB.
class RayFrameMask
{
public:
  RayFrameMask(const std::vector<Point> & points, const Point & sensor, double r)
  {
    minimum_ = key(sensor, r);
    Key maximum = minimum_;
    for (const auto & p : points) {
      const auto k = key(p, r);
      for (int a = 0; a < 3; ++a) {
        minimum_[a] = std::min(minimum_[a], k[a]);
        maximum[a] = std::max(maximum[a], k[a]);
      }
    }
    maximum_ = maximum;
    constexpr std::size_t max_voxels = 128U * 1024U * 1024U;
    std::size_t size = 1;
    for (int a = 0; a < 3; ++a) {
      dimensions_[a] = static_cast<std::size_t>(
        static_cast<std::int64_t>(maximum[a]) - minimum_[a] + 1);
      if (dimensions_[a] > max_voxels / size) {size = 0; break;}
      size *= dimensions_[a];
    }
    if (size > 0) {words_.resize((size + 63U) / 64U);}
    else {
      endpoints_.reserve(points.size());
      traversed_.reserve(points.size() * 64U);
    }
    for (const auto & p : points) {
      const auto k = key(p, r);
      if (words_.empty()) {endpoints_.insert(k);}
      else {
        const auto i = index(k);
        words_[i / 64U].endpoints |= std::uint64_t{1} << (i % 64U);
      }
    }
  }

  bool clearOnce(const Key & k)
  {
    if (words_.empty()) {
      if (endpoints_.count(k) || !traversed_.insert(k).second) {return false;}
    } else {
      const auto i = index(k);
      auto & word = words_[i / 64U];
      const auto bit = std::uint64_t{1} << (i % 64U);
      if ((word.endpoints | word.traversed) & bit) {return false;}
      word.traversed |= bit;
    }
    ++visited_;
    return true;
  }

  void addCandidate(const Key & k)
  {
    for (int a = 0; a < 3; ++a) {
      if (k[a] < minimum_[a] || k[a] > maximum_[a]) {return;}
    }
    if (words_.empty()) {traversed_.insert(k); return;}
    const auto i = index(k);
    words_[i / 64U].traversed |= std::uint64_t{1} << (i % 64U);
  }

  bool candidate(const Key & k) const
  {
    if (words_.empty()) {return traversed_.count(k) && !endpoints_.count(k);}
    const auto i = index(k);
    const auto & word = words_[i / 64U];
    return ((word.traversed & ~word.endpoints) & (std::uint64_t{1} << (i % 64U))) != 0;
  }

  void removeCandidate(const Key & k)
  {
    if (words_.empty()) {traversed_.erase(k); return;}
    const auto i = index(k);
    words_[i / 64U].traversed &= ~(std::uint64_t{1} << (i % 64U));
  }

  std::size_t visited() const {return visited_;}

private:
  std::size_t index(const Key & k) const
  {
    return static_cast<std::size_t>(static_cast<std::int64_t>(k[0]) - minimum_[0]) +
           dimensions_[0] * (
      static_cast<std::size_t>(static_cast<std::int64_t>(k[1]) - minimum_[1]) +
      dimensions_[1] * static_cast<std::size_t>(static_cast<std::int64_t>(k[2]) - minimum_[2]));
  }
  struct Word {std::uint64_t endpoints{0}, traversed{0};};
  Key minimum_{}, maximum_{};
  std::array<std::size_t, 3> dimensions_{};
  std::vector<Word> words_;
  std::unordered_set<Key, Hash> endpoints_, traversed_;
  std::size_t visited_{0};
};

// Amanatides-Woo traversal. A return is the occlusion boundary: never visit
// its endpoint voxel or extend a ray beyond it. Equal boundary times advance
// together, excluding voxels touched only at a zero-area corner.
template<class Visit>
void ray(const Point & start, const Point & end, double r, Visit visit)
{
  auto k = key(start, r);
  const auto target = key(end, r);
  std::array<int, 3> step{};
  Point next{}, delta{};
  for (int a = 0; a < 3; ++a) {
    const double d = end[a] - start[a];
    step[a] = d > 0 ? 1 : (d < 0 ? -1 : 0);
    delta[a] = step[a] == 0 ? std::numeric_limits<double>::infinity() : r / std::abs(d);
    next[a] = step[a] == 0 ? delta[a] :
      ((k[a] + (step[a] > 0 ? 1 : 0)) * r - start[a]) / d;
  }
  for (int iteration = 0; k != target && iteration < 10000; ++iteration) {
    const double t = std::min({next[0], next[1], next[2]});
    if (!std::isfinite(t) || t > 1.0) {break;}
    visit(k, Point{start[0] + (end[0] - start[0]) * std::max(0.0, t - 1e-8),
      start[1] + (end[1] - start[1]) * std::max(0.0, t - 1e-8),
      start[2] + (end[2] - start[2]) * std::max(0.0, t - 1e-8)});
    for (int a = 0; a < 3; ++a) {
      if (next[a] <= t + 1e-10) {k[a] += step[a]; next[a] += delta[a];}
    }
  }
}
}  // namespace

OdomLayerFrameStats integrateOdomLayerFrame(
  OdomLayerStore & layer, GroundColumnGrid & columns, const std::vector<Point> & points,
  const Point & sensor, const Point & robot_ground, double stamp, double frame_dt)
{
  const auto begin = std::chrono::steady_clock::now();
  OdomLayerFrameStats stats;
  const auto & c = layer.config();
  const auto & gc = columns.config();
  const double r = std::min(gc.voxel_resolution, c.cloud_resolution);
  const auto cell_count = static_cast<std::size_t>(c.width) * c.height;
  std::vector<unsigned char> flags(cell_count, 0);
  std::vector<double> ground_cache(cell_count, std::numeric_limits<double>::quiet_NaN());
  std::vector<std::size_t> touched;
  touched.reserve(std::min(cell_count, points.size() * 8U));
  const auto touch = [&](std::size_t i) {
      if (!(flags[i] & 2U)) {flags[i] |= 2U; touched.push_back(i);}
    };
  std::vector<Point> valid;
  valid.reserve(points.size());
  for (const auto & p : points) {
    if (!std::all_of(p.begin(), p.end(), [](double v) {return std::isfinite(v);})) {continue;}
    valid.push_back(p);
  }
  const bool sensor_valid = std::all_of(sensor.begin(), sensor.end(), [](double v) {return std::isfinite(v);});
  // A nonfinite sensor has no rays, so it must not enter voxel indexing.
  RayFrameMask mask(valid, sensor_valid ? sensor : Point{}, r);
  RayFrameMask cloud_mask(valid, sensor_valid ? sensor : Point{}, c.cloud_resolution);
  for (const auto & k : layer.rayClearCandidates(stamp)) {cloud_mask.addCandidate(k);}
  const auto prepared = std::chrono::steady_clock::now();
  if (sensor_valid) {
    for (const auto & p : valid) {
      const auto clear_cloud = [&](const Key & k) {
          if (!cloud_mask.candidate(k)) {return;}
          const Point center{(k[0] + 0.5) * c.cloud_resolution,
            (k[1] + 0.5) * c.cloud_resolution, (k[2] + 0.5) * c.cloud_resolution};
          layer.clearCloudVoxelAlongRay(center, sensor, p, stamp);
          // A grazing miss remains a candidate for every subsequent ray.
          // No new cloud point is inserted until all rays are finished.
          if (!layer.hasRayClearCandidate(center, stamp)) {cloud_mask.removeCandidate(k);}
        };
      const auto clear_column = [&](const Key & k, const Point & actual) {
        const int x = static_cast<int>(std::floor((actual[0] - c.origin_x) / c.resolution));
        const int y = static_cast<int>(std::floor((actual[1] - c.origin_y) / c.resolution));
        if (!layer.validCell(x, y)) {return;}
        if (r == c.cloud_resolution) {clear_cloud(k);}
        const auto i = layer.index(x, y);
        if (!(flags[i] & 2U)) {
          if (!(flags[i] & 1U)) {
            flags[i] |= 1U;
            if (const auto ground = columns.ground(x, y)) {ground_cache[i] = *ground;}
          }
          const double ground = ground_cache[i];
          if (actual[2] >= ground + gc.surface_max &&
            actual[2] <= ground + gc.robot_height + gc.top_margin)
          {
            columns.observeRayCell(x, y);
            touch(i);
          }
        }
        const Point center{(k[0] + 0.5) * r, (k[1] + 0.5) * r, (k[2] + 0.5) * r};
        if (!mask.clearOnce(k)) {return;}
        columns.clearVoxel(center, actual[2]);
        };
      ray(sensor, p, r, clear_column);
      if (r != c.cloud_resolution) {
        // Different voxel resolutions require their own boundary traversal;
        // a fine voxel centre can lie in a different coarse voxel than the ray.
        ray(sensor, p, c.cloud_resolution, [&](const Key & k, const Point &) {clear_cloud(k);});
      }
    }
  }
  const auto rays_done = std::chrono::steady_clock::now();
  columns.observe(valid, robot_ground);
  const auto ground_done = std::chrono::steady_clock::now();
  for (const auto & p : valid) {
    const int x = static_cast<int>(std::floor((p[0] - c.origin_x) / c.resolution));
    const int y = static_cast<int>(std::floor((p[1] - c.origin_y) / c.resolution));
    if (!layer.validCell(x, y)) {
      ++stats.out_of_range;
      if (!stats.out_of_range_sample) {stats.out_of_range_sample = p;}
      continue;
    }
    const auto i = layer.index(x, y);
    touch(i);
    const auto ground = columns.ground(x, y);
    if (ground && p[2] > *ground + gc.surface_max) {flags[i] |= 4U;}
    layer.observePoint(p, stamp, frame_dt);
  }
  for (const auto i : touched) {
    const int x = static_cast<int>(i % c.width), y = static_cast<int>(i / c.width);
    const auto state = columns.state(x, y);
    if (state == ColumnState::kOccupied) {
      if (flags[i] & 4U) {layer.observeHit(x, y, stamp, frame_dt);}
      else {layer.observeVisible(x, y, stamp);}
    }
    else if (state == ColumnState::kFree) {
      stats.ray_cleared_cells += layer.observeFree(x, y, stamp, frame_dt) ? 1U : 0U;
      ++stats.non_obstacle;
    }
  }
  stats.ray_voxels = mask.visited();
  layer.tick(stamp);
  const auto end = std::chrono::steady_clock::now();
  stats.prepare_ms = 1000.0 * std::chrono::duration<double>(prepared - begin).count();
  stats.ray_ms = 1000.0 * std::chrono::duration<double>(rays_done - prepared).count();
  stats.ground_ms = 1000.0 * std::chrono::duration<double>(ground_done - rays_done).count();
  stats.evidence_ms = 1000.0 * std::chrono::duration<double>(end - ground_done).count();
  return stats;
}
}  // namespace ats_rog_map
