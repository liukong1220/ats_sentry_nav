// Copyright 2026

#include "ats_rog_map/ground_column.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace ats_rog_map
{
ColumnState classifyGroundColumn(
  const std::vector<double> & occupied_z, const int observed_count,
  const std::optional<double> ground_z, const GroundColumnConfig & config,
  const std::optional<std::size_t> occupied_voxel_count)
{
  if (observed_count < config.min_observed || !ground_z || !std::isfinite(*ground_z) ||
    !std::all_of(occupied_z.begin(), occupied_z.end(), [](double z) {return std::isfinite(z);}))
  {
    return ColumnState::kUnknown;
  }
  if (occupied_z.empty()) {return ColumnState::kFree;}
  std::vector<double> heights = occupied_z;
  std::sort(heights.begin(), heights.end());
  heights.erase(std::unique(heights.begin(), heights.end()), heights.end());
  // Grid callers retain both height extrema for every occupied voxel. They
  // are clearance evidence, not two independent occupied voxels. Counting
  // extrema twice can turn a floor/board gap into a dense wall.
  const auto voxel_count = occupied_voxel_count.value_or(heights.size());
  if (voxel_count == 0) {return ColumnState::kUnknown;}
  const double span = heights.back() - heights.front();
  const double ratio = span > 1e-9 ? std::clamp(
    (voxel_count - 1) * config.voxel_resolution / span, 0.0, 1.0) : 1.0;
  // Surface thickness is relative to z_g. A thin low box is not a floor.
  if (heights.back() <= *ground_z + config.surface_max + 1e-9) {
    return ColumnState::kFree;
  }
  const double clearance = config.robot_height + config.top_margin;
  for (const double z : heights) {
    const double height = z - *ground_z;
    if (height > config.surface_max + 1e-9 && height < clearance - 1e-9) {
      return ColumnState::kOccupied;
    }
  }
  // Every non-floor return in this column lies at or above `clearance` (`robot_height + top_margin`),
  // so the robot body height band `(surface_max, clearance)` is completely obstacle-free.
  // Do NOT apply `wall_span_min`/`wall_ratio_min` to superstructure returns strictly above
  // `clearance` (e.g. a 30-100 cm thick overhead bridge beam or tunnel portal front face).
  const auto ceiling = std::find_if(heights.begin(), heights.end(), [&](double z) {
    return z > *ground_z + config.surface_max + 1e-9;
  });
  if (ceiling != heights.end() && *ceiling - *ground_z >= clearance - 1e-9) {
    return config.tunnel_free_enabled ? ColumnState::kTunnelFree : ColumnState::kFree;
  }
  if (span >= config.wall_span_min && ratio >= config.wall_ratio_min) {
    return ColumnState::kOccupied;
  }
  return ColumnState::kOccupied;
}

GroundColumnGrid::GroundColumnGrid(
  int width, int height, double resolution, double origin_x, double origin_y,
  GroundColumnConfig config)
: width_(width), height_(height), resolution_(resolution), origin_x_(origin_x),
  origin_y_(origin_y), config_(config)
{
  if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(config.voxel_resolution) || config.voxel_resolution <= 0.0)
  {throw std::invalid_argument("invalid ground column geometry");}
  columns_.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  ground_.resize(columns_.size());
  ray_observed_.resize(columns_.size(), false);
  frame_touched_.resize(columns_.size(), false);
  bfs_queued_.resize(columns_.size(), 0U);
  observed_scratch_.resize(columns_.size(), 0U);
  tunnel_free_observations_.resize(columns_.size(), 0U);
}

void GroundColumnGrid::markFrameTouched(std::size_t i)
{
  if (!frame_touched_[i]) {
    frame_touched_[i] = true;
    frame_touched_indices_.push_back(i);
  }
}

void GroundColumnGrid::invalidateSupportPlanes3x3(int x, int y)
{
  for (int sy = -1; sy <= 1; ++sy) {
    for (int sx = -1; sx <= 1; ++sx) {
      if (valid(x + sx, y + sy)) {
        support_planes_.erase(index(x + sx, y + sy));
      }
    }
  }
}

bool GroundColumnGrid::valid(int x, int y) const
{return x >= 0 && y >= 0 && x < width_ && y < height_;}

std::size_t GroundColumnGrid::index(int x, int y) const
{return static_cast<std::size_t>(y) * width_ + x;}

bool GroundColumnGrid::extendExtent(ReturnExtent & target, const ReturnExtent & source)
{
  bool changed = false;
  if (source.low[2] < target.low[2]) {target.low = source.low; changed = true;}
  if (source.high[2] > target.high[2]) {target.high = source.high; changed = true;}
  for (int a = 0; a < 2; ++a) {
    if (source.xy_bounds[a] < target.xy_bounds[a]) {
      target.xy_bounds[a] = source.xy_bounds[a];
      changed = true;
    }
    if (source.xy_bounds[a + 2] > target.xy_bounds[a + 2]) {
      target.xy_bounds[a + 2] = source.xy_bounds[a + 2];
      changed = true;
    }
  }
  return changed;
}

bool GroundColumnGrid::extendReturns(VoxelReturns & target, const VoxelReturns & source)
{
  bool changed = extendExtent(target, source);
  for (const auto & patch : source.patches) {
    const auto found = std::find_if(target.patches.begin(), target.patches.end(),
      [&](const auto & entry) {return entry.first == patch.first;});
    if (found == target.patches.end()) {
      target.patches.push_back(patch);
      changed = true;
    } else if (extendExtent(found->second, patch.second)) {
      changed = true;
    }
  }
  return changed;
}

void GroundColumnGrid::observe(
  const std::vector<std::array<double, 3>> & points,
  const std::array<double, 3> & robot_ground)
{
  raw_state_cache_.clear();
  last_updated_ground_indices_.clear();
  bool any_ground_changed = false;
  std::vector<std::size_t> observed_cells;
  observed_cells.reserve(std::min(points.size(), columns_.size()));
  for (const auto & p : points) {
    if (!std::all_of(p.begin(), p.end(), [](double v) {return std::isfinite(v);})) {continue;}
    const int x = static_cast<int>(std::floor((p[0] - origin_x_) / resolution_));
    const int y = static_cast<int>(std::floor((p[1] - origin_y_) / resolution_));
    if (!valid(x, y)) {++out_of_range_; continue;}
    const auto i = index(x, y);
    const bool overhead_before = hasOverhead(x, y);
    auto & column = columns_[i];
    const int bin = static_cast<int>(std::floor(p[2] / config_.voxel_resolution));
    const int px = std::clamp(static_cast<int>(
      4 * ((p[0] - origin_x_) / resolution_ - x)), 0, 3);
    const int py = std::clamp(static_cast<int>(
      4 * ((p[1] - origin_y_) / resolution_ - y)), 0, 3);
    const ReturnExtent extent{p, p, {p[0], p[1], p[0], p[1]}};
    const VoxelReturns observed{extent, {{py * 4 + px, extent}}};
    const auto inserted = column.emplace(bin, observed);
    if (!inserted.second) {extendReturns(inserted.first->second, observed);}
    if (!overhead_before && hasOverhead(x, y)) {
      support_planes_.erase(i);
    }
    markFrameTouched(i);
    if (!observed_scratch_[i]) {
      observed_scratch_[i] = 1U;
      observed_cells.push_back(i);
    }
  }
  for (const auto i : observed_cells) {observed_scratch_[i] = 0U;}
  // Preserve the connected identity while retaining every higher return in its
  // fixed surface band, including another Z voxel. A disconnected lower surface
  // must not replace it, and the band cannot climb while the bins are merged.
  const auto merge_support_band = [&](std::size_t i) -> bool {
      bool changed = false;
      const auto anchor = ground_[i]->low;
      const double top = anchor[2] + config_.surface_max;
      for (const auto & bin : columns_[i]) {
        const auto & observed = bin.second;
        if (observed.high[2] > top + 1e-9) {continue;}
        if (observed.low[2] < anchor[2] - config_.max_ground_slope *
          std::hypot(observed.low[0] - anchor[0], observed.low[1] - anchor[1]) -
          config_.support_noise - 1e-9) {continue;}
        if (extendReturns(*ground_[i], observed)) {
          changed = true;
        }
      }
      if (changed) {
        const int gx = static_cast<int>(i % static_cast<std::size_t>(width_));
        const int gy = static_cast<int>(i / static_cast<std::size_t>(width_));
        invalidateSupportPlanes3x3(gx, gy);
        any_ground_changed = true;
      }
      return changed;
    };
  std::queue<std::pair<int, int>> queue;
  std::vector<std::size_t> queued_indices;
  const auto enqueue = [&](int x, int y) {
      const auto idx = index(x, y);
      if (!bfs_queued_[idx]) {
        bfs_queued_[idx] = 1U;
        queued_indices.push_back(idx);
        queue.emplace(x, y);
      }
    };
  const bool have_robot_ground =
    std::isfinite(robot_ground[0]) && std::isfinite(robot_ground[1]) &&
    std::isfinite(robot_ground[2]);
  // Find the Z-bin in column matching a reference surface point within bounded slope.
  // Checking all bins (not just begin()) allows a robot on an elevated deck with a hollow
  // underside (e.g. a bridge/platform shell seen from below earlier) to connect its true
  // deck surface rather than staying permanently latched to the lower underside shell.
  const auto find_matching_bin = [&](const std::map<int, VoxelReturns> & col,
      const Point & ref, bool is_robot_seed) -> const VoxelReturns * {
      const VoxelReturns * best = nullptr;
      double best_dz = std::numeric_limits<double>::infinity();
      for (const auto & entry : col) {
        const auto & cand = entry.second;
        const double dz = std::abs(cand.low[2] - ref[2]);
        const double slope_limit = config_.max_ground_slope *
          std::hypot(cand.low[0] - ref[0], cand.low[1] - ref[1]) +
          config_.support_noise + 1e-6;
        const double limit = is_robot_seed ? 0.05 : slope_limit;
        if (dz <= limit && dz < best_dz) {
          best_dz = dz;
          best = &cand;
        }
      }
      return best;
    };
  // Prune stale hollow-deck underside-shell bins strictly in (surface_max, clearance) below
  // an active upper deck (too tight to be a real navigable lower floor under an overpass) so
  // mesh underside artifacts observed earlier do not poison hasDisconnectedLowerReturn(),
  // while preserving genuine 2-level overpass ramps (>= clearance below the upper bridge).
  const double clearance_gap = config_.robot_height + config_.top_margin;
  const auto prune_underside_bins = [&](std::size_t i, const VoxelReturns & deck) -> bool {
      auto & col = columns_[i];
      bool pruned = false;
      for (auto it = col.begin(); it != col.end(); ) {
        const double depth = deck.low[2] - it->second.low[2];
        if (depth > config_.surface_max + 1e-9 && depth < clearance_gap - 1e-9 &&
          it->second.high[2] < deck.low[2] - 0.02)
        {
          it = col.erase(it);
          pruned = true;
        } else {
          ++it;
        }
      }
      return pruned;
    };
  // Seed nearby returns inside ground_seed_radius around robot_ground.
  if (have_robot_ground)
  {
    const int cx = static_cast<int>(std::floor((robot_ground[0] - origin_x_) / resolution_));
    const int cy = static_cast<int>(std::floor((robot_ground[1] - origin_y_) / resolution_));
    const int r_cells = static_cast<int>(std::ceil(config_.ground_seed_radius / resolution_)) + 1;
    const int x0 = std::max(0, cx - r_cells);
    const int x1 = std::min(width_ - 1, cx + r_cells);
    const int y0 = std::max(0, cy - r_cells);
    const int y1 = std::min(height_ - 1, cy + r_cells);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const auto i = index(x, y);
        if (columns_[i].empty()) {continue;}
        const bool need_reseed = !ground_[i] ||
          std::abs(ground_[i]->low[2] - robot_ground[2]) > 0.08;
        if (!need_reseed) {
          if (ground_[i] && prune_underside_bins(i, *ground_[i])) {
            markFrameTouched(i);
            last_updated_ground_indices_.push_back(i);
          }
          continue;
        }
        const double distance = std::hypot(origin_x_ + (x + 0.5) * resolution_ - robot_ground[0],
          origin_y_ + (y + 0.5) * resolution_ - robot_ground[1]);
        if (distance > config_.ground_seed_radius) {continue;}
        const auto * candidate = find_matching_bin(columns_[i], robot_ground, true);
        if (candidate != nullptr)
        {
          const VoxelReturns seeded = *candidate;
          prune_underside_bins(i, seeded);
          ground_[i] = seeded;
          invalidateSupportPlanes3x3(x, y);
          any_ground_changed = true;
          merge_support_band(i);
          markFrameTouched(i);
          last_updated_ground_indices_.push_back(i);
          enqueue(x, y);
        }
      }
    }
  }
  const auto seed_from_neighbors = [&](std::size_t i) {
      if (columns_[i].empty() || ground_[i]) {return;}
      const int x = static_cast<int>(i % static_cast<std::size_t>(width_));
      const int y = static_cast<int>(i / static_cast<std::size_t>(width_));
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if ((dx == 0 && dy == 0) || !valid(x + dx, y + dy)) {continue;}
          if (ground_[index(x + dx, y + dy)]) {enqueue(x + dx, y + dy);}
        }
      }
    };
  // Existing connected ground is stable in odom; only update observed cells and
  // propagate from neighbours of newly observed or recently cleared cells.
  for (const auto i : observed_cells) {
    if (ground_[i]) {
      if (have_robot_ground) {
        // If ground_[i] is stuck on a hollow-deck underside shell strictly in
        // (surface_max, clearance_gap) below a higher surface bin in the same column
        // that matches robot_ground[2] (e.g. hollow bridge deck z=0.32 over underside
        // z=0.20), immediately upgrade ground_[i] so flat decks ahead of the robot are
        // never falsely classified as occupied walls before BFS reaches them.
        const double cur_diff = std::abs(ground_[i]->low[2] - robot_ground[2]);
        if (cur_diff > 0.08) {
          const VoxelReturns * upper = nullptr;
          double best_upper_diff = 0.12;
          for (const auto & entry : columns_[i]) {
            const auto & cand = entry.second;
            const double depth = cand.low[2] - ground_[i]->low[2];
            const double diff = std::abs(cand.low[2] - robot_ground[2]);
            if (depth > config_.surface_max + 1e-9 && depth < clearance_gap - 1e-9 &&
              diff <= best_upper_diff && diff + 0.05 < cur_diff)
            {
              best_upper_diff = diff;
              upper = &cand;
            }
          }
          if (upper != nullptr) {
            const VoxelReturns upgraded = *upper;
            prune_underside_bins(i, upgraded);
            ground_[i] = upgraded;
            const int gx = static_cast<int>(i % static_cast<std::size_t>(width_));
            const int gy = static_cast<int>(i / static_cast<std::size_t>(width_));
            invalidateSupportPlanes3x3(gx, gy);
            any_ground_changed = true;
            markFrameTouched(i);
            last_updated_ground_indices_.push_back(i);
          }
        }
        if (std::abs(ground_[i]->low[2] - robot_ground[2]) <= 0.12) {
          if (prune_underside_bins(i, *ground_[i])) {
            markFrameTouched(i);
            last_updated_ground_indices_.push_back(i);
          }
        }
      }
      merge_support_band(i);
      const int x = static_cast<int>(i % static_cast<std::size_t>(width_));
      const int y = static_cast<int>(i / static_cast<std::size_t>(width_));
      enqueue(x, y);
    } else {
      seed_from_neighbors(i);
    }
  }
  for (const auto i : cleared_ground_indices_) {
    seed_from_neighbors(i);
  }
  cleared_ground_indices_.clear();
  while (!queue.empty()) {
    const auto [x, y] = queue.front();
    queue.pop();
    const auto & support = ground_[index(x, y)]->low;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || !valid(x + dx, y + dy)) {continue;}
        const auto i = index(x + dx, y + dy);
        if (columns_[i].empty()) {continue;}
        if (ground_[i]) {
          // If an adjacent traversed surface is continuous with a different Z-level bin in this
          // column (> surface_max away from ground_[i], e.g. hollow-deck underside OR a 2-level
          // overpass where ground_[i] is on the other deck), switch ground_[i] to the level
          // closest to the robot's current elevation and invalidate stale cached support planes.
          if (have_robot_ground && std::abs(ground_[i]->low[2] - support[2]) > config_.surface_max) {
            const auto * matched = find_matching_bin(columns_[i], support, false);
            if (matched != nullptr &&
              std::abs(matched->low[2] - robot_ground[2]) + 0.05 <
              std::abs(ground_[i]->low[2] - robot_ground[2]))
            {
              const VoxelReturns switched = *matched;
              if (switched.low[2] > ground_[i]->low[2]) {
                prune_underside_bins(i, switched);
              }
              ground_[i] = switched;
              invalidateSupportPlanes3x3(x + dx, y + dy);
              any_ground_changed = true;
              merge_support_band(i);
              markFrameTouched(i);
              last_updated_ground_indices_.push_back(i);
              enqueue(x + dx, y + dy);
            }
          }
          continue;
        }
        // Surface identity follows measured XY, not cell-centre spacing: the
        // latter can turn a near-vertical cell-edge step into a supporting ramp
        // and propagate false ground beyond the real floor's neighbourhood.
        // Keep the explicit measurement-noise allowance for adjacent returns.
        const auto * candidate = find_matching_bin(columns_[i], support, false);
        if (candidate != nullptr)
        {
          const VoxelReturns connected = *candidate;
          if (have_robot_ground && std::abs(connected.low[2] - robot_ground[2]) <= 0.10) {
            prune_underside_bins(i, connected);
          }
          ground_[i] = connected;
          invalidateSupportPlanes3x3(x + dx, y + dy);
          any_ground_changed = true;
          merge_support_band(i);
          markFrameTouched(i);
          last_updated_ground_indices_.push_back(i);
          enqueue(x + dx, y + dy);
        }
      }
    }
  }
  if (any_ground_changed) {
    floor_planes_.clear();
  }
  for (const auto idx : queued_indices) {bfs_queued_[idx] = 0U;}
  if (config_.tunnel_free_enabled) {
    for (const auto i : frame_touched_indices_) {
      frame_touched_[i] = false;
      const int x = static_cast<int>(i % static_cast<std::size_t>(width_));
      const int y = static_cast<int>(i / static_cast<std::size_t>(width_));
      if (!hasOverhead(x, y)) {
        if (tunnel_free_observations_[i] > 0U) {
          const auto raw = rawState(x, y);
          if (raw == ColumnState::kOccupied || raw == ColumnState::kUnknown) {
            tunnel_free_observations_[i] = 0U;
          }
        }
        continue;
      }
      const auto raw = rawState(x, y);
      if (raw == ColumnState::kTunnelFree) {
        if (tunnel_free_observations_[i] < 65535U) {
          ++tunnel_free_observations_[i];
        }
      } else if (raw == ColumnState::kOccupied || raw == ColumnState::kUnknown) {
        tunnel_free_observations_[i] = 0U;
      }
    }
  } else {
    for (const auto i : frame_touched_indices_) {frame_touched_[i] = false;}
  }
  frame_touched_indices_.clear();
}

void GroundColumnGrid::observeBoundaryPoints(
  const std::vector<std::array<double, 3>> & points)
{
  // Insert obstacle-band boundary samples without wiping cached support_planes_
  // (these points lie strictly above ground + surface_max so they never alter ground_).
  for (const auto & p : points) {
    if (!std::all_of(p.begin(), p.end(), [](double v) {return std::isfinite(v);})) {continue;}
    const int x = static_cast<int>(std::floor((p[0] - origin_x_) / resolution_));
    const int y = static_cast<int>(std::floor((p[1] - origin_y_) / resolution_));
    if (!valid(x, y)) {continue;}
    const auto i = index(x, y);
    const bool overhead_before = hasOverhead(x, y);
    auto & column = columns_[i];
    const int bin = static_cast<int>(std::floor(p[2] / config_.voxel_resolution));
    const int px = std::clamp(static_cast<int>(
      4 * ((p[0] - origin_x_) / resolution_ - x)), 0, 3);
    const int py = std::clamp(static_cast<int>(
      4 * ((p[1] - origin_y_) / resolution_ - y)), 0, 3);
    const ReturnExtent extent{p, p, {p[0], p[1], p[0], p[1]}};
    const VoxelReturns observed{extent, {{py * 4 + px, extent}}};
    const auto inserted = column.emplace(bin, observed);
    if (!inserted.second) {extendReturns(inserted.first->second, observed);}
    if (!overhead_before && hasOverhead(x, y)) {
      support_planes_.erase(i);
    }
    raw_state_cache_.erase(i);
  }
}

std::optional<double> GroundColumnGrid::ground(int x, int y) const
{
  if (!valid(x, y)) {return std::nullopt;}
  const auto bounds = supportBounds(x, y, origin_x_ + (x + 0.5) * resolution_,
    origin_y_ + (y + 0.5) * resolution_);
  if (!bounds) {return std::nullopt;}
  return (*bounds)[0];
}


bool GroundColumnGrid::clearVoxel(const std::array<double, 3> & point)
{
  return clearVoxel(point, -std::numeric_limits<double>::infinity());
}

bool GroundColumnGrid::clearVoxel(const std::array<double, 3> & point, double ray_z)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return clearVoxel(point, Point{nan, nan, ray_z});
}

bool GroundColumnGrid::clearVoxel(
  const std::array<double, 3> & point, const std::array<double, 3> & ray,
  bool * protected_voxel)
{
  if (protected_voxel) {*protected_voxel = false;}
  const double ray_z = ray[2];
  const int x = static_cast<int>(std::floor((point[0] - origin_x_) / resolution_));
  const int y = static_cast<int>(std::floor((point[1] - origin_y_) / resolution_));
  if (!valid(x, y)) {return false;}
  const bool disconnected_before = hasDisconnectedLowerReturn(x, y);
  auto & column = columns_[index(x, y)];
  const auto found = column.find(static_cast<int>(std::floor(point[2] / config_.voxel_resolution)));
  if (found == column.end()) {return false;}
  // A ray passing above a stored ground-surface return does not contradict it: a
  // grazing ray inside the same voxel would otherwise erase a horizontal surface
  // (a platform edge) and leave the column empty and ray-observed, i.e. falsely
  // free. Only a ray at or below the stored height shows that return is gone.
  // If a voxel holds both a low floor point and a higher removed-obstacle point
  // (e.g. a box bottom return inside the floor Z-bin), a ray passing strictly
  // below high[2] - max_ground_slope * d_high - support_noise proves that high
  // surface is gone even though the ray stays above low[2]; clearing lets the
  // next scan cleanly re-observe only the real floor.
  // The protection is for the column's own ground. A return above ground +
  // surface_max is an obstacle, and the few rays that reach a low face voxel pass
  // just above it, so an obstacle voxel is cleared by any ray that crosses it.
  // On a slope the stored return and the ray sample have different XY: the
  // surface under the ray can be as low as low.z - slope * d, not low.z.
  const auto & low = found->second.low;
  const auto & high = found->second.high;
  const double d = std::isfinite(ray[0]) && std::isfinite(ray[1]) ?
    std::hypot(ray[0] - low[0], ray[1] - low[1]) : 0.0;
  const double d_high = std::isfinite(ray[0]) && std::isfinite(ray[1]) ?
    std::hypot(ray[0] - high[0], ray[1] - high[1]) : 0.0;
  const bool ray_below_high = std::isfinite(ray_z) &&
    ray_z < high[2] - config_.max_ground_slope * d_high - std::min(config_.support_noise, 0.005) - 1e-9;
  if (!ray_below_high &&
    std::isfinite(ray_z) && ray_z > low[2] - config_.max_ground_slope * d + 1e-9)
  {
    // The column's own connected ground, not the 3x3 minimum: beside a step the
    // minimum belongs to the lower neighbour and would make the upper surface
    // look like an obstacle. Protect the column's own floor bin and any
    // non-obstacle ground-band voxel. Any higher Z-bin that rawState() classifies
    // as an obstacle above support->lower + surface_max (e.g. a low front-face
    // box voxel at 0.06-0.10 m above a flat floor) must remain ray-clearable.
    const auto & floor = ground_[index(x, y)];
    if (floor) {
      const int floor_low_bin =
        static_cast<int>(std::floor(floor->low[2] / config_.voxel_resolution));
      const bool thick_or_higher_bin =
        found->first > floor_low_bin ||
        found->second.high[2] - floor->low[2] > config_.plane_residual_max + 1e-9;
      if (!thick_or_higher_bin) {
        // Fast path: thin floor-bin return (<= plane_residual_max above floor->low).
        if (protected_voxel) {*protected_voxel = true;}
        return false;
      }
      const auto support = voxelSupportInterval(x, y, found->second);
      const bool raw_obstacle_above_floor =
        support && (found->second.high[2] - support->lower > config_.surface_max + 1e-9);
      if (!raw_obstacle_above_floor &&
        found->second.high[2] <= floor->low[2] + config_.surface_max + 1e-9)
      {
        if (protected_voxel) {*protected_voxel = true;}
        return false;
      }
    } else {
      const auto support = voxelSupportInterval(x, y, found->second);
      if (!support || found->second.high[2] <= support->upper + config_.surface_max + 1e-9) {
        if (protected_voxel) {*protected_voxel = true;}
        return false;
      }
    }
  }
  auto & floor = ground_[index(x, y)];
  const bool removed_support = floor &&
    static_cast<int>(std::floor(floor->low[2] / config_.voxel_resolution)) <= found->first &&
    static_cast<int>(std::floor(floor->high[2] / config_.voxel_resolution)) >= found->first;
  if (removed_support) {
    floor.reset();
    tunnel_free_observations_[index(x, y)] = 0U;
    cleared_ground_indices_.push_back(index(x, y));
  }
  column.erase(found);
  if (!hasOverhead(x, y)) {
    tunnel_free_observations_[index(x, y)] = 0U;
  }
  // Removing the last overhead return disables the local board-support model.
  support_planes_.erase(index(x, y));
  raw_state_cache_.clear();
  const bool support_changed = removed_support ||
    disconnected_before != hasDisconnectedLowerReturn(x, y);
  if (support_changed) {
    // A support contributes only to its 3 x 3 neighbourhood. Clearing an
    // obstacle does not change a floor fit unless it changes support ambiguity.
    // A newly usable neighbour must invalidate formerly trimmed local fits.
    invalidateSupportPlanes3x3(x, y);
    floor_planes_.clear();
  }
  return support_changed;
}

void GroundColumnGrid::observeRayCell(int x, int y)
{
  if (valid(x, y)) {
    const auto i = index(x, y);
    ray_observed_[i] = true;
    raw_state_cache_.erase(i);
    markFrameTouched(i);
  }
}

void GroundColumnGrid::clear()
{
  support_planes_.clear();
  floor_planes_.clear();
  raw_state_cache_.clear();
  for (auto & c : columns_) {c.clear();}
  std::fill(ground_.begin(), ground_.end(), std::nullopt);
  std::fill(ray_observed_.begin(), ray_observed_.end(), false);
  std::fill(frame_touched_.begin(), frame_touched_.end(), false);
  frame_touched_indices_.clear();
  cleared_ground_indices_.clear();
  last_updated_ground_indices_.clear();
  std::fill(tunnel_free_observations_.begin(), tunnel_free_observations_.end(), 0U);
}
}  // namespace ats_rog_map
