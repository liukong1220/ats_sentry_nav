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
  if (span >= config.wall_span_min && ratio >= config.wall_ratio_min) {
    return ColumnState::kOccupied;
  }
  const double clearance = config.robot_height + config.top_margin;
  for (const double z : heights) {
    const double height = z - *ground_z;
    if (height > config.surface_max + 1e-9 && height < clearance - 1e-9) {
      return ColumnState::kOccupied;
    }
  }
  // A hollow column is accepted only after checking actual headroom. High
  // overhead surfaces also clear the robot, even when outside the tunnel band.
  const bool hollow = span >= config.tunnel_span_min && span <= config.tunnel_span_max &&
    ratio <= config.tunnel_ratio_max;
  const auto ceiling = std::find_if(heights.begin(), heights.end(), [&](double z) {
    return z > *ground_z + config.surface_max + 1e-9;
  });
  if (ceiling != heights.end() && *ceiling - *ground_z >= clearance - 1e-9 &&
    (hollow || span > config.tunnel_span_max || heights.front() > *ground_z + config.surface_max))
  {
    return ColumnState::kFree;
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
}

bool GroundColumnGrid::valid(int x, int y) const
{return x >= 0 && y >= 0 && x < width_ && y < height_;}

std::size_t GroundColumnGrid::index(int x, int y) const
{return static_cast<std::size_t>(y) * width_ + x;}

void GroundColumnGrid::extendExtent(ReturnExtent & target, const ReturnExtent & source)
{
  if (source.low[2] < target.low[2]) {target.low = source.low;}
  if (source.high[2] > target.high[2]) {target.high = source.high;}
  for (int a = 0; a < 2; ++a) {
    target.xy_bounds[a] = std::min(target.xy_bounds[a], source.xy_bounds[a]);
    target.xy_bounds[a + 2] = std::max(target.xy_bounds[a + 2], source.xy_bounds[a + 2]);
  }
}

void GroundColumnGrid::extendReturns(VoxelReturns & target, const VoxelReturns & source)
{
  extendExtent(target, source);
  for (const auto & patch : source.patches) {
    const auto found = std::find_if(target.patches.begin(), target.patches.end(),
      [&](const auto & entry) {return entry.first == patch.first;});
    if (found == target.patches.end()) {target.patches.push_back(patch);}
    else {extendExtent(found->second, patch.second);}
  }
}

void GroundColumnGrid::observe(
  const std::vector<std::array<double, 3>> & points,
  const std::array<double, 3> & robot_ground)
{
  support_planes_.clear();
  for (const auto & p : points) {
    if (!std::all_of(p.begin(), p.end(), [](double v) {return std::isfinite(v);})) {continue;}
    const int x = static_cast<int>(std::floor((p[0] - origin_x_) / resolution_));
    const int y = static_cast<int>(std::floor((p[1] - origin_y_) / resolution_));
    if (!valid(x, y)) {++out_of_range_; continue;}
    auto & column = columns_[index(x, y)];
    const int bin = static_cast<int>(std::floor(p[2] / config_.voxel_resolution));
    const int px = std::clamp(static_cast<int>(
      4 * ((p[0] - origin_x_) / resolution_ - x)), 0, 3);
    const int py = std::clamp(static_cast<int>(
      4 * ((p[1] - origin_y_) / resolution_ - y)), 0, 3);
    const ReturnExtent extent{p, p, {p[0], p[1], p[0], p[1]}};
    const VoxelReturns observed{extent, {{py * 4 + px, extent}}};
    const auto inserted = column.emplace(bin, observed);
    if (!inserted.second) {extendReturns(inserted.first->second, observed);}
  }
  // Preserve the connected identity while retaining every higher return in its
  // fixed surface band, including another Z voxel. A disconnected lower surface
  // must not replace it, and the band cannot climb while the bins are merged.
  const auto merge_support_band = [&](std::size_t i) {
      const auto anchor = ground_[i]->low;
      const double top = anchor[2] + config_.surface_max;
      for (const auto & bin : columns_[i]) {
        const auto & observed = bin.second;
        if (observed.high[2] > top + 1e-9) {continue;}
        if (observed.low[2] < anchor[2] - config_.max_ground_slope *
          std::hypot(observed.low[0] - anchor[0], observed.low[1] - anchor[1]) -
          config_.support_noise - 1e-9) {continue;}
        extendReturns(*ground_[i], observed);
      }
    };
  std::queue<std::pair<int, int>> queue;
  // Existing connected ground is stable in odom; map->odom corrections do not
  // move it. Nearby returns seed only across the bounded supporting slope.
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const auto i = index(x, y);
      if (columns_[i].empty()) {continue;}
      const double distance = std::hypot(origin_x_ + (x + 0.5) * resolution_ - robot_ground[0],
        origin_y_ + (y + 0.5) * resolution_ - robot_ground[1]);
      const auto & candidate = columns_[i].begin()->second;
      // Slope is checked between connected neighbors below. A distance-based
      // seed tolerance would let a nearby hanging plate invent its own floor.
      if (ground_[i] || (distance <= config_.ground_seed_radius &&
        std::abs(candidate.low[2] - robot_ground[2]) <= 0.05))
      {
        if (!ground_[i]) {ground_[i] = candidate;}
        merge_support_band(i);
        queue.emplace(x, y);
      }
    }
  }
  while (!queue.empty()) {
    const auto [x, y] = queue.front();
    queue.pop();
    const auto & support = ground_[index(x, y)]->low;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || !valid(x + dx, y + dy)) {continue;}
        const auto i = index(x + dx, y + dy);
        if (ground_[i] || columns_[i].empty()) {continue;}
        const auto & candidate = columns_[i].begin()->second;
        // Surface identity follows measured XY, not cell-centre spacing: the
        // latter can turn a near-vertical cell-edge step into a supporting ramp
        // and propagate false ground beyond the real floor's neighbourhood.
        // Keep the explicit measurement-noise allowance for adjacent returns.
        if (std::abs(candidate.low[2] - support[2]) <= config_.max_ground_slope *
            std::hypot(candidate.low[0] - support[0], candidate.low[1] - support[1]) +
            config_.support_noise + 1e-6)
        {ground_[i] = candidate; merge_support_band(i); queue.emplace(x + dx, y + dy);}
      }
    }
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

std::optional<GroundColumnGrid::SupportPlane> GroundColumnGrid::fitSupportPlane(
  const std::vector<const VoxelReturns *> & supports) const
{
  if (supports.size() < 6) {return std::nullopt;}
  std::vector<const ReturnExtent *> patches;
  for (const auto * support : supports) {
    for (const auto & patch : support->patches) {patches.push_back(&patch.second);}
  }
  Point mean{};
  for (const auto * patch : patches) {
    for (int a = 0; a < 3; ++a) {mean[a] += patch->high[a] / patches.size();}
  }
  double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
  std::vector<std::array<double, 2>> positions;
  for (const auto * patch : patches) {
    // A later lower return never erases the previously measured high surface.
    const auto & p = patch->high;
    const double u = p[0] - mean[0], v = p[1] - mean[1], w = p[2] - mean[2];
    xx += u * u; xy += u * v; yy += v * v; xz += u * w; yz += v * w;
    positions.push_back({p[0], p[1]});
  }
  const double determinant = xx * yy - xy * xy;
  if (determinant <= 1e-8 * (xx + yy) * (xx + yy)) {return std::nullopt;}
  const double a = (xz * yy - yz * xy) / determinant;
  const double b = (yz * xx - xz * xy) / determinant;
  if (std::hypot(a, b) > config_.max_ground_slope) {return std::nullopt;}
  double residual = 0;
  for (const auto * patch : patches) {
    // Fit the highest retained supporting surface in each XY patch. Lower
    // scatter cannot create headroom; higher support remains in every fit.
    for (const double px : {patch->xy_bounds[0], patch->xy_bounds[2]}) {
      for (const double py : {patch->xy_bounds[1], patch->xy_bounds[3]}) {
        const double fitted = mean[2] + a * (px - mean[0]) + b * (py - mean[1]);
        residual = std::max(residual, std::abs(patch->high[2] - fitted));
      }
    }
  }
  if (residual > config_.plane_residual_max) {return std::nullopt;}
  std::sort(positions.begin(), positions.end());
  positions.erase(std::unique(positions.begin(), positions.end()), positions.end());
  const auto cross = [](const auto & o, const auto & p, const auto & q) {
      return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
    };
  std::vector<std::array<double, 2>> hull;
  for (const auto & p : positions) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const auto lower_size = hull.size();
  for (auto it = positions.rbegin() + 1; it != positions.rend(); ++it) {
    while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *it) <= 0) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  hull.pop_back();
  if (hull.size() < 3) {return std::nullopt;}
  return SupportPlane{CeilingPlane{mean[0], mean[1], mean[2], a, b, residual}, std::move(hull)};
}

bool GroundColumnGrid::hasOverhead(int x, int y) const
{
  const auto & support = ground_[index(x, y)];
  if (!support) {return false;}
  const double surface = support->high[2] + config_.surface_max + 1e-9;
  const auto & bins = columns_[index(x, y)];
  return std::any_of(bins.begin(), bins.end(), [surface](const auto & bin) {
      return bin.second.low[2] > surface;
    });
}

std::optional<std::array<double, 2>> GroundColumnGrid::supportBounds(
  int x, int y, double query_x, double query_y, double * plane_z) const
{
  if (plane_z) {*plane_z = std::numeric_limits<double>::quiet_NaN();}
  // A neighbouring identity contradicted by a disconnected lower return must
  // not constrain an independently measured level to which it cannot connect.
  // Keep the column's own support and all unambiguous neighbours; absent own
  // support keeps the conservative original neighbourhood.
  const auto compatible = [&](int nx, int ny) {
      const auto & own = ground_[index(x, y)];
      const auto & other = ground_[index(nx, ny)];
      if (!own || !other || (x == nx && y == ny)) {return true;}
      const auto & a = own->low;
      const auto & b = other->low;
      const bool directly_connected = std::abs(a[2] - b[2]) <= config_.max_ground_slope *
        std::hypot(a[0] - b[0], a[1] - b[1]) + config_.support_noise + 1e-6;
      return directly_connected || !hasDisconnectedLowerReturn(nx, ny);
    };
  std::vector<const VoxelReturns *> supports;
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      const auto & p = ground_[index(x + dx, y + dy)];
      if (!p || !compatible(x + dx, y + dy)) {continue;}
      supports.push_back(&*p);
      lower = std::max(lower, p->low[2] - config_.max_ground_slope *
        std::hypot(query_x - p->low[0], query_y - p->low[1]));
      upper = std::min(upper, p->high[2] + config_.max_ground_slope *
        std::hypot(query_x - p->high[0], query_y - p->high[1]));
    }
  }
  if (supports.empty()) {return std::nullopt;}
  // Fit only observed connected supports, not arbitrary low points or a
  // diagnostic/interpolated floor. Reject steps, rank deficiency and extrapolation.
  auto cached = support_planes_.find(index(x, y));
  if (cached == support_planes_.end()) {
    std::vector<SupportPlane> fits;
    if (auto fit = fitSupportPlane(supports)) {fits.push_back(std::move(*fit));}
    else if (hasOverhead(x, y)) {
      // Two adjacent rows/columns retain this column and exclude only the
      // opposite outer row. Every fit still needs six observed connected columns.
      for (int side = 0; side < 4; ++side) {
        std::vector<const VoxelReturns *> local;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if ((side == 0 && dx == -1) || (side == 1 && dx == 1) ||
              (side == 2 && dy == -1) || (side == 3 && dy == 1) ||
              !valid(x + dx, y + dy)) {continue;}
            const auto & floor = ground_[index(x + dx, y + dy)];
            if (floor && compatible(x + dx, y + dy)) {local.push_back(&*floor);}
          }
        }
        if (auto fit = fitSupportPlane(local)) {fits.push_back(std::move(*fit));}
      }
    }
    cached = support_planes_.emplace(index(x, y), std::move(fits)).first;
  }
  bool have_fit = false;
  double fit_lower = std::numeric_limits<double>::infinity();
  double fit_upper = -std::numeric_limits<double>::infinity();
  double fit_estimate = -std::numeric_limits<double>::infinity();
  for (const auto & fit : cached->second) {
    bool enclosed = true;
    for (std::size_t i = 0; i < fit.hull.size(); ++i) {
      const auto & a = fit.hull[i];
      const auto & b = fit.hull[(i + 1) % fit.hull.size()];
      const double cross = (b[0] - a[0]) * (query_y - a[1]) -
        (b[1] - a[1]) * (query_x - a[0]);
      if (cross < -1e-12) {enclosed = false; break;}
    }
    if (enclosed) {
      const double z = fit.plane.at(query_x, query_y);
      have_fit = true;
      fit_lower = std::min(fit_lower, z - fit.plane.tol);
      fit_upper = std::max(fit_upper, z + fit.plane.tol);
      fit_estimate = std::max(fit_estimate, z);
    }
  }
  if (have_fit) {
    if (plane_z) {*plane_z = fit_estimate;}
    return std::array<double, 2>{fit_lower, fit_upper};
  }
  // No valid planar support: retain the full Lipschitz uncertainty. An
  // inconsistent neighbourhood is unknown, never repaired by selecting a minimum.
  // Returns a few mm apart carry measurement noise beyond the slope bound.
  // Up to support_noise the interval spans both ends; beyond it is unknown.
  if (lower > upper + config_.support_noise + 1e-9) {return std::nullopt;}
  return std::array<double, 2>{std::min(lower, upper), std::max(lower, upper)};
}

std::optional<GroundColumnGrid::SupportInterval> GroundColumnGrid::voxelSupportInterval(
  int x, int y, const ReturnExtent & evidence) const
{
  // z extrema alone lose a low-clearance point on a slope. Enclose ALL
  // measured XY, including returns between the two stored z extrema.
  double lower = std::numeric_limits<double>::infinity();
  double upper = -std::numeric_limits<double>::infinity();
  double estimate = -std::numeric_limits<double>::infinity();
  bool all_planar = true;
  for (const double qx : {evidence.xy_bounds[0], evidence.xy_bounds[2]}) {
    for (const double qy : {evidence.xy_bounds[1], evidence.xy_bounds[3]}) {
      double plane_z = 0.0;
      const auto bounds = supportBounds(x, y, qx, qy, &plane_z);
      if (!bounds) {return std::nullopt;}
      all_planar = all_planar && std::isfinite(plane_z);
      if (std::isfinite(plane_z)) {estimate = std::max(estimate, plane_z);}
      lower = std::min(lower, (*bounds)[0]);
      upper = std::max(upper, (*bounds)[1]);
    }
  }
  // Include interior extrema of the slope-bound cones when no plane exists.
  const double diameter = std::hypot(evidence.xy_bounds[2] - evidence.xy_bounds[0],
    evidence.xy_bounds[3] - evidence.xy_bounds[1]);
  // supportBounds is Lipschitz; corners alone need not enclose its interior.
  // The centre query plus the rectangle radius supplies an independent bound.
  double centre_plane = 0.0;
  const auto centre = supportBounds(x, y,
    (evidence.xy_bounds[0] + evidence.xy_bounds[2]) * 0.5,
    (evidence.xy_bounds[1] + evidence.xy_bounds[3]) * 0.5, &centre_plane);
  if (!centre) {return std::nullopt;}
  // One fitted plane is linear: its extrema over the rectangle are at the
  // corners, so the slope-cone allowance only applies without a plane.
  // The corner bound is valid only for a plane defined over this WHOLE
  // extent. Per-query model switches must not create a fictitious linear patch.
  bool extent_planar = false;
  double extent_estimate = -std::numeric_limits<double>::infinity();
  const auto models = support_planes_.find(index(x, y));
  if (all_planar && std::isfinite(centre_plane) && models != support_planes_.end()) {
    for (const auto & fit : models->second) {
      bool enclosed = true;
      double height = -std::numeric_limits<double>::infinity();
      for (const double qx : {evidence.xy_bounds[0], evidence.xy_bounds[2]}) {
        for (const double qy : {evidence.xy_bounds[1], evidence.xy_bounds[3]}) {
          for (std::size_t i = 0; i < fit.hull.size(); ++i) {
            const auto & a = fit.hull[i];
            const auto & b = fit.hull[(i + 1) % fit.hull.size()];
            if ((b[0] - a[0]) * (qy - a[1]) - (b[1] - a[1]) * (qx - a[0]) < -1e-12) {
              enclosed = false;
            }
          }
          height = std::max(height, fit.plane.at(qx, qy));
        }
      }
      if (enclosed) {
        extent_planar = true;
        extent_estimate = std::max(extent_estimate, height);
      }
    }
  }
  if (extent_planar) {
    estimate = extent_estimate;
    // A well-posed trimmed fit over the connected supports gives the floor height
    // without the per-voxel extrema scatter; the corners still bound the voxel XY.
    if (evidence.high[2] - lower > config_.surface_max + 1e-9) {
     if (const auto floor = floorPlane(x, y)) {
      double fitted = -std::numeric_limits<double>::infinity();
      for (const double qx : {evidence.xy_bounds[0], evidence.xy_bounds[2]}) {
        for (const double qy : {evidence.xy_bounds[1], evidence.xy_bounds[3]}) {
          fitted = std::max(fitted, floor->at(qx, qy));
        }
      }
      estimate = std::min(estimate, fitted);
     }
    }
    return SupportInterval{lower, upper, estimate};
  }
  lower = std::min(lower, (*centre)[0] - config_.max_ground_slope * diameter * 0.5);
  upper = std::max(upper, (*centre)[1] + config_.max_ground_slope * diameter * 0.5);
  return SupportInterval{lower, upper, upper};
}

std::optional<GroundColumnGrid::CeilingPlane> GroundColumnGrid::ceilingPlane(int x, int y) const
{
  if (!config_.ceiling_plane_enabled) {return std::nullopt;}
  const double clearance = config_.robot_height + config_.top_margin;
  std::vector<Point> points;
  for (int dy = -config_.ceiling_plane_radius; dy <= config_.ceiling_plane_radius; ++dy) {
    for (int dx = -config_.ceiling_plane_radius; dx <= config_.ceiling_plane_radius; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      for (const auto & bin : columns_[index(x + dx, y + dy)]) {
        const auto support = supportBounds(x + dx, y + dy, bin.second.low[0], bin.second.low[1]);
        if (!support) {continue;}
        const double height = bin.second.low[2] - (*support)[1];
        // Non-floor returns that could be a ceiling for this robot, not a far roof.
        if (height <= config_.surface_max + 1e-9 || height > clearance + 0.30) {continue;}
        points.push_back(bin.second.low);
        if (bin.second.high[2] > bin.second.low[2]) {points.push_back(bin.second.high);}
      }
    }
  }
  return robustPlane(points, config_.ceiling_plane_band, config_.ceiling_plane_max_slope);
}

std::optional<GroundColumnGrid::CeilingPlane> GroundColumnGrid::robustPlane(
  const std::vector<Point> & points, double band, double max_slope) const
{
  if (static_cast<int>(points.size()) < config_.ceiling_plane_min_points) {return std::nullopt;}
  Point mean{};
  for (const auto & p : points) {for (int a = 0; a < 3; ++a) {mean[a] += p[a] / points.size();}}
  // Deterministic RANSAC start: mis-identified support or a wall edge among the
  // points would tilt a plain least-squares plane and collapse the trimmed set.
  std::vector<bool> inlier(points.size(), false);
  {
    std::uint32_t state = 2463534242u;
    const auto next = [&state]() {state ^= state << 13; state ^= state >> 17; state ^= state << 5; return state;};
    std::size_t best = 0;
    const std::size_t n = points.size();
    for (int trial = 0; trial < 96; ++trial) {
      const std::size_t i0 = next() % n, i1 = next() % n, i2 = next() % n;
      if (i0 == i1 || i1 == i2 || i0 == i2) {continue;}
      const auto & p0 = points[i0]; const auto & p1 = points[i1]; const auto & p2 = points[i2];
      const double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
      const double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
      const double det = ux * vy - uy * vx;
      if (std::abs(det) < 1e-4) {continue;}
      const double ga = (uz * vy - uy * vz) / det;
      const double gb = (ux * vz - uz * vx) / det;
      if (std::hypot(ga, gb) > max_slope) {continue;}
      std::size_t votes = 0;
      for (const auto & p : points) {
        if (std::abs(p[2] - (p0[2] + ga * (p[0] - p0[0]) + gb * (p[1] - p0[1]))) <= band) {++votes;}
      }
      if (votes > best) {
        best = votes;
        for (std::size_t i = 0; i < n; ++i) {
          inlier[i] = std::abs(points[i][2] - (p0[2] + ga * (points[i][0] - p0[0]) +
            gb * (points[i][1] - p0[1]))) <= band;
        }
      }
    }
    if (static_cast<int>(best) < config_.ceiling_plane_min_points) {return std::nullopt;}
  }
  double a = 0.0, b = 0.0, c = mean[2], sigma = 0.0;
  std::size_t count = points.size();
  for (int iteration = 0; iteration < 10; ++iteration) {
    Point m{};
    count = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!inlier[i]) {continue;}
      ++count;
      for (int k = 0; k < 3; ++k) {m[k] += points[i][k];}
    }
    if (static_cast<int>(count) < config_.ceiling_plane_min_points) {return std::nullopt;}
    for (int k = 0; k < 3; ++k) {m[k] /= count;}
    double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!inlier[i]) {continue;}
      const double u = points[i][0] - m[0], v = points[i][1] - m[1], w = points[i][2] - m[2];
      xx += u * u; xy += u * v; yy += v * v; xz += u * w; yz += v * w;
    }
    const double determinant = xx * yy - xy * xy;
    if (determinant <= 1e-8 * (xx + yy) * (xx + yy)) {return std::nullopt;}
    a = (xz * yy - yz * xy) / determinant;
    b = (yz * xx - xz * xy) / determinant;
    c = m[2]; mean = m;
    double sum = 0.0;
    std::vector<double> residual(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      residual[i] = points[i][2] - (c + a * (points[i][0] - m[0]) + b * (points[i][1] - m[1]));
      if (inlier[i]) {sum += residual[i] * residual[i];}
    }
    sigma = std::sqrt(sum / count);
    const double limit = std::min(band, std::max(2.5 * sigma, 0.004));
    bool changed = false;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const bool keep = std::abs(residual[i]) <= limit;
      if (keep != inlier[i]) {inlier[i] = keep; changed = true;}
    }
    if (!changed) {break;}
  }
  if (static_cast<int>(count) < config_.ceiling_plane_min_points ||
    count < config_.ceiling_plane_min_inlier_ratio * points.size() ||
    sigma > config_.ceiling_plane_max_sigma || std::hypot(a, b) > max_slope)
  {return std::nullopt;}
  double lo_x = 1e9, hi_x = -1e9, lo_y = 1e9, hi_y = -1e9;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!inlier[i]) {continue;}
    lo_x = std::min(lo_x, points[i][0]); hi_x = std::max(hi_x, points[i][0]);
    lo_y = std::min(lo_y, points[i][1]); hi_y = std::max(hi_y, points[i][1]);
  }
  if (hi_x - lo_x < config_.ceiling_plane_min_extent ||
    hi_y - lo_y < config_.ceiling_plane_min_extent) {return std::nullopt;}
  return CeilingPlane{mean[0], mean[1], c, a, b, std::min(band, std::max(2.5 * sigma, 0.004))};
}

std::optional<GroundColumnGrid::CeilingPlane> GroundColumnGrid::floorPlane(int x, int y) const
{
  if (!config_.floor_plane_enabled) {return std::nullopt;}
  std::vector<Point> points;
  for (int dy = -config_.ceiling_plane_radius; dy <= config_.ceiling_plane_radius; ++dy) {
    for (int dx = -config_.ceiling_plane_radius; dx <= config_.ceiling_plane_radius; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      const auto & support = ground_[index(x + dx, y + dy)];
      if (!support) {continue;}
      points.push_back(support->low);
      if (support->high[2] > support->low[2]) {points.push_back(support->high);}
    }
  }
  const auto plane = robustPlane(points, config_.floor_plane_band, config_.max_ground_slope);
  const auto & own_support = ground_[index(x, y)];
  if (!plane || !own_support) {return std::nullopt;}
  // A broad fit can reject a real local step as an outlier. It may refine this
  // column's headroom only if its OWN connected support remains on that plane.
  // Validate every retained XY patch, not only the two whole-column extrema.
  for (const auto & entry : own_support->patches) {
    const auto & patch = entry.second;
    for (const double px : {patch.xy_bounds[0], patch.xy_bounds[2]}) {
      for (const double py : {patch.xy_bounds[1], patch.xy_bounds[3]}) {
        const double fitted = plane->at(px, py);
        if (std::max(std::abs(patch.low[2] - fitted), std::abs(patch.high[2] - fitted)) >
          plane->tol + 1e-9)
        {return std::nullopt;}
      }
    }
  }
  return plane;
}

double GroundColumnGrid::snapToPlane(
  const std::optional<CeilingPlane> & plane, const std::array<double, 3> & p, double band)
{
  if (!plane) {return p[2];}
  const double z = plane->at(p[0], p[1]);
  return std::abs(p[2] - z) <= std::min(band, plane->tol) + 1e-9 ? z : p[2];
}

std::optional<double> GroundColumnGrid::boardClearance(
  int x, int y, const ReturnExtent & evidence, const std::optional<CeilingPlane> & ceiling) const
{
  if (!ceiling) {return std::nullopt;}
  const double band = config_.ceiling_plane_band;
  const auto on_board = [&](const Point & p) {
      return std::abs(p[2] - ceiling->at(p[0], p[1])) <= std::min(band, ceiling->tol) + 1e-9;
    };
  // Both stored z extrema must be the board; anything between them is no lower
  // than the lower extremum, so it is board or higher.
  if (!on_board(evidence.low) || !on_board(evidence.high)) {return std::nullopt;}
  const auto floor = floorPlane(x, y);
  if (!floor) {return std::nullopt;}
  double worst = std::numeric_limits<double>::infinity();
  for (const double qx : {evidence.xy_bounds[0], evidence.xy_bounds[2]}) {
    for (const double qy : {evidence.xy_bounds[1], evidence.xy_bounds[3]}) {
      worst = std::min(worst, ceiling->at(qx, qy) - floor->at(qx, qy));
    }
  }
  return worst;
}

std::optional<ColumnHeadroom> GroundColumnGrid::ceilingHeadroom(int x, int y) const
{
  if (!valid(x, y)) {return std::nullopt;}
  std::optional<ColumnHeadroom> result;
  std::optional<CeilingPlane> plane;
  bool plane_done = false;
  for (const auto & bin : columns_[index(x, y)]) {
    const auto whole_interval = voxelSupportInterval(x, y, bin.second);
    if (!whole_interval) {continue;}
    std::vector<const ReturnExtent *> ranges;
    if (hasOverhead(x, y) &&
      bin.second.high[2] - whole_interval->lower > config_.surface_max + 1e-9 &&
      bin.second.patches.size() > 1) {
      for (const auto & patch : bin.second.patches) {ranges.push_back(&patch.second);}
    } else {ranges.push_back(&bin.second);}
    for (const auto * range : ranges) {
      const auto & evidence = *range;
      const auto interval = range == &bin.second ? whole_interval :
        voxelSupportInterval(x, y, evidence);
      if (!interval || evidence.high[2] - interval->lower <= config_.surface_max + 1e-9) {continue;}
      if (!plane_done) {plane = ceilingPlane(x, y); plane_done = true;}
      const double low = snapToPlane(plane, evidence.low, config_.ceiling_plane_band);
      double headroom = low - interval->estimate;
      if (const auto board = boardClearance(x, y, evidence, plane)) {
        headroom = std::max(headroom, *board);
      }
      if (!result || headroom < result->headroom) {
        result = ColumnHeadroom{low, interval->lower, interval->upper,
          low - headroom, headroom};
      }
    }
  }
  return result;
}

bool GroundColumnGrid::hasDisconnectedLowerReturn(int x, int y) const
{
  const auto & floor = ground_[index(x, y)];
  if (!floor) {return false;}
  const auto & bins = columns_[index(x, y)];
  if (bins.empty()) {return false;}
  const auto & low = bins.begin()->second.low;
  const auto & anchor = floor->low;
  const double allowance = std::max(config_.surface_max, config_.max_ground_slope *
    std::hypot(low[0] - anchor[0], low[1] - anchor[1]) + config_.support_noise);
  return low[2] < anchor[2] - allowance - 1e-9;
}

ColumnState GroundColumnGrid::state(int x, int y) const
{
  if (!valid(x, y)) {return ColumnState::kUnknown;}
  const auto observed = std::max(static_cast<int>(columns_[index(x, y)].size()),
    ray_observed_[index(x, y)] ? 1 : 0);
  if (observed < config_.min_observed) {return ColumnState::kUnknown;}
  std::vector<double> heights;
  bool ceiling = false;
  std::optional<CeilingPlane> plane;
  bool plane_done = false;
  const double clearance = config_.robot_height + config_.top_margin;
  for (const auto & bin : columns_[index(x, y)]) {
    const auto whole_interval = voxelSupportInterval(x, y, bin.second);
    if (!whole_interval) {return ColumnState::kUnknown;}
    std::vector<const ReturnExtent *> ranges;
    if (hasOverhead(x, y) &&
      bin.second.high[2] - whole_interval->lower > config_.surface_max + 1e-9 &&
      bin.second.patches.size() > 1) {
      for (const auto & patch : bin.second.patches) {ranges.push_back(&patch.second);}
    } else {ranges.push_back(&bin.second);}
    for (const auto * range : ranges) {
      const auto & evidence = *range;
      const auto interval = range == &bin.second ? whole_interval :
        voxelSupportInterval(x, y, evidence);
      if (!interval) {return ColumnState::kUnknown;}
      const double lower = interval->lower;
      const double support = interval->estimate;
      double low_z = evidence.low[2];
      double high_z = evidence.high[2];
      if (evidence.high[2] - lower > config_.surface_max + 1e-9) {
        if (!plane_done) {plane = ceilingPlane(x, y); plane_done = true;}
        low_z = snapToPlane(plane, evidence.low, config_.ceiling_plane_band);
        high_z = snapToPlane(plane, evidence.high, config_.ceiling_plane_band);
        double headroom = low_z - support;
        if (const auto board = boardClearance(x, y, evidence, plane)) {
          headroom = std::max(headroom, *board);
          // The board is one surface: the heights below describe it at this clearance.
          low_z = support + headroom;
          high_z = std::max(high_z, low_z);
        }
        if (headroom < clearance - 1e-9) {return ColumnState::kOccupied;}
        ceiling = true;
      }
      // A lower disconnected return is an unexplained drop in this column.
      // It cannot become support or poison neighbours through the floor fit.
      if (evidence.low[2] < lower - config_.surface_max - 1e-9) {
        return ColumnState::kUnknown;
      }
      heights.push_back(low_z - support);
      heights.push_back(high_z - support);
    }
  }
  // Neighbouring floor evidence can identify a wall, but cannot establish
  // traversable support under a hanging plate. A column with only floor-band
  // returns, or none, keeps the neighbourhood support of appendix A.1: sparse
  // downsampled LIO endpoints leave most open-floor columns without their own.
  if (!ground_[index(x, y)]) {
    if (ceiling || !supportBounds(x, y, origin_x_ + (x + 0.5) * resolution_,
        origin_y_ + (y + 0.5) * resolution_))
    {return ColumnState::kUnknown;}
  }
  // Contradictory support stays blocked locally; earlier occupied decisions win.
  if (hasDisconnectedLowerReturn(x, y)) {return ColumnState::kUnknown;}
  return classifyGroundColumn(heights,
    observed, 0.0, config_, columns_[index(x, y)].size());
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
  const std::array<double, 3> & point, const std::array<double, 3> & ray)
{
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
  // The protection is for the column's own ground. A return above ground +
  // surface_max is an obstacle, and the few rays that reach a low face voxel pass
  // just above it, so an obstacle voxel is cleared by any ray that crosses it.
  // On a slope the stored return and the ray sample have different XY: the
  // surface under the ray can be as low as low.z - slope * d, not low.z.
  const auto & low = found->second.low;
  const double d = std::isfinite(ray[0]) && std::isfinite(ray[1]) ?
    std::hypot(ray[0] - low[0], ray[1] - low[1]) : 0.0;
  if (std::isfinite(ray_z) && ray_z > low[2] - config_.max_ground_slope * d + 1e-9) {
    // The column's own connected ground, not the 3x3 minimum: beside a step the
    // minimum belongs to the lower neighbour and would make the upper surface
    // look like an obstacle. Without own ground the voxel stays protected.
    const auto & floor = ground_[index(x, y)];
    if (!floor || found->second.low[2] <= floor->high[2] + config_.surface_max + 1e-9) {return false;}
  }
  auto & floor = ground_[index(x, y)];
  const bool removed_support = floor &&
    static_cast<int>(std::floor(floor->low[2] / config_.voxel_resolution)) <= found->first &&
    static_cast<int>(std::floor(floor->high[2] / config_.voxel_resolution)) >= found->first;
  if (removed_support) {floor.reset();}
  column.erase(found);
  // Removing the last overhead return disables the local board-support model.
  support_planes_.erase(index(x, y));
  const bool support_changed = removed_support ||
    disconnected_before != hasDisconnectedLowerReturn(x, y);
  if (support_changed) {
    // A support contributes only to its 3 x 3 neighbourhood. Clearing an
    // obstacle does not change a floor fit unless it changes support ambiguity.
    // A newly usable neighbour must invalidate formerly trimmed local fits.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (valid(x + dx, y + dy)) {support_planes_.erase(index(x + dx, y + dy));}
      }
    }
  }
  return support_changed;
}

void GroundColumnGrid::observeRayCell(int x, int y)
{
  if (valid(x, y)) {ray_observed_[index(x, y)] = true;}
}

void GroundColumnGrid::clear()
{
  support_planes_.clear();
  for (auto & c : columns_) {c.clear();}
  std::fill(ground_.begin(), ground_.end(), std::nullopt);
  std::fill(ray_observed_.begin(), ray_observed_.end(), false);
}
}  // namespace ats_rog_map
