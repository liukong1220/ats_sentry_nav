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
      // When this column already has its own connected ground, only neighbours on the same
      // continuous elevation surface may bound its local Lipschitz slope cone. Mixing a
      // disconnected upper platform or wall top (e.g. +0.20..0.40 m across a retaining wall)
      // makes lower > upper and turns the 5 cm floor strip along every tunnel wall into kUnknown.
      return std::abs(a[2] - b[2]) <= config_.max_ground_slope *
        std::hypot(a[0] - b[0], a[1] - b[1]) + config_.support_noise + 1e-6;
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
    // Skip 9x9 RANSAC floor fitting for returns strictly far below or far above
    // the headroom decision band (where lowering estimate cannot change classification).
    if (evidence.high[2] - lower > config_.surface_max + 1e-9) {
      const double clearance = config_.robot_height + config_.top_margin;
      const double plane_slack =
        config_.ceiling_plane_band + config_.floor_plane_band + 0.03;
      const bool near_surface = evidence.high[2] - lower <= config_.surface_max + 0.06;
      const bool near_clearance =
        (evidence.low[2] - lower >= clearance - plane_slack - 0.02) &&
        (evidence.low[2] - extent_estimate <= clearance + plane_slack + 0.02);
      if (near_surface || near_clearance) {
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
    }
    return SupportInterval{lower, upper, estimate, true};
  }
  lower = std::min(lower, (*centre)[0] - config_.max_ground_slope * diameter * 0.5);
  upper = std::max(upper, (*centre)[1] + config_.max_ground_slope * diameter * 0.5);
  return SupportInterval{lower, upper, upper, false};
}

std::optional<GroundColumnGrid::CeilingPlane> GroundColumnGrid::ceilingPlane(int x, int y) const
{
  if (!config_.ceiling_plane_enabled) {return std::nullopt;}
  const double clearance = config_.robot_height + config_.top_margin;
  const int plane_radius = std::max(
    config_.ceiling_plane_radius,
    static_cast<int>(std::ceil((config_.ceiling_plane_min_extent + 0.05) / resolution_)));
  std::vector<Point> points;
  for (int dy = -plane_radius; dy <= plane_radius; ++dy) {
    for (int dx = -plane_radius; dx <= plane_radius; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      for (const auto & bin : columns_[index(x + dx, y + dy)]) {
        const auto support = supportBounds(x + dx, y + dy, bin.second.low[0], bin.second.low[1]);
        if (!support) {continue;}
        const double height = bin.second.low[2] - (*support)[1];
        // Take ONLY the lowest non-floor return per column inside the headroom band
        // (the tunnel underside). Higher bins or bin.high on a vertical beam front face
        // would otherwise dilute the horizontal underside RANSAC inlier ratio below 0.6.
        if (height <= config_.surface_max + 1e-9) {continue;}
        if (height <= clearance + 0.08) {
          points.push_back(bin.second.low);
        }
        break;
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
  const double span_x = hi_x - lo_x;
  const double span_y = hi_y - lo_y;
  const double min_narrow_extent = std::min(config_.ceiling_plane_min_extent, 0.14);
  if (std::max(span_x, span_y) < config_.ceiling_plane_min_extent ||
    std::min(span_x, span_y) < min_narrow_extent)
  {return std::nullopt;}
  return CeilingPlane{
    mean[0], mean[1], c, a, b,
    std::min(band, std::max(2.5 * sigma, std::max(0.006, config_.support_noise)))};
}

std::optional<GroundColumnGrid::CeilingPlane> GroundColumnGrid::floorPlane(int x, int y) const
{
  if (!config_.floor_plane_enabled) {return std::nullopt;}
  const auto idx = index(x, y);
  const auto cached = floor_planes_.find(idx);
  if (cached != floor_planes_.end()) {return cached->second;}
  const int plane_radius = std::max(
    config_.ceiling_plane_radius, static_cast<int>(std::lround(0.20 / resolution_)));
  std::vector<Point> points;
  for (int dy = -plane_radius; dy <= plane_radius; ++dy) {
    for (int dx = -plane_radius; dx <= plane_radius; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      const auto & support = ground_[index(x + dx, y + dy)];
      if (!support) {continue;}
      points.push_back(support->low);
      if (support->high[2] > support->low[2]) {points.push_back(support->high);}
    }
  }
  const auto plane = robustPlane(points, config_.floor_plane_band, config_.max_ground_slope);
  const auto & own_support = ground_[idx];
  if (!plane || !own_support) {
    floor_planes_.emplace(idx, std::nullopt);
    return std::nullopt;
  }
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
        {
          floor_planes_.emplace(idx, std::nullopt);
          return std::nullopt;
        }
      }
    }
  }
  floor_planes_.emplace(idx, plane);
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
  const double tol = std::min(band, ceiling->tol) + 1e-9;
  const auto on_board = [&](const Point & p) {
      return std::abs(p[2] - ceiling->at(p[0], p[1])) <= tol;
    };
  // The lowest return must lie on the fitted underside board; higher returns in the same
  // voxel (e.g. vertical beam front-face points above the underside) are strictly safer.
  if (!on_board(evidence.low) ||
    evidence.high[2] < ceiling->at(evidence.high[0], evidence.high[1]) - tol)
  {return std::nullopt;}
  const auto floor = floorPlane(x, y);
  if (!floor) {return std::nullopt;}
  const double cx = 0.5 * (evidence.xy_bounds[0] + evidence.xy_bounds[2]);
  const double cy = 0.5 * (evidence.xy_bounds[1] + evidence.xy_bounds[3]);
  return ceiling->at(cx, cy) - floor->at(cx, cy);
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

bool GroundColumnGrid::isDropOffStep(int x, int y) const
{
  if (!valid(x, y)) {return false;}
  const auto i = index(x, y);
  const bool has_ground = ground_[i].has_value();
  const auto & bins = columns_[i];
  if (!has_ground && bins.empty()) {return false;}
  const Point & highest = has_ground ? ground_[i]->high : bins.rbegin()->second.high;
  // For connected ground adjacent to an elevated deck connected via a separate ramp elsewhere
  // (e.g. a 0.18 m bridge drop-off), only check immediate 1-cell neighbors so continuous ramps
  // (<= 3-4 cm per 5 cm cell) are never affected while sheer bridge cliffs are sealed cleanly.
  const int r_cells = has_ground
    ? 1
    : std::max(1, std::min(3, static_cast<int>(std::lround(0.15 / resolution_))));
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if ((dx == 0 && dy == 0) || !valid(x + dx, y + dy)) {continue;}
      const auto nidx = index(x + dx, y + dy);
      const auto & ng = ground_[nidx];
      if (!ng) {continue;}
      const double horiz = std::hypot(highest[0] - ng->low[0], highest[1] - ng->low[1]);
      const double drop = ng->low[2] - highest[2];
      const double threshold = std::max(
        config_.surface_max + 0.02,
        config_.max_ground_slope * horiz + config_.support_noise + 0.02);
      if (drop > threshold) {
        // If the neighbor column also contains a surface bin continuous with highest[2]
        // (e.g. a passable ramp continuing underneath a 2-level overpass deck), this is an
        // overpass entrance rather than a sheer drop-off cliff.
        bool neighbor_has_wall_or_ramp = false;
        for (const auto & nbin : columns_[nidx]) {
          const double nhoriz = std::hypot(
            highest[0] - nbin.second.low[0], highest[1] - nbin.second.low[1]);
          const double nlimit = std::max(
            config_.surface_max,
            config_.max_ground_slope * nhoriz + config_.support_noise + 1e-6);
          if (std::abs(nbin.second.low[2] - highest[2]) <= nlimit) {
            neighbor_has_wall_or_ramp = true;
            break;
          }
          // When this lower cell is already connected ground and the adjacent higher cell
          // already contains vertical wall returns below its top, that neighbor cell itself
          // is already occupied; do not inflate the wall 5 cm inward onto the tunnel floor.
          if (has_ground && nbin.second.low[2] < ng->low[2] - 0.04) {
            neighbor_has_wall_or_ramp = true;
            break;
          }
        }
        if (neighbor_has_wall_or_ramp) {continue;}
        return true;
      }
    }
  }
  return false;
}

bool GroundColumnGrid::isRoughBump(int x, int y) const
{
  if (!valid(x, y) || hasOverhead(x, y)) {return false;}
  const auto & gc = ground_[index(x, y)];
  if (!gc) {return false;}
  // Detect high-frequency non-monotonic ground undulations (e.g. cobblestone/speed-bump
  // obstacle zones taller than wheel climb clearance) using exact distance-weighted
  // second difference across nearest observed neighbors within 5..15 cm on each side.
  // Flat floors, constant-slope ramps (even with unevenly spaced missing cells), and
  // monotonic ramp creases produce <= 0 product or zero residual and are unaffected.
  const int max_s = std::max(2, std::min(4, static_cast<int>(std::lround(0.15 / resolution_))));
  const double z0 = gc->low[2];
  const auto sampleNearbyZ = [&](int nx, int ny, double & out_z) -> bool {
      if (!valid(nx, ny)) {return false;}
      const auto idx = index(nx, ny);
      if (ground_[idx]) {
        out_z = ground_[idx]->low[2];
        return true;
      }
      const auto & col = columns_[idx];
      if (!col.empty()) {
        const double z_cand = col.begin()->second.low[2];
        if (std::abs(z_cand - z0) <= 0.10) {
          out_z = z_cand;
          return true;
        }
      }
      return false;
    };
  const auto checkAxis = [&](int ax, int ay) -> bool {
      double zm = 0.0, zp = 0.0;
      int dm_cells = 0, dp_cells = 0;
      const int order[3] = {std::min(2, max_s), max_s, 1};
      for (int s : order) {
        if (sampleNearbyZ(x - ax * s, y - ay * s, zm)) {dm_cells = s; break;}
      }
      for (int s : order) {
        if (sampleNearbyZ(x + ax * s, y + ay * s, zp)) {dp_cells = s; break;}
      }
      if (dm_cells == 0 || dp_cells == 0) {return false;}
      const double dm = z0 - zm;
      const double dp = z0 - zp;
      if (dm * dp <= 0.0 || std::min(std::abs(dm), std::abs(dp)) <= 0.018) {return false;}
      const double z_interp =
        (static_cast<double>(dp_cells) * zm + static_cast<double>(dm_cells) * zp) /
        static_cast<double>(dm_cells + dp_cells);
      return std::abs(z0 - z_interp) > 0.036;
    };
  return checkAxis(1, 0) || checkAxis(0, 1);
}

ColumnState GroundColumnGrid::rawState(int x, int y) const
{
  if (!valid(x, y)) {return ColumnState::kUnknown;}
  const auto cell_idx = index(x, y);
  const auto cached = raw_state_cache_.find(cell_idx);
  if (cached != raw_state_cache_.end()) {return cached->second;}
  const ColumnState computed = [&]() -> ColumnState {
  if (isDropOffStep(x, y) || isRoughBump(x, y)) {return ColumnState::kOccupied;}
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
        // Only returns near the robot headroom threshold require 9x9 RANSAC ceiling/floor
        // plane fitting; returns clearly below are guaranteed walls and returns clearly above
        // are guaranteed high overhangs/roofs.
        const double plane_slack =
          config_.ceiling_plane_band + config_.floor_plane_band + 0.03;
        const double raw_headroom = evidence.low[2] - support;
        const double lower_headroom = evidence.low[2] - lower;
        if (raw_headroom < clearance - plane_slack &&
          (interval->planar || lower_headroom < clearance - plane_slack))
        {
          return ColumnState::kOccupied;
        }
        if (raw_headroom < clearance + plane_slack) {
          if (!plane_done) {plane = ceilingPlane(x, y); plane_done = true;}
          low_z = snapToPlane(plane, evidence.low, config_.ceiling_plane_band);
          high_z = snapToPlane(plane, evidence.high, config_.ceiling_plane_band);
          double headroom = low_z - support;
          const double board_min_clearance =
            config_.robot_height + std::max(0.0, config_.top_margin - config_.support_noise);
          if (const auto board = boardClearance(x, y, evidence, plane)) {
            headroom = std::max(headroom, *board);
            if (headroom >= board_min_clearance - 1e-9 && headroom < clearance) {
              headroom = clearance;
            }
            // The board is one surface: the heights below describe it at this clearance.
            low_z = support + headroom;
            high_z = std::max(high_z, low_z);
          } else if (plane &&
            std::abs(evidence.low[2] - plane->at(evidence.low[0], evidence.low[1])) <=
              std::min(config_.ceiling_plane_band,
                std::max(plane->tol, config_.support_noise + 0.003)) + 1e-9 &&
            headroom >= board_min_clearance - 1e-9 && headroom < clearance)
          {
            // When the underside return snaps to a verified horizontal ceiling plane (even
            // near a tunnel mouth where floorPlane is incomplete or LIO Z jitters 4-8 mm),
            // absorb <= support_noise jitter while strictly keeping >= 0.244 m (> 0.242 m).
            headroom = clearance;
            low_z = support + headroom;
            high_z = std::max(high_z, low_z);
          }
          if (headroom < clearance - 1e-9) {
            // When no local floor plane is fitted yet (e.g. viewing a passable 0.25 m tunnel
            // beam front edge from > 2 m away with sparse floor samples), non-planar Lipschitz
            // slope-cone inflation can push support up by ~1.3 cm. Likewise, before a horizontal
            // ceiling plane is observed across the underside, 1-5 mm single-point LiDAR/LIO
            // noise on a vertical beam front face can dip a true 0.250 m beam just below 0.249 m.
            // Keep such borderline unverified edges kUnknown (never kFree) until closer scans
            // establish both floor and ceiling planes for the exact 0.249 m headroom gate.
            if ((!interval->planar && (low_z - lower >= clearance - 1e-9)) ||
              (!plane && headroom >= clearance - config_.support_noise - 1e-9))
            {
              return ColumnState::kUnknown;
            }
            return ColumnState::kOccupied;
          }
        }
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
  }();
  raw_state_cache_.emplace(cell_idx, computed);
  return computed;
}

ColumnState GroundColumnGrid::state(int x, int y) const
{
  const ColumnState raw = rawState(x, y);
  if (raw == ColumnState::kTunnelFree && config_.tunnel_free_enabled &&
    config_.tunnel_free_confirm_frames > 1)
  {
    if (tunnel_free_observations_[index(x, y)] <
      static_cast<unsigned int>(config_.tunnel_free_confirm_frames))
    {
      return ColumnState::kFree;
    }
  }
  return raw;
}


}  // namespace ats_rog_map
