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
  const std::optional<double> ground_z, const GroundColumnConfig & config)
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
  const double span = heights.back() - heights.front();
  const double ratio = span > 1e-9 ? std::clamp(
    (heights.size() - 1) * config.voxel_resolution / span, 0.0, 1.0) : 1.0;
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

void GroundColumnGrid::observe(
  const std::vector<std::array<double, 3>> & points,
  const std::array<double, 3> & robot_ground)
{
  for (const auto & p : points) {
    if (!std::all_of(p.begin(), p.end(), [](double v) {return std::isfinite(v);})) {continue;}
    const int x = static_cast<int>(std::floor((p[0] - origin_x_) / resolution_));
    const int y = static_cast<int>(std::floor((p[1] - origin_y_) / resolution_));
    if (!valid(x, y)) {++out_of_range_; continue;}
    auto & column = columns_[index(x, y)];
    const int bin = static_cast<int>(std::floor(p[2] / config_.voxel_resolution));
    const auto inserted = column.emplace(bin, VoxelReturns{p, p, {p[0], p[1], p[0], p[1]}});
    if (!inserted.second) {
      auto & evidence = inserted.first->second;
      if (p[2] < evidence.low[2]) {evidence.low = p;}
      if (p[2] > evidence.high[2]) {evidence.high = p;}
      evidence.xy_bounds[0] = std::min(evidence.xy_bounds[0], p[0]);
      evidence.xy_bounds[1] = std::min(evidence.xy_bounds[1], p[1]);
      evidence.xy_bounds[2] = std::max(evidence.xy_bounds[2], p[0]);
      evidence.xy_bounds[3] = std::max(evidence.xy_bounds[3], p[1]);
    }
  }
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
        else {
          // Do not turn downward jitter into extra headroom. Keep the observed
          // support interval until ray contradiction or an epoch reset.
          auto & support = *ground_[i];
          if (candidate.low[2] < support.low[2]) {support.low = candidate.low;}
          if (candidate.high[2] > support.high[2]) {support.high = candidate.high;}
          for (int a = 0; a < 2; ++a) {
            support.xy_bounds[a] = std::min(support.xy_bounds[a], candidate.xy_bounds[a]);
            support.xy_bounds[a + 2] = std::max(support.xy_bounds[a + 2], candidate.xy_bounds[a + 2]);
          }
        }
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
        // Connectivity is surface identity, not a headroom bound: two returns a
        // few mm apart across a cell edge must not split on LIO noise.
        if (std::abs(candidate.low[2] - support[2]) <= config_.max_ground_slope * std::max(
            resolution_ * std::hypot(dx, dy),
            std::hypot(candidate.low[0] - support[0], candidate.low[1] - support[1])) + 1e-6)
        {ground_[i] = candidate; queue.emplace(x + dx, y + dy);}
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

std::optional<std::array<double, 2>> GroundColumnGrid::supportBounds(
  int x, int y, double query_x, double query_y, double * plane_z) const
{
  if (plane_z) {*plane_z = std::numeric_limits<double>::quiet_NaN();}
  std::vector<VoxelReturns> supports;
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      const auto & p = ground_[index(x + dx, y + dy)];
      if (!p) {continue;}
      supports.push_back(*p);
      lower = std::max(lower, p->low[2] - config_.max_ground_slope *
        std::hypot(query_x - p->low[0], query_y - p->low[1]));
      upper = std::min(upper, p->high[2] + config_.max_ground_slope *
        std::hypot(query_x - p->high[0], query_y - p->high[1]));
    }
  }
  if (supports.empty()) {return std::nullopt;}
  // Fit only observed connected supports, not arbitrary low points or a
  // diagnostic/interpolated floor. Reject steps, rank deficiency and extrapolation.
  if (supports.size() >= 6) {
    Point mean{};
    for (const auto & support : supports) {
      for (int a = 0; a < 3; ++a) {mean[a] += support.high[a] / supports.size();}
    }
    double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
    std::vector<double> angles;
    for (const auto & support : supports) {
      // The plane follows the highest measured support return: a later lower
      // return must not turn into extra headroom (appendix A.1), so the gate
      // never sits below a floor height that was actually measured.
      const auto & p = support.high;
      const double u = p[0] - mean[0], v = p[1] - mean[1], w = p[2] - mean[2];
      xx += u * u; xy += u * v; yy += v * v; xz += u * w; yz += v * w;
      angles.push_back(std::atan2(p[1] - query_y, p[0] - query_x));
    }
    const double determinant = xx * yy - xy * xy;
    if (determinant > 1e-8 * (xx + yy) * (xx + yy)) {
      const double a = (xz * yy - yz * xy) / determinant;
      const double b = (yz * xx - xz * xy) / determinant;
      double residual = 0;
      for (const auto & support : supports) {
        // Enclose the whole measured support interval, not only a historical
        // minimum or the two z-extreme points on a sloping surface.
        for (const double px : {support.xy_bounds[0], support.xy_bounds[2]}) {
          for (const double py : {support.xy_bounds[1], support.xy_bounds[3]}) {
            const double fitted = mean[2] + a * (px - mean[0]) + b * (py - mean[1]);
            residual = std::max({residual, std::abs(support.low[2] - fitted),
              std::abs(support.high[2] - fitted)});
          }
        }
      }
      std::sort(angles.begin(), angles.end());
      constexpr double pi = 3.14159265358979323846;
      double gap = angles.front() + 2 * pi - angles.back();
      for (std::size_t i = 1; i < angles.size(); ++i) {
        gap = std::max(gap, angles[i] - angles[i - 1]);
      }
      if (std::hypot(a, b) <= config_.max_ground_slope && residual <= config_.plane_residual_max &&
        gap <= pi + 1e-9)
      {
        const double z = mean[2] + a * (query_x - mean[0]) + b * (query_y - mean[1]);
        if (plane_z) {*plane_z = z;}
        return std::array<double, 2>{z - residual, z + residual};
      }
    }
  }
  // No valid planar support: retain the full Lipschitz uncertainty. An
  // inconsistent neighbourhood is unknown, never repaired by selecting a minimum.
  // Returns a few mm apart carry measurement noise beyond the slope bound.
  // Up to support_noise the interval spans both ends; beyond it is unknown.
  if (lower > upper + config_.support_noise + 1e-9) {return std::nullopt;}
  return std::array<double, 2>{std::min(lower, upper), std::max(lower, upper)};
}

std::optional<GroundColumnGrid::SupportInterval> GroundColumnGrid::voxelSupportInterval(
  int x, int y, const VoxelReturns & evidence) const
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
  if (all_planar && std::isfinite(centre_plane)) {
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
  return robustPlane(points, config_.floor_plane_band, config_.max_ground_slope);
}

double GroundColumnGrid::snapToPlane(
  const std::optional<CeilingPlane> & plane, const std::array<double, 3> & p, double band)
{
  if (!plane) {return p[2];}
  const double z = plane->at(p[0], p[1]);
  return std::abs(p[2] - z) <= std::min(band, plane->tol) + 1e-9 ? z : p[2];
}

std::optional<double> GroundColumnGrid::boardClearance(
  int x, int y, const VoxelReturns & evidence, const std::optional<CeilingPlane> & ceiling) const
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
    const auto interval = voxelSupportInterval(x, y, bin.second);
    if (!interval || bin.second.high[2] - interval->lower <= config_.surface_max + 1e-9) {continue;}
    if (!plane_done) {plane = ceilingPlane(x, y); plane_done = true;}
    const double low = snapToPlane(plane, bin.second.low, config_.ceiling_plane_band);
    double headroom = low - interval->estimate;
    if (const auto board = boardClearance(x, y, bin.second, plane)) {
      headroom = std::max(headroom, *board);
    }
    if (!result || headroom < result->headroom) {
      result = ColumnHeadroom{low, interval->lower, interval->upper,
        low - headroom, headroom};
    }
  }
  return result;
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
    const auto interval = voxelSupportInterval(x, y, bin.second);
    if (!interval) {return ColumnState::kUnknown;}
    const double lower = interval->lower;
    const double support = interval->estimate;
    double low_z = bin.second.low[2];
    double high_z = bin.second.high[2];
    if (bin.second.high[2] - lower > config_.surface_max + 1e-9) {
      if (!plane_done) {plane = ceilingPlane(x, y); plane_done = true;}
      low_z = snapToPlane(plane, bin.second.low, config_.ceiling_plane_band);
      high_z = snapToPlane(plane, bin.second.high, config_.ceiling_plane_band);
      double headroom = low_z - support;
      if (const auto board = boardClearance(x, y, bin.second, plane)) {
        headroom = std::max(headroom, *board);
        // The board is one surface: the heights below describe it at this clearance.
        low_z = support + headroom;
        high_z = std::max(high_z, low_z);
      }
      if (headroom < clearance - 1e-9) {return ColumnState::kOccupied;}
      ceiling = true;
    }
    // Without its own connected floor, a return below the neighbourhood
    // support is an unexplained lower surface (a drop), not floor.
    if (!ground_[index(x, y)] && bin.second.low[2] < lower - config_.surface_max - 1e-9) {
      return ColumnState::kUnknown;
    }
    heights.push_back(low_z - support);
    heights.push_back(high_z - support);
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
  return classifyGroundColumn(heights,
    observed, 0.0, config_);
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
    (static_cast<int>(std::floor(floor->low[2] / config_.voxel_resolution)) == found->first ||
    static_cast<int>(std::floor(floor->high[2] / config_.voxel_resolution)) == found->first);
  if (removed_support) {floor.reset();}
  column.erase(found);
  return removed_support;
}

void GroundColumnGrid::observeRayCell(int x, int y)
{
  if (valid(x, y)) {ray_observed_[index(x, y)] = true;}
}

void GroundColumnGrid::clear()
{
  for (auto & c : columns_) {c.clear();}
  std::fill(ground_.begin(), ground_.end(), std::nullopt);
  std::fill(ray_observed_.begin(), ray_observed_.end(), false);
}
}  // namespace ats_rog_map
