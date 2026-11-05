// Copyright 2026

#include "ats_rog_map/ground_column.hpp"
#include <algorithm>
#include <cmath>
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
    columns_[index(x, y)][static_cast<int>(std::floor(p[2] / config_.voxel_resolution))] = p[2];
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
      const double z = columns_[i].begin()->second;
      // Slope is checked between connected neighbors below. A distance-based
      // seed tolerance would let a nearby hanging plate invent its own floor.
      if (ground_[i] || (distance <= config_.ground_seed_radius &&
        std::abs(z - robot_ground[2]) <= 0.05))
      {
        if (!ground_[i]) {ground_[i] = z;}
        queue.emplace(x, y);
      }
    }
  }
  while (!queue.empty()) {
    const auto [x, y] = queue.front();
    queue.pop();
    const double z = *ground_[index(x, y)];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || !valid(x + dx, y + dy)) {continue;}
        const auto i = index(x + dx, y + dy);
        if (ground_[i] || columns_[i].empty()) {continue;}
        const double candidate = columns_[i].begin()->second;
        if (std::abs(candidate - z) <= config_.max_ground_slope *
          resolution_ * std::hypot(dx, dy) + 1e-6)
        {ground_[i] = candidate; queue.emplace(x + dx, y + dy);}
      }
    }
  }
}

std::optional<double> GroundColumnGrid::ground(int x, int y) const
{
  if (!valid(x, y)) {return std::nullopt;}
  auto result = ground_[index(x, y)];
  // A wall may have no floor return in its own column. Use the lowest
  // connected surface in its 3x3 neighborhood, as specified in appendix A.1.
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (!valid(x + dx, y + dy)) {continue;}
      const auto z = ground_[index(x + dx, y + dy)];
      if (z && (!result || *z < *result)) {result = z;}
    }
  }
  return result;
}

ColumnState GroundColumnGrid::state(int x, int y) const
{
  if (!valid(x, y)) {return ColumnState::kUnknown;}
  std::vector<double> heights;
  for (const auto & bin : columns_[index(x, y)]) {heights.push_back(bin.second);}
  return classifyGroundColumn(heights,
    std::max(static_cast<int>(heights.size()), ray_observed_[index(x, y)] ? 1 : 0),
    ground(x, y), config_);
}

void GroundColumnGrid::clearVoxel(const std::array<double, 3> & point)
{
  clearVoxel(point, -std::numeric_limits<double>::infinity());
}

void GroundColumnGrid::clearVoxel(const std::array<double, 3> & point, double ray_z)
{
  const int x = static_cast<int>(std::floor((point[0] - origin_x_) / resolution_));
  const int y = static_cast<int>(std::floor((point[1] - origin_y_) / resolution_));
  if (!valid(x, y)) {return;}
  auto & column = columns_[index(x, y)];
  const auto found = column.find(static_cast<int>(std::floor(point[2] / config_.voxel_resolution)));
  if (found == column.end()) {return;}
  // A ray passing above the stored return does not contradict it: a grazing ray
  // inside the same voxel would otherwise erase a horizontal surface (a platform
  // edge) and leave the column empty and ray-observed, i.e. falsely free. Only a
  // ray at or below the stored height shows that return is gone.
  if (std::isfinite(ray_z) && ray_z > found->second + 1e-9) {return;}
  column.erase(found);
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
