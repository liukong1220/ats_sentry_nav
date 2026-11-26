// Copyright 2026

#include "ats_rog_map/odom_layer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace ats_rog_map
{
namespace
{
constexpr double kEpsilon = 1e-9;
}

OdomLayerStore::OdomLayerStore(const OdomLayerConfig & config)
: config_(config),
  cells_(static_cast<std::size_t>(config.width) * config.height),
  dynamic_occupied_tracked_(static_cast<std::size_t>(config.width) * config.height, 0U)
{
  config_.resolution = std::max(config_.resolution, kEpsilon);
  config_.static_promotion_seconds = std::max(config_.static_promotion_seconds, 0.0);
  config_.dynamic_hold_seconds = std::max(config_.dynamic_hold_seconds, 0.0);
  config_.max_frame_dt_seconds = std::max(config_.max_frame_dt_seconds, 0.0);
  config_.cloud_resolution = std::max(config_.cloud_resolution, kEpsilon);
  config_.cloud_clear_ray_radius = std::max(config_.cloud_clear_ray_radius, 0.0);
  config_.cloud_clear_endpoint_margin = std::max(config_.cloud_clear_endpoint_margin, 0.0);
}

bool OdomLayerStore::validCell(const int x, const int y) const
{
  return x >= 0 && y >= 0 && x < static_cast<int>(config_.width) &&
         y < static_cast<int>(config_.height);
}

std::size_t OdomLayerStore::index(const int x, const int y) const
{
  return static_cast<std::size_t>(y) * config_.width + static_cast<std::size_t>(x);
}

OdomLayerCell * OdomLayerStore::cell(const int x, const int y)
{
  return validCell(x, y) ? &cells_[index(x, y)] : nullptr;
}

const OdomLayerCell * OdomLayerStore::cell(const int x, const int y) const
{
  return validCell(x, y) ? &cells_[index(x, y)] : nullptr;
}

bool OdomLayerStore::observeHit(
  const int x, const int y, const double stamp, const double observed_dt)
{
  auto * value = cell(x, y);
  if (value == nullptr || !std::isfinite(stamp)) {
    return false;
  }
  // A cloud may contain many returns in the same cell. Count a timestamp only
  // once, otherwise point density rather than observation time promotes it.
  if (value->state == OdomCellState::kOccupied && stamp <= value->last_hit_time) {
    return false;
  }
  const bool was_occupied = value->state == OdomCellState::kOccupied;
  const double raw_dt = (was_occupied && value->last_hit_time > 0.0)
    ? (stamp - value->last_hit_time)
    : observed_dt;
  const double bounded_dt = std::clamp(raw_dt, 0.0, config_.max_frame_dt_seconds);
  if (!was_occupied) {
    value->first_hit_time = stamp;
    value->hit_time_accum = 0.0;
    // 持久比按本次占据期计算：之前的 free 观测时间不应稀释新出现的障碍。
    value->observed_time = 0.0;
    value->layer = OdomCellLayer::kDynamic;
  }
  value->hit_time_accum += bounded_dt;
  value->last_hit_time = stamp;
  value->last_visible_time = stamp;
  value->observed_time += bounded_dt;
  value->state = OdomCellState::kOccupied;
  value->tunnel_free_count = 0U;
  const double required_promotion = value->cleared_once
    ? std::max(config_.static_promotion_seconds, 1.5 * config_.static_promotion_seconds)
    : config_.static_promotion_seconds;
  if (value->layer == OdomCellLayer::kDynamic && value->hit_time_accum >= required_promotion) {
    value->layer = OdomCellLayer::kStatic;
    value->cleared_once = false;
  } else if (value->layer == OdomCellLayer::kDynamic && value->state == OdomCellState::kOccupied) {
    const auto idx = index(x, y);
    if (!dynamic_occupied_tracked_[idx]) {
      dynamic_occupied_tracked_[idx] = 1U;
      dynamic_occupied_indices_.push_back(idx);
    }
  }
  return true;
}

bool OdomLayerStore::observeVisible(const int x, const int y, const double stamp)
{
  auto * value = cell(x, y);
  if (value == nullptr || !std::isfinite(stamp) || stamp < value->last_visible_time) {
    return false;
  }
  value->last_visible_time = stamp;
  return true;
}

bool OdomLayerStore::invalidateFree(const int x, const int y, const double stamp)
{
  auto * value = cell(x, y);
  if (value == nullptr || !std::isfinite(stamp) || stamp < value->last_visible_time ||
    value->footprint_traversed ||
    (value->state != OdomCellState::kFree && value->state != OdomCellState::kTunnelFree))
  {return false;}
  value->state = OdomCellState::kUnknown;
  value->tunnel_free_count = 0U;
  value->last_visible_time = stamp;
  return true;
}

bool OdomLayerStore::observePoint(
  const std::array<double, 3> & point, const double stamp, const double observed_dt)
{
  if (!std::isfinite(stamp) || !std::all_of(point.begin(), point.end(),
    [](double value) {return std::isfinite(value);})) {return false;}
  const int x = static_cast<int>(std::floor((point[0] - config_.origin_x) / config_.resolution));
  const int y = static_cast<int>(std::floor((point[1] - config_.origin_y) / config_.resolution));
  if (!validCell(x, y)) {return false;}
  const std::array<int, 3> key{
    static_cast<int>(std::floor(point[0] / config_.cloud_resolution)),
    static_cast<int>(std::floor(point[1] / config_.cloud_resolution)),
    static_cast<int>(std::floor(point[2] / config_.cloud_resolution))};
  auto & voxel = cloud_voxels_[key];
  auto & evidence = voxel.evidence;
  const bool was_occupied = evidence.state == OdomCellState::kOccupied;
  if (was_occupied && stamp <= evidence.last_hit_time) {
    return false;
  }
  if (!was_occupied) {
    evidence.first_hit_time = stamp;
    evidence.hit_time_accum = 0.0;
    evidence.observed_time = 0.0;
    evidence.layer = OdomCellLayer::kDynamic;
  }
  voxel.point = point;
  evidence.state = OdomCellState::kOccupied;
  const double interval = (was_occupied && evidence.last_hit_time > 0.0) ?
    (stamp - evidence.last_hit_time) : observed_dt;
  evidence.last_hit_time = stamp;
  if (voxel.candidate_index == std::numeric_limits<std::size_t>::max()) {
    ray_candidates_.push_back({key, stamp});
    voxel.candidate_index = ray_candidates_.size() - 1;
  } else {
    ray_candidates_[voxel.candidate_index].last_hit_time = stamp;
  }
  const double bounded_dt = std::clamp(interval, 0.0, config_.max_frame_dt_seconds);
  evidence.hit_time_accum += bounded_dt;
  evidence.observed_time = evidence.hit_time_accum;
  const double required_promotion = evidence.cleared_once
    ? std::max(config_.static_promotion_seconds, 1.5 * config_.static_promotion_seconds)
    : config_.static_promotion_seconds;
  if (evidence.layer == OdomCellLayer::kDynamic && evidence.hit_time_accum >= required_promotion) {
    evidence.layer = OdomCellLayer::kStatic;
    evidence.cleared_once = false;
  }
  return true;
}

void OdomLayerStore::clearCloudVoxel(const std::array<double, 3> & point, double stamp)
{
  const std::array<int, 3> key{
    static_cast<int>(std::floor(point[0] / config_.cloud_resolution)),
    static_cast<int>(std::floor(point[1] / config_.cloud_resolution)),
    static_cast<int>(std::floor(point[2] / config_.cloud_resolution))};
  const auto found = cloud_voxels_.find(key);
  if (found == cloud_voxels_.end() || found->second.evidence.last_hit_time >= stamp) {return;}
  auto & e = found->second.evidence;
  e.cleared_once = true;
  e.state = OdomCellState::kUnknown;
  e.layer = OdomCellLayer::kDynamic;
  e.hit_time_accum = 0.0;
  e.observed_time = 0.0;
  removeRayClearCandidate(found->second);
}

void OdomLayerStore::removeRayClearCandidate(CloudVoxel & voxel)
{
  const auto i = voxel.candidate_index;
  if (i == std::numeric_limits<std::size_t>::max()) {return;}
  if (i + 1 < ray_candidates_.size()) {
    ray_candidates_[i] = ray_candidates_.back();
    cloud_voxels_.at(ray_candidates_[i].key).candidate_index = i;
  }
  ray_candidates_.pop_back();
  voxel.candidate_index = std::numeric_limits<std::size_t>::max();
}

std::vector<std::array<int, 3>> OdomLayerStore::rayClearCandidates(const double stamp) const
{
  std::vector<std::array<int, 3>> result;
  result.reserve(ray_candidates_.size());
  for (const auto & candidate : ray_candidates_) {
    if (candidate.last_hit_time < stamp) {
      result.push_back(candidate.key);
    }
  }
  return result;
}

bool OdomLayerStore::hasRayClearCandidate(
  const std::array<double, 3> & location, const double stamp) const
{
  const std::array<int, 3> key{
    static_cast<int>(std::floor(location[0] / config_.cloud_resolution)),
    static_cast<int>(std::floor(location[1] / config_.cloud_resolution)),
    static_cast<int>(std::floor(location[2] / config_.cloud_resolution))};
  const auto found = cloud_voxels_.find(key);
  return found != cloud_voxels_.end() &&
         found->second.evidence.state == OdomCellState::kOccupied &&
         found->second.evidence.last_hit_time < stamp;
}

void OdomLayerStore::clearCloudVoxelAlongRay(
  const std::array<double, 3> & location, const std::array<double, 3> & sensor,
  const std::array<double, 3> & endpoint, const double stamp)
{
  const std::array<int, 3> key{
    static_cast<int>(std::floor(location[0] / config_.cloud_resolution)),
    static_cast<int>(std::floor(location[1] / config_.cloud_resolution)),
    static_cast<int>(std::floor(location[2] / config_.cloud_resolution))};
  const auto found = cloud_voxels_.find(key);
  if (found == cloud_voxels_.end() ||
    found->second.evidence.state != OdomCellState::kOccupied ||
    found->second.evidence.last_hit_time >= stamp) {return;}
  const auto & point = found->second.point;
  double norm = 0.0, projection = 0.0, distance_squared = 0.0;
  for (int a = 0; a < 3; ++a) {
    const double d = endpoint[a] - sensor[a], v = point[a] - sensor[a];
    norm += d * d;
    projection += v * d;
    distance_squared += v * v;
  }
  if (norm <= kEpsilon || projection <= 0.0) {return;}
  const double length = std::sqrt(norm), along = projection / length;
  if (along >= length - config_.cloud_clear_endpoint_margin) {return;}
  const double perpendicular_squared = std::max(0.0, distance_squared - along * along);
  if (perpendicular_squared > config_.cloud_clear_ray_radius * config_.cloud_clear_ray_radius) {
    return;
  }
  auto & evidence = found->second.evidence;
  evidence.cleared_once = true;
  evidence.state = OdomCellState::kUnknown;
  evidence.layer = OdomCellLayer::kDynamic;
  evidence.hit_time_accum = 0.0;
  evidence.observed_time = 0.0;
  removeRayClearCandidate(found->second);
}

bool OdomLayerStore::observeFree(int x, int y, double stamp, double observed_dt)
{
  auto * value = cell(x, y);
  if (!value || !std::isfinite(stamp) || stamp <= value->last_visible_time) {return false;}
  const bool cleared = observeRayClear(x, y, stamp, true);
  value->last_visible_time = stamp;
  value->observed_time += std::clamp(observed_dt, 0.0, config_.max_frame_dt_seconds);
  if (value->state != OdomCellState::kTunnelFree) {
    value->state = OdomCellState::kFree;
  }
  return cleared;
}

bool OdomLayerStore::observeTunnelFree(int x, int y, double stamp, double observed_dt)
{
  auto * value = cell(x, y);
  if (!value || !std::isfinite(stamp) || stamp <= value->last_visible_time) {return false;}
  const bool cleared = observeRayClear(x, y, stamp, true);
  value->last_visible_time = stamp;
  value->observed_time += std::clamp(observed_dt, 0.0, config_.max_frame_dt_seconds);
  if (config_.tunnel_free_enabled) {
    if (value->tunnel_free_count < 65535U) {
      ++value->tunnel_free_count;
    }
    const int required = std::max(1, config_.tunnel_free_confirm_frames);
    value->state = (static_cast<int>(value->tunnel_free_count) >= required) ?
      OdomCellState::kTunnelFree : OdomCellState::kFree;
  } else {
    value->state = OdomCellState::kFree;
  }
  return cleared;
}

void OdomLayerStore::markUnknownFootprintFree(
  const double center_x, const double center_y,
  const double half_x, const double half_y, const double stamp)
{
  if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
    !std::isfinite(half_x) || !std::isfinite(half_y) || !std::isfinite(stamp) ||
    half_x <= 0.0 || half_y <= 0.0)
  {
    return;
  }
  const int x0 = std::max(
    0, static_cast<int>(std::floor((center_x - half_x - config_.origin_x) / config_.resolution)));
  const int x1 = std::min(
    static_cast<int>(config_.width) - 1,
    static_cast<int>(std::floor((center_x + half_x - config_.origin_x) / config_.resolution)));
  const int y0 = std::max(
    0, static_cast<int>(std::floor((center_y - half_y - config_.origin_y) / config_.resolution)));
  const int y1 = std::min(
    static_cast<int>(config_.height) - 1,
    static_cast<int>(std::floor((center_y + half_y - config_.origin_y) / config_.resolution)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      auto * value = cell(x, y);
      if (value != nullptr && value->state != OdomCellState::kOccupied) {
        value->footprint_traversed = true;
        if (value->state == OdomCellState::kUnknown) {
          value->state = OdomCellState::kFree;
          if (stamp > value->last_visible_time) {
            value->last_visible_time = stamp;
          }
        }
      }
    }
  }
}

std::int8_t OdomLayerStore::sampleFootprint(
  double min_x, double min_y, double max_x, double max_y) const
{
  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
    !std::isfinite(max_y) || max_x <= min_x || max_y <= min_y) {return -1;}
  const auto lattice = [&](double coordinate, double origin) {
      const double value = (coordinate - origin) / config_.resolution;
      if (!std::isfinite(value)) {return value;}
      const double boundary = std::round(value);
      // Only absorb arithmetic / float32 OccupancyGrid::info.resolution roundoff when
      // expressing the same grid edge in world coordinates and back (float32 0.05f drifts
      // ~1.5e-8/cell -> ~1e-5 cells over 800 cells, whereas 1e-4 cells = 5 micrometers).
      const double roundoff = std::max(
        1e-4,
        8 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(coordinate / config_.resolution) +
        std::abs(origin / config_.resolution)));
      return std::abs(value - boundary) <= roundoff ? boundary : value;
    };
  const double x0 = lattice(min_x, config_.origin_x), x1 = lattice(max_x, config_.origin_x);
  const double y0 = lattice(min_y, config_.origin_y), y1 = lattice(max_y, config_.origin_y);
  // Clamp in floating point before converting, including very distant queries.
  const int first_x = static_cast<int>(std::floor(std::clamp(x0, 0.0, double(config_.width))));
  const int first_y = static_cast<int>(std::floor(std::clamp(y0, 0.0, double(config_.height))));
  const int last_x = static_cast<int>(std::ceil(std::clamp(x1, 0.0, double(config_.width)))) - 1;
  const int last_y = static_cast<int>(std::ceil(std::clamp(y1, 0.0, double(config_.height)))) - 1;
  if (first_x > last_x || first_y > last_y) {return -1;}
  bool all_free = x0 >= 0.0 && y0 >= 0.0 && x1 <= config_.width && y1 <= config_.height;
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      const auto state = cell(x, y)->state;
      if (state == OdomCellState::kOccupied) {return 100;}
      all_free = all_free &&
        (state == OdomCellState::kFree || state == OdomCellState::kTunnelFree);
    }
  }
  return all_free ? 0 : -1;
}

bool OdomLayerStore::sampleTunnelFree(
  double min_x, double min_y, double max_x, double max_y) const
{
  if (!config_.tunnel_free_enabled || !std::isfinite(min_x) || !std::isfinite(min_y) ||
    !std::isfinite(max_x) || !std::isfinite(max_y) || max_x <= min_x || max_y <= min_y)
  {return false;}
  const auto lattice = [&](double coordinate, double origin) {
      const double value = (coordinate - origin) / config_.resolution;
      if (!std::isfinite(value)) {return value;}
      const double boundary = std::round(value);
      const double roundoff = std::max(
        1e-4,
        8 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(coordinate / config_.resolution) +
        std::abs(origin / config_.resolution)));
      return std::abs(value - boundary) <= roundoff ? boundary : value;
    };
  const double x0 = lattice(min_x, config_.origin_x), x1 = lattice(max_x, config_.origin_x);
  const double y0 = lattice(min_y, config_.origin_y), y1 = lattice(max_y, config_.origin_y);
  if (x0 < 0.0 || y0 < 0.0 || x1 > config_.width || y1 > config_.height) {return false;}
  const int first_x = static_cast<int>(std::floor(std::clamp(x0, 0.0, double(config_.width))));
  const int first_y = static_cast<int>(std::floor(std::clamp(y0, 0.0, double(config_.height))));
  const int last_x = static_cast<int>(std::ceil(std::clamp(x1, 0.0, double(config_.width)))) - 1;
  const int last_y = static_cast<int>(std::ceil(std::clamp(y1, 0.0, double(config_.height)))) - 1;
  if (first_x > last_x || first_y > last_y) {return false;}
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      if (cell(x, y)->state != OdomCellState::kTunnelFree) {
        return false;
      }
    }
  }
  return true;
}

bool OdomLayerStore::observeRayClear(
  const int x, const int y, const double stamp, const bool visible)
{
  auto * value = cell(x, y);
  if (value == nullptr || !visible || !std::isfinite(stamp)) {
    return false;
  }
  if (value->state != OdomCellState::kOccupied) {
    value->last_visible_time = stamp;
    return false;
  }
  value->state = OdomCellState::kUnknown;
  value->layer = OdomCellLayer::kDynamic;
  value->hit_time_accum = 0.0;
  value->observed_time = 0.0;
  value->cleared_once = true;
  ++value->clear_count;
  value->last_visible_time = stamp;
  return true;
}

void OdomLayerStore::tick(const double stamp)
{
  if (!std::isfinite(stamp)) {
    return;
  }
  for (std::size_t k = 0; k < dynamic_occupied_indices_.size(); ) {
    const std::size_t idx = dynamic_occupied_indices_[k];
    auto & value = cells_[idx];
    if (value.layer != OdomCellLayer::kDynamic || value.state != OdomCellState::kOccupied) {
      dynamic_occupied_tracked_[idx] = 0U;
      dynamic_occupied_indices_[k] = dynamic_occupied_indices_.back();
      dynamic_occupied_indices_.pop_back();
      continue;
    }
    if (value.last_hit_time > 0.0 &&
      stamp - std::max(value.last_hit_time, value.last_visible_time) >
      config_.dynamic_hold_seconds)
    {
      value.state = OdomCellState::kUnknown;
      dynamic_occupied_tracked_[idx] = 0U;
      dynamic_occupied_indices_[k] = dynamic_occupied_indices_.back();
      dynamic_occupied_indices_.pop_back();
      continue;
    }
    ++k;
  }
}

bool OdomLayerStore::rebase(const OdomLayerTransform & old_from_new)
{
  if (!std::isfinite(old_from_new.x) || !std::isfinite(old_from_new.y) ||
    !std::isfinite(old_from_new.yaw))
  {
    clear();
    return false;
  }
  const auto old_cells = cells_;
  clear();
  const double c = std::cos(old_from_new.yaw);
  const double s = std::sin(old_from_new.yaw);
  for (int new_y = 0; new_y < static_cast<int>(config_.height); ++new_y) {
    for (int new_x = 0; new_x < static_cast<int>(config_.width); ++new_x) {
      const double nx = config_.origin_x + (static_cast<double>(new_x) + 0.5) * config_.resolution;
      const double ny = config_.origin_y + (static_cast<double>(new_y) + 0.5) * config_.resolution;
      const double ox = old_from_new.x + c * nx - s * ny;
      const double oy = old_from_new.y + s * nx + c * ny;
      const int old_x = static_cast<int>(std::floor((ox - config_.origin_x) / config_.resolution));
      const int old_y = static_cast<int>(std::floor((oy - config_.origin_y) / config_.resolution));
      if (!validCell(old_x, old_y)) {
        continue;
      }
      const auto & source = old_cells[index(old_x, old_y)];
      auto & destination = cells_[index(new_x, new_y)];
      destination = source;
      if (destination.layer == OdomCellLayer::kDynamic &&
        destination.state == OdomCellState::kOccupied)
      {
        const auto nidx = index(new_x, new_y);
        if (!dynamic_occupied_tracked_[nidx]) {
          dynamic_occupied_tracked_[nidx] = 1U;
          dynamic_occupied_indices_.push_back(nidx);
        }
      }
    }
  }
  return true;
}

void OdomLayerStore::clear()
{
  cells_.assign(cells_.size(), OdomLayerCell{});
  dynamic_occupied_indices_.clear();
  std::fill(dynamic_occupied_tracked_.begin(), dynamic_occupied_tracked_.end(), 0U);
  cloud_voxels_.clear();
  ray_candidates_.clear();
}

bool OdomLayerStore::saveEligible(const int x, const int y) const
{
  const auto * value = cell(x, y);
  if (value == nullptr || value->layer != OdomCellLayer::kStatic ||
    value->state != OdomCellState::kOccupied || value->cleared_once ||
    value->observed_time <= 0.0)
  {
    return false;
  }
  return value->hit_time_accum / value->observed_time >= config_.save_persistence_ratio;
}

bool OdomLayerStore::save(
  const std::string & output_prefix, const std::string & frame_id,
  const OdomLayerTransform & map_from_odom, OdomLayerSaveStats * stats) const
{
  if (output_prefix.empty() || frame_id.empty() || !std::isfinite(map_from_odom.x) ||
    !std::isfinite(map_from_odom.y) || !std::isfinite(map_from_odom.z) ||
    !std::isfinite(map_from_odom.yaw) || !std::isfinite(map_from_odom.roll) ||
    !std::isfinite(map_from_odom.pitch))
  {
    return false;
  }
  const std::filesystem::path prefix(output_prefix);
  if (prefix.has_parent_path()) {
    std::error_code error;
    std::filesystem::create_directories(prefix.parent_path(), error);
    if (error) {
      return false;
    }
  }
  const auto pgm_path = prefix.string() + ".pgm";
  const auto yaml_path = prefix.string() + ".yaml";
  const auto pcd_path = prefix.string() + ".pcd";
  std::ofstream pgm(pgm_path, std::ios::binary);
  std::ofstream yaml(yaml_path);
  OdomLayerSaveStats counts;
  const double c = std::cos(map_from_odom.yaw);
  const double s = std::sin(map_from_odom.yaw);
  const double cr = std::cos(map_from_odom.roll), sr = std::sin(map_from_odom.roll);
  const double cp = std::cos(map_from_odom.pitch), sp = std::sin(map_from_odom.pitch);
  const auto map_point = [&](double x, double y, double z) -> std::array<double, 3> {
      const double ry = cr * y - sr * z, rz = sr * y + cr * z;
      const double px = cp * x + sp * rz, pz = -sp * x + cp * rz;
      return {map_from_odom.x + c * px - s * ry,
        map_from_odom.y + s * px + c * ry, map_from_odom.z + pz};
    };
  const auto transform = [&](double x, double y, double z) -> std::array<float, 3> {
      const auto p = map_point(x, y, z);
      return {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
    };
  std::vector<std::array<float, 3>> points;
  points.reserve(cells_.size() / 10U);
  if (!pgm || !yaml) {
    return false;
  }
  std::vector<unsigned char> pixels(cells_.size(), 205U);
  std::ofstream ineligible(prefix.string() + ".ineligible.csv");
  ineligible << "x,y,reason,hit_time,observed_time,clear_count\n";
  for (std::uint32_t row = 0; row < config_.height; ++row) {
    // PGM row zero is the top row; OccupancyGrid y zero is the bottom row.
    const std::uint32_t y = config_.height - 1U - row;
    for (std::uint32_t x = 0; x < config_.width; ++x) {
      const auto & value = cells_[index(static_cast<int>(x), static_cast<int>(y))];
      if (value.state == OdomCellState::kOccupied) {++counts.occupied_cells;}
      unsigned char pixel = 205U;  // unknown
      const bool eligible = saveEligible(static_cast<int>(x), static_cast<int>(y));
      if (value.state == OdomCellState::kOccupied && !eligible) {
        // Why an occupied cell is left out of the saved raster, so a missing
        // obstacle can be traced to dynamic layer, a past ray clear or low ratio.
        const char * reason = value.cleared_once ? "cleared_once" :
          value.layer != OdomCellLayer::kStatic ? "dynamic" :
          value.observed_time <= 0.0 ? "unobserved" : "low_ratio";
        const auto q = map_point(
          config_.origin_x + (static_cast<double>(x) + 0.5) * config_.resolution,
          config_.origin_y + (static_cast<double>(y) + 0.5) * config_.resolution, 0.0);
        ineligible << q[0] << "," << q[1] << "," << reason << "," << value.hit_time_accum
                   << "," << value.observed_time << "," << value.clear_count << "\n";
        if (reason[0] == 'c') {++counts.occupied_cleared_once;}
        else if (reason[0] == 'd') {++counts.occupied_dynamic;}
        else {++counts.occupied_other;}
      }
      if (eligible) {
        pixel = 0U;
        ++counts.persistent_cells;
        points.push_back(transform(
          config_.origin_x + (static_cast<double>(x) + 0.5) * config_.resolution,
          config_.origin_y + (static_cast<double>(y) + 0.5) * config_.resolution, 0.0));
      } else if (value.state == OdomCellState::kFree || value.state == OdomCellState::kTunnelFree) {
        pixel = 254U;
      }
      pixels[index(x, y)] = pixel;
    }
  }
  {
    std::vector<std::int8_t> save_grid(cells_.size(), -1);
    for (std::size_t i = 0; i < cells_.size(); ++i) {
      if (pixels[i] == 0U) {
        save_grid[i] = 100;
      } else if (pixels[i] == 254U) {
        save_grid[i] = 0;
      }
    }
    const bool changed_speckles =
      filterIsolatedOccupiedSpeckles(save_grid, config_.width, config_.height) > 0U;
    const bool changed_gaps =
      fillIsolatedUnknownGaps(save_grid, config_.width, config_.height) > 0U;
    if (changed_speckles || changed_gaps) {
      for (std::size_t i = 0; i < cells_.size(); ++i) {
        if ((pixels[i] == 205U || pixels[i] == 0U) && save_grid[i] == 0) {
          pixels[i] = 254U;
        }
      }
    }
  }
  auto origin = map_point(config_.origin_x, config_.origin_y, 0.0);
  double grid_yaw = map_from_odom.yaw;
  std::uint32_t width = config_.width, height = config_.height;
  if (std::abs(map_from_odom.roll) > 1e-8 || std::abs(map_from_odom.pitch) > 1e-8) {
    // A tilted grid projects to a parallelogram, which map_server cannot
    // represent with origin/yaw alone. Rasterize positive-area overlap once.
    const auto corners = [&](double x, double y, double size_x, double size_y) {
        return std::array<std::array<double, 3>, 4>{map_point(x, y, 0.0),
          map_point(x + size_x, y, 0.0), map_point(x + size_x, y + size_y, 0.0),
          map_point(x, y + size_y, 0.0)};
      };
    const auto bounds = corners(config_.origin_x, config_.origin_y,
      config_.width * config_.resolution, config_.height * config_.resolution);
    double min_x = bounds[0][0], min_y = bounds[0][1], max_x = min_x, max_y = min_y;
    for (const auto & p : bounds) {
      min_x = std::min(min_x, p[0]); min_y = std::min(min_y, p[1]);
      max_x = std::max(max_x, p[0]); max_y = std::max(max_y, p[1]);
    }
    origin = {std::floor(min_x / config_.resolution) * config_.resolution,
      std::floor(min_y / config_.resolution) * config_.resolution, 0.0};
    width = static_cast<std::uint32_t>(std::ceil((max_x - origin[0]) / config_.resolution));
    height = static_cast<std::uint32_t>(std::ceil((max_y - origin[1]) / config_.resolution));
    if (width == 0 || height == 0) {return false;}
    std::vector<unsigned char> raster(static_cast<std::size_t>(width) * height, 205U);
    const auto overlaps = [&](const auto & polygon, double x, double y) {
        const std::array<std::array<double, 2>, 4> box{{{x, y}, {x + config_.resolution, y},
          {x + config_.resolution, y + config_.resolution}, {x, y + config_.resolution}}};
        for (int a = 0; a < 6; ++a) {
          const double nx = a == 4 ? 1.0 : a == 5 ? 0.0 :
            -(polygon[(a + 1) % 4][1] - polygon[a][1]);
          const double ny = a == 4 ? 0.0 : a == 5 ? 1.0 :
            polygon[(a + 1) % 4][0] - polygon[a][0];
          if (std::hypot(nx, ny) < 1e-12) {continue;}
          double pmin = std::numeric_limits<double>::infinity(), pmax = -pmin;
          double bmin = pmin, bmax = pmax;
          for (int i = 0; i < 4; ++i) {
            const double p = nx * polygon[i][0] + ny * polygon[i][1];
            const double b = nx * box[i][0] + ny * box[i][1];
            pmin = std::min(pmin, p); pmax = std::max(pmax, p);
            bmin = std::min(bmin, b); bmax = std::max(bmax, b);
          }
          if (std::min(pmax, bmax) - std::max(pmin, bmin) <= 1e-10) {return false;}
        }
        return true;
      };
    for (std::uint32_t y = 0; y < config_.height; ++y) {
      for (std::uint32_t x = 0; x < config_.width; ++x) {
        const auto pixel = pixels[index(x, y)];
        if (pixel == 205U) {continue;}
        const auto polygon = corners(config_.origin_x + x * config_.resolution,
          config_.origin_y + y * config_.resolution, config_.resolution, config_.resolution);
        double lx = polygon[0][0], ly = polygon[0][1], hx = lx, hy = ly;
        for (const auto & p : polygon) {
          lx = std::min(lx, p[0]); ly = std::min(ly, p[1]);
          hx = std::max(hx, p[0]); hy = std::max(hy, p[1]);
        }
        for (int my = std::max(0, static_cast<int>(std::floor((ly - origin[1]) / config_.resolution)));
          my < static_cast<int>(height) && origin[1] + my * config_.resolution < hy - 1e-10; ++my)
        {
          for (int mx = std::max(0, static_cast<int>(std::floor((lx - origin[0]) / config_.resolution)));
            mx < static_cast<int>(width) && origin[0] + mx * config_.resolution < hx - 1e-10; ++mx)
          {
            if (!overlaps(polygon, origin[0] + mx * config_.resolution,
              origin[1] + my * config_.resolution)) {continue;}
            auto & value = raster[static_cast<std::size_t>(my) * width + mx];
            if (pixel == 0U || value == 205U) {value = pixel;}
          }
        }
      }
    }
    pixels = std::move(raster);
    grid_yaw = 0.0;
  }
  counts.raster_occupied_cells = std::count(pixels.begin(), pixels.end(), 0U);
  pgm << "P5\n" << width << " " << height << "\n255\n";
  for (std::uint32_t row = 0; row < height; ++row) {
    pgm.write(reinterpret_cast<const char *>(pixels.data() + (height - 1U - row) * width), width);
  }
  yaml << std::fixed << std::setprecision(6)
       << "image: " << std::filesystem::path(pgm_path).filename().string() << "\n"
       << "resolution: " << config_.resolution << "\n"
       << "origin: [" << origin[0] << ", " << origin[1] << ", " << grid_yaw << "]\n"
       << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
       << "frame_id: " << frame_id << "\n"
       << "map_from_odom: [" << map_from_odom.x << ", " << map_from_odom.y << ", "
       << map_from_odom.z << ", " << map_from_odom.yaw << "]\n";
  yaml << "map_from_odom_rpy: [" << map_from_odom.roll << ", " << map_from_odom.pitch
       << ", " << map_from_odom.yaw << "]\n";

  const auto write_pcd = [](const std::string & path,
      const std::vector<std::array<float, 3>> & data) {
      std::ofstream pcd(path);
      if (!pcd) {return false;}
      pcd << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
      << "WIDTH " << data.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << data.size() << "\nDATA ascii\n";
      pcd << std::setprecision(9);
      for (const auto & point : data) {
        pcd << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
      }
      return pcd.good();
    };
  if (!write_pcd(prefix.string() + ".cells.pcd", points)) {return false;}
  points.clear();
  std::size_t cloud_promoted = 0, cloud_cleared = 0, cloud_column_cleared = 0;
  // Preserve the previous odom-voxel ordering in saved artifacts, independent
  // of the hash index used by the real-time integration path.
  std::vector<std::array<int, 3>> ordered_keys;
  ordered_keys.reserve(cloud_voxels_.size());
  for (const auto & entry : cloud_voxels_) {ordered_keys.push_back(entry.first);}
  std::sort(ordered_keys.begin(), ordered_keys.end());
  for (const auto & key : ordered_keys) {
    const auto & voxel = cloud_voxels_.at(key);
    const auto & e = voxel.evidence;
    const int x = static_cast<int>(std::floor((voxel.point[0] - config_.origin_x) / config_.resolution));
    const int y = static_cast<int>(std::floor((voxel.point[1] - config_.origin_y) / config_.resolution));
    const auto * column = cell(x, y);
    cloud_promoted += e.layer == OdomCellLayer::kStatic ? 1U : 0U;
    cloud_cleared += e.cleared_once ? 1U : 0U;
    cloud_column_cleared += column && column->cleared_once ? 1U : 0U;
    if (e.layer != OdomCellLayer::kStatic || e.cleared_once ||
      e.state != OdomCellState::kOccupied || e.observed_time <= 0.0 ||
      e.hit_time_accum / e.observed_time < config_.save_persistence_ratio ||
      (column && column->cleared_once)) {continue;}
    points.push_back(transform(voxel.point[0], voxel.point[1], voxel.point[2]));
  }
  counts.persistent_voxels = points.size();
  yaml << "cloud_voxels: " << cloud_voxels_.size() << "\ncloud_promoted_voxels: "
       << cloud_promoted << "\ncloud_ray_cleared_voxels: " << cloud_cleared
       << "\ncloud_column_cleared_voxels: " << cloud_column_cleared << "\n";
  yaml << "occupied_cells: " << counts.occupied_cells << "\npersistent_cells: "
       << counts.persistent_cells << "\npersistent_voxels: " << counts.persistent_voxels << "\n";
  yaml << "raster_occupied_cells: " << counts.raster_occupied_cells << "\n";
  yaml << "occupied_not_saved_dynamic: " << counts.occupied_dynamic
       << "\noccupied_not_saved_cleared_once: " << counts.occupied_cleared_once
       << "\noccupied_not_saved_other: " << counts.occupied_other << "\n";
  if (stats) {*stats = counts;}
  return write_pcd(pcd_path, points) && pgm.good() && yaml.good();
}

std::size_t OdomLayerStore::fillIsolatedUnknownGaps(
  std::vector<std::int8_t> & data, std::uint32_t width, std::uint32_t height,
  int max_component_cells, int max_bbox_span, int occupied_threshold)
{
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (width < 3U || height < 3U || data.size() != cell_count ||
    max_component_cells <= 0 || max_bbox_span <= 0)
  {
    return 0U;
  }
  const int w = static_cast<int>(width);
  const int h = static_cast<int>(height);
  const auto is_free = [occupied_threshold](std::int8_t v) {
      return v >= 0 && v < occupied_threshold;
    };
  const auto idx = [w](int x, int y) {
      return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
    };

  std::size_t total_filled = 0U;
  std::vector<std::pair<int, int>> component;
  component.reserve(static_cast<std::size_t>(max_component_cells) + 1U);

  for (int sy = 1; sy < h - 1; ++sy) {
    const std::size_t row = static_cast<std::size_t>(sy) * static_cast<std::size_t>(w);
    for (int sx = 1; sx < w - 1; ++sx) {
      const std::size_t sidx = row + static_cast<std::size_t>(sx);
      if (data[sidx] >= 0) {
        continue;
      }
      // Only seed from unknown cells adjacent to at least one known-free 4-neighbor.
      if (!is_free(data[sidx - 1U]) && !is_free(data[sidx + 1U]) &&
        !is_free(data[sidx - width]) && !is_free(data[sidx + width]))
      {
        continue;
      }
      component.clear();
      component.emplace_back(sx, sy);
      int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
      bool valid = true;
      for (std::size_t head = 0U; head < component.size() && valid; ++head) {
        const auto [cx, cy] = component[head];
        if (cx <= 0 || cy <= 0 || cx >= w - 1 || cy >= h - 1) {
          valid = false;
          break;
        }
        for (int dy = -1; dy <= 1 && valid; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = cx + dx;
            const int ny = cy + dy;
            const std::int8_t nv = data[idx(nx, ny)];
            if (nv >= occupied_threshold) {
              valid = false;
              break;
            }
            if (nv < 0) {
              bool already = false;
              for (const auto & p : component) {
                if (p.first == nx && p.second == ny) {
                  already = true;
                  break;
                }
              }
              if (!already) {
                if (static_cast<int>(component.size()) >= max_component_cells) {
                  valid = false;
                  break;
                }
                min_x = std::min(min_x, nx);
                max_x = std::max(max_x, nx);
                min_y = std::min(min_y, ny);
                max_y = std::max(max_y, ny);
                if (max_x - min_x + 1 > max_bbox_span || max_y - min_y + 1 > max_bbox_span) {
                  valid = false;
                  break;
                }
                component.emplace_back(nx, ny);
              }
            }
          }
        }
      }
      if (valid) {
        for (const auto & [fx, fy] : component) {
          data[idx(fx, fy)] = 0;
        }
        total_filled += component.size();
      }
    }
  }
  // Second pass: bridge thin 1-2 cell (5-10 cm) unknown scan-ring or self-filter slits
  // strictly bracketed on both opposite sides by known-free cells with zero occupied
  // cells in their 3x3 neighborhood (never erodes walls or expands into open unknown).
  if (w >= 5 && h >= 5) {
    std::vector<std::size_t> slit_cells;
    for (int sy = 2; sy < h - 2; ++sy) {
      const std::size_t row = static_cast<std::size_t>(sy) * static_cast<std::size_t>(w);
      for (int sx = 2; sx < w - 2; ++sx) {
        const std::size_t sidx = row + static_cast<std::size_t>(sx);
        if (data[sidx] >= 0) {continue;}
        bool near_occ = false;
        for (int dy = -1; dy <= 1 && !near_occ; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (data[idx(sx + dx, sy + dy)] >= occupied_threshold) {
              near_occ = true;
              break;
            }
          }
        }
        if (near_occ) {continue;}
        const bool horiz_bridge =
          (is_free(data[idx(sx - 1, sy)]) && is_free(data[idx(sx + 1, sy)])) ||
          (is_free(data[idx(sx - 1, sy)]) && data[idx(sx + 1, sy)] < 0 && is_free(data[idx(sx + 2, sy)])) ||
          (is_free(data[idx(sx - 2, sy)]) && data[idx(sx - 1, sy)] < 0 && is_free(data[idx(sx + 1, sy)]));
        const bool vert_bridge =
          (is_free(data[idx(sx, sy - 1)]) && is_free(data[idx(sx, sy + 1)])) ||
          (is_free(data[idx(sx, sy - 1)]) && data[idx(sx, sy + 1)] < 0 && is_free(data[idx(sx, sy + 2)])) ||
          (is_free(data[idx(sx, sy - 2)]) && data[idx(sx, sy - 1)] < 0 && is_free(data[idx(sx, sy + 1)]));
        if (horiz_bridge || vert_bridge) {
          slit_cells.push_back(sidx);
        }
      }
    }
    for (const std::size_t sidx : slit_cells) {
      data[sidx] = 0;
    }
    total_filled += slit_cells.size();
  }
  return total_filled;
}

std::size_t OdomLayerStore::filterIsolatedOccupiedSpeckles(
  std::vector<std::int8_t> & data, std::uint32_t width, std::uint32_t height,
  int max_component_cells, int occupied_threshold)
{
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (width < 3U || height < 3U || data.size() != cell_count || max_component_cells <= 0) {
    return 0U;
  }
  const int w = static_cast<int>(width);
  const int h = static_cast<int>(height);
  const auto is_free = [occupied_threshold](std::int8_t v) {
      return v >= 0 && v < occupied_threshold;
    };
  const auto idx = [w](int x, int y) {
      return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
    };
  std::size_t total_removed = 0U;
  std::vector<std::pair<int, int>> component;
  component.reserve(static_cast<std::size_t>(max_component_cells) + 1U);
  for (int sy = 1; sy < h - 1; ++sy) {
    const std::size_t row = static_cast<std::size_t>(sy) * static_cast<std::size_t>(w);
    for (int sx = 1; sx < w - 1; ++sx) {
      const std::size_t sidx = row + static_cast<std::size_t>(sx);
      if (data[sidx] < occupied_threshold) {continue;}
      component.clear();
      component.emplace_back(sx, sy);
      bool isolated = true;
      for (std::size_t head = 0U; head < component.size() && isolated; ++head) {
        const auto [cx, cy] = component[head];
        if (cx <= 0 || cy <= 0 || cx >= w - 1 || cy >= h - 1) {
          isolated = false;
          break;
        }
        for (int dy = -1; dy <= 1 && isolated; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {continue;}
            const int nx = cx + dx;
            const int ny = cy + dy;
            const std::int8_t nv = data[idx(nx, ny)];
            if (nv < 0) {
              // Touching unknown shadow -> could be the edge of a real obstacle.
              isolated = false;
              break;
            }
            if (nv >= occupied_threshold) {
              bool already = false;
              for (const auto & p : component) {
                if (p.first == nx && p.second == ny) {already = true; break;}
              }
              if (!already) {
                if (static_cast<int>(component.size()) >= max_component_cells) {
                  isolated = false;
                  break;
                }
                component.emplace_back(nx, ny);
              }
            } else if (!is_free(nv)) {
              isolated = false;
              break;
            }
          }
        }
      }
      if (isolated) {
        for (const auto & [fx, fy] : component) {
          data[idx(fx, fy)] = 0;
        }
        total_removed += component.size();
      }
    }
  }
  return total_removed;
}

}  // namespace ats_rog_map
