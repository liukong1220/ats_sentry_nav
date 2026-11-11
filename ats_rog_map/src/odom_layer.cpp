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
  cells_(static_cast<std::size_t>(config.width) * config.height)
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
  const double bounded_dt = std::clamp(observed_dt, 0.0, config_.max_frame_dt_seconds);
  if (value->state != OdomCellState::kOccupied) {
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
  if (value->layer == OdomCellLayer::kDynamic && !value->cleared_once &&
    value->hit_time_accum >= config_.static_promotion_seconds)
  {
    value->layer = OdomCellLayer::kStatic;
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
    value->state != OdomCellState::kFree)
  {return false;}
  value->state = OdomCellState::kUnknown;
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
  if (evidence.state == OdomCellState::kOccupied && stamp <= evidence.last_hit_time) {
    return false;
  }
  voxel.point = point;
  evidence.state = OdomCellState::kOccupied;
  const double interval = evidence.last_hit_time > 0.0 ?
    stamp - evidence.last_hit_time : observed_dt;
  evidence.last_hit_time = stamp;
  if (!evidence.cleared_once) {
    if (voxel.candidate_index == std::numeric_limits<std::size_t>::max()) {
      ray_candidates_.push_back({key, stamp});
      voxel.candidate_index = ray_candidates_.size() - 1;
    } else {
      ray_candidates_[voxel.candidate_index].last_hit_time = stamp;
    }
  }
  evidence.hit_time_accum += std::clamp(interval, 0.0, config_.max_frame_dt_seconds);
  evidence.observed_time = evidence.hit_time_accum;
  if (!evidence.cleared_once && evidence.hit_time_accum >= config_.static_promotion_seconds) {
    evidence.layer = OdomCellLayer::kStatic;
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
  return found != cloud_voxels_.end() && !found->second.evidence.cleared_once &&
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
  if (found == cloud_voxels_.end() || found->second.evidence.cleared_once ||
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
  removeRayClearCandidate(found->second);
}

bool OdomLayerStore::observeFree(int x, int y, double stamp, double observed_dt)
{
  auto * value = cell(x, y);
  if (!value || !std::isfinite(stamp) || stamp <= value->last_visible_time) {return false;}
  const bool cleared = observeRayClear(x, y, stamp, true);
  value->last_visible_time = stamp;
  value->observed_time += std::clamp(observed_dt, 0.0, config_.max_frame_dt_seconds);
  value->state = OdomCellState::kFree;
  return cleared;
}

std::int8_t OdomLayerStore::sampleFootprint(
  double min_x, double min_y, double max_x, double max_y) const
{
  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
    !std::isfinite(max_y) || max_x <= min_x || max_y <= min_y) {return -1;}
  const int first_x = std::max(0, static_cast<int>(std::floor((min_x - config_.origin_x) / config_.resolution)));
  const int first_y = std::max(0, static_cast<int>(std::floor((min_y - config_.origin_y) / config_.resolution)));
  const int last_x = std::min(static_cast<int>(config_.width) - 1,
    static_cast<int>(std::ceil((max_x - config_.origin_x) / config_.resolution - 1e-8)) - 1);
  const int last_y = std::min(static_cast<int>(config_.height) - 1,
    static_cast<int>(std::ceil((max_y - config_.origin_y) / config_.resolution - 1e-8)) - 1);
  bool free = false;
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      const auto state = cell(x, y)->state;
      if (state == OdomCellState::kOccupied) {return 100;}
      free = free || state == OdomCellState::kFree;
    }
  }
  return free ? 0 : -1;
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
  for (auto & value : cells_) {
    if (value.layer == OdomCellLayer::kDynamic &&
      value.state == OdomCellState::kOccupied && value.last_hit_time > 0.0 &&
      stamp - std::max(value.last_hit_time, value.last_visible_time) >
      config_.dynamic_hold_seconds)
    {
      value.state = OdomCellState::kUnknown;
    }
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
    }
  }
  return true;
}

void OdomLayerStore::clear()
{
  cells_.assign(cells_.size(), OdomLayerCell{});
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
      } else if (value.state == OdomCellState::kFree) {
        pixel = 254U;
      }
      pixels[index(x, y)] = pixel;
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

}  // namespace ats_rog_map
