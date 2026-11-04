// Copyright 2026

#include "ats_rog_map/odom_layer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

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
  if (value->state == OdomCellState::kUnknown) {
    value->first_hit_time = stamp;
    value->hit_time_accum = 0.0;
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

bool OdomLayerStore::save(const std::string & output_prefix, const std::string & frame_id) const
{
  if (output_prefix.empty() || frame_id.empty()) {
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
  std::vector<std::array<float, 3>> points;
  points.reserve(cells_.size() / 10U);
  if (!pgm || !yaml) {
    return false;
  }
  pgm << "P5\n" << config_.width << " " << config_.height << "\n255\n";
  for (std::uint32_t row = 0; row < config_.height; ++row) {
    // PGM row zero is the top row; OccupancyGrid y zero is the bottom row.
    const std::uint32_t y = config_.height - 1U - row;
    for (std::uint32_t x = 0; x < config_.width; ++x) {
      const auto & value = cells_[index(static_cast<int>(x), static_cast<int>(y))];
      unsigned char pixel = 205U;  // unknown
      if (saveEligible(static_cast<int>(x), static_cast<int>(y))) {
        pixel = 0U;
        points.push_back({
          static_cast<float>(config_.origin_x + (static_cast<double>(x) + 0.5) * config_.resolution),
          static_cast<float>(config_.origin_y + (static_cast<double>(y) + 0.5) * config_.resolution),
          0.0F});
      } else if (value.state == OdomCellState::kFree) {
        pixel = 254U;
      }
      pgm.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }
  yaml << std::fixed << std::setprecision(6)
       << "image: " << std::filesystem::path(pgm_path).filename().string() << "\n"
       << "resolution: " << config_.resolution << "\n"
       << "origin: [" << config_.origin_x << ", " << config_.origin_y << ", 0.0]\n"
       << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
       << "frame_id: " << frame_id << "\n";

  std::ofstream pcd(pcd_path);
  if (!pcd) {
    return false;
  }
  pcd << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
      << "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << points.size() << "\nDATA ascii\n";
  pcd << std::setprecision(6);
  for (const auto & point : points) {
    pcd << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
  }
  return pgm.good() && yaml.good() && pcd.good();
}

}  // namespace ats_rog_map
