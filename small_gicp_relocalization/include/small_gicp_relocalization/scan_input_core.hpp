// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SCAN_INPUT_CORE_HPP_
#define SMALL_GICP_RELOCALIZATION__SCAN_INPUT_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace small_gicp_relocalization
{

struct ScanInputConfig
{
  std::string expected_frame;
  double max_age_s{1.0};
  double max_future_s{0.10};
  double min_range_m{0.10};
  double max_range_m{100.0};
  double min_z_m{-5.0};
  double max_z_m{5.0};
  double min_valid_ratio{0.50};
};

struct ScanInputStats
{
  std::size_t total_points{0};
  std::size_t finite_points{0};
  std::size_t valid_points{0};
};

/// Validate message-level freshness before a cloud can enter the accumulation window.
/// Nanosecond comparisons intentionally avoid mixing ROS and system clock types.
inline std::string validateScanMetadata(
  const std::string & frame_id, std::int64_t stamp_ns, std::int64_t now_ns,
  const std::int64_t last_stamp_ns, const ScanInputConfig & config)
{
  if (frame_id.empty()) {
    return "scan frame_id is empty";
  }
  if (!config.expected_frame.empty() && frame_id != config.expected_frame) {
    return "scan frame_id mismatch: expected " + config.expected_frame + ", got " + frame_id;
  }
  if (stamp_ns <= 0) {
    return "scan stamp is zero or negative";
  }
  if (last_stamp_ns > 0 && stamp_ns <= last_stamp_ns) {
    return stamp_ns == last_stamp_ns ? "duplicate scan stamp" : "out-of-order scan stamp";
  }
  if (now_ns <= 0) {
    return {};
  }
  const double age_s = static_cast<double>(now_ns - stamp_ns) * 1e-9;
  if (config.max_age_s > 0.0 && age_s > config.max_age_s) {
    return "scan stamp is stale by " + std::to_string(age_s) + " s";
  }
  if (config.max_future_s >= 0.0 && age_s < -config.max_future_s) {
    return "scan stamp is too far in the future by " + std::to_string(-age_s) + " s";
  }
  return {};
}

inline bool scanPointInRange(
  const float x, const float y, const float z, const ScanInputConfig & config)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  const double range_sq =
    static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
  const double min_range_sq = std::max(0.0, config.min_range_m) * std::max(0.0, config.min_range_m);
  const double max_range_sq = std::max(config.min_range_m, config.max_range_m) *
                              std::max(config.min_range_m, config.max_range_m);
  return range_sq >= min_range_sq && range_sq <= max_range_sq &&
         static_cast<double>(z) >= config.min_z_m && static_cast<double>(z) <= config.max_z_m;
}

inline std::string validateScanPointStats(
  const ScanInputStats & stats, const ScanInputConfig & config)
{
  if (stats.total_points == 0) {
    return "scan contains no points";
  }
  if (stats.valid_points == 0) {
    return "scan contains no finite in-range points";
  }
  const double valid_ratio =
    static_cast<double>(stats.valid_points) / static_cast<double>(stats.total_points);
  if (valid_ratio < std::clamp(config.min_valid_ratio, 0.0, 1.0)) {
    return "scan valid point ratio below threshold: " + std::to_string(valid_ratio);
  }
  return {};
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SCAN_INPUT_CORE_HPP_
