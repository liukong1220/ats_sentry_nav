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
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

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
  if (frame_id != config.expected_frame) {
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

/// Own whole scan chunks; callbacks never concatenate or shift the existing window.
/// PointVector is the cloud's native vector type (including its aligned allocator).
template <typename PointVector>
class ScanAccumulationWindow
{
public:
  struct Trim
  {
    std::size_t sampled_points{0};
    std::size_t evicted_frames{0};
  };

  ScanAccumulationWindow(
    std::size_t max_points, std::size_t max_frames, std::size_t required_frames)
  : max_points_(max_points), max_frames_(max_frames)
  {
    if (required_frames == 0 || required_frames > max_frames || max_points < max_frames) {
      throw std::invalid_argument(
        "accumulate_frames must be positive and <= max_accumulated_frames; "
        "max_accumulated_points must retain at least one point per maximum frame");
    }
    // Reserve room for the full temporal window, not just required_frames: the
    // registration gate also requires a minimum scan timestamp span. With quota
    // max_points/required_frames, dense scans could still starve that age gate.
    // Cadences needing more than max_frames to span that duration remain invalid
    // operational inputs; neither the frame bound nor the age gate is relaxed.
    per_scan_points_ = max_points / max_frames;
  }

  Trim append(PointVector points, std::int64_t stamp_ns)
  {
    if (points.empty()) {
      throw std::invalid_argument("cannot accumulate an empty scan");
    }
    Trim trim;
    if (points.size() > per_scan_points_) {
      trim.sampled_points = points.size() - per_scan_points_;
      PointVector sampled;
      sampled.reserve(per_scan_points_);
      // Sample the entire ordered scan, not a first-N crop. Both endpoints survive
      // when there is room for two points. Integer stepping avoids index overflow.
      if (per_scan_points_ == 1) {
        sampled.push_back(points[points.size() / 2]);
      } else {
        const auto intervals = per_scan_points_ - 1;
        const auto step = (points.size() - 1) / intervals;
        const auto remainder = (points.size() - 1) % intervals;
        std::size_t index = 0;
        std::size_t error = 0;
        for (std::size_t i = 0; i < per_scan_points_; ++i) {
          sampled.push_back(points[index]);
          if (i + 1 < per_scan_points_) {
            index += step;
            if (error >= intervals - remainder) {
              ++index;
              error -= intervals - remainder;
            } else {
              error += remainder;
            }
          }
        }
      }
      points = std::move(sampled);
    }
    while (!frames_.empty() &&
           (frames_.size() >= max_frames_ || point_count_ > max_points_ - points.size())) {
      point_count_ -= frames_.front().points.size();
      frames_.pop_front();
      ++trim.evicted_frames;
    }
    const auto added_points = points.size();
    frames_.push_back({std::move(points), stamp_ns});
    point_count_ += added_points;
    return trim;
  }

  void clear()
  {
    frames_.clear();
    point_count_ = 0;
  }

  std::size_t pointCount() const { return point_count_; }
  std::size_t frameCount() const { return frames_.size(); }
  std::optional<std::int64_t> firstStamp() const
  {
    return frames_.empty() ? std::nullopt : std::optional<std::int64_t>(frames_.front().stamp_ns);
  }

  void assemble(PointVector & points) const
  {
    points.clear();
    points.reserve(point_count_);
    for (const auto & frame : frames_) {
      points.insert(points.end(), frame.points.begin(), frame.points.end());
    }
  }

private:
  struct Frame
  {
    PointVector points;
    std::int64_t stamp_ns;
  };
  std::size_t max_points_;
  std::size_t max_frames_;
  std::size_t per_scan_points_;
  std::size_t point_count_{0};
  std::deque<Frame> frames_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SCAN_INPUT_CORE_HPP_
