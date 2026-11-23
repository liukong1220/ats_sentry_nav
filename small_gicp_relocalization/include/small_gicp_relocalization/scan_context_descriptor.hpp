// Copyright 2026 ATS Sentry Navigation
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

#ifndef SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_DESCRIPTOR_HPP_
#define SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_DESCRIPTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace small_gicp_relocalization
{

struct ScanContextConfig
{
  int num_rings{20};
  int num_sectors{60};
  double max_radius_m{15.0};
  double min_radius_m{0.35};
  double min_z_m{0.15};
  double max_z_m{2.20};
  double grid_step_xy_m{2.0};
  std::size_t min_anchor_points{120};
  int ring_key_candidates{24};
};

struct ScanContextDescriptor
{
  int num_rings{20};
  int num_sectors{60};
  int occupied_bins{0};
  std::vector<float> matrix;      // [r * num_sectors + s], 0.0f when empty
  std::vector<float> ring_key;    // [num_rings], mean occupancy/height per ring
  std::vector<float> sector_key;  // [num_sectors], mean height per sector
};

struct ScanContextAnchor
{
  double x{0.0};
  double y{0.0};
  double z_ground{0.0};
  ScanContextDescriptor descriptor;
};

struct ScanContextMatch
{
  double x{0.0};
  double y{0.0};
  double z_ground{0.0};
  double yaw{0.0};
  double distance{1.0};
};

class ScanContextDatabase
{
public:
  ScanContextDatabase() = default;
  explicit ScanContextDatabase(const ScanContextConfig & config) : config_(sanitize(config)) {}

  void setConfig(const ScanContextConfig & config) { config_ = sanitize(config); }
  const ScanContextConfig & config() const { return config_; }
  std::size_t size() const { return anchors_.size(); }
  bool empty() const { return anchors_.empty(); }

  template <typename PointT>
  static double estimateLocalGroundZ(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>> & points,
    double cx, double cy, double radius_xy = 2.0)
  {
    std::vector<float> near_z;
    near_z.reserve(256);
    const double r2 = radius_xy * radius_xy;
    for (const auto & pt : points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double dx = static_cast<double>(pt.x) - cx;
      const double dy = static_cast<double>(pt.y) - cy;
      if (dx * dx + dy * dy <= r2) {
        near_z.push_back(pt.z);
      }
    }
    if (near_z.empty()) {
      return 0.0;
    }
    const std::size_t idx = near_z.size() / 10U;
    std::nth_element(near_z.begin(), near_z.begin() + idx, near_z.end());
    return static_cast<double>(near_z[idx]);
  }

  template <typename PointContainer>
  ScanContextDescriptor computeDescriptor(
    const PointContainer & points, double cx, double cy, double z_ground,
    std::size_t * used_points = nullptr) const
  {
    ScanContextDescriptor desc;
    desc.num_rings = config_.num_rings;
    desc.num_sectors = config_.num_sectors;
    const std::size_t total_bins =
      static_cast<std::size_t>(desc.num_rings) * static_cast<std::size_t>(desc.num_sectors);
    desc.matrix.assign(total_bins, 0.0f);
    desc.ring_key.assign(static_cast<std::size_t>(desc.num_rings), 0.0f);
    desc.sector_key.assign(static_cast<std::size_t>(desc.num_sectors), 0.0f);

    const double min_r2 = config_.min_radius_m * config_.min_radius_m;
    const double max_r2 = config_.max_radius_m * config_.max_radius_m;
    const double ring_span = std::max(1e-3, config_.max_radius_m - config_.min_radius_m);
    std::size_t valid_count = 0;

    for (const auto & pt : points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double dx = static_cast<double>(pt.x) - cx;
      const double dy = static_cast<double>(pt.y) - cy;
      const double r2 = dx * dx + dy * dy;
      if (r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      const double rel_z = static_cast<double>(pt.z) - z_ground;
      if (rel_z < config_.min_z_m || rel_z > config_.max_z_m) {
        continue;
      }
      ++valid_count;
      const double r = std::sqrt(r2);
      int ring = static_cast<int>(
        std::floor(((r - config_.min_radius_m) / ring_span) * config_.num_rings));
      ring = std::clamp(ring, 0, config_.num_rings - 1);

      double angle = std::atan2(dy, dx);
      if (angle < 0.0) {
        angle += 2.0 * M_PI;
      }
      int sector = static_cast<int>(
        std::floor((angle / (2.0 * M_PI)) * config_.num_sectors));
      sector = std::clamp(sector, 0, config_.num_sectors - 1);

      const float encoded_h = static_cast<float>(rel_z - config_.min_z_m + 0.05);
      float & cell = desc.matrix[
        static_cast<std::size_t>(ring) * static_cast<std::size_t>(desc.num_sectors) +
        static_cast<std::size_t>(sector)];
      if (encoded_h > cell) {
        cell = encoded_h;
      }
    }

    if (used_points != nullptr) {
      *used_points = valid_count;
    }
    int occupied = 0;
    for (int r = 0; r < desc.num_rings; ++r) {
      float row_sum = 0.0f;
      for (int s = 0; s < desc.num_sectors; ++s) {
        const float v = desc.matrix[
          static_cast<std::size_t>(r) * static_cast<std::size_t>(desc.num_sectors) +
          static_cast<std::size_t>(s)];
        if (v > 0.0f) {
          ++occupied;
          row_sum += v;
          desc.sector_key[static_cast<std::size_t>(s)] += v;
        }
      }
      desc.ring_key[static_cast<std::size_t>(r)] =
        row_sum / static_cast<float>(desc.num_sectors);
    }
    for (int s = 0; s < desc.num_sectors; ++s) {
      desc.sector_key[static_cast<std::size_t>(s)] /= static_cast<float>(desc.num_rings);
    }
    desc.occupied_bins = occupied;
    return desc;
  }

  template <typename PointT>
  void buildFromCloud(const std::vector<PointT, Eigen::aligned_allocator<PointT>> & cloud_points)
  {
    anchors_.clear();
    if (cloud_points.empty()) {
      return;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & pt : cloud_points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y)) {
      return;
    }

    const double step = config_.grid_step_xy_m;
    const int nx = std::max(1, static_cast<int>(std::floor((max_x - min_x) / step)) + 1);
    const int ny = std::max(1, static_cast<int>(std::floor((max_y - min_y) / step)) + 1);
    anchors_.reserve(static_cast<std::size_t>(nx * ny));

    for (int ix = 0; ix < nx; ++ix) {
      const double cx = min_x + (static_cast<double>(ix) + 0.5) * step;
      if (cx > max_x) {
        continue;
      }
      for (int iy = 0; iy < ny; ++iy) {
        const double cy = min_y + (static_cast<double>(iy) + 0.5) * step;
        if (cy > max_y) {
          continue;
        }
        const double z_ground = estimateLocalGroundZ(cloud_points, cx, cy, 2.2);
        std::size_t used = 0;
        ScanContextDescriptor desc = computeDescriptor(cloud_points, cx, cy, z_ground, &used);
        if (used < config_.min_anchor_points || desc.occupied_bins < 16) {
          continue;
        }
        anchors_.push_back(ScanContextAnchor{cx, cy, z_ground, std::move(desc)});
      }
    }
  }

  template <typename PointT>
  std::vector<ScanContextMatch> queryRobotCentricCloud(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>> & local_points,
    std::size_t top_k, double max_distance, double min_xy_separation = 1.5) const
  {
    std::vector<ScanContextMatch> matches;
    if (anchors_.empty() || local_points.empty() || top_k == 0U) {
      return matches;
    }
    const double z_ground = estimateLocalGroundZ(local_points, 0.0, 0.0, 2.5);
    std::size_t used = 0;
    const ScanContextDescriptor query_desc =
      computeDescriptor(local_points, 0.0, 0.0, z_ground, &used);
    if (used < config_.min_anchor_points / 2U || query_desc.occupied_bins < 12) {
      return matches;
    }

    struct RingCandidate
    {
      float l1_dist{0.0f};
      std::size_t anchor_idx{0};
    };
    std::vector<RingCandidate> ring_candidates;
    ring_candidates.reserve(anchors_.size());
    for (std::size_t idx = 0; idx < anchors_.size(); ++idx) {
      const auto & rk = anchors_[idx].descriptor.ring_key;
      float l1 = 0.0f;
      for (std::size_t r = 0; r < rk.size(); ++r) {
        l1 += std::fabs(rk[r] - query_desc.ring_key[r]);
      }
      ring_candidates.push_back(RingCandidate{l1, idx});
    }
    const std::size_t shortlist = std::min<std::size_t>(
      ring_candidates.size(),
      static_cast<std::size_t>(std::max(static_cast<int>(top_k) * 4, config_.ring_key_candidates)));
    std::partial_sort(
      ring_candidates.begin(), ring_candidates.begin() + shortlist, ring_candidates.end(),
      [](const RingCandidate & a, const RingCandidate & b) { return a.l1_dist < b.l1_dist; });

    std::vector<ScanContextMatch> raw_matches;
    raw_matches.reserve(shortlist);
    for (std::size_t i = 0; i < shortlist; ++i) {
      const ScanContextAnchor & anchor = anchors_[ring_candidates[i].anchor_idx];
      int best_shift = 0;
      const double dist = shiftedCosineDistance(query_desc, anchor.descriptor, &best_shift);
      if (!std::isfinite(dist) || dist > max_distance) {
        continue;
      }
      // Query sector s aligns to anchor sector (s + best_shift) mod N_s:
      // yaw_map_from_robot = best_shift * (2*pi / N_s), wrapped to [-pi, pi].
      double yaw = (2.0 * M_PI * static_cast<double>(best_shift)) /
        static_cast<double>(config_.num_sectors);
      yaw = std::atan2(std::sin(yaw), std::cos(yaw));
      raw_matches.push_back(ScanContextMatch{anchor.x, anchor.y, anchor.z_ground, yaw, dist});
    }

    std::sort(
      raw_matches.begin(), raw_matches.end(),
      [](const ScanContextMatch & a, const ScanContextMatch & b) {
        return a.distance < b.distance;
      });
    for (const auto & cand : raw_matches) {
      bool distinct = true;
      for (const auto & kept : matches) {
        const double dxy = std::hypot(cand.x - kept.x, cand.y - kept.y);
        const double dyaw = std::fabs(std::atan2(
          std::sin(cand.yaw - kept.yaw), std::cos(cand.yaw - kept.yaw)));
        if (dxy < min_xy_separation && dyaw < 0.45) {
          distinct = false;
          break;
        }
      }
      if (distinct) {
        matches.push_back(cand);
        if (matches.size() >= top_k) {
          break;
        }
      }
    }
    return matches;
  }

private:
  static ScanContextConfig sanitize(ScanContextConfig cfg)
  {
    cfg.num_rings = std::clamp(cfg.num_rings, 8, 60);
    cfg.num_sectors = std::clamp(cfg.num_sectors, 16, 180);
    cfg.min_radius_m = std::max(0.05, cfg.min_radius_m);
    cfg.max_radius_m = std::max(cfg.min_radius_m + 1.0, cfg.max_radius_m);
    cfg.max_z_m = std::max(cfg.min_z_m + 0.2, cfg.max_z_m);
    cfg.grid_step_xy_m = std::max(0.5, cfg.grid_step_xy_m);
    cfg.min_anchor_points = std::max<std::size_t>(16U, cfg.min_anchor_points);
    cfg.ring_key_candidates = std::max(4, cfg.ring_key_candidates);
    return cfg;
  }

  static double columnCosineDistanceAtShift(
    const ScanContextDescriptor & query, const ScanContextDescriptor & target, int shift)
  {
    const int nr = query.num_rings;
    const int ns = query.num_sectors;
    double cosine_sum = 0.0;
    int valid_cols = 0;
    for (int s = 0; s < ns; ++s) {
      const int ts = ((s + shift) % ns + ns) % ns;
      double dot = 0.0;
      double nq = 0.0;
      double nt = 0.0;
      for (int r = 0; r < nr; ++r) {
        const double qv = query.matrix[
          static_cast<std::size_t>(r) * static_cast<std::size_t>(ns) +
          static_cast<std::size_t>(s)];
        const double tv = target.matrix[
          static_cast<std::size_t>(r) * static_cast<std::size_t>(ns) +
          static_cast<std::size_t>(ts)];
        dot += qv * tv;
        nq += qv * qv;
        nt += tv * tv;
      }
      if (nq <= 1e-9 && nt <= 1e-9) {
        continue;
      }
      ++valid_cols;
      if (nq > 1e-9 && nt > 1e-9) {
        cosine_sum += dot / std::sqrt(nq * nt);
      }
    }
    if (valid_cols == 0) {
      return 1.0;
    }
    return std::clamp(1.0 - cosine_sum / static_cast<double>(valid_cols), 0.0, 1.0);
  }

  static double shiftedCosineDistance(
    const ScanContextDescriptor & query, const ScanContextDescriptor & target, int * best_shift)
  {
    const int ns = query.num_sectors;
    int coarse_shift = 0;
    double best_sector_dot = -1.0;
    for (int shift = 0; shift < ns; ++shift) {
      double dot = 0.0;
      for (int s = 0; s < ns; ++s) {
        const int ts = (s + shift) % ns;
        dot += static_cast<double>(query.sector_key[static_cast<std::size_t>(s)]) *
          static_cast<double>(target.sector_key[static_cast<std::size_t>(ts)]);
      }
      if (dot > best_sector_dot) {
        best_sector_dot = dot;
        coarse_shift = shift;
      }
    }
    double min_dist = 1.0;
    int refined_shift = coarse_shift;
    for (int offset = -3; offset <= 3; ++offset) {
      const int shift = ((coarse_shift + offset) % ns + ns) % ns;
      const double dist = columnCosineDistanceAtShift(query, target, shift);
      if (dist < min_dist) {
        min_dist = dist;
        refined_shift = shift;
      }
    }
    if (best_shift != nullptr) {
      *best_shift = refined_shift;
    }
    return min_dist;
  }

  ScanContextConfig config_;
  std::vector<ScanContextAnchor> anchors_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_DESCRIPTOR_HPP_
