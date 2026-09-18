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

#ifndef SMALL_GICP_RELOCALIZATION__RELOCALIZATION_CANDIDATE_CORE_HPP_
#define SMALL_GICP_RELOCALIZATION__RELOCALIZATION_CANDIDATE_CORE_HPP_

// 重定位候选调度与评分的纯逻辑核心。这里只放不依赖 ROS 节点状态的函数，
// 以便用 gtest 锁定“候选覆盖公平性、非有限误差语义、硬门、组合评分与歧义拒绝”。

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace small_gicp_relocalization
{

/// 从旋转矩阵直接取 yaw。Eigen 的 eulerAngles(0,1,2) 在 roll/pitch 出现极小负值时
/// 会翻转到 (pi, pi-eps, yaw+pi) 分支，导致 yaw 出现 180 度跳变，进而使整个候选
/// 格网被旋转。重定位链上所有 yaw 提取都必须走这里。
inline double yawOf(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

inline double yawOf(const Eigen::Isometry3d & pose) { return yawOf(pose.rotation()); }

inline double wrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

inline double yawDistance(const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  return std::abs(wrapAngle(yawOf(lhs) - yawOf(rhs)));
}

/// [0, count) 的确定性二分广度优先排列。任意前缀都近似均匀铺满整个区间，
/// 因此按预算截断时不会像 x->y->yaw 三重循环那样只覆盖搜索窗的一侧。
inline std::vector<std::size_t> bisectionOrder(std::size_t count)
{
  std::vector<std::size_t> order;
  if (count == 0) {
    return order;
  }
  order.reserve(count);
  std::vector<std::pair<std::size_t, std::size_t>> pending;
  pending.reserve(2 * count + 1);
  pending.emplace_back(0, count);
  for (std::size_t cursor = 0; cursor < pending.size(); ++cursor) {
    const std::pair<std::size_t, std::size_t> range = pending[cursor];
    if (range.first >= range.second) {
      continue;
    }
    const std::size_t middle = range.first + (range.second - range.first) / 2;
    order.push_back(middle);
    pending.emplace_back(range.first, middle);
    pending.emplace_back(middle + 1, range.second);
  }
  return order;
}

/// 覆盖 [-pi, pi) 的中心向外 yaw 偏移序列：0, +step, -step, +2step, -2step, ...
/// +pi 与 -pi 是同一旋转，只发出一次。
inline std::vector<double> centerOutYawOffsets(double step_yaw)
{
  std::vector<double> offsets;
  const double step = std::max(1e-3, std::abs(step_yaw));
  offsets.push_back(0.0);
  const int max_index = static_cast<int>(std::floor((M_PI + 1e-9) / step));
  for (int index = 1; index <= max_index; ++index) {
    const double offset = static_cast<double>(index) * step;
    if (offset >= M_PI - 1e-9) {
      offsets.push_back(M_PI);
      break;
    }
    offsets.push_back(offset);
    offsets.push_back(-offset);
  }
  return offsets;
}

struct CandidateLatticeConfig
{
  double search_half_xy{2.0};
  double step_xy{1.0};
  double step_yaw{M_PI / 4.0};
  std::vector<double> z_offsets{0.0};
};

struct CandidateLatticeStats
{
  std::size_t generated{0};
  std::size_t rings{0};
  double max_radius{0.0};
  double max_abs_yaw_offset{0.0};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
};

inline bool sameCandidate(
  const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs, double translation_epsilon = 1e-6,
  double yaw_epsilon = 1e-6)
{
  return (lhs.translation() - rhs.translation()).norm() <= translation_epsilon &&
         yawDistance(lhs, rhs) <= yaw_epsilon;
}

/// 分层交错候选调度：seed 本身优先，然后按 XY 半径由小到大；每个半径内先按
/// 中心向外的 yaw 偏移分轮，再在轮内用二分序交错扫过方向，且每轮换一个方向起点。
/// 生成集合与原来的方形搜索窗一致，只改变枚举顺序，因此按预算截断得到的子集
/// 在方向、yaw 与半径上都是平衡的。
inline std::vector<Eigen::Isometry3d> buildLayeredCandidateLattice(
  const Eigen::Isometry3d & seed, const CandidateLatticeConfig & config,
  CandidateLatticeStats * stats = nullptr)
{
  const double seed_yaw = yawOf(seed);
  const double seed_x = seed.translation().x();
  const double seed_y = seed.translation().y();
  const double seed_z = seed.translation().z();

  std::vector<double> z_offsets = config.z_offsets;
  if (z_offsets.empty()) {
    z_offsets.push_back(0.0);
  }
  const double step_xy = std::max(1e-3, config.step_xy);
  const double half_xy = std::max(0.0, config.search_half_xy);
  const int ring_limit = static_cast<int>(std::floor(half_xy / step_xy + 1e-9));
  const std::vector<double> yaw_offsets = centerOutYawOffsets(config.step_yaw);

  struct GridOffset
  {
    double radius{0.0};
    double angle{0.0};
    double dx{0.0};
    double dy{0.0};
  };
  std::vector<GridOffset> offsets;
  const std::size_t span = static_cast<std::size_t>(2 * ring_limit + 1);
  offsets.reserve(span * span);
  for (int ix = -ring_limit; ix <= ring_limit; ++ix) {
    for (int iy = -ring_limit; iy <= ring_limit; ++iy) {
      GridOffset offset;
      offset.dx = static_cast<double>(ix) * step_xy;
      offset.dy = static_cast<double>(iy) * step_xy;
      offset.radius = std::hypot(offset.dx, offset.dy);
      offset.angle = std::atan2(offset.dy, offset.dx);
      offsets.push_back(offset);
    }
  }
  std::sort(offsets.begin(), offsets.end(), [](const GridOffset & lhs, const GridOffset & rhs) {
    if (std::abs(lhs.radius - rhs.radius) > 1e-6) {
      return lhs.radius < rhs.radius;
    }
    return lhs.angle < rhs.angle;
  });

  std::vector<Eigen::Isometry3d> candidates;
  candidates.reserve(offsets.size() * yaw_offsets.size() * z_offsets.size() + 1);
  candidates.push_back(seed);

  std::size_t rings = 0;
  std::size_t ring_begin = 0;
  while (ring_begin < offsets.size()) {
    std::size_t ring_end = ring_begin + 1;
    while (ring_end < offsets.size() &&
           std::abs(offsets[ring_end].radius - offsets[ring_begin].radius) <= 1e-6) {
      ++ring_end;
    }
    ++rings;
    const std::size_t ring_size = ring_end - ring_begin;
    const std::vector<std::size_t> direction_order = bisectionOrder(ring_size);
    for (std::size_t yaw_index = 0; yaw_index < yaw_offsets.size(); ++yaw_index) {
      const double yaw = wrapAngle(seed_yaw + yaw_offsets[yaw_index]);
      const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      for (std::size_t step = 0; step < ring_size; ++step) {
        const GridOffset & offset =
          offsets[ring_begin + direction_order[(step + yaw_index) % ring_size]];
        for (const double dz : z_offsets) {
          Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
          guess.translation() << seed_x + offset.dx, seed_y + offset.dy, seed_z + dz;
          guess.linear() = rotation;
          if (sameCandidate(guess, seed)) {
            continue;
          }
          candidates.push_back(guess);
        }
      }
    }
    ring_begin = ring_end;
  }

  if (stats) {
    CandidateLatticeStats summary;
    summary.generated = candidates.size();
    summary.rings = rings;
    summary.min_x = seed_x;
    summary.max_x = seed_x;
    summary.min_y = seed_y;
    summary.max_y = seed_y;
    for (const auto & candidate : candidates) {
      const double x = candidate.translation().x();
      const double y = candidate.translation().y();
      summary.min_x = std::min(summary.min_x, x);
      summary.max_x = std::max(summary.max_x, x);
      summary.min_y = std::min(summary.min_y, y);
      summary.max_y = std::max(summary.max_y, y);
      summary.max_radius = std::max(summary.max_radius, std::hypot(x - seed_x, y - seed_y));
      summary.max_abs_yaw_offset =
        std::max(summary.max_abs_yaw_offset, std::abs(wrapAngle(yawOf(candidate) - seed_yaw)));
    }
    *stats = summary;
  }
  return candidates;
}

/// 单个候选的全部可观测证据。真值误差不在这里，禁止参与线上评分。
struct CandidateEvidence
{
  bool converged{false};
  bool transform_finite{false};
  std::size_t num_inliers{0};
  std::size_t source_points{0};
  double registration_error{std::numeric_limits<double>::infinity()};
  double overlap_ratio{0.0};
  double min_information_eigenvalue{0.0};
  double information_condition_number{std::numeric_limits<double>::infinity()};
  /// 与上一帧假设按 odometry 推算后的位姿残差 [m]，无参考时为 0。
  double motion_residual{0.0};
  /// 与 seed / 上一可信位姿的平移偏差 [m]，仅作软约束。
  double prior_deviation{0.0};
};

struct CandidateHardGates
{
  int min_inliers{1};
  /// < 0 关闭。
  double max_registration_error{-1.0};
  /// 0 关闭。
  double min_overlap_ratio{0.0};
  /// 0 关闭。
  double min_information_eigenvalue{0.0};
  /// 0 关闭。
  double max_information_condition_number{0.0};
  /// 仿真放宽只允许放过 optimizer 的 converged 标志，其他门一律保留。
  bool allow_unconverged{false};
};

/// 由验收门派生粗筛选门。筛选级只检查证据可用性：有限 transform、有限误差和
/// 足够内点。converged、overlap、information 与最大误差都是验收语义，只在
/// fine 级生效：稀疏筛选云上的 overlap 与信息矩阵量级和验收级不可比，提前套用
/// 会丢掉 fine 收敛后本可命中真值的候选。非有限误差与非有限 transform 无论哪
/// 一级都不可通过。
inline CandidateHardGates screenStageGates(CandidateHardGates gates)
{
  gates.allow_unconverged = true;
  gates.max_registration_error = -1.0;
  gates.min_overlap_ratio = 0.0;
  gates.min_information_eigenvalue = 0.0;
  gates.max_information_condition_number = 0.0;
  return gates;
}

/// 返回空字符串表示通过硬门。非有限 registration error 与非有限 transform
/// 永远不可被任何放宽绕过。
inline std::string candidateRejectReason(
  const CandidateEvidence & evidence, const CandidateHardGates & gates)
{
  if (!evidence.transform_finite) {
    return "non-finite transform";
  }
  if (!std::isfinite(evidence.registration_error) || evidence.registration_error < 0.0) {
    return "non-finite registration error";
  }
  if (
    static_cast<std::int64_t>(evidence.num_inliers) <
    static_cast<std::int64_t>(gates.min_inliers)) {
    return "insufficient inliers";
  }
  if (
    gates.max_registration_error >= 0.0 &&
    evidence.registration_error > gates.max_registration_error) {
    return "registration error too large";
  }
  if (!evidence.converged && !gates.allow_unconverged) {
    return "not converged";
  }
  if (gates.min_overlap_ratio > 0.0 && evidence.overlap_ratio < gates.min_overlap_ratio) {
    return "overlap ratio below threshold";
  }
  if (
    gates.min_information_eigenvalue > 0.0 &&
    evidence.min_information_eigenvalue < gates.min_information_eigenvalue) {
    return "minimum information eigenvalue below threshold";
  }
  if (
    gates.max_information_condition_number > 0.0 &&
    !(evidence.information_condition_number <= gates.max_information_condition_number)) {
    return "information matrix is too ill-conditioned";
  }
  return std::string();
}

struct CandidateScoreWeights
{
  double error{1.0};
  double overlap{1.0};
  double information{0.5};
  double motion{0.5};
  double prior{0.2};
  /// 饱和尺度：各项归一化到 [0, 1]，因此总分范围是 [0, sum(weights)]。
  double error_scale{1.0};
  double information_eigenvalue_reference{1.0};
  double condition_number_reference{1.0e4};
  double motion_scale{0.5};
  double prior_scale{2.0};
};

struct CandidateScore
{
  double total{std::numeric_limits<double>::infinity()};
  double error_term{1.0};
  double overlap_term{1.0};
  double information_term{1.0};
  double motion_term{0.0};
  double prior_term{0.0};
};

/// x/(x+scale)：单调、有界于 [0,1]，非有限输入取最差值 1。
inline double saturatingPenalty(double value, double scale)
{
  const double safe_scale = std::max(1e-9, scale);
  if (!std::isfinite(value)) {
    return 1.0;
  }
  const double clamped = std::max(0.0, value);
  return clamped / (clamped + safe_scale);
}

/// 组合评分，越小越好。仅在硬门通过后使用。
inline CandidateScore scoreCandidate(
  const CandidateEvidence & evidence, const CandidateScoreWeights & weights)
{
  CandidateScore score;
  if (!evidence.transform_finite || !std::isfinite(evidence.registration_error)) {
    return score;
  }
  score.error_term = saturatingPenalty(evidence.registration_error, weights.error_scale);
  score.overlap_term = 1.0 - std::clamp(evidence.overlap_ratio, 0.0, 1.0);
  const double eigenvalue_reference = std::max(1e-12, weights.information_eigenvalue_reference);
  const double eigenvalue = std::isfinite(evidence.min_information_eigenvalue)
                              ? std::max(0.0, evidence.min_information_eigenvalue)
                              : 0.0;
  const double eigenvalue_term = eigenvalue_reference / (eigenvalue_reference + eigenvalue);
  const double condition_term =
    saturatingPenalty(evidence.information_condition_number, weights.condition_number_reference);
  score.information_term = 0.5 * (eigenvalue_term + condition_term);
  score.motion_term = saturatingPenalty(evidence.motion_residual, weights.motion_scale);
  score.prior_term = saturatingPenalty(evidence.prior_deviation, weights.prior_scale);
  score.total = weights.error * score.error_term + weights.overlap * score.overlap_term +
                weights.information * score.information_term + weights.motion * score.motion_term +
                weights.prior * score.prior_term;
  return score;
}

struct AmbiguityConfig
{
  /// <= 0 关闭歧义拒绝；阈值应由 correct/wrong 候选分布决定，不要凭空拍定。
  double min_score_margin{0.0};
  /// 判定“不同假设”的最小几何分离；分离不足的次优解属于同一 basin 的一致解，
  /// 不构成歧义，否则相邻格点收敛到同一极小值会永久堵死恢复。
  double min_separation_xy{0.5};
  double min_separation_yaw{0.35};
};

struct RankedCandidate
{
  double score{std::numeric_limits<double>::infinity()};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
};

struct CandidateSelection
{
  bool has_best{false};
  std::size_t best_index{0};
  bool has_alternative{false};
  std::size_t alternative_index{0};
  double best_score{std::numeric_limits<double>::infinity()};
  double alternative_score{std::numeric_limits<double>::infinity()};
  double score_margin{std::numeric_limits<double>::infinity()};
  bool ambiguous{false};
};

/// 从硬门通过的候选中选最优解，并给出第一个几何上真正不同的次优解。
/// S2 - S1 < min_score_margin 时判为 ambiguous，调用方必须保持 LOST。
inline CandidateSelection selectCandidate(
  const std::vector<RankedCandidate> & candidates, const AmbiguityConfig & config)
{
  CandidateSelection selection;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (!std::isfinite(candidates[index].score)) {
      continue;
    }
    if (!selection.has_best || candidates[index].score < selection.best_score) {
      selection.has_best = true;
      selection.best_index = index;
      selection.best_score = candidates[index].score;
    }
  }
  if (!selection.has_best) {
    return selection;
  }

  const RankedCandidate & best = candidates[selection.best_index];
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (index == selection.best_index || !std::isfinite(candidates[index].score)) {
      continue;
    }
    const RankedCandidate & other = candidates[index];
    const double translation_gap =
      (other.transform.translation() - best.transform.translation()).head<2>().norm();
    const double yaw_gap = yawDistance(other.transform, best.transform);
    if (
      translation_gap < std::max(0.0, config.min_separation_xy) &&
      yaw_gap < std::max(0.0, config.min_separation_yaw)) {
      continue;
    }
    if (!selection.has_alternative || other.score < selection.alternative_score) {
      selection.has_alternative = true;
      selection.alternative_index = index;
      selection.alternative_score = other.score;
    }
  }
  if (selection.has_alternative) {
    selection.score_margin = selection.alternative_score - selection.best_score;
    selection.ambiguous =
      config.min_score_margin > 0.0 && selection.score_margin < config.min_score_margin;
  }
  return selection;
}

struct ObservationQualityResult
{
  /// 原始 registration error 是否有限且非负。false 时禁止发布 STATUS_ACCEPTED。
  bool error_finite{false};
  double quality{0.0};
};

/// 非有限或负的 registration error 一律 quality=0 且 error_finite=false。
/// 绝不把 inf/NaN 改写成 0.0 误差或 1.0 误差质量。
inline ObservationQualityResult observationQuality(
  bool accepted, double registration_error, std::size_t inliers, std::size_t source_points)
{
  ObservationQualityResult result;
  result.error_finite = std::isfinite(registration_error) && registration_error >= 0.0;
  if (!result.error_finite || !accepted) {
    return result;
  }
  const double inlier_ratio =
    source_points > 0 ? static_cast<double>(inliers) / static_cast<double>(source_points) : 0.0;
  const double error_quality = std::exp(-std::min(registration_error, 10.0));
  result.quality = std::clamp(inlier_ratio * error_quality, 0.0, 1.0);
  return result;
}

struct ConfirmationMotionResidual
{
  bool valid{false};
  double translation{std::numeric_limits<double>::infinity()};
  double yaw{std::numeric_limits<double>::infinity()};
};

/// 两次估计隐含的机体相对运动与 odometry 相对运动之差。
/// map->base_i = map_to_odom_i * odom_to_base_i，因此 map->odom 的 yaw 偏差会在
/// 机体处放大成平移误差，比直接比较两个 map->odom 平移更严格。
inline ConfirmationMotionResidual confirmationMotionResidual(
  const Eigen::Isometry3d & first_map_to_odom, const Eigen::Isometry3d & first_odom_to_base,
  const Eigen::Isometry3d & second_map_to_odom, const Eigen::Isometry3d & second_odom_to_base)
{
  ConfirmationMotionResidual residual;
  const Eigen::Isometry3d first_map_to_base = first_map_to_odom * first_odom_to_base;
  const Eigen::Isometry3d second_map_to_base = second_map_to_odom * second_odom_to_base;
  const Eigen::Isometry3d estimated_motion = first_map_to_base.inverse() * second_map_to_base;
  const Eigen::Isometry3d odometry_motion = first_odom_to_base.inverse() * second_odom_to_base;
  const Eigen::Isometry3d difference = estimated_motion.inverse() * odometry_motion;
  if (!difference.matrix().allFinite()) {
    return residual;
  }
  residual.valid = true;
  residual.translation = difference.translation().norm();
  residual.yaw = std::abs(wrapAngle(yawOf(difference)));
  return residual;
}

struct ConfirmationGates
{
  double translation_tolerance{0.15};
  double yaw_tolerance{0.10};
  /// 两次确认扫描之间的最小时间间隔 [s]，禁止同一窗口重复计数。
  double min_interval_s{0.05};
  double motion_translation_tolerance{0.25};
  double motion_yaw_tolerance{0.15};
};

struct ConfirmationSample
{
  Eigen::Isometry3d map_to_odom{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d odom_to_base{Eigen::Isometry3d::Identity()};
  double scan_time_s{0.0};
};

struct ConfirmationDecision
{
  bool consistent{false};
  std::string reason;
  double translation_delta{std::numeric_limits<double>::infinity()};
  double yaw_delta{std::numeric_limits<double>::infinity()};
  double motion_translation{std::numeric_limits<double>::infinity()};
  double motion_yaw{std::numeric_limits<double>::infinity()};
};

/// 多帧确认契约：扫描时间必须严格递增且满足最小间隔，两次 map->odom 必须一致，
/// 且两次估计隐含的相对运动必须与 odometry 一致。
inline ConfirmationDecision evaluateConfirmation(
  const ConfirmationSample & pending, const ConfirmationSample & candidate,
  const ConfirmationGates & gates)
{
  ConfirmationDecision decision;
  if (!(candidate.scan_time_s > pending.scan_time_s)) {
    decision.reason = "confirmation scan stamp not increasing";
    return decision;
  }
  const double interval = candidate.scan_time_s - pending.scan_time_s;
  if (interval < std::max(0.0, gates.min_interval_s)) {
    decision.reason = "confirmation scan interval too short";
    return decision;
  }
  decision.translation_delta =
    (candidate.map_to_odom.translation() - pending.map_to_odom.translation()).norm();
  decision.yaw_delta = yawDistance(candidate.map_to_odom, pending.map_to_odom);
  if (
    !std::isfinite(decision.translation_delta) ||
    decision.translation_delta > std::max(0.0, gates.translation_tolerance) ||
    decision.yaw_delta > std::max(0.0, gates.yaw_tolerance)) {
    decision.reason = "confirmation transform mismatch";
    return decision;
  }
  const ConfirmationMotionResidual residual = confirmationMotionResidual(
    pending.map_to_odom, pending.odom_to_base, candidate.map_to_odom, candidate.odom_to_base);
  if (!residual.valid) {
    decision.reason = "confirmation motion residual not finite";
    return decision;
  }
  decision.motion_translation = residual.translation;
  decision.motion_yaw = residual.yaw;
  if (
    residual.translation > std::max(0.0, gates.motion_translation_tolerance) ||
    residual.yaw > std::max(0.0, gates.motion_yaw_tolerance)) {
    decision.reason = "confirmation inconsistent with odometry motion";
    return decision;
  }
  decision.consistent = true;
  return decision;
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__RELOCALIZATION_CANDIDATE_CORE_HPP_
