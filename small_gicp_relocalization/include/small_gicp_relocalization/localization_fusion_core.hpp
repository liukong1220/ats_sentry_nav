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

#ifndef SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_
#define SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"

namespace small_gicp_relocalization
{

inline bool observationMatchesFrontend(
  std::uint64_t instance, std::uint64_t epoch,
  std::uint64_t current_instance, std::uint64_t current_epoch)
{
  return instance != 0 && epoch != 0 && instance == current_instance && epoch == current_epoch;
}

struct OdomPoseSample
{
  rclcpp::Time stamp;
  tf2::Transform odom_to_base;
};

struct CorrectionDelta
{
  double translation{0.0};
  double yaw{0.0};
};

struct CorrectionUpdate
{
  tf2::Transform map_to_odom{tf2::Transform::getIdentity()};
  CorrectionDelta delta;
  bool apply{false};
  bool advance_epoch{false};
  // 超过 epoch 阈值但不超过连续修正上界：直接改 map->odom，不推进 epoch。
  bool continuous{false};
};

/// Evict by measurement time first, then enforce a hard sample-count bound.
/// Return count-cap drops separately from ordinary time-window eviction.
inline std::size_t pruneOdomHistory(
  std::deque<OdomPoseSample> & history, const rclcpp::Time & cutoff,
  std::size_t max_samples)
{
  while (!history.empty() && history.front().stamp < cutoff) {
    history.pop_front();
  }
  const std::size_t dropped = history.size() > max_samples ? history.size() - max_samples : 0;
  for (std::size_t index = 0; index < dropped; ++index) {
    history.pop_front();
  }
  return dropped;
}

inline std::optional<tf2::Transform> interpolateOdomPose(
  const std::deque<OdomPoseSample> & history, const rclcpp::Time & stamp,
  double boundary_tolerance_s, double maximum_interpolation_gap_s)
{
  if (history.empty()) {
    return std::nullopt;
  }
  const auto tolerance = rclcpp::Duration::from_seconds(std::max(0.0, boundary_tolerance_s));
  if (stamp + tolerance < history.front().stamp || stamp > history.back().stamp + tolerance) {
    return std::nullopt;
  }

  const auto newer = std::lower_bound(
    history.begin(), history.end(), stamp,
    [](const OdomPoseSample & sample, const rclcpp::Time & target) {
      return sample.stamp < target;
    });
  if (newer == history.begin()) {
    return newer->odom_to_base;
  }
  if (newer == history.end()) {
    return history.back().odom_to_base;
  }
  if (newer->stamp == stamp) {
    return newer->odom_to_base;
  }

  const auto older = std::prev(newer);
  const double gap = (newer->stamp - older->stamp).seconds();
  if (gap <= 0.0 || (maximum_interpolation_gap_s > 0.0 && gap > maximum_interpolation_gap_s)) {
    return std::nullopt;
  }
  const double alpha = std::clamp((stamp - older->stamp).seconds() / gap, 0.0, 1.0);
  tf2::Transform interpolated;
  interpolated.setOrigin(
    older->odom_to_base.getOrigin().lerp(newer->odom_to_base.getOrigin(), alpha));
  tf2::Quaternion rotation =
    older->odom_to_base.getRotation().slerp(newer->odom_to_base.getRotation(), alpha);
  rotation.normalize();
  interpolated.setRotation(rotation);
  return interpolated;
}

inline bool observationIsNewer(
  const rclcpp::Time & stamp, std::uint64_t sequence,
  const std::optional<rclcpp::Time> & last_stamp,
  const std::optional<std::uint64_t> & last_sequence)
{
  if (!last_stamp) {
    return true;
  }
  if (stamp > *last_stamp) {
    return true;
  }
  if (stamp < *last_stamp) {
    return false;
  }
  return !last_sequence || sequence > *last_sequence;
}

inline CorrectionDelta correctionDelta(const tf2::Transform & newer, const tf2::Transform & older)
{
  CorrectionDelta delta;
  delta.translation = (newer.getOrigin() - older.getOrigin()).length();
  const double yaw_delta = tf2::getYaw(newer.getRotation()) - tf2::getYaw(older.getRotation());
  delta.yaw = std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
  return delta;
}

inline bool correctionWithinLimits(const CorrectionDelta & delta, double translation, double yaw)
{
  return std::isfinite(delta.translation) && std::isfinite(delta.yaw) &&
         delta.translation >= 0.0 && delta.yaw >= 0.0 &&
         (translation <= 0.0 || delta.translation <= translation) &&
         (yaw <= 0.0 || delta.yaw <= yaw);
}

// continuous_max_translation/yaw > 0 时，超过 epoch 阈值但不超过该上界的修正按连续修正应用：
// map->odom 改变而 epoch 不变。MPC 在 odom 里执行，不受 map->odom 影响；规划器对已提交参考的
// 复检按当前 TF 重新表达，GoalManager 的位姿门也用当前 TF，所以这类修正不必作废整条参考链。
// 超过上界（大幅跳变、重新定位）仍推进 epoch，下游照旧停车重建。
inline CorrectionUpdate selectCorrectionUpdate(
  const tf2::Transform & candidate, const std::optional<tf2::Transform> & current,
  double translation_threshold, double yaw_threshold,
  double continuous_max_translation = 0.0, double continuous_max_yaw = 0.0)
{
  CorrectionUpdate update;
  if (!current) {
    update.map_to_odom = candidate;
    update.apply = true;
    update.advance_epoch = true;
    return update;
  }

  update.map_to_odom = *current;
  update.delta = correctionDelta(candidate, *current);
  const auto reaches = [](double delta, double threshold) {
      return delta > 0.0 && delta >= std::max(0.0, threshold);
    };
  const bool beyond_threshold = reaches(update.delta.translation, translation_threshold) ||
    reaches(update.delta.yaw, yaw_threshold);
  const bool continuous_enabled = continuous_max_translation > 0.0 && continuous_max_yaw > 0.0;
  update.continuous = beyond_threshold && continuous_enabled &&
    update.delta.translation <= continuous_max_translation &&
    update.delta.yaw <= continuous_max_yaw;
  update.advance_epoch = beyond_threshold && !update.continuous;
  update.apply = beyond_threshold;
  if (update.apply) {
    update.map_to_odom = candidate;
  }
  return update;
}

struct TrackingFilterConfig
{
  bool enable{true};
  // 死区：小于该值的跟踪期抖动只刷新健康心跳、不改写 map->odom（非 settling 状态）。
  double deadband_translation{0.04};
  double deadband_yaw{0.03};
  // 指数低通阻尼系数 alpha in (0, 1]：每次接受观测仅吸收一部分残差，消除阶跃跳变。
  double lpf_alpha_translation{0.25};
  double lpf_alpha_yaw{0.20};
  // 进入平滑吸收后收敛到该残差内即贴齐并退出 settling，保证稳态无残差。
  double settle_translation_tolerance{0.001};
  double settle_yaw_tolerance{0.002};
  // 相对 Point-LIO 帧间运动一致性门控：单次跳变超限时需连续一致确认才放行。
  double motion_consistency_base_m{0.12};
  double motion_consistency_ratio{0.05};
  double motion_consistency_base_yaw{0.08};
  int motion_jump_confirm_count{3};
  double motion_jump_confirm_tolerance_m{0.04};
  double motion_jump_confirm_tolerance_yaw{0.04};
};

inline tf2::Transform interpolateTransform(
  const tf2::Transform & from, const tf2::Transform & to,
  double alpha_translation, double alpha_yaw)
{
  const double at = std::clamp(alpha_translation, 0.0, 1.0);
  const double ay = std::clamp(alpha_yaw, 0.0, 1.0);
  tf2::Transform out;
  out.setOrigin(from.getOrigin().lerp(to.getOrigin(), at));
  tf2::Quaternion q = from.getRotation().slerp(to.getRotation(), ay);
  q.normalize();
  out.setRotation(q);
  return out;
}

struct DirectionalFusionConfig
{
  bool enable{true};
  double weak_axis_damping{0.0};
  double weak_yaw_damping{0.2};
  double max_strong_nis{36.0};
};

struct DirectionalFusionResult
{
  bool accepted{true};
  bool damped_weak_axis{false};
  double strong_nis{0.0};
  tf2::Transform filtered_map_to_odom;
  std::string reject_reason;
};

/// O5b: 在隧道/长走廊退化场景下，对不可观测弱轴方向上的修正做正交投影抑制，
/// 同时对强可观测子空间执行归一化新息平方（NIS）门控，保留法向贴壁对齐并阻断轴向滑移。
inline DirectionalFusionResult applyDirectionalDegeneracyFusion(
  const tf2::Transform & current_map_to_odom,
  const tf2::Transform & observed_map_to_base,
  const tf2::Transform & odom_to_base,
  const std::array<double, 36> & pose_covariance,
  std::uint8_t degeneracy_mask,
  double weak_dir_x,
  double weak_dir_y,
  const DirectionalFusionConfig & config)
{
  DirectionalFusionResult out;
  out.filtered_map_to_odom = observed_map_to_base * odom_to_base.inverse();
  if (!config.enable) {
    return out;
  }

  const tf2::Transform pred_map_to_base = current_map_to_odom * odom_to_base;
  const Eigen::Vector2d p_pred(
    pred_map_to_base.getOrigin().x(), pred_map_to_base.getOrigin().y());
  const Eigen::Vector2d p_obs(
    observed_map_to_base.getOrigin().x(), observed_map_to_base.getOrigin().y());
  Eigen::Vector2d dp = p_obs - p_pred;
  const double yaw_pred = tf2::getYaw(pred_map_to_base.getRotation());
  const double yaw_obs = tf2::getYaw(observed_map_to_base.getRotation());
  double dyaw = std::atan2(std::sin(yaw_obs - yaw_pred), std::cos(yaw_obs - yaw_pred));

  const bool weak_trans = (degeneracy_mask & 1U) != 0U;
  const bool weak_yaw = (degeneracy_mask & 2U) != 0U;
  Eigen::Vector2d u_weak(weak_dir_x, weak_dir_y);
  if (!u_weak.allFinite() || u_weak.norm() < 1e-6) {
    u_weak = Eigen::Vector2d(1.0, 0.0);
  } else {
    u_weak.normalize();
  }
  const Eigen::Vector2d u_strong(-u_weak.y(), u_weak.x());

  // Extract calibrated SE(2) covariance on [x, y, yaw] (indices 0, 1, 5 of 6x6 PoseWithCovariance).
  // Inflate covariance floor by realistic sensor/map discretization floor (0.04 m / 0.03 rad)
  // so tight sub-millimeter GICP covariances do not over-reject valid tracking corrections.
  constexpr std::array<int, 3> kIdx{{0, 1, 5}};
  Eigen::Matrix3d cov_se2 = Eigen::Matrix3d::Zero();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cov_se2(r, c) = pose_covariance[static_cast<std::size_t>(6 * kIdx[r] + kIdx[c])];
    }
  }
  cov_se2 = 0.5 * (cov_se2 + cov_se2.transpose());
  cov_se2(0, 0) += 0.05 * 0.05;
  cov_se2(1, 1) += 0.05 * 0.05;
  cov_se2(2, 2) += 0.04 * 0.04;

  if (config.max_strong_nis > 0.0 && cov_se2.allFinite()) {
    if (weak_trans) {
      const double r_perp = dp.dot(u_strong);
      const double var_perp = std::max(
        1e-4, (u_strong.transpose() * cov_se2.block<2, 2>(0, 0) * u_strong)(0, 0));
      out.strong_nis = (r_perp * r_perp) / var_perp;
      if (!weak_yaw) {
        out.strong_nis += (dyaw * dyaw) / std::max(1e-4, cov_se2(2, 2));
      }
    } else {
      Eigen::Vector3d r_vec(dp.x(), dp.y(), dyaw);
      Eigen::LDLT<Eigen::Matrix3d> ldlt(cov_se2);
      if (ldlt.info() == Eigen::Success) {
        out.strong_nis = std::max(0.0, r_vec.dot(ldlt.solve(r_vec)));
      }
    }
    if (std::isfinite(out.strong_nis) && out.strong_nis > config.max_strong_nis) {
      out.accepted = false;
      out.reject_reason = "relocalization observation failed strong-subspace NIS gate";
      return out;
    }
  }

  if (weak_trans || weak_yaw) {
    if (weak_trans) {
      const double damp = std::clamp(config.weak_axis_damping, 0.0, 1.0);
      dp -= (1.0 - damp) * dp.dot(u_weak) * u_weak;
      out.damped_weak_axis = true;
    }
    if (weak_yaw) {
      const double damp_y = std::clamp(config.weak_yaw_damping, 0.0, 1.0);
      dyaw *= damp_y;
      out.damped_weak_axis = true;
    }
    double obs_roll = 0.0;
    double obs_pitch = 0.0;
    double obs_yaw_unused = 0.0;
    tf2::Matrix3x3(observed_map_to_base.getRotation()).getRPY(obs_roll, obs_pitch, obs_yaw_unused);
    tf2::Quaternion fused_q;
    fused_q.setRPY(obs_roll, obs_pitch, yaw_pred + dyaw);
    fused_q.normalize();
    tf2::Transform fused_map_to_base;
    fused_map_to_base.setOrigin(tf2::Vector3(
      p_pred.x() + dp.x(), p_pred.y() + dp.y(), observed_map_to_base.getOrigin().z()));
    fused_map_to_base.setRotation(fused_q);
    out.filtered_map_to_odom = fused_map_to_base * odom_to_base.inverse();
  }
  return out;
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_
