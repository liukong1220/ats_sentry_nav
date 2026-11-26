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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp_relocalization/localization_status_input_core.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{


namespace
{
std::array<double, 36> registrationCovariance(
  const Eigen::Matrix<double, 6, 6> & information, double error, std::size_t inliers)
{
  Eigen::Matrix<double, 6, 6> covariance_rt = Eigen::Matrix<double, 6, 6>::Zero();
  const bool error_usable = std::isfinite(error) && error >= 0.0;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    0.5 * (information + information.transpose()));
  if (
    error_usable && solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
    solver.eigenvectors().allFinite()) {
    const double degrees_of_freedom = std::max(1.0, 3.0 * static_cast<double>(inliers) - 6.0);
    const double residual_scale = std::clamp(2.0 * error / degrees_of_freedom, 1e-6, 1e3);
    Eigen::Matrix<double, 6, 1> variances;
    for (int index = 0; index < 6; ++index) {
      variances(index) =
        std::clamp(residual_scale / std::max(1e-9, solver.eigenvalues()(index)), 1e-6, 1e3);
    }
    covariance_rt =
      solver.eigenvectors() * variances.asDiagonal() * solver.eigenvectors().transpose();
  } else {
    covariance_rt.diagonal() << 1.0, 1.0, 1.0, 4.0, 4.0, 4.0;
  }

  // small_gicp 的李代数顺序是 [rx, ry, rz, x, y, z]，ROS PoseWithCovariance
  // 使用 [x, y, z, rx, ry, rz]。
  constexpr std::array<int, 6> kPoseToRegistration{{3, 4, 5, 0, 1, 2}};
  std::array<double, 36> covariance{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      covariance[static_cast<std::size_t>(6 * row + column)] =
        covariance_rt(kPoseToRegistration[row], kPoseToRegistration[column]);
    }
  }
  return covariance;
}

}  // namespace

void SmallGicpRelocalizationNode::drainAsyncMultiGuessResult()
{
  MultiGuessOutcome outcome;
  rclcpp::Time scan_time;
  std::size_t source_points = 0;
  {
    std::lock_guard<std::mutex> lock(async_result_mutex_);
    if (!async_result_ready_) {
      return;
    }
    outcome = async_result_;
    scan_time = async_result_scan_time_;
    source_points = async_result_source_points_;
    async_result_ready_ = false;
  }

  if (!relocalization_generation_.admit(outcome.generation, [&]() {
  const bool cancelled = outcome.best.reject_reason == "multi_guess cancelled";
  // 确认复核用的是待确认假设作为种子，其 cursor 不代表探索进度：
  // 若把它写回，未评估候选的扫描进度会被反复重置。
  if (!cancelled && !outcome.confirmation_recheck) {
    multi_guess_cursor_ = outcome.next_cursor;
    if (outcome.wrapped) {
      ++multi_guess_sweep_;
    }
  }
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), log_throttle_ms_,
    "【重定位多猜测完成】生成=%zu screened=%zu refined=%zu gated=%zu skipped=%zu "
    "next_cursor=%zu "
    "已回绕=%s 预算耗尽=%s 用时=%.3fs 最佳得分=%.4f 次佳=%.4f 裕量=%.4f "
    "存在歧义=%s",
    outcome.lattice.generated, outcome.evaluated, outcome.refined, outcome.gated, outcome.skipped,
    outcome.next_cursor, outcome.wrapped ? "true" : "false",
    outcome.budget_exhausted ? "true" : "false", outcome.elapsed_s, outcome.selection.best_score,
    outcome.selection.alternative_score, outcome.selection.score_margin,
    outcome.selection.ambiguous ? "true" : "false");

  if (cancelled) {
    return;
  }
  // Pending confirmation owns the episode: only confirmation_recheck may advance
  // or reject it. A late lattice worker started before pending was set would
  // otherwise land ~0.55 m away and clear the hypothesis (straight188).
  if (pending_confirmation_ && !outcome.confirmation_recheck) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【重定位丢弃】确认待定期间丢弃非复核 multi_guess "
      "(generated=%zu gated=%zu)",
      outcome.lattice.generated, outcome.gated);
    return;
  }
  if (outcome.seed_stage_ran) {
    ++cold_start_seed_tried_;
  }
  handleRegistrationAttempt(outcome.best, scan_time, source_points);
  })) {
    ++stale_async_result_count_;
    RCLCPP_WARN(
      this->get_logger(),
      "【重定位丢弃】丢弃过期 multi_guess 结果（累计=%s 结果代际=%s 当前代际=%s 候选通过=%zu "
      "最佳=%s）",
      std::to_string(stale_async_result_count_).c_str(), std::to_string(outcome.generation).c_str(),
      std::to_string(relocalization_generation_.current()).c_str(), outcome.gated,
      outcome.best.ok ? "有" : "无");
  }
}

void SmallGicpRelocalizationNode::clearConfirmation()
{
  pending_confirmation_.reset();
  pending_confirmation_count_ = 0;
  confirmation_started_at_.reset();
  pending_holds_lattice_origin_ = false;
}

void SmallGicpRelocalizationNode::invalidateRecovery()
{
  relocalization_generation_.invalidate();
  cancel_multi_guess_.store(true);
  clearAccumulation();
  clearConfirmation();
  last_hypothesis_.reset();
  last_hypothesis_time_.reset();
  multi_guess_cursor_ = 0;
  need_coarse_alignment_ = true;
}

bool SmallGicpRelocalizationNode::expireConfirmation()
{
  if (!confirmation_started_at_ ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() -
      *confirmation_started_at_).count() < confirmation_timeout_s_) {
    return false;
  }
  invalidateRecovery();
  return true;
}

ConfirmationDecision SmallGicpRelocalizationNode::evaluateCandidateConfirmation(
  const Eigen::Isometry3d & candidate, const rclcpp::Time & scan_time,
  const Eigen::Isometry3d & odom_to_base) const
{
  ConfirmationDecision decision;
  if (!pending_confirmation_) {
    decision.reason = "no pending confirmation anchor";
    return decision;
  }
  ConfirmationGates gates;
  gates.translation_tolerance = confirmation_translation_tolerance_;
  gates.yaw_tolerance = confirmation_yaw_tolerance_;
  gates.min_interval_s = confirmation_min_interval_s_;
  gates.motion_translation_tolerance = confirmation_motion_translation_tolerance_;
  gates.motion_yaw_tolerance = confirmation_motion_yaw_tolerance_;

  ConfirmationSample sample;
  sample.map_to_odom = candidate;
  sample.odom_to_base = odom_to_base;
  sample.scan_time_s = scan_time.seconds();
  return evaluateConfirmation(*pending_confirmation_, sample, gates);
}

void SmallGicpRelocalizationNode::handleRegistrationAttempt(
  RegistrationAttempt attempt, const rclcpp::Time & scan_time, std::size_t source_points)
{
  const auto covariance =
    registrationCovariance(attempt.information, attempt.registration_error, attempt.num_inliers);
  if (expireConfirmation()) {
    publishObservation(
      scan_time, false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
      "confirmation episode expired", attempt.num_inliers, attempt.registration_error,
      source_points, Eigen::Isometry3d::Identity(), covariance);
    return;
  }

  if (log_registration_details_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【GICP结果】阶段=%s 成功=%s 收敛=%s 迭代=%zu 内点=%zu 误差=%.6f "
      "重叠率=%.3f 最小特征值=%.4g 条件数=%.4g 运动=%.3f 先验=%.3f 得分=%.4f 输入点=%zu",
      attempt.stage.c_str(), attempt.ok ? "true" : "false", attempt.converged ? "true" : "false",
      attempt.iterations, attempt.num_inliers, attempt.registration_error, attempt.overlap_ratio,
      attempt.min_information_eigenvalue, attempt.information_condition_number,
      attempt.motion_residual, attempt.prior_deviation, attempt.score, source_points);
  }

  if (!attempt.ok) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【GICP拒绝】阶段=%s reason=%s converged=%s 内点=%zu/%d 误差=%.6f "
      "重叠率=%.3f 最大误差=%.6f",
      attempt.stage.c_str(), attempt.reject_reason.c_str(), attempt.converged ? "true" : "false",
      attempt.num_inliers, min_inliers_, attempt.registration_error, attempt.overlap_ratio,
      max_registration_error_);
    // Any failed attempt while a confirmation episode is open must retain the
    // pending hypothesis. fine_only/coarse+fine rejects after fusion left LOST
    // previously called clearConfirmation and aborted a prior-valid pending
    // (straight190: pending@3.9s then fine_only wipe@5.7s).
    if (!pending_confirmation_) {
      clearConfirmation();
      if (!has_accepted_alignment_ || isRecoveringLocalization() || !follow_localization_status_) {
        need_coarse_alignment_ = true;
      }
    } else if (!pending_holds_lattice_origin_ &&
               attempt.reject_reason == "confirmation_recheck drifted from pending seed") {
      // 已锁定跟踪期（非 LOST 格点恢复）若 confirmation_recheck 偏离旧 pending seed
      // （例如隧道内单帧轴向滑移污染了 pending），立即清掉坏锚点，避免空耗 10 s 确认超时；
      // 仍严格要求后续重新形成两次连续一致确认，不放宽任何验收门。
      clearConfirmation();
    }
    publishObservation(
      scan_time, false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
      attempt.reject_reason.empty() ? "rejected" : attempt.reject_reason, attempt.num_inliers,
      attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
    return;
  }

  // 确认与接受都必须锚定同一扫描时刻的 odom 位姿；拿不到就不能推进确认状态。
  // Never relabel a latest pose as scan_time: fusion composes the observation
  // with odometry at that stamp, so a fallback would inject robot motion into
  // map->odom. A TF miss preserves an open episode without advancing it.
  const auto odom_to_robot_base = getOdomToRobotBase(scan_time);

  if (!odom_to_robot_base) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【重定位观测缺失】找不到 odom->robot_base（pending=%s stage=%s），保留当前确认过程",
      pending_confirmation_ ? "true" : "false", attempt.stage.c_str());
    if (!pending_confirmation_) {
      clearConfirmation();
      last_hypothesis_.reset();
      last_hypothesis_time_.reset();
    }
    publishObservation(
      scan_time, false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_NO_ODOM,
      "odom->robot_base unavailable at observation time", attempt.num_inliers,
      attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
    return;
  }


  const Eigen::Isometry3d candidate = attempt.transform;
  if (has_accepted_alignment_ && follow_localization_status_ && !isRecoveringLocalization() &&
      localization_state_ == ats_navigation_interfaces::msg::LocalizationStatus::STATE_TRACKING &&
      !pending_confirmation_ && attempt.stage == "fine_only")
  {
    const bool motion_jump = tracking_max_motion_residual_ > 0.0 &&
      std::isfinite(attempt.motion_residual) &&
      attempt.motion_residual > tracking_max_motion_residual_;
    const bool prior_jump = tracking_max_prior_deviation_ > 0.0 &&
      std::isfinite(attempt.prior_deviation) &&
      attempt.prior_deviation > tracking_max_prior_deviation_;
    if (motion_jump || prior_jump) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), log_throttle_ms_,
        "【GICP跟踪跳变拒绝】阶段=%s motion=%.3f(限%.3f) prior=%.3f(限%.3f) inliers=%zu cond=%.3g，保持当前锚点。",
        attempt.stage.c_str(), attempt.motion_residual, tracking_max_motion_residual_,
        attempt.prior_deviation, tracking_max_prior_deviation_, attempt.num_inliers,
        attempt.information_condition_number);
      publishObservation(
        scan_time, false,
        ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
        "tracking fine_only jump exceeded motion/prior gate", attempt.num_inliers,
        attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
      return;
    }
  }

  ConfirmationSample sample;
  sample.map_to_odom = candidate;
  sample.odom_to_base = *odom_to_robot_base;
  sample.scan_time_s = scan_time.seconds();
  sample.last_counted_scan_time_s = sample.scan_time_s;

  // 记录本帧假设，供下一帧候选 motion 一致性软约束使用。
  last_hypothesis_ = sample;
  last_hypothesis_time_ = scan_time;

  if (!has_accepted_alignment_ &&
      (cold_start_prior_max_xy_m_ > 0.0 || cold_start_prior_max_yaw_rad_ > 0.0)) {
    Eigen::Isometry3d prior = Eigen::Isometry3d::Identity();
    if (!init_pose_.empty() && init_pose_.size() >= 6) {
      prior.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
      prior.linear() =
        (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
    }
    if (mapping_mode_) {prior = growing_seed_transform_;}
    const double dxy =
      (candidate.translation() - prior.translation()).head<2>().norm();
    const double dyaw = yawDistance(candidate, prior);
    if ((cold_start_prior_max_xy_m_ > 0.0 && dxy > cold_start_prior_max_xy_m_) ||
        (cold_start_prior_max_yaw_rad_ > 0.0 && dyaw > cold_start_prior_max_yaw_rad_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), log_throttle_ms_,
        "【GICP拒绝】冷启动结果偏离初始先验：dxy=%.3f dyaw=%.3f 限值=%.3f/%.3f 阶段=%s",
        dxy, dyaw, cold_start_prior_max_xy_m_, cold_start_prior_max_yaw_rad_,
        attempt.stage.c_str());
      if (!(pending_confirmation_ && attempt.stage == "confirmation_recheck")) {
        clearConfirmation();
        need_coarse_alignment_ = true;
      }
      publishObservation(
        scan_time, false,
        ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
        "cold-start hypothesis outside init_pose prior", attempt.num_inliers,
        attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
      return;
    }
  }

  if (confirmation_count_ > 1) {
    if (!pending_confirmation_) {
      using LS = ats_navigation_interfaces::msg::LocalizationStatus;
      // Latch BEFORE installing pending: preferMultiGuess() is true while a
      // confirmation episode is open (straight190 ownership), so evaluating it
      // after assignment would force lattice-hold on every cold open and break
      // UNINITIALIZED adopt semantics (ConfirmationRetainsFirstGeometry...).
      pending_holds_lattice_origin_ =
        preferMultiGuess() || localization_state_ == LS::STATE_LOST ||
        localization_state_ == LS::STATE_RELOCALIZING ||
        localization_state_ == LS::STATE_BOOTSTRAP;
      pending_confirmation_ = sample;
      pending_confirmation_count_ = 1;
      confirmation_started_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), log_throttle_ms_,
        "【重定位确认待定】计数=1/%d stage=%s", confirmation_count_,
        attempt.stage.c_str());

      // odometry_stale LOST (recovery180) reopened confirmation with
      // has_accepted_alignment_=true; mismatch then adopted a bad pending and
      // recentered multi_guess (seed yaw ~π). Retain last trusted pose until a
      // full confirmation succeeds. UNINITIALIZED/TRACKING still adopt.
    } else {
      const ConfirmationDecision decision =
        evaluateCandidateConfirmation(candidate, scan_time, *odom_to_robot_base);
      if (decision.consistent) {
        ++pending_confirmation_count_;
        pending_confirmation_->last_counted_scan_time_s = sample.scan_time_s;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), log_throttle_ms_,
          "【重定位确认重启】%s (dxy=%.3f dyaw=%.3f motion_dxy=%.3f motion_dyaw=%.3f)",
          decision.reason.c_str(), decision.translation_delta, decision.yaw_delta,
          decision.motion_translation, decision.motion_yaw);
        // 时间戳不递增/间隔不足意味着这是同一扫描窗口，不能重置锚点后重复计数。
        const bool same_window = decision.reason == "confirmation scan stamp not increasing" ||
                                 decision.reason == "confirmation scan interval too short";
        if (!same_window) {
          // Recovery lattice pending must not recenter multi_guess on mismatch.
          // Seeded UNINITIALIZED confirmation still adopts the new sample
          // (ConfirmationRetainsFirstGeometryAndEpisodeDeadline).
          if (pending_holds_lattice_origin_) {
            // confirmation_recheck is a re-hit of the SAME hypothesis: retain
            // the pending anchor and wait for the next scan (straight187).
            // Far lattice mismatches still clear so a bad pending cannot stick.
            if (attempt.stage != "confirmation_recheck") {
              clearConfirmation();
              need_coarse_alignment_ = true;
            }
            publishObservation(
              scan_time, false,
              ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
              attempt.stage == "confirmation_recheck"
                ? "confirmation_recheck mismatch; retaining pending hypothesis"
                : "recovery confirmation mismatch; retaining lattice origin",
              attempt.num_inliers, attempt.registration_error, source_points,
              Eigen::Isometry3d::Identity(), covariance);
            return;
          }
          pending_confirmation_ = sample;
          pending_confirmation_count_ = 1;
        }
      }
    }
    if (pending_confirmation_count_ < confirmation_count_) {
      publishObservation(
        scan_time, false,
        ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_PENDING_CONFIRMATION,
        "等待连续一致的 GICP 确认", attempt.num_inliers, attempt.registration_error,
        source_points, candidate * *odom_to_robot_base, covariance);
      if (coarse_first_window_only_ && fine_alignment_enabled_) {
        need_coarse_alignment_ = false;
      }
      return;
    }
  }

  result_t_ = previous_result_t_ = candidate;
  last_accepted_observation_scan_time_ = scan_time;
  if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
    last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
  }
  clearConfirmation();
  has_accepted_alignment_ = true;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), log_throttle_ms_,
    "【GICP对齐接受】阶段=%s inliers=%zu error=%.6f",
    attempt.stage.c_str(), attempt.num_inliers, attempt.registration_error);
  // 接受后 seed 变了，格网 cursor 必须从新 seed 的中心重新开始。
  multi_guess_cursor_ = 0;

  if (coarse_first_window_only_ && fine_alignment_enabled_) {
    using LS = ats_navigation_interfaces::msg::LocalizationStatus;
    if (!follow_localization_status_) {
      need_coarse_alignment_ = false;
    } else {
      need_coarse_alignment_ = localization_state_ == LS::STATE_LOST ||
                               (localization_state_ == LS::STATE_RELOCALIZING && recovery_from_lost_);
    }
  }

  const Se2DirectionalInformation se2_info = computeSe2SchurInformation(
    attempt.information, result_t_, *odom_to_robot_base,
    weak_translation_eigenvalue_threshold_, weak_yaw_eigenvalue_threshold_,
    attempt.num_inliers, max_translation_condition_ratio_,
    weak_translation_per_inlier_threshold_);
  if (log_registration_details_ || se2_info.degeneracy_mask != 0U) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【SE2退化评估】inliers=%zu lam_min=%.1f lam_max=%.1f cond_xy=%.2f per_inlier=%.3f "
      "yaw_info=%.1f mask=%u weak_dir=(%.2f,%.2f)",
      attempt.num_inliers, se2_info.min_translation_eigenvalue,
      se2_info.max_translation_eigenvalue, se2_info.translation_condition_ratio,
      se2_info.min_translation_per_inlier, se2_info.yaw_marginal_information,
      static_cast<unsigned>(se2_info.degeneracy_mask),
      se2_info.weak_direction_x, se2_info.weak_direction_y);
  }
  publishObservation(
    scan_time, true, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_ACCEPTED,
    "accepted:" + attempt.stage, attempt.num_inliers, attempt.registration_error, source_points,
    result_t_ * *odom_to_robot_base, covariance, se2_info);
  if (mapping_mode_) {
    growing_pending_source_.clear();
    for (const auto & p : source_->points) {growing_pending_source_.push_back({p.x, p.y, p.z});}
    growing_pending_transform_ = result_t_;
    growing_pending_sequence_ = observation_sequence_;
    growing_pending_scan_ns_ = scan_time.nanoseconds();
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!publish_tf_) {
    return;
  }
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  const auto current_time = now();
  rclcpp::Time tf_stamp = current_time;
  if (has_received_scan_) {
    tf_stamp = last_scan_time_ + rclcpp::Duration::from_seconds(transform_future_offset_s_);
    const double lag_s = (current_time - last_scan_time_).seconds();
    if (max_scan_stamp_lag_s_ > 0.0 && lag_s > max_scan_stamp_lag_s_) {
      tf_stamp = current_time + rclcpp::Duration::from_seconds(transform_future_offset_s_);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "【重定位时间修正】small_gicp TF 时间落后当前时间 %.3fs，map->odom 时间戳已限制为当前时间。",
        lag_s);
    }
  }
  transform_stamped.header.stamp = tf_stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::publishObservation(
  const rclcpp::Time & scan_time, bool accepted, std::uint8_t status,
  const std::string & message, std::size_t inliers,
  double error, std::size_t source_points, const Eigen::Isometry3d & map_to_robot_base,
  const std::array<double, 36> & covariance,
  const Se2DirectionalInformation & se2_info)
{
  if (!observation_pub_) {
    return;
  }

  using Observation = ats_navigation_interfaces::msg::RelocalizationObservation;
  const ObservationQualityResult quality =
    observationQuality(accepted, error, inliers, source_points);

  bool effective_accepted = accepted;
  std::uint8_t effective_status = status;
  std::string effective_message = message;
  // 非有限 registration error 绝不允许成为 STATUS_ACCEPTED，也绝不改写成 0.0。
  if (!quality.error_finite && (accepted || status == Observation::STATUS_ACCEPTED)) {
    effective_accepted = false;
    effective_status = Observation::STATUS_INVALID;
    effective_message = "配准误差为非有限值，禁止接受";
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_,
      "【GICP拒绝】配准误差为非有限值（%f），报告 STATUS_INVALID",
      error);
  }

  Observation observation;
  observation.header.stamp = scan_time;
  observation.header.frame_id = map_frame_;
  observation.child_frame_id = robot_base_frame_;
  observation.sequence = ++observation_sequence_;
  observation.odom_instance_id = odom_instance_id_;
  observation.odom_epoch = odom_epoch_;
  observation.accepted = effective_accepted;
  observation.status = effective_status;
  observation.inlier_count = static_cast<std::uint32_t>(
    std::min<std::size_t>(inliers, std::numeric_limits<std::uint32_t>::max()));
  observation.source_points = static_cast<std::uint32_t>(
    std::min<std::size_t>(source_points, std::numeric_limits<std::uint32_t>::max()));
  observation.registration_error = error;
  observation.quality = quality.quality;
  observation.message = effective_message;

  const Eigen::Vector3d translation = map_to_robot_base.translation();
  const Eigen::Quaterniond rotation(map_to_robot_base.rotation());
  observation.pose.pose.position.x = translation.x();
  observation.pose.pose.position.y = translation.y();
  observation.pose.pose.position.z = translation.z();
  observation.pose.pose.orientation.x = rotation.x();
  observation.pose.pose.orientation.y = rotation.y();
  observation.pose.pose.orientation.z = rotation.z();
  observation.pose.pose.orientation.w = rotation.w();

  observation.pose.covariance = covariance;
  observation.se2_information = se2_info.se2_information;
  observation.min_se2_eigenvalue = se2_info.min_se2_eigenvalue;
  observation.weak_direction_x = se2_info.weak_direction_x;
  observation.weak_direction_y = se2_info.weak_direction_y;
  observation.degeneracy_mask = se2_info.degeneracy_mask;

  observation_pub_->publish(observation);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Invalidate even when TF lookup below fails: old work cannot answer a new reset.
  invalidateRecovery();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), log_throttle_ms_,
    "【重定位初始位姿】收到 [x: %f, y: %f, z: %f]",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    initial_pose_override_time_ = this->now();
    need_coarse_alignment_ = true;
    has_accepted_alignment_ = false;
    // Drop any in-flight LOST lattice search; seeded coarse+fine owns recovery now.
    cancel_multi_guess_.store(true);
    if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
      last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), log_throttle_ms_, "【重定位初始位姿失败】无法从 %s 转换到 %s：%s",
      robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
  }
}

void SmallGicpRelocalizationNode::localizationStatusCallback(
  const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  const auto received_at = this->now();
  const auto rejection = validateLocalizationStatusMetadata(
    msg->header.frame_id, map_frame_, msg->header.stamp.sec, msg->header.stamp.nanosec,
    received_at.nanoseconds(), last_status_stamp_ns_, msg->epoch, localization_epoch_,
    msg->state == LS::STATE_TRACKING, status_stale_skip_registration_s_, max_scan_future_s_);
  if (!rejection.empty() || !isKnownLocalizationState<LS>(msg->state)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "【重定位状态拒绝】%s",
      rejection.empty() ? "未知状态" : rejection.c_str());
    return;
  }
  last_status_stamp_ns_ =
    static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
  const bool frontend_changed = odom_instance_id_ != msg->odom_instance_id ||
    odom_epoch_ != msg->odom_epoch;
  if ((mapping_mode_ || require_odom_identity_) && frontend_changed) {
    invalidateRecovery();
    clearAccumulation();
    has_received_scan_ = false;
    last_received_scan_stamp_ns_ = 0;
    has_accepted_alignment_ = false;
    need_coarse_alignment_ = true;
    growing_pending_source_.clear();
    mapping_reseed_needed_ = mapping_mode_ && mapping_last_map_base_.has_value();
  }
  odom_instance_id_ = msg->odom_instance_id;
  odom_epoch_ = msg->odom_epoch;
  odom_epoch_start_ns_ = rclcpp::Time(msg->odom_epoch_start_stamp).nanoseconds();
  odom_identity_latched_ = msg->odom_identity_latched;
  if (mapping_mode_ && !growing_pending_source_.empty() && !odom_identity_latched_ &&
    msg->observation_sequence == growing_pending_sequence_ &&
    rclcpp::Time(msg->last_observation_stamp).nanoseconds() == growing_pending_scan_ns_ &&
    msg->map_to_odom_valid && msg->map_to_odom.header.frame_id == map_frame_ &&
    msg->map_to_odom.child_frame_id == odom_frame_ &&
    msg->map_to_odom.header.stamp == msg->header.stamp)
  {
    const auto & rotation = msg->map_to_odom.transform.rotation;
    const Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    const auto committed = tf2::transformToEigen(msg->map_to_odom.transform);
    if (committed.matrix().allFinite() && std::isfinite(quaternion.norm()) &&
      std::abs(quaternion.norm() - 1.0) < 1e-6)
    {
      if (growing_map_->appendAccepted(growing_pending_source_, committed, true)) {
        rebuildGrowingTarget();
      }
      growing_fused_transform_ = committed;
      RCLCPP_DEBUG(get_logger(), "S3 committed growth observation=%llu candidate_tf_delta=%.6f",
        static_cast<unsigned long long>(growing_pending_sequence_),
        (committed.translation() - growing_pending_transform_.translation()).norm());
      result_t_ = previous_result_t_ = committed;
      growing_pending_source_.clear();
    }
  }
  if (!mapping_mode_ && has_accepted_alignment_ && !pending_confirmation_ && !odom_identity_latched_ &&
    msg->observation_sequence > 0U &&
    (msg->state == LS::STATE_TRACKING || msg->state == LS::STATE_CONFIRMED) &&
    msg->map_to_odom_valid && msg->map_to_odom.header.frame_id == map_frame_ &&
    msg->map_to_odom.child_frame_id == odom_frame_)
  {
    const auto & rotation = msg->map_to_odom.transform.rotation;
    const Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    const auto committed = tf2::transformToEigen(msg->map_to_odom.transform);
    if (committed.matrix().allFinite() && std::isfinite(quaternion.norm()) &&
      std::abs(quaternion.norm() - 1.0) < 1e-6)
    {
      result_t_ = previous_result_t_ = committed;
    }
  }
  const std::uint8_t previous = localization_state_;
  const bool epoch_changed = localization_epoch_ != msg->epoch;
  localization_state_ = msg->state;
  localization_epoch_ = msg->epoch;
  observation_silence_sec_ = msg->observation_silence_sec;
  last_status_receive_time_ = received_at;

  // RELOCALIZING continues pending evidence; CONFIRMED ends that episode once.
  // Soft DEGRADED also occurs while refreshing an accepted alignment on slow
  // simulation clocks (d12). Keep the same-epoch pair and its original steady
  // deadline; LOST, epoch and frontend boundaries still fence that evidence.
  const bool state_changed = previous != localization_state_;
  const bool pre_accept_pending = pending_confirmation_ && !has_accepted_alignment_;
  const bool recovery_boundary = state_changed &&
    ((localization_state_ == LS::STATE_LOST && !pre_accept_pending) ||
     localization_state_ == LS::STATE_UNINITIALIZED ||
     localization_state_ == LS::STATE_CONFIRMED ||
     (localization_state_ == LS::STATE_DEGRADED && !pending_confirmation_) ||
     (previous == LS::STATE_RELOCALIZING && localization_state_ == LS::STATE_TRACKING) ||
     (!pre_accept_pending && localization_state_ == LS::STATE_BOOTSTRAP));

  if (localization_state_ == LS::STATE_RELOCALIZING) {
    recovery_from_lost_ = previous == LS::STATE_LOST ||
      (previous == LS::STATE_RELOCALIZING && recovery_from_lost_);
  } else {
    recovery_from_lost_ = localization_state_ == LS::STATE_LOST;
  }
  // Epoch bumps alone must not abort an open cold-start confirmation episode.
  // gazebo196: 50x confirmation_recheck gated=1 but 0 accepts / 0 Confirmation
  // restart — each pending count=1 was wiped by epoch_changed invalidate before
  // the second consistent sample could land.
  if ((epoch_changed && !pre_accept_pending) || recovery_boundary) {
    invalidateRecovery();
  }

  if (localization_state_ == LS::STATE_TRACKING || localization_state_ == LS::STATE_CONFIRMED) {
    need_coarse_alignment_ = !has_accepted_alignment_;
  }
}

bool SmallGicpRelocalizationNode::shouldRunRegistration()
{
  if (!has_received_scan_ || scan_window_.pointCount() == 0) {
    return false;
  }

  if (scan_window_.frameCount() < static_cast<std::size_t>(accumulate_frames_)) {
    return false;
  }

  if (scan_window_.firstStamp()) {
    const double accumulation_age = accumulatedCloudAgeSeconds();
    if (max_accumulation_age_s_ > 0.0 && accumulation_age < max_accumulation_age_s_) {
      return false;
    }
  }

  // After a process stall, cached TRACKING can be older than fusion's LOST.
  // Wait for a fresh /localization/status before opening new lattice work.
  // Do NOT wipe an open confirmation episode: Gazebo odom floods make status
  // stamps non-monotonic ("stamp is not increasing"), so last_status_receive
  // stops advancing and a 1s stale timer previously invalidateRecovery()'d
  // every pending (gazebo193).
  if (
    follow_localization_status_ &&
    (!last_status_receive_time_ ||
     (status_stale_skip_registration_s_ > 0.0 &&
      (this->now() - *last_status_receive_time_).seconds() > status_stale_skip_registration_s_))) {
    return pending_confirmation_ && pending_confirmation_count_ > 0;
  }

  if (inInitialPoseForceWindow()) {
    return true;
  }

  if (pending_confirmation_) {
    return true;
  }


  // Until the first accepted alignment, keep trying (needed for multi_guess / cold start).
  if (!has_accepted_alignment_) {
    return true;
  }

  // LOST/DEGRADED: correctness over frequency — keep attempting recovery windows.
  if (force_registration_when_lost_ && isRecoveringLocalization()) {
    return true;
  }

  // A stationary chassis still receives new LiDAR measurements. Re-run the
  // complete registration and quality gates before fusion's accepted-observation
  // lease expires; never refresh status from an old scan alone.
  std::optional<double> last_accepted_scan_time_s;
  if (last_accepted_observation_scan_time_) {
    last_accepted_scan_time_s.emplace(last_accepted_observation_scan_time_->seconds());
  }
  // Fusion expires accepted evidence on steady time. A 1 s scan-time refresh
  // can wait ~15 wall seconds on this host before even opening its two-scan
  // confirmation. Start revalidation when that same steady lease needs it,
  // while retaining every scan-span, new-stamp and registration quality gate.
  const bool steady_refresh_due = last_accepted_observation_scan_time_ &&
    last_scan_time_ > *last_accepted_observation_scan_time_ &&
    std::isfinite(accepted_observation_refresh_interval_s_) &&
    accepted_observation_refresh_interval_s_ > 0.0 &&
    std::isfinite(observation_silence_sec_) &&
    observation_silence_sec_ >= accepted_observation_refresh_interval_s_;
  if (
    steady_refresh_due ||
    acceptedObservationRefreshDue(
      last_accepted_scan_time_s, last_scan_time_.seconds(),
      accepted_observation_refresh_interval_s_)) {
    return true;
  }

  auto current_robot_base_to_odom = getCurrentRobotBaseToOdom();
  if (!current_robot_base_to_odom) {
    return false;
  }

  if (!last_registration_robot_base_to_odom_) {
    last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    return false;
  }

  const double translation_delta = translationDeltaFromLastTrigger(*current_robot_base_to_odom);
  const double yaw_delta = yawDeltaFromLastTrigger(*current_robot_base_to_odom);
  return translation_delta >= std::max(0.0, min_registration_translation_delta_) ||
         yaw_delta >= std::max(0.0, min_registration_yaw_delta_);
}

double SmallGicpRelocalizationNode::accumulatedCloudAgeSeconds() const
{
  const auto first_stamp = scan_window_.firstStamp();
  if (!first_stamp) {
    return 0.0;
  }
  return std::max(0.0, static_cast<double>(last_scan_time_.nanoseconds() - *first_stamp) * 1e-9);
}

std::optional<Eigen::Isometry3d> SmallGicpRelocalizationNode::getCurrentRobotBaseToOdom() const
{
  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Isometry3d> SmallGicpRelocalizationNode::getOdomToRobotBase(
  const rclcpp::Time & stamp) const
{
  try {
    // Zero-wait only. A positive TF timeout needs the node's executor to arm the
    // wait timer; async completion and unit tests call this off-spin and would
    // otherwise block forever when the target frame is missing.
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, stamp, rclcpp::Duration::from_seconds(0.0));
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

double SmallGicpRelocalizationNode::translationDeltaFromLastTrigger(
  const Eigen::Isometry3d & current_robot_base_to_odom) const
{
  if (!last_registration_robot_base_to_odom_) {
    return 0.0;
  }
  return (current_robot_base_to_odom.translation() -
          last_registration_robot_base_to_odom_->translation())
    .norm();
}

double SmallGicpRelocalizationNode::yawDeltaFromLastTrigger(
  const Eigen::Isometry3d & current_robot_base_to_odom) const
{
  if (!last_registration_robot_base_to_odom_) {
    return 0.0;
  }
  return yawDistance(current_robot_base_to_odom, *last_registration_robot_base_to_odom_);
}

}  // namespace small_gicp_relocalization
