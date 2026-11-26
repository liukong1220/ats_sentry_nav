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

constexpr char kCandidateCsvHeader[] =
  "sweep,cursor,candidate_index,seed_x,seed_y,seed_yaw,guess_x,guess_y,guess_yaw,stage,converged,"
  "iterations,inliers,source_points,overlap,error,min_info_eigenvalue,condition_number,"
  "motion_residual,prior_deviation,score,result_x,result_y,result_yaw,reject_reason\n";

double normalizedRegistrationError(double error, std::size_t inliers)
{
  return inliers > 0 && std::isfinite(error) ? error / static_cast<double>(inliers)
                                             : std::numeric_limits<double>::infinity();
}

/// 协方差退化与 registration error 语义分离：solver 失败或误差非有限时退回保守
void fillInformationMetrics(
  const Eigen::Matrix<double, 6, 6> & information, double & min_eigenvalue,
  double & condition_number)
{
  min_eigenvalue = 0.0;
  condition_number = std::numeric_limits<double>::infinity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    0.5 * (information + information.transpose()));
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return;
  }
  min_eigenvalue = solver.eigenvalues().minCoeff();
  const double max_eigenvalue = solver.eigenvalues().maxCoeff();
  if (min_eigenvalue > 0.0 && std::isfinite(max_eigenvalue)) {
    condition_number = max_eigenvalue / min_eigenvalue;
  }
}

}  // namespace


SmallGicpRelocalizationNode::RegistrationAttempt SmallGicpRelocalizationNode::alignOnce(
  const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations,
  int num_threads) const
{
  if (!register_) {
    RegistrationAttempt attempt;
    attempt.reject_reason = "registration inputs unavailable";
    return attempt;
  }
  const bool use_submap = local_submap_enable_ && !isRecoveringLocalization() &&
    !preferMultiGuess() && submap_target_ && submap_target_tree_;
  const auto & active_target = use_submap ? submap_target_ : target_;
  const auto & active_tree = use_submap ? submap_target_tree_ : target_tree_;
  RegistrationAttempt attempt;
  if (!source_ || !source_tree_ || !active_target || !active_tree) {
    attempt.reject_reason = "registration inputs unavailable";
    return attempt;
  }
  register_->reduction.num_threads = num_threads;
  register_->rejector.max_dist_sq = max_dist_sq;
  register_->optimizer.max_iterations = max_iterations;
  const auto result = register_->align(*active_target, *source_, *active_tree, initial_guess);
  attempt.converged = result.converged;
  attempt.iterations = result.iterations;
  attempt.num_inliers = result.num_inliers;
  attempt.registration_error = normalizedRegistrationError(result.error, result.num_inliers);
  attempt.transform = result.T_target_source;
  attempt.information = result.H;
  attempt.overlap_ratio = source_->empty()
    ? 0.0
    : static_cast<double>(result.num_inliers) / static_cast<double>(source_->size());
  fillInformationMetrics(
    result.H, attempt.min_information_eigenvalue, attempt.information_condition_number);
  return attempt;
}

SmallGicpRelocalizationNode::RegistrationAttempt SmallGicpRelocalizationNode::alignOnceOn(
  const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations,
  const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
  GicpRegistration & registration, int num_threads) const
{
  RegistrationAttempt attempt;
  if (!source || !source_tree || !target_ || !target_tree_) {
    attempt.reject_reason = "registration inputs unavailable";
    return attempt;
  }

  registration.reduction.num_threads = num_threads;
  registration.rejector.max_dist_sq = max_dist_sq;
  registration.optimizer.max_iterations = max_iterations;

  const auto result = registration.align(*target_, *source, *target_tree_, initial_guess);
  attempt.converged = result.converged;
  attempt.iterations = result.iterations;
  attempt.num_inliers = result.num_inliers;
  attempt.registration_error = normalizedRegistrationError(result.error, result.num_inliers);
  attempt.transform = result.T_target_source;
  attempt.information = result.H;
  attempt.overlap_ratio =
    source->empty() ? 0.0
                    : static_cast<double>(result.num_inliers) / static_cast<double>(source->size());
  fillInformationMetrics(
    result.H, attempt.min_information_eigenvalue, attempt.information_condition_number);
  return attempt;
}

CandidateHardGates SmallGicpRelocalizationNode::hardGates(bool allow_unconverged) const
{
  CandidateHardGates gates;
  gates.min_inliers = min_inliers_;
  gates.max_registration_error = max_registration_error_;
  gates.min_overlap_ratio = min_overlap_ratio_;
  gates.min_information_eigenvalue = min_information_eigenvalue_;
  gates.max_information_condition_number = max_information_condition_number_;
  gates.allow_unconverged = allow_unconverged;
  return gates;
}

bool SmallGicpRelocalizationNode::passesQualityGates(
  RegistrationAttempt & attempt, bool allow_unconverged, GateStage stage) const
{
  CandidateEvidence evidence;
  evidence.converged = attempt.converged;
  evidence.transform_finite = attempt.transform.matrix().allFinite();
  evidence.num_inliers = attempt.num_inliers;
  evidence.registration_error = attempt.registration_error;
  evidence.overlap_ratio = attempt.overlap_ratio;
  evidence.min_information_eigenvalue = attempt.min_information_eigenvalue;
  evidence.information_condition_number = attempt.information_condition_number;

  // 放宽 converged 仍要求内点数远超门限，避免把明显失败的优化放进来。
  const bool relax_permitted =
    allow_unconverged && static_cast<int>(attempt.num_inliers) >= (min_inliers_ * 2);
  const bool screen_stage = stage == GateStage::kScreen;
  CandidateHardGates gates = hardGates(relax_permitted);
  if (screen_stage) {
    gates = screenStageGates(gates);
    // The screening cloud is intentionally much sparser than the full source
    // cloud.  Applying the full-cloud min_inliers (150 in Gazebo) made every
    // candidate impossible (the 0.70 m screen had about 40--60 points), so
    // coarse screening uses a separately bounded evidence floor.  Fine-stage
    // acceptance still uses the configured full-cloud gate unchanged.
    const int auto_screen_min = std::max(3, min_inliers_ / 4);
    gates.min_inliers = multi_guess_screen_min_inliers_ > 0
      ? multi_guess_screen_min_inliers_ : auto_screen_min;
  }
  const std::string reason = candidateRejectReason(evidence, gates);
  if (!reason.empty()) {
    attempt.ok = false;
    attempt.reject_reason = reason;
    return false;
  }
  if (!screen_stage && relax_permitted && !attempt.converged) {
    attempt.stage += "+sim_relax";
  }
  attempt.reject_reason.clear();
  attempt.ok = true;
  return true;
}

CandidateEvidence SmallGicpRelocalizationNode::attemptEvidence(
  const RegistrationAttempt & attempt, const Eigen::Isometry3d & seed,
  const std::optional<ConfirmationSample> & reference,
  const std::optional<Eigen::Isometry3d> & current_odom_to_base) const
{
  CandidateEvidence evidence;
  evidence.converged = attempt.converged;
  evidence.transform_finite = attempt.transform.matrix().allFinite();
  evidence.num_inliers = attempt.num_inliers;
  evidence.registration_error = attempt.registration_error;
  evidence.overlap_ratio = attempt.overlap_ratio;
  evidence.min_information_eigenvalue = attempt.min_information_eigenvalue;
  evidence.information_condition_number = attempt.information_condition_number;
  evidence.prior_deviation =
    (attempt.transform.translation() - seed.translation()).head<2>().norm();
  if (reference && current_odom_to_base) {
    const ConfirmationMotionResidual residual = confirmationMotionResidual(
      reference->map_to_odom, reference->odom_to_base, attempt.transform, *current_odom_to_base);
    if (residual.valid) {
      evidence.motion_residual = residual.translation;
    }
  }
  return evidence;
}

CandidateLatticeConfig SmallGicpRelocalizationNode::latticeConfig() const
{
  CandidateLatticeConfig config;
  config.search_half_xy = multi_guess_search_half_xy_;
  config.step_xy = multi_guess_step_xy_;
  config.step_yaw = multi_guess_step_yaw_;
  config.z_offsets = multi_guess_z_candidates_;
  return config;
}

void SmallGicpRelocalizationNode::writeCandidateDiagnostics(const std::string & rows) const
{
  if (multi_guess_candidate_log_path_.empty() || rows.empty()) {
    return;
  }
  std::error_code error_code;
  const bool exists = std::filesystem::exists(multi_guess_candidate_log_path_, error_code);
  std::ofstream stream(multi_guess_candidate_log_path_, std::ios::app);
  if (!stream) {
    RCLCPP_ERROR(
      this->get_logger(), "【重定位诊断错误】无法打开候选诊断文件 '%s'",
      multi_guess_candidate_log_path_.c_str());
    return;
  }
  if (!exists) {
    stream << kCandidateCsvHeader;
  }
  stream << rows;
}

SmallGicpRelocalizationNode::MultiGuessOutcome
SmallGicpRelocalizationNode::runMultiGuessAlignmentOn(
  const MultiGuessRequest & request, const std::atomic<bool> & cancel_flag)
{
  MultiGuessOutcome outcome;
  outcome.generation = request.generation;
  outcome.cursor = request.cursor;
  outcome.next_cursor = request.cursor;
  outcome.confirmation_recheck = request.confirmation_recheck;
  outcome.best.reject_reason = "multi_guess produced no valid candidate";
  if (!request.source || !request.source_tree || !target_ || !target_tree_) {
    outcome.best.reject_reason = "registration inputs unavailable";
    return outcome;
  }

  // Confirmation must re-hit the SAME pending hypothesis on a new scan.
  // Expanding the LOST lattice here rediscovers unrelated local minima
  // (straight186: dyaw~0.7-1.4 Confirmation restart) and prevents accept.
  if (request.confirmation_recheck) {
    const auto start = std::chrono::steady_clock::now();
    GicpRegistration local_register;
    outcome.lattice.generated = 1;
    outcome.lattice.min_x = request.seed.translation().x();
    outcome.lattice.max_x = request.seed.translation().x();
    outcome.lattice.min_y = request.seed.translation().y();
    outcome.lattice.max_y = request.seed.translation().y();
    outcome.lattice.max_radius = 0.0;
    outcome.lattice.rings = 0;
    outcome.evaluated = 1;
    outcome.skipped = 0;
    outcome.wrapped = false;
    outcome.next_cursor = request.cursor;

    const float corr =
      fine_alignment_enabled_ ? fine_max_dist_sq_ : max_dist_sq_;
    const int iters =
      fine_alignment_enabled_ ? fine_max_iterations_ : coarse_max_iterations_;
    RegistrationAttempt attempt = alignOnceOn(
      request.seed, corr, iters, request.source, request.source_tree, local_register,
      request.num_threads);
    attempt.stage = "confirmation_recheck";
    if (fine_alignment_enabled_) {
      ++outcome.refined;
    }
    const bool accepted =
      passesQualityGates(attempt, request.allow_unconverged, GateStage::kAccept);
    const CandidateEvidence evidence =
      attemptEvidence(attempt, request.seed, request.reference, request.current_odom_to_base);
    attempt.motion_residual = evidence.motion_residual;
    attempt.prior_deviation = evidence.prior_deviation;
    const CandidateScore score = scoreCandidate(evidence, score_weights_);
    attempt.score = accepted ? score.total : std::numeric_limits<double>::infinity();
    // Fine alignment from the pending seed can still walk into a neighboring
    // basin (straight187: dxy~0.5 while confirmation_tol=0.15). That must not
    // count as a confirmation sample — it would clear the lattice pending.
    if (accepted && std::isfinite(attempt.score)) {
      const double dxy = (attempt.transform.translation() - request.seed.translation()).head<2>().norm();
      const double dyaw = yawDistance(attempt.transform, request.seed);
      if (dxy > confirmation_translation_tolerance_ || dyaw > confirmation_yaw_tolerance_) {
        attempt.ok = false;
        attempt.reject_reason = "confirmation_recheck drifted from pending seed";
        attempt.score = std::numeric_limits<double>::infinity();
        outcome.best = attempt;
      } else {
        ++outcome.gated;
        outcome.best = attempt;
        outcome.selection.has_best = true;
        outcome.selection.best_index = 0;
        outcome.selection.best_score = attempt.score;
        outcome.selection.alternative_score = std::numeric_limits<double>::infinity();
        outcome.selection.score_margin = std::numeric_limits<double>::infinity();
        outcome.selection.ambiguous = false;
      }
    } else {
      outcome.best = attempt;
      outcome.best.ok = false;
      if (outcome.best.reject_reason.empty()) {
        outcome.best.reject_reason = "confirmation_recheck failed quality gates";
      }
    }
    outcome.elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return outcome;
  }

  // 冷启动种子优先（报告 5.1.4 B1）：init_pose 是已知的，先单独对它做粗配准加精配准，使用同样的
  // 硬门，不占格点预算、不受格点截止约束。通过就直接作为结果，之后仍走冷启动先验门和两次一致确认；
  // 不通过就记录原因，格点搜索照旧（预算从这里之后才开始计）。
  bool seed_stage_ran = false;
  if (request.seed_first) {
    seed_stage_ran = true;
    outcome.seed_stage_ran = true;
    const auto seed_started = std::chrono::steady_clock::now();
    GicpRegistration seed_register;
    RegistrationAttempt seed_attempt = alignOnceOn(
      request.seed, max_dist_sq_, coarse_max_iterations_, request.source, request.source_tree,
      seed_register, request.num_threads);
    seed_attempt.stage = "cold_seed";
    bool seed_ok = passesQualityGates(
      seed_attempt, request.allow_unconverged,
      fine_alignment_enabled_ ? GateStage::kScreen : GateStage::kAccept);
    if (seed_ok && fine_alignment_enabled_ && !cancel_flag.load()) {
      seed_attempt = alignOnceOn(
        seed_attempt.transform, fine_max_dist_sq_, fine_max_iterations_, request.source,
        request.source_tree, seed_register, request.num_threads);
      seed_attempt.stage = "cold_seed";
      seed_ok = passesQualityGates(seed_attempt, request.allow_unconverged, GateStage::kAccept);
    }
    const CandidateEvidence seed_evidence =
      attemptEvidence(seed_attempt, request.seed, request.reference, request.current_odom_to_base);
    seed_attempt.motion_residual = seed_evidence.motion_residual;
    seed_attempt.prior_deviation = seed_evidence.prior_deviation;
    seed_attempt.score =
      seed_ok ? scoreCandidate(seed_evidence, score_weights_).total
              : std::numeric_limits<double>::infinity();
    const double seed_elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - seed_started).count();
    char seed_line[640];
    std::snprintf(
      seed_line, sizeof(seed_line),
      "【重定位冷启动种子】判决=%s(%s) 用时=%.3fs 初值=(%.3f,%.3f,%.3f) converged=%s 内点=%zu "
      "重叠率=%.3f 误差=%.6f 先验偏差=%.3f 结果=(%.3f,%.3f,%.3f)",
      seed_ok ? "通过" : "拒绝", seed_ok ? "-" : seed_attempt.reject_reason.c_str(), seed_elapsed_s,
      request.seed.translation().x(), request.seed.translation().y(), yawOf(request.seed),
      seed_attempt.converged ? "true" : "false", seed_attempt.num_inliers,
      seed_attempt.overlap_ratio, seed_attempt.registration_error, seed_attempt.prior_deviation,
      seed_attempt.transform.translation().x(), seed_attempt.transform.translation().y(),
      yawOf(seed_attempt.transform));
    if (seed_ok) {
      RCLCPP_INFO(this->get_logger(), "%s", seed_line);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", seed_line);
    }
    if (cancel_flag.load()) {
      outcome.best.ok = false;
      outcome.best.reject_reason = "multi_guess cancelled";
      outcome.elapsed_s = seed_elapsed_s;
      return outcome;
    }
    if (seed_ok && std::isfinite(seed_attempt.score)) {
      // Same bookkeeping as the confirmation recheck: one hypothesis, no lattice progress.
      outcome.lattice.generated = 1;
      outcome.lattice.min_x = outcome.lattice.max_x = request.seed.translation().x();
      outcome.lattice.min_y = outcome.lattice.max_y = request.seed.translation().y();
      outcome.lattice.max_radius = 0.0;
      outcome.lattice.rings = 0;
      outcome.evaluated = 1;
      outcome.refined = fine_alignment_enabled_ ? 1 : 0;
      outcome.gated = 1;
      outcome.skipped = 0;
      outcome.wrapped = false;
      outcome.next_cursor = request.cursor;
      outcome.best = seed_attempt;
      outcome.selection.has_best = true;
      outcome.selection.best_index = 0;
      outcome.selection.best_score = seed_attempt.score;
      outcome.selection.alternative_score = std::numeric_limits<double>::infinity();
      outcome.selection.score_margin = std::numeric_limits<double>::infinity();
      outcome.selection.ambiguous = false;
      outcome.elapsed_s = seed_elapsed_s;
      return outcome;
    }
  }

  auto candidates =
    buildLayeredCandidateLattice(request.seed, latticeConfig(), &outcome.lattice);
  if (!request.descriptor_seeds.empty()) {
    std::vector<Eigen::Isometry3d> combined;
    combined.reserve(request.descriptor_seeds.size() + candidates.size());
    for (const auto & sc_seed : request.descriptor_seeds) {
      combined.push_back(sc_seed);
    }
    for (const auto & cand : candidates) {
      bool dup = false;
      for (const auto & sc_seed : request.descriptor_seeds) {
        if (sameCandidate(cand, sc_seed, 0.20, 0.15)) {
          dup = true;
          break;
        }
      }
      if (!dup) {
        combined.push_back(cand);
      }
    }
    candidates.swap(combined);
    outcome.lattice.generated = candidates.size();
  }
  const std::size_t total = candidates.size();
  if (total == 0) {
    return outcome;
  }
  std::size_t cursor = request.cursor % total;
  // The seed stage already evaluated lattice candidate 0 (same input, deterministic), so the
  // lattice does not spend budget on it again.
  if (
    seed_stage_ran && cursor == 0 && total > 1 &&
    candidates[0].matrix().isApprox(request.seed.matrix(), 1e-9)) {
    cursor = 1;
  }
  outcome.cursor = cursor;
  const std::size_t budget_count =
    static_cast<std::size_t>(std::max(1, multi_guess_max_candidates_per_scan_));
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(multi_guess_time_budget_s_));

  // coarse 筛选跑稀疏云以提高每次扫描的候选吞吐；fine 与验收始终用完整源云。
  const PointCovarianceCloud::Ptr & screen_source =
    request.screen_source ? request.screen_source : request.source;
  const std::shared_ptr<PointKdTree> & screen_tree =
    request.screen_source && request.screen_source_tree ? request.screen_source_tree
                                                        : request.source_tree;

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), log_throttle_ms_,
    "【重定位多猜测】扫描=%s 游标=%zu/%zu 预算=%zu/%.2fs 初值=(%.3f,%.3f,%.3f) "
    "覆盖范围 x=[%.2f,%.2f] y=[%.2f,%.2f] 最大半径=%.2f 环数=%zu "
    "筛选点=%zu 输入点=%zu",
    std::to_string(request.sweep).c_str(), cursor, total, budget_count, multi_guess_time_budget_s_,
    request.seed.translation().x(), request.seed.translation().y(), yawOf(request.seed),
    outcome.lattice.min_x, outcome.lattice.max_x, outcome.lattice.min_y, outcome.lattice.max_y,
    outcome.lattice.max_radius, outcome.lattice.rings, screen_source ? screen_source->size() : 0,
    request.source_points);

  GicpRegistration local_register;
  std::vector<RankedCandidate> ranked;
  std::vector<RegistrationAttempt> retained;
  std::string diagnostics;

  const bool log_rows = multi_guess_log_candidates_ || !multi_guess_candidate_log_path_.empty();
  const auto logRow = [&](
                        std::size_t index, const Eigen::Isometry3d & guess,
                        const RegistrationAttempt & attempt, const std::string & verdict) {
    if (!log_rows) {
      return;
    }
    std::ostringstream row;
    row.setf(std::ios::fixed, std::ios::floatfield);
    row.precision(6);
    row << request.sweep << ',' << cursor << ',' << index << ',' << request.seed.translation().x()
        << ',' << request.seed.translation().y() << ',' << yawOf(request.seed) << ','
        << guess.translation().x() << ',' << guess.translation().y() << ',' << yawOf(guess) << ','
        << attempt.stage << ',' << (attempt.converged ? 1 : 0) << ',' << attempt.iterations << ','
        << attempt.num_inliers << ',' << request.source_points << ',' << attempt.overlap_ratio
        << ',' << attempt.registration_error << ',' << attempt.min_information_eigenvalue << ','
        << attempt.information_condition_number << ',' << attempt.motion_residual << ','
        << attempt.prior_deviation << ',' << attempt.score << ','
        << attempt.transform.translation().x() << ',' << attempt.transform.translation().y() << ','
        << yawOf(attempt.transform) << ',' << verdict << '\n';
    diagnostics += row.str();
    if (multi_guess_log_candidates_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), log_throttle_ms_,
        "【重定位候选%zu】阶段=%s 初值=(%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f) 内点=%zu "
        "重叠率=%.3f 误差=%.6f 最小特征值=%.4g 条件数=%.4g 运动=%.3f 先验=%.3f 得分=%.4f %s",
        index, attempt.stage.c_str(), guess.translation().x(), guess.translation().y(),
        yawOf(guess), attempt.transform.translation().x(), attempt.transform.translation().y(),
        yawOf(attempt.transform), attempt.num_inliers, attempt.overlap_ratio,
        attempt.registration_error, attempt.min_information_eigenvalue,
        attempt.information_condition_number, attempt.motion_residual, attempt.prior_deviation,
        attempt.score, verdict.c_str());
    }
  };

  // 第一遍：在稀疏筛选云上粗筛尽量多的候选。这一级只排序，不验收。
  struct ScreenedCandidate
  {
    double score;
    std::size_t index;
    RegistrationAttempt attempt;
  };
  std::vector<ScreenedCandidate> screened;
  std::size_t consumed = 0;
  for (std::size_t step = 0; step < total; ++step) {
    if (cancel_flag.load()) {
      outcome.best.ok = false;
      outcome.best.reject_reason = "multi_guess cancelled";
      outcome.evaluated = consumed;
      outcome.skipped = total - consumed;
      outcome.elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      return outcome;
    }
    const auto screening_now = std::chrono::steady_clock::now();
    const bool reserve_for_fine = !screened.empty() &&
      screening_now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(multi_guess_fine_time_reserve_s_)) >= deadline;
    if (outcome.evaluated >= budget_count || reserve_for_fine ||
        multiGuessDeadlineExpired(screening_now, deadline)) {
      outcome.budget_exhausted = true;
      break;
    }
    const std::size_t index = (cursor + step) % total;
    const Eigen::Isometry3d & guess = candidates[index];
    ++consumed;
    ++outcome.evaluated;

    RegistrationAttempt attempt = alignOnceOn(
      guess, max_dist_sq_, coarse_max_iterations_, screen_source, screen_tree, local_register,
      request.num_threads);
    attempt.stage = "multi_guess_screen";
    // 截断的 coarse 只做筛选，不认定收敛；fine 级才是验收权威。
    const bool screened_ok =
      passesQualityGates(attempt, request.allow_unconverged, GateStage::kScreen);
    const CandidateEvidence screen_evidence =
      attemptEvidence(attempt, request.seed, request.reference, request.current_odom_to_base);
    attempt.motion_residual = screen_evidence.motion_residual;
    attempt.prior_deviation = screen_evidence.prior_deviation;
    const CandidateScore screen_score = scoreCandidate(screen_evidence, score_weights_);
    attempt.score = screened_ok ? screen_score.total : std::numeric_limits<double>::infinity();
    logRow(index, guess, attempt, screened_ok ? std::string("screened") : attempt.reject_reason);
    if (!screened_ok || !std::isfinite(attempt.score)) {
      continue;
    }
    screened.push_back(ScreenedCandidate{attempt.score, index, attempt});
  }

  // 第二遍：只把筛选分数最好的少数候选送进 fine 与完整硬门。
  // 每次扫描的精配准次数因此有上界，候选覆盖速度不再被 fine 成本吞掉。
  std::sort(
    screened.begin(), screened.end(),
    [](const ScreenedCandidate & lhs, const ScreenedCandidate & rhs) {
      return lhs.score < rhs.score;
    });
  const std::size_t refine_budget =
    static_cast<std::size_t>(std::max(1, multi_guess_max_fine_per_scan_));
  const std::size_t refine_count =
    fine_alignment_enabled_ ? std::min(screened.size(), refine_budget) : screened.size();
  // 诊断：筛选阶段已用掉多少预算。fine 只剩 budget - screening_elapsed，
  // 与 multi_guess.fine_time_reserve_s 比较即可看出 fine 是否有机会在截止前完成。
  RCLCPP_INFO(
    this->get_logger(),
    "【重定位筛选完成】通过筛选=%zu/%zu 送入精配准=%zu 筛选用时=%.3fs 预算=%.2fs 剩余=%.3fs "
    "fine预留=%.2fs",
    screened.size(), consumed, refine_count,
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(),
    multi_guess_time_budget_s_,
    std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count(),
    multi_guess_fine_time_reserve_s_);
  for (std::size_t i = 0; i < refine_count; ++i) {
    if (cancel_flag.load()) {
      outcome.best.ok = false;
      outcome.best.reject_reason = "multi_guess cancelled";
      outcome.evaluated = consumed;
      outcome.skipped = total - consumed;
      outcome.elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      return outcome;
    }
    // A coarse pass may consume the configured wall budget.  The screening
    // loop reserves time for this first fine pass, so do not discard a valid
    // screened candidate merely because the reserve boundary was reached.
    if (i > 0 && multiGuessDeadlineExpired(std::chrono::steady_clock::now(), deadline)) {
      outcome.budget_exhausted = true;
      break;
    }
    const ScreenedCandidate & candidate = screened[i];
    RegistrationAttempt attempt = candidate.attempt;
    const auto fine_started = std::chrono::steady_clock::now();
    if (fine_alignment_enabled_) {
      attempt = alignOnceOn(
        candidate.attempt.transform, fine_max_dist_sq_, fine_max_iterations_, request.source,
        request.source_tree, local_register, request.num_threads);
      attempt.stage = "multi_guess_fine";
      ++outcome.refined;
    }
    const auto fine_finished = std::chrono::steady_clock::now();
    if (multiGuessDeadlineExpired(fine_finished, deadline)) {
      // 诊断（不改判决）：精配准在截止时间之后才完成，结果被整体丢弃。此前这里没有任何输出，
      // retained 为空时节点只报 budget_exhausted，看不出是候选质量差还是只是慢。
      // 这里把"如果没有被丢弃，质量门会怎么判"打出来，并写进候选 CSV。
      RegistrationAttempt probe = attempt;
      const bool would_accept =
        passesQualityGates(probe, request.allow_unconverged, GateStage::kAccept);
      const CandidateEvidence probe_evidence =
        attemptEvidence(probe, request.seed, request.reference, request.current_odom_to_base);
      probe.motion_residual = probe_evidence.motion_residual;
      probe.prior_deviation = probe_evidence.prior_deviation;
      probe.score = would_accept ? scoreCandidate(probe_evidence, score_weights_).total
                                 : std::numeric_limits<double>::infinity();
      const auto seconds_between = [](auto from, auto to) {
        return std::chrono::duration<double>(to - from).count();
      };
      RCLCPP_WARN(
        this->get_logger(),
        "【重定位精配准丢弃】候选=%zu 精配准用时=%.3fs 本次多猜测累计=%.3fs 预算=%.2fs "
        "超出=%.3fs 若不丢弃质量门判决=%s(%s) converged=%s 内点=%zu 重叠率=%.3f 误差=%.6f "
        "先验偏差=%.3f 结果=(%.3f,%.3f,%.3f)",
        candidate.index, seconds_between(fine_started, fine_finished),
        seconds_between(start, fine_finished), multi_guess_time_budget_s_,
        seconds_between(deadline, fine_finished), would_accept ? "通过" : "拒绝",
        would_accept ? "-" : probe.reject_reason.c_str(), probe.converged ? "true" : "false",
        probe.num_inliers, probe.overlap_ratio, probe.registration_error, probe.prior_deviation,
        probe.transform.translation().x(), probe.transform.translation().y(),
        yawOf(probe.transform));
      logRow(
        candidate.index, candidates[candidate.index], probe,
        would_accept ? std::string("discarded_late(would_accept)")
                     : "discarded_late(" + probe.reject_reason + ")");
      outcome.budget_exhausted = true;
      break;
    }
    const bool accepted =
      passesQualityGates(attempt, request.allow_unconverged, GateStage::kAccept);
    const CandidateEvidence evidence =
      attemptEvidence(attempt, request.seed, request.reference, request.current_odom_to_base);
    attempt.motion_residual = evidence.motion_residual;
    attempt.prior_deviation = evidence.prior_deviation;
    const CandidateScore score = scoreCandidate(evidence, score_weights_);
    attempt.score = accepted ? score.total : std::numeric_limits<double>::infinity();
    logRow(
      candidate.index, candidates[candidate.index], attempt,
      accepted ? std::string("accepted") : attempt.reject_reason);
    // 每个进入精配准的候选都打印判决（每次多猜测最多 max_fine_per_scan 行），
    // 不依赖 multi_guess.log_candidates：精配准阶段的拒绝原因必须留在节点日志里。
    // 拒绝用 WARN（runner 默认 log_level=warn 也能看到），通过用 INFO。
    char verdict_line[640];
    std::snprintf(
      verdict_line, sizeof(verdict_line),
      "【重定位精配准】候选=%zu 判决=%s(%s) 精配准用时=%.3fs 本次多猜测累计=%.3fs "
      "converged=%s 内点=%zu 重叠率=%.3f 误差=%.6f 先验偏差=%.3f 结果=(%.3f,%.3f,%.3f)",
      candidate.index, accepted ? "通过" : "拒绝", accepted ? "-" : attempt.reject_reason.c_str(),
      std::chrono::duration<double>(fine_finished - fine_started).count(),
      std::chrono::duration<double>(fine_finished - start).count(),
      attempt.converged ? "true" : "false", attempt.num_inliers, attempt.overlap_ratio,
      attempt.registration_error, attempt.prior_deviation, attempt.transform.translation().x(),
      attempt.transform.translation().y(), yawOf(attempt.transform));
    if (accepted) {
      RCLCPP_INFO(this->get_logger(), "%s", verdict_line);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", verdict_line);
    }
    if (!accepted || !std::isfinite(attempt.score)) {
      continue;
    }
    ++outcome.gated;
    ranked.push_back(RankedCandidate{attempt.score, attempt.transform});
    retained.push_back(attempt);
  }

  outcome.skipped = total - consumed;
  outcome.wrapped = cursor + consumed >= total;
  outcome.next_cursor = total > 0 ? (cursor + consumed) % total : 0;
  outcome.elapsed_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  // Budget exhaustion may interrupt further screening, but must not discard
  // candidates that already passed the accept gate on this scan.
  if (retained.empty() &&
      multiGuessDeadlineExpired(std::chrono::steady_clock::now(), deadline)) {
    outcome.budget_exhausted = true;
    outcome.best.ok = false;
    outcome.best.reject_reason = "multi_guess budget_exhausted";
    writeCandidateDiagnostics(diagnostics);
    return outcome;
  }
  outcome.selection = selectCandidate(ranked, ambiguity_);
  if (!outcome.selection.has_best) {
    outcome.best.ok = false;
    writeCandidateDiagnostics(diagnostics);
    return outcome;
  }

  outcome.best = retained[outcome.selection.best_index];
  outcome.best.score = outcome.selection.best_score;
  outcome.best.alternative_score = outcome.selection.alternative_score;
  outcome.best.score_margin = outcome.selection.score_margin;
  outcome.best.ambiguous = outcome.selection.ambiguous;
  if (outcome.selection.ambiguous) {
    outcome.best.ok = false;
    std::ostringstream reason;
    reason.setf(std::ios::fixed, std::ios::floatfield);
    reason.precision(4);
    reason << "ambiguous candidates (best=" << outcome.selection.best_score
           << " second=" << outcome.selection.alternative_score
           << " margin=" << outcome.selection.score_margin << " < " << ambiguity_.min_score_margin
           << ")";
    outcome.best.reject_reason = reason.str();
  }
  writeCandidateDiagnostics(diagnostics);
  outcome.elapsed_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  // align() cannot be interrupted. Mark over-budget, but keep a completed gated
  // best so confirmation can progress (straight186: gated>=1 + budget_exhausted).
  if (multiGuessDeadlineExpired(std::chrono::steady_clock::now(), deadline)) {
    outcome.budget_exhausted = true;
  }
  if (cancel_flag.load()) {
    outcome.best.ok = false;
    outcome.best.reject_reason = "multi_guess cancelled";
  }
  return outcome;
}

SmallGicpRelocalizationNode::RegistrationAttempt
SmallGicpRelocalizationNode::runCoarseFineAlignment(
  const Eigen::Isometry3d & initial_guess, bool allow_unconverged)
{
  const int num_threads = registration_num_threads_;
  const bool coarse_allowed_by_state = !follow_localization_status_ ||
                                       isRecoveringLocalization() ||
                                       inInitialPoseForceWindow();
  const bool coarse_required = !fine_alignment_enabled_ || !coarse_first_window_only_ ||
                               !has_accepted_alignment_ ||
                               (need_coarse_alignment_ && coarse_allowed_by_state);

  RegistrationAttempt attempt;
  if (coarse_required || !fine_alignment_enabled_) {
    attempt = alignOnce(initial_guess, max_dist_sq_, coarse_max_iterations_, num_threads);
    attempt.stage = fine_alignment_enabled_ ? "coarse" : "single";
    // coarse 后面还有 fine 时按筛选级放行 converged，否则 coarse 自己承担验收。
    const GateStage coarse_stage =
      fine_alignment_enabled_ ? GateStage::kScreen : GateStage::kAccept;
    if (!passesQualityGates(attempt, allow_unconverged, coarse_stage)) {
      return attempt;
    }
    if (fine_alignment_enabled_) {
      auto fine =
        alignOnce(attempt.transform, fine_max_dist_sq_, fine_max_iterations_, num_threads);
      fine.stage = "coarse+fine";
      passesQualityGates(fine, allow_unconverged, GateStage::kAccept);
      return fine;
    }
    return attempt;
  }

  attempt = alignOnce(initial_guess, fine_max_dist_sq_, fine_max_iterations_, num_threads);
  attempt.stage = "fine_only";
  passesQualityGates(attempt, allow_unconverged, GateStage::kAccept);
  return attempt;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  expireConfirmation();
  drainAsyncMultiGuessResult();

  if (!shouldRunRegistration()) {
    return;
  }

  if (scan_window_.pointCount() == 0) {
    return;
  }

  if (scan_window_.pointCount() < static_cast<std::size_t>(std::max(0, min_source_points_))) {
    return;
  }

  // 跟踪期单线程；LOST 多猜测与恢复中的粗配准才用 recovery_num_threads（报告 9.3 第 5 项）。
  // 这里的判定是之后 preferMultiGuess() 的超集：multi_guess 模式下它只多看 need_coarse_alignment_，
  // 而下面恰好在恢复期把它置真。
  const bool recovery = isRecoveringLocalization() || preferMultiGuess();
  registration_num_threads_ = recovery ? recovery_num_threads_ : num_threads_;

  // Preprocess the accumulated window once; coarse and fine reuse the same source.
  preprocessAccumulatedSource(registration_num_threads_);
  if (!source_ || !source_tree_ || source_->empty()) {
    return;
  }
  if (mapping_mode_ && !target_) {
    GrowingMapStore::Points points;
    for (const auto & p : source_->points) {points.push_back({p.x, p.y, p.z});}
    if (growing_map_->seed(points, growing_seed_transform_)) {rebuildGrowingTarget();}
    // The seed window is never registered against itself and never publishes
    // an accepted observation. A disjoint later window must pass every gate.
    clearAccumulation();
    return;
  }

  if (isRecoveringLocalization()) {
    need_coarse_alignment_ = true;
  }

  // LOST multi_guess must not block the ROS callback thread (/initialpose, status, scans).
  if (preferMultiGuess()) {
    startAsyncMultiGuess();
    clearAccumulation();
    return;
  }

  if (local_submap_enable_ && !isRecoveringLocalization()) {
    if (const auto odom_to_base = getOdomToRobotBase(last_scan_time_)) {
      const Eigen::Isometry3d est_map_base = previous_result_t_ * *odom_to_base;
      updateTrackingSubmap(est_map_base.translation().head<2>());
    }
  }

  RegistrationAttempt attempt = runCoarseFineAlignment(previous_result_t_, simRelaxAllowed());
  const CandidateEvidence evidence = attemptEvidence(
    attempt, previous_result_t_, last_hypothesis_, getOdomToRobotBase(last_scan_time_));
  attempt.motion_residual = evidence.motion_residual;
  attempt.prior_deviation = evidence.prior_deviation;
  attempt.score = attempt.ok ? scoreCandidate(evidence, score_weights_).total
                             : std::numeric_limits<double>::infinity();
  handleRegistrationAttempt(attempt, last_scan_time_, source_->size());
  clearAccumulation();
}

void SmallGicpRelocalizationNode::startAsyncMultiGuess()
{
  bool expected = false;
  if (!multi_guess_running_.compare_exchange_strong(expected, true)) {
    return;
  }
  cancel_multi_guess_.store(false);

  MultiGuessRequest request;
  request.generation = relocalization_generation_.current();
  request.source = source_;
  request.source_tree = source_tree_;
  request.screen_source = screen_source_;
  request.screen_source_tree = screen_source_tree_;
  request.seed = previous_result_t_;
  request.cursor = multi_guess_cursor_;
  // Pending confirmation must refine THAT hypothesis only. Cursor/lattice search
  // would evaluate unrelated cells and fail confirmation transform matching.
  if (pending_confirmation_ && pending_confirmation_count_ > 0) {
    request.seed = pending_confirmation_->map_to_odom;
    request.cursor = 0;
    request.confirmation_recheck = true;
  }
  // Cold start: the known init_pose is tried by itself before the lattice (report 5.1.4 B1).
  // Bounded by cold_start_seed_attempts so a wrong init_pose cannot loop here forever.
  if (
    !request.confirmation_recheck && !has_accepted_alignment_ &&
    cold_start_seed_tried_ < cold_start_seed_attempts_) {
    // The attempt is counted when its result is adopted (drainAsyncMultiGuessResult), not here:
    // a result fenced off by a generation bump (state UNINITIALIZED -> BOOTSTRAP during the run)
    // or dropped while a confirmation is pending must not use up one of the few tries (5.1.4 B3).
    request.seed_first = true;
  }
  request.sweep = multi_guess_sweep_;
  request.allow_unconverged = simRelaxAllowed();
  request.source_points = source_ ? source_->size() : 0;
  request.num_threads = registration_num_threads_;
  request.current_odom_to_base = getOdomToRobotBase(last_scan_time_);
  if (scan_context_enable_ && !scan_context_db_.empty() &&
    !request.confirmation_recheck && !request.seed_first &&
    request.current_odom_to_base && source_ && !source_->empty())
  {
    const Eigen::Isometry3d base_from_odom = request.current_odom_to_base->inverse();
    PointCovarianceCloud local_cloud;
    local_cloud.points.reserve(source_->points.size());
    for (const auto & pt : source_->points) {
      const Eigen::Vector3d pb =
        base_from_odom * Eigen::Vector3d(pt.x, pt.y, pt.z);
      pcl::PointCovariance qpt = pt;
      qpt.x = static_cast<float>(pb.x());
      qpt.y = static_cast<float>(pb.y());
      qpt.z = static_cast<float>(pb.z());
      local_cloud.points.push_back(qpt);
    }
    const auto matches = scan_context_db_.queryRobotCentricCloud(
      local_cloud.points, static_cast<std::size_t>(scan_context_top_k_),
      scan_context_max_distance_);
    for (const auto & m : matches) {
      Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
      map_to_base.translation() << m.x, m.y, request.seed.translation().z();
      map_to_base.linear() =
        Eigen::AngleAxisd(m.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      request.descriptor_seeds.push_back(map_to_base * base_from_odom);
    }
    if (!request.descriptor_seeds.empty()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), log_throttle_ms_,
        "【ScanContext召回】LOST 候选种子=%zu 最佳距离=%.3f 首选=(%.2f,%.2f,%.2f)",
        request.descriptor_seeds.size(), matches.front().distance,
        matches.front().x, matches.front().y, matches.front().yaw);
    }
  }
  if (
    last_hypothesis_ && last_hypothesis_time_ &&
    (last_scan_time_ - *last_hypothesis_time_).seconds() <= motion_reference_max_age_s_) {
    request.reference = last_hypothesis_;
  }
  const rclcpp::Time scan_time = last_scan_time_;
  const std::size_t source_points = request.source_points;

  if (multi_guess_thread_.joinable()) {
    multi_guess_thread_.join();
  }

  multi_guess_thread_ = std::thread([this, request, scan_time, source_points]() {
    MultiGuessOutcome outcome = runMultiGuessAlignmentOn(request, cancel_multi_guess_);
    {
      std::lock_guard<std::mutex> lock(async_result_mutex_);
      async_result_ = outcome;
      async_result_scan_time_ = scan_time;
      async_result_source_points_ = source_points;
      async_result_ready_ = true;
    }
    multi_guess_running_.store(false);
  });
}

void SmallGicpRelocalizationNode::cancelAsyncMultiGuess()
{
  cancel_multi_guess_.store(true);
  if (multi_guess_thread_.joinable()) {
    multi_guess_thread_.join();
  }
  multi_guess_running_.store(false);
  std::lock_guard<std::mutex> lock(async_result_mutex_);
  async_result_ready_ = false;
}


}  // namespace small_gicp_relocalization
