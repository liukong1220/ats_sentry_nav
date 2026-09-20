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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include "small_gicp_relocalization/relocalization_candidate_core.hpp"

namespace small_gicp_relocalization
{

namespace
{

Eigen::Isometry3d makePose(double x, double y, double yaw, double z = 0.0)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << x, y, z;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

CandidateEvidence healthyEvidence()
{
  CandidateEvidence evidence;
  evidence.converged = true;
  evidence.transform_finite = true;
  evidence.num_inliers = 900;
  evidence.source_points = 1000;
  evidence.registration_error = 0.05;
  evidence.overlap_ratio = 0.9;
  evidence.min_information_eigenvalue = 50.0;
  evidence.information_condition_number = 30.0;
  return evidence;
}

CandidateHardGates strictGates()
{
  CandidateHardGates gates;
  gates.min_inliers = 150;
  gates.max_registration_error = 1.0;
  gates.min_overlap_ratio = 0.3;
  gates.min_information_eigenvalue = 1.0;
  gates.max_information_condition_number = 1.0e4;
  return gates;
}

}  // namespace

TEST(RelocalizationCandidateCore, YawExtractionSurvivesTinyNegativeRollPitch)
{
  // Eigen eulerAngles(0,1,2) flips to the (pi, pi-eps, yaw+pi) branch here.
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(-1e-7, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(-1e-7, Eigen::Vector3d::UnitX()))
                                     .toRotationMatrix();
  EXPECT_NEAR(yawOf(rotation), 0.7, 1e-5);
}

TEST(RelocalizationCandidateCore, FreshScanRefreshesAcceptedObservationWithoutMotion)
{
  EXPECT_FALSE(acceptedObservationRefreshDue(10.0, 11.0, 0.0));
  EXPECT_TRUE(acceptedObservationRefreshDue(std::nullopt, 11.0, 1.0));
  EXPECT_FALSE(acceptedObservationRefreshDue(10.0, 10.99, 1.0));
  EXPECT_TRUE(acceptedObservationRefreshDue(10.0, 11.0, 1.0));
  EXPECT_FALSE(acceptedObservationRefreshDue(10.0, 9.0, 1.0));
  EXPECT_FALSE(acceptedObservationRefreshDue(10.0, std::numeric_limits<double>::quiet_NaN(), 1.0));
}

TEST(RelocalizationCandidateCore, BisectionOrderIsAPermutationWithSpreadPrefix)
{
  const auto order = bisectionOrder(9);
  ASSERT_EQ(order.size(), 9U);
  const std::set<std::size_t> unique(order.begin(), order.end());
  EXPECT_EQ(unique.size(), 9U);
  EXPECT_EQ(*unique.begin(), 0U);
  EXPECT_EQ(*unique.rbegin(), 8U);
  // A 3-element prefix must already touch both halves of the range.
  const std::vector<std::size_t> prefix(order.begin(), order.begin() + 3);
  EXPECT_TRUE(std::any_of(prefix.begin(), prefix.end(), [](std::size_t i) { return i < 4; }));
  EXPECT_TRUE(std::any_of(prefix.begin(), prefix.end(), [](std::size_t i) { return i > 4; }));
  EXPECT_TRUE(bisectionOrder(0).empty());
  EXPECT_EQ(bisectionOrder(1).size(), 1U);
}

TEST(RelocalizationCandidateCore, YawOffsetsAreCenterOutAndCoverTheCircleOnce)
{
  const auto offsets = centerOutYawOffsets(M_PI / 4.0);
  ASSERT_EQ(offsets.size(), 8U);
  EXPECT_DOUBLE_EQ(offsets.front(), 0.0);
  EXPECT_NEAR(offsets[1], M_PI / 4.0, 1e-12);
  EXPECT_NEAR(offsets[2], -M_PI / 4.0, 1e-12);
  EXPECT_NEAR(offsets.back(), M_PI, 1e-12);
  // +pi and -pi are the same rotation and must not both appear.
  EXPECT_EQ(
    std::count_if(
      offsets.begin(), offsets.end(), [](double v) { return std::abs(std::abs(v) - M_PI) < 1e-9; }),
    1);
}

TEST(RelocalizationCandidateCore, LatticeKeepsSquareWindowSetAndPutsSeedFirst)
{
  CandidateLatticeConfig config;
  config.search_half_xy = 2.0;
  config.step_xy = 1.0;
  config.step_yaw = M_PI / 4.0;
  CandidateLatticeStats stats;
  const auto seed = makePose(3.0, -1.0, 0.3);
  const auto candidates = buildLayeredCandidateLattice(seed, config, &stats);

  // 5x5 positions x 8 yaw samples, seed emitted exactly once.
  EXPECT_EQ(candidates.size(), 200U);
  EXPECT_EQ(stats.generated, 200U);
  EXPECT_TRUE(sameCandidate(candidates.front(), seed));
  EXPECT_NEAR(stats.max_radius, std::hypot(2.0, 2.0), 1e-9);
  EXPECT_NEAR(stats.min_x, 1.0, 1e-9);
  EXPECT_NEAR(stats.max_x, 5.0, 1e-9);
  EXPECT_NEAR(stats.min_y, -3.0, 1e-9);
  EXPECT_NEAR(stats.max_y, 1.0, 1e-9);
  EXPECT_NEAR(stats.max_abs_yaw_offset, M_PI, 1e-9);
  EXPECT_EQ(stats.rings, 6U);  // radii 0, 1, sqrt2, 2, sqrt5, 2sqrt2

  // Radius must be non-decreasing over the emitted order (seed excluded).
  double previous_radius = -1.0;
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    const double radius = (candidates[index].translation() - seed.translation()).head<2>().norm();
    EXPECT_GE(radius + 1e-6, previous_radius);
    previous_radius = std::max(previous_radius, radius);
  }
}

TEST(RelocalizationCandidateCore, TruncatedLatticePrefixCoversEveryDirectionAndYawSign)
{
  CandidateLatticeConfig config;
  config.search_half_xy = 2.0;
  config.step_xy = 1.0;
  config.step_yaw = M_PI / 4.0;
  const auto seed = makePose(0.0, 0.0, 0.0);
  const auto candidates = buildLayeredCandidateLattice(seed, config, nullptr);
  ASSERT_GT(candidates.size(), 48U);

  bool positive_x = false;
  bool negative_x = false;
  bool positive_y = false;
  bool negative_y = false;
  bool positive_yaw = false;
  bool negative_yaw = false;
  bool opposite_yaw = false;
  for (std::size_t index = 0; index < 48U; ++index) {
    const double x = candidates[index].translation().x();
    const double y = candidates[index].translation().y();
    const double yaw = wrapAngle(yawOf(candidates[index]));
    positive_x = positive_x || x > 0.5;
    negative_x = negative_x || x < -0.5;
    positive_y = positive_y || y > 0.5;
    negative_y = negative_y || y < -0.5;
    positive_yaw = positive_yaw || yaw > 0.1;
    negative_yaw = negative_yaw || yaw < -0.1;
    opposite_yaw = opposite_yaw || std::abs(yaw) > 2.0;
  }
  EXPECT_TRUE(positive_x);
  EXPECT_TRUE(negative_x);
  EXPECT_TRUE(positive_y);
  EXPECT_TRUE(negative_y);
  EXPECT_TRUE(positive_yaw);
  EXPECT_TRUE(negative_yaw);
  EXPECT_TRUE(opposite_yaw);
}

TEST(RelocalizationCandidateCore, NonFiniteRegistrationErrorIsNeverGatedThrough)
{
  CandidateHardGates gates = strictGates();
  gates.allow_unconverged = true;
  gates.max_registration_error = -1.0;  // gate disabled
  CandidateEvidence evidence = healthyEvidence();
  evidence.num_inliers = 100000;

  evidence.registration_error = std::numeric_limits<double>::infinity();
  EXPECT_EQ(candidateRejectReason(evidence, gates), "non-finite registration error");
  evidence.registration_error = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(candidateRejectReason(evidence, gates), "non-finite registration error");
  evidence.registration_error = -0.5;
  EXPECT_EQ(candidateRejectReason(evidence, gates), "non-finite registration error");

  evidence.registration_error = 0.05;
  evidence.transform_finite = false;
  EXPECT_EQ(candidateRejectReason(evidence, gates), "non-finite transform");
}

TEST(RelocalizationCandidateCore, RelaxationOnlyForgivesTheConvergedFlag)
{
  CandidateEvidence evidence = healthyEvidence();
  evidence.converged = false;
  CandidateHardGates gates = strictGates();

  EXPECT_EQ(candidateRejectReason(evidence, gates), "not converged");
  gates.allow_unconverged = true;
  EXPECT_TRUE(candidateRejectReason(evidence, gates).empty());

  CandidateEvidence thin_overlap = evidence;
  thin_overlap.overlap_ratio = 0.1;
  EXPECT_EQ(candidateRejectReason(thin_overlap, gates), "overlap ratio below threshold");

  CandidateEvidence degenerate = evidence;
  degenerate.min_information_eigenvalue = 1e-6;
  EXPECT_EQ(
    candidateRejectReason(degenerate, gates), "minimum information eigenvalue below threshold");

  CandidateEvidence ill_conditioned = evidence;
  ill_conditioned.information_condition_number = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    candidateRejectReason(ill_conditioned, gates), "information matrix is too ill-conditioned");

  CandidateEvidence starved = evidence;
  starved.num_inliers = 10;
  EXPECT_EQ(candidateRejectReason(starved, gates), "insufficient inliers");
}

TEST(RelocalizationCandidateCore, ScreenStageDropsAcceptGatesButNotEvidenceValidity)
{
  const CandidateHardGates screen = screenStageGates(strictGates());

  // A coarse candidate on the decimated cloud: thin overlap, weak information,
  // unconverged, residual above the accept ceiling. It must still be refined,
  // because those magnitudes are not comparable to the fine stage.
  CandidateEvidence coarse = healthyEvidence();
  coarse.converged = false;
  coarse.overlap_ratio = 0.02;
  coarse.min_information_eigenvalue = 1e-9;
  coarse.information_condition_number = 1.0e9;
  coarse.registration_error = 5.0;
  EXPECT_TRUE(candidateRejectReason(coarse, screen).empty());
  EXPECT_FALSE(candidateRejectReason(coarse, strictGates()).empty());

  // Evidence validity is never screened away.
  CandidateEvidence non_finite = coarse;
  non_finite.registration_error = std::numeric_limits<double>::infinity();
  EXPECT_EQ(candidateRejectReason(non_finite, screen), "non-finite registration error");

  CandidateEvidence broken_transform = coarse;
  broken_transform.transform_finite = false;
  EXPECT_EQ(candidateRejectReason(broken_transform, screen), "non-finite transform");

  CandidateEvidence starved = coarse;
  starved.num_inliers = 10;
  EXPECT_EQ(candidateRejectReason(starved, screen), "insufficient inliers");
  EXPECT_EQ(screen.min_inliers, strictGates().min_inliers);
}

TEST(RelocalizationCandidateCore, CombinedScorePrefersEvidenceOverResidualAlone)
{
  CandidateScoreWeights weights;
  // Wrong hypothesis in a repetitive corridor: smaller residual, thin overlap,
  // degenerate information.
  CandidateEvidence wrong = healthyEvidence();
  wrong.registration_error = 0.02;
  wrong.overlap_ratio = 0.25;
  wrong.min_information_eigenvalue = 0.05;
  wrong.information_condition_number = 5.0e4;

  CandidateEvidence correct = healthyEvidence();
  correct.registration_error = 0.06;

  const auto wrong_score = scoreCandidate(wrong, weights);
  const auto correct_score = scoreCandidate(correct, weights);
  EXPECT_LT(wrong.registration_error, correct.registration_error);
  EXPECT_LT(correct_score.total, wrong_score.total);
  EXPECT_GE(correct_score.total, 0.0);
}

TEST(RelocalizationCandidateCore, ScoreTermsStayBoundedAndInfiniteOnNonFiniteError)
{
  CandidateScoreWeights weights;
  CandidateEvidence evidence = healthyEvidence();
  evidence.registration_error = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(std::isfinite(scoreCandidate(evidence, weights).total));

  evidence.registration_error = 1e9;
  evidence.overlap_ratio = 0.0;
  evidence.min_information_eigenvalue = 0.0;
  evidence.information_condition_number = std::numeric_limits<double>::infinity();
  evidence.motion_residual = 1e6;
  evidence.prior_deviation = 1e6;
  const auto worst = scoreCandidate(evidence, weights);
  const double weight_sum =
    weights.error + weights.overlap + weights.information + weights.motion + weights.prior;
  EXPECT_LE(worst.total, weight_sum + 1e-9);
  EXPECT_NEAR(worst.error_term, 1.0, 1e-6);
  EXPECT_NEAR(worst.overlap_term, 1.0, 1e-9);
  EXPECT_NEAR(worst.information_term, 1.0, 1e-6);
}

TEST(RelocalizationCandidateCore, SameBasinRunnerUpIsNotTreatedAsAmbiguity)
{
  AmbiguityConfig config;
  config.min_score_margin = 0.2;
  std::vector<RankedCandidate> candidates;
  candidates.push_back({0.40, makePose(1.0, 1.0, 0.0)});
  candidates.push_back({0.41, makePose(1.02, 0.99, 0.01)});  // same minimum
  candidates.push_back({0.95, makePose(9.0, -4.0, 2.0)});    // clearly worse elsewhere

  const auto selection = selectCandidate(candidates, config);
  ASSERT_TRUE(selection.has_best);
  EXPECT_EQ(selection.best_index, 0U);
  ASSERT_TRUE(selection.has_alternative);
  EXPECT_EQ(selection.alternative_index, 2U);
  EXPECT_NEAR(selection.score_margin, 0.55, 1e-9);
  EXPECT_FALSE(selection.ambiguous);
}

TEST(RelocalizationCandidateCore, DistinctAndCloseScoringRunnerUpIsAmbiguous)
{
  AmbiguityConfig config;
  config.min_score_margin = 0.2;
  std::vector<RankedCandidate> candidates;
  candidates.push_back({0.40, makePose(1.0, 1.0, 0.0)});
  candidates.push_back({0.45, makePose(1.0, 5.0, 0.0)});  // parallel wall, 4 m away

  const auto selection = selectCandidate(candidates, config);
  ASSERT_TRUE(selection.has_best);
  ASSERT_TRUE(selection.has_alternative);
  EXPECT_NEAR(selection.score_margin, 0.05, 1e-9);
  EXPECT_TRUE(selection.ambiguous);

  // The gate stays inert until a data-driven margin is configured.
  AmbiguityConfig disabled = config;
  disabled.min_score_margin = 0.0;
  EXPECT_FALSE(selectCandidate(candidates, disabled).ambiguous);
}

TEST(RelocalizationCandidateCore, SelectionIgnoresNonFiniteScores)
{
  AmbiguityConfig config;
  std::vector<RankedCandidate> candidates;
  candidates.push_back({std::numeric_limits<double>::infinity(), makePose(0.0, 0.0, 0.0)});
  EXPECT_FALSE(selectCandidate(candidates, config).has_best);

  candidates.push_back({0.3, makePose(2.0, 0.0, 0.0)});
  const auto selection = selectCandidate(candidates, config);
  ASSERT_TRUE(selection.has_best);
  EXPECT_EQ(selection.best_index, 1U);
  EXPECT_FALSE(selection.has_alternative);
}

TEST(RelocalizationCandidateCore, ObservationQualityRefusesNonFiniteError)
{
  const auto infinite =
    observationQuality(true, std::numeric_limits<double>::infinity(), 900, 1000);
  EXPECT_FALSE(infinite.error_finite);
  EXPECT_DOUBLE_EQ(infinite.quality, 0.0);

  const auto nan_error =
    observationQuality(true, std::numeric_limits<double>::quiet_NaN(), 900, 1000);
  EXPECT_FALSE(nan_error.error_finite);
  EXPECT_DOUBLE_EQ(nan_error.quality, 0.0);

  const auto negative = observationQuality(true, -1.0, 900, 1000);
  EXPECT_FALSE(negative.error_finite);
  EXPECT_DOUBLE_EQ(negative.quality, 0.0);

  const auto rejected = observationQuality(false, 0.05, 900, 1000);
  EXPECT_TRUE(rejected.error_finite);
  EXPECT_DOUBLE_EQ(rejected.quality, 0.0);

  const auto accepted = observationQuality(true, 0.05, 900, 1000);
  EXPECT_TRUE(accepted.error_finite);
  EXPECT_NEAR(accepted.quality, 0.9 * std::exp(-0.05), 1e-12);
}

TEST(RelocalizationCandidateCore, ConfirmationRequiresIncreasingStampAndInterval)
{
  ConfirmationGates gates;
  gates.min_interval_s = 0.10;
  ConfirmationSample pending;
  pending.scan_time_s = 10.0;
  ConfirmationSample candidate = pending;

  candidate.scan_time_s = 10.0;
  EXPECT_EQ(
    evaluateConfirmation(pending, candidate, gates).reason,
    "confirmation scan stamp not increasing");
  candidate.scan_time_s = 9.9;
  EXPECT_EQ(
    evaluateConfirmation(pending, candidate, gates).reason,
    "confirmation scan stamp not increasing");
  candidate.scan_time_s = 10.05;
  EXPECT_EQ(
    evaluateConfirmation(pending, candidate, gates).reason, "confirmation scan interval too short");
  candidate.scan_time_s = 10.2;
  EXPECT_TRUE(evaluateConfirmation(pending, candidate, gates).consistent);
}

TEST(RelocalizationCandidateCore, ConfirmationRejectsOdometryInconsistentPair)
{
  ConfirmationGates gates;
  gates.translation_tolerance = 0.15;
  gates.yaw_tolerance = 0.10;
  gates.motion_translation_tolerance = 0.25;
  gates.min_interval_s = 0.05;

  ConfirmationSample pending;
  pending.scan_time_s = 5.0;
  pending.map_to_odom = makePose(0.0, 0.0, 0.0);
  pending.odom_to_base = makePose(10.0, 0.0, 0.0);

  ConfirmationSample candidate;
  candidate.scan_time_s = 5.3;
  // Small map->odom yaw drift passes the raw transform tolerance, but at 10 m
  // lever arm it implies ~0.5 m of robot motion that odometry never reported.
  candidate.map_to_odom = makePose(0.0, 0.0, 0.05);
  candidate.odom_to_base = makePose(10.0, 0.0, 0.0);

  const auto decision = evaluateConfirmation(pending, candidate, gates);
  EXPECT_LT(decision.translation_delta, gates.translation_tolerance);
  EXPECT_LT(decision.yaw_delta, gates.yaw_tolerance);
  EXPECT_FALSE(decision.consistent);
  EXPECT_EQ(decision.reason, "confirmation inconsistent with odometry motion");
  EXPECT_GT(decision.motion_translation, gates.motion_translation_tolerance);
}

TEST(RelocalizationCandidateCore, ConfirmationAcceptsMovingRobotWithStableCorrection)
{
  ConfirmationGates gates;
  ConfirmationSample pending;
  pending.scan_time_s = 5.0;
  pending.map_to_odom = makePose(1.17, -0.44, 0.20);
  pending.odom_to_base = makePose(3.0, 2.0, 0.4);

  ConfirmationSample candidate;
  candidate.scan_time_s = 5.4;
  candidate.map_to_odom = pending.map_to_odom;
  candidate.odom_to_base = makePose(3.5, 2.2, 0.5);  // robot really moved

  const auto decision = evaluateConfirmation(pending, candidate, gates);
  EXPECT_TRUE(decision.consistent);
  EXPECT_NEAR(decision.motion_translation, 0.0, 1e-9);
  EXPECT_NEAR(decision.motion_yaw, 0.0, 1e-9);
}

TEST(RelocalizationCandidateCore, ConfirmationRejectsTransformJump)
{
  ConfirmationGates gates;
  ConfirmationSample pending;
  pending.scan_time_s = 1.0;
  ConfirmationSample candidate;
  candidate.scan_time_s = 1.5;
  candidate.map_to_odom = makePose(0.9, 0.0, 0.0);
  EXPECT_EQ(
    evaluateConfirmation(pending, candidate, gates).reason, "confirmation transform mismatch");
}

TEST(RelocalizationLifecycle, ResetRejectsCompletedAndLateWorkerResultsBeforeMutation)
{
  RelocalizationGeneration generation;
  const auto completed_request = generation.current();
  const auto in_flight_request = generation.current();
  generation.invalidate();  // /initialpose or recovery reset
  int observations = 0;
  double seed = 7.0;
  const auto apply_old_result = [&]() {
    ++observations;
    seed = -3.0;
  };
  EXPECT_FALSE(generation.admit(completed_request, apply_old_result));
  EXPECT_FALSE(generation.admit(in_flight_request, apply_old_result));
  EXPECT_EQ(observations, 0);
  EXPECT_DOUBLE_EQ(seed, 7.0);
  EXPECT_TRUE(generation.admit(generation.current(), [&]() { ++observations; }));
  EXPECT_EQ(observations, 1);
  generation.invalidate();
  EXPECT_FALSE(generation.admit(completed_request, apply_old_result));
}

TEST(RelocalizationLifecycle, FineDeadlineRejectsOverrunAndStopsScheduling)
{
  using Clock = std::chrono::steady_clock;
  auto now = Clock::time_point{};
  const auto deadline = now + std::chrono::milliseconds(10);
  int align_calls = 0;
  int admitted = 0;
  for (int candidate = 0; candidate < 3; ++candidate) {
    if (multiGuessDeadlineExpired(now, deadline)) {
      break;
    }
    ++align_calls;
    now += std::chrono::milliseconds(11);  // Non-interruptible alignment completes late.
    if (multiGuessDeadlineExpired(now, deadline)) {
      break;
    }
    ++admitted;
  }
  EXPECT_EQ(align_calls, 1);
  EXPECT_EQ(admitted, 0);
  EXPECT_TRUE(multiGuessDeadlineExpired(deadline, deadline));
  EXPECT_FALSE(multiGuessDeadlineExpired(deadline - std::chrono::nanoseconds(1), deadline));
}

TEST(RelocalizationLifecycle, FinalAdmissionRejectsEarlierCandidateAfterBudgetExpires)
{
  const auto start = std::chrono::steady_clock::time_point{};
  const auto deadline = start + std::chrono::milliseconds(10);
  EXPECT_FALSE(multiGuessDeadlineExpired(start + std::chrono::milliseconds(4), deadline));
  // A good early candidate cannot survive later refinement/diagnostic overrun.
  EXPECT_TRUE(multiGuessDeadlineExpired(start + std::chrono::milliseconds(12), deadline));
}

}  // namespace small_gicp_relocalization
