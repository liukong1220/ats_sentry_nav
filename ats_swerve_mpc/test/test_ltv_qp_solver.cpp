// Copyright 2026

#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

namespace {

using ats_swerve_mpc::Control;
using ats_swerve_mpc::LtvQpCandidateAudit;
using ats_swerve_mpc::LtvQpCandidateSafety;
using ats_swerve_mpc::LtvQpCandidateValidator;
using ats_swerve_mpc::LtvQpProblem;
using ats_swerve_mpc::LtvQpSolveResult;
using ats_swerve_mpc::LtvQpSolverSettings;
using ats_swerve_mpc::LtvQpSolverStatus;
using ats_swerve_mpc::LtvQpSparseProblem;
using ats_swerve_mpc::LtvQpSparseStructure;
using ats_swerve_mpc::LtvQpWarmStart;
using ats_swerve_mpc::Se2Model;
using ats_swerve_mpc::Se2MpcConfig;
using ats_swerve_mpc::Se2Reference;
using ats_swerve_mpc::State;
using ats_swerve_mpc::ZeroSpeedGuardConfig;

std::vector<Se2Reference> references(int horizon, double dt,
                                     const Control &control) {
  std::vector<Se2Reference> output;
  output.reserve(static_cast<std::size_t>(horizon + 1));
  State state = State::Zero();
  Se2Model model(dt);
  for (int step = 0; step <= horizon; ++step) {
    Se2Reference reference;
    reference.state = state;
    reference.control = control;
    output.push_back(reference);
    state = model.dynamics(state, control);
  }
  return output;
}

Se2MpcConfig validConfig(int horizon) {
  Se2MpcConfig config;
  config.horizon = horizon;
  config.dt = 0.1;
  config.max_vx = 2.0;
  config.max_vy = 2.0;
  config.max_wz = 2.0;
  config.max_ax = 20.0;
  config.max_ay = 20.0;
  config.max_awz = 20.0;
  config.wheel_base_x = 0.27;
  config.wheel_base_y = 0.27;
  config.max_wheel_speed = 2.0;
  config.max_wheel_acceleration = 20.0;
  config.max_steer_rate = 20.0;
  return config;
}

LtvQpSolverSettings validSettings() {
  LtvQpSolverSettings settings;
  settings.max_iterations = 20;
  settings.time_limit_ms = 10.0;
  settings.max_primal_residual = 1e-5;
  settings.max_dual_residual = 1e-5;
  settings.max_tracking_slack = 0.0;
  settings.max_hard_constraint_violation = 1e-8;
  return settings;
}

LtvQpCandidateSafety healthySafety() {
  LtvQpCandidateSafety safety;
  safety.inputs_healthy = true;
  safety.emergency_stop_active = false;
  safety.collision_free = true;
  safety.localization_fresh = true;
  safety.reference_fresh = true;
  safety.execution_lease_valid = true;
  safety.gimbal_valid = true;
  safety.map_fresh = true;
  return safety;
}

struct CandidateFixture {
  Se2MpcConfig config;
  std::vector<Control> nominal_controls;
  std::vector<Se2Reference> refs;
  LtvQpProblem problem;
  LtvQpSolveResult result;

  CandidateFixture()
      : config(validConfig(2)),
        nominal_controls{Control(0.2, 0.05, 0.1),
                         Control(0.2, 0.05, 0.1)},
        refs(references(config.horizon, config.dt, nominal_controls.front())),
        problem(ats_swerve_mpc::LtvQpBuilder::build(
            State::Zero(),
            Se2Model(config.dt).rollout(State::Zero(), nominal_controls),
            nominal_controls, refs, Control::Zero(), config)) {
    result.status = LtvQpSolverStatus::kSolved;
    result.iterations = 3;
    result.solve_time_ms = 1.0;
    result.primal_residual = 1e-7;
    result.dual_residual = 1e-7;
    result.slack_maximum = 0.0;
    result.hard_constraint_maximum_violation = 0.0;
    result.primal_solution = Eigen::VectorXd::Zero(problem.decisionSize());
    result.dual_solution = Eigen::VectorXd::Zero(
        problem.equality_matrix.rows() + problem.inequality_matrix.rows() +
        problem.decisionSize());
  }

  LtvQpCandidateAudit audit() const {
    return LtvQpCandidateValidator::validate(
        problem, nominal_controls, Control::Zero(), config,
        ZeroSpeedGuardConfig(), validSettings(), healthySafety(), result);
  }
};

TEST(LtvQpSparseContract, RejectsChangingOrMalformedCscStructure) {
  LtvQpSparseStructure structure;
  structure.rows = 2;
  structure.columns = 2;
  structure.column_offsets = {0, 2, 4};
  structure.row_indices = {0, 1, 0, 1};
  EXPECT_TRUE(structure.valid());

  LtvQpSparseProblem problem;
  problem.decision_size = 2;
  problem.constraint_rows = 2;
  problem.hessian_structure = structure;
  problem.hessian_values = {2.0, 0.0, 0.0, 2.0};
  problem.constraint_structure = structure;
  problem.constraint_values = {1.0, 0.0, 0.0, 1.0};
  problem.gradient = Eigen::Vector2d::Zero();
  problem.lower = Eigen::Vector2d::Constant(-1.0);
  problem.upper = Eigen::Vector2d::Constant(1.0);
  EXPECT_TRUE(problem.valid());

  const auto expected = structure;
  problem.constraint_structure.row_indices[2] = 1;
  EXPECT_FALSE(problem.constraint_structure.samePattern(expected));
  EXPECT_FALSE(problem.valid());
}

TEST(LtvQpSparseContract, WarmStartRequiresExactFiniteDimensions) {
  LtvQpWarmStart warm_start;
  warm_start.primal = Eigen::Vector2d::Zero();
  warm_start.dual = Eigen::Vector3d::Zero();
  EXPECT_TRUE(warm_start.validFor(2, 3));
  EXPECT_FALSE(warm_start.validFor(3, 3));
  warm_start.primal(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(warm_start.validFor(2, 3));
}

TEST(LtvQpCandidateValidator, AcceptsOnlyACompleteFeasibleCandidate) {
  const CandidateFixture fixture;
  const auto audit = fixture.audit();
  EXPECT_TRUE(fixture.problem.valid) << fixture.problem.validation_error;
  EXPECT_TRUE(audit.feasible) << audit.rejection_reason;
  EXPECT_NEAR(audit.actual_hard_constraint_maximum_violation, 0.0, 1e-8);
}

TEST(LtvQpCandidateReconstructor, RebuildsDeltaControlAndNonlinearRollout) {
  CandidateFixture fixture;
  fixture.result.primal_solution(fixture.problem.controlOffset(0)) = 0.10;
  fixture.result.primal_solution(fixture.problem.controlOffset(0) + 1) = -0.02;
  const auto candidate = ats_swerve_mpc::LtvQpCandidateReconstructor::reconstruct(
      State(1.0, -2.0, 3.13), fixture.problem, fixture.nominal_controls,
      fixture.config, fixture.result);
  ASSERT_TRUE(candidate.valid) << candidate.validation_error;
  ASSERT_EQ(candidate.controls.size(), fixture.nominal_controls.size());
  EXPECT_NEAR(candidate.controls.front()(0),
              fixture.nominal_controls.front()(0) + 0.10, 1e-12);
  EXPECT_NEAR(candidate.controls.front()(1),
              fixture.nominal_controls.front()(1) - 0.02, 1e-12);
  ASSERT_EQ(candidate.states.size(), fixture.nominal_controls.size() + 1);
  EXPECT_TRUE(candidate.states.front().isApprox(State(1.0, -2.0, 3.13)));
  EXPECT_GE(candidate.states.back()(2), -3.141592653589793);
  EXPECT_LE(candidate.states.back()(2), 3.141592653589793);
}

TEST(LtvQpCandidateValidator, ReconstructionRejectsMalformedNominalDimensions) {
  CandidateFixture fixture;
  const auto candidate = ats_swerve_mpc::LtvQpCandidateReconstructor::reconstruct(
      State::Zero(), fixture.problem, std::vector<Control>{Control::Zero()},
      fixture.config, fixture.result);
  EXPECT_FALSE(candidate.valid);
  const auto audit = LtvQpCandidateValidator::validate(
      fixture.problem, std::vector<Control>{Control::Zero()}, Control::Zero(),
      fixture.config, ZeroSpeedGuardConfig(), validSettings(), healthySafety(),
      fixture.result, candidate);
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason, "nonlinear_rollout_reconstruction_reject");
}

TEST(LtvQpCandidateValidator, RejectsDeadlineResidualAndUnapprovedStatus) {
  CandidateFixture fixture;
  fixture.result.solve_time_ms = 20.0;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "iteration_or_deadline_reject");

  fixture.result.solve_time_ms = 1.0;
  fixture.result.wall_solve_time_ms = 20.0;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "iteration_or_deadline_reject");

  fixture.result.wall_solve_time_ms = 1.0;
  fixture.result.wall_update_time_ms = 20.0;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "iteration_or_deadline_reject");

  fixture.result.wall_update_time_ms = 1.0;
  fixture.result.primal_residual = 1e-3;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "residual_reject");

  fixture.result.primal_residual = 1e-7;
  fixture.result.status = LtvQpSolverStatus::kSolvedInaccurate;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "solver_status_not_solved");
}

TEST(LtvQpCandidateValidator, RejectsMissingOrMalformedDualPayload) {
  CandidateFixture fixture;
  fixture.result.dual_solution.resize(0);
  auto audit = fixture.audit();
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason, "invalid_problem_or_candidate_dimensions");

  fixture.result.dual_solution = Eigen::VectorXd::Zero(
      fixture.problem.equality_matrix.rows() +
      fixture.problem.inequality_matrix.rows() + fixture.problem.decisionSize());
  fixture.result.dual_solution(0) = std::numeric_limits<double>::quiet_NaN();
  audit = fixture.audit();
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason, "non_finite_matrix_or_result");
}

TEST(LtvQpCandidateValidator, RejectsMalformedDenseLayoutAndBounds) {
  CandidateFixture fixture;
  fixture.problem.inequality_upper.conservativeResize(
      fixture.problem.inequality_upper.size() - 1);
  auto audit = fixture.audit();
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason,
            "invalid_problem_or_candidate_dimensions");

  CandidateFixture reversed_bounds_fixture;
  reversed_bounds_fixture.problem.lower_bound(0) = 1.0;
  reversed_bounds_fixture.problem.upper_bound(0) = -1.0;
  audit = reversed_bounds_fixture.audit();
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason,
            "invalid_problem_or_candidate_dimensions");
}

TEST(LtvQpCandidateValidator, RejectsEveryNonSolvedBackendStatus) {
  const std::array<LtvQpSolverStatus, 8> rejected_statuses{{
      LtvQpSolverStatus::kSolvedInaccurate,
      LtvQpSolverStatus::kMaxIterations,
      LtvQpSolverStatus::kTimeLimit,
      LtvQpSolverStatus::kPrimalInfeasible,
      LtvQpSolverStatus::kDualInfeasible,
      LtvQpSolverStatus::kNumericalFailure,
      LtvQpSolverStatus::kInvalidProblem,
      LtvQpSolverStatus::kBackendUnavailable,
  }};
  for (const auto status : rejected_statuses) {
    CandidateFixture fixture;
    fixture.result.status = status;
    const auto audit = fixture.audit();
    EXPECT_FALSE(audit.feasible) << ats_swerve_mpc::ltvQpSolverStatusName(status);
    EXPECT_EQ(audit.rejection_reason, "solver_status_not_solved");
  }
}

TEST(LtvQpCandidateValidator, RejectsInputHealthEmergencyAndCollisionFailures) {
  CandidateFixture fixture;
  auto safety = healthySafety();
  safety.inputs_healthy = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.emergency_stop_active = true;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.collision_free = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.localization_fresh = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.reference_fresh = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.execution_lease_valid = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.gimbal_valid = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);

  safety = healthySafety();
  safety.map_fresh = false;
  EXPECT_FALSE(LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control::Zero(), fixture.config,
      ZeroSpeedGuardConfig(), validSettings(), safety, fixture.result).feasible);
}

TEST(LtvQpCandidateValidator, RejectsRealWheelSpeedAndVectorIncrementViolations) {
  CandidateFixture fixture;
  fixture.config.max_vx = 10.0;
  fixture.config.max_vy = 10.0;
  fixture.config.max_wz = 10.0;
  fixture.config.max_wheel_speed = 0.1;
  fixture.nominal_controls.assign(2, Control(1.0, 0.0, 0.0));
  fixture.result.primal_solution.setZero();
  const auto wheel_audit = fixture.audit();
  EXPECT_FALSE(wheel_audit.feasible);
  EXPECT_EQ(wheel_audit.rejection_reason, "hard_constraint_reject");

  fixture.config.max_wheel_speed = 10.0;
  fixture.config.max_wheel_acceleration = 0.1;
  const auto increment_audit = fixture.audit();
  EXPECT_FALSE(increment_audit.feasible);
  EXPECT_EQ(increment_audit.rejection_reason, "hard_constraint_reject");
}

TEST(LtvQpCandidateValidator, UsesZeroSpeedGuardBeforeCheckingSteeringDirection) {
  CandidateFixture fixture;
  fixture.config.max_steer_rate = 0.01;
  fixture.nominal_controls.assign(2, Control(0.005, 0.0, 0.0));
  fixture.result.primal_solution.setZero();
  const auto audit = fixture.audit();
  EXPECT_TRUE(audit.feasible) << audit.rejection_reason;
  EXPECT_EQ(audit.steering_rate_constraints_checked, 0);
  EXPECT_GT(audit.steering_rate_constraints_skipped_by_zero_speed_guard, 0);
}

TEST(LtvQpCandidateValidator, RejectsDefinedSteeringRateReversal) {
  CandidateFixture fixture;
  fixture.config.max_ax = 20.0;
  fixture.config.max_steer_rate = 0.1;
  fixture.nominal_controls.assign(2, Control(-0.5, 0.0, 0.0));
  fixture.result.primal_solution.setZero();
  const auto audit = LtvQpCandidateValidator::validate(
      fixture.problem, fixture.nominal_controls, Control(0.5, 0.0, 0.0),
      fixture.config, ZeroSpeedGuardConfig(), validSettings(), healthySafety(),
      fixture.result);
  EXPECT_FALSE(audit.feasible);
  EXPECT_EQ(audit.rejection_reason, "hard_constraint_reject");
  EXPECT_GT(audit.steering_rate_constraints_checked, 0);
}

}  // namespace
