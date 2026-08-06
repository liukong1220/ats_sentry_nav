// Copyright 2026

#include <gtest/gtest.h>

#include <limits>

#include "ats_swerve_mpc/ltv_qp_solver.hpp"

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

TEST(LtvQpCandidateValidator, RejectsDeadlineResidualAndUnapprovedStatus) {
  CandidateFixture fixture;
  fixture.result.solve_time_ms = 20.0;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "iteration_or_deadline_reject");

  fixture.result.solve_time_ms = 1.0;
  fixture.result.primal_residual = 1e-3;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "residual_reject");

  fixture.result.primal_residual = 1e-7;
  fixture.result.status = LtvQpSolverStatus::kSolvedInaccurate;
  EXPECT_FALSE(fixture.audit().feasible);
  EXPECT_EQ(fixture.audit().rejection_reason, "solver_status_not_solved");
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
