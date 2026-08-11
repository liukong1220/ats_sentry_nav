// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>

#include "ats_swerve_mpc/qp/ltv_qp_osqp_solver.hpp"

namespace {

using ats_swerve_mpc::LtvQpOsqpSolver;
using ats_swerve_mpc::LtvQpSolveResult;
using ats_swerve_mpc::LtvQpSolverSettings;
using ats_swerve_mpc::LtvQpSolverStatus;
using ats_swerve_mpc::LtvQpSparseProblem;
using ats_swerve_mpc::Control;
using ats_swerve_mpc::LtvQpBuilder;
using ats_swerve_mpc::LtvQpProblem;
using ats_swerve_mpc::Se2MpcConfig;
using ats_swerve_mpc::Se2Reference;
using ats_swerve_mpc::Se2Model;
using ats_swerve_mpc::State;

LtvQpSolverSettings settings() {
  LtvQpSolverSettings value;
  value.max_iterations = 200;
  value.time_limit_ms = 50.0;
  value.max_primal_residual = 1e-5;
  value.max_dual_residual = 1e-5;
  value.max_tracking_slack = 0.0;
  value.max_hard_constraint_violation = 1e-8;
  return value;
}

LtvQpSparseProblem scalarProblem(double gradient) {
  LtvQpSparseProblem problem;
  problem.decision_size = 1;
  problem.constraint_rows = 1;
  problem.hessian_structure = LtvQpOsqpSolver::denseUpperPattern(1);
  problem.constraint_structure = LtvQpOsqpSolver::densePattern(1, 1);
  problem.hessian_values = {2.0};
  problem.constraint_values = {1.0};
  problem.gradient = Eigen::VectorXd::Constant(1, gradient);
  problem.lower = Eigen::VectorXd::Constant(1, 0.0);
  problem.upper = Eigen::VectorXd::Constant(1, 1.0);
  return problem;
}

TEST(LtvQpOsqpSolver, SolvesAndReusesFixedWorkspaceWithWarmStart) {
  const auto solver_settings = settings();
  LtvQpOsqpSolver solver(1, 1, solver_settings);
  ASSERT_TRUE(solver.initialized());
  ASSERT_EQ(solver.setupCount(), 1u);

  const auto first = solver.solve(scalarProblem(-1.0), solver_settings, nullptr);
  EXPECT_EQ(first.status, LtvQpSolverStatus::kSolved);
  ASSERT_EQ(first.primal_solution.size(), 1);
  EXPECT_NEAR(first.primal_solution(0), 0.5, 1e-3);
  EXPECT_TRUE(std::isfinite(first.solve_time_ms));
  EXPECT_TRUE(std::isfinite(first.update_time_ms));
  EXPECT_TRUE(std::isfinite(first.wall_solve_time_ms));
  EXPECT_TRUE(std::isfinite(first.wall_update_time_ms));
  EXPECT_TRUE(std::isfinite(first.wall_qp_phase_time_ms));
  EXPECT_TRUE(std::isfinite(first.primal_residual));
  EXPECT_TRUE(std::isfinite(first.dual_residual));

  ats_swerve_mpc::LtvQpWarmStart warm_start;
  warm_start.primal = first.primal_solution;
  warm_start.dual = first.dual_solution;
  const auto second = solver.solve(scalarProblem(-0.5), solver_settings,
                                   &warm_start);
  EXPECT_EQ(second.status, LtvQpSolverStatus::kSolved);
  EXPECT_TRUE(second.warm_start_used);
  EXPECT_NEAR(second.primal_solution(0), 0.25, 1e-3);
  EXPECT_EQ(solver.setupCount(), 1u);
  EXPECT_EQ(solver.numericUpdateCallCount(), 2u);
}

TEST(LtvQpOsqpSolver, RejectsChangedCscPatternWithoutSetup) {
  const auto solver_settings = settings();
  LtvQpOsqpSolver solver(1, 1, solver_settings);
  ASSERT_TRUE(solver.initialized());
  auto problem = scalarProblem(-1.0);
  problem.constraint_structure.row_indices.push_back(0);
  const auto result = solver.solve(problem, solver_settings, nullptr);
  EXPECT_EQ(result.status, LtvQpSolverStatus::kInvalidProblem);
  EXPECT_EQ(solver.setupCount(), 1u);
}

TEST(LtvQpOsqpSolver, InvalidSetupIsBackendUnavailable) {
  auto invalid = settings();
  invalid.time_limit_ms = 0.0;
  LtvQpOsqpSolver solver(1, 1, invalid);
  EXPECT_FALSE(solver.initialized());
  const auto result = solver.solve(scalarProblem(-1.0), settings(), nullptr);
  EXPECT_EQ(result.status, LtvQpSolverStatus::kBackendUnavailable);
}

TEST(LtvQpOsqpSolver, ConvertsLtvBlocksToAStableSparsePattern) {
  Se2MpcConfig config;
  config.horizon = 2;
  config.dt = 0.1;
  config.max_vx = config.max_vy = config.max_wz = 1.0;
  config.max_ax = config.max_ay = config.max_awz = 2.0;
  config.wheel_base_x = config.wheel_base_y = 0.27;
  config.max_wheel_speed = 2.0;
  config.max_wheel_acceleration = 5.0;
  config.max_steer_rate = 5.0;
  std::vector<Control> controls(2, Control::Zero());
  std::vector<State> states(3, State::Zero());
  std::vector<Se2Reference> references(3);
  for (auto &reference : references) {
    reference.control = Control(0.1, 0.0, 0.0);
  }
  const auto dense = LtvQpBuilder::build(
      State::Zero(), states, controls, references, Control::Zero(), config);
  ASSERT_TRUE(dense.valid) << dense.validation_error;
  const auto sparse = ats_swerve_mpc::makeLtvQpSparseProblem(dense);
  ASSERT_TRUE(sparse.valid());
  EXPECT_EQ(sparse.hessian_structure.row_indices.size(), 18u);
  EXPECT_LT(sparse.constraint_structure.row_indices.size(),
            static_cast<std::size_t>(sparse.decision_size * sparse.constraint_rows));
  LtvQpOsqpSolver solver(sparse.decision_size, sparse.constraint_rows, settings());
  ASSERT_TRUE(solver.initialized());
  const auto result = solver.solve(sparse, settings(), nullptr);
  EXPECT_TRUE(result.status == LtvQpSolverStatus::kSolved ||
              result.status == LtvQpSolverStatus::kMaxIterations)
      << "status=" << ats_swerve_mpc::ltvQpSolverStatusName(result.status)
      << " iter=" << result.iterations << " prim=" << result.primal_residual
      << " dual=" << result.dual_residual;
  EXPECT_LE(result.primal_residual, settings().max_primal_residual);
  EXPECT_LE(result.dual_residual, settings().max_dual_residual);
}

TEST(LtvQpOsqpSolver, RejectsCorruptFinitePayloadBeforeOsqpNumericUpdate) {
  Se2MpcConfig config;
  config.horizon = 2;
  config.dt = 0.1;
  config.max_vx = config.max_vy = config.max_wz = 1.0;
  config.max_ax = config.max_ay = config.max_awz = 2.0;
  config.wheel_base_x = config.wheel_base_y = 0.27;
  config.max_wheel_speed = 2.0;
  config.max_wheel_acceleration = 5.0;
  config.max_steer_rate = 5.0;
  std::vector<Control> controls(2, Control::Zero());
  std::vector<State> states(3, State::Zero());
  std::vector<Se2Reference> references(3);
  for (auto &reference : references) {
    reference.control = Control(0.1, 0.0, 0.0);
  }

  const auto dimensions = ats_swerve_mpc::checkedLtvQpDimensions(config.horizon);
  ASSERT_TRUE(dimensions.valid);
  const auto makeProblem = [&]() {
    return LtvQpBuilder::build(
        State::Zero(), states, controls, references, Control::Zero(), config);
  };
  LtvQpOsqpSolver solver(dimensions, settings());
  ASSERT_TRUE(solver.initialized());

  const auto expectRejected = [&solver](LtvQpProblem &problem) {
    ASSERT_TRUE(problem.valid);
    const std::size_t updates = solver.numericUpdateCallCount();
    const auto result = solver.solveLtvProblem(problem, settings(), nullptr);
    EXPECT_EQ(result.status, LtvQpSolverStatus::kInvalidProblem);
    EXPECT_EQ(solver.numericUpdateCallCount(), updates);
  };

  auto hessian = makeProblem();
  hessian.hessian(0, 0) = std::numeric_limits<double>::quiet_NaN();
  expectRejected(hessian);
  auto gradient = makeProblem();
  gradient.gradient(0) = std::numeric_limits<double>::infinity();
  expectRejected(gradient);
  auto equality = makeProblem();
  equality.equality_matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();
  expectRejected(equality);
  auto equality_bound = makeProblem();
  equality_bound.equality_upper(0) = std::numeric_limits<double>::infinity();
  expectRejected(equality_bound);
  auto inequality_bound = makeProblem();
  inequality_bound.inequality_lower(0) = std::numeric_limits<double>::quiet_NaN();
  expectRejected(inequality_bound);
  auto variable_bound = makeProblem();
  variable_bound.lower_bound(0) = std::numeric_limits<double>::quiet_NaN();
  expectRejected(variable_bound);
}

}  // namespace
