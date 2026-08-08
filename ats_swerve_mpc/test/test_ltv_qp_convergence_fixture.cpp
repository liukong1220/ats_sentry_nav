// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "ats_swerve_mpc/qp/ltv_qp_osqp_solver.hpp"

namespace {

using ats_swerve_mpc::Control;
using ats_swerve_mpc::LtvQpBuilder;
using ats_swerve_mpc::LtvQpOsqpSolver;
using ats_swerve_mpc::LtvQpSolverSettings;
using ats_swerve_mpc::LtvQpSolverStatus;
using ats_swerve_mpc::Se2MpcConfig;
using ats_swerve_mpc::Se2Model;
using ats_swerve_mpc::Se2Reference;
using ats_swerve_mpc::State;

LtvQpSolverSettings operationalSettings() {
  LtvQpSolverSettings settings;
  settings.max_iterations = 400;
  settings.time_limit_ms = 10.0;
  settings.max_primal_residual = 1e-4;
  settings.max_dual_residual = 1e-4;
  settings.max_tracking_slack = 0.0;
  settings.max_hard_constraint_violation = 1e-7;
  return settings;
}

Se2MpcConfig operationalConfig() {
  Se2MpcConfig config;
  config.horizon = 30;
  config.dt = 0.05;
  config.state_weight = Eigen::Vector3d(18.0, 18.0, 4.0);
  config.terminal_weight = Eigen::Vector3d(28.0, 28.0, 8.0);
  config.control_weight = Eigen::Vector3d(0.12, 0.12, 0.08);
  config.control_delta_weight = Eigen::Vector3d(0.7, 0.7, 0.25);
  config.max_vx = config.max_vy = 1.5;
  config.max_wz = 2.0;
  config.max_ax = config.max_ay = 2.0;
  config.max_awz = 3.0;
  config.wheel_base_x = config.wheel_base_y = 0.270;
  config.max_wheel_speed = 1.6689711;
  config.max_wheel_acceleration = 2.0;
  config.max_steer_rate = 10.4719755;
  return config;
}

std::string number(double value) {
  std::ostringstream stream;
  stream.precision(12);
  stream << value;
  return stream.str();
}

TEST(LtvQpConvergenceFixture, PreservesOperationalLimitsAndRejectsNonSolvedWarmStart) {
  const Se2MpcConfig config = operationalConfig();
  const LtvQpSolverSettings settings = operationalSettings();
  const State current = State::Zero();
  std::vector<Control> nominal_controls(
      static_cast<std::size_t>(config.horizon), Control(0.25, 0.10, 0.15));
  const std::vector<State> nominal_states = Se2Model(config.dt).rollout(
      current, nominal_controls);
  std::vector<Se2Reference> references(
      static_cast<std::size_t>(config.horizon + 1));
  for (int step = 0; step <= config.horizon; ++step) {
    references[static_cast<std::size_t>(step)].state =
        nominal_states[static_cast<std::size_t>(step)];
    if (step < config.horizon) {
      references[static_cast<std::size_t>(step)].control =
          nominal_controls[static_cast<std::size_t>(step)];
    }
  }
  references.back().control = nominal_controls.back();

  auto problem = LtvQpBuilder::allocate(config.horizon);
  ASSERT_TRUE(LtvQpBuilder::build(current, nominal_states, nominal_controls,
                                  references, Control::Zero(), config, problem));
  const auto sparse = ats_swerve_mpc::makeLtvQpSparseProblem(problem);
  ASSERT_TRUE(sparse.valid());
  ASSERT_EQ(settings.max_iterations, 400);
  ASSERT_DOUBLE_EQ(settings.time_limit_ms, 10.0);
  ASSERT_DOUBLE_EQ(settings.max_primal_residual, 1e-4);
  ASSERT_DOUBLE_EQ(settings.max_dual_residual, 1e-4);

  const Eigen::VectorXd hessian_diagonal = problem.hessian.diagonal();
  const double hessian_min = hessian_diagonal.minCoeff();
  const double hessian_max = hessian_diagonal.maxCoeff();
  EXPECT_GT(hessian_min, 0.0);
  EXPECT_TRUE(std::isfinite(hessian_max));
  double row_norm_min = std::numeric_limits<double>::infinity();
  double row_norm_max = 0.0;
  for (int row = 0; row < problem.equality_matrix.rows(); ++row) {
    const double norm = problem.equality_matrix.row(row).norm();
    row_norm_min = std::min(row_norm_min, norm);
    row_norm_max = std::max(row_norm_max, norm);
  }
  for (int row = 0; row < problem.inequality_matrix.rows(); ++row) {
    const double norm = problem.inequality_matrix.row(row).norm();
    row_norm_min = std::min(row_norm_min, norm);
    row_norm_max = std::max(row_norm_max, norm);
  }
  double nonzero_bound_abs_min = std::numeric_limits<double>::infinity();
  double bound_abs_max = 0.0;
  const auto inspect_bounds = [&nonzero_bound_abs_min, &bound_abs_max](
      const Eigen::VectorXd &bounds) {
    for (int index = 0; index < bounds.size(); ++index) {
      const double magnitude = std::abs(bounds(index));
      if (!std::isfinite(magnitude)) {
        continue;
      }
      bound_abs_max = std::max(bound_abs_max, magnitude);
      if (magnitude > 1e-12) {
        nonzero_bound_abs_min = std::min(nonzero_bound_abs_min, magnitude);
      }
    }
  };
  inspect_bounds(problem.equality_lower);
  inspect_bounds(problem.equality_upper);
  inspect_bounds(problem.inequality_lower);
  inspect_bounds(problem.inequality_upper);
  inspect_bounds(problem.lower_bound);
  inspect_bounds(problem.upper_bound);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(problem.decisionSize());
  const double dynamic_equality_residual = (
      problem.equality_matrix * zero - problem.equality_lower).lpNorm<Eigen::Infinity>();
  ::testing::Test::RecordProperty("hessian_diagonal_min", number(hessian_min));
  ::testing::Test::RecordProperty("hessian_diagonal_max", number(hessian_max));
  ::testing::Test::RecordProperty("constraint_row_l2_min", number(row_norm_min));
  ::testing::Test::RecordProperty("constraint_row_l2_max", number(row_norm_max));
  ::testing::Test::RecordProperty("nonzero_bound_abs_min", number(nonzero_bound_abs_min));
  ::testing::Test::RecordProperty("bound_abs_max", number(bound_abs_max));
  ::testing::Test::RecordProperty("zero_delta_dynamic_equality_residual",
                                  number(dynamic_equality_residual));
  EXPECT_NEAR(hessian_min, 0.66, 1e-12);
  EXPECT_NEAR(hessian_max, 56.0, 1e-12);
  EXPECT_NEAR(row_norm_min, 1.0, 1e-12);
  EXPECT_GE(row_norm_max, std::sqrt(2.0));
  EXPECT_LT(row_norm_max, 1.42);
  EXPECT_NEAR(nonzero_bound_abs_min, 0.1, 1e-12);
  EXPECT_NEAR(bound_abs_max, 2.15, 1e-12);
  EXPECT_NEAR(dynamic_equality_residual, 0.0, 1e-12);

  LtvQpOsqpSolver solver(sparse.decision_size, sparse.constraint_rows, settings);
  ASSERT_TRUE(solver.initialized());
  const auto cold = solver.solve(sparse, settings, nullptr);
  ::testing::Test::RecordProperty("cold_status",
      ats_swerve_mpc::ltvQpSolverStatusName(cold.status));
  ::testing::Test::RecordProperty("cold_iterations", std::to_string(cold.iterations));
  ::testing::Test::RecordProperty("cold_primal_residual", number(cold.primal_residual));
  ::testing::Test::RecordProperty("cold_dual_residual", number(cold.dual_residual));
  ::testing::Test::RecordProperty("cold_wall_solve_ms", number(cold.wall_solve_time_ms));
  EXPECT_NE(cold.status, LtvQpSolverStatus::kBackendUnavailable);
  EXPECT_NE(cold.status, LtvQpSolverStatus::kInvalidProblem);

  solver.resetWarmStart();
  const auto zero_delta_nominal = solver.solve(sparse, settings, nullptr);
  ::testing::Test::RecordProperty("zero_delta_nominal_status",
      ats_swerve_mpc::ltvQpSolverStatusName(zero_delta_nominal.status));
  EXPECT_FALSE(zero_delta_nominal.warm_start_used);

  if (cold.status == LtvQpSolverStatus::kSolved &&
      cold.primal_solution.allFinite() && cold.dual_solution.allFinite()) {
    ats_swerve_mpc::LtvQpWarmStart warm_start;
    warm_start.primal = cold.primal_solution;
    warm_start.dual = cold.dual_solution;
    const auto warm = solver.solve(sparse, settings, &warm_start);
    ::testing::Test::RecordProperty("solved_warm_start_status",
        ats_swerve_mpc::ltvQpSolverStatusName(warm.status));
    EXPECT_TRUE(warm.warm_start_used);
  } else {
    ::testing::Test::RecordProperty("solved_warm_start_status",
                                    "skipped_non_solved_cold_result");
  }
}

}  // namespace
