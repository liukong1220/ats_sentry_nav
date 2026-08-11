// Copyright 2026

#include <gtest/gtest.h>

#include <limits>

#include "ats_swerve_mpc/qp/ltv_qp_problem.hpp"

namespace {

std::vector<ats_swerve_mpc::Se2Reference> references(int horizon, double dt) {
  std::vector<ats_swerve_mpc::Se2Reference> output;
  output.reserve(static_cast<std::size_t>(horizon + 1));
  ats_swerve_mpc::State state = ats_swerve_mpc::State::Zero();
  ats_swerve_mpc::Control control;
  control << 0.3, 0.1, 0.05;
  for (int step = 0; step <= horizon; ++step) {
    ats_swerve_mpc::Se2Reference reference;
    reference.state = state;
    reference.control = control;
    output.push_back(reference);
    state(0) += dt * control(0);
    state(1) += dt * control(1);
    state(2) += dt * control(2);
  }
  return output;
}

TEST(LtvQpBuilder, BuildsSharedDynamicsAndBoundedControlIncrementProblem) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 3;
  config.dt = 0.1;
  config.max_vx = 1.0;
  config.max_vy = 1.0;
  config.max_wz = 1.0;
  config.max_ax = 0.5;
  config.max_ay = 0.5;
  config.max_awz = 0.5;
  const auto refs = references(config.horizon, config.dt);
  std::vector<ats_swerve_mpc::Control> controls;
  for (int step = 0; step < config.horizon; ++step) {
    controls.push_back(refs[static_cast<std::size_t>(step)].control);
  }
  ats_swerve_mpc::Se2Model model(config.dt);
  const auto states = model.rollout(ats_swerve_mpc::State::Zero(), controls);
  const auto problem = ats_swerve_mpc::LtvQpBuilder::build(
      ats_swerve_mpc::State::Zero(), states, controls, refs,
      ats_swerve_mpc::Control::Zero(), config);

  ASSERT_TRUE(problem.valid) << problem.validation_error;
  const int expected_size = 3 * (config.horizon + 1) + 3 * config.horizon;
  EXPECT_EQ(problem.hessian.rows(), expected_size);
  EXPECT_EQ(problem.hessian.cols(), expected_size);
  EXPECT_EQ(problem.equality_matrix.rows(), 3 * (config.horizon + 1));
  EXPECT_EQ(problem.inequality_matrix.rows(), 3 * config.horizon);
  EXPECT_TRUE((problem.hessian - problem.hessian.transpose()).norm() < 1e-12);
  EXPECT_TRUE(problem.hessian.allFinite());
  EXPECT_TRUE(problem.gradient.allFinite());
  EXPECT_TRUE(problem.equality_matrix.allFinite());
  EXPECT_TRUE(problem.inequality_matrix.allFinite());
  EXPECT_NEAR(problem.equality_lower(0), 0.0, 1e-12);
  EXPECT_NEAR(problem.equality_upper(3), 0.0, 1e-12);
  EXPECT_NEAR(problem.inequality_lower(0), -0.05 - 0.3, 1e-12);
  EXPECT_NEAR(problem.inequality_upper(0), 0.05 - 0.3, 1e-12);
}

TEST(LtvQpBuilder, ActivatesZeroSpeedGuardInsteadOfInventingSteeringDirections) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 2;
  config.dt = 0.1;
  const std::vector<ats_swerve_mpc::Control> controls(
      2, ats_swerve_mpc::Control::Zero());
  const std::vector<ats_swerve_mpc::State> states(
      3, ats_swerve_mpc::State::Zero());
  const auto refs = references(config.horizon, config.dt);
  const auto problem = ats_swerve_mpc::LtvQpBuilder::build(
      ats_swerve_mpc::State::Zero(), states, controls, refs,
      ats_swerve_mpc::Control::Zero(), config);

  ASSERT_TRUE(problem.valid) << problem.validation_error;
  EXPECT_TRUE(problem.zero_speed_guard_active);
  for (const bool valid : problem.angle_rate_linearization_valid) {
    EXPECT_FALSE(valid);
  }
}

TEST(LtvQpBuilder, RejectsMalformedNominalHorizon) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 3;
  config.dt = 0.1;
  const std::vector<ats_swerve_mpc::State> states(2,
                                                  ats_swerve_mpc::State::Zero());
  const std::vector<ats_swerve_mpc::Control> controls(3,
                                                       ats_swerve_mpc::Control::Zero());
  const auto refs = references(config.horizon, config.dt);
  const auto problem = ats_swerve_mpc::LtvQpBuilder::build(
      ats_swerve_mpc::State::Zero(), states, controls, refs,
      ats_swerve_mpc::Control::Zero(), config);
  EXPECT_FALSE(problem.valid);
  EXPECT_FALSE(problem.validation_error.empty());
}

TEST(LtvQpBuilder, RejectsNonConvexWeightsAndNonFiniteLimits) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 2;
  config.dt = 0.1;
  const auto refs = references(config.horizon, config.dt);
  std::vector<ats_swerve_mpc::Control> controls(
      static_cast<std::size_t>(config.horizon), refs.front().control);
  const auto states = ats_swerve_mpc::Se2Model(config.dt).rollout(
      ats_swerve_mpc::State::Zero(), controls);

  config.state_weight(0) = -1.0;
  auto problem = ats_swerve_mpc::LtvQpBuilder::build(
      ats_swerve_mpc::State::Zero(), states, controls, refs,
      ats_swerve_mpc::Control::Zero(), config);
  EXPECT_FALSE(problem.valid);
  EXPECT_EQ(problem.validation_error,
            "QP weights must be finite and non-negative");

  config = ats_swerve_mpc::Se2MpcConfig();
  config.horizon = 2;
  config.dt = 0.1;
  config.max_vx = std::numeric_limits<double>::infinity();
  problem = ats_swerve_mpc::LtvQpBuilder::build(
      ats_swerve_mpc::State::Zero(), states, controls, refs,
      ats_swerve_mpc::Control::Zero(), config);
  EXPECT_FALSE(problem.valid);
  EXPECT_EQ(problem.validation_error,
            "non-finite or negative dynamics limit");
}

TEST(LtvQpBuilder, RejectsHorizonThatCannotFitFixedLtvDimensions) {
  const auto problem = ats_swerve_mpc::LtvQpBuilder::allocate(
      std::numeric_limits<int>::max());
  EXPECT_FALSE(problem.valid);
  EXPECT_EQ(problem.validation_error,
            "horizon must be positive and fit fixed LTV dimensions");
}

}  // namespace
