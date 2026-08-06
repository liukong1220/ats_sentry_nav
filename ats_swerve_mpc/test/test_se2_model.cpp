// Copyright 2026

#include <gtest/gtest.h>

#include "ats_swerve_mpc/se2_model.hpp"

namespace {

TEST(Se2Model, PreservesBodyToWorldHolonomicKinematics) {
  ats_swerve_mpc::Se2Model model(0.1);
  ats_swerve_mpc::State state;
  state << 1.0, -2.0, 1.5707963267948966;
  ats_swerve_mpc::Control control;
  control << 0.0, 0.5, 0.2;

  const auto next = model.dynamics(state, control);
  EXPECT_NEAR(next(0), 0.95, 1e-9);
  EXPECT_NEAR(next(1), -2.0, 1e-9);
  EXPECT_NEAR(next(2), 1.5907963267948966, 1e-9);
}

TEST(Se2Model, RolloutUsesTheSameDynamicsAsTheSingleStepModel) {
  ats_swerve_mpc::Se2Model model(0.05);
  const ats_swerve_mpc::State initial = ats_swerve_mpc::State::Zero();
  const std::vector<ats_swerve_mpc::Control> controls(3,
                                                       (ats_swerve_mpc::Control()
                                                        << 0.2, 0.1, 0.05)
                                                           .finished());

  const auto states = model.rollout(initial, controls);
  ASSERT_EQ(states.size(), 4u);
  EXPECT_TRUE((states[1] - model.dynamics(initial, controls[0])).norm() < 1e-12);
  EXPECT_TRUE((states[2] - model.dynamics(states[1], controls[1])).norm() < 1e-12);
  EXPECT_TRUE((states[3] - model.dynamics(states[2], controls[2])).norm() < 1e-12);
}

TEST(Se2Model, JacobiansMatchFiniteDifferenceLocally) {
  ats_swerve_mpc::Se2Model model(0.05);
  ats_swerve_mpc::State state;
  state << 0.4, -0.2, 0.7;
  ats_swerve_mpc::Control control;
  control << 0.3, -0.1, 0.2;
  Eigen::Matrix3d a;
  Eigen::Matrix3d b;
  model.jacobians(state, control, a, b);

  constexpr double epsilon = 1e-7;
  Eigen::Matrix3d numeric_a;
  Eigen::Matrix3d numeric_b;
  for (int column = 0; column < 3; ++column) {
    auto plus_state = state;
    auto minus_state = state;
    plus_state(column) += epsilon;
    minus_state(column) -= epsilon;
    numeric_a.col(column) =
        (model.dynamics(plus_state, control) -
         model.dynamics(minus_state, control)) /
        (2.0 * epsilon);

    auto plus_control = control;
    auto minus_control = control;
    plus_control(column) += epsilon;
    minus_control(column) -= epsilon;
    numeric_b.col(column) =
        (model.dynamics(state, plus_control) -
         model.dynamics(state, minus_control)) /
        (2.0 * epsilon);
  }
  EXPECT_TRUE((a - numeric_a).norm() < 1e-6);
  EXPECT_TRUE((b - numeric_b).norm() < 1e-6);
}

}  // namespace
