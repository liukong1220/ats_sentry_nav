// Copyright 2026

#include <gtest/gtest.h>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

namespace
{

std::vector<ats_swerve_mpc::Se2Reference> lateralReferences(int horizon, double dt)
{
  std::vector<ats_swerve_mpc::Se2Reference> references;
  references.reserve(static_cast<std::size_t>(horizon + 1));
  for (int step = 0; step <= horizon; ++step) {
    ats_swerve_mpc::Se2Reference reference;
    reference.state << 0.0, 0.5 * step * dt, 0.0;
    reference.control << 0.0, 0.5, 0.0;
    references.push_back(reference);
  }
  return references;
}

TEST(Se2MpcController, CommandsTrueLateralMotionWithoutYawCoupling)
{
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 15;
  config.dt = 0.1;
  config.max_vx = 1.0;
  config.max_vy = 1.0;
  config.max_wz = 1.0;
  config.max_ax = 10.0;
  config.max_ay = 10.0;
  config.max_awz = 10.0;
  ats_swerve_mpc::Se2MpcController controller(config);
  const auto result = controller.solve(
    ats_swerve_mpc::State::Zero(), lateralReferences(config.horizon, config.dt),
    ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  EXPECT_NEAR(result.controls.front()(0), 0.0, 0.05);
  EXPECT_GT(result.controls.front()(1), 0.2);
  EXPECT_NEAR(result.controls.front()(2), 0.0, 0.05);
  EXPECT_GT(result.states.back()(1), 0.2);
  EXPECT_NEAR(result.states.back()(2), 0.0, 0.05);
}

TEST(Se2MpcController, RespectsVelocityAndAccelerationBounds)
{
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 10;
  config.dt = 0.1;
  config.max_vy = 0.4;
  config.max_ay = 0.5;
  ats_swerve_mpc::Se2MpcController controller(config);
  const auto result = controller.solve(
    ats_swerve_mpc::State::Zero(), lateralReferences(config.horizon, config.dt),
    ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  EXPECT_LE(std::abs(result.controls.front()(1)), config.max_ay * config.dt + 1e-9);
  for (const auto & control : result.controls) {
    EXPECT_LE(std::abs(control(1)), config.max_vy + 1e-9);
  }
}

}  // namespace
