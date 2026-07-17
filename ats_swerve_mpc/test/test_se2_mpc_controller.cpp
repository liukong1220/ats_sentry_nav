// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

namespace {

std::vector<ats_swerve_mpc::Se2Reference> lateralReferences(int horizon,
                                                            double dt) {
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

double maxWheelSpeed(const ats_swerve_mpc::Control &control,
                     double wheel_base_x, double wheel_base_y) {
  double maximum = 0.0;
  for (const double x : {-wheel_base_x, wheel_base_x}) {
    for (const double y : {-wheel_base_y, wheel_base_y}) {
      const double wheel_vx = control(0) - control(2) * y;
      const double wheel_vy = control(1) + control(2) * x;
      maximum = std::max(maximum, std::hypot(wheel_vx, wheel_vy));
    }
  }
  return maximum;
}

std::vector<Eigen::Vector2d> wheelVelocities(
    const ats_swerve_mpc::Control &control, double wheel_base_x,
    double wheel_base_y) {
  std::vector<Eigen::Vector2d> velocities;
  for (const double x : {-wheel_base_x, wheel_base_x}) {
    for (const double y : {-wheel_base_y, wheel_base_y}) {
      velocities.emplace_back(control(0) - control(2) * y,
                              control(1) + control(2) * x);
    }
  }
  return velocities;
}

TEST(Se2MpcController, CommandsTrueLateralMotionWithoutYawCoupling) {
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
  const auto result =
      controller.solve(ats_swerve_mpc::State::Zero(),
                       lateralReferences(config.horizon, config.dt),
                       ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  EXPECT_NEAR(result.controls.front()(0), 0.0, 0.05);
  EXPECT_GT(result.controls.front()(1), 0.2);
  EXPECT_NEAR(result.controls.front()(2), 0.0, 0.05);
  EXPECT_GT(result.states.back()(1), 0.2);
  EXPECT_NEAR(result.states.back()(2), 0.0, 0.05);
}

TEST(Se2MpcController, RespectsVelocityAndAccelerationBounds) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 10;
  config.dt = 0.1;
  config.max_vy = 0.4;
  config.max_ay = 0.5;
  ats_swerve_mpc::Se2MpcController controller(config);
  const auto result =
      controller.solve(ats_swerve_mpc::State::Zero(),
                       lateralReferences(config.horizon, config.dt),
                       ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  EXPECT_LE(std::abs(result.controls.front()(1)),
            config.max_ay * config.dt + 1e-9);
  for (const auto &control : result.controls) {
    EXPECT_LE(std::abs(control(1)), config.max_vy + 1e-9);
  }
}

TEST(Se2MpcController, ClampsBodyCommandByPerWheelSpeed) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 4;
  config.dt = 0.1;
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_wz = 10.0;
  config.max_ax = 100.0;
  config.max_ay = 100.0;
  config.max_awz = 100.0;
  config.wheel_base_x = 0.270;
  config.wheel_base_y = 0.270;
  config.max_wheel_speed = 0.6;
  ats_swerve_mpc::Se2MpcController controller(config);

  auto references = lateralReferences(config.horizon, config.dt);
  for (auto &reference : references) {
    reference.control << 3.0, 3.0, 8.0;
  }
  const auto result =
      controller.solve(ats_swerve_mpc::State::Zero(), references,
                       ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  for (const auto &control : result.controls) {
    EXPECT_LE(maxWheelSpeed(control, config.wheel_base_x, config.wheel_base_y),
              0.6000001);
  }
}

TEST(Se2MpcController, ClampsEveryWheelVelocityIncrement) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 5;
  config.dt = 0.1;
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_wz = 10.0;
  config.max_ax = 100.0;
  config.max_ay = 100.0;
  config.max_awz = 100.0;
  config.wheel_base_x = 0.270;
  config.wheel_base_y = 0.270;
  config.max_wheel_speed = 5.0;
  config.max_wheel_acceleration = 0.5;
  ats_swerve_mpc::Se2MpcController controller(config);

  auto references = lateralReferences(config.horizon, config.dt);
  for (auto &reference : references) {
    reference.control << 2.0, 2.0, 4.0;
  }
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(), references,
                                       ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ats_swerve_mpc::Control previous = ats_swerve_mpc::Control::Zero();
  for (const auto &control : result.controls) {
    const auto before = wheelVelocities(previous, config.wheel_base_x,
                                        config.wheel_base_y);
    const auto after = wheelVelocities(control, config.wheel_base_x,
                                       config.wheel_base_y);
    for (std::size_t index = 0; index < after.size(); ++index) {
      EXPECT_LE((after[index] - before[index]).norm(),
                config.max_wheel_acceleration * config.dt + 1e-7);
    }
    previous = control;
  }
}

TEST(Se2MpcController, ClampsMovingWheelSteerRate) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 5;
  config.dt = 0.1;
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_wz = 5.0;
  config.max_ax = 100.0;
  config.max_ay = 100.0;
  config.max_awz = 100.0;
  config.wheel_base_x = 0.270;
  config.wheel_base_y = 0.270;
  config.max_wheel_speed = 5.0;
  config.max_steer_rate = 0.5;
  ats_swerve_mpc::Se2MpcController controller(config);

  auto references = lateralReferences(config.horizon, config.dt);
  const ats_swerve_mpc::Control previous_control(0.5, 0.0, 0.0);
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(), references,
                                       previous_control);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  const auto before = wheelVelocities(previous_control, config.wheel_base_x,
                                      config.wheel_base_y);
  const auto after = wheelVelocities(result.controls.front(), config.wheel_base_x,
                                     config.wheel_base_y);
  for (std::size_t index = 0; index < after.size(); ++index) {
    const double cosine = std::clamp(
        std::abs(after[index].dot(before[index])) /
            (after[index].norm() * before[index].norm()),
        0.0, 1.0);
    EXPECT_LE(std::acos(cosine), config.max_steer_rate * config.dt + 1e-7);
  }
}

} // namespace
