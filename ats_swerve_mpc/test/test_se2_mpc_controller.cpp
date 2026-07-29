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

// 生成沿给定车体控制匀速前进的参考序列（状态由该控制积分得到）。
std::vector<ats_swerve_mpc::Se2Reference> straightReferences(
    int horizon, double dt, const ats_swerve_mpc::Control &control) {
  std::vector<ats_swerve_mpc::Se2Reference> references;
  references.reserve(static_cast<std::size_t>(horizon + 1));
  ats_swerve_mpc::State state = ats_swerve_mpc::State::Zero();
  for (int step = 0; step <= horizon; ++step) {
    ats_swerve_mpc::Se2Reference reference;
    reference.state = state;
    reference.control = control;
    references.push_back(reference);
    state(0) += dt * control(0);
    state(1) += dt * control(1);
    state(2) += dt * control(2);
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
    // 舵角变化量必须取带符号点积：对点积取绝对值会把 180° 反向翻转判成 0°。
    const double cosine = std::clamp(
        after[index].dot(before[index]) /
            (after[index].norm() * before[index].norm()),
        -1.0, 1.0);
    EXPECT_LE(std::acos(cosine), config.max_steer_rate * config.dt + 1e-7);
  }
}

// 反向翻转回归：参考要求瞬间从 +x 横移切到 -x 横移，舵角速率约束必须拦住 pi 翻转。
TEST(Se2MpcController, RejectsInstantModuleFlipUnderSteerRateLimit) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 4;
  config.dt = 0.1;
  config.max_vx = 2.0;
  config.max_vy = 2.0;
  config.max_wz = 2.0;
  config.max_ax = 100.0;
  config.max_ay = 100.0;
  config.max_awz = 100.0;
  config.wheel_base_x = 0.270;
  config.wheel_base_y = 0.270;
  config.max_wheel_speed = 5.0;
  config.max_steer_rate = 0.5;
  ats_swerve_mpc::Se2MpcController controller(config);

  const ats_swerve_mpc::Control previous_control(1.0, 0.0, 0.0);
  auto references =
      straightReferences(config.horizon, config.dt,
                         ats_swerve_mpc::Control(-1.0, 0.0, 0.0));
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(),
                                       references, previous_control);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  const auto before = wheelVelocities(previous_control, config.wheel_base_x,
                                      config.wheel_base_y);
  const auto after = wheelVelocities(result.controls.front(),
                                     config.wheel_base_x, config.wheel_base_y);
  for (std::size_t index = 0; index < after.size(); ++index) {
    if (after[index].norm() < 1e-4 || before[index].norm() < 1e-4) {
      continue;
    }
    const double cosine = std::clamp(
        after[index].dot(before[index]) /
            (after[index].norm() * before[index].norm()),
        -1.0, 1.0);
    EXPECT_LE(std::acos(cosine), config.max_steer_rate * config.dt + 1e-7);
  }
  // 首个下发控制不允许直接反向：舵角速率约束只允许先减速到零速死区（1e-4 m/s
  // 量级），下一周期再由舵机转向，绝不能一步跨到 -1.0 的反向速度。
  EXPECT_GT(result.controls.front()(0), -2e-4);
  // 该周期必须如实上报"增量被模块约束回退"，节点据此打印控制饱和告警。
  EXPECT_TRUE(result.increment_limited);
}

// 半轴偏置缺配时，单轮速度上限仍必须约束住平移分量（此前会被整体关闭）。
TEST(Se2MpcController, KeepsWheelSpeedLimitWhenWheelBaseUnset) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 6;
  config.dt = 0.1;
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_wz = 5.0;
  config.max_ax = 100.0;
  config.max_ay = 100.0;
  config.max_awz = 100.0;
  config.wheel_base_x = 0.0;
  config.wheel_base_y = 0.0;
  config.max_wheel_speed = 0.6;
  ats_swerve_mpc::Se2MpcController controller(config);

  auto references = lateralReferences(config.horizon, config.dt);
  for (auto &reference : references) {
    reference.control << 3.0, 3.0, 0.0;
  }
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(),
                                       references,
                                       ats_swerve_mpc::Control::Zero());

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.controls.empty());
  for (const auto &control : result.controls) {
    EXPECT_LE(std::hypot(control(0), control(1)), 0.6000001);
  }
  // 模块级约束未完全配置时必须如实上报，供节点打印实车高危配置告警。
  EXPECT_FALSE(result.module_limits_active);
}

// 参考序列长度不足时必须判定失败，节点据此进入零速度兜底。
TEST(Se2MpcController, FailsWhenReferenceHorizonTooShort) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 8;
  config.dt = 0.1;
  ats_swerve_mpc::Se2MpcController controller(config);

  const auto references = lateralReferences(config.horizon - 2, config.dt);
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(),
                                       references,
                                       ats_swerve_mpc::Control::Zero());

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.controls.empty());
  EXPECT_EQ(result.accepted_iterations, 0);
}

// 迭代预算为零时不可能产生可信解，success 必须为 false（此前只看序列长度会误报成功）。
TEST(Se2MpcController, FailsWhenNoIterationIsCertified) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 8;
  config.dt = 0.1;
  config.max_iterations = 0;
  ats_swerve_mpc::Se2MpcController controller(config);

  const auto references = lateralReferences(config.horizon, config.dt);
  const auto result = controller.solve(ats_swerve_mpc::State::Zero(),
                                       references,
                                       ats_swerve_mpc::Control::Zero());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(result.accepted_iterations, 0);
}

// 已收敛（参考即当前解）时，即使没有迭代被接受也应判定成功，避免误触兜底。
TEST(Se2MpcController, SucceedsAtStationaryWarmStart) {
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 10;
  config.dt = 0.1;
  config.max_iterations = 12;
  ats_swerve_mpc::Se2MpcController controller(config);

  const auto references =
      straightReferences(config.horizon, config.dt,
                         ats_swerve_mpc::Control(0.5, 0.0, 0.0));
  ats_swerve_mpc::Se2MpcResult result;
  // 连续求解让暖启动收敛到驻点，最后一次求解应处于"零更新量"分支。
  for (int cycle = 0; cycle < 30; ++cycle) {
    result = controller.solve(ats_swerve_mpc::State::Zero(), references,
                              ats_swerve_mpc::Control(0.5, 0.0, 0.0));
    ASSERT_TRUE(result.success);
  }
  EXPECT_TRUE(result.success);
}

} // namespace
