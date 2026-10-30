// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

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

// 附录 A.9 旋转相位补偿：没有延迟或角速度时必须原样返回，位置永远不变，yaw 归一化。
TEST(Se2MpcController, RotationPhaseCompensationOnlyShiftsYaw) {
  const ats_swerve_mpc::State measured(1.0, -2.0, 0.3);
  EXPECT_EQ(ats_swerve_mpc::rotationPhaseCompensatedState(measured, 3.0, 0.0),
            measured);
  EXPECT_EQ(ats_swerve_mpc::rotationPhaseCompensatedState(measured, 0.0, 0.2),
            measured);
  EXPECT_EQ(ats_swerve_mpc::rotationPhaseCompensatedState(
                measured, std::numeric_limits<double>::quiet_NaN(), 0.2),
            measured);

  const auto shifted =
      ats_swerve_mpc::rotationPhaseCompensatedState(measured, 3.0, 0.2);
  EXPECT_DOUBLE_EQ(shifted(0), measured(0));
  EXPECT_DOUBLE_EQ(shifted(1), measured(1));
  EXPECT_NEAR(shifted(2), 0.3 + 3.0 * 0.2 / 2.0, 1e-12);

  // 越过 +pi 时折回 [-pi, pi]，与 Se2Model::normalizeAngle 一致。
  const ats_swerve_mpc::State near_pi(0.0, 0.0, M_PI - 0.05);
  const auto wrapped =
      ats_swerve_mpc::rotationPhaseCompensatedState(near_pi, 3.0, 0.1);
  EXPECT_NEAR(wrapped(2), -M_PI + 0.10, 1e-12);
}

// 3 rad/s 自转下直线平移：指令在延迟 t_d 后才生效，生效时车体已多转 omega * t_d / 2。
// 按测量 yaw 旋转的车体系指令序列在世界系横向漂移，漂移率约 |v| * sin(omega * t_d / 2)；按
// psi + omega * t_d / 2 旋转（节点用的同一个 rotationPhaseCompensatedState）后漂移应基本消失。
// 度量：把 MPC 求出的整条控制序列从“指令生效时刻的真实状态”用同一个 Se2Model 重放，取世界系
// 横向（y）偏离直线 reference 的最大值。本用例只隔离旋转相位这一项：reference 取在指令生效
// 时刻（reference yaw = 生效 yaw，位置与测量重合）。
TEST(Se2MpcController, RotationPhaseCompensationReducesLateralDeviationWhileSpinning) {
  constexpr double kSpinRate = 3.0;   // rad/s，车体自转
  constexpr double kSpeed = 1.5;      // m/s，世界系沿 +x 直线平移
  constexpr double kDelay = 0.1;      // s，控制延迟 t_d
  constexpr double kYaw = 0.4;        // rad，当前测量 yaw
  // 指令生效时的真实 yaw，取 A.9 的相位：psi + omega * t_d / 2。
  constexpr double kExecutionYaw = kYaw + kSpinRate * kDelay / 2.0;
  ats_swerve_mpc::Se2MpcConfig config;
  config.horizon = 20;
  config.dt = 0.1;
  config.max_vx = 2.0;
  config.max_vy = 2.0;
  config.max_wz = 4.0;
  // 增量限幅不是本用例的考察对象，放宽到不会截断第一拍。
  config.max_ax = 10.0;
  config.max_ay = 10.0;
  config.max_awz = 10.0;

  // 世界系参考：位置沿 +x 匀速，yaw 自生效 yaw 起以 kSpinRate 自转；车体系前馈按参考 yaw 旋转。
  std::vector<ats_swerve_mpc::Se2Reference> references;
  for (int step = 0; step <= config.horizon; ++step) {
    const double time = step * config.dt;
    ats_swerve_mpc::Se2Reference reference;
    const double yaw = kExecutionYaw + kSpinRate * time;
    reference.state << kSpeed * time, 0.0, yaw;
    reference.control << std::cos(yaw) * kSpeed, -std::sin(yaw) * kSpeed,
        kSpinRate;
    references.push_back(reference);
  }
  const ats_swerve_mpc::State measured(0.0, 0.0, kYaw);
  // 上一拍已发布的指令：稳态自转 + 直线平移，wz 同时就是补偿用的 omega。
  const ats_swerve_mpc::Control last_control(
      std::cos(kExecutionYaw) * kSpeed, -std::sin(kExecutionYaw) * kSpeed,
      kSpinRate);

  ats_swerve_mpc::Se2MpcController uncompensated(config);
  ats_swerve_mpc::Se2MpcController compensated(config);
  const auto off = uncompensated.solve(measured, references, last_control);
  const auto on = compensated.solve(
      ats_swerve_mpc::rotationPhaseCompensatedState(
          measured, last_control(2), kDelay),
      references, last_control);
  ASSERT_TRUE(off.success);
  ASSERT_TRUE(on.success);

  // 指令生效时刻的真实状态：位置与测量重合，yaw 为 psi + omega * t_d / 2。
  const ats_swerve_mpc::State executed(0.0, 0.0, kExecutionYaw);
  const ats_swerve_mpc::Se2Model model(config.dt);
  const auto maxLateralDeviation =
      [&model, &executed](const std::vector<ats_swerve_mpc::Control> &controls) {
        double deviation = 0.0;
        for (const auto &state : model.rollout(executed, controls)) {
          deviation = std::max(deviation, std::abs(state(1)));
        }
        return deviation;
      };
  const double lateral_off = maxLateralDeviation(off.controls);
  const double lateral_on = maxLateralDeviation(on.controls);

  // 不补偿时整条序列按错相位旋转，约 2 s 内横向漂出数十厘米；补偿后只剩求解器的小残差。
  EXPECT_GT(lateral_off, 0.15);
  EXPECT_LT(lateral_on, 0.05);
  EXPECT_LT(lateral_on, 0.25 * lateral_off);
}

} // namespace
