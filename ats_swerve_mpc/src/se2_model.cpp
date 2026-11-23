// Copyright 2026

#include "ats_swerve_mpc/se2_model.hpp"

#include <cmath>

namespace ats_swerve_mpc {

/**
 * @brief 以车体系全向速度推进世界系状态一个 dt（SE(2) 解析指数映射积分）。
 * @details 对任意步长 dt，恒定车体系速度 [vx, vy, wz] 在 SE(2) 上的解析积分为
 *          dt * sinc(wz*dt/2) * R(yaw + wz*dt/2) * [vx, vy]^T，彻底消除前向欧拉
 *          在不同步长下的离散截断误差；该函数是 iLQR 名义轨迹和 QP primal 重建共用的唯一实现。
 */
State Se2Model::dynamics(const State &state, const Control &control) const {
  return dynamics(state, control, dt_);
}

State Se2Model::dynamics(const State &state, const Control &control, double dt) {
  const double half_theta = 0.5 * dt * control(2);
  const double half_yaw = state(2) + half_theta;
  const double cosine = std::cos(half_yaw);
  const double sine = std::sin(half_yaw);
  double sinc_half = 1.0;
  if (std::abs(half_theta) < 1e-4) {
    const double a2 = half_theta * half_theta;
    sinc_half = 1.0 - a2 / 6.0 + (a2 * a2) / 120.0;
  } else {
    sinc_half = std::sin(half_theta) / half_theta;
  }
  const double scale = dt * sinc_half;
  State next = state;
  next(0) += scale * (control(0) * cosine - control(1) * sine);
  next(1) += scale * (control(0) * sine + control(1) * cosine);
  next(2) = normalizeAngle(state(2) + dt * control(2));
  return next;
}

/**
 * @brief 计算 SE(2) 解析指数映射动力学的一阶解析 Jacobian。
 * @details 状态 Jacobian 保留 yaw 对世界系平移的耦合，控制 Jacobian 包含车体系到世界系的
 *          旋转映射以及 sinc 缩放与中点航向对 wz 的精确偏导，确保 LTV-QP 与 iLQR 线性化严格一致。
 */
void Se2Model::jacobians(const State &state, const Control &control,
                         Eigen::Matrix3d &state_jacobian,
                         Eigen::Matrix3d &control_jacobian) const {
  jacobians(state, control, dt_, state_jacobian, control_jacobian);
}

void Se2Model::jacobians(const State &state, const Control &control, double dt,
                         Eigen::Matrix3d &state_jacobian,
                         Eigen::Matrix3d &control_jacobian) {
  const double half_theta = 0.5 * dt * control(2);
  const double half_yaw = state(2) + half_theta;
  const double cosine = std::cos(half_yaw);
  const double sine = std::sin(half_yaw);
  double sinc_half = 1.0;
  double dsinc_dhalf = 0.0;
  if (std::abs(half_theta) < 1e-4) {
    const double a2 = half_theta * half_theta;
    sinc_half = 1.0 - a2 / 6.0 + (a2 * a2) / 120.0;
    dsinc_dhalf = -half_theta / 3.0 + (half_theta * a2) / 30.0;
  } else {
    const double sin_a = std::sin(half_theta);
    const double cos_a = std::cos(half_theta);
    sinc_half = sin_a / half_theta;
    dsinc_dhalf = (half_theta * cos_a - sin_a) / (half_theta * half_theta);
  }
  const double vw_x = control(0) * cosine - control(1) * sine;
  const double vw_y = control(0) * sine + control(1) * cosine;
  const double dvw_dyaw_x = -control(0) * sine - control(1) * cosine;
  const double dvw_dyaw_y = control(0) * cosine - control(1) * sine;
  const double scale = dt * sinc_half;
  const double half_dt_sq = 0.5 * dt * dt;
  state_jacobian.setIdentity();
  state_jacobian(0, 2) = scale * dvw_dyaw_x;
  state_jacobian(1, 2) = scale * dvw_dyaw_y;
  control_jacobian.setZero();
  control_jacobian(0, 0) = scale * cosine;
  control_jacobian(0, 1) = -scale * sine;
  control_jacobian(0, 2) = half_dt_sq * (dsinc_dhalf * vw_x + sinc_half * dvw_dyaw_x);
  control_jacobian(1, 0) = scale * sine;
  control_jacobian(1, 1) = scale * cosine;
  control_jacobian(1, 2) = half_dt_sq * (dsinc_dhalf * vw_y + sinc_half * dvw_dyaw_y);
  control_jacobian(2, 2) = dt;
}

/**
 * @brief 对控制序列执行确定性前向仿真。
 * @details 返回值始终包含初始状态，故长度为 controls.size()+1；调用者必须自行对
 *          输入有限性和长度进行门控，避免把无效求解结果转为诊断轨迹。
 */
std::vector<State> Se2Model::rollout(
    const State &initial, const std::vector<Control> &controls) const {
  std::vector<State> states;
  states.reserve(controls.size() + 1);
  states.push_back(initial);
  for (const auto &control : controls) {
    states.push_back(dynamics(states.back(), control));
  }
  return states;
}

Control Se2Model::rotateControlLinear(const Control &control, double yaw_delta) {
  if (!std::isfinite(yaw_delta) || std::abs(yaw_delta) <= 1e-12) {
    return control;
  }
  const double c = std::cos(yaw_delta);
  const double s = std::sin(yaw_delta);
  Control rotated = control;
  rotated(0) = c * control(0) + s * control(1);
  rotated(1) = -s * control(0) + c * control(1);
  return rotated;
}

Eigen::Matrix3d Se2Model::rotateControlLinearJacobian(const Control &control,
                                                      double step_dt) {
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
  if (!std::isfinite(step_dt) || std::abs(step_dt) <= 1e-12) {
    return jacobian;
  }
  const double yaw_delta = control(2) * step_dt;
  if (!std::isfinite(yaw_delta)) {
    return jacobian;
  }
  const double c = std::cos(yaw_delta);
  const double s = std::sin(yaw_delta);
  jacobian(0, 0) = c;
  jacobian(0, 1) = s;
  jacobian(0, 2) = step_dt * (-s * control(0) + c * control(1));
  jacobian(1, 0) = -s;
  jacobian(1, 1) = c;
  jacobian(1, 2) = step_dt * (-c * control(0) - s * control(1));
  jacobian(2, 0) = 0.0;
  jacobian(2, 1) = 0.0;
  jacobian(2, 2) = 1.0;
  return jacobian;
}

double Se2Model::actuatorLagGamma(double dt, double lag_tau_sec) {
  if (!std::isfinite(lag_tau_sec) || lag_tau_sec <= 1e-6 ||
      !std::isfinite(dt) || dt <= 1e-9) {
    return 1.0;
  }
  const double ratio = dt / lag_tau_sec;
  double gamma = 1.0;
  if (ratio < 1e-4) {
    gamma = 0.5 * ratio - (ratio * ratio) / 6.0 +
            (ratio * ratio * ratio) / 24.0;
  } else {
    gamma = 1.0 - (1.0 - std::exp(-ratio)) / ratio;
  }
  if (gamma < 0.05) {
    return 0.05;
  }
  if (gamma > 1.0) {
    return 1.0;
  }
  return gamma;
}

Control Se2Model::effectiveControlWithLag(const Control &control,
                                          const Control &previous_control,
                                          double previous_carry_dt,
                                          double step_dt,
                                          double lag_tau_sec) {
  const double gamma = actuatorLagGamma(step_dt, lag_tau_sec);
  if (gamma >= 1.0 - 1e-12) {
    return control;
  }
  const Control carried_previous = rotateControlLinear(
      previous_control, previous_control(2) * previous_carry_dt);
  return (1.0 - gamma) * carried_previous + gamma * control;
}

State Se2Model::dynamicsWithLag(const State &state, const Control &control,
                                const Control &previous_control,
                                double previous_carry_dt, double dt,
                                double lag_tau_sec) {
  const Control effective = effectiveControlWithLag(
      control, previous_control, previous_carry_dt, dt, lag_tau_sec);
  return dynamics(state, effective, dt);
}

void Se2Model::jacobiansWithLag(const State &state, const Control &control,
                                const Control &previous_control,
                                double previous_carry_dt, double dt,
                                double lag_tau_sec,
                                Eigen::Matrix3d &state_jacobian,
                                Eigen::Matrix3d &current_control_jacobian,
                                Eigen::Matrix3d &previous_control_jacobian) {
  const double gamma = actuatorLagGamma(dt, lag_tau_sec);
  if (gamma >= 1.0 - 1e-12) {
    jacobians(state, control, dt, state_jacobian, current_control_jacobian);
    previous_control_jacobian.setZero();
    return;
  }
  const Control effective = effectiveControlWithLag(
      control, previous_control, previous_carry_dt, dt, lag_tau_sec);
  Eigen::Matrix3d effective_b = Eigen::Matrix3d::Zero();
  jacobians(state, effective, dt, state_jacobian, effective_b);
  current_control_jacobian = gamma * effective_b;
  previous_control_jacobian =
      (1.0 - gamma) * effective_b *
      rotateControlLinearJacobian(previous_control, previous_carry_dt);
}

std::vector<State> Se2Model::rolloutWithLag(
    const State &initial, const std::vector<Control> &controls,
    const Control &last_control, double first_step_dt, double dt,
    double lag_tau_sec) {
  if (!std::isfinite(lag_tau_sec) || lag_tau_sec <= 1e-6) {
    return Se2Model(dt).rollout(initial, controls);
  }
  const double effective_first_dt =
      (std::isfinite(first_step_dt) && first_step_dt > 0.0) ? first_step_dt
                                                            : dt;
  std::vector<State> states;
  states.reserve(controls.size() + 1);
  states.push_back(initial);
  Control previous = last_control;
  for (std::size_t index = 0; index < controls.size(); ++index) {
    const double carry_dt = index == 0 ? effective_first_dt : dt;
    states.push_back(dynamicsWithLag(states.back(), controls[index], previous,
                                     carry_dt, dt, lag_tau_sec));
    previous = controls[index];
  }
  return states;
}

/** @brief 用 atan2(sin,cos) 折返航向，避免在 +/-pi 边界累积角度漂移。 */
double Se2Model::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

/** @brief 返回 lhs-rhs 的 SE(2) 误差，其中 yaw 采用最短旋转方向。 */
State Se2Model::stateDifference(const State &lhs, const State &rhs) {
  State difference = lhs - rhs;
  difference(2) = normalizeAngle(difference(2));
  return difference;
}

}  // namespace ats_swerve_mpc
