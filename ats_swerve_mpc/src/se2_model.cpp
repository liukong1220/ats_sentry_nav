// Copyright 2026

#include "ats_swerve_mpc/se2_model.hpp"

#include <cmath>

namespace ats_swerve_mpc {

/**
 * @brief 以车体系全向速度推进世界系状态一个 dt。
 * @details vx/vy 先按当前 yaw 旋转至世界系，wz 只更新 yaw；该函数是 iLQR
 *          名义轨迹和 QP primal 重建共用的唯一离散动力学实现。
 */
State Se2Model::dynamics(const State &state, const Control &control) const {
  const double yaw = state(2);
  State next = state;
  next(0) += dt_ * (control(0) * std::cos(yaw) - control(1) * std::sin(yaw));
  next(1) += dt_ * (control(0) * std::sin(yaw) + control(1) * std::cos(yaw));
  next(2) = normalizeAngle(state(2) + dt_ * control(2));
  return next;
}

/**
 * @brief 计算离散动力学的一阶 Jacobian。
 * @details 状态 Jacobian 保留 yaw 对世界系平移的耦合，控制 Jacobian 完成车体系到
 *          世界系的旋转映射，确保 LTV-QP 和 iLQR 使用同一线性化假设。
 */
void Se2Model::jacobians(const State &state, const Control &control,
                         Eigen::Matrix3d &state_jacobian,
                         Eigen::Matrix3d &control_jacobian) const {
  const double yaw = state(2);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  state_jacobian.setIdentity();
  state_jacobian(0, 2) = dt_ * (-control(0) * sine - control(1) * cosine);
  state_jacobian(1, 2) = dt_ * (control(0) * cosine - control(1) * sine);
  control_jacobian.setZero();
  control_jacobian(0, 0) = dt_ * cosine;
  control_jacobian(0, 1) = -dt_ * sine;
  control_jacobian(1, 0) = dt_ * sine;
  control_jacobian(1, 1) = dt_ * cosine;
  control_jacobian(2, 2) = dt_;
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
