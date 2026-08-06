// Copyright 2026

#include "ats_swerve_mpc/se2_model.hpp"

#include <cmath>

namespace ats_swerve_mpc {

State Se2Model::dynamics(const State &state, const Control &control) const {
  const double yaw = state(2);
  State next = state;
  next(0) += dt_ * (control(0) * std::cos(yaw) - control(1) * std::sin(yaw));
  next(1) += dt_ * (control(0) * std::sin(yaw) + control(1) * std::cos(yaw));
  next(2) = normalizeAngle(state(2) + dt_ * control(2));
  return next;
}

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

double Se2Model::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

State Se2Model::stateDifference(const State &lhs, const State &rhs) {
  State difference = lhs - rhs;
  difference(2) = normalizeAngle(difference(2));
  return difference;
}

}  // namespace ats_swerve_mpc
