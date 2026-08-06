// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MODEL_HPP_
#define ATS_SWERVE_MPC__SE2_MODEL_HPP_

#include <vector>

#include <Eigen/Core>

namespace ats_swerve_mpc {

using State = Eigen::Vector3d;
using Control = Eigen::Vector3d;

/**
 * @brief Shared holonomic SE(2) prediction model.
 *
 * State is world-frame [x, y, yaw]. Control is body-frame [vx, vy, wz].
 * The model is intentionally independent of ROS and solver implementation so
 * iLQR and LTV-QP can use the same dynamics, Jacobians and rollout.
 */
class Se2Model {
public:
  explicit Se2Model(double dt = 0.1) : dt_(dt) {}

  void setTimeStep(double dt) { dt_ = dt; }
  double timeStep() const { return dt_; }

  State dynamics(const State &state, const Control &control) const;
  void jacobians(const State &state, const Control &control,
                 Eigen::Matrix3d &state_jacobian,
                 Eigen::Matrix3d &control_jacobian) const;
  std::vector<State> rollout(const State &initial,
                             const std::vector<Control> &controls) const;

  static double normalizeAngle(double angle);
  static State stateDifference(const State &lhs, const State &rhs);

private:
  double dt_ = 0.1;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__SE2_MODEL_HPP_
