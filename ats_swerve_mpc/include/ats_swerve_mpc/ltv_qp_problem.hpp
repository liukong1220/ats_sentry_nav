// Copyright 2026

#ifndef ATS_SWERVE_MPC__LTV_QP_PROBLEM_HPP_
#define ATS_SWERVE_MPC__LTV_QP_PROBLEM_HPP_

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/zero_speed_guard.hpp"

namespace ats_swerve_mpc {

/**
 * @brief Solver-independent LTV-MPC quadratic program description.
 *
 * Decision ordering is [delta_x_0 ... delta_x_N, delta_u_0 ... delta_u_N-1].
 * This type deliberately contains no solver status or ROS state.  A later
 * OSQP/HPIPM/qpOASES adapter must validate the hard constraints before its
 * result can reach the existing fail-stop control node.
 */
struct LtvQpProblem {
  bool valid = false;
  std::string validation_error;
  int horizon = 0;
  int state_dimension = 3;
  int control_dimension = 3;
  bool zero_speed_guard_active = false;
  std::array<bool, 4> angle_rate_linearization_valid{{false, false, false, false}};

  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd equality_matrix;
  Eigen::VectorXd equality_lower;
  Eigen::VectorXd equality_upper;
  Eigen::MatrixXd inequality_matrix;
  Eigen::VectorXd inequality_lower;
  Eigen::VectorXd inequality_upper;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;

  int stateOffset(int step) const { return state_dimension * step; }
  int controlOffset(int step) const {
    return state_dimension * (horizon + 1) + control_dimension * step;
  }
  int decisionSize() const {
    return state_dimension * (horizon + 1) + control_dimension * horizon;
  }
};

class LtvQpBuilder {
public:
  /**
   * @brief Build a convex quadratic tracking problem around a nominal rollout.
   *
   * The first phase includes linearized SE(2) dynamics, body velocity bounds
   * and body acceleration/control-increment bounds.  Wheel norm and steering
   * angle constraints are intentionally not approximated here; their low-speed
   * singularity is exposed through angle_rate_linearization_valid and must be
   * handled by the future backend before it is enabled in the control chain.
   */
  static LtvQpProblem build(
      const State &current_state, const std::vector<State> &nominal_states,
      const std::vector<Control> &nominal_controls,
      const std::vector<Se2Reference> &references, const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config = ZeroSpeedGuardConfig());
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__LTV_QP_PROBLEM_HPP_
