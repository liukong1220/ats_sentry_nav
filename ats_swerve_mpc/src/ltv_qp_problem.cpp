// Copyright 2026

#include "ats_swerve_mpc/ltv_qp_problem.hpp"

#include <cmath>
#include <limits>

namespace ats_swerve_mpc {
namespace {

using Matrix3 = Eigen::Matrix3d;

bool finiteStates(const std::vector<State> &states) {
  for (const auto &state : states) {
    if (!state.allFinite()) {
      return false;
    }
  }
  return true;
}

bool finiteControls(const std::vector<Control> &controls) {
  for (const auto &control : controls) {
    if (!control.allFinite()) {
      return false;
    }
  }
  return true;
}

std::array<Eigen::Vector2d, 4> moduleVelocities(
    const Control &control, const Se2MpcConfig &config) {
  const double x = std::max(0.0, config.wheel_base_x);
  const double y = std::max(0.0, config.wheel_base_y);
  const std::array<Eigen::Vector2d, 4> positions{{
      Eigen::Vector2d(x, y), Eigen::Vector2d(x, -y),
      Eigen::Vector2d(-x, y), Eigen::Vector2d(-x, -y)}};
  std::array<Eigen::Vector2d, 4> velocities;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    velocities[index] << control(0) - control(2) * positions[index](1),
        control(1) + control(2) * positions[index](0);
  }
  return velocities;
}

void addQuadraticBlock(LtvQpProblem &problem, int offset, const Matrix3 &weight,
                       const Eigen::Vector3d &error) {
  problem.hessian.block<3, 3>(offset, offset).noalias() += 2.0 * weight;
  problem.gradient.segment<3>(offset).noalias() += 2.0 * weight * error;
}

void addDeltaQuadratic(LtvQpProblem &problem, int current_offset,
                       int previous_offset, const Matrix3 &weight,
                       const Control &nominal_delta) {
  problem.hessian.block<3, 3>(current_offset, current_offset).noalias() +=
      2.0 * weight;
  problem.gradient.segment<3>(current_offset).noalias() +=
      2.0 * weight * nominal_delta;
  if (previous_offset < 0) {
    return;
  }
  problem.hessian.block<3, 3>(previous_offset, previous_offset).noalias() +=
      2.0 * weight;
  problem.gradient.segment<3>(previous_offset).noalias() -=
      2.0 * weight * nominal_delta;
  problem.hessian.block<3, 3>(current_offset, previous_offset).noalias() -=
      2.0 * weight;
  problem.hessian.block<3, 3>(previous_offset, current_offset).noalias() -=
      2.0 * weight;
}

}  // namespace

LtvQpProblem LtvQpBuilder::build(
    const State &current_state, const std::vector<State> &nominal_states,
    const std::vector<Control> &nominal_controls,
    const std::vector<Se2Reference> &references, const Control &last_control,
    const Se2MpcConfig &config, const ZeroSpeedGuardConfig &guard_config) {
  LtvQpProblem problem;
  problem.horizon = config.horizon;
  if (config.horizon <= 0 || config.dt <= 0.0) {
    problem.validation_error = "horizon and dt must be positive";
    return problem;
  }
  const std::size_t horizon = static_cast<std::size_t>(config.horizon);
  if (nominal_states.size() != horizon + 1 ||
      nominal_controls.size() != horizon || references.size() < horizon + 1) {
    problem.validation_error = "nominal/reference horizon size mismatch";
    return problem;
  }
  if (!current_state.allFinite() || !last_control.allFinite() ||
      !finiteStates(nominal_states) || !finiteControls(nominal_controls)) {
    problem.validation_error = "non-finite state or control input";
    return problem;
  }
  if ((config.max_vx < 0.0) || (config.max_vy < 0.0) ||
      (config.max_wz < 0.0) || (config.max_ax < 0.0) ||
      (config.max_ay < 0.0) || (config.max_awz < 0.0)) {
    problem.validation_error = "negative body limit";
    return problem;
  }

  const int decision_size = problem.decisionSize();
  const int equality_rows = 3 * (config.horizon + 1);
  const int inequality_rows = 3 * config.horizon;
  const double infinity = std::numeric_limits<double>::infinity();
  problem.hessian = Eigen::MatrixXd::Zero(decision_size, decision_size);
  problem.gradient = Eigen::VectorXd::Zero(decision_size);
  problem.equality_matrix = Eigen::MatrixXd::Zero(equality_rows, decision_size);
  problem.equality_lower = Eigen::VectorXd::Zero(equality_rows);
  problem.equality_upper = Eigen::VectorXd::Zero(equality_rows);
  problem.inequality_matrix = Eigen::MatrixXd::Zero(inequality_rows, decision_size);
  problem.inequality_lower = Eigen::VectorXd::Zero(inequality_rows);
  problem.inequality_upper = Eigen::VectorXd::Zero(inequality_rows);
  problem.lower_bound = Eigen::VectorXd::Constant(decision_size, -infinity);
  problem.upper_bound = Eigen::VectorXd::Constant(decision_size, infinity);

  const Matrix3 q = config.state_weight.asDiagonal();
  const Matrix3 r = config.control_weight.asDiagonal();
  const Matrix3 rd = config.control_delta_weight.asDiagonal();
  const Matrix3 terminal = config.terminal_weight.asDiagonal();
  Se2Model model(config.dt);

  for (int step = 0; step < config.horizon; ++step) {
    const int state_offset = problem.stateOffset(step);
    const int control_offset = problem.controlOffset(step);
    addQuadraticBlock(
        problem, state_offset, q,
        Se2Model::stateDifference(nominal_states[static_cast<std::size_t>(step)],
                                  references[static_cast<std::size_t>(step)].state));
    addQuadraticBlock(
        problem, control_offset, r,
        nominal_controls[static_cast<std::size_t>(step)] -
            references[static_cast<std::size_t>(step)].control);

    const Control previous_nominal =
        step == 0 ? last_control
                   : nominal_controls[static_cast<std::size_t>(step - 1)];
    const Control nominal_delta =
        nominal_controls[static_cast<std::size_t>(step)] - previous_nominal;
    const int previous_offset =
        step == 0 ? -1 : problem.controlOffset(step - 1);
    addDeltaQuadratic(problem, control_offset, previous_offset, rd,
                      nominal_delta);

    const int inequality_offset = 3 * step;
    problem.inequality_matrix.block<3, 3>(inequality_offset, control_offset)
        .setIdentity();
    if (previous_offset >= 0) {
      problem.inequality_matrix
          .block<3, 3>(inequality_offset, previous_offset)
          .setIdentity();
      problem.inequality_matrix
          .block<3, 3>(inequality_offset, previous_offset) *= -1.0;
    }
    const Control max_delta(config.max_ax * config.dt,
                           config.max_ay * config.dt,
                           config.max_awz * config.dt);
    problem.inequality_lower.segment<3>(inequality_offset) =
        -max_delta - nominal_delta;
    problem.inequality_upper.segment<3>(inequality_offset) =
        max_delta - nominal_delta;

    problem.lower_bound.segment<3>(control_offset) =
        Control(-config.max_vx, -config.max_vy, -config.max_wz) -
        nominal_controls[static_cast<std::size_t>(step)];
    problem.upper_bound.segment<3>(control_offset) =
        Control(config.max_vx, config.max_vy, config.max_wz) -
        nominal_controls[static_cast<std::size_t>(step)];

    const auto nominal_modules = moduleVelocities(
        nominal_controls[static_cast<std::size_t>(step)], config);
    const auto previous_modules = moduleVelocities(previous_nominal, config);
    ZeroSpeedGuard guard(guard_config);
    std::array<double, 4> speeds{};
    for (std::size_t index = 0; index < speeds.size(); ++index) {
      speeds[index] = nominal_modules[index].norm();
      problem.angle_rate_linearization_valid[index] =
          speeds[index] > guard.enterThreshold() &&
          previous_modules[index].norm() > guard.enterThreshold();
    }
    problem.zero_speed_guard_active = problem.zero_speed_guard_active ||
                                      guard.update(speeds);
  }
  addQuadraticBlock(
      problem, problem.stateOffset(config.horizon), terminal,
      Se2Model::stateDifference(nominal_states.back(), references[horizon].state));

  const int initial_offset = problem.stateOffset(0);
  problem.equality_matrix.block<3, 3>(0, initial_offset).setIdentity();
  problem.equality_lower.segment<3>(0) =
      current_state - nominal_states.front();
  problem.equality_upper.segment<3>(0) =
      current_state - nominal_states.front();

  for (int step = 0; step < config.horizon; ++step) {
    const int row = 3 * (step + 1);
    const int current_offset = problem.stateOffset(step);
    const int next_offset = problem.stateOffset(step + 1);
    const int control_offset = problem.controlOffset(step);
    Eigen::Matrix3d a;
    Eigen::Matrix3d b;
    model.jacobians(nominal_states[static_cast<std::size_t>(step)],
                    nominal_controls[static_cast<std::size_t>(step)], a, b);
    problem.equality_matrix.block<3, 3>(row, current_offset) = -a;
    problem.equality_matrix.block<3, 3>(row, control_offset) = -b;
    problem.equality_matrix.block<3, 3>(row, next_offset).setIdentity();
    const State residual =
        model.dynamics(nominal_states[static_cast<std::size_t>(step)],
                       nominal_controls[static_cast<std::size_t>(step)]) -
        nominal_states[static_cast<std::size_t>(step + 1)];
    problem.equality_lower.segment<3>(row) = residual;
    problem.equality_upper.segment<3>(row) = residual;
  }

  problem.valid = problem.hessian.allFinite() && problem.gradient.allFinite() &&
                  problem.equality_matrix.allFinite() &&
                  problem.inequality_matrix.allFinite();
  if (!problem.valid) {
    problem.validation_error = "constructed QP contains non-finite values";
  }
  return problem;
}

}  // namespace ats_swerve_mpc
