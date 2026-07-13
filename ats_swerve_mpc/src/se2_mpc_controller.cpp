// Copyright 2026

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ats_swerve_mpc
{

Se2MpcController::Se2MpcController(const Se2MpcConfig & config)
: config_(config)
{
}

void Se2MpcController::setConfig(const Se2MpcConfig & config)
{
  config_ = config;
  reset();
}

void Se2MpcController::reset()
{
  warm_controls_.clear();
  has_warm_start_ = false;
}

double Se2MpcController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

State Se2MpcController::stateDifference(const State & lhs, const State & rhs)
{
  State difference = lhs - rhs;
  difference(2) = normalizeAngle(difference(2));
  return difference;
}

State Se2MpcController::dynamics(const State & state, const Control & control) const
{
  const double yaw = state(2);
  State next = state;
  next(0) += config_.dt * (control(0) * std::cos(yaw) - control(1) * std::sin(yaw));
  next(1) += config_.dt * (control(0) * std::sin(yaw) + control(1) * std::cos(yaw));
  next(2) = normalizeAngle(state(2) + config_.dt * control(2));
  return next;
}

void Se2MpcController::jacobians(
  const State & state,
  const Control & control,
  Matrix3 & state_jacobian,
  Matrix3 & control_jacobian) const
{
  const double yaw = state(2);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  state_jacobian.setIdentity();
  state_jacobian(0, 2) = config_.dt * (-control(0) * sine - control(1) * cosine);
  state_jacobian(1, 2) = config_.dt * (control(0) * cosine - control(1) * sine);
  control_jacobian.setZero();
  control_jacobian(0, 0) = config_.dt * cosine;
  control_jacobian(0, 1) = -config_.dt * sine;
  control_jacobian(1, 0) = config_.dt * sine;
  control_jacobian(1, 1) = config_.dt * cosine;
  control_jacobian(2, 2) = config_.dt;
}

Control Se2MpcController::clampControl(const Control & control) const
{
  Control clamped = control;
  clamped(0) = std::clamp(clamped(0), -config_.max_vx, config_.max_vx);
  clamped(1) = std::clamp(clamped(1), -config_.max_vy, config_.max_vy);
  clamped(2) = std::clamp(clamped(2), -config_.max_wz, config_.max_wz);
  return clamped;
}

Control Se2MpcController::clampIncrement(
  const Control & target,
  const Control & previous) const
{
  const Control max_delta(
    config_.max_ax * config_.dt,
    config_.max_ay * config_.dt,
    config_.max_awz * config_.dt);
  Control delta = target - previous;
  for (int axis = 0; axis < 3; ++axis) {
    delta(axis) = std::clamp(delta(axis), -max_delta(axis), max_delta(axis));
  }
  return clampControl(previous + delta);
}

std::vector<State> Se2MpcController::rollout(
  const State & initial,
  const std::vector<Control> & controls) const
{
  std::vector<State> states;
  states.reserve(controls.size() + 1);
  states.push_back(initial);
  for (const auto & control : controls) {
    states.push_back(dynamics(states.back(), control));
  }
  return states;
}

double Se2MpcController::cost(
  const std::vector<State> & states,
  const std::vector<Control> & controls,
  const std::vector<Se2Reference> & references,
  const Control & last_control) const
{
  if (states.size() != controls.size() + 1 || references.size() < states.size()) {
    return std::numeric_limits<double>::infinity();
  }
  const Matrix3 q = config_.state_weight.asDiagonal();
  const Matrix3 r = config_.control_weight.asDiagonal();
  const Matrix3 rd = config_.control_delta_weight.asDiagonal();
  const Matrix3 terminal = config_.terminal_weight.asDiagonal();
  double total = 0.0;
  Control previous = last_control;
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const State state_error = stateDifference(states[i], references[i].state);
    const Control control_error = controls[i] - references[i].control;
    const Control delta = controls[i] - previous;
    total += state_error.dot(q * state_error);
    total += control_error.dot(r * control_error);
    total += delta.dot(rd * delta);
    previous = controls[i];
  }
  const State final_error = stateDifference(states.back(), references[controls.size()].state);
  return total + final_error.dot(terminal * final_error);
}

void Se2MpcController::initializeControls(
  const std::vector<Se2Reference> & references,
  const Control & last_control)
{
  if (!has_warm_start_ || static_cast<int>(warm_controls_.size()) != config_.horizon) {
    warm_controls_.assign(static_cast<std::size_t>(config_.horizon), Control::Zero());
  } else {
    std::rotate(warm_controls_.begin(), warm_controls_.begin() + 1, warm_controls_.end());
  }
  Control previous = last_control;
  for (int step = 0; step < config_.horizon; ++step) {
    const Control seed = has_warm_start_ ?
      0.7 * warm_controls_[static_cast<std::size_t>(step)] +
      0.3 * references[static_cast<std::size_t>(step)].control :
      references[static_cast<std::size_t>(step)].control;
    warm_controls_[static_cast<std::size_t>(step)] = clampIncrement(seed, previous);
    previous = warm_controls_[static_cast<std::size_t>(step)];
  }
  has_warm_start_ = true;
}

Se2MpcController::BackwardResult Se2MpcController::backwardPass(
  const std::vector<State> & states,
  const std::vector<Control> & controls,
  const std::vector<Se2Reference> & references,
  const Control & last_control) const
{
  BackwardResult result;
  if (states.size() != controls.size() + 1 || references.size() < states.size()) {
    return result;
  }
  result.feedforward.assign(controls.size(), Control::Zero());
  result.feedback.assign(controls.size(), Matrix3::Zero());
  const Matrix3 q = config_.state_weight.asDiagonal();
  const Matrix3 r = config_.control_weight.asDiagonal();
  const Matrix3 rd = config_.control_delta_weight.asDiagonal();
  const Matrix3 terminal = config_.terminal_weight.asDiagonal();
  State value_gradient =
    2.0 * terminal * stateDifference(states.back(), references[controls.size()].state);
  Matrix3 value_hessian = 2.0 * terminal;

  for (int step = static_cast<int>(controls.size()) - 1; step >= 0; --step) {
    const std::size_t index = static_cast<std::size_t>(step);
    Matrix3 a;
    Matrix3 b;
    jacobians(states[index], controls[index], a, b);
    const State state_error = stateDifference(states[index], references[index].state);
    const Control control_error = controls[index] - references[index].control;
    const Control previous = step == 0 ? last_control : controls[index - 1];
    const Control control_delta = controls[index] - previous;
    const State stage_x = 2.0 * q * state_error;
    const Control stage_u = 2.0 * r * control_error + 2.0 * rd * control_delta;
    const State q_x = stage_x + a.transpose() * value_gradient;
    const Control q_u = stage_u + b.transpose() * value_gradient;
    const Matrix3 q_xx = 2.0 * q + a.transpose() * value_hessian * a;
    const Matrix3 q_ux = b.transpose() * value_hessian * a;
    Matrix3 q_uu = 2.0 * (r + rd) + b.transpose() * value_hessian * b;
    q_uu += config_.regularization * Matrix3::Identity();
    Eigen::LDLT<Matrix3> factor(q_uu);
    if (factor.info() != Eigen::Success) {
      return result;
    }
    const Control feedforward = -factor.solve(q_u);
    const Matrix3 feedback = -factor.solve(q_ux);
    if (!feedforward.allFinite() || !feedback.allFinite()) {
      return result;
    }
    result.feedforward[index] = feedforward;
    result.feedback[index] = feedback;
    value_gradient = q_x + feedback.transpose() * q_uu * feedforward +
      feedback.transpose() * q_u + q_ux.transpose() * feedforward;
    value_hessian = q_xx + feedback.transpose() * q_uu * feedback +
      feedback.transpose() * q_ux + q_ux.transpose() * feedback;
    value_hessian = 0.5 * (value_hessian + value_hessian.transpose());
  }
  result.success = true;
  return result;
}

Se2MpcResult Se2MpcController::solve(
  const State & current_state,
  const std::vector<Se2Reference> & references,
  const Control & last_control)
{
  Se2MpcResult result;
  const auto start_time = std::chrono::steady_clock::now();
  if (config_.horizon <= 0 || config_.dt <= 0.0 ||
    references.size() < static_cast<std::size_t>(config_.horizon + 1))
  {
    return result;
  }
  initializeControls(references, last_control);
  std::vector<Control> controls = warm_controls_;
  std::vector<State> states = rollout(current_state, controls);
  double current_cost = cost(states, controls, references, last_control);

  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const BackwardResult backward = backwardPass(states, controls, references, last_control);
    if (!backward.success) {
      break;
    }
    double max_update = 0.0;
    for (const auto & update : backward.feedforward) {
      max_update = std::max(max_update, update.lpNorm<Eigen::Infinity>());
    }
    bool accepted = false;
    for (double alpha = 1.0; alpha >= config_.min_line_search_step;
      alpha *= config_.line_search_decay)
    {
      std::vector<Control> candidate_controls(
        static_cast<std::size_t>(config_.horizon), Control::Zero());
      std::vector<State> candidate_states;
      candidate_states.reserve(static_cast<std::size_t>(config_.horizon + 1));
      candidate_states.push_back(current_state);
      Control previous = last_control;
      for (int step = 0; step < config_.horizon; ++step) {
        const std::size_t index = static_cast<std::size_t>(step);
        const State error = stateDifference(candidate_states.back(), states[index]);
        const Control update =
          alpha * backward.feedforward[index] + backward.feedback[index] * error;
        candidate_controls[index] = clampIncrement(controls[index] + update, previous);
        candidate_states.push_back(dynamics(candidate_states.back(), candidate_controls[index]));
        previous = candidate_controls[index];
      }
      const double candidate_cost =
        cost(candidate_states, candidate_controls, references, last_control);
      if (candidate_cost + 1e-9 < current_cost) {
        controls = std::move(candidate_controls);
        states = std::move(candidate_states);
        current_cost = candidate_cost;
        accepted = true;
        break;
      }
    }
    result.iterations = iteration + 1;
    if (!accepted || max_update < config_.convergence_tolerance) {
      break;
    }
  }

  warm_controls_ = controls;
  result.controls = std::move(controls);
  result.states = std::move(states);
  result.cost = current_cost;
  result.success = result.controls.size() == static_cast<std::size_t>(config_.horizon) &&
    result.states.size() == static_cast<std::size_t>(config_.horizon + 1) &&
    std::isfinite(result.cost);
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();
  return result;
}

}  // namespace ats_swerve_mpc
