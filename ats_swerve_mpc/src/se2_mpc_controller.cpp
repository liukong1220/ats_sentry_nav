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

//清空上一次优化的控制序列
void Se2MpcController::reset()
{
  warm_controls_.clear();
  has_warm_start_ = false;
}

//将任意角度归一到 (-π, π]，避免角度跳变
double Se2MpcController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}


//计算SE(2) 状态（x, y, yaw）的差，角度差值归一化到 (-π, π]
State Se2MpcController::stateDifference(const State & lhs, const State & rhs)
{
  State difference = lhs - rhs;
  difference(2) = normalizeAngle(difference(2));
  return difference;
}

//将 SE(2) 状态（x, y, yaw）与控制（vx, vy, wz）应用于离散时间动力学模型，计算下一状态
State Se2MpcController::dynamics(const State & state, const Control & control) const
{
  // 状态位于世界系 [x, y, yaw]，而 vx/vy 是车体系控制；此处完成 SE2 坐标变换。
  const double yaw = state(2);
  State next = state;
  next(0) += config_.dt * (control(0) * std::cos(yaw) - control(1) * std::sin(yaw));
  next(1) += config_.dt * (control(0) * std::sin(yaw) + control(1) * std::cos(yaw));
  next(2) = normalizeAngle(state(2) + config_.dt * control(2));
  return next;
}

//计算离散时间动力学模型的雅可比矩阵，分别对状态和控制求偏导
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

//将控制输入限制在最大速度和加速度范围内，避免过快或过大控制指令
Control Se2MpcController::clampControl(const Control & control) const
{
  Control clamped = control;
  clamped(0) = std::clamp(clamped(0), -config_.max_vx, config_.max_vx);
  clamped(1) = std::clamp(clamped(1), -config_.max_vy, config_.max_vy);
  clamped(2) = std::clamp(clamped(2), -config_.max_wz, config_.max_wz);
  return clamped;
}

//将控制增量限制在最大加速度范围内，避免每个 dt 内的速度变化过大
Control Se2MpcController::clampIncrement(
  const Control & target,
  const Control & previous) const
{
  const Control max_delta(
    config_.max_ax * config_.dt,
    config_.max_ay * config_.dt,
    config_.max_awz * config_.dt);
  // 同时限制速度幅值和每个 dt 内的增量，近似轮速与舵角执行器的加速度边界。
  Control delta = target - previous;
  for (int axis = 0; axis < 3; ++axis) {
    delta(axis) = std::clamp(delta(axis), -max_delta(axis), max_delta(axis));
  }
  return clampControl(previous + delta);
}

//根据初始状态和控制序列，沿离散时间动力学模型前向滚动计算状态序列
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

//计算给定状态序列、控制序列和参考轨迹的总代价，包括状态误差、控制误差、控制增量误差和终端状态误差
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

//根据当前状态、参考轨迹和上一次控制量，调用 iLQR 算法求解最优控制序列，并返回求解结果，包括控制序列、状态序列、总代价和求解时间
void Se2MpcController::initializeControls(
  const std::vector<Se2Reference> & references,
  const Control & last_control)
{
  if (!has_warm_start_ || static_cast<int>(warm_controls_.size()) != config_.horizon) {
    warm_controls_.assign(static_cast<std::size_t>(config_.horizon), Control::Zero());
  } else {
    // 将上一次控制序列左移一格，减少每个控制周期从零开始求解的时间。
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

//反向传播:根据当前状态、参考轨迹和上一次控制量，调用 iLQR 算法求解最优控制序列，并返回求解结果，包括控制序列、状态序列、总代价和求解时间
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
    // 正则化 Q_uu，避免在低速或权重极端时反向递推出现奇异求解。
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

//主求解:根据当前状态、参考轨迹和上一次控制量，调用 iLQR 算法求解最优控制序列，并返回求解结果，包括控制序列、状态序列、总代价和求解时间
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
    // 线搜索只接受代价下降的候选控制，失败则逐步缩小 iLQR 更新量。
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
