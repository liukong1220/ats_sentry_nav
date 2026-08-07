// Copyright 2026

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ats_swerve_mpc {

/** @brief 用配置初始化 iLQR 状态；SE(2) 模型步长必须与 config.dt 同步。 */
Se2MpcController::Se2MpcController(const Se2MpcConfig &config)
    : config_(config), model_(config.dt) {}

/** @brief 切换控制配置后同步模型步长并清除旧 horizon 的 warm start。 */
void Se2MpcController::setConfig(const Se2MpcConfig &config) {
  config_ = config;
  model_.setTimeStep(config.dt);
  reset();
}

/** @brief 清空上次求解序列，防止急停、重定位或新轨迹复用过期控制。 */
void Se2MpcController::reset() {
  warm_controls_.clear();
  has_warm_start_ = false;
}

/** @brief 委托共享模型归一化 yaw，使 iLQR/QP 的角度语义一致。 */
double Se2MpcController::normalizeAngle(double angle) {
  return Se2Model::normalizeAngle(angle);
}

/** @brief 计算世界系位置差与最短 yaw 差，作为 iLQR 阶段和终端误差。 */
State Se2MpcController::stateDifference(const State &lhs, const State &rhs) {
  return Se2Model::stateDifference(lhs, rhs);
}

/** @brief 以共享 SE(2) 模型推进一拍，禁止在 controller 内维护另一套动力学。 */
State Se2MpcController::dynamics(const State &state,
                                 const Control &control) const {
  return model_.dynamics(state, control);
}

/** @brief 以共享 SE(2) 模型获得反向递推所需的局部 Jacobian。 */
void Se2MpcController::jacobians(const State &state, const Control &control,
                                 Matrix3 &state_jacobian,
                                 Matrix3 &control_jacobian) const {
  model_.jacobians(state, control, state_jacobian, control_jacobian);
}

/**
 * @brief 四舵轮模块级约束是否完全生效。
 * @return true=半轴偏置与单轮速度上限均已配置；false=配置缺失，仅车体级限幅可用。
 * @note 由控制节点在参数加载后与运行期调用，用于输出实车高危配置告警。
 */
bool Se2MpcController::moduleLimitsActive() const {
  return config_.wheel_base_x > 0.0 && config_.wheel_base_y > 0.0 &&
         config_.max_wheel_speed > 1e-6;
}

/**
 * @brief 将车体控制量限制在速度与单轮速度边界内。
 * @param control 待限幅的车体系控制 [vx, vy, wz]。
 * @param report  可选输出：记录本次是否触及车体级/轮级上限，供饱和日志使用。
 * @return 限幅后的控制量。
 * @note 每个控制周期在 clampIncrement 与 solve 内被反复调用。
 *       轮级限幅采用整体等比缩放，保证四轮速度向量仍来自同一车体 Twist，
 *       否则会破坏舵轮的瞬心一致性（实车表现为轮胎互相拖拽、打滑）。
 */
Control Se2MpcController::clampControl(const Control &control,
                                       SaturationReport *report) const {
  Control clamped = control;
  clamped(0) = std::clamp(clamped(0), -config_.max_vx, config_.max_vx);
  clamped(1) = std::clamp(clamped(1), -config_.max_vy, config_.max_vy);
  clamped(2) = std::clamp(clamped(2), -config_.max_wz, config_.max_wz);
  if (report != nullptr &&
      ((clamped - control).lpNorm<Eigen::Infinity>() > 1e-9)) {
    report->body = true;
  }
  const double module_speed = maxModuleSpeed(clamped);
  if (config_.max_wheel_speed > 1e-6 &&
      module_speed > config_.max_wheel_speed) {
    clamped *= config_.max_wheel_speed / module_speed;
    if (report != nullptr) {
      report->wheel = true;
    }
  }
  return clamped;
}

/**
 * @brief 计算给定车体控制下四个舵轮模块中的最大轮速。
 * @param control 车体系控制 [vx, vy, wz]。
 * @return 四轮线速度模长的最大值 [m/s]。
 * @note 数学关系：v_i = [vx - wz*y_i, vy + wz*x_i]，其中 (x_i, y_i) 为轮心相对
 *       底盘中心的位置。半轴偏置未配置（<=0）时按 0 处理，四轮退化为车体平移速度，
 *       此时函数返回 hypot(vx, vy)，仍能让单轮速度上限约束住平移分量。
 *       此前实现直接返回 0.0，会让 clampControl 的轮速判据永远不成立，
 *       等于把执行器边界整体关闭——属于"仿真通过实车超速"的典型来源。
 */
double Se2MpcController::maxModuleSpeed(const Control &control) const {
  double maximum = 0.0;
  for (const auto &velocity : moduleVelocities(control)) {
    maximum = std::max(maximum, velocity.norm());
  }
  return maximum;
}

/**
 * @brief 按底盘几何把 [vx,vy,wz] 映射为四个轮心速度向量。
 * @details 每个轮速为 [vx-wz*y_i, vy+wz*x_i]；该向量是轮速、增量和舵角速率
 *          约束的共同物理依据，禁止把四轮独立改写成不一致的 Twist。
 */
std::array<Eigen::Vector2d, 4>
Se2MpcController::moduleVelocities(const Control &control) const {
  const double x = std::max(0.0, config_.wheel_base_x);
  const double y = std::max(0.0, config_.wheel_base_y);
  const std::array<Eigen::Vector2d, 4> positions{{
      Eigen::Vector2d(x, y),
      Eigen::Vector2d(x, -y),
      Eigen::Vector2d(-x, y),
      Eigen::Vector2d(-x, -y),
  }};
  std::array<Eigen::Vector2d, 4> velocities;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    velocities[index] << control(0) - control(2) * positions[index](1),
        control(1) + control(2) * positions[index](0);
  }
  return velocities;
}

/**
 * @brief 判断候选控制相对上一控制是否满足四舵轮执行器增量约束。
 * @param candidate 候选车体控制 [vx, vy, wz]。
 * @param previous  上一周期实际下发的车体控制。
 * @return true=四个模块的轮速、轮加速度、舵角速率都在一个 dt 内可达。
 * @note 由 clampIncrement 的二分线搜索反复调用。
 *       舵角变化量取两次轮速向量的真实夹角
 *       \f$ \Delta\theta = \arccos\frac{v_k \cdot v_{k-1}}{\|v_k\|\|v_{k-1}\|} \f$，
 *       不能对点积取绝对值：取绝对值会把 180° 反向翻转判成 0° 变化，
 *       实车表现为舵轮被要求瞬间掉头，转向电机堵转或底盘被硬拽。
 */
bool Se2MpcController::moduleIncrementFeasible(
    const Control &candidate, const Control &previous) const {
  const auto candidate_velocities = moduleVelocities(candidate);
  const auto previous_velocities = moduleVelocities(previous);
  const double max_speed_delta = config_.max_wheel_acceleration * config_.dt;
  const double max_steer_delta = config_.max_steer_rate * config_.dt;
  for (std::size_t index = 0; index < candidate_velocities.size(); ++index) {
    const double candidate_speed = candidate_velocities[index].norm();
    const double previous_speed = previous_velocities[index].norm();
    if (config_.max_wheel_speed > 1e-6 &&
        candidate_speed > config_.max_wheel_speed + 1e-9) {
      return false;
    }
    if (config_.max_wheel_acceleration > 1e-6 &&
        (candidate_velocities[index] - previous_velocities[index]).norm() >
            max_speed_delta + 1e-9) {
      return false;
    }
    if (config_.max_steer_rate > 1e-6 && candidate_speed > 1e-4 &&
        previous_speed > 1e-4) {
      // 取带符号点积并归一到 [-1, 1]，使反向翻转得到 pi 的真实舵角变化量。
      const double cosine = std::clamp(
          candidate_velocities[index].dot(previous_velocities[index]) /
              (candidate_speed * previous_speed),
          -1.0, 1.0);
      if (std::acos(cosine) > max_steer_delta + 1e-9) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 将控制增量限制在车体加速度与四舵轮执行器可达范围内。
 * @param target            期望的车体控制 [vx, vy, wz]。
 * @param previous          上一周期实际下发的车体控制。
 * @param report            可选输出：车体级/轮级速度限幅是否命中。
 * @param increment_limited 可选输出：是否因模块增量不可达而被线搜索回退。
 * @return 一个 dt 内实车可达的车体控制。
 * @note 每步预测与每次线搜索都会调用。先按车体加速度上限裁剪 Δv，再做速度限幅；
 *       若模块级增量仍不可达，则在 previous→limited 线段上做 40 次二分，
 *       取最大可行比例，保证四轮速度向量始终来自同一车体 Twist。
 */
Control Se2MpcController::clampIncrement(const Control &target,
                                         const Control &previous,
                                         SaturationReport *report,
                                         bool *increment_limited) const {
  const Control max_delta(config_.max_ax * config_.dt,
                          config_.max_ay * config_.dt,
                          config_.max_awz * config_.dt);
  // 同时限制速度幅值和每个 dt 内的增量，近似轮速与舵角执行器的加速度边界。
  Control delta = target - previous;
  for (int axis = 0; axis < 3; ++axis) {
    delta(axis) = std::clamp(delta(axis), -max_delta(axis), max_delta(axis));
  }
  const Control limited = clampControl(previous + delta, report);
  if (moduleIncrementFeasible(limited, previous)) {
    return limited;
  }
  if (increment_limited != nullptr) {
    *increment_limited = true;
  }

  // 沿上一控制到候选控制做保守线搜索，保持四轮速度向量来自同一车体 Twist。
  double feasible_scale = 0.0;
  double infeasible_scale = 1.0;
  for (int iteration = 0; iteration < 40; ++iteration) {
    const double scale = 0.5 * (feasible_scale + infeasible_scale);
    const Control candidate = previous + scale * (limited - previous);
    if (moduleIncrementFeasible(candidate, previous)) {
      feasible_scale = scale;
    } else {
      infeasible_scale = scale;
    }
  }
  return clampControl(previous + feasible_scale * (limited - previous), report);
}

/** @brief 生成 iLQR 名义/候选状态序列，结果长度恒为控制数加一。 */
std::vector<State>
Se2MpcController::rollout(const State &initial,
                          const std::vector<Control> &controls) const {
  return model_.rollout(initial, controls);
}

/**
 * @brief 评估一个完整控制序列的二次 iLQR 目标。
 * @details 包含阶段状态跟踪、控制参考、相邻控制增量及 terminal 误差；输入长度不一致
 *          时返回 infinity，保证线搜索不会接纳结构错误的候选。
 */
double Se2MpcController::cost(const std::vector<State> &states,
                              const std::vector<Control> &controls,
                              const std::vector<Se2Reference> &references,
                              const Control &last_control) const {
  if (states.size() != controls.size() + 1 ||
      references.size() < states.size()) {
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
  const State final_error =
      stateDifference(states.back(), references[controls.size()].state);
  return total + final_error.dot(terminal * final_error);
}

/**
 * @brief 构造或移动 iLQR warm start，并逐步投影到真实四轮执行器可达集合。
 * @details 首步单独记录饱和/增量回退，因为只有它会进入 `/cmd_vel_mpc`。
 */
void Se2MpcController::initializeControls(
    const std::vector<Se2Reference> &references, const Control &last_control) {
  if (!has_warm_start_ ||
      static_cast<int>(warm_controls_.size()) != config_.horizon) {
    warm_controls_.assign(static_cast<std::size_t>(config_.horizon),
                          Control::Zero());
  } else {
    // 将上一次控制序列左移一格，减少每个控制周期从零开始求解的时间。
    std::rotate(warm_controls_.begin(), warm_controls_.begin() + 1,
                warm_controls_.end());
  }
  Control previous = last_control;
  for (int step = 0; step < config_.horizon; ++step) {
    const Control seed =
        has_warm_start_
            ? 0.7 * warm_controls_[static_cast<std::size_t>(step)] +
                  0.3 * references[static_cast<std::size_t>(step)].control
            : references[static_cast<std::size_t>(step)].control;
    // 仅第 0 步是本周期真正下发的控制，记录其限幅命中情况用于饱和诊断。
    SaturationReport *report = step == 0 ? &first_step_saturation_ : nullptr;
    bool *increment_limited =
        step == 0 ? &first_step_increment_limited_ : nullptr;
    warm_controls_[static_cast<std::size_t>(step)] =
        clampIncrement(seed, previous, report, increment_limited);
    previous = warm_controls_[static_cast<std::size_t>(step)];
  }
  has_warm_start_ = true;
}

/**
 * @brief 从 terminal cost 向前执行 iLQR 反向递推。
 * @details 对 Q_uu 做正则化并要求 LDLT、前馈和反馈均有限；任一失败均拒绝把旧
 *          warm-start 当成当前闭环解。
 */
Se2MpcController::BackwardResult
Se2MpcController::backwardPass(const std::vector<State> &states,
                               const std::vector<Control> &controls,
                               const std::vector<Se2Reference> &references,
                               const Control &last_control) const {
  BackwardResult result;
  if (states.size() != controls.size() + 1 ||
      references.size() < states.size()) {
    return result;
  }
  result.feedforward.assign(controls.size(), Control::Zero());
  result.feedback.assign(controls.size(), Matrix3::Zero());
  const Matrix3 q = config_.state_weight.asDiagonal();
  const Matrix3 r = config_.control_weight.asDiagonal();
  const Matrix3 rd = config_.control_delta_weight.asDiagonal();
  const Matrix3 terminal = config_.terminal_weight.asDiagonal();
  State value_gradient =
      2.0 * terminal *
      stateDifference(states.back(), references[controls.size()].state);
  Matrix3 value_hessian = 2.0 * terminal;

  for (int step = static_cast<int>(controls.size()) - 1; step >= 0; --step) {
    const std::size_t index = static_cast<std::size_t>(step);
    Matrix3 a;
    Matrix3 b;
    jacobians(states[index], controls[index], a, b);
    const State state_error =
        stateDifference(states[index], references[index].state);
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
                     feedback.transpose() * q_u +
                     q_ux.transpose() * feedforward;
    value_hessian = q_xx + feedback.transpose() * q_uu * feedback +
                    feedback.transpose() * q_ux + q_ux.transpose() * feedback;
    value_hessian = 0.5 * (value_hessian + value_hessian.transpose());
  }
  result.success = true;
  return result;
}

/**
 * @brief iLQR 主求解入口。
 * @details 先建立物理可达 warm start，再交替进行 rollout、backward pass 与受约束线搜索。
 *          只有成功完成至少一次数值有效的反向递推后才返回 success，失败交由 node
 *          发布确定性零速度；该函数不创建第二个速度发布者。
 */
Se2MpcResult
Se2MpcController::solve(const State &current_state,
                        const std::vector<Se2Reference> &references,
                        const Control &last_control) {
  Se2MpcResult result;
  const auto start_time = std::chrono::steady_clock::now();
  if (config_.horizon <= 0 || config_.dt <= 0.0 ||
      references.size() < static_cast<std::size_t>(config_.horizon + 1)) {
    return result;
  }
  first_step_saturation_ = SaturationReport{};
  first_step_increment_limited_ = false;
  initializeControls(references, last_control);
  std::vector<Control> controls = warm_controls_;
  std::vector<State> states = rollout(current_state, controls);
  double current_cost = cost(states, controls, references, last_control);

  // 解的数值可信凭据：至少完成一次成功的反向递推（Q_uu 可分解、增益有限）。
  // 不能把"有迭代被线搜索接受"当作成功的必要条件——在速度/轮速/舵角速率约束
  // 激活时，限幅会把所有候选压回当前序列，代价自然无法下降，但当前序列仍是
  // 约束集内可执行的最优解。此前只校验序列长度的实现则相反：反向递推奇异
  // （Q_uu 不可分解或增益出现 NaN）时仍判成功，等于把上一周期的暖启动序列
  // 当作本周期解持续下发，实车表现为"MPC 看似正常但控制已失去反馈"。
  bool solution_certified = false;
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const BackwardResult backward =
        backwardPass(states, controls, references, last_control);
    if (!backward.success) {
      break;
    }
    solution_certified = true;
    double max_update = 0.0;
    for (const auto &update : backward.feedforward) {
      max_update = std::max(max_update, update.lpNorm<Eigen::Infinity>());
    }
    bool accepted = false;
    // 线搜索只接受代价下降的候选控制，失败则逐步缩小 iLQR 更新量。
    for (double alpha = 1.0; alpha >= config_.min_line_search_step;
         alpha *= config_.line_search_decay) {
      std::vector<Control> candidate_controls(
          static_cast<std::size_t>(config_.horizon), Control::Zero());
      std::vector<State> candidate_states;
      candidate_states.reserve(static_cast<std::size_t>(config_.horizon + 1));
      candidate_states.push_back(current_state);
      Control previous = last_control;
      SaturationReport candidate_saturation;
      bool candidate_increment_limited = false;
      for (int step = 0; step < config_.horizon; ++step) {
        const std::size_t index = static_cast<std::size_t>(step);
        const State error =
            stateDifference(candidate_states.back(), states[index]);
        const Control update = alpha * backward.feedforward[index] +
                               backward.feedback[index] * error;
        // 只有第 0 步会被真正下发，其限幅命中情况才有实车诊断意义。
        candidate_controls[index] = clampIncrement(
            controls[index] + update, previous,
            step == 0 ? &candidate_saturation : nullptr,
            step == 0 ? &candidate_increment_limited : nullptr);
        candidate_states.push_back(
            dynamics(candidate_states.back(), candidate_controls[index]));
        previous = candidate_controls[index];
      }
      const double candidate_cost =
          cost(candidate_states, candidate_controls, references, last_control);
      if (candidate_cost + 1e-9 < current_cost) {
        controls = std::move(candidate_controls);
        states = std::move(candidate_states);
        current_cost = candidate_cost;
        // 该候选被采纳，其首步限幅记录即本周期下发控制的饱和状态。
        first_step_saturation_ = candidate_saturation;
        first_step_increment_limited_ = candidate_increment_limited;
        accepted = true;
        break;
      }
    }
    result.iterations = iteration + 1;
    if (accepted) {
      ++result.accepted_iterations;
    }
    if (max_update < config_.convergence_tolerance) {
      // 已处于驻点：暖启动序列本身就是当前时域的最优解。
      break;
    }
    if (!accepted) {
      break;
    }
  }

  // 首步饱和标志来自前向生成过程（暖启动或被采纳的线搜索候选）；
  // 不能在此处对 controls.front() 再限幅一次取标志——它已经是限幅后的值，
  // 再限幅恒为恒等映射，命中标志永远为 false。
  result.body_limit_saturated = first_step_saturation_.body;
  result.wheel_limit_saturated = first_step_saturation_.wheel;
  result.increment_limited = first_step_increment_limited_;
  result.module_limits_active = moduleLimitsActive();

  warm_controls_ = controls;
  result.controls = std::move(controls);
  result.states = std::move(states);
  result.cost = current_cost;
  // 求解成功必须同时满足：序列长度合法、代价有限、且至少完成一次成功反向递推。
  // 只判断长度会把"反向递推奇异后原样返回暖启动序列"误判为成功。
  result.success =
      result.controls.size() == static_cast<std::size_t>(config_.horizon) &&
      result.states.size() == static_cast<std::size_t>(config_.horizon + 1) &&
      std::isfinite(result.cost) && solution_certified;
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start_time)
                             .count();
  return result;
}

} // namespace ats_swerve_mpc
