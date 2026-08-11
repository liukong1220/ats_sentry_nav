// Copyright 2026

#include "ats_swerve_mpc/qp/ltv_qp_problem.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace ats_swerve_mpc {
namespace {

using Matrix3 = Eigen::Matrix3d;

/** @brief checked size_t 加法，防止 dimension 计算在中间步骤回绕。 */
bool checkedAdd(std::size_t left, std::size_t right, std::size_t &result) {
  if (right > std::numeric_limits<std::size_t>::max() - left) {
    return false;
  }
  result = left + right;
  return true;
}

/** @brief checked size_t 乘法，防止矩阵元素数量计算在中间步骤回绕。 */
bool checkedMultiply(std::size_t left, std::size_t right,
                     std::size_t &result) {
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

/** @brief 将 checked size_t 安全收窄到 Eigen/OSQP 使用的 int 维度。 */
bool fitsInt(std::size_t value) {
  return value <= static_cast<std::size_t>(std::numeric_limits<int>::max());
}

/** @brief 允许 OSQP 的无穷边界，但拒绝 NaN 传播到后端。 */
bool finiteOrInfinity(double value) {
  return !std::isnan(value);
}

/** @brief 校验边界向量中的每个元素；仅变量 bounds 可以合法使用无穷。 */
bool vectorFiniteOrInfinity(const Eigen::VectorXd &values) {
  for (int index = 0; index < values.size(); ++index) {
    if (!finiteOrInfinity(values(index))) {
      return false;
    }
  }
  return true;
}

/** @brief 权重必须有限且非负，确保送入 OSQP 的 Hessian 保持凸性。 */
bool validWeight(const Eigen::Vector3d &weight) {
  return weight.allFinite() && (weight.array() >= 0.0).all();
}

/** @brief 动力学上限允许为零以表达禁用轴，但不得为 NaN/Inf 或负数。 */
bool validNonnegativeLimit(double value) {
  return std::isfinite(value) && value >= 0.0;
}

/** @brief 在同维向量上逐项检查双边约束，NaN 比较会自然 fail-closed。 */
bool boundsOrdered(const Eigen::VectorXd &lower, const Eigen::VectorXd &upper) {
  return lower.size() == upper.size() &&
         (lower.array() <= upper.array()).all();
}

/** @brief 检查名义状态序列是否全部有限，防止 NaN 进入矩阵线性化。 */
bool finiteStates(const std::vector<State> &states) {
  for (const auto &state : states) {
    if (!state.allFinite()) {
      return false;
    }
  }
  return true;
}

/** @brief 检查名义控制序列是否全部有限，防止无效 Twist 污染 QP。 */
bool finiteControls(const std::vector<Control> &controls) {
  for (const auto &control : controls) {
    if (!control.allFinite()) {
      return false;
    }
  }
  return true;
}

/** @brief 用与 iLQR 相同的四轮几何计算名义轮速向量，仅用于低速方向可定义性。 */
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

/** @brief 向目标的 Hessian/gradient 加入一个三维二次误差块。 */
void addQuadraticBlock(LtvQpProblem &problem, int offset, const Matrix3 &weight,
                       const Eigen::Vector3d &error) {
  problem.hessian.block<3, 3>(offset, offset).noalias() += 2.0 * weight;
  problem.gradient.segment<3>(offset).noalias() += 2.0 * weight * error;
}

/** @brief 写入相邻控制偏差的二次惩罚，保留固定 banded Hessian 结构。 */
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

LtvQpDimensions checkedLtvQpDimensions(int horizon) {
  LtvQpDimensions dimensions;
  if (horizon <= 0) {
    dimensions.validation_error = "horizon must be positive";
    return dimensions;
  }
  if (horizon > kLtvQpMaximumHorizon) {
    dimensions.validation_error = "horizon exceeds audited LTV-QP maximum";
    return dimensions;
  }

  const std::size_t h = static_cast<std::size_t>(horizon);
  std::size_t h_plus_one = 0;
  std::size_t decision = 0;
  std::size_t equality_rows = 0;
  std::size_t inequality_rows = 0;
  if (!checkedAdd(h, 1U, h_plus_one) ||
      !checkedMultiply(h_plus_one, 3U, equality_rows) ||
      !checkedMultiply(h, 3U, inequality_rows) ||
      !checkedMultiply(h, 6U, decision) ||
      !checkedAdd(decision, 3U, decision) || !fitsInt(decision) ||
      !fitsInt(equality_rows) || !fitsInt(inequality_rows)) {
    dimensions.validation_error = "LTV-QP dimensions do not fit int";
    return dimensions;
  }

  std::size_t coefficient_elements = 0;
  std::size_t hessian_elements = 0;
  std::size_t equality_elements = 0;
  std::size_t inequality_elements = 0;
  std::size_t vector_elements = 0;
  std::size_t dense_elements = 0;
  std::size_t constraint_coefficient_rows = 0;
  std::size_t constraint_rows = 0;
  if (!checkedMultiply(decision, decision, hessian_elements) ||
      !checkedMultiply(equality_rows, decision, equality_elements) ||
      !checkedMultiply(inequality_rows, decision, inequality_elements) ||
      !checkedAdd(equality_elements, inequality_elements,
                  coefficient_elements) ||
      !checkedAdd(hessian_elements, coefficient_elements, dense_elements) ||
      !checkedMultiply(decision, 3U, vector_elements) ||
      !checkedAdd(dense_elements, vector_elements, dense_elements) ||
      !checkedAdd(equality_rows, inequality_rows,
                  constraint_coefficient_rows) ||
      !checkedMultiply(constraint_coefficient_rows, 2U, vector_elements) ||
      !checkedAdd(dense_elements, vector_elements, dense_elements) ||
      !checkedMultiply(dense_elements, sizeof(double),
                       dimensions.dense_buffer_bytes) ||
      dimensions.dense_buffer_bytes > kLtvQpDenseBufferMaximumBytes ||
      !checkedAdd(constraint_coefficient_rows, decision, constraint_rows)) {
    dimensions.validation_error = "LTV-QP dense buffer exceeds audited resource cap";
    return dimensions;
  }

  if (!fitsInt(constraint_rows)) {
    dimensions.validation_error = "LTV-QP constraint rows do not fit int";
    return dimensions;
  }
  dimensions.valid = true;
  dimensions.horizon = horizon;
  dimensions.decision_size = static_cast<int>(decision);
  dimensions.equality_rows = static_cast<int>(equality_rows);
  dimensions.inequality_rows = static_cast<int>(inequality_rows);
  dimensions.constraint_rows = static_cast<int>(constraint_rows);
  dimensions.validation_error = "valid";
  return dimensions;
}

/** @brief 验证预分配 dense QP 缓冲的固定 LTV 维度，防止后端访问错位。 */
bool LtvQpProblem::hasExpectedLayout() const {
  const auto dimensions = checkedLtvQpDimensions(horizon);
  if (!dimensions.valid || state_dimension != 3 || control_dimension != 3) {
    return false;
  }
  return hessian.rows() == dimensions.decision_size &&
         hessian.cols() == dimensions.decision_size &&
         gradient.size() == dimensions.decision_size &&
         equality_matrix.rows() == dimensions.equality_rows &&
         equality_matrix.cols() == dimensions.decision_size &&
         equality_lower.size() == dimensions.equality_rows &&
         equality_upper.size() == dimensions.equality_rows &&
         inequality_matrix.rows() == dimensions.inequality_rows &&
         inequality_matrix.cols() == dimensions.decision_size &&
         inequality_lower.size() == dimensions.inequality_rows &&
         inequality_upper.size() == dimensions.inequality_rows &&
         lower_bound.size() == dimensions.decision_size &&
         upper_bound.size() == dimensions.decision_size;
}

/** @brief 验证 LTV 等式、不等式和变量 bounds 的逐项下界/上界关系。 */
bool LtvQpProblem::hasOrderedBounds() const {
  return hasExpectedLayout() &&
         boundsOrdered(equality_lower, equality_upper) &&
         boundsOrdered(inequality_lower, inequality_upper) &&
         boundsOrdered(lower_bound, upper_bound);
}

bool LtvQpProblem::hasFiniteNumerics() const {
  return hasExpectedLayout() && hessian.allFinite() && gradient.allFinite() &&
         equality_matrix.allFinite() && equality_lower.allFinite() &&
         equality_upper.allFinite() && inequality_matrix.allFinite() &&
         inequality_lower.allFinite() && inequality_upper.allFinite() &&
         vectorFiniteOrInfinity(lower_bound) &&
         vectorFiniteOrInfinity(upper_bound);
}

/**
 * @brief 在控制 timer 外分配指定 horizon 的 dense LTV-QP 数值缓冲。
 * @details 固定的 decision/row 数和矩阵尺寸是后续 OSQP CSC pattern 一次 setup 的前提。
 */
LtvQpProblem LtvQpBuilder::allocate(int horizon) {
  LtvQpProblem problem;
  problem.horizon = horizon;
  const auto dimensions = checkedLtvQpDimensions(horizon);
  if (!dimensions.valid) {
    problem.validation_error = dimensions.validation_error;
    return problem;
  }
  const int decision_size = dimensions.decision_size;
  const int equality_rows = dimensions.equality_rows;
  const int inequality_rows = dimensions.inequality_rows;
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
  return problem;
}

/** @brief 便捷构造一次性 LTV 问题；shadow runtime 使用其预分配 overload。 */
LtvQpProblem LtvQpBuilder::build(
    const State &current_state, const std::vector<State> &nominal_states,
    const std::vector<Control> &nominal_controls,
    const std::vector<Se2Reference> &references, const Control &last_control,
    const Se2MpcConfig &config, const ZeroSpeedGuardConfig &guard_config) {
  LtvQpProblem problem = allocate(config.horizon);
  build(current_state, nominal_states, nominal_controls, references, last_control,
        config, problem, guard_config);
  return problem;
}

/**
 * @brief 围绕 iLQR 名义 rollout 原地刷新 LTV-QP 的全部数值。
 * @details 写入状态/控制/增量代价、初值与线性化动力学等式、body 速度与加速度硬边界；
 *          不把轮速圆、轮向量增量和舵角速率伪装为未经验证的线性 QP 约束，后者由
 *          candidate validator 和 ZeroSpeedGuard 作真实复核。
 */
bool LtvQpBuilder::build(
    const State &current_state, const std::vector<State> &nominal_states,
    const std::vector<Control> &nominal_controls,
    const std::vector<Se2Reference> &references, const Control &last_control,
    const Se2MpcConfig &config, LtvQpProblem &problem,
    const ZeroSpeedGuardConfig &guard_config) {
  const auto dimensions = checkedLtvQpDimensions(config.horizon);
  if (!dimensions.valid || !std::isfinite(config.dt) || config.dt <= 0.0) {
    problem.valid = false;
    problem.validation_error = !dimensions.valid
                                   ? dimensions.validation_error
                                   : "dt must be finite and positive";
    return false;
  }
  if (problem.horizon != config.horizon || !problem.hasExpectedLayout()) {
    problem.valid = false;
    problem.validation_error = "preallocated LTV-QP buffer layout mismatch";
    return false;
  }
  problem.valid = false;
  problem.validation_error.clear();
  problem.zero_speed_guard_active = false;
  problem.angle_rate_linearization_valid.fill(false);
  const std::size_t horizon = static_cast<std::size_t>(config.horizon);
  if (nominal_states.size() != horizon + 1 ||
      nominal_controls.size() != horizon || references.size() < horizon + 1) {
    problem.validation_error = "nominal/reference horizon size mismatch";
    return false;
  }
  if (!current_state.allFinite() || !last_control.allFinite() ||
      !finiteStates(nominal_states) || !finiteControls(nominal_controls)) {
    problem.validation_error = "non-finite state or control input";
    return false;
  }
  if (!validNonnegativeLimit(config.max_vx) ||
      !validNonnegativeLimit(config.max_vy) ||
      !validNonnegativeLimit(config.max_wz) ||
      !validNonnegativeLimit(config.max_ax) ||
      !validNonnegativeLimit(config.max_ay) ||
      !validNonnegativeLimit(config.max_awz) ||
      !validNonnegativeLimit(config.wheel_base_x) ||
      !validNonnegativeLimit(config.wheel_base_y) ||
      !validNonnegativeLimit(config.max_wheel_speed) ||
      !validNonnegativeLimit(config.max_wheel_acceleration) ||
      !validNonnegativeLimit(config.max_steer_rate)) {
    problem.validation_error = "non-finite or negative dynamics limit";
    return false;
  }
  if (!validWeight(config.state_weight) ||
      !validWeight(config.control_weight) ||
      !validWeight(config.control_delta_weight) ||
      !validWeight(config.terminal_weight)) {
    problem.validation_error = "QP weights must be finite and non-negative";
    return false;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  problem.hessian.setZero();
  problem.gradient.setZero();
  problem.equality_matrix.setZero();
  problem.equality_lower.setZero();
  problem.equality_upper.setZero();
  problem.inequality_matrix.setZero();
  problem.inequality_lower.setZero();
  problem.inequality_upper.setZero();
  problem.lower_bound.setConstant(-infinity);
  problem.upper_bound.setConstant(infinity);

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

  problem.valid = problem.hasOrderedBounds() && problem.hasFiniteNumerics();
  if (!problem.valid) {
    problem.validation_error = "constructed QP contains non-finite values";
  }
  return problem.valid;
}

}  // namespace ats_swerve_mpc
