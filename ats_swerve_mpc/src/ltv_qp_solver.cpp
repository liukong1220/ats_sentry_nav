// Copyright 2026

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ats_swerve_mpc {
namespace {

constexpr double kConstraintTolerance = 1e-9;

/** @brief 接受有限数与 +/-infinity 的约束边界，但拒绝 NaN。 */
bool finiteOrInfinity(double value) {
  return !std::isnan(value);
}

/** @brief 检查 CSC 数值数组没有 NaN/Inf，矩阵结构由另一层单独校验。 */
bool vectorFinite(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

/** @brief 检查重建出的每一拍车体系控制均有限。 */
bool finiteControls(const std::vector<Control> &controls) {
  return std::all_of(controls.begin(), controls.end(),
                     [](const Control &control) { return control.allFinite(); });
}

/** @brief 检查共享 SE(2) 非线性 rollout 的每一状态均有限。 */
bool finiteStates(const std::vector<State> &states) {
  return std::all_of(states.begin(), states.end(),
                     [](const State &state) { return state.allFinite(); });
}

/** @brief 检查 Eigen 约束向量，允许 OSQP 使用的无穷边界。 */
bool eigenVectorFiniteOrInfinity(const Eigen::VectorXd &values) {
  for (int index = 0; index < values.size(); ++index) {
    if (!finiteOrInfinity(values(index))) {
      return false;
    }
  }
  return true;
}

/** @brief 向可选错误输出写入稳定原因，避免校验函数抛出异常。 */
void setError(std::string *error, const char *message) {
  if (error != nullptr) {
    *error = message;
  }
}

/** @brief 将同一车体 Twist 映射到真实四个轮心速度，用作独立 hard-check。 */
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

/** @brief 返回 value 超过单边上限的非负违反量。 */
double upperViolation(double value, double upper) {
  return std::max(0.0, value - upper);
}

/** @brief 返回绝对值约束的违反量，适用于车体速度和加速度。 */
double absoluteViolation(double value, double bound) {
  return upperViolation(std::abs(value), bound);
}

/** @brief 统一记录 fail-closed 审计拒绝原因。 */
void reject(LtvQpCandidateAudit &audit, const char *reason) {
  audit.feasible = false;
  audit.rejection_reason = reason;
}

}  // namespace

/** @brief 将统一状态码转为日志字段，保持 OSQP 原始 status 的可审计性。 */
const char *ltvQpSolverStatusName(LtvQpSolverStatus status) {
  switch (status) {
    case LtvQpSolverStatus::kSolved:
      return "solved";
    case LtvQpSolverStatus::kSolvedInaccurate:
      return "solved_inaccurate";
    case LtvQpSolverStatus::kMaxIterations:
      return "max_iterations";
    case LtvQpSolverStatus::kTimeLimit:
      return "time_limit";
    case LtvQpSolverStatus::kPrimalInfeasible:
      return "primal_infeasible";
    case LtvQpSolverStatus::kDualInfeasible:
      return "dual_infeasible";
    case LtvQpSolverStatus::kNumericalFailure:
      return "numerical_failure";
    case LtvQpSolverStatus::kInvalidProblem:
      return "invalid_problem";
    case LtvQpSolverStatus::kBackendUnavailable:
      return "backend_unavailable";
  }
  return "unknown";
}

/** @brief 验证 CSC 的列偏移、行索引范围和严格有序性。 */
bool LtvQpSparseStructure::valid(std::string *error) const {
  if (rows <= 0 || columns <= 0 ||
      column_offsets.size() != static_cast<std::size_t>(columns + 1) ||
      column_offsets.empty() || column_offsets.front() != 0 ||
      column_offsets.back() != static_cast<int>(row_indices.size())) {
    setError(error, "invalid CSC dimensions or column offsets");
    return false;
  }
  for (int column = 0; column < columns; ++column) {
    const int begin = column_offsets[static_cast<std::size_t>(column)];
    const int end = column_offsets[static_cast<std::size_t>(column + 1)];
    if (begin > end || begin < 0 || end > static_cast<int>(row_indices.size())) {
      setError(error, "invalid CSC column range");
      return false;
    }
    int previous_row = -1;
    for (int offset = begin; offset < end; ++offset) {
      const int row = row_indices[static_cast<std::size_t>(offset)];
      if (row < 0 || row >= rows || row <= previous_row) {
        setError(error, "CSC rows must be in range and strictly ordered");
        return false;
      }
      previous_row = row;
    }
  }
  return true;
}

/** @brief 比较两个 CSC layout 的所有形状与索引，拒绝 timer 内结构漂移。 */
bool LtvQpSparseStructure::samePattern(
    const LtvQpSparseStructure &other) const {
  return rows == other.rows && columns == other.columns &&
         column_offsets == other.column_offsets &&
         row_indices == other.row_indices;
}

/** @brief 同时验证 CSC 数值、gradient 与 lower<=upper 的固定维度契约。 */
bool LtvQpSparseProblem::valid(std::string *error) const {
  if (decision_size <= 0 || constraint_rows <= 0 ||
      !hessian_structure.valid(error) || !constraint_structure.valid(error)) {
    return false;
  }
  if (hessian_structure.rows != decision_size ||
      hessian_structure.columns != decision_size ||
      constraint_structure.rows != constraint_rows ||
      constraint_structure.columns != decision_size) {
    setError(error, "sparse matrix dimensions do not match QP dimensions");
    return false;
  }
  if (hessian_values.size() != hessian_structure.row_indices.size() ||
      constraint_values.size() != constraint_structure.row_indices.size() ||
      !vectorFinite(hessian_values) || !vectorFinite(constraint_values) ||
      gradient.size() != decision_size || lower.size() != constraint_rows ||
      upper.size() != constraint_rows || !gradient.allFinite()) {
    setError(error, "invalid sparse values or vector dimensions");
    return false;
  }
  for (int row = 0; row < constraint_rows; ++row) {
    if (!finiteOrInfinity(lower(row)) || !finiteOrInfinity(upper(row)) ||
        lower(row) > upper(row)) {
      setError(error, "invalid lower or upper constraint bound");
      return false;
    }
  }
  return true;
}

/** @brief 仅接受与当前 decision/constraint 完全同维且有限的 primal/dual 初值。 */
bool LtvQpWarmStart::validFor(int decision_size, int constraint_rows) const {
  return primal.size() == decision_size && dual.size() == constraint_rows &&
         primal.allFinite() && dual.allFinite();
}

/** @brief 校验所有 solver 接受门限是有限、非负且具有正的资源上界。 */
bool LtvQpSolverSettings::valid() const {
  return max_iterations > 0 && std::isfinite(time_limit_ms) &&
         time_limit_ms > 0.0 && std::isfinite(max_primal_residual) &&
         max_primal_residual >= 0.0 && std::isfinite(max_dual_residual) &&
         max_dual_residual >= 0.0 && std::isfinite(max_tracking_slack) &&
         max_tracking_slack >= 0.0 &&
         std::isfinite(max_hard_constraint_violation) &&
         max_hard_constraint_violation >= 0.0;
}

/**
 * @brief 从 OSQP primal 的 delta_u 重建绝对控制并进行非线性 SE(2) 前向复算。
 * @details 该过程使 candidate 复核不依赖 LTV 近似；任何尺寸、有限性或 rollout 问题
 *          都只产生拒绝诊断，绝不生成可发布的 QP Twist。
 */
LtvQpPrimalCandidate LtvQpCandidateReconstructor::reconstruct(
    const State &current_state, const LtvQpProblem &problem,
    const std::vector<Control> &nominal_controls,
    const Se2MpcConfig &config, const LtvQpSolveResult &result) {
  LtvQpPrimalCandidate candidate;
  if (!problem.valid || problem.horizon <= 0 ||
      problem.decisionSize() <= 0 || !current_state.allFinite() ||
      config.horizon != problem.horizon || !std::isfinite(config.dt) ||
      config.dt <= 0.0 ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      result.primal_solution.size() != problem.decisionSize() ||
      !result.primal_solution.allFinite()) {
    candidate.validation_error = "invalid QP primal reconstruction input";
    return candidate;
  }
  candidate.controls.reserve(static_cast<std::size_t>(problem.horizon));
  for (int step = 0; step < problem.horizon; ++step) {
    const std::size_t index = static_cast<std::size_t>(step);
    if (!nominal_controls[index].allFinite()) {
      candidate.validation_error = "non-finite nominal control";
      candidate.controls.clear();
      return candidate;
    }
    const Control control = nominal_controls[index] +
        result.primal_solution.segment<3>(problem.controlOffset(step));
    if (!control.allFinite()) {
      candidate.validation_error = "non-finite reconstructed control";
      candidate.controls.clear();
      return candidate;
    }
    candidate.controls.push_back(control);
  }
  candidate.states = Se2Model(config.dt).rollout(current_state,
                                                  candidate.controls);
  if (candidate.states.size() != candidate.controls.size() + 1 ||
      !finiteStates(candidate.states)) {
    candidate.validation_error = "non-finite nonlinear rollout";
    candidate.controls.clear();
    candidate.states.clear();
    return candidate;
  }
  candidate.valid = true;
  return candidate;
}

/**
 * @brief 基于 nominal+delta_u 执行后端无关的完整安全审计。
 * @details 依次核验 QP 维度/状态/资源、残差/slack、外部健康门和真实四轮速度、
 *          轮向量增量、ZeroSpeedGuard 下有效舵角速率；任一步失败都 fail-closed。
 */
LtvQpCandidateAudit LtvQpCandidateValidator::validate(
    const LtvQpProblem &problem,
    const std::vector<Control> &nominal_controls,
    const Control &last_control,
    const Se2MpcConfig &config,
    const ZeroSpeedGuardConfig &guard_config,
    const LtvQpSolverSettings &settings,
    const LtvQpCandidateSafety &safety,
    const LtvQpSolveResult &result) {
  LtvQpCandidateAudit audit;
  if (!settings.valid()) {
    reject(audit, "invalid_solver_settings");
    return audit;
  }
  if (!problem.valid || problem.horizon <= 0 ||
      problem.decisionSize() <= 0 ||
      result.primal_solution.size() != problem.decisionSize() ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      config.horizon != problem.horizon || config.dt <= 0.0 ||
      !last_control.allFinite()) {
    reject(audit, "invalid_problem_or_candidate_dimensions");
    return audit;
  }
  if (!problem.hessian.allFinite() || !problem.gradient.allFinite() ||
      !problem.equality_matrix.allFinite() ||
      !problem.equality_lower.allFinite() || !problem.equality_upper.allFinite() ||
      !problem.inequality_matrix.allFinite() ||
      !problem.inequality_lower.allFinite() ||
      !problem.inequality_upper.allFinite() ||
      !eigenVectorFiniteOrInfinity(problem.lower_bound) ||
      !eigenVectorFiniteOrInfinity(problem.upper_bound) ||
      !result.primal_solution.allFinite() ||
      !std::isfinite(result.solve_time_ms) ||
      !std::isfinite(result.update_time_ms) ||
      !std::isfinite(result.primal_residual) ||
      !std::isfinite(result.dual_residual) ||
      !std::isfinite(result.slack_maximum) ||
      !std::isfinite(result.hard_constraint_maximum_violation) ||
      result.slack_maximum < 0.0 ||
      result.hard_constraint_maximum_violation < 0.0) {
    reject(audit, "non_finite_matrix_or_result");
    return audit;
  }
  if (result.status != LtvQpSolverStatus::kSolved) {
    reject(audit, "solver_status_not_solved");
    return audit;
  }
  if (result.iterations < 0 || result.iterations > settings.max_iterations ||
      result.solve_time_ms > settings.time_limit_ms) {
    reject(audit, "iteration_or_deadline_reject");
    return audit;
  }
  if (result.primal_residual > settings.max_primal_residual ||
      result.dual_residual > settings.max_dual_residual) {
    reject(audit, "residual_reject");
    return audit;
  }
  if (!safety.inputs_healthy || safety.emergency_stop_active ||
      !safety.collision_free || !safety.localization_fresh ||
      !safety.reference_fresh || !safety.execution_lease_valid ||
      !safety.gimbal_valid || !safety.map_fresh) {
    reject(audit, "input_health_emergency_or_collision_reject");
    return audit;
  }
  if (config.max_vx < 0.0 || config.max_vy < 0.0 || config.max_wz < 0.0 ||
      config.max_ax < 0.0 || config.max_ay < 0.0 || config.max_awz < 0.0 ||
      config.wheel_base_x <= 0.0 || config.wheel_base_y <= 0.0 ||
      config.max_wheel_speed <= 0.0 || config.max_wheel_acceleration <= 0.0 ||
      config.max_steer_rate <= 0.0) {
    reject(audit, "unverifiable_hard_constraint_configuration");
    return audit;
  }

  for (const double slack : result.tracking_slacks) {
    if (!std::isfinite(slack)) {
      reject(audit, "non_finite_slack");
      return audit;
    }
    audit.actual_slack_maximum =
        std::max(audit.actual_slack_maximum, slack);
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation, std::max(0.0, -slack));
  }
  // The current LtvQpBuilder allocates no tracking/terminal slack columns.
  // Rejecting a nonempty vector prevents an adapter from silently softening a
  // hard constraint before the slack layout and bounds are reviewed.
  if (!result.tracking_slacks.empty() ||
      result.slack_maximum > settings.max_tracking_slack ||
      audit.actual_slack_maximum > settings.max_tracking_slack) {
    reject(audit, "unexpected_or_excessive_tracking_slack");
    return audit;
  }

  Control previous = last_control;
  ZeroSpeedGuard zero_speed_guard(guard_config);
  const auto initial_modules = moduleVelocities(previous, config);
  std::array<double, 4> initial_speeds{};
  for (std::size_t index = 0; index < initial_speeds.size(); ++index) {
    initial_speeds[index] = initial_modules[index].norm();
  }
  zero_speed_guard.update(initial_speeds);

  for (int step = 0; step < problem.horizon; ++step) {
    const std::size_t index = static_cast<std::size_t>(step);
    const Control candidate =
        nominal_controls[index] +
        result.primal_solution.segment<3>(problem.controlOffset(step));
    if (!candidate.allFinite() || !nominal_controls[index].allFinite()) {
      reject(audit, "non_finite_body_control");
      return audit;
    }
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(candidate(0), config.max_vx));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(candidate(1), config.max_vy));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(candidate(2), config.max_wz));

    const Control body_delta = candidate - previous;
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(0), config.max_ax * config.dt));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(1), config.max_ay * config.dt));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(2), config.max_awz * config.dt));

    const auto previous_modules = moduleVelocities(previous, config);
    const auto candidate_modules = moduleVelocities(candidate, config);
    std::array<double, 4> candidate_speeds{};
    for (std::size_t module = 0; module < candidate_modules.size(); ++module) {
      const double previous_speed = previous_modules[module].norm();
      const double candidate_speed = candidate_modules[module].norm();
      candidate_speeds[module] = candidate_speed;
      audit.actual_hard_constraint_maximum_violation = std::max(
          audit.actual_hard_constraint_maximum_violation,
          upperViolation(candidate_speed, config.max_wheel_speed));
      audit.actual_hard_constraint_maximum_violation = std::max(
          audit.actual_hard_constraint_maximum_violation,
          upperViolation((candidate_modules[module] - previous_modules[module]).norm(),
                         config.max_wheel_acceleration * config.dt));

      if (zero_speed_guard.angleConstraintDefined(previous_speed,
                                                  candidate_speed)) {
        ++audit.steering_rate_constraints_checked;
        const double cosine = std::clamp(
            candidate_modules[module].dot(previous_modules[module]) /
                (candidate_speed * previous_speed),
            -1.0, 1.0);
        audit.actual_hard_constraint_maximum_violation = std::max(
            audit.actual_hard_constraint_maximum_violation,
            upperViolation(std::acos(cosine), config.max_steer_rate * config.dt));
      } else {
        ++audit.steering_rate_constraints_skipped_by_zero_speed_guard;
      }
    }
    zero_speed_guard.update(candidate_speeds);
    previous = candidate;
  }

  audit.actual_hard_constraint_maximum_violation = std::max(
      audit.actual_hard_constraint_maximum_violation,
      result.hard_constraint_maximum_violation);
  if (audit.actual_hard_constraint_maximum_violation >
      settings.max_hard_constraint_violation + kConstraintTolerance) {
    reject(audit, "hard_constraint_reject");
    return audit;
  }
  audit.feasible = true;
  return audit;
}

/** @brief 在基础 hard-check 前确认重建控制与 primal identity 及 nonlinear rollout 一致。 */
LtvQpCandidateAudit LtvQpCandidateValidator::validate(
    const LtvQpProblem &problem,
    const std::vector<Control> &nominal_controls,
    const Control &last_control,
    const Se2MpcConfig &config,
    const ZeroSpeedGuardConfig &guard_config,
    const LtvQpSolverSettings &settings,
    const LtvQpCandidateSafety &safety,
    const LtvQpSolveResult &result,
    const LtvQpPrimalCandidate &candidate) {
  LtvQpCandidateAudit audit;
  if (!problem.valid || problem.horizon <= 0 ||
      result.primal_solution.size() != problem.decisionSize() ||
      !candidate.valid ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      candidate.controls.size() != static_cast<std::size_t>(problem.horizon) ||
      candidate.states.size() != candidate.controls.size() + 1 ||
      !finiteControls(candidate.controls) || !finiteStates(candidate.states)) {
    reject(audit, "nonlinear_rollout_reconstruction_reject");
    return audit;
  }
  for (int step = 0; step < problem.horizon; ++step) {
    const Control reconstructed = nominal_controls[static_cast<std::size_t>(step)] +
        result.primal_solution.segment<3>(problem.controlOffset(step));
    if (!reconstructed.allFinite() ||
        (candidate.controls[static_cast<std::size_t>(step)] - reconstructed)
                .lpNorm<Eigen::Infinity>() > 1e-9) {
      reject(audit, "nonlinear_rollout_identity_reject");
      return audit;
    }
  }
  return validate(problem, nominal_controls, last_control, config, guard_config,
                  settings, safety, result);
}

}  // namespace ats_swerve_mpc
