// Copyright 2026

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ats_swerve_mpc {
namespace {

constexpr double kConstraintTolerance = 1e-9;

bool finiteOrInfinity(double value) {
  return !std::isnan(value);
}

bool finiteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool vectorFinite(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool finiteControls(const std::vector<Control> &controls) {
  return std::all_of(controls.begin(), controls.end(),
                     [](const Control &control) { return control.allFinite(); });
}

bool finiteStates(const std::vector<State> &states) {
  return std::all_of(states.begin(), states.end(),
                     [](const State &state) { return state.allFinite(); });
}

bool eigenVectorFiniteOrInfinity(const Eigen::VectorXd &values) {
  for (int index = 0; index < values.size(); ++index) {
    if (!finiteOrInfinity(values(index))) {
      return false;
    }
  }
  return true;
}

void setError(std::string *error, const char *message) {
  if (error != nullptr) {
    *error = message;
  }
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

double upperViolation(double value, double upper) {
  return std::max(0.0, value - upper);
}

double absoluteViolation(double value, double bound) {
  return upperViolation(std::abs(value), bound);
}

void reject(LtvQpCandidateAudit &audit, const char *reason) {
  audit.feasible = false;
  audit.rejection_reason = reason;
}

}  // namespace

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

bool LtvQpSparseProblem::valid(std::string *error) const {
  if (decision_size <= 0 || constraint_rows <= 0) {
    setError(error, "sparse QP dimensions must be positive");
    return false;
  }
  if (!hessian_structure.valid(error) ||
      hessian_structure.rows != decision_size ||
      hessian_structure.columns != decision_size ||
      hessian_values.size() != hessian_structure.row_indices.size()) {
    setError(error, "Hessian CSC pattern or value size mismatch");
    return false;
  }
  for (int col = 0; col < decision_size; ++col) {
    const int begin =
        hessian_structure.column_offsets[static_cast<std::size_t>(col)];
    const int end =
        hessian_structure.column_offsets[static_cast<std::size_t>(col + 1)];
    for (int offset = begin; offset < end; ++offset) {
      if (hessian_structure.row_indices[static_cast<std::size_t>(offset)] >
          col) {
        setError(error, "OSQP Hessian must be upper triangular");
        return false;
      }
    }
  }
  if (!constraint_structure.valid(error) ||
      constraint_structure.rows != constraint_rows ||
      constraint_structure.columns != decision_size ||
      constraint_values.size() != constraint_structure.row_indices.size()) {
    setError(error, "constraint CSC pattern or value size mismatch");
    return false;
  }
  if (!vectorFinite(hessian_values) || !vectorFinite(constraint_values) ||
      gradient.size() != decision_size || !gradient.allFinite() ||
      lower.size() != constraint_rows || upper.size() != constraint_rows ||
      !eigenVectorFiniteOrInfinity(lower) ||
      !eigenVectorFiniteOrInfinity(upper) ||
      !(lower.array() <= upper.array()).all()) {
    setError(error, "sparse QP numerical values or bounds invalid");
    return false;
  }
  return true;
}

bool LtvQpWarmStart::validFor(int decision_size, int constraint_rows) const {
  return decision_size > 0 && constraint_rows > 0 &&
         primal.size() == decision_size && dual.size() == constraint_rows &&
         primal.allFinite() && dual.allFinite();
}

bool LtvQpSolverSettings::valid() const {
  return max_iterations > 0 && std::isfinite(time_limit_ms) &&
         time_limit_ms > 0.0 && std::isfinite(max_primal_residual) &&
         max_primal_residual > 0.0 && std::isfinite(max_dual_residual) &&
         max_dual_residual > 0.0 && finiteNonnegative(max_tracking_slack) &&
         finiteNonnegative(max_hard_constraint_violation);
}

LtvQpPrimalCandidate LtvQpCandidateReconstructor::reconstruct(
    const State &current_state, const LtvQpProblem &problem,
    const std::vector<Control> &nominal_controls, const Se2MpcConfig &config,
    const LtvQpSolveResult &result, const Control &last_control,
    double first_step_dt) {
  LtvQpPrimalCandidate candidate;
  if (!problem.valid || problem.horizon <= 0 ||
      !problem.hasExpectedLayout() || !problem.hasOrderedBounds() ||
      !problem.hasFiniteNumerics() || problem.decisionSize() <= 0 ||
      !current_state.allFinite() || !last_control.allFinite() ||
      config.horizon != problem.horizon || !std::isfinite(config.dt) ||
      config.dt <= 0.0 ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      result.primal_solution.size() != problem.decisionSize() ||
      !result.primal_solution.allFinite()) {
    candidate.validation_error = "invalid QP primal reconstruction input";
    return candidate;
  }
  const Se2MpcController projector(config);
  const double effective_first_dt =
      (std::isfinite(first_step_dt) && first_step_dt > 0.0) ? first_step_dt
                                                            : config.dt;
  candidate.controls.reserve(static_cast<std::size_t>(problem.horizon));
  Control prev_control = last_control;
  for (int step = 0; step < problem.horizon; ++step) {
    const std::size_t index = static_cast<std::size_t>(step);
    if (!nominal_controls[index].allFinite()) {
      candidate.validation_error = "non-finite nominal control";
      candidate.controls.clear();
      return candidate;
    }
    const Control raw_control =
        nominal_controls[index] +
        result.primal_solution.segment<3>(problem.controlOffset(step));
    if (!raw_control.allFinite()) {
      candidate.validation_error = "non-finite reconstructed control";
      candidate.controls.clear();
      return candidate;
    }
    const double step_dt = step == 0 ? effective_first_dt : config.dt;
    const Control projected =
        projector.clampStepIncrement(raw_control, prev_control, step_dt);
    if (!projected.allFinite()) {
      candidate.validation_error = "non-finite projected control";
      candidate.controls.clear();
      return candidate;
    }
    candidate.controls.push_back(projected);
    prev_control = projected;
  }
  candidate.states = Se2Model::rolloutWithLag(
      current_state, candidate.controls, last_control, first_step_dt, config.dt,
      config.actuator_lag_tau_sec);
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

LtvQpCandidateAudit LtvQpCandidateValidator::validate(
    const LtvQpProblem &problem, const std::vector<Control> &nominal_controls,
    const Control &last_control, const Se2MpcConfig &config,
    const ZeroSpeedGuardConfig &guard_config,
    const LtvQpSolverSettings &settings, const LtvQpCandidateSafety &safety,
    const LtvQpSolveResult &result, double first_step_dt) {
  LtvQpCandidateAudit audit;
  if (!settings.valid()) {
    reject(audit, "invalid_solver_settings");
    return audit;
  }
  if (!problem.valid || problem.horizon <= 0 ||
      !problem.hasExpectedLayout() || !problem.hasOrderedBounds() ||
      !problem.hasFiniteNumerics() || problem.decisionSize() <= 0 ||
      result.primal_solution.size() != problem.decisionSize() ||
      result.dual_solution.size() != problem.constraintRows() ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      config.horizon != problem.horizon || !std::isfinite(config.dt) ||
      config.dt <= 0.0 || !last_control.allFinite()) {
    reject(audit, "invalid_problem_or_candidate_dimensions");
    return audit;
  }
  if (!result.primal_solution.allFinite() ||
      !result.dual_solution.allFinite() ||
      !std::isfinite(result.solve_time_ms) ||
      !std::isfinite(result.update_time_ms) ||
      !std::isfinite(result.wall_update_time_ms) ||
      !std::isfinite(result.wall_solve_time_ms) ||
      !std::isfinite(result.wall_qp_phase_time_ms) ||
      !std::isfinite(result.wall_complete_qp_phase_time_ms) ||
      !std::isfinite(result.primal_residual) ||
      !std::isfinite(result.dual_residual) ||
      !std::isfinite(result.slack_maximum) ||
      !std::isfinite(result.hard_constraint_maximum_violation) ||
      result.slack_maximum < 0.0 ||
      result.hard_constraint_maximum_violation < 0.0 ||
      result.solve_time_ms < 0.0 || result.update_time_ms < 0.0 ||
      result.wall_update_time_ms < 0.0 || result.wall_solve_time_ms < 0.0 ||
      result.wall_qp_phase_time_ms < 0.0 ||
      result.wall_complete_qp_phase_time_ms < 0.0 ||
      result.primal_residual < 0.0 || result.dual_residual < 0.0) {
    reject(audit, "non_finite_matrix_or_result");
    return audit;
  }
  if (result.status != LtvQpSolverStatus::kSolved &&
      result.status != LtvQpSolverStatus::kSolvedInaccurate) {
    reject(audit, "solver_status_not_solved");
    return audit;
  }
  if (result.iterations < 0 || result.iterations > settings.max_iterations ||
      result.solve_time_ms > settings.time_limit_ms ||
      result.update_time_ms > settings.time_limit_ms ||
      result.wall_update_time_ms > settings.time_limit_ms ||
      result.wall_solve_time_ms > settings.time_limit_ms ||
      result.wall_qp_phase_time_ms > settings.time_limit_ms ||
      result.wall_complete_qp_phase_time_ms > settings.time_limit_ms) {
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
  if (!finiteNonnegative(config.max_vx) ||
      !finiteNonnegative(config.max_vy) ||
      !finiteNonnegative(config.max_wz) ||
      !finiteNonnegative(config.max_ax) ||
      !finiteNonnegative(config.max_ay) ||
      !finiteNonnegative(config.max_awz) ||
      !std::isfinite(config.wheel_base_x) || config.wheel_base_x <= 0.0 ||
      !std::isfinite(config.wheel_base_y) || config.wheel_base_y <= 0.0 ||
      !std::isfinite(config.max_wheel_speed) || config.max_wheel_speed <= 0.0 ||
      !std::isfinite(config.max_wheel_acceleration) ||
      config.max_wheel_acceleration <= 0.0 ||
      !std::isfinite(config.max_steer_rate) || config.max_steer_rate <= 0.0) {
    reject(audit, "unverifiable_hard_constraint_configuration");
    return audit;
  }

  for (int step = 0; step < problem.horizon; ++step) {
    const double slack = result.primal_solution(problem.slackOffset(step));
    if (!std::isfinite(slack)) {
      reject(audit, "non_finite_slack");
      return audit;
    }
    audit.actual_slack_maximum =
        std::max(audit.actual_slack_maximum, std::max(0.0, slack));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation, std::max(0.0, -slack));
  }
  for (const double slack : result.tracking_slacks) {
    if (!std::isfinite(slack)) {
      reject(audit, "non_finite_slack");
      return audit;
    }
    audit.actual_slack_maximum =
        std::max(audit.actual_slack_maximum, std::max(0.0, slack));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation, std::max(0.0, -slack));
  }
  if (result.slack_maximum >
          settings.max_tracking_slack + settings.max_primal_residual ||
      audit.actual_slack_maximum >
          settings.max_tracking_slack + settings.max_primal_residual) {
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

  const Se2MpcController projector(config);
  for (int step = 0; step < problem.horizon; ++step) {
    const std::size_t index = static_cast<std::size_t>(step);
    const Control raw_candidate =
        nominal_controls[index] +
        result.primal_solution.segment<3>(problem.controlOffset(step));
    if (!raw_candidate.allFinite() || !nominal_controls[index].allFinite()) {
      reject(audit, "non_finite_body_control");
      return audit;
    }
    const double step_dt =
        (step == 0 && std::isfinite(first_step_dt) && first_step_dt > 0.0)
            ? first_step_dt
            : config.dt;
    const Control candidate =
        projector.clampStepIncrement(raw_candidate, previous, step_dt);
    if (!candidate.allFinite()) {
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

    // 旋转系速度搬运 R(-wz*dt)：与 Se2MpcController 惯性加速度约束严格统一
    const Control carried_previous =
        Se2Model::rotateControlLinear(previous, previous(2) * step_dt);
    const Control body_delta = candidate - carried_previous;
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(0), config.max_ax * step_dt));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(1), config.max_ay * step_dt));
    audit.actual_hard_constraint_maximum_violation = std::max(
        audit.actual_hard_constraint_maximum_violation,
        absoluteViolation(body_delta(2), config.max_awz * step_dt));

    const auto previous_modules = moduleVelocities(previous, config);
    const auto candidate_modules = moduleVelocities(candidate, config);
    std::array<double, 4> candidate_speeds{};
    for (std::size_t module = 0; module < candidate_modules.size(); ++module) {
      const double previous_speed = previous_modules[module].norm();
      const double candidate_speed = candidate_modules[module].norm();
      candidate_speeds[module] = candidate_speed;
      const double wheel_speed_viol =
          upperViolation(candidate_speed, config.max_wheel_speed);
      audit.max_wheel_speed_violation =
          std::max(audit.max_wheel_speed_violation, wheel_speed_viol);
      audit.actual_hard_constraint_maximum_violation = std::max(
          audit.actual_hard_constraint_maximum_violation, wheel_speed_viol);

      double cosine = 1.0;
      const bool both_moving = candidate_speed > 1e-4 && previous_speed > 1e-4;
      if (both_moving) {
        cosine = std::clamp(
            candidate_modules[module].dot(previous_modules[module]) /
                (candidate_speed * previous_speed),
            -1.0, 1.0);
      }
      // 统一滚速标量增量 + 舵角速率解耦审计（与 Se2MpcController::moduleIncrementFeasible 对齐）
      const double rolling_speed_delta =
          both_moving ? (cosine >= 0.0 ? std::abs(candidate_speed - previous_speed)
                                       : (candidate_speed + previous_speed))
                      : (candidate_modules[module] - previous_modules[module]).norm();
      audit.actual_hard_constraint_maximum_violation = std::max(
          audit.actual_hard_constraint_maximum_violation,
          upperViolation(rolling_speed_delta,
                         config.max_wheel_acceleration * step_dt));

      if (zero_speed_guard.angleConstraintDefined(previous_speed,
                                                  candidate_speed)) {
        ++audit.steering_rate_constraints_checked;
        audit.actual_hard_constraint_maximum_violation = std::max(
            audit.actual_hard_constraint_maximum_violation,
            upperViolation(std::acos(std::abs(cosine)),
                           config.max_steer_rate * step_dt));
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

LtvQpCandidateAudit LtvQpCandidateValidator::validate(
    const LtvQpProblem &problem, const std::vector<Control> &nominal_controls,
    const Control &last_control, const Se2MpcConfig &config,
    const ZeroSpeedGuardConfig &guard_config,
    const LtvQpSolverSettings &settings, const LtvQpCandidateSafety &safety,
    const LtvQpSolveResult &result, const LtvQpPrimalCandidate &candidate,
    double first_step_dt) {
  LtvQpCandidateAudit audit;
  if (!problem.valid || problem.horizon <= 0 ||
      !problem.hasExpectedLayout() || !problem.hasOrderedBounds()) {
    reject(audit, "invalid_problem_or_candidate_dimensions");
    return audit;
  }
  if (!problem.hasFiniteNumerics()) {
    reject(audit, "non_finite_matrix_or_result");
    return audit;
  }
  const int decision_size = problem.decisionSize();
  if (!settings.valid()) {
    reject(audit, "invalid_solver_settings");
    return audit;
  }
  if (decision_size <= 0 || config.horizon != problem.horizon ||
      !std::isfinite(config.dt) || config.dt <= 0.0 ||
      !last_control.allFinite() ||
      result.primal_solution.size() != decision_size ||
      result.dual_solution.size() != problem.constraintRows() ||
      !result.primal_solution.allFinite() ||
      !result.dual_solution.allFinite() ||
      !finiteControls(nominal_controls) || !candidate.valid ||
      nominal_controls.size() != static_cast<std::size_t>(problem.horizon) ||
      candidate.controls.size() != static_cast<std::size_t>(problem.horizon) ||
      candidate.states.size() != candidate.controls.size() + 1 ||
      !finiteControls(candidate.controls) || !finiteStates(candidate.states)) {
    reject(audit, "nonlinear_rollout_reconstruction_reject");
    return audit;
  }
  const Se2MpcController projector(config);
  const double effective_first_dt =
      (std::isfinite(first_step_dt) && first_step_dt > 0.0) ? first_step_dt
                                                            : config.dt;
  Control prev_control = last_control;
  for (int step = 0; step < problem.horizon; ++step) {
    const int control_offset = problem.controlOffset(step);
    if (control_offset < 0 || control_offset > decision_size - 3) {
      reject(audit, "invalid_problem_or_candidate_dimensions");
      return audit;
    }
    const Control raw =
        nominal_controls[static_cast<std::size_t>(step)] +
        result.primal_solution.segment<3>(control_offset);
    const double step_dt = step == 0 ? effective_first_dt : config.dt;
    const Control reconstructed =
        projector.clampStepIncrement(raw, prev_control, step_dt);
    if (!reconstructed.allFinite() ||
        (candidate.controls[static_cast<std::size_t>(step)] - reconstructed)
                .lpNorm<Eigen::Infinity>() > 1e-9) {
      reject(audit, "nonlinear_rollout_identity_reject");
      return audit;
    }
    prev_control = reconstructed;
  }
  return validate(problem, nominal_controls, last_control, config, guard_config,
                  settings, safety, result, first_step_dt);
}

}  // namespace ats_swerve_mpc
