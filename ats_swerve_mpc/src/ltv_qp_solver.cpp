// Copyright 2026

#include "ats_swerve_mpc/ltv_qp_solver.hpp"

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

bool vectorFinite(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
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

bool LtvQpSparseStructure::samePattern(
    const LtvQpSparseStructure &other) const {
  return rows == other.rows && columns == other.columns &&
         column_offsets == other.column_offsets &&
         row_indices == other.row_indices;
}

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

bool LtvQpWarmStart::validFor(int decision_size, int constraint_rows) const {
  return primal.size() == decision_size && dual.size() == constraint_rows &&
         primal.allFinite() && dual.allFinite();
}

bool LtvQpSolverSettings::valid() const {
  return max_iterations > 0 && std::isfinite(time_limit_ms) &&
         time_limit_ms > 0.0 && std::isfinite(max_primal_residual) &&
         max_primal_residual >= 0.0 && std::isfinite(max_dual_residual) &&
         max_dual_residual >= 0.0 && std::isfinite(max_tracking_slack) &&
         max_tracking_slack >= 0.0 &&
         std::isfinite(max_hard_constraint_violation) &&
         max_hard_constraint_violation >= 0.0;
}

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
      !safety.collision_free) {
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

}  // namespace ats_swerve_mpc
