// Copyright 2026

#include "ats_swerve_mpc/ltv_qp_osqp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <osqp.h>

namespace ats_swerve_mpc {
namespace {

int ltvHorizonForDecisionSize(int decision_size) {
  if (decision_size < 9 || (decision_size - 3) % 6 != 0) {
    return -1;
  }
  return (decision_size - 3) / 6;
}

bool isLtvDimensions(int decision_size, int constraint_rows) {
  const int horizon = ltvHorizonForDecisionSize(decision_size);
  return horizon > 0 && constraint_rows == 12 * horizon + 6;
}

double ltvConstraintValue(const LtvQpProblem &problem, int row, int column) {
  const int equality_rows = problem.equality_matrix.rows();
  const int inequality_rows = problem.inequality_matrix.rows();
  if (row < equality_rows) {
    return problem.equality_matrix(row, column);
  }
  if (row < equality_rows + inequality_rows) {
    return problem.inequality_matrix(row - equality_rows, column);
  }
  return row - equality_rows - inequality_rows == column ? 1.0 : 0.0;
}

}  // namespace

LtvQpSparseProblem makeLtvQpSparseProblem(const LtvQpProblem &problem) {
  LtvQpSparseProblem sparse;
  if (!problem.valid || problem.decisionSize() <= 0 ||
      problem.hessian.rows() != problem.decisionSize() ||
      problem.hessian.cols() != problem.decisionSize()) {
    return sparse;
  }
  const int n = problem.decisionSize();
  sparse.decision_size = n;
  sparse.constraint_rows = problem.equality_matrix.rows() +
                           problem.inequality_matrix.rows() + n;
  sparse.hessian_structure = LtvQpOsqpSolver::ltvHessianPattern(n);
  sparse.constraint_structure =
      LtvQpOsqpSolver::ltvConstraintPattern(sparse.decision_size,
                                             sparse.constraint_rows);
  sparse.hessian_values.reserve(sparse.hessian_structure.row_indices.size());
  for (int column = 0; column < n; ++column) {
    const int begin = sparse.hessian_structure.column_offsets[
        static_cast<std::size_t>(column)];
    const int end = sparse.hessian_structure.column_offsets[
        static_cast<std::size_t>(column + 1)];
    for (int offset = begin; offset < end; ++offset) {
      const int row = sparse.hessian_structure.row_indices[
          static_cast<std::size_t>(offset)];
      sparse.hessian_values.push_back(problem.hessian(row, column));
    }
  }
  sparse.constraint_values.reserve(
      sparse.constraint_structure.row_indices.size());
  for (int column = 0; column < n; ++column) {
    const int begin = sparse.constraint_structure.column_offsets[
        static_cast<std::size_t>(column)];
    const int end = sparse.constraint_structure.column_offsets[
        static_cast<std::size_t>(column + 1)];
    for (int offset = begin; offset < end; ++offset) {
      sparse.constraint_values.push_back(ltvConstraintValue(
          problem, sparse.constraint_structure.row_indices[
                       static_cast<std::size_t>(offset)], column));
    }
  }
  sparse.gradient = problem.gradient;
  sparse.lower.resize(sparse.constraint_rows);
  sparse.upper.resize(sparse.constraint_rows);
  const int equality_rows = problem.equality_matrix.rows();
  const int inequality_rows = problem.inequality_matrix.rows();
  sparse.lower.head(equality_rows) = problem.equality_lower;
  sparse.upper.head(equality_rows) = problem.equality_upper;
  sparse.lower.segment(equality_rows, inequality_rows) = problem.inequality_lower;
  sparse.upper.segment(equality_rows, inequality_rows) = problem.inequality_upper;
  sparse.lower.tail(n) = problem.lower_bound;
  sparse.upper.tail(n) = problem.upper_bound;
  if (!sparse.valid()) {
    sparse = LtvQpSparseProblem();
  }
  return sparse;
}

LtvQpSparseStructure LtvQpOsqpSolver::denseUpperPattern(int size) {
  LtvQpSparseStructure pattern;
  pattern.rows = size;
  pattern.columns = size;
  pattern.column_offsets.reserve(static_cast<std::size_t>(size + 1));
  pattern.row_indices.reserve(static_cast<std::size_t>(size * (size + 1) / 2));
  pattern.column_offsets.push_back(0);
  for (int column = 0; column < size; ++column) {
    for (int row = 0; row <= column; ++row) {
      pattern.row_indices.push_back(row);
    }
    pattern.column_offsets.push_back(
        static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

LtvQpSparseStructure LtvQpOsqpSolver::densePattern(int rows, int columns) {
  LtvQpSparseStructure pattern;
  pattern.rows = rows;
  pattern.columns = columns;
  pattern.column_offsets.reserve(static_cast<std::size_t>(columns + 1));
  pattern.row_indices.reserve(static_cast<std::size_t>(rows * columns));
  pattern.column_offsets.push_back(0);
  for (int column = 0; column < columns; ++column) {
    for (int row = 0; row < rows; ++row) {
      pattern.row_indices.push_back(row);
    }
    pattern.column_offsets.push_back(
        static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

LtvQpSparseStructure LtvQpOsqpSolver::ltvHessianPattern(
    int decision_size) {
  const int horizon = ltvHorizonForDecisionSize(decision_size);
  if (horizon <= 0) {
    return denseUpperPattern(decision_size);
  }
  LtvQpSparseStructure pattern;
  pattern.rows = decision_size;
  pattern.columns = decision_size;
  pattern.column_offsets.reserve(static_cast<std::size_t>(decision_size + 1));
  pattern.column_offsets.push_back(0);
  const int control_start = 3 * (horizon + 1);
  for (int column = 0; column < decision_size; ++column) {
    if (column >= control_start && column >= control_start + 3) {
      pattern.row_indices.push_back(column - 3);
    }
    pattern.row_indices.push_back(column);
    pattern.column_offsets.push_back(
        static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

LtvQpSparseStructure LtvQpOsqpSolver::ltvConstraintPattern(
    int decision_size, int constraint_rows) {
  if (!isLtvDimensions(decision_size, constraint_rows)) {
    return densePattern(constraint_rows, decision_size);
  }
  const int horizon = ltvHorizonForDecisionSize(decision_size);
  const int equality_rows = 3 * (horizon + 1);
  const int inequality_rows = 3 * horizon;
  const int control_start = equality_rows;
  std::vector<std::vector<int>> rows_by_column(
      static_cast<std::size_t>(decision_size));
  const auto add = [&rows_by_column](int row, int column) {
      rows_by_column[static_cast<std::size_t>(column)].push_back(row);
    };
  for (int axis = 0; axis < 3; ++axis) {
    add(axis, axis);
  }
  for (int step = 0; step < horizon; ++step) {
    const int row = 3 * (step + 1);
    const int current_state = 3 * step;
    const int next_state = 3 * (step + 1);
    const int control = control_start + 3 * step;
    for (int column_axis = 0; column_axis < 3; ++column_axis) {
      for (int row_axis = 0; row_axis < 3; ++row_axis) {
        add(row + row_axis, current_state + column_axis);
        add(row + row_axis, control + column_axis);
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      add(row + axis, next_state + axis);
    }
  }
  for (int step = 0; step < horizon; ++step) {
    const int row = equality_rows + 3 * step;
    const int control = control_start + 3 * step;
    for (int axis = 0; axis < 3; ++axis) {
      add(row + axis, control + axis);
      if (step > 0) {
        add(row + axis, control - 3 + axis);
      }
    }
  }
  const int bound_offset = equality_rows + inequality_rows;
  for (int index = 0; index < decision_size; ++index) {
    add(bound_offset + index, index);
  }
  LtvQpSparseStructure pattern;
  pattern.rows = constraint_rows;
  pattern.columns = decision_size;
  pattern.column_offsets.reserve(static_cast<std::size_t>(decision_size + 1));
  pattern.column_offsets.push_back(0);
  for (auto &rows : rows_by_column) {
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    pattern.row_indices.insert(pattern.row_indices.end(), rows.begin(), rows.end());
    pattern.column_offsets.push_back(static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

void LtvQpOsqpSolver::OsqpDeleter::operator()(OSQPSolver *solver) const {
  if (solver != nullptr) {
    osqp_cleanup(solver);
  }
}

LtvQpOsqpSolver::LtvQpOsqpSolver(
    int decision_size, int constraint_rows,
    const LtvQpSolverSettings &setup_settings)
    : decision_size_(decision_size), constraint_rows_(constraint_rows) {
  if (decision_size_ <= 0 || constraint_rows_ <= 0 || !setup_settings.valid()) {
    return;
  }
  hessian_structure_ = ltvHessianPattern(decision_size_);
  constraint_structure_ = ltvConstraintPattern(decision_size_, constraint_rows_);
  hessian_values_.assign(hessian_structure_.row_indices.size(), 0.0);
  constraint_values_.assign(constraint_structure_.row_indices.size(), 0.0);
  gradient_.assign(static_cast<std::size_t>(decision_size_), 0.0);
  lower_.assign(static_cast<std::size_t>(constraint_rows_),
                -std::numeric_limits<double>::infinity());
  upper_.assign(static_cast<std::size_t>(constraint_rows_),
                std::numeric_limits<double>::infinity());
  hessian_column_offsets_.assign(hessian_structure_.column_offsets.begin(),
                                 hessian_structure_.column_offsets.end());
  hessian_row_indices_.assign(hessian_structure_.row_indices.begin(),
                              hessian_structure_.row_indices.end());
  constraint_column_offsets_.assign(constraint_structure_.column_offsets.begin(),
                                    constraint_structure_.column_offsets.end());
  constraint_row_indices_.assign(constraint_structure_.row_indices.begin(),
                                 constraint_structure_.row_indices.end());

  OSQPCscMatrix hessian{
      static_cast<OSQPInt>(decision_size_), static_cast<OSQPInt>(decision_size_),
      hessian_column_offsets_.data(), hessian_row_indices_.data(),
      hessian_values_.data(), static_cast<OSQPInt>(hessian_values_.size()), -1, 0};
  OSQPCscMatrix constraints{
      static_cast<OSQPInt>(constraint_rows_), static_cast<OSQPInt>(decision_size_),
      constraint_column_offsets_.data(), constraint_row_indices_.data(),
      constraint_values_.data(), static_cast<OSQPInt>(constraint_values_.size()), -1,
      0};
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.profiler_level = 0;
  settings.allocate_solution = 1;
  settings.warm_starting = 1;
  settings.max_iter = static_cast<OSQPInt>(setup_settings.max_iterations);
  settings.time_limit = setup_settings.time_limit_ms / 1000.0;
  settings.eps_abs = std::min(setup_settings.max_primal_residual,
                              setup_settings.max_dual_residual);
  settings.eps_rel = 1e-6;
  settings.polishing = 0;
  OSQPSolver *raw_solver = nullptr;
  const OSQPInt setup_result = osqp_setup(
      &raw_solver, &hessian, gradient_.data(), &constraints, lower_.data(),
      upper_.data(), static_cast<OSQPInt>(constraint_rows_),
      static_cast<OSQPInt>(decision_size_), &settings);
  if (setup_result == 0) {
    solver_.reset(raw_solver);
    setup_count_ = 1;
  }
}

LtvQpOsqpSolver::~LtvQpOsqpSolver() = default;

void LtvQpOsqpSolver::resetWarmStart() {
  if (solver_ != nullptr) {
    osqp_cold_start(solver_.get());
  }
}

bool LtvQpOsqpSolver::copyNumericalValues(
    const LtvQpSparseProblem &problem, std::vector<double> &hessian_values,
    std::vector<double> &constraint_values, std::vector<double> &gradient,
    std::vector<double> &lower, std::vector<double> &upper) {
  if (!problem.valid() || hessian_values.size() != problem.hessian_values.size() ||
      constraint_values.size() != problem.constraint_values.size() ||
      gradient.size() != static_cast<std::size_t>(problem.decision_size) ||
      lower.size() != static_cast<std::size_t>(problem.constraint_rows) ||
      upper.size() != static_cast<std::size_t>(problem.constraint_rows)) {
    return false;
  }
  std::copy(problem.hessian_values.begin(), problem.hessian_values.end(),
            hessian_values.begin());
  std::copy(problem.constraint_values.begin(), problem.constraint_values.end(),
            constraint_values.begin());
  for (int index = 0; index < problem.gradient.size(); ++index) {
    gradient[static_cast<std::size_t>(index)] = problem.gradient(index);
  }
  for (int index = 0; index < problem.constraint_rows; ++index) {
    lower[static_cast<std::size_t>(index)] = problem.lower(index);
    upper[static_cast<std::size_t>(index)] = problem.upper(index);
  }
  return true;
}

LtvQpSolverStatus LtvQpOsqpSolver::mapStatus(int status) {
  switch (status) {
    case OSQP_SOLVED:
      return LtvQpSolverStatus::kSolved;
    case OSQP_SOLVED_INACCURATE:
      return LtvQpSolverStatus::kSolvedInaccurate;
    case OSQP_MAX_ITER_REACHED:
      return LtvQpSolverStatus::kMaxIterations;
    case OSQP_TIME_LIMIT_REACHED:
      return LtvQpSolverStatus::kTimeLimit;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
      return LtvQpSolverStatus::kPrimalInfeasible;
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return LtvQpSolverStatus::kDualInfeasible;
    case OSQP_NON_CVX:
    case OSQP_SIGINT:
    case OSQP_UNSOLVED:
    default:
      return LtvQpSolverStatus::kNumericalFailure;
  }
}

LtvQpSolveResult LtvQpOsqpSolver::solve(
    const LtvQpSparseProblem &problem, const LtvQpSolverSettings &settings,
    const LtvQpWarmStart *warm_start) {
  LtvQpSolveResult result;
  if (solver_ == nullptr || !settings.valid() || !problem.valid() ||
      problem.decision_size != decision_size_ ||
      problem.constraint_rows != constraint_rows_ ||
      !problem.hessian_structure.samePattern(hessian_structure_) ||
      !problem.constraint_structure.samePattern(constraint_structure_)) {
    result.status = solver_ == nullptr ? LtvQpSolverStatus::kBackendUnavailable
                                       : LtvQpSolverStatus::kInvalidProblem;
    return result;
  }
  if (!copyNumericalValues(problem, hessian_values_, constraint_values_, gradient_,
                           lower_, upper_)) {
    result.status = LtvQpSolverStatus::kInvalidProblem;
    return result;
  }
  return solvePrepared(settings, warm_start);
}

bool LtvQpOsqpSolver::copyLtvNumericalValues(const LtvQpProblem &problem) {
  if (!problem.valid || problem.decisionSize() != decision_size_ ||
      !isLtvDimensions(decision_size_, constraint_rows_) ||
      problem.equality_matrix.rows() + problem.inequality_matrix.rows() +
          decision_size_ != constraint_rows_) {
    return false;
  }
  for (int column = 0; column < decision_size_; ++column) {
    const int begin = hessian_structure_.column_offsets[
        static_cast<std::size_t>(column)];
    const int end = hessian_structure_.column_offsets[
        static_cast<std::size_t>(column + 1)];
    for (int offset = begin; offset < end; ++offset) {
      const int row = hessian_structure_.row_indices[
          static_cast<std::size_t>(offset)];
      hessian_values_[static_cast<std::size_t>(offset)] =
          problem.hessian(row, column);
    }
    const int constraint_begin = constraint_structure_.column_offsets[
        static_cast<std::size_t>(column)];
    const int constraint_end = constraint_structure_.column_offsets[
        static_cast<std::size_t>(column + 1)];
    for (int offset = constraint_begin; offset < constraint_end; ++offset) {
      const int row = constraint_structure_.row_indices[
          static_cast<std::size_t>(offset)];
      constraint_values_[static_cast<std::size_t>(offset)] =
          ltvConstraintValue(problem, row, column);
    }
    gradient_[static_cast<std::size_t>(column)] = problem.gradient(column);
  }
  const int equality_rows = problem.equality_matrix.rows();
  const int inequality_rows = problem.inequality_matrix.rows();
  for (int row = 0; row < equality_rows; ++row) {
    lower_[static_cast<std::size_t>(row)] = problem.equality_lower(row);
    upper_[static_cast<std::size_t>(row)] = problem.equality_upper(row);
  }
  for (int row = 0; row < inequality_rows; ++row) {
    const int target = equality_rows + row;
    lower_[static_cast<std::size_t>(target)] = problem.inequality_lower(row);
    upper_[static_cast<std::size_t>(target)] = problem.inequality_upper(row);
  }
  for (int row = 0; row < decision_size_; ++row) {
    const int target = equality_rows + inequality_rows + row;
    lower_[static_cast<std::size_t>(target)] = problem.lower_bound(row);
    upper_[static_cast<std::size_t>(target)] = problem.upper_bound(row);
  }
  return true;
}

LtvQpSolveResult LtvQpOsqpSolver::solveLtvProblem(
    const LtvQpProblem &problem, const LtvQpSolverSettings &settings,
    const LtvQpWarmStart *warm_start) {
  LtvQpSolveResult result;
  if (solver_ == nullptr || !settings.valid() || !copyLtvNumericalValues(problem)) {
    result.status = solver_ == nullptr ? LtvQpSolverStatus::kBackendUnavailable
                                       : LtvQpSolverStatus::kInvalidProblem;
    return result;
  }
  return solvePrepared(settings, warm_start);
}

LtvQpSolveResult LtvQpOsqpSolver::solvePrepared(
    const LtvQpSolverSettings &settings, const LtvQpWarmStart *warm_start) {
  LtvQpSolveResult result;
  if (solver_->settings != nullptr) {
    OSQPSettings updated = *solver_->settings;
    updated.max_iter = static_cast<OSQPInt>(settings.max_iterations);
    updated.time_limit = settings.time_limit_ms / 1000.0;
    updated.eps_abs = std::min(settings.max_primal_residual,
                               settings.max_dual_residual);
    updated.eps_rel = 1e-6;
    if (osqp_update_settings(solver_.get(), &updated) != 0) {
      result.status = LtvQpSolverStatus::kNumericalFailure;
      return result;
    }
  }
  if (osqp_update_data_vec(solver_.get(), gradient_.data(), lower_.data(),
                           upper_.data()) != 0 ||
      osqp_update_data_mat(solver_.get(), hessian_values_.data(), nullptr,
                           static_cast<OSQPInt>(hessian_values_.size()),
                           constraint_values_.data(), nullptr,
                           static_cast<OSQPInt>(constraint_values_.size())) != 0) {
    result.status = LtvQpSolverStatus::kNumericalFailure;
    return result;
  }
  if (warm_start != nullptr &&
      warm_start->validFor(decision_size_, constraint_rows_)) {
    result.warm_start_used =
        osqp_warm_start(solver_.get(), warm_start->primal.data(),
                        warm_start->dual.data()) == 0;
  }
  const OSQPInt solve_result = osqp_solve(solver_.get());
  if (solver_->info == nullptr) {
    result.status = solve_result == 0 ? LtvQpSolverStatus::kNumericalFailure
                                      : LtvQpSolverStatus::kBackendUnavailable;
    return result;
  }
  const OSQPInfo &info = *solver_->info;
  result.status = mapStatus(info.status_val);
  if (solve_result != 0 && result.status == LtvQpSolverStatus::kSolved) {
    result.status = LtvQpSolverStatus::kNumericalFailure;
  }
  result.iterations = static_cast<int>(info.iter);
  result.solve_time_ms = 1000.0 * static_cast<double>(info.solve_time);
  result.primal_residual = static_cast<double>(info.prim_res);
  result.dual_residual = static_cast<double>(info.dual_res);
  result.slack_maximum = 0.0;
  result.hard_constraint_maximum_violation = 0.0;
  if (solver_->solution != nullptr && solver_->solution->x != nullptr &&
      solver_->solution->y != nullptr) {
    result.primal_solution = Eigen::VectorXd::Zero(decision_size_);
    result.dual_solution = Eigen::VectorXd::Zero(constraint_rows_);
    for (int index = 0; index < decision_size_; ++index) {
      result.primal_solution(index) = solver_->solution->x[index];
    }
    for (int index = 0; index < constraint_rows_; ++index) {
      result.dual_solution(index) = solver_->solution->y[index];
    }
  }
  return result;
}

}  // namespace ats_swerve_mpc
