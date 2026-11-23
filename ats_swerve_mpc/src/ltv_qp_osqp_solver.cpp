// Copyright 2026

#include "ats_swerve_mpc/qp/ltv_qp_osqp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

#include <osqp.h>

namespace ats_swerve_mpc {
namespace {

int encodedLtvHorizonForDecisionSize(int decision_size) {
  if (decision_size < 10 || (decision_size - 3) % 7 != 0) {
    return -1;
  }
  return (decision_size - 3) / 7;
}

int ltvHorizonForDecisionSize(int decision_size) {
  const int horizon = encodedLtvHorizonForDecisionSize(decision_size);
  if (horizon <= 0) {
    return -1;
  }
  const auto dimensions = checkedLtvQpDimensions(horizon);
  return dimensions.valid && dimensions.decision_size == decision_size
             ? horizon
             : -1;
}

bool isLtvDimensions(int decision_size, int constraint_rows) {
  const int horizon = ltvHorizonForDecisionSize(decision_size);
  if (horizon <= 0) {
    return false;
  }
  const auto dimensions = checkedLtvQpDimensions(horizon);
  return dimensions.valid && dimensions.decision_size == decision_size &&
         dimensions.constraint_rows == constraint_rows;
}

bool sameRuntimeOsqpSettings(const LtvQpSolverSettings &lhs,
                             const LtvQpSolverSettings &rhs) {
  return lhs.max_iterations == rhs.max_iterations &&
         std::abs(lhs.time_limit_ms - rhs.time_limit_ms) <= 1e-9 &&
         std::abs(lhs.max_primal_residual - rhs.max_primal_residual) <= 1e-12 &&
         std::abs(lhs.max_dual_residual - rhs.max_dual_residual) <= 1e-12;
}

}  // namespace

LtvQpSparseProblem makeLtvQpSparseProblem(const LtvQpProblem &problem) {
  LtvQpSparseProblem sparse;
  if (!problem.valid || !problem.hasExpectedLayout() ||
      !problem.hasOrderedBounds() || !problem.hasFiniteNumerics()) {
    return sparse;
  }
  sparse.decision_size = problem.decisionSize();
  sparse.constraint_rows = problem.constraintRows();
  sparse.hessian_structure = problem.hessian_structure;
  sparse.hessian_values = problem.hessian_values;
  sparse.constraint_structure = problem.constraint_structure;
  sparse.constraint_values = problem.constraint_values;
  sparse.gradient = problem.gradient;
  sparse.lower = problem.lower;
  sparse.upper = problem.upper;
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

LtvQpSparseStructure LtvQpOsqpSolver::ltvHessianPattern(int decision_size) {
  const int horizon = ltvHorizonForDecisionSize(decision_size);
  if (horizon <= 0) {
    return denseUpperPattern(decision_size);
  }
  return LtvQpBuilder::buildHessianPattern(horizon);
}

LtvQpSparseStructure LtvQpOsqpSolver::ltvConstraintPattern(
    int decision_size, int constraint_rows) {
  if (!isLtvDimensions(decision_size, constraint_rows)) {
    return densePattern(constraint_rows, decision_size);
  }
  return LtvQpBuilder::buildConstraintPattern(
      ltvHorizonForDecisionSize(decision_size));
}

void LtvQpOsqpSolver::OsqpDeleter::operator()(OSQPSolver *solver) const {
  if (solver != nullptr) {
    osqp_cleanup(solver);
  }
}

LtvQpOsqpSolver::LtvQpOsqpSolver(int decision_size, int constraint_rows,
                                 const LtvQpSolverSettings &setup_settings)
    : decision_size_(decision_size), constraint_rows_(constraint_rows) {
  if (decision_size_ <= 0 || constraint_rows_ <= 0 || !setup_settings.valid()) {
    return;
  }
  const int encoded_horizon = encodedLtvHorizonForDecisionSize(decision_size_);
  if (encoded_horizon > 0) {
    const auto dimensions = checkedLtvQpDimensions(encoded_horizon);
    if (!dimensions.valid || dimensions.decision_size != decision_size_ ||
        dimensions.constraint_rows != constraint_rows_) {
      return;
    }
    ltv_horizon_ = encoded_horizon;
  }
  hessian_structure_ = ltvHessianPattern(decision_size_);
  constraint_structure_ = ltvConstraintPattern(decision_size_, constraint_rows_);
  hessian_values_.assign(hessian_structure_.row_indices.size(),
                         static_cast<OSQPFloat>(0.0));
  constraint_values_.assign(constraint_structure_.row_indices.size(),
                            static_cast<OSQPFloat>(0.0));
  gradient_.assign(static_cast<std::size_t>(decision_size_),
                   static_cast<OSQPFloat>(0.0));
  lower_.assign(static_cast<std::size_t>(constraint_rows_),
                static_cast<OSQPFloat>(-1.0));
  upper_.assign(static_cast<std::size_t>(constraint_rows_),
                static_cast<OSQPFloat>(1.0));

  // 初始化为正定对角与真实等式/不等式分类（避免首周期 update_rho_vec 重因式分解）
  for (int col = 0; col < decision_size_; ++col) {
    const int end =
        hessian_structure_.column_offsets[static_cast<std::size_t>(col + 1)];
    if (end > 0) {
      hessian_values_[static_cast<std::size_t>(end - 1)] =
          static_cast<OSQPFloat>(2.0);
    }
  }
  if (ltv_horizon_ > 0) {
    const auto dims = checkedLtvQpDimensions(ltv_horizon_);
    for (int r = 0; r < dims.equality_rows; ++r) {
      lower_[static_cast<std::size_t>(r)] = static_cast<OSQPFloat>(0.0);
      upper_[static_cast<std::size_t>(r)] = static_cast<OSQPFloat>(0.0);
    }
    const int esdf_base = dims.equality_rows + dims.body_accel_rows +
                          dims.wheel_octagon_rows + dims.wheel_rate_rows;
    const int slack_bound_base =
        esdf_base + dims.esdf_corridor_rows + 3 * ltv_horizon_;
    for (int s = 0; s < ltv_horizon_; ++s) {
      lower_[static_cast<std::size_t>(esdf_base + s)] =
          static_cast<OSQPFloat>(-1.0);
      upper_[static_cast<std::size_t>(esdf_base + s)] =
          static_cast<OSQPFloat>(OSQP_INFTY);
      lower_[static_cast<std::size_t>(slack_bound_base + s)] =
          static_cast<OSQPFloat>(0.0);
      upper_[static_cast<std::size_t>(slack_bound_base + s)] =
          static_cast<OSQPFloat>(std::max(0.01, setup_settings.max_tracking_slack));
    }
    // Seed diagonal identity entries in A so initial Ruiz scaling sees O(1) norms
    for (int col = 0; col < decision_size_; ++col) {
      const int begin = constraint_structure_.column_offsets[
          static_cast<std::size_t>(col)];
      const int end = constraint_structure_.column_offsets[
          static_cast<std::size_t>(col + 1)];
      if (begin < end) {
        constraint_values_[static_cast<std::size_t>(begin)] =
            static_cast<OSQPFloat>(1.0);
      }
    }
  }

  hessian_column_offsets_.resize(hessian_structure_.column_offsets.size());
  for (std::size_t i = 0; i < hessian_structure_.column_offsets.size(); ++i) {
    hessian_column_offsets_[i] =
        static_cast<OSQPInt>(hessian_structure_.column_offsets[i]);
  }
  hessian_row_indices_.resize(hessian_structure_.row_indices.size());
  for (std::size_t i = 0; i < hessian_structure_.row_indices.size(); ++i) {
    hessian_row_indices_[i] =
        static_cast<OSQPInt>(hessian_structure_.row_indices[i]);
  }
  constraint_column_offsets_.resize(constraint_structure_.column_offsets.size());
  for (std::size_t i = 0; i < constraint_structure_.column_offsets.size();
       ++i) {
    constraint_column_offsets_[i] =
        static_cast<OSQPInt>(constraint_structure_.column_offsets[i]);
  }
  constraint_row_indices_.resize(constraint_structure_.row_indices.size());
  for (std::size_t i = 0; i < constraint_structure_.row_indices.size(); ++i) {
    constraint_row_indices_[i] =
        static_cast<OSQPInt>(constraint_structure_.row_indices[i]);
  }

  OSQPCscMatrix hessian{
      static_cast<OSQPInt>(decision_size_),
      static_cast<OSQPInt>(decision_size_),
      hessian_column_offsets_.data(),
      hessian_row_indices_.data(),
      hessian_values_.data(),
      static_cast<OSQPInt>(hessian_values_.size()),
      -1,
      0};
  OSQPCscMatrix constraints{
      static_cast<OSQPInt>(constraint_rows_),
      static_cast<OSQPInt>(decision_size_),
      constraint_column_offsets_.data(),
      constraint_row_indices_.data(),
      constraint_values_.data(),
      static_cast<OSQPInt>(constraint_values_.size()),
      -1,
      0};
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.profiler_level = 0;
  settings.allocate_solution = 1;
  settings.warm_starting = 1;
  settings.scaling = 10;
  settings.rho = static_cast<OSQPFloat>(1.0);
  settings.alpha = static_cast<OSQPFloat>(1.8);
  settings.adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_DISABLED;
  settings.check_termination = 5;
  settings.check_dualgap = 0;
  settings.max_iter = static_cast<OSQPInt>(setup_settings.max_iterations);
  settings.time_limit =
      static_cast<OSQPFloat>(0.80 * setup_settings.time_limit_ms / 1000.0);
  settings.eps_abs = static_cast<OSQPFloat>(0.5 * std::min(
      setup_settings.max_primal_residual, setup_settings.max_dual_residual));
  settings.eps_rel = static_cast<OSQPFloat>(1e-5);
  settings.polishing = 0;
  OSQPSolver *raw_solver = nullptr;
  const OSQPInt setup_result = osqp_setup(
      &raw_solver, &hessian, gradient_.data(), &constraints, lower_.data(),
      upper_.data(), static_cast<OSQPInt>(constraint_rows_),
      static_cast<OSQPInt>(decision_size_), &settings);
  if (setup_result == 0) {
    solver_.reset(raw_solver);
    setup_count_ = 1;
    applied_settings_ = setup_settings;
    has_applied_settings_ = true;
  }
}

LtvQpOsqpSolver::LtvQpOsqpSolver(const LtvQpDimensions &dimensions,
                                 const LtvQpSolverSettings &setup_settings)
    : LtvQpOsqpSolver(dimensions.valid ? dimensions.decision_size : 0,
                      dimensions.valid ? dimensions.constraint_rows : 0,
                      setup_settings) {}

LtvQpOsqpSolver::~LtvQpOsqpSolver() = default;

void LtvQpOsqpSolver::resetWarmStart() {
  if (solver_ != nullptr) {
    osqp_cold_start(solver_.get());
  }
}

bool LtvQpOsqpSolver::copyNumericalValues(
    const LtvQpSparseProblem &problem, std::vector<OSQPFloat> &hessian_values,
    std::vector<OSQPFloat> &constraint_values, std::vector<OSQPFloat> &gradient,
    std::vector<OSQPFloat> &lower, std::vector<OSQPFloat> &upper) {
  if (!problem.valid() ||
      hessian_values.size() != problem.hessian_values.size() ||
      constraint_values.size() != problem.constraint_values.size() ||
      gradient.size() != static_cast<std::size_t>(problem.decision_size) ||
      lower.size() != static_cast<std::size_t>(problem.constraint_rows) ||
      upper.size() != static_cast<std::size_t>(problem.constraint_rows)) {
    return false;
  }
  for (std::size_t i = 0; i < problem.hessian_values.size(); ++i) {
    hessian_values[i] = static_cast<OSQPFloat>(problem.hessian_values[i]);
  }
  for (std::size_t i = 0; i < problem.constraint_values.size(); ++i) {
    constraint_values[i] = static_cast<OSQPFloat>(problem.constraint_values[i]);
  }
  for (int index = 0; index < problem.gradient.size(); ++index) {
    gradient[static_cast<std::size_t>(index)] =
        static_cast<OSQPFloat>(problem.gradient(index));
  }
  for (int index = 0; index < problem.constraint_rows; ++index) {
    lower[static_cast<std::size_t>(index)] =
        static_cast<OSQPFloat>(problem.lower(index));
    upper[static_cast<std::size_t>(index)] =
        static_cast<OSQPFloat>(problem.upper(index));
  }
  return true;
}

LtvQpSolverStatus LtvQpOsqpSolver::mapStatus(OSQPInt status) {
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
  if (solver_ == nullptr || !settings.valid() ||
      problem.decision_size != decision_size_ ||
      problem.constraint_rows != constraint_rows_ ||
      !problem.hessian_structure.samePattern(hessian_structure_) ||
      !problem.constraint_structure.samePattern(constraint_structure_)) {
    result.status = solver_ == nullptr ? LtvQpSolverStatus::kBackendUnavailable
                                       : LtvQpSolverStatus::kInvalidProblem;
    return result;
  }
  const auto complete_phase_start = std::chrono::steady_clock::now();
  if (!copyNumericalValues(problem, hessian_values_, constraint_values_,
                           gradient_, lower_, upper_)) {
    result.status = LtvQpSolverStatus::kInvalidProblem;
    result.wall_complete_qp_phase_time_ms =
        1000.0 * std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - complete_phase_start)
                     .count();
    return result;
  }
  return solvePrepared(ltv_horizon_, settings, warm_start, complete_phase_start);
}

bool LtvQpOsqpSolver::copyLtvNumericalValues(const LtvQpProblem &problem) {
  if (!problem.valid || !problem.hasExpectedLayout() ||
      !problem.hasOrderedBounds() || !problem.hasFiniteNumerics() ||
      problem.decisionSize() != decision_size_ ||
      problem.constraintRows() != constraint_rows_ ||
      hessian_values_.size() != problem.hessian_values.size() ||
      constraint_values_.size() != problem.constraint_values.size()) {
    return false;
  }
  if constexpr (sizeof(OSQPFloat) == sizeof(double)) {
    std::memcpy(hessian_values_.data(), problem.hessian_values.data(),
                hessian_values_.size() * sizeof(double));
    std::memcpy(constraint_values_.data(), problem.constraint_values.data(),
                constraint_values_.size() * sizeof(double));
    std::memcpy(gradient_.data(), problem.gradient.data(),
                gradient_.size() * sizeof(double));
    std::memcpy(lower_.data(), problem.lower.data(),
                lower_.size() * sizeof(double));
    std::memcpy(upper_.data(), problem.upper.data(),
                upper_.size() * sizeof(double));
    for (int i = 0; i < constraint_rows_; ++i) {
      if (upper_[static_cast<std::size_t>(i)] >= static_cast<OSQPFloat>(0.99e4)) {
        upper_[static_cast<std::size_t>(i)] = static_cast<OSQPFloat>(OSQP_INFTY);
      }
    }
  } else {
    for (std::size_t i = 0; i < hessian_values_.size(); ++i) {
      hessian_values_[i] = static_cast<OSQPFloat>(problem.hessian_values[i]);
    }
    for (std::size_t i = 0; i < constraint_values_.size(); ++i) {
      constraint_values_[i] =
          static_cast<OSQPFloat>(problem.constraint_values[i]);
    }
    for (int i = 0; i < decision_size_; ++i) {
      gradient_[static_cast<std::size_t>(i)] =
          static_cast<OSQPFloat>(problem.gradient(i));
    }
    for (int i = 0; i < constraint_rows_; ++i) {
      lower_[static_cast<std::size_t>(i)] =
          static_cast<OSQPFloat>(problem.lower(i));
      const double u_val = problem.upper(i);
      upper_[static_cast<std::size_t>(i)] = u_val >= 0.99e4
          ? static_cast<OSQPFloat>(OSQP_INFTY)
          : static_cast<OSQPFloat>(u_val);
    }
  }
  return true;
}

LtvQpSolveResult LtvQpOsqpSolver::solveLtvProblem(
    const LtvQpProblem &problem, const LtvQpSolverSettings &settings,
    const LtvQpWarmStart *warm_start) {
  LtvQpSolveResult result;
  if (solver_ == nullptr || !settings.valid()) {
    result.status = solver_ == nullptr ? LtvQpSolverStatus::kBackendUnavailable
                                       : LtvQpSolverStatus::kInvalidProblem;
    return result;
  }
  const auto complete_phase_start = std::chrono::steady_clock::now();
  if (!copyLtvNumericalValues(problem)) {
    result.status = LtvQpSolverStatus::kInvalidProblem;
    result.wall_complete_qp_phase_time_ms =
        1000.0 * std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - complete_phase_start)
                     .count();
    return result;
  }
  return solvePrepared(problem.horizon, settings, warm_start,
                       complete_phase_start);
}

LtvQpSolveResult LtvQpOsqpSolver::solvePrepared(
    int horizon, const LtvQpSolverSettings &settings,
    const LtvQpWarmStart *warm_start,
    std::chrono::steady_clock::time_point complete_phase_start) {
  LtvQpSolveResult result;
  const auto backend_phase_start = std::chrono::steady_clock::now();
  const auto elapsedBackendPhaseMs = [&backend_phase_start]() {
    return 1000.0 * std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - backend_phase_start)
                        .count();
  };
  const auto elapsedCompletePhaseMs = [&complete_phase_start]() {
    return 1000.0 *
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         complete_phase_start)
               .count();
  };
  const auto finalizePhaseTimes = [&result, &elapsedBackendPhaseMs,
                                   &elapsedCompletePhaseMs]() {
    result.wall_qp_phase_time_ms = elapsedBackendPhaseMs();
    result.wall_complete_qp_phase_time_ms = elapsedCompletePhaseMs();
  };

  // 仅当运行时配置真有变化时才调用 osqp_update_settings，彻底消除每周期冗余调用
  if (solver_->settings != nullptr &&
      (!has_applied_settings_ ||
       !sameRuntimeOsqpSettings(applied_settings_, settings))) {
    OSQPSettings updated = *solver_->settings;
    updated.max_iter = static_cast<OSQPInt>(settings.max_iterations);
    updated.time_limit =
        static_cast<OSQPFloat>(0.80 * settings.time_limit_ms / 1000.0);
    updated.eps_abs = static_cast<OSQPFloat>(
        0.5 * std::min(settings.max_primal_residual, settings.max_dual_residual));
    updated.eps_rel = static_cast<OSQPFloat>(1e-5);
    updated.check_termination = 5;
    updated.check_dualgap = 0;
    if (osqp_update_settings(solver_.get(), &updated) != 0) {
      result.wall_update_time_ms = elapsedBackendPhaseMs();
      finalizePhaseTimes();
      result.status = LtvQpSolverStatus::kNumericalFailure;
      return result;
    }
    applied_settings_ = settings;
    has_applied_settings_ = true;
  }

  const auto update_start = std::chrono::steady_clock::now();
  ++numeric_update_call_count_;
  // Update q first so scale_data inside osqp_update_data_mat sees the new cost
  // norm, then update P/A (recomputing Ruiz scaling & KKT), then scale l/u with
  // the new row factors E.
  if (osqp_update_data_vec(solver_.get(), gradient_.data(), nullptr, nullptr) !=
          0 ||
      osqp_update_data_mat(
          solver_.get(), hessian_values_.data(), nullptr,
          static_cast<OSQPInt>(hessian_values_.size()),
          constraint_values_.data(), nullptr,
          static_cast<OSQPInt>(constraint_values_.size())) != 0 ||
      osqp_update_data_vec(solver_.get(), nullptr, lower_.data(),
                           upper_.data()) != 0) {
    result.wall_update_time_ms =
        1000.0 * std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - update_start)
                     .count();
    finalizePhaseTimes();
    result.status = LtvQpSolverStatus::kNumericalFailure;
    return result;
  }
  if (warm_start != nullptr &&
      warm_start->validFor(decision_size_, constraint_rows_)) {
    if constexpr (sizeof(OSQPFloat) == sizeof(double)) {
      result.warm_start_used =
          osqp_warm_start(solver_.get(), warm_start->primal.data(),
                          warm_start->dual.data()) == 0;
    } else {
      std::vector<OSQPFloat> px(static_cast<std::size_t>(decision_size_));
      std::vector<OSQPFloat> dy(static_cast<std::size_t>(constraint_rows_));
      for (int i = 0; i < decision_size_; ++i) {
        px[static_cast<std::size_t>(i)] =
            static_cast<OSQPFloat>(warm_start->primal(i));
      }
      for (int i = 0; i < constraint_rows_; ++i) {
        dy[static_cast<std::size_t>(i)] =
            static_cast<OSQPFloat>(warm_start->dual(i));
      }
      result.warm_start_used =
          osqp_warm_start(solver_.get(), px.data(), dy.data()) == 0;
    }
  }
  result.wall_update_time_ms =
      1000.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             update_start)
                   .count();
  const auto solve_start = std::chrono::steady_clock::now();
  const OSQPInt solve_result = osqp_solve(solver_.get());
  result.wall_solve_time_ms =
      1000.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             solve_start)
                   .count();
  finalizePhaseTimes();
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
  result.update_time_ms = 1000.0 * static_cast<double>(info.update_time);
  result.primal_residual = static_cast<double>(info.prim_res);
  result.dual_residual = static_cast<double>(info.dual_res);
  result.rho = solver_->settings != nullptr
                   ? static_cast<double>(solver_->settings->rho)
                   : 0.0;
  result.rho_updates = static_cast<int>(info.rho_updates);
  result.slack_maximum = 0.0;
  result.hard_constraint_maximum_violation = 0.0;
  if (solver_->solution != nullptr && solver_->solution->x != nullptr &&
      solver_->solution->y != nullptr) {
    result.primal_solution = Eigen::VectorXd::Zero(decision_size_);
    result.dual_solution = Eigen::VectorXd::Zero(constraint_rows_);
    for (int index = 0; index < decision_size_; ++index) {
      result.primal_solution(index) =
          static_cast<double>(solver_->solution->x[index]);
    }
    for (int index = 0; index < constraint_rows_; ++index) {
      result.dual_solution(index) =
          static_cast<double>(solver_->solution->y[index]);
    }
    if (horizon > 0) {
      const auto dims = checkedLtvQpDimensions(horizon);
      if (dims.valid && dims.decision_size == decision_size_) {
        const int slack_start = dims.state_vars + dims.control_vars;
        result.tracking_slacks.resize(static_cast<std::size_t>(horizon), 0.0);
        for (int s = 0; s < horizon; ++s) {
          const double sv = result.primal_solution(slack_start + s);
          result.tracking_slacks[static_cast<std::size_t>(s)] = sv;
          if (std::isfinite(sv) && sv > result.slack_maximum) {
            result.slack_maximum = sv;
          }
        }
      }
    }
  }
  return result;
}

}  // namespace ats_swerve_mpc
