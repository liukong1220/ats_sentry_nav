// Copyright 2026

#ifndef ATS_SWERVE_MPC__LTV_QP_SOLVER_HPP_
#define ATS_SWERVE_MPC__LTV_QP_SOLVER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/ltv_qp_problem.hpp"

namespace ats_swerve_mpc {

/**
 * @brief Backend-neutral QP terminal states.
 *
 * Only kSolved is eligible for command candidate admission.  In particular,
 * kSolvedInaccurate is preserved for diagnostics but is not a permissive
 * command status.
 */
enum class LtvQpSolverStatus {
  kSolved,
  kSolvedInaccurate,
  kMaxIterations,
  kTimeLimit,
  kPrimalInfeasible,
  kDualInfeasible,
  kNumericalFailure,
  kInvalidProblem,
  kBackendUnavailable,
};

const char *ltvQpSolverStatusName(LtvQpSolverStatus status);

/**
 * @brief Immutable compressed-column sparsity pattern for a QP matrix.
 *
 * The row indices and column offsets are part of the backend contract.  A
 * concrete solver may update only values after setup; a changed pattern must
 * be rejected and trigger a new explicit setup, never an implicit timer-path
 * allocation.
 */
struct LtvQpSparseStructure {
  int rows = 0;
  int columns = 0;
  std::vector<int> column_offsets;
  std::vector<int> row_indices;

  bool valid(std::string *error = nullptr) const;
  bool samePattern(const LtvQpSparseStructure &other) const;
};

/**
 * @brief Fixed-structure sparse QP input for a future concrete backend.
 *
 * Values use the CSC pattern described above.  The stacked constraint matrix
 * represents lower <= A * z <= upper.  This interface deliberately does not
 * select or implement a solver.
 */
struct LtvQpSparseProblem {
  int decision_size = 0;
  int constraint_rows = 0;
  LtvQpSparseStructure hessian_structure;
  std::vector<double> hessian_values;
  LtvQpSparseStructure constraint_structure;
  std::vector<double> constraint_values;
  Eigen::VectorXd gradient;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;

  bool valid(std::string *error = nullptr) const;
};

struct LtvQpWarmStart {
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;

  bool validFor(int decision_size, int constraint_rows) const;
};

struct LtvQpSolverSettings {
  int max_iterations = 0;
  double time_limit_ms = 0.0;
  double max_primal_residual = 0.0;
  double max_dual_residual = 0.0;
  double max_tracking_slack = 0.0;
  double max_hard_constraint_violation = 0.0;

  bool valid() const;
};

/**
 * @brief Mandatory solver telemetry and primal candidate payload.
 */
struct LtvQpSolveResult {
  LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
  int iterations = 0;
  double solve_time_ms = 0.0;
  double primal_residual = 0.0;
  double dual_residual = 0.0;
  double slack_maximum = 0.0;
  double hard_constraint_maximum_violation = 0.0;
  bool warm_start_used = false;
  Eigen::VectorXd primal_solution;
  Eigen::VectorXd dual_solution;
  std::vector<double> tracking_slacks;
};

/**
 * @brief Interface that a reviewed C++ QP backend must implement.
 */
class LtvQpSolver {
public:
  virtual ~LtvQpSolver() = default;

  virtual const char *backendName() const = 0;
  virtual bool supportsFixedSparsity() const = 0;
  virtual bool supportsWarmStart() const = 0;
  virtual void resetWarmStart() = 0;
  virtual LtvQpSolveResult solve(const LtvQpSparseProblem &problem,
                                 const LtvQpSolverSettings &settings,
                                 const LtvQpWarmStart *warm_start) = 0;
};

/**
 * @brief External hard gates that cannot be relaxed by tracking slack.
 */
struct LtvQpCandidateSafety {
  bool inputs_healthy = false;
  bool emergency_stop_active = true;
  bool collision_free = false;
};

/**
 * @brief Candidate admission result calculated independently of the backend.
 */
struct LtvQpCandidateAudit {
  bool feasible = false;
  std::string rejection_reason;
  double actual_slack_maximum = 0.0;
  double actual_hard_constraint_maximum_violation = 0.0;
  int steering_rate_constraints_checked = 0;
  int steering_rate_constraints_skipped_by_zero_speed_guard = 0;
};

/**
 * @brief Verifies a QP primal candidate against the real ATS hard limits.
 *
 * This validator is intentionally solver-independent.  It reconstructs the
 * body command from the LTV decision offsets, then evaluates the actual four
 * wheel velocity vectors.  Directional steering-rate checks are applied only
 * while ZeroSpeedGuard says both vectors have a defined direction; vector
 * increment constraints remain hard in every speed regime.
 */
class LtvQpCandidateValidator {
public:
  static LtvQpCandidateAudit validate(
      const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config,
      const LtvQpSolverSettings &settings,
      const LtvQpCandidateSafety &safety,
      const LtvQpSolveResult &result);
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__LTV_QP_SOLVER_HPP_
