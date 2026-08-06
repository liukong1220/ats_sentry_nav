// Copyright 2026

#ifndef ATS_SWERVE_MPC__LTV_QP_OSQP_SOLVER_HPP_
#define ATS_SWERVE_MPC__LTV_QP_OSQP_SOLVER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include <osqp.h>

#include "ats_swerve_mpc/ltv_qp_solver.hpp"

namespace ats_swerve_mpc {

/**
 * @brief Convert the dense, solver-neutral LTV problem to a fixed CSC layout.
 *
 * The rows are equality rows, inequality rows, then identity rows for the
 * decision bounds.  P contains its upper triangle, as required by OSQP.
 */
LtvQpSparseProblem makeLtvQpSparseProblem(const LtvQpProblem &problem);

/**
 * @brief OSQP v1.0.0 adapter with one-time setup and numerical-only updates.
 *
 * The constructor allocates the fixed CSC pattern and calls osqp_setup().
 * solve() rejects a changed pattern; it never reconfigures the workspace.
 */
class LtvQpOsqpSolver final : public LtvQpSolver {
public:
  LtvQpOsqpSolver(int decision_size, int constraint_rows,
                  const LtvQpSolverSettings &setup_settings);
  ~LtvQpOsqpSolver() override;

  LtvQpOsqpSolver(const LtvQpOsqpSolver &) = delete;
  LtvQpOsqpSolver &operator=(const LtvQpOsqpSolver &) = delete;

  const char *backendName() const override { return "osqp-1.0.0"; }
  bool supportsFixedSparsity() const override { return true; }
  bool supportsWarmStart() const override { return true; }
  void resetWarmStart() override;
  LtvQpSolveResult solve(const LtvQpSparseProblem &problem,
                         const LtvQpSolverSettings &settings,
                         const LtvQpWarmStart *warm_start) override;
  LtvQpSolveResult solveLtvProblem(const LtvQpProblem &problem,
                                   const LtvQpSolverSettings &settings,
                                   const LtvQpWarmStart *warm_start);

  bool initialized() const { return solver_ != nullptr; }
  std::size_t setupCount() const { return setup_count_; }
  const LtvQpSparseStructure &hessianStructure() const {
    return hessian_structure_;
  }
  const LtvQpSparseStructure &constraintStructure() const {
    return constraint_structure_;
  }

  static LtvQpSparseStructure denseUpperPattern(int size);
  static LtvQpSparseStructure densePattern(int rows, int columns);
  static LtvQpSparseStructure ltvHessianPattern(int decision_size);
  static LtvQpSparseStructure ltvConstraintPattern(int decision_size,
                                                    int constraint_rows);

private:
  static bool copyNumericalValues(const LtvQpSparseProblem &problem,
                                  std::vector<double> &hessian_values,
                                  std::vector<double> &constraint_values,
                                  std::vector<double> &gradient,
                                  std::vector<double> &lower,
                                  std::vector<double> &upper);
  bool copyLtvNumericalValues(const LtvQpProblem &problem);
  LtvQpSolveResult solvePrepared(const LtvQpSolverSettings &settings,
                                 const LtvQpWarmStart *warm_start);
  static LtvQpSolverStatus mapStatus(int status);

  int decision_size_ = 0;
  int constraint_rows_ = 0;
  LtvQpSparseStructure hessian_structure_;
  LtvQpSparseStructure constraint_structure_;
  std::vector<long long> hessian_column_offsets_;
  std::vector<long long> hessian_row_indices_;
  std::vector<long long> constraint_column_offsets_;
  std::vector<long long> constraint_row_indices_;
  std::vector<double> hessian_values_;
  std::vector<double> constraint_values_;
  std::vector<double> gradient_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  struct OsqpDeleter {
    void operator()(OSQPSolver *solver) const;
  };
  std::unique_ptr<OSQPSolver, OsqpDeleter> solver_;
  std::size_t setup_count_ = 0;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__LTV_QP_OSQP_SOLVER_HPP_
