// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_
#define ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <osqp.h>

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

namespace ats_swerve_mpc {

/** @brief 从已填充的固定稀疏 LTV-QP 生成独立 CSC 快照（供审计/离线检查）。 */
LtvQpSparseProblem makeLtvQpSparseProblem(const LtvQpProblem &problem);

/**
 * @brief OSQP v1.0.0 零分配适配器：一次 setup 锁定 CSC 结构与约束分类，周期内只更新非零数值。
 */
class LtvQpOsqpSolver final : public LtvQpSolver {
public:
  LtvQpOsqpSolver(int decision_size, int constraint_rows,
                  const LtvQpSolverSettings &setup_settings);
  LtvQpOsqpSolver(const LtvQpDimensions &dimensions,
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
  /** @brief 直接消费预分配稀疏 LtvQpProblem 的 CSC 数组，消除稠密拷贝与冗余 settings 更新。 */
  LtvQpSolveResult solveLtvProblem(const LtvQpProblem &problem,
                                   const LtvQpSolverSettings &settings,
                                   const LtvQpWarmStart *warm_start);

  bool initialized() const { return solver_ != nullptr; }
  std::size_t setupCount() const { return setup_count_; }
  std::size_t numericUpdateCallCount() const { return numeric_update_call_count_; }
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
                                  std::vector<OSQPFloat> &hessian_values,
                                  std::vector<OSQPFloat> &constraint_values,
                                  std::vector<OSQPFloat> &gradient,
                                  std::vector<OSQPFloat> &lower,
                                  std::vector<OSQPFloat> &upper);
  bool copyLtvNumericalValues(const LtvQpProblem &problem);
  LtvQpSolveResult solvePrepared(
      int horizon, const LtvQpProblem *ltv_problem,
      const LtvQpSolverSettings &settings, const LtvQpWarmStart *warm_start,
      std::chrono::steady_clock::time_point complete_phase_start);
  static LtvQpSolverStatus mapStatus(OSQPInt status);

  int decision_size_ = 0;
  int constraint_rows_ = 0;
  int ltv_horizon_ = -1;
  LtvQpSparseStructure hessian_structure_;
  LtvQpSparseStructure constraint_structure_;
  std::vector<OSQPInt> hessian_column_offsets_;
  std::vector<OSQPInt> hessian_row_indices_;
  std::vector<OSQPInt> constraint_column_offsets_;
  std::vector<OSQPInt> constraint_row_indices_;
  std::vector<OSQPFloat> hessian_values_;
  std::vector<OSQPFloat> constraint_values_;
  std::vector<OSQPFloat> gradient_;
  std::vector<OSQPFloat> lower_;
  std::vector<OSQPFloat> upper_;
  std::vector<std::uint8_t> oct_pruned_;
  bool any_oct_pruned_ = false;
  std::vector<OSQPFloat> warm_primal_buf_;
  std::vector<OSQPFloat> warm_dual_buf_;
  LtvQpSolverSettings applied_settings_{};
  bool has_applied_settings_ = false;
  struct OsqpDeleter {
    void operator()(OSQPSolver *solver) const;
  };
  std::unique_ptr<OSQPSolver, OsqpDeleter> solver_;
  std::size_t setup_count_ = 0;
  std::size_t numeric_update_call_count_ = 0;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_
