// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_
#define ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include <osqp.h>

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

namespace ats_swerve_mpc {

/**
 * @brief 将稠密、后端无关的 LTV-QP 转换为固定 CSC 布局。
 * @details 行顺序固定为等式、一般不等式、决策变量 identity bounds；OSQP 所需的 Hessian
 *          仅存上三角。该布局一旦 setup 后只能更新数值，不能在 timer 中重排。
 */
LtvQpSparseProblem makeLtvQpSparseProblem(const LtvQpProblem &problem);

/**
 * @brief OSQP v1.0.0 适配器：一次 setup，后续仅更新固定 CSC 数值。
 * @details 构造函数分配固定 pattern 并调用 osqp_setup()；solve() 会拒绝任意 pattern 漂移，
 *          不得隐式重建 workspace 或在控制 timer 内改变稀疏结构。
 */
class LtvQpOsqpSolver final : public LtvQpSolver {
public:
  /**
   * @brief 预分配固定 CSC 数组并执行唯一一次 osqp_setup。
   * @details decision_size/constraint_rows 随 horizon 固定；后续周期只允许更新数值。
   */
  LtvQpOsqpSolver(int decision_size, int constraint_rows,
                  const LtvQpSolverSettings &setup_settings);
  /**
   * @brief 使用 checked LTV dimensions 创建生产 OSQP workspace。
   * @details 无效 dimensions 直接保持 backend unavailable；节点必须在 controller/workspace
   *          创建前拒绝启动，避免再次独立计算 decision/constraint rows。
   */
  LtvQpOsqpSolver(const LtvQpDimensions &dimensions,
                  const LtvQpSolverSettings &setup_settings);
  /** @brief 释放 OSQP workspace；该析构不承担任何控制发布责任。 */
  ~LtvQpOsqpSolver() override;

  LtvQpOsqpSolver(const LtvQpOsqpSolver &) = delete;
  LtvQpOsqpSolver &operator=(const LtvQpOsqpSolver &) = delete;

  /** @brief 返回锁定版本，避免诊断把未知系统库误认为批准后端。 */
  const char *backendName() const override { return "osqp-1.0.0"; }
  /** @brief OSQP 数据更新保持首次 setup 的 CSC 行列结构。 */
  bool supportsFixedSparsity() const override { return true; }
  /** @brief OSQP C API 支持显式 primal/dual warm-start。 */
  bool supportsWarmStart() const override { return true; }
  /** @brief 请求 OSQP 清除内部迭代初值，并使 node 丢弃对应缓存。 */
  void resetWarmStart() override;
  /** @brief 求解任意已固定 CSC 的输入；pattern 不一致时返回 invalid_problem。 */
  LtvQpSolveResult solve(const LtvQpSparseProblem &problem,
                         const LtvQpSolverSettings &settings,
                         const LtvQpWarmStart *warm_start) override;
  /** @brief 将预分配的 ATS LTV dense buffer 映射到固定 CSC 后求解。 */
  LtvQpSolveResult solveLtvProblem(const LtvQpProblem &problem,
                                   const LtvQpSolverSettings &settings,
                                   const LtvQpWarmStart *warm_start);

  /** @brief 返回一次性 setup 是否成功；失败时 qp_shadow 只能诊断并保留 iLQR。 */
  bool initialized() const { return solver_ != nullptr; }
  /** @brief 暴露 setup 次数以锁定“控制 timer 不重建 workspace”的测试契约。 */
  std::size_t setupCount() const { return setup_count_; }
  /** @brief 返回真正调用 OSQP 数值 update 的次数，锁定 invalid input 不触碰 backend。 */
  std::size_t numericUpdateCallCount() const { return numeric_update_call_count_; }
  /** @brief 返回 setup 时冻结的 Hessian CSC pattern。 */
  const LtvQpSparseStructure &hessianStructure() const {
    return hessian_structure_;
  }
  /** @brief 返回 setup 时冻结的约束矩阵 CSC pattern。 */
  const LtvQpSparseStructure &constraintStructure() const {
    return constraint_structure_;
  }

  /** @brief 生成通用上三角 Hessian pattern，仅用于非 LTV 测试输入。 */
  static LtvQpSparseStructure denseUpperPattern(int size);
  /** @brief 生成通用稠密约束 pattern，仅用于非 LTV 测试输入。 */
  static LtvQpSparseStructure densePattern(int rows, int columns);
  /** @brief 为 ATS [delta_x,delta_u] 布局生成稀疏 Hessian pattern。 */
  static LtvQpSparseStructure ltvHessianPattern(int decision_size);
  /** @brief 为动力学、增量和 identity bounds 生成固定约束 pattern。 */
  static LtvQpSparseStructure ltvConstraintPattern(int decision_size,
                                                    int constraint_rows);

private:
  /** @brief 将外部 CSC 数值复制入预分配缓冲；不改动 row/column pattern。 */
  static bool copyNumericalValues(const LtvQpSparseProblem &problem,
                                  std::vector<double> &hessian_values,
                                  std::vector<double> &constraint_values,
                                  std::vector<double> &gradient,
                                  std::vector<double> &lower,
                                  std::vector<double> &upper);
  /** @brief 将 ATS dense LTV 各约束块按固定列序更新进 OSQP 数值缓冲。 */
  bool copyLtvNumericalValues(const LtvQpProblem &problem);
  /** @brief 更新 q/l/u/P/A、可选 warm-start 并采集 OSQP 原始诊断。 */
  LtvQpSolveResult solvePrepared(const LtvQpSolverSettings &settings,
                                 const LtvQpWarmStart *warm_start);
  /** @brief 把 OSQP 终止码映射到 ATS 审计状态；solved_inaccurate 不放行。 */
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
    /** @brief 用 OSQP 官方析构 API 回收 workspace，避免跨 ABI 的 delete。 */
    void operator()(OSQPSolver *solver) const;
  };
  std::unique_ptr<OSQPSolver, OsqpDeleter> solver_;
  std::size_t setup_count_ = 0;
  std::size_t numeric_update_call_count_ = 0;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_OSQP_SOLVER_HPP_
