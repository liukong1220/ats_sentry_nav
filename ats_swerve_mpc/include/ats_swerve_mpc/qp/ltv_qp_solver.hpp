// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__LTV_QP_SOLVER_HPP_
#define ATS_SWERVE_MPC__QP__LTV_QP_SOLVER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/qp/ltv_qp_problem.hpp"

namespace ats_swerve_mpc {

/**
 * @brief 后端无关的 QP 终止状态。
 * @details kSolved 及通过独立残差/非线性硬约束复核的 kSolvedInaccurate 可进入候选准入；
 *          其余未收敛或不可行状态一律拒绝。
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

/** @brief 将统一状态码转为稳定诊断字符串；不用于放宽准入判断。 */
const char *ltvQpSolverStatusName(LtvQpSolverStatus status);

/**
 * @brief 固定结构稀疏 QP 输入，供已准入的具体后端使用。
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

  /** @brief 校验固定 CSC 数值、向量维度及双边约束上下界。 */
  bool valid(std::string *error = nullptr) const;
};

struct LtvQpWarmStart {
  Eigen::VectorXd primal;
  Eigen::VectorXd dual;

  /** @brief 检查 primal/dual warm-start 是否与当前固定布局完全同维且有限。 */
  bool validFor(int decision_size, int constraint_rows) const;
};

struct LtvQpSolverSettings {
  int max_iterations = 0;
  double time_limit_ms = 0.0;
  double max_primal_residual = 0.0;
  double max_dual_residual = 0.0;
  double max_tracking_slack = 0.0;
  double max_hard_constraint_violation = 0.0;

  /** @brief 校验最大迭代、deadline、残差、slack 和 hard violation 门限。 */
  bool valid() const;
};

/**
 * @brief 后端回填的求解遥测与 primal/dual 解数据。
 */
struct LtvQpSolveResult {
  LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
  int iterations = 0;
  double solve_time_ms = 0.0;
  double update_time_ms = 0.0;
  double wall_update_time_ms = 0.0;
  double wall_solve_time_ms = 0.0;
  double wall_qp_phase_time_ms = 0.0;
  double wall_complete_qp_phase_time_ms = 0.0;
  double primal_residual = 0.0;
  double dual_residual = 0.0;
  double rho = 0.0;
  int rho_updates = 0;
  double slack_maximum = 0.0;
  double hard_constraint_maximum_violation = 0.0;
  bool warm_start_used = false;
  Eigen::VectorXd primal_solution;
  Eigen::VectorXd dual_solution;
  std::vector<double> tracking_slacks;
};

/**
 * @brief 经审计 C++ QP 后端统一接口。
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
 * @brief 不可被 ESDF 走廊 slack 放宽的外部安全硬门。
 */
struct LtvQpCandidateSafety {
  bool inputs_healthy = false;
  bool emergency_stop_active = true;
  bool collision_free = false;
  bool localization_fresh = false;
  bool reference_fresh = false;
  bool execution_lease_valid = false;
  bool gimbal_valid = false;
  bool map_fresh = false;
};

/** @brief 与后端独立计算的 candidate 准入结果和拒绝原因。 */
struct LtvQpCandidateAudit {
  bool feasible = false;
  std::string rejection_reason;
  double actual_slack_maximum = 0.0;
  double actual_hard_constraint_maximum_violation = 0.0;
  double max_wheel_speed_violation = 0.0;
  int steering_rate_constraints_checked = 0;
  int steering_rate_constraints_skipped_by_zero_speed_guard = 0;
};

/**
 * @brief 对 OSQP delta-control primal 做共享模型非线性重建。
 */
struct LtvQpPrimalCandidate {
  bool valid = false;
  std::string validation_error;
  std::vector<Control> controls;
  std::vector<State> states;
};

class LtvQpCandidateReconstructor {
public:
  /** @brief 从 primal 的 delta_u 重建绝对控制并做共享模型（含一阶滞后）非线性 rollout。 */
  static LtvQpPrimalCandidate reconstruct(
      const State &current_state, const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Se2MpcConfig &config, const LtvQpSolveResult &result,
      const Control &last_control = Control::Zero(),
      double first_step_dt = -1.0);
};

/**
 * @brief 按真实 ATS 硬约束（旋转系速度搬运 + 滚速标量增量 + 舵角速率解耦）独立复核 QP candidate。
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
      const LtvQpSolveResult &result,
      double first_step_dt = -1.0);

  static LtvQpCandidateAudit validate(
      const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config,
      const LtvQpSolverSettings &settings,
      const LtvQpCandidateSafety &safety,
      const LtvQpSolveResult &result,
      const LtvQpPrimalCandidate &candidate,
      double first_step_dt = -1.0);
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_SOLVER_HPP_
