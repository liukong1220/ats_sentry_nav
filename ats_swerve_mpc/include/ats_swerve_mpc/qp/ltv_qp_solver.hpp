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
 * @details 只有 kSolved 可进入候选准入；尤其 kSolvedInaccurate 只保留给诊断，绝不能
 *          放宽为可执行状态。
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
 * @brief QP 矩阵的不可变压缩列（CSC）稀疏 pattern。
 * @details 行索引和列偏移属于后端契约。具体求解器 setup 后只可更新数值；pattern 变化必须
 *          被拒绝并在控制 timer 外显式 setup，不能触发隐式分配。
 */
struct LtvQpSparseStructure {
  int rows = 0;
  int columns = 0;
  std::vector<int> column_offsets;
  std::vector<int> row_indices;

  /** @brief 校验 CSC 尺寸、列偏移和行序；失败即禁止 backend 数值更新。 */
  bool valid(std::string *error = nullptr) const;
  /** @brief 比较完整行/列 pattern，防止 timer 内隐式重建 solver workspace。 */
  bool samePattern(const LtvQpSparseStructure &other) const;
};

/**
 * @brief 固定结构稀疏 QP 输入，供已准入的具体后端使用。
 * @details 数值使用上述 CSC pattern；叠加约束表达为 lower <= A*z <= upper。
 *          此接口不自行选择或实现求解器，避免绕开后端准入流程。
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
 * @brief 后端必须回填的求解遥测与 primal candidate 数据。
 * @details 包含状态、迭代、时间、原始/对偶残差、slack、hard violation 及 primal/dual，
 *          以便 shadow 在发布前完全独立复核。
 */
struct LtvQpSolveResult {
  LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
  int iterations = 0;
  double solve_time_ms = 0.0;
  double update_time_ms = 0.0;
  // OSQPInfo 计时之外的完整 C API 墙钟，用于区分 backend reported time 和 adapter 开销。
  double wall_update_time_ms = 0.0;
  double wall_solve_time_ms = 0.0;
  // 从 settings update 开始到 osqp_solve 返回的完整 backend phase 墙钟。
  double wall_qp_phase_time_ms = 0.0;
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
 * @brief 经审计 C++ QP 后端必须实现的统一接口。
 * @details 接口只接受固定 pattern 的数值更新与显式 warm-start，不定义任何 ROS 发布或
 *          控制所有权。
 */
class LtvQpSolver {
public:
  virtual ~LtvQpSolver() = default;

  /** @brief 返回带版本的后端名称，用于审计日志。 */
  virtual const char *backendName() const = 0;
  /** @brief 声明后端是否能在一次 setup 后保持 CSC pattern 不变。 */
  virtual bool supportsFixedSparsity() const = 0;
  /** @brief 声明后端是否同时支持 primal/dual warm-start。 */
  virtual bool supportsWarmStart() const = 0;
  /** @brief 清除后端内部 warm-start，避免坏解跨周期污染。 */
  virtual void resetWarmStart() = 0;
  /** @brief 在既定 pattern 内更新数值并求解；不得在该接口隐式重新 setup。 */
  virtual LtvQpSolveResult solve(const LtvQpSparseProblem &problem,
                                 const LtvQpSolverSettings &settings,
                                 const LtvQpWarmStart *warm_start) = 0;
};

/**
 * @brief 不可被 tracking slack 放宽的外部安全硬门。
 * @details 输入、急停、碰撞、定位、reference、lease、gimbal 与地图任一无效时，candidate
 *          必须拒绝；这些布尔量不由 OSQP 成功状态替代。
 */
struct LtvQpCandidateSafety {
  bool inputs_healthy = false;
  bool emergency_stop_active = true;
  bool collision_free = false;
  bool localization_fresh = false;
  bool reference_fresh = false;
  bool execution_lease_valid = false;
  bool gimbal_valid = false;
  // 当前节点尚无独立地图健康 producer；非零 ExecutionCommand generation 仅是身份，
  // 不是新鲜度证据，因此在接入真实 snapshot 前必须维持 false。
  bool map_fresh = false;
};

/** @brief 与后端独立计算的 candidate 准入结果和拒绝原因。 */
struct LtvQpCandidateAudit {
  bool feasible = false;
  std::string rejection_reason;
  double actual_slack_maximum = 0.0;
  double actual_hard_constraint_maximum_violation = 0.0;
  int steering_rate_constraints_checked = 0;
  int steering_rate_constraints_skipped_by_zero_speed_guard = 0;
};

/**
 * @brief 对 OSQP delta-control primal 做非线性重建。
 * @details `states` 由和 iLQR 相同的 current state 与共享 Se2Model 生成；在 qp_shadow
 *          中仅用于诊断，绝不流向 ROS 速度发布者。
 */
struct LtvQpPrimalCandidate {
  bool valid = false;
  std::string validation_error;
  std::vector<Control> controls;
  std::vector<State> states;
};

class LtvQpCandidateReconstructor {
public:
  /** @brief 从 primal 的 delta_u 重建绝对控制并做共享模型非线性 rollout。 */
  static LtvQpPrimalCandidate reconstruct(
      const State &current_state, const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Se2MpcConfig &config, const LtvQpSolveResult &result);
};

/**
 * @brief 按真实 ATS 硬约束独立复核 QP primal candidate。
 * @details 该验证器与具体求解器无关：它从 LTV 决策偏差重建车体控制，再计算真实四轮速度
 *          向量。仅当 ZeroSpeedGuard 确认两向量方向均有效时检查舵角速率；向量增量则在
 *          所有速度区间始终保持硬约束。
 */
class LtvQpCandidateValidator {
public:
  /** @brief 以 nominal+delta_u 独立复核所有数值、后端、四轮和外部硬门。 */
  static LtvQpCandidateAudit validate(
      const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config,
      const LtvQpSolverSettings &settings,
      const LtvQpCandidateSafety &safety,
      const LtvQpSolveResult &result);

  /** @brief 额外要求重建 rollout 与 primal 身份一致，供 qp_shadow 审计调用。 */
  static LtvQpCandidateAudit validate(
      const LtvQpProblem &problem,
      const std::vector<Control> &nominal_controls,
      const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config,
      const LtvQpSolverSettings &settings,
      const LtvQpCandidateSafety &safety,
      const LtvQpSolveResult &result,
      const LtvQpPrimalCandidate &candidate);
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_SOLVER_HPP_
