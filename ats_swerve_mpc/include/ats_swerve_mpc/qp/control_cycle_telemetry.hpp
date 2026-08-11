// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__CONTROL_CYCLE_TELEMETRY_HPP_
#define ATS_SWERVE_MPC__QP__CONTROL_CYCLE_TELEMETRY_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "ats_swerve_mpc/qp/ltv_qp_solver.hpp"

namespace ats_swerve_mpc {

/** @brief 控制 callback 内各阶段的单调时钟测量项，单位统一为毫秒。 */
enum class ControlCycleTimingStage : std::size_t {
  kStateTrajectorySnapshot = 0,
  kReferenceExtraction,
  kIlqrSolve,
  // 以下五项是 kIlqrSolve 的内部拆分，只用于实时性归因，不是新增的串行开销。
  // kIlqrJacobian 内含于 kIlqrBackwardPass，累加时不得重复计入。
  kIlqrWarmStart,
  kIlqrRollout,
  kIlqrBackwardPass,
  kIlqrJacobian,
  kIlqrLineSearch,
  kIlqrCommandPublish,
  kQpProblemBuild,
  kOsqpNumericUpdate,
  kQpBackendPhase,
  kOsqpSolve,
  kPrimalReconstructionRollout,
  kCandidateHardCheck,
  kTelemetryRingWrite,
  kPercentileAggregation,
  kLoggingPublish,
  kFullCallback,
  kTimerInterarrival,
  kCount,
};

/** @brief 返回阶段数组索引；只用于固定容量 telemetry 内部布局。 */
constexpr std::size_t controlCycleTimingStageIndex(
    ControlCycleTimingStage stage) {
  return static_cast<std::size_t>(stage);
}

constexpr std::size_t kControlCycleTimingStageCount =
    controlCycleTimingStageIndex(ControlCycleTimingStage::kCount);

/** @brief 统一的超期根因分类；每项计数均为无符号 64 位饱和计数。 */
enum class ControlCycleDeadlineCause : std::size_t {
  kOsqpTimeLimitStatus = 0,
  kOsqpSolveBudgetOverrun,
  kCallbackPeriodOverrun,
  kIlqrSolveBudgetOverrun,
  kQpProblemBuildBudgetOverrun,
  kQpUpdateBudgetOverrun,
  kQpPhaseBudgetOverrun,
  kQpCandidateAuditBudgetOverrun,
  kTelemetryAggregationBudgetOverrun,
  kLoggingPublishBudgetOverrun,
  kTimerInterarrivalOverrun,
  kCount,
};

/** @brief 返回 deadline 根因数组索引；保持 root-cause 序列化顺序稳定。 */
constexpr std::size_t controlCycleDeadlineCauseIndex(
    ControlCycleDeadlineCause cause) {
  return static_cast<std::size_t>(cause);
}

constexpr std::size_t kControlCycleDeadlineCauseCount =
    controlCycleDeadlineCauseIndex(ControlCycleDeadlineCause::kCount);

/** @brief 固定容量窗口中一个阶段的分布统计，单位毫秒。 */
struct ControlCycleTimingDistribution {
  std::size_t count = 0;
  double p50_ms = 0.0;
  double p95_ms = 0.0;
  double p99_ms = 0.0;
  double max_ms = 0.0;
};

/** @brief 一个控制周期的定长、无路径内容性能与 QP 审计记录。 */
struct ControlCycleTelemetrySample {
  std::uint64_t cycle_sequence = 0;
  bool qp_shadow_attempted = false;
  std::uint64_t snapshot_identity_digest = 0;
  bool same_snapshot_identity = false;
  // 下列字段共同定义采样窗口的 Execute/reference/map 身份；它们仅用于诊断配对，
  // 不能授予 QP 发布权限或放宽任一安全 gate。
  std::uint64_t manager_incarnation = 0;
  std::uint64_t command_sequence = 0;
  std::uint64_t goal_id = 0;
  std::uint64_t localization_epoch = 0;
  std::uint64_t map_generation = 0;
  std::uint64_t map_publication_sequence = 0;
  std::int64_t reference_stamp_ns = 0;
  std::int64_t reference_deadline_ns = 0;
  std::array<char, 64> reference_frame{{}};
  bool execution_lease_valid = false;
  bool reference_fresh = false;

  LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
  int iterations = 0;
  bool warm_start_used = false;
  double osqp_reported_update_ms = 0.0;
  double osqp_reported_solve_ms = 0.0;
  // OSQPInfo 的内部计时与 C API 墙钟分别保留，避免把前者误作完整 adapter 开销。
  double osqp_wall_update_ms = 0.0;
  double osqp_wall_solve_ms = 0.0;
  double osqp_wall_qp_phase_ms = 0.0;
  double primal_residual = 0.0;
  double dual_residual = 0.0;
  double hard_constraint_margin = 0.0;
  double slack_maximum = 0.0;
  std::array<double, 3> qp_first_control{{0.0, 0.0, 0.0}};
  std::array<double, 3> ilqr_first_control{{0.0, 0.0, 0.0}};
  std::array<double, 3> first_control_delta{{0.0, 0.0, 0.0}};
  bool candidate_feasible = false;
  std::array<char, 96> rejection_reason{{}};
  bool collision_gate = false;
  bool map_gate = false;
  bool qp_problem_metrics_recorded = false;
  double hessian_diagonal_minimum = 0.0;
  double hessian_diagonal_maximum = 0.0;
  double constraint_row_l2_minimum = 0.0;
  double constraint_row_l2_maximum = 0.0;
  double nonzero_bound_abs_minimum = 0.0;
  double bound_abs_maximum = 0.0;
  double zero_delta_dynamic_equality_residual = 0.0;

  std::array<double, kControlCycleTimingStageCount> stage_ms{{}};
  std::array<bool, kControlCycleTimingStageCount> stage_recorded{{}};
};

/** @brief 单个 dump 的运行时参数口径；场景元数据由外部回归脚本写入同一 JSON 文件。 */
struct ControlCycleTelemetryMetadata {
  std::string solver_mode;
  double control_rate_hz = 0.0;
  double control_period_ms = 0.0;
  int qp_max_iterations = 0;
  double qp_time_limit_ms = 0.0;
  double qp_max_primal_residual = 0.0;
  double qp_max_dual_residual = 0.0;
  double qp_max_tracking_slack = 0.0;
  double qp_max_hard_constraint_violation = 0.0;
  bool use_sim_time = false;
  int ros_domain_id = -1;
};

/**
 * @brief 控制 timer 使用的 128 槽性能与 deadline 根因环。
 * @details `push`、计数和分位数统计只操作 `std::array`，不保存 reference、矩阵或无界历史。
 *          显式启用采样窗口时，首个有效 Execute/reference/map 身份冻结窗口，随后只接受
 *          同一 lease/reference/map 身份的精确周期数；Execute heartbeat 的 command sequence
 *          会逐拍递增，故只记录而不作为恒等字段。身份变化会保留原始残片并终止窗口，绝不混入
 *          下一条 reference。
 *          JSON 生成只允许在诊断 service 回调中调用，绝不能在控制 timer 内调用。
 */
class ControlCycleTelemetryRing {
public:
  static constexpr std::size_t kCapacity = 128;

  /**
   * @brief 以覆盖最旧样本的方式提交一个完整控制周期，容量永远不增长。
   * @details 该函数在写槽内记录真实固定环写入耗时；调用者传入的 full callback 时间必须
   *          已包含控制算法、发布、汇总与日志，环写本身作为独立阶段公开，避免重复归因。
   */
  bool push(ControlCycleTelemetrySample sample,
            std::chrono::steady_clock::time_point cycle_start);
  /** @brief 配置一次固定采样周期数；0 保持既有的最近 128 周期诊断行为。 */
  void configureSamplingWindow(std::size_t requested_cycles);
  /** @brief 对指定根因执行饱和加一；达到 UINT64_MAX 后保持上限。 */
  void incrementDeadlineCause(ControlCycleDeadlineCause cause);
  /** @brief 读取指定根因的累计次数。 */
  std::uint64_t deadlineCauseCount(ControlCycleDeadlineCause cause) const;
  /** @brief 返回所有根因累计次数，供诊断 service 拷贝后序列化。 */
  const std::array<std::uint64_t, kControlCycleDeadlineCauseCount> &
  deadlineCauseCounts() const {
    return deadline_cause_counts_;
  }
  /** @brief 返回当前有效样本数，范围固定为 [0,128]。 */
  std::size_t size() const { return count_; }
  /** @brief 按从最旧到最新的顺序读取有效槽位，绝不返回未填充槽位。 */
  const ControlCycleTelemetrySample &sampleAt(std::size_t chronological_index) const;
  /** @brief 计算一个阶段的 p50/p95/p99/max；只纳入标记为 recorded 的有限样本。 */
  ControlCycleTimingDistribution distribution(
      ControlCycleTimingStage stage) const;
  /** @brief 在非实时上下文导出稳定 JSON；不包含完整轨迹、QP 矩阵或控制路径。 */
  std::string toJson(const ControlCycleTelemetryMetadata &metadata) const;

  /** @brief 暴露独立可测的饱和加一规则，避免计数器回绕为零。 */
  static void saturatingIncrement(std::uint64_t &value);

private:
  /** @brief Execute lease/reference/map 身份是否足以作为固定窗口的首条锚点。 */
  static bool samplingIdentityEligible(const ControlCycleTelemetrySample &sample);
  /** @brief 仅比较可配对采样契约字段，禁止把 OSQP 结果或 wall-time 混入 identity。 */
  static bool sameSamplingIdentity(const ControlCycleTelemetrySample &left,
                                   const ControlCycleTelemetrySample &right);
  /** @brief 返回稳定窗口状态名，供 JSON/离线分析器 fail-closed 使用。 */
  const char *samplingWindowStatus() const;

  std::array<ControlCycleTelemetrySample, kCapacity> samples_{{}};
  std::array<std::uint64_t, kControlCycleDeadlineCauseCount>
      deadline_cause_counts_{{}};
  std::size_t cursor_ = 0;
  std::size_t count_ = 0;
  std::size_t sampling_window_requested_cycles_ = 0;
  bool sampling_window_started_ = false;
  bool sampling_window_identity_changed_ = false;
  ControlCycleTelemetrySample sampling_window_identity_;
};

/** @brief 返回稳定 JSON 字段使用的阶段名。 */
const char *controlCycleTimingStageName(ControlCycleTimingStage stage);
/** @brief 返回稳定 JSON 字段使用的 deadline 根因名。 */
const char *controlCycleDeadlineCauseName(ControlCycleDeadlineCause cause);

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__CONTROL_CYCLE_TELEMETRY_HPP_
