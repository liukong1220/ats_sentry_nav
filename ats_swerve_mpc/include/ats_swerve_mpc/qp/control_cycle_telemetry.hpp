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
  kIlqrSolve,
  kIlqrCommandPublish,
  kQpProblemBuild,
  kOsqpNumericUpdate,
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
  std::uint64_t command_sequence = 0;
  std::uint64_t goal_id = 0;
  std::uint64_t map_generation = 0;
  std::int64_t reference_stamp_ns = 0;
  std::array<char, 64> reference_frame{{}};

  LtvQpSolverStatus status = LtvQpSolverStatus::kBackendUnavailable;
  int iterations = 0;
  bool warm_start_used = false;
  double osqp_reported_update_ms = 0.0;
  double osqp_reported_solve_ms = 0.0;
  // OSQPInfo 的内部计时与 C API 墙钟分别保留，避免把前者误作完整 adapter 开销。
  double osqp_wall_update_ms = 0.0;
  double osqp_wall_solve_ms = 0.0;
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
  double zero_delta_dynamic_equality_residual = 0.0;

  std::array<double, kControlCycleTimingStageCount> stage_ms{{}};
  std::array<bool, kControlCycleTimingStageCount> stage_recorded{{}};
};

/** @brief 单个 dump 的运行时参数口径；场景元数据由外部回归脚本写入同一 JSON 文件。 */
struct ControlCycleTelemetryMetadata {
  std::string solver_mode;
  double control_rate_hz = 0.0;
  double control_period_ms = 0.0;
  double qp_time_limit_ms = 0.0;
  bool use_sim_time = false;
  int ros_domain_id = -1;
};

/**
 * @brief 控制 timer 使用的 128 槽性能与 deadline 根因环。
 * @details `push`、计数和分位数统计只操作 `std::array`，不保存 reference、矩阵或无界历史。
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
  void push(ControlCycleTelemetrySample sample,
            std::chrono::steady_clock::time_point cycle_start);
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
  std::array<ControlCycleTelemetrySample, kCapacity> samples_{{}};
  std::array<std::uint64_t, kControlCycleDeadlineCauseCount>
      deadline_cause_counts_{{}};
  std::size_t cursor_ = 0;
  std::size_t count_ = 0;
};

/** @brief 返回稳定 JSON 字段使用的阶段名。 */
const char *controlCycleTimingStageName(ControlCycleTimingStage stage);
/** @brief 返回稳定 JSON 字段使用的 deadline 根因名。 */
const char *controlCycleDeadlineCauseName(ControlCycleDeadlineCause cause);

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__CONTROL_CYCLE_TELEMETRY_HPP_
