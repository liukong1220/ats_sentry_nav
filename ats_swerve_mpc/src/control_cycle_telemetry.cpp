// Copyright 2026

#include "ats_swerve_mpc/qp/control_cycle_telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace ats_swerve_mpc {
namespace {

double percentile(std::array<double, ControlCycleTelemetryRing::kCapacity> values,
                  std::size_t count, double quantile) {
  if (count == 0) {
    return 0.0;
  }
  std::sort(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count));
  const std::size_t index = std::min(
      count - 1,
      static_cast<std::size_t>(quantile * static_cast<double>(count - 1)));
  return values[index];
}

void appendJsonString(std::ostringstream &stream, const char *value) {
  stream << '"';
  for (const char *cursor = value; *cursor != '\0'; ++cursor) {
    switch (*cursor) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default: stream << *cursor; break;
    }
  }
  stream << '"';
}

void appendJsonString(std::ostringstream &stream, const std::string &value) {
  appendJsonString(stream, value.c_str());
}

void appendDouble(std::ostringstream &stream, double value) {
  if (std::isfinite(value)) {
    stream << value;
  } else {
    stream << "null";
  }
}

}  // namespace

const char *controlCycleTimingStageName(ControlCycleTimingStage stage) {
  switch (stage) {
    case ControlCycleTimingStage::kStateTrajectorySnapshot: return "state_trajectory_snapshot_ms";
    case ControlCycleTimingStage::kReferenceExtraction: return "reference_extraction_ms";
    case ControlCycleTimingStage::kIlqrSolve: return "ilqr_solve_ms";
    case ControlCycleTimingStage::kIlqrWarmStart: return "ilqr_warm_start_ms";
    case ControlCycleTimingStage::kIlqrRollout: return "ilqr_rollout_ms";
    case ControlCycleTimingStage::kIlqrBackwardPass: return "ilqr_backward_pass_ms";
    case ControlCycleTimingStage::kIlqrJacobian: return "ilqr_jacobian_ms";
    case ControlCycleTimingStage::kIlqrLineSearch: return "ilqr_line_search_ms";
    case ControlCycleTimingStage::kIlqrCommandPublish: return "ilqr_command_publish_ms";
    case ControlCycleTimingStage::kQpProblemBuild: return "qp_problem_build_ms";
    case ControlCycleTimingStage::kOsqpNumericUpdate: return "osqp_numeric_update_ms";
    case ControlCycleTimingStage::kQpBackendPhase: return "qp_backend_phase_ms";
    case ControlCycleTimingStage::kQpCompletePhase: return "qp_complete_phase_ms";
    case ControlCycleTimingStage::kOsqpSolve: return "osqp_solve_ms";
    case ControlCycleTimingStage::kPrimalReconstructionRollout: return "primal_reconstruction_rollout_ms";
    case ControlCycleTimingStage::kCandidateHardCheck: return "candidate_hard_check_ms";
    case ControlCycleTimingStage::kTelemetryRingWrite: return "telemetry_ring_write_ms";
    case ControlCycleTimingStage::kPercentileAggregation: return "percentile_aggregation_ms";
    case ControlCycleTimingStage::kLoggingPublish: return "logging_publish_ms";
    case ControlCycleTimingStage::kFullCallback: return "full_callback_ms";
    case ControlCycleTimingStage::kTimerInterarrival: return "timer_interarrival_ms";
    case ControlCycleTimingStage::kCount: break;
  }
  return "unknown";
}

const char *controlCycleDeadlineCauseName(ControlCycleDeadlineCause cause) {
  switch (cause) {
    case ControlCycleDeadlineCause::kOsqpTimeLimitStatus: return "osqp_time_limit_status_count";
    case ControlCycleDeadlineCause::kOsqpSolveBudgetOverrun: return "osqp_solve_budget_overrun_count";
    case ControlCycleDeadlineCause::kCallbackPeriodOverrun: return "callback_period_overrun_count";
    case ControlCycleDeadlineCause::kIlqrSolveBudgetOverrun: return "ilqr_solve_budget_overrun_count";
    case ControlCycleDeadlineCause::kQpProblemBuildBudgetOverrun: return "qp_problem_build_budget_overrun_count";
    case ControlCycleDeadlineCause::kQpUpdateBudgetOverrun: return "qp_update_budget_overrun_count";
    case ControlCycleDeadlineCause::kQpPhaseBudgetOverrun: return "qp_phase_budget_overrun_count";
    case ControlCycleDeadlineCause::kQpCompletePhaseBudgetOverrun:
      return "qp_complete_phase_budget_overrun_count";
    case ControlCycleDeadlineCause::kQpCandidateAuditBudgetOverrun: return "qp_candidate_audit_budget_overrun_count";
    case ControlCycleDeadlineCause::kTelemetryAggregationBudgetOverrun: return "telemetry_aggregation_budget_overrun_count";
    case ControlCycleDeadlineCause::kLoggingPublishBudgetOverrun: return "logging_publish_budget_overrun_count";
    case ControlCycleDeadlineCause::kTimerInterarrivalOverrun: return "timer_interarrival_overrun_count";
    case ControlCycleDeadlineCause::kCount: break;
  }
  return "unknown";
}

void ControlCycleTelemetryRing::saturatingIncrement(std::uint64_t &value) {
  if (value != std::numeric_limits<std::uint64_t>::max()) {
    ++value;
  }
}

bool ControlCycleTelemetryRing::samplingIdentityEligible(
    const ControlCycleTelemetrySample &sample) {
  return sample.execution_lease_valid && sample.reference_fresh &&
         sample.manager_incarnation != 0 && sample.command_sequence != 0 &&
         sample.goal_id != 0 && sample.localization_epoch != 0 &&
         sample.map_generation != 0 && sample.map_publication_sequence != 0 &&
         sample.reference_stamp_ns > 0 &&
         sample.reference_deadline_ns > sample.reference_stamp_ns &&
         sample.reference_frame.front() != '\0';
}

bool ControlCycleTelemetryRing::sameSamplingIdentity(
    const ControlCycleTelemetrySample &left,
    const ControlCycleTelemetrySample &right) {
  return left.manager_incarnation == right.manager_incarnation &&
         left.goal_id == right.goal_id &&
         left.localization_epoch == right.localization_epoch &&
         left.map_generation == right.map_generation &&
         left.map_publication_sequence == right.map_publication_sequence &&
         left.reference_stamp_ns == right.reference_stamp_ns &&
         left.reference_deadline_ns == right.reference_deadline_ns &&
         left.execution_lease_valid == right.execution_lease_valid &&
         left.reference_fresh == right.reference_fresh &&
         std::strncmp(left.reference_frame.data(), right.reference_frame.data(),
                      left.reference_frame.size()) == 0;
}

const char *ControlCycleTelemetryRing::samplingWindowStatus() const {
  if (sampling_window_requested_cycles_ == 0) {
    return "disabled";
  }
  if (sampling_window_identity_changed_) {
    return "identity_changed_before_complete";
  }
  if (!sampling_window_started_) {
    return "waiting_for_identity";
  }
  if (count_ == sampling_window_requested_cycles_) {
    return "complete";
  }
  return "collecting";
}

void ControlCycleTelemetryRing::configureSamplingWindow(
    std::size_t requested_cycles) {
  sampling_window_requested_cycles_ = std::min(requested_cycles, kCapacity);
  sampling_window_started_ = false;
  sampling_window_identity_changed_ = requested_cycles > kCapacity;
  sampling_window_identity_ = ControlCycleTelemetrySample{};
  samples_.fill(ControlCycleTelemetrySample{});
  deadline_cause_counts_.fill(0);
  cursor_ = 0;
  count_ = 0;
}

bool ControlCycleTelemetryRing::push(
    ControlCycleTelemetrySample sample,
    std::chrono::steady_clock::time_point cycle_start) {
  if (sampling_window_requested_cycles_ != 0) {
    if (sampling_window_identity_changed_ ||
        count_ == sampling_window_requested_cycles_) {
      return false;
    }
    if (!sampling_window_started_) {
      if (!samplingIdentityEligible(sample)) {
        return false;
      }
      sampling_window_identity_ = sample;
      sampling_window_started_ = true;
    } else if (!sameSamplingIdentity(sample, sampling_window_identity_)) {
      sampling_window_identity_changed_ = true;
      return false;
    }
  }
  const auto write_start = std::chrono::steady_clock::now();
  samples_[cursor_] = sample;
  const auto write_end = std::chrono::steady_clock::now();
  ControlCycleTelemetrySample &stored = samples_[cursor_];
  const std::size_t ring_write_index = controlCycleTimingStageIndex(
      ControlCycleTimingStage::kTelemetryRingWrite);
  stored.stage_ms[ring_write_index] = 1000.0 * std::chrono::duration<double>(
      write_end - write_start).count();
  stored.stage_recorded[ring_write_index] = true;
  const std::size_t callback_index = controlCycleTimingStageIndex(
      ControlCycleTimingStage::kFullCallback);
  if (!stored.stage_recorded[callback_index]) {
    stored.stage_ms[callback_index] = 1000.0 * std::chrono::duration<double>(
        write_end - cycle_start).count();
    stored.stage_recorded[callback_index] = true;
  }
  cursor_ = (cursor_ + 1) % kCapacity;
  count_ = std::min(kCapacity, count_ + 1);
  return true;
}

void ControlCycleTelemetryRing::incrementDeadlineCause(
    ControlCycleDeadlineCause cause) {
  saturatingIncrement(deadline_cause_counts_[controlCycleDeadlineCauseIndex(cause)]);
}

std::uint64_t ControlCycleTelemetryRing::deadlineCauseCount(
    ControlCycleDeadlineCause cause) const {
  return deadline_cause_counts_[controlCycleDeadlineCauseIndex(cause)];
}

const ControlCycleTelemetrySample &ControlCycleTelemetryRing::sampleAt(
    std::size_t chronological_index) const {
  if (chronological_index >= count_) {
    throw std::out_of_range("control telemetry chronological index");
  }
  const std::size_t oldest = (cursor_ + kCapacity - count_) % kCapacity;
  return samples_[(oldest + chronological_index) % kCapacity];
}

ControlCycleTimingDistribution ControlCycleTelemetryRing::distribution(
    ControlCycleTimingStage stage) const {
  ControlCycleTimingDistribution result;
  std::array<double, kCapacity> values{};
  const std::size_t index = controlCycleTimingStageIndex(stage);
  for (std::size_t sample_index = 0; sample_index < count_; ++sample_index) {
    const ControlCycleTelemetrySample &sample = sampleAt(sample_index);
    const double value = sample.stage_ms[index];
    if (sample.stage_recorded[index] && std::isfinite(value) && value >= 0.0) {
      values[result.count++] = value;
      result.max_ms = std::max(result.max_ms, value);
    }
  }
  result.p50_ms = percentile(values, result.count, 0.50);
  result.p95_ms = percentile(values, result.count, 0.95);
  result.p99_ms = percentile(values, result.count, 0.99);
  return result;
}

std::string ControlCycleTelemetryRing::toJson(
    const ControlCycleTelemetryMetadata &metadata) const {
  std::ostringstream stream;
  stream << std::setprecision(17);
  stream << "{\"schema_version\":4,\"metadata\":{\"solver_mode\":";
  appendJsonString(stream, metadata.solver_mode);
  stream << ",\"control_rate_hz\":";
  appendDouble(stream, metadata.control_rate_hz);
  stream << ",\"control_period_ms\":";
  appendDouble(stream, metadata.control_period_ms);
  stream << ",\"qp_max_iterations\":" << metadata.qp_max_iterations;
  stream << ",\"qp_time_limit_ms\":";
  appendDouble(stream, metadata.qp_time_limit_ms);
  stream << ",\"qp_max_primal_residual\":";
  appendDouble(stream, metadata.qp_max_primal_residual);
  stream << ",\"qp_max_dual_residual\":";
  appendDouble(stream, metadata.qp_max_dual_residual);
  stream << ",\"qp_max_tracking_slack\":";
  appendDouble(stream, metadata.qp_max_tracking_slack);
  stream << ",\"qp_max_hard_constraint_violation\":";
  appendDouble(stream, metadata.qp_max_hard_constraint_violation);
  stream << ",\"use_sim_time\":" << (metadata.use_sim_time ? "true" : "false")
         << ",\"ros_domain_id\":" << metadata.ros_domain_id << "},";
  stream << "\"sampling_window\":{\"requested_cycle_count\":"
         << sampling_window_requested_cycles_
         << ",\"collected_cycle_count\":" << count_
         << ",\"status\":";
  appendJsonString(stream, samplingWindowStatus());
  stream << "},";
  stream << "\"deadline_counters\":{";
  for (std::size_t index = 0; index < kControlCycleDeadlineCauseCount; ++index) {
    if (index != 0) {
      stream << ',';
    }
    appendJsonString(stream, controlCycleDeadlineCauseName(
        static_cast<ControlCycleDeadlineCause>(index)));
    stream << ':' << deadline_cause_counts_[index];
  }
  stream << "},\"timing_summary\":{";
  for (std::size_t index = 0; index < kControlCycleTimingStageCount; ++index) {
    if (index != 0) {
      stream << ',';
    }
    const ControlCycleTimingDistribution stats = distribution(
        static_cast<ControlCycleTimingStage>(index));
    appendJsonString(stream, controlCycleTimingStageName(
        static_cast<ControlCycleTimingStage>(index)));
    stream << ":{\"count\":" << stats.count << ",\"p50_ms\":";
    appendDouble(stream, stats.p50_ms);
    stream << ",\"p95_ms\":";
    appendDouble(stream, stats.p95_ms);
    stream << ",\"p99_ms\":";
    appendDouble(stream, stats.p99_ms);
    stream << ",\"max_ms\":";
    appendDouble(stream, stats.max_ms);
    stream << '}';
  }
  stream << "},\"samples\":[";
  for (std::size_t sample_index = 0; sample_index < count_; ++sample_index) {
    if (sample_index != 0) {
      stream << ',';
    }
    const ControlCycleTelemetrySample &sample = sampleAt(sample_index);
    stream << "{\"cycle_sequence\":" << sample.cycle_sequence
           << ",\"qp_shadow_attempted\":"
           << (sample.qp_shadow_attempted ? "true" : "false")
           << ",\"snapshot_identity_digest\":" << sample.snapshot_identity_digest
           << ",\"same_snapshot_identity\":"
           << (sample.same_snapshot_identity ? "true" : "false")
           << ",\"manager_incarnation\":" << sample.manager_incarnation
           << ",\"command_sequence\":" << sample.command_sequence
           << ",\"goal_id\":" << sample.goal_id
           << ",\"localization_epoch\":" << sample.localization_epoch
           << ",\"map_generation\":" << sample.map_generation
           << ",\"map_publication_sequence\":"
           << sample.map_publication_sequence
           << ",\"reference_stamp_ns\":" << sample.reference_stamp_ns
           << ",\"reference_deadline_ns\":" << sample.reference_deadline_ns
           << ",\"reference_frame\":";
    appendJsonString(stream, sample.reference_frame.data());
    stream << ",\"execution_lease_valid\":"
           << (sample.execution_lease_valid ? "true" : "false")
           << ",\"reference_fresh\":"
           << (sample.reference_fresh ? "true" : "false");
    stream << ",\"status\":";
    appendJsonString(stream, ltvQpSolverStatusName(sample.status));
    stream << ",\"iterations\":" << sample.iterations
           << ",\"warm_start_used\":"
           << (sample.warm_start_used ? "true" : "false")
           << ",\"osqp_reported_update_ms\":";
    appendDouble(stream, sample.osqp_reported_update_ms);
    stream << ",\"osqp_reported_solve_ms\":";
    appendDouble(stream, sample.osqp_reported_solve_ms);
    stream << ",\"osqp_wall_update_ms\":";
    appendDouble(stream, sample.osqp_wall_update_ms);
    stream << ",\"osqp_wall_solve_ms\":";
    appendDouble(stream, sample.osqp_wall_solve_ms);
    stream << ",\"qp_complete_phase_ms\":";
    appendDouble(stream, sample.qp_complete_phase_ms);
    stream << ",\"osqp_wall_qp_phase_ms\":";
    appendDouble(stream, sample.osqp_wall_qp_phase_ms);
    stream << ",\"primal_residual\":";
    appendDouble(stream, sample.primal_residual);
    stream << ",\"dual_residual\":";
    appendDouble(stream, sample.dual_residual);
    stream << ",\"hard_constraint_margin\":";
    appendDouble(stream, sample.hard_constraint_margin);
    stream << ",\"slack_maximum\":";
    appendDouble(stream, sample.slack_maximum);
    stream << ",\"qp_first_control\":[";
    for (std::size_t axis = 0; axis < sample.qp_first_control.size(); ++axis) {
      if (axis != 0) {
        stream << ',';
      }
      appendDouble(stream, sample.qp_first_control[axis]);
    }
    stream << "],\"ilqr_first_control\":[";
    for (std::size_t axis = 0; axis < sample.ilqr_first_control.size(); ++axis) {
      if (axis != 0) {
        stream << ',';
      }
      appendDouble(stream, sample.ilqr_first_control[axis]);
    }
    stream << "],\"first_control_delta\":[";
    for (std::size_t axis = 0; axis < sample.first_control_delta.size(); ++axis) {
      if (axis != 0) {
        stream << ',';
      }
      appendDouble(stream, sample.first_control_delta[axis]);
    }
    stream << ']';
    stream << ",\"candidate_feasible\":"
           << (sample.candidate_feasible ? "true" : "false")
           << ",\"rejection_reason\":";
    appendJsonString(stream, sample.rejection_reason.data());
    stream << ",\"collision_gate\":"
           << (sample.collision_gate ? "true" : "false")
           << ",\"map_gate\":" << (sample.map_gate ? "true" : "false")
           << ",\"qp_problem_metrics\":";
    if (sample.qp_problem_metrics_recorded) {
      stream << "{\"hessian_diagonal_minimum\":";
      appendDouble(stream, sample.hessian_diagonal_minimum);
      stream << ",\"hessian_diagonal_maximum\":";
      appendDouble(stream, sample.hessian_diagonal_maximum);
      stream << ",\"constraint_row_l2_minimum\":";
      appendDouble(stream, sample.constraint_row_l2_minimum);
      stream << ",\"constraint_row_l2_maximum\":";
      appendDouble(stream, sample.constraint_row_l2_maximum);
      stream << ",\"nonzero_bound_abs_minimum\":";
      appendDouble(stream, sample.nonzero_bound_abs_minimum);
      stream << ",\"bound_abs_maximum\":";
      appendDouble(stream, sample.bound_abs_maximum);
      stream << ",\"zero_delta_dynamic_equality_residual\":";
      appendDouble(stream, sample.zero_delta_dynamic_equality_residual);
      stream << '}';
    } else {
      stream << "null";
    }
    stream << ",\"timing_ms\":{";
    for (std::size_t index = 0; index < kControlCycleTimingStageCount; ++index) {
      if (index != 0) {
        stream << ',';
      }
      appendJsonString(stream, controlCycleTimingStageName(
          static_cast<ControlCycleTimingStage>(index)));
      stream << ':';
      if (sample.stage_recorded[index]) {
        appendDouble(stream, sample.stage_ms[index]);
      } else {
        stream << "null";
      }
    }
    stream << "}}";
  }
  stream << "]}";
  return stream.str();
}

}  // namespace ats_swerve_mpc
