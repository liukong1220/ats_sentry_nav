// Copyright 2026

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <string>

#include "ats_swerve_mpc/qp/control_cycle_telemetry.hpp"

namespace {

using ats_swerve_mpc::ControlCycleDeadlineCause;
using ats_swerve_mpc::ControlCycleTelemetryMetadata;
using ats_swerve_mpc::ControlCycleTelemetryRing;
using ats_swerve_mpc::ControlCycleTelemetrySample;
using ats_swerve_mpc::ControlCycleTimingStage;

ControlCycleTelemetrySample sample(std::uint64_t sequence, double callback_ms) {
  ControlCycleTelemetrySample value;
  value.cycle_sequence = sequence;
  value.qp_shadow_attempted = true;
  value.qp_first_control = {{0.1, -0.2, 0.3}};
  value.ilqr_first_control = {{0.05, -0.1, 0.2}};
  value.first_control_delta = {{0.05, -0.1, 0.1}};
  const auto callback_index = ats_swerve_mpc::controlCycleTimingStageIndex(
      ControlCycleTimingStage::kFullCallback);
  value.stage_ms[callback_index] = callback_ms;
  value.stage_recorded[callback_index] = true;
  return value;
}

ControlCycleTelemetrySample pairedSample(std::uint64_t sequence) {
  auto value = sample(sequence, 4.0);
  value.manager_incarnation = 17;
  value.command_sequence = 29;
  value.goal_id = 31;
  value.localization_epoch = 43;
  value.map_generation = 47;
  value.map_publication_sequence = 53;
  value.reference_stamp_ns = 1'000'000;
  value.reference_deadline_ns = 2'000'000;
  std::snprintf(value.reference_frame.data(), value.reference_frame.size(), "%s", "odom");
  value.execution_lease_valid = true;
  value.reference_fresh = true;
  return value;
}

TEST(ControlCycleTelemetryRing, IgnoresUnrecordedAndNonFiniteSlotsInDistribution) {
  ControlCycleTelemetryRing ring;
  auto first = sample(1, 1.0);
  auto second = sample(2, 9.0);
  const auto callback_index = ats_swerve_mpc::controlCycleTimingStageIndex(
      ControlCycleTimingStage::kFullCallback);
  second.stage_ms[callback_index] = std::numeric_limits<double>::quiet_NaN();
  ring.push(first, std::chrono::steady_clock::now());
  ring.push(second, std::chrono::steady_clock::now());

  const auto stats = ring.distribution(ControlCycleTimingStage::kFullCallback);
  EXPECT_EQ(stats.count, 1u);
  EXPECT_DOUBLE_EQ(stats.p50_ms, 1.0);
  EXPECT_DOUBLE_EQ(stats.p95_ms, 1.0);
  EXPECT_DOUBLE_EQ(stats.p99_ms, 1.0);
  EXPECT_DOUBLE_EQ(stats.max_ms, 1.0);
}

TEST(ControlCycleTelemetryRing, OverwritesOnlyTheOldestSlotAtFixedCapacity) {
  ControlCycleTelemetryRing ring;
  const auto cycle_start = std::chrono::steady_clock::now();
  for (std::size_t index = 0; index < ControlCycleTelemetryRing::kCapacity + 3; ++index) {
    ring.push(sample(static_cast<std::uint64_t>(index + 1),
                     static_cast<double>(index)), cycle_start);
  }

  EXPECT_EQ(ring.size(), ControlCycleTelemetryRing::kCapacity);
  EXPECT_EQ(ring.sampleAt(0).cycle_sequence, 4u);
  EXPECT_EQ(ring.sampleAt(ring.size() - 1).cycle_sequence,
            ControlCycleTelemetryRing::kCapacity + 3);
}

TEST(ControlCycleTelemetryRing, DeadlineCountersSaturateAndNamesStayStable) {
  std::uint64_t counter = std::numeric_limits<std::uint64_t>::max() - 1;
  ControlCycleTelemetryRing::saturatingIncrement(counter);
  ControlCycleTelemetryRing::saturatingIncrement(counter);
  EXPECT_EQ(counter, std::numeric_limits<std::uint64_t>::max());

  ControlCycleTelemetryRing ring;
  ring.incrementDeadlineCause(ControlCycleDeadlineCause::kOsqpTimeLimitStatus);
  EXPECT_EQ(ring.deadlineCauseCount(
      ControlCycleDeadlineCause::kOsqpTimeLimitStatus), 1u);
  EXPECT_STREQ(ats_swerve_mpc::controlCycleDeadlineCauseName(
      ControlCycleDeadlineCause::kCallbackPeriodOverrun),
      "callback_period_overrun_count");
  EXPECT_STREQ(ats_swerve_mpc::controlCycleTimingStageName(
      ControlCycleTimingStage::kQpBackendPhase), "qp_backend_phase_ms");
  EXPECT_STREQ(ats_swerve_mpc::controlCycleDeadlineCauseName(
      ControlCycleDeadlineCause::kQpPhaseBudgetOverrun),
      "qp_phase_budget_overrun_count");
}

TEST(ControlCycleTelemetryRing, JsonContainsRuntimeMetadataControlsAndWallTimes) {
  ControlCycleTelemetryRing ring;
  auto value = sample(7, 4.0);
  value.osqp_reported_update_ms = 0.2;
  value.osqp_reported_solve_ms = 1.2;
  value.osqp_wall_update_ms = 0.3;
  value.osqp_wall_solve_ms = 1.3;
  value.osqp_wall_qp_phase_ms = 1.5;
  ring.push(value, std::chrono::steady_clock::now());

  ControlCycleTelemetryMetadata metadata;
  metadata.solver_mode = "qp_shadow";
  metadata.control_rate_hz = 20.0;
  metadata.control_period_ms = 50.0;
  metadata.qp_max_iterations = 400;
  metadata.qp_time_limit_ms = 10.0;
  metadata.qp_max_primal_residual = 1e-4;
  metadata.qp_max_dual_residual = 1e-4;
  metadata.qp_max_tracking_slack = 0.0;
  metadata.qp_max_hard_constraint_violation = 1e-7;
  metadata.ros_domain_id = 231;
  const std::string json = ring.toJson(metadata);
  EXPECT_NE(json.find("\"schema_version\":3"), std::string::npos);
  EXPECT_NE(json.find("\"control_period_ms\":50"), std::string::npos);
  EXPECT_NE(json.find("\"qp_max_iterations\":400"), std::string::npos);
  EXPECT_NE(json.find("\"qp_max_primal_residual\":0.0001"),
            std::string::npos);
  EXPECT_NE(json.find("\"qp_max_dual_residual\":0.0001"),
            std::string::npos);
  EXPECT_NE(json.find("\"osqp_wall_qp_phase_ms\":1.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"qp_backend_phase_ms\""), std::string::npos);
  EXPECT_NE(json.find("\"qp_max_hard_constraint_violation\":9.9999999999999995e-08"),
            std::string::npos);
  EXPECT_NE(json.find("\"osqp_wall_update_ms\":"), std::string::npos);
  EXPECT_NE(json.find("\"qp_first_control\":["), std::string::npos);
  EXPECT_NE(json.find("\"first_control_delta\":["), std::string::npos);
  EXPECT_NE(json.find("\"timer_interarrival_overrun_count\""),
            std::string::npos);
}

TEST(ControlCycleTelemetryRing, FreezesExactIdentityWindowAndRejectsLaterCycles) {
  ControlCycleTelemetryRing ring;
  ring.configureSamplingWindow(3);
  const auto started = std::chrono::steady_clock::now();
  EXPECT_TRUE(ring.push(pairedSample(10), started));
  EXPECT_TRUE(ring.push(pairedSample(11), started));
  EXPECT_TRUE(ring.push(pairedSample(12), started));
  EXPECT_FALSE(ring.push(pairedSample(13), started));
  EXPECT_EQ(ring.size(), 3u);
  EXPECT_EQ(ring.sampleAt(0).cycle_sequence, 10u);
  EXPECT_EQ(ring.sampleAt(2).cycle_sequence, 12u);

  const std::string json = ring.toJson(ControlCycleTelemetryMetadata{});
  EXPECT_NE(json.find("\"requested_cycle_count\":3"), std::string::npos);
  EXPECT_NE(json.find("\"collected_cycle_count\":3"), std::string::npos);
  EXPECT_NE(json.find("\"status\":\"complete\""), std::string::npos);
  EXPECT_NE(json.find("\"manager_incarnation\":17"), std::string::npos);
  EXPECT_NE(json.find("\"execution_lease_valid\":true"), std::string::npos);
}

TEST(ControlCycleTelemetryRing, StopsWindowWhenExecuteReferenceOrMapIdentityChanges) {
  ControlCycleTelemetryRing ring;
  ring.configureSamplingWindow(3);
  const auto started = std::chrono::steady_clock::now();
  EXPECT_TRUE(ring.push(pairedSample(10), started));
  auto changed = pairedSample(11);
  changed.map_generation += 1;
  EXPECT_FALSE(ring.push(changed, started));
  EXPECT_FALSE(ring.push(pairedSample(12), started));
  EXPECT_EQ(ring.size(), 1u);

  const std::string json = ring.toJson(ControlCycleTelemetryMetadata{});
  EXPECT_NE(json.find("\"status\":\"identity_changed_before_complete\""),
            std::string::npos);
}

TEST(ControlCycleTelemetryRing, WaitsForMeaningfulLocalizationAndMapIdentity) {
  ControlCycleTelemetryRing ring;
  ring.configureSamplingWindow(2);
  const auto started = std::chrono::steady_clock::now();
  auto invalid = pairedSample(10);
  invalid.localization_epoch = 0;
  EXPECT_FALSE(ring.push(invalid, started));
  invalid = pairedSample(11);
  invalid.map_generation = 0;
  EXPECT_FALSE(ring.push(invalid, started));
  invalid = pairedSample(12);
  invalid.map_publication_sequence = 0;
  EXPECT_FALSE(ring.push(invalid, started));
  EXPECT_TRUE(ring.push(pairedSample(13), started));
  EXPECT_EQ(ring.size(), 1u);
  EXPECT_NE(ring.toJson(ControlCycleTelemetryMetadata{}).find(
                "\"status\":\"collecting\""),
            std::string::npos);
}

}  // namespace
