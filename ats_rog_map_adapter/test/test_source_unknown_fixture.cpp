// Copyright 2026

#include <cmath>
#include <limits>
#include <vector>

#include "ats_rog_map_adapter/ground_projection_fusion.hpp"
#include "ats_rog_map_adapter/test_fault_authorization.hpp"
#include "gtest/gtest.h"

namespace
{

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

/// @brief Build a structurally valid all-unknown numeric projection snapshot.
ats_rog_map_adapter::RogMapEsdfSnapshot makeAllUnknownSnapshot(
  std::uint32_t width = 4, std::uint32_t height = 4, std::uint64_t generation = 7)
{
  ats_rog_map_adapter::RogMapEsdfSnapshot snapshot;
  snapshot.header.frame_id = "odom";
  snapshot.header.stamp.sec = 12;
  snapshot.header.stamp.nanosec = 340000000U;
  snapshot.info.resolution = 0.10F;
  snapshot.info.width = width;
  snapshot.info.height = height;
  snapshot.info.origin.orientation.w = 1.0;
  snapshot.generation = generation;
  snapshot.ready = true;
  snapshot.stale = false;
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  snapshot.occupancy.assign(cells, static_cast<int8_t>(-1));
  snapshot.signed_distance.assign(cells, kNan);
  snapshot.gradient_x.assign(cells, kNan);
  snapshot.gradient_y.assign(cells, kNan);
  return snapshot;
}

TEST(SourceUnknownEvidence, AcceptsStrictlyAllUnknownProjectionWithAdvancedGeneration)
{
  const auto snapshot = makeAllUnknownSnapshot();
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_TRUE(evidence.structurally_valid);
  EXPECT_TRUE(evidence.all_unknown);
  EXPECT_EQ(evidence.cell_count, 16U);
  EXPECT_EQ(evidence.unknown_cells, 16U);
  EXPECT_EQ(evidence.free_cells, 0U);
  EXPECT_EQ(evidence.occupied_cells, 0U);
  EXPECT_EQ(evidence.out_of_range_cells, 0U);
  EXPECT_EQ(evidence.finite_numeric_cells, 0U);
  EXPECT_EQ(evidence.generation, 7U);
  EXPECT_FALSE(evidence.reason.empty());
}

TEST(SourceUnknownEvidence, RejectsMixedUnknownAndOccupiedCells)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.occupancy[5] = 100;
  snapshot.signed_distance[5] = 0.0F;
  snapshot.gradient_x[5] = 1.0F;
  snapshot.gradient_y[5] = 0.0F;
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_TRUE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_EQ(evidence.occupied_cells, 1U);
  EXPECT_EQ(evidence.unknown_cells, 15U);
  EXPECT_NE(evidence.reason.find("mixes unknown"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsMixedUnknownAndFreeCells)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.occupancy[0] = 0;
  snapshot.signed_distance[0] = 0.4F;
  snapshot.gradient_x[0] = 0.0F;
  snapshot.gradient_y[0] = 1.0F;
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_TRUE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_EQ(evidence.free_cells, 1U);
  EXPECT_EQ(evidence.unknown_cells, 15U);
  EXPECT_NE(evidence.reason.find("free=1"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsEmptyGrid)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.info.width = 0;
  snapshot.info.height = 0;
  snapshot.occupancy.clear();
  snapshot.signed_distance.clear();
  snapshot.gradient_x.clear();
  snapshot.gradient_y.clear();
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_FALSE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_NE(evidence.reason.find("empty"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsOccupancyLengthMismatch)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.occupancy.pop_back();
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_FALSE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_NE(evidence.reason.find("occupancy length"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsNumericArrayLengthMismatch)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.gradient_y.pop_back();
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_FALSE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_NE(evidence.reason.find("gradient length"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsFiniteNumericSamplesInUnknownCells)
{
  auto snapshot = makeAllUnknownSnapshot();
  snapshot.signed_distance[3] = 0.25F;
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_TRUE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_EQ(evidence.unknown_cells, 16U);
  EXPECT_EQ(evidence.finite_numeric_cells, 1U);
  EXPECT_NE(evidence.reason.find("finite"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsNotReadyOrStaleProjection)
{
  auto not_ready = makeAllUnknownSnapshot();
  not_ready.ready = false;
  const auto not_ready_evidence =
    ats_rog_map_adapter::evaluateSourceUnknownEvidence(not_ready, 6);
  EXPECT_FALSE(not_ready_evidence.all_unknown);
  EXPECT_NE(not_ready_evidence.reason.find("not ready"), std::string::npos);

  auto stale = makeAllUnknownSnapshot();
  stale.stale = true;
  const auto stale_evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(stale, 6);
  EXPECT_FALSE(stale_evidence.all_unknown);
  EXPECT_NE(stale_evidence.reason.find("stale"), std::string::npos);
}

TEST(SourceUnknownEvidence, RejectsMissingFrameOrStampOrResolution)
{
  auto no_frame = makeAllUnknownSnapshot();
  no_frame.header.frame_id.clear();
  EXPECT_FALSE(ats_rog_map_adapter::evaluateSourceUnknownEvidence(no_frame, 6).all_unknown);

  auto no_stamp = makeAllUnknownSnapshot();
  no_stamp.header.stamp.sec = 0;
  no_stamp.header.stamp.nanosec = 0U;
  EXPECT_FALSE(ats_rog_map_adapter::evaluateSourceUnknownEvidence(no_stamp, 6).all_unknown);

  auto bad_resolution = makeAllUnknownSnapshot();
  bad_resolution.info.resolution = 0.0F;
  EXPECT_FALSE(
    ats_rog_map_adapter::evaluateSourceUnknownEvidence(bad_resolution, 6).all_unknown);
}

TEST(SourceUnknownEvidence, RejectsGenerationThatDidNotAdvancePastPreFaultBaseline)
{
  const auto snapshot = makeAllUnknownSnapshot(4, 4, 6);
  const auto evidence = ats_rog_map_adapter::evaluateSourceUnknownEvidence(snapshot, 6);
  EXPECT_TRUE(evidence.structurally_valid);
  EXPECT_FALSE(evidence.all_unknown);
  EXPECT_NE(evidence.reason.find("did not advance"), std::string::npos);
}

TEST(TestFaultAuthorization, RefusesGuardedParametersWhileGateIsOff)
{
  const std::vector<std::string> guarded{
    "test_mask_secondary_evidence", "test_inject_dynamic_obstacle"};
  const auto verdict = ats_rog_map_adapter::screenTestFaultParameters(
    false, {"test_mask_secondary_evidence"}, guarded);
  EXPECT_FALSE(verdict.accepted);
  EXPECT_NE(verdict.reason.find("enable_test_fault_injection=false"), std::string::npos);
}

TEST(TestFaultAuthorization, AllowsGuardedParametersOnlyWhileGateIsOn)
{
  const std::vector<std::string> guarded{
    "test_mask_secondary_evidence", "test_inject_dynamic_obstacle"};
  const auto verdict = ats_rog_map_adapter::screenTestFaultParameters(
    true, {"test_mask_secondary_evidence"}, guarded);
  EXPECT_TRUE(verdict.accepted);
  EXPECT_TRUE(verdict.reason.empty());
}

TEST(TestFaultAuthorization, RefusesRuntimeChangesToTheGateItself)
{
  for (const bool gate : {false, true}) {
    const auto verdict = ats_rog_map_adapter::screenTestFaultParameters(
      gate, {"enable_test_fault_injection"}, {"test_mask_secondary_evidence"});
    EXPECT_FALSE(verdict.accepted);
    EXPECT_NE(verdict.reason.find("startup-only"), std::string::npos);
  }
}

TEST(TestFaultAuthorization, LeavesUnrelatedParametersUntouchedWhileGateIsOff)
{
  const auto verdict = ats_rog_map_adapter::screenTestFaultParameters(
    false, {"projection_rate_hz", "unknown_is_obstacle"}, {"test_mask_secondary_evidence"});
  EXPECT_TRUE(verdict.accepted);
}

}  // namespace
