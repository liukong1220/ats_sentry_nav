// Copyright 2026

#include <limits>

#include <gtest/gtest.h>

#include "ats_swerve_mpc/qp/control_cycle_snapshot.hpp"

namespace {

using ats_swerve_mpc::Control;
using ats_swerve_mpc::ControlCycleSnapshot;
using ats_swerve_mpc::Se2Reference;
using ats_swerve_mpc::State;

/** @brief 构造含所有 canonical 审计字段的有限快照，供摘要变更回归复用。 */
ControlCycleSnapshot completeSnapshot() {
  ControlCycleSnapshot snapshot;
  snapshot.current_state = State(1.25, -2.5, 0.75);
  snapshot.last_control_before_solve = Control(0.2, -0.1, 0.3);
  snapshot.reference_frame = "odom";
  snapshot.reference_stamp_ns = 1234567890123LL;
  snapshot.reference_deadline_ns = 1234567990123LL;
  snapshot.manager_incarnation = 101;
  snapshot.command_sequence = 102;
  snapshot.goal_id = 103;
  snapshot.localization_epoch = 104;
  snapshot.map_generation = 105;
  snapshot.map_publication_sequence = 106;

  Se2Reference first;
  first.state = State(1.5, -2.0, 0.8);
  first.control = Control(0.25, -0.05, 0.35);
  Se2Reference second;
  second.state = State(1.75, -1.5, 0.85);
  second.control = Control(0.3, 0.0, 0.4);
  snapshot.references = {first, second};
  return snapshot;
}

TEST(ControlCycleSnapshotIdentity, IsStableForExactCanonicalInputs) {
  const ControlCycleSnapshot snapshot = completeSnapshot();
  ASSERT_TRUE(ats_swerve_mpc::controlCycleSnapshotIdentityInputsFinite(snapshot));
  const std::uint64_t first =
      ats_swerve_mpc::controlCycleSnapshotIdentityDigest(snapshot);
  const std::uint64_t second =
      ats_swerve_mpc::controlCycleSnapshotIdentityDigest(snapshot);
  EXPECT_NE(first, 0U);
  EXPECT_EQ(first, second);
}

TEST(ControlCycleSnapshotIdentity, ChangesForEveryAuditedInputCategory) {
  const ControlCycleSnapshot baseline = completeSnapshot();
  const std::uint64_t digest =
      ats_swerve_mpc::controlCycleSnapshotIdentityDigest(baseline);
  ASSERT_NE(digest, 0U);

  auto changed = baseline;
  changed.current_state(0) += 0.01;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.reference_stamp_ns += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.reference_frame = "map";
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.references[1].state(2) += 0.01;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.references[1].control(1) += 0.01;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.last_control_before_solve(2) += 0.01;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.manager_incarnation += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.command_sequence += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.goal_id += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.localization_epoch += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.map_generation += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));

  changed = baseline;
  changed.map_publication_sequence += 1;
  EXPECT_NE(digest, ats_swerve_mpc::controlCycleSnapshotIdentityDigest(changed));
}

TEST(ControlCycleSnapshotIdentity, RejectsNonFiniteAuditedValues) {
  auto snapshot = completeSnapshot();
  snapshot.current_state(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ats_swerve_mpc::controlCycleSnapshotIdentityInputsFinite(snapshot));
  EXPECT_EQ(ats_swerve_mpc::controlCycleSnapshotIdentityDigest(snapshot), 0U);

  snapshot = completeSnapshot();
  snapshot.references[0].control(1) =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ats_swerve_mpc::controlCycleSnapshotIdentityInputsFinite(snapshot));
  EXPECT_EQ(ats_swerve_mpc::controlCycleSnapshotIdentityDigest(snapshot), 0U);

  snapshot = completeSnapshot();
  snapshot.last_control_before_solve(2) =
      -std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ats_swerve_mpc::controlCycleSnapshotIdentityInputsFinite(snapshot));
  EXPECT_EQ(ats_swerve_mpc::controlCycleSnapshotIdentityDigest(snapshot), 0U);
}

}  // namespace
