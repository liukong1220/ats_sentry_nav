// Copyright 2026

#include <limits>

#include "ats_navigation_interfaces/msg/planner_candidate.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "minco_planner/nodes/atomic_planning_contract.hpp"
#include "gtest/gtest.h"

namespace {

ats_navigation_interfaces::msg::PlanningMapSnapshot makeSnapshot() {
  ats_navigation_interfaces::msg::PlanningMapSnapshot snapshot;
  snapshot.header.frame_id = "map";
  snapshot.ready = true;
  snapshot.unknown_is_obstacle = true;
  snapshot.occupied_value_threshold = 50;
  snapshot.source_generation = 9;
  snapshot.publication_sequence = 17;
  snapshot.info.resolution = 0.1F;
  snapshot.info.width = 2;
  snapshot.info.height = 2;
  snapshot.info.origin.orientation.w = 1.0;
  snapshot.occupancy = {0, 100, -1, 0};
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  snapshot.signed_distance_m = {0.5F, -0.1F, nan, 0.3F};
  snapshot.gradient_x = {1.0F, 0.0F, nan, 0.5F};
  snapshot.gradient_y = {0.0F, 1.0F, nan, -0.5F};
  return snapshot;
}

nav_msgs::msg::Path makePath() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped first;
  first.header.frame_id = "map";
  first.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped second = first;
  second.header.stamp.sec = 1;
  second.pose.position.x = 1.0;
  path.poses = {first, second};
  return path;
}

ats_navigation_interfaces::msg::PlannerCandidate makeCandidate() {
  ats_navigation_interfaces::msg::PlannerCandidate candidate;
  candidate.header.frame_id = "map";
  candidate.state = candidate.STATE_READY;
  candidate.planner_incarnation = 3;
  candidate.candidate_sequence = 4;
  candidate.goal_id = 5;
  candidate.localization_epoch = 6;
  candidate.map_snapshot_generation = 7;
  candidate.map_source_generation = 8;
  candidate.map_publication_sequence = 9;
  candidate.failure_reason = candidate.FAILURE_NONE;
  candidate.yaw_authority = candidate.YAW_AUTHORITY_BODY_YAW_FOLLOW;
  candidate.requires_gimbal_lock = true;
  candidate.footprint_safe = true;
  candidate.dynamics_feasible = true;
  candidate.footprint_collision_samples = 0;
  candidate.minimum_clearance_m = 0.2F;
  candidate.lease_duration.nanosec = 500000000U;
  candidate.content_digest[0] = 0xA5U;
  candidate.raw_path = makePath();
  candidate.reference = makePath();
  return candidate;
}

TEST(AtomicPlanningContract, AcceptsCompleteNumericSnapshot) {
  EXPECT_TRUE(minco_planner::validPlanningMapSnapshot(makeSnapshot()));
}

TEST(AtomicPlanningContract, RejectsMixedGenerationAndEsdfSemantics) {
  auto snapshot = makeSnapshot();
  snapshot.source_generation = 0;
  EXPECT_FALSE(minco_planner::validPlanningMapSnapshot(snapshot));

  snapshot = makeSnapshot();
  snapshot.signed_distance_m.pop_back();
  EXPECT_FALSE(minco_planner::validPlanningMapSnapshot(snapshot));

  snapshot = makeSnapshot();
  snapshot.signed_distance_m[2] = 1.0F;
  EXPECT_FALSE(minco_planner::validPlanningMapSnapshot(snapshot));

  snapshot = makeSnapshot();
  snapshot.signed_distance_m[1] = 0.1F;
  EXPECT_FALSE(minco_planner::validPlanningMapSnapshot(snapshot));
}

TEST(AtomicPlanningContract,
     RejectsReadyCandidateWithoutSafeIdentityAndContent) {
  EXPECT_TRUE(minco_planner::validPlannerCandidate(makeCandidate()));

  auto candidate = makeCandidate();
  candidate.content_digest.fill(0U);
  EXPECT_FALSE(minco_planner::validPlannerCandidate(candidate));

  candidate = makeCandidate();
  candidate.footprint_collision_samples = 1;
  EXPECT_FALSE(minco_planner::validPlannerCandidate(candidate));

  candidate = makeCandidate();
  candidate.reference.poses[1].header.frame_id = "odom";
  EXPECT_FALSE(minco_planner::validPlannerCandidate(candidate));
}

TEST(AtomicPlanningContract, RejectedCandidateCannotLookLikeAReferenceLease) {
  auto candidate = makeCandidate();
  candidate.state = candidate.STATE_REJECTED;
  candidate.failure_reason = candidate.FAILURE_MAP_UNREADY;
  EXPECT_TRUE(minco_planner::validPlannerCandidate(candidate));

  candidate.failure_reason = candidate.FAILURE_NONE;
  EXPECT_FALSE(minco_planner::validPlannerCandidate(candidate));
}

} // namespace
