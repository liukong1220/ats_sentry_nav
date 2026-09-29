// Copyright 2026

#include <cmath>
#include <limits>

#include "ats_goal_manager/planning_snapshot_safety.hpp"
#include "gtest/gtest.h"

namespace
{

ats_navigation_interfaces::msg::PlanningMapSnapshot makeSnapshot()
{
  ats_navigation_interfaces::msg::PlanningMapSnapshot snapshot;
  snapshot.header.frame_id = "odom";
  snapshot.ready = true;
  snapshot.occupied_value_threshold = 50;
  snapshot.source_generation = 4U;
  snapshot.publication_sequence = 8U;
  snapshot.info.resolution = 1.0F;
  snapshot.info.width = 8U;
  snapshot.info.height = 8U;
  snapshot.info.origin.orientation.w = 1.0;
  snapshot.occupancy.assign(64, 0);
  snapshot.signed_distance_m.assign(64, 1.0F);
  snapshot.gradient_x.assign(64, 0.0F);
  snapshot.gradient_y.assign(64, 0.0F);
  return snapshot;
}

geometry_msgs::msg::Pose poseAt(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

TEST(PlanningSnapshotSafety, RequiresCompleteNumericSnapshot)
{
  auto snapshot = makeSnapshot();
  EXPECT_TRUE(ats_goal_manager::validPlanningSnapshot(snapshot));
  snapshot.gradient_x.pop_back();
  EXPECT_FALSE(ats_goal_manager::validPlanningSnapshot(snapshot));
}

TEST(PlanningSnapshotSafety, RejectsOutsideUnknownAndOccupiedRobotCells)
{
  const auto params = ats_goal_manager::PlanningSnapshotSafetyParams{};
  auto snapshot = makeSnapshot();
  EXPECT_TRUE(
    ats_goal_manager::checkPlanningSnapshotFootprint(snapshot, poseAt(1.5, 1.5), params)
      .footprint_safe);

  auto outside = ats_goal_manager::checkPlanningSnapshotFootprint(
    snapshot, poseAt(-0.1, 1.5), params);
  EXPECT_FALSE(outside.inside_map);
  EXPECT_FALSE(outside.footprint_safe);

  snapshot.occupancy[1U * snapshot.info.width + 1U] = -1;
  snapshot.signed_distance_m[1U * snapshot.info.width + 1U] =
    std::numeric_limits<float>::quiet_NaN();
  snapshot.gradient_x[1U * snapshot.info.width + 1U] =
    std::numeric_limits<float>::quiet_NaN();
  snapshot.gradient_y[1U * snapshot.info.width + 1U] =
    std::numeric_limits<float>::quiet_NaN();
  const auto unknown = ats_goal_manager::checkPlanningSnapshotFootprint(
    snapshot, poseAt(1.5, 1.5), params);
  EXPECT_TRUE(unknown.inside_map);
  EXPECT_FALSE(unknown.robot_cell_free);
  EXPECT_FALSE(unknown.footprint_safe);

  snapshot = makeSnapshot();
  snapshot.occupancy[1U * snapshot.info.width + 2U] = 100;
  snapshot.signed_distance_m[1U * snapshot.info.width + 2U] = -1.0F;
  auto wide_params = params;
  wide_params.footprint_length = 1.5;
  const auto occupied = ats_goal_manager::checkPlanningSnapshotFootprint(
    snapshot, poseAt(1.5, 1.5), wide_params);
  EXPECT_TRUE(occupied.inside_map);
  EXPECT_TRUE(occupied.robot_cell_free);
  EXPECT_FALSE(occupied.footprint_safe);
}

TEST(PlanningSnapshotSafety, ShallowContactToleratesOnlyBoundedDepth)
{
  ats_goal_manager::PlanningSnapshotSafetyParams params;
  params.footprint_length = 2.0;
  params.footprint_width = 2.0;
  params.footprint_safety_margin = 0.0;
  auto snapshot = makeSnapshot();
  // 占据格 x in [5,6)。车心 x=4.05 时足迹 [3.05,5.05]，压进 0.05 m。
  for (int y = 0; y < 8; ++y) {
    snapshot.occupancy[static_cast<std::size_t>(y) * 8U + 5U] = 100;
    snapshot.signed_distance_m[static_cast<std::size_t>(y) * 8U + 5U] = -1.0F;
  }
  const auto pose = poseAt(4.05, 3.5);
  EXPECT_FALSE(
    ats_goal_manager::checkPlanningSnapshotFootprint(snapshot, pose, params).footprint_safe);
  EXPECT_FALSE(ats_goal_manager::planningSnapshotContactShallow(snapshot, pose, params, 0.0));
  EXPECT_TRUE(ats_goal_manager::planningSnapshotContactShallow(snapshot, pose, params, 0.1));
  // 压进 0.4 m 时 0.1 m 深度上界不放行。
  EXPECT_FALSE(
    ats_goal_manager::planningSnapshotContactShallow(snapshot, poseAt(4.4, 3.5), params, 0.1));
}

TEST(PlanningSnapshotSafety, HonorsMapOriginYawForFootprintChecks)
{
  auto snapshot = makeSnapshot();
  constexpr double pi = 3.14159265358979323846;
  const double yaw = 0.5 * pi;
  snapshot.info.origin.position.x = -2.0;
  snapshot.info.origin.position.y = 1.0;
  snapshot.info.origin.orientation.z = std::sin(0.5 * yaw);
  snapshot.info.origin.orientation.w = std::cos(0.5 * yaw);
  const auto pose = poseAt(-3.5, 4.5, yaw);
  const auto result = ats_goal_manager::checkPlanningSnapshotFootprint(snapshot, pose);
  EXPECT_TRUE(result.inside_map);
  EXPECT_TRUE(result.footprint_safe);
}

}  // namespace
