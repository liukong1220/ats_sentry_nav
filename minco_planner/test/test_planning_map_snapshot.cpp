// Copyright 2026

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.info.resolution = 1.0;
  grid.info.width = 5;
  grid.info.height = 5;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(25, 0);
  return grid;
}

// 地图所有者一侧的做法：同一个 provider 算场，按 float32 装进消息。
ats_navigation_interfaces::msg::PlanningMapSnapshot makeMessage(
  const nav_msgs::msg::OccupancyGrid & grid, int threshold, bool unknown_is_obstacle)
{
  ats_rc_esdf::RcTraversabilityEsdfProvider provider;
  provider.updateGrid(grid, threshold, unknown_is_obstacle);
  std::vector<double> field;
  EXPECT_TRUE(provider.copySignedDistanceField(field));
  ats_navigation_interfaces::msg::PlanningMapSnapshot message;
  message.header.frame_id = grid.header.frame_id;
  message.source_stamp = grid.header.stamp;
  message.ready = true;
  message.unknown_is_obstacle = unknown_is_obstacle;
  message.occupied_value_threshold = static_cast<std::int8_t>(threshold);
  message.localization_epoch = 1U;
  message.source_generation = 3U;
  message.publication_sequence = 5U;
  message.info = grid.info;
  message.occupancy = grid.data;
  message.signed_distance_m.assign(field.begin(), field.end());
  message.gradient_x.assign(field.size(), 0.0F);
  message.gradient_y.assign(field.size(), 0.0F);
  return message;
}

nav_msgs::msg::OccupancyGrid makeRandomGrid(std::mt19937 & rng, unsigned width, unsigned height)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.header.stamp.sec = 42;
  grid.info.resolution = 0.1F;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = -2.05;
  grid.info.origin.position.y = 1.3;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  // 若干矩形障碍与 unknown 块，加少量散点。
  std::uniform_int_distribution<unsigned> px(0U, width - 1U);
  std::uniform_int_distribution<unsigned> py(0U, height - 1U);
  std::uniform_int_distribution<unsigned> extent(1U, 6U);
  for (int block = 0; block < 14; ++block) {
    const unsigned x0 = px(rng);
    const unsigned y0 = py(rng);
    const unsigned w = extent(rng);
    const unsigned h = extent(rng);
    const std::int8_t value = block % 4 == 0 ? -1 : (block % 4 == 1 ? 60 : 100);
    for (unsigned y = y0; y < std::min(height, y0 + h); ++y) {
      for (unsigned x = x0; x < std::min(width, x0 + w); ++x) {
        grid.data[static_cast<std::size_t>(y) * width + x] = value;
      }
    }
  }
  for (int dot = 0; dot < 20; ++dot) {
    grid.data[static_cast<std::size_t>(py(rng)) * width + px(rng)] = 30;
  }
  return grid;
}

geometry_msgs::msg::PoseStamped poseAt(
  const nav_msgs::msg::OccupancyGrid & grid, unsigned x, unsigned y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = grid.header;
  pose.pose.position.x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
  pose.pose.position.y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(PlanningMapSnapshotMessage, ReproducesTheLocallyComputedFields)
{
  std::mt19937 rng(11U);
  for (int trial = 0; trial < 20; ++trial) {
    const bool unknown_is_obstacle = trial % 3 != 0;
    const auto grid = makeRandomGrid(rng, 60U, 45U);
    const auto local = minco_planner::PlanningMapSnapshot::create(
      9U, grid, 50, unknown_is_obstacle);
    const auto loaded = minco_planner::PlanningMapSnapshot::fromMessage(
      9U, makeMessage(grid, 50, unknown_is_obstacle), 50, unknown_is_obstacle);
    ASSERT_NE(local, nullptr);
    ASSERT_NE(loaded, nullptr) << "trial " << trial;
    EXPECT_EQ(loaded->grid.data, grid.data);
    EXPECT_EQ(loaded->grid.header.stamp.sec, 42);
    EXPECT_EQ(loaded->safety_content_digest, local->safety_content_digest);
    ASSERT_EQ(*loaded->blocked_squared_cells, *local->blocked_squared_cells) << "trial " << trial;
    // 双线性插值只在格心之间定义：x 格心 [-2.0, 3.9]，y 格心 [1.35, 5.75]。
    for (double y = 1.36; y < 5.74; y += 0.053) {
      for (double x = -1.99; x < 3.89; x += 0.047) {
        ASSERT_TRUE(std::isfinite(local->clearance_esdf->getDistance(x, y))) << x << "," << y;
        EXPECT_NEAR(
          loaded->clearance_esdf->getDistance(x, y), local->clearance_esdf->getDistance(x, y),
          1e-5);
      }
    }
  }
}

TEST(PlanningMapSnapshotMessage, RejectsNotReadyBrokenOrForeignPolicySnapshots)
{
  std::mt19937 rng(3U);
  const auto grid = makeRandomGrid(rng, 20U, 16U);
  const auto good = makeMessage(grid, 50, true);
  ASSERT_NE(minco_planner::PlanningMapSnapshot::fromMessage(1U, good, 50, true), nullptr);

  auto message = good;
  message.ready = false;
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::fromMessage(1U, message, 50, true), nullptr);
  message = good;
  message.signed_distance_m[7] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::fromMessage(1U, message, 50, true), nullptr);
  message = good;
  message.signed_distance_m.pop_back();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::fromMessage(1U, message, 50, true), nullptr);
  // 阻塞语义与规划器参数不同：场不能复用。
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::fromMessage(1U, good, 60, true), nullptr);
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::fromMessage(1U, good, 50, false), nullptr);
}

TEST(PlanningMapSnapshotMessage, GraphSearchOnTheSnapshotFieldMatchesTheLocalTransform)
{
  std::mt19937 rng(29U);
  int successes = 0;
  for (int trial = 0; trial < 60; ++trial) {
    const bool unknown_is_obstacle = trial % 2 == 0;
    const auto grid = makeRandomGrid(rng, 50U, 40U);
    const auto loaded = minco_planner::PlanningMapSnapshot::fromMessage(
      1U, makeMessage(grid, 50, unknown_is_obstacle), 50, unknown_is_obstacle);
    ASSERT_NE(loaded, nullptr);
    minco_planner::GridAstarParams params;
    params.unknown_is_obstacle = unknown_is_obstacle;
    params.safe_distance = 0.25;
    params.min_safe_distance = 0.1;
    params.clearance_cost_weight = trial % 3 == 0 ? 0.0 : 2.0;
    params.clearance_cost_distance = 0.6;
    params.start_escape_max_steps = trial % 4 == 0 ? 0 : 8;
    const minco_planner::GridAstar astar(params);
    const auto start = poseAt(grid, 2U + trial % 5, 3U);
    const auto goal = poseAt(grid, 46U - trial % 7, 36U);
    for (const double clearance : {-1.0, 0.15}) {
      const auto own = astar.planWithClearance(grid, start, goal, clearance);
      const auto shared = astar.planWithClearance(
        grid, start, goal, clearance, loaded->blocked_squared_cells.get());
      ASSERT_EQ(own.success, shared.success) << "trial " << trial << " " << own.reason;
      EXPECT_EQ(own.reason, shared.reason);
      EXPECT_EQ(own.expanded_nodes, shared.expanded_nodes);
      EXPECT_DOUBLE_EQ(own.length, shared.length);
      ASSERT_EQ(own.path.poses.size(), shared.path.poses.size());
      for (std::size_t i = 0; i < own.path.poses.size(); ++i) {
        EXPECT_DOUBLE_EQ(own.path.poses[i].pose.position.x, shared.path.poses[i].pose.position.x);
        EXPECT_DOUBLE_EQ(own.path.poses[i].pose.position.y, shared.path.poses[i].pose.position.y);
      }
      successes += own.success ? 1 : 0;
    }
  }
  // 随机图要同时覆盖成功与失败分支，否则差分没有意义。
  EXPECT_GT(successes, 10);
  EXPECT_LT(successes, 120);
}

TEST(PlanningMapSnapshot, KeepsGridAndEsdfImmutableAcrossGenerations)
{
  auto first_grid = makeGrid();
  first_grid.data[12] = 100;
  const auto first = minco_planner::PlanningMapSnapshot::create(7, first_grid, 50, true);
  const double first_distance = first->clearance_esdf->getDistance(2.5, 2.5);

  auto second_grid = makeGrid();
  second_grid.data[0] = 100;
  const auto second = minco_planner::PlanningMapSnapshot::create(8, second_grid, 50, true);

  EXPECT_EQ(first->generation, 7U);
  EXPECT_EQ(second->generation, 8U);
  EXPECT_EQ(first->grid.data[12], 100);
  EXPECT_EQ(second->grid.data[12], 0);
  EXPECT_DOUBLE_EQ(first->clearance_esdf->getDistance(2.5, 2.5), first_distance);
  EXPECT_LT(first_distance, 0.0);
  EXPECT_GT(second->clearance_esdf->getDistance(2.5, 2.5), 0.0);
}

TEST(PlanningMapSnapshot, SafetyIdentityIgnoresHeartbeatButRejectsSemanticChanges)
{
  auto base_grid = makeGrid();
  base_grid.header.stamp.sec = 10;
  const auto base = minco_planner::PlanningMapSnapshot::create(7, base_grid, 50, true);
  ASSERT_NE(base, nullptr);

  auto heartbeat_grid = base_grid;
  heartbeat_grid.header.stamp.sec = 11;
  const auto heartbeat =
    minco_planner::PlanningMapSnapshot::create(8, heartbeat_grid, 50, true);
  ASSERT_NE(heartbeat, nullptr);
  EXPECT_EQ(base->safety_content_digest, heartbeat->safety_content_digest);
  EXPECT_TRUE(base->hasSameSafetyContent(*heartbeat));

  auto unknown_grid = base_grid;
  unknown_grid.data[0] = -1;
  const auto unknown = minco_planner::PlanningMapSnapshot::create(9, unknown_grid, 50, true);
  ASSERT_NE(unknown, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*unknown));

  auto occupied_grid = base_grid;
  occupied_grid.data[0] = 100;
  const auto occupied = minco_planner::PlanningMapSnapshot::create(10, occupied_grid, 50, true);
  ASSERT_NE(occupied, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*occupied));

  auto shifted_grid = base_grid;
  shifted_grid.info.origin.position.x = 0.1;
  const auto shifted = minco_planner::PlanningMapSnapshot::create(11, shifted_grid, 50, true);
  ASSERT_NE(shifted, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*shifted));

  const auto changed_unknown_policy =
    minco_planner::PlanningMapSnapshot::create(12, base_grid, 50, false);
  ASSERT_NE(changed_unknown_policy, nullptr);
  EXPECT_FALSE(base->hasSameSafetyContent(*changed_unknown_policy));
}

TEST(PlanningMapSnapshot, RejectsMalformedGrid)
{
  auto grid = makeGrid();
  grid.data.pop_back();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);

  grid = makeGrid();
  grid.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);

  grid.info.resolution = std::numeric_limits<float>::infinity();
  EXPECT_EQ(minco_planner::PlanningMapSnapshot::create(1, grid, 50, true), nullptr);
}

TEST(PlannerSafetyState, RequiresBothReadyMapAndSafePlan)
{
  minco_planner::PlannerSafetyState state;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.map_ready = true;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.plan_safe = true;
  EXPECT_FALSE(state.emergencyStopRequired());
  state.map_ready = false;
  EXPECT_TRUE(state.emergencyStopRequired());
  state.map_ready = true;
  state.plan_safe = false;
  EXPECT_TRUE(state.emergencyStopRequired());
}

TEST(PlannerSafetyState, HealthLossRequiresANewerMapSnapshot)
{
  minco_planner::PlannerSafetyState state;
  state.map_ready = true;
  EXPECT_TRUE(state.mapSnapshotUsable(7));

  state.invalidateMap(7);
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  state.map_ready = true;
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  EXPECT_TRUE(state.mapSnapshotUsable(8));
}

TEST(PlannerSafetyState, NewMapInvalidatesPlanButKeepsHealthyMapLease)
{
  minco_planner::PlannerSafetyState state;
  state.map_ready = true;
  state.plan_safe = true;
  state.minimum_map_generation = 7;

  state.invalidatePlanForNewMap(8);

  EXPECT_TRUE(state.map_ready);
  EXPECT_FALSE(state.plan_safe);
  EXPECT_TRUE(state.emergencyStopRequired());
  EXPECT_FALSE(state.mapSnapshotUsable(7));
  EXPECT_TRUE(state.mapSnapshotUsable(8));
}

ats_navigation_interfaces::msg::PlannerStatus makeReadyStatus()
{
  ats_navigation_interfaces::msg::PlannerStatus status;
  status.header.stamp.sec = 10;
  status.goal_id = 12;
  status.localization_epoch = 3;
  status.plan_request_sequence = 7;
  status.map_generation = 5;
  status.map_publication_sequence = 19;
  status.reference_stamp.sec = 9;
  status.state = status.STATE_REFERENCE_READY;
  status.failure_reason = status.FAILURE_NONE;
  return status;
}

TEST(PlannerStatusState, MapLossRetainsIdentityWithoutAnActiveReference)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  EXPECT_FALSE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  ASSERT_TRUE(state.latest.has_value());
  EXPECT_EQ(state.latest->state, ready.STATE_FAILED);
  EXPECT_EQ(state.latest->failure_reason, ready.FAILURE_MAP_UNREADY);
  EXPECT_EQ(state.latest->goal_id, ready.goal_id);
  EXPECT_EQ(state.latest->localization_epoch, ready.localization_epoch);
  EXPECT_EQ(state.latest->plan_request_sequence, ready.plan_request_sequence);
  EXPECT_EQ(state.latest->map_generation, ready.map_generation);
  EXPECT_EQ(state.latest->map_publication_sequence, ready.map_publication_sequence);
  EXPECT_EQ(state.latest->reference_stamp, ready.reference_stamp);
  EXPECT_EQ(state.latest->header.stamp.nanosec, 1U);

  // Repeated false-ready/watchdog reports retain the same invalidated identity.
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_MAP_UNREADY, ready.header.stamp));
  EXPECT_EQ(state.latest->header.stamp.nanosec, 2U);
  EXPECT_FALSE(state.update(ready));
  ++ready.plan_request_sequence;
  EXPECT_FALSE(state.update(ready));
  ++ready.map_generation;
  EXPECT_TRUE(state.update(ready));
}

TEST(PlannerStatusState, PlanFailureAndMapReplacementPermitFreshReference)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(5, ready.FAILURE_RUNTIME_UNSAFE, ready.header.stamp));
  ++ready.plan_request_sequence;
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(6, ready.FAILURE_SNAPSHOT_CHANGED, ready.header.stamp));
  EXPECT_FALSE(state.update(ready));
  ++ready.map_generation;
  ++ready.plan_request_sequence;
  EXPECT_TRUE(state.update(ready));
  EXPECT_EQ(state.latest->state, ready.STATE_REFERENCE_READY);
}

TEST(PlannerStatusState, RejectsObsoleteEpochGenerationAndRequest)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ASSERT_TRUE(state.update(ready));
  auto stale = ready;
  --stale.localization_epoch;
  ++stale.map_generation;
  EXPECT_FALSE(state.update(stale));
  stale = ready;
  --stale.map_generation;
  EXPECT_FALSE(state.update(stale));
  stale = ready;
  --stale.plan_request_sequence;
  EXPECT_FALSE(state.update(stale));
  EXPECT_EQ(state.latest->goal_id, ready.goal_id);
  EXPECT_EQ(state.latest->map_generation, ready.map_generation);

  ++ready.localization_epoch;
  ready.map_generation = 1;
  ready.plan_request_sequence = 1;
  EXPECT_TRUE(state.update(ready));
}

TEST(PlannerStatusState, NewGoalRestartsPlanRequestSequence)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ASSERT_TRUE(state.update(ready));
  ASSERT_TRUE(state.invalidate(6, ready.FAILURE_SNAPSHOT_CHANGED, ready.header.stamp));
  // GoalManager restarts plan_request_sequence at 1 for each new goal.
  auto next_goal = ready;
  ++next_goal.goal_id;
  next_goal.plan_request_sequence = 1;
  next_goal.map_generation = 7;
  ASSERT_TRUE(state.update(next_goal));
  EXPECT_EQ(state.latest->goal_id, next_goal.goal_id);
  EXPECT_EQ(state.latest->state, ready.STATE_REFERENCE_READY);
  // Map generation stays globally monotonic across goals.
  auto stale_map = next_goal;
  ++stale_map.goal_id;
  stale_map.map_generation = 6;
  EXPECT_FALSE(state.update(stale_map));
}

TEST(PlannerStatusState, SerializesEqualAndBackwardClockStamps)
{
  minco_planner::PlannerStatusState state;
  auto ready = makeReadyStatus();
  ready.header.stamp.nanosec = 999999999U;
  ASSERT_TRUE(state.update(ready));
  auto heartbeat = ready;
  ASSERT_TRUE(state.update(heartbeat));
  EXPECT_EQ(heartbeat.header.stamp.sec, 11);
  EXPECT_EQ(heartbeat.header.stamp.nanosec, 0U);
  heartbeat.header.stamp.sec = 1;
  ASSERT_TRUE(state.update(heartbeat));
  EXPECT_EQ(heartbeat.header.stamp.sec, 11);
  EXPECT_EQ(heartbeat.header.stamp.nanosec, 1U);
  EXPECT_EQ(heartbeat.map_generation, ready.map_generation);
  EXPECT_EQ(heartbeat.state, ready.STATE_REFERENCE_READY);
}

TEST(HeartbeatLease, RejectsMissingFutureAndExpiredSignals)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::time_point(std::chrono::seconds(10));

  EXPECT_FALSE(minco_planner::steadyHeartbeatLeaseValid(std::nullopt, start, 1.0));
  EXPECT_FALSE(
    minco_planner::steadyHeartbeatLeaseValid(
      start, start - std::chrono::milliseconds(1), 1.0));
  EXPECT_TRUE(
    minco_planner::steadyHeartbeatLeaseValid(start, start + std::chrono::seconds(1), 1.0));
  EXPECT_FALSE(
    minco_planner::steadyHeartbeatLeaseValid(
      start, start + std::chrono::milliseconds(1001), 1.0));
}

TEST(ReferenceTrajectory, RequiresFiniteMonotonicSamples)
{
  minco_planner::ReferenceTrajectory trajectory;
  EXPECT_FALSE(trajectory.valid());
  minco_planner::ReferencePoint first;
  minco_planner::ReferencePoint second;
  second.t = 1.0;
  second.s = 1.0;
  second.x = 1.0;
  trajectory.points = {first, second};
  EXPECT_TRUE(trajectory.valid());
  trajectory.points.back().t = 0.0;
  EXPECT_FALSE(trajectory.valid());
}

TEST(FootprintSafetyChecker, RejectsEmptyTrajectory)
{
  minco_planner::FootprintSafetyChecker checker;
  minco_planner::ReferenceTrajectory trajectory;
  EXPECT_FALSE(checker.check(trajectory, makeGrid()).safe);
}

}  // namespace
