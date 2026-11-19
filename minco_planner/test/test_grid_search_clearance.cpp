// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>

#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_clearance.hpp"
#include "minco_planner/planning/grid_jps.hpp"
#include "minco_planner/planning/grid_search.hpp"
#include "minco_planner/safety/goal_pose_admission.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

// RMUC 2025 MuJoCo profile: 0.60 x 0.50 m body plus 0.02 m margin.
constexpr double kCircumscribedRadius = 0.4187;  // hypot(0.32, 0.27)
constexpr double kInscribedRadius = 0.27;        // 0.25 + 0.02
constexpr int kHardObstacle = 100;
constexpr int kThreshold = 100;

nav_msgs::msg::OccupancyGrid makeOpenGrid(unsigned int width, unsigned int height)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.1F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

void block(nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  if (
    x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
    y >= static_cast<int>(grid.info.height)) {
    return;
  }
  grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = kHardObstacle;
}

void blockRow(nav_msgs::msg::OccupancyGrid & grid, int y, int x_begin, int x_end)
{
  for (int x = x_begin; x <= x_end; ++x) {
    block(grid, x, y);
  }
}

geometry_msgs::msg::PoseStamped poseAt(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

minco_planner::GridOccupancyPolicy policy()
{
  minco_planner::GridOccupancyPolicy p;
  p.obstacle_value_threshold = kThreshold;
  p.unknown_is_obstacle = false;
  return p;
}

/// Two parallel walls leaving an 11-cell (1.10 m) free band, matching the
/// measured RMUC west corridor: the band centre clears the inscribed radius but
/// the cells near the goal only clear 0.30 m, below the circumscribed radius.
nav_msgs::msg::OccupancyGrid makeWestCorridorGrid()
{
  auto grid = makeOpenGrid(60, 30);
  blockRow(grid, 20, 0, 45);  // north wall
  blockRow(grid, 8, 10, 59);  // south wall
  // Local pinch near the west goal: narrows the usable band to 0.30 m.
  blockRow(grid, 11, 2, 6);
  return grid;
}

}  // namespace

// --- Defect B: the JPS -> A* fallback used to drop the clearance requirement ---

TEST(GridSearchClearance, AstarHonoursSafeDistanceInsteadOfHuggingWalls)
{
  auto grid = makeOpenGrid(40, 20);
  // Wall with a 3-cell (0.30 m) opening: passable with no clearance budget,
  // impassable for a body needing the circumscribed radius.
  for (int y = 0; y < 20; ++y) {
    if (y >= 9 && y <= 11) {
      continue;
    }
    block(grid, 20, y);
  }

  minco_planner::GridAstarParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;

  params.safe_distance = 0.0;
  const auto unconstrained =
    minco_planner::GridAstar(params).plan(grid, poseAt(0.55, 1.05), poseAt(3.55, 1.05));
  ASSERT_TRUE(unconstrained.success) << unconstrained.reason;

  params.safe_distance = kCircumscribedRadius;
  const auto constrained =
    minco_planner::GridAstar(params).plan(grid, poseAt(0.55, 1.05), poseAt(3.55, 1.05));
  EXPECT_FALSE(constrained.success)
    << "A* routed through a 0.30 m gap while claiming " << kCircumscribedRadius << " m clearance";
}

TEST(GridSearchClearance, JpsAndAstarShareTheSameTraversabilityVerdict)
{
  auto grid = makeOpenGrid(40, 20);
  for (int y = 0; y < 20; ++y) {
    if (y >= 9 && y <= 11) {
      continue;
    }
    block(grid, 20, y);
  }

  minco_planner::GridJpsParams jps_params;
  jps_params.obstacle_value_threshold = kThreshold;
  jps_params.unknown_is_obstacle = false;
  jps_params.safe_distance = kCircumscribedRadius;
  minco_planner::GridAstarParams astar_params = jps_params;

  const auto start = poseAt(0.55, 1.05);
  const auto goal = poseAt(3.55, 1.05);
  const auto jps_result = minco_planner::GridJps(jps_params).plan(grid, start, goal);
  const auto astar_result = minco_planner::GridAstar(astar_params).plan(grid, start, goal);
  EXPECT_EQ(jps_result.success, astar_result.success);
}

// --- Defect A: the start ring turned a tight pose into a permanent deadlock ---

TEST(GridSearchClearance, StartBelowClearanceIsAdmittedBecauseTheRobotIsAlreadyThere)
{
  const auto grid = makeWestCorridorGrid();
  const auto start = poseAt(5.05, 1.05);  // 0.20 m from the south wall
  const auto goal = poseAt(5.05, 1.45);   // 0.60 m clearance

  ASSERT_NEAR(minco_planner::measureGridClearance(grid, 50, 10, 1.0, policy()), 0.20, 1e-3);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kInscribedRadius;

  params.assume_start_traversable = false;
  const auto rejected = minco_planner::GridJps(params).plan(grid, start, goal);
  ASSERT_FALSE(rejected.success);
  EXPECT_EQ(rejected.reason, "start occupied");

  params.assume_start_traversable = true;
  const auto admitted = minco_planner::GridJps(params).plan(grid, start, goal);
  EXPECT_TRUE(admitted.success) << admitted.reason;
}

TEST(GridSearchClearance, StartCellPaintedOccupiedIsStillAdmitted)
{
  // The fused grid transiently paints the robot's own cell as a hard obstacle
  // because there is no ego carve-out for occupied cells; A* reported
  // "start is occupied" and the goal manager retried it as transient forever.
  auto grid = makeOpenGrid(30, 20);
  block(grid, 5, 5);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = 0.0;

  params.assume_start_traversable = false;
  const auto strict =
    minco_planner::GridJps(params).plan(grid, poseAt(0.55, 0.55), poseAt(2.5, 1.5));
  ASSERT_FALSE(strict.success);
  EXPECT_EQ(strict.reason, "start occupied");

  params.assume_start_traversable = true;
  const auto admitted =
    minco_planner::GridJps(params).plan(grid, poseAt(0.55, 0.55), poseAt(2.5, 1.5));
  EXPECT_TRUE(admitted.success) << admitted.reason;
}

// --- Defect A2: the start cell is admitted but none of its neighbours is, so A* stops at expanded=1 ---
//
// Recorded 2025 red-box stops (20 events, runs d141..d222) all look like this: the robot was pushed
// to 0.1-0.2 m from a wall, the search floor is 0.376 m, every neighbour of the start cell is below
// it, and the search ends after expanding the start cell only. Escape prefix and local repair run
// after the search, so they never got a path to act on.

constexpr double kSearchFloor = 0.376;  // inscribed 0.3055 + half-cell diagonal 0.0707

minco_planner::GridAstarParams escapeParams(int max_steps)
{
  minco_planner::GridAstarParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kSearchFloor;
  params.min_safe_distance = kInscribedRadius;
  params.start_escape_max_steps = max_steps;
  return params;
}

/// One wall row at y = 10 (cell centre 1.05 m); the field above it is open.
nav_msgs::msg::OccupancyGrid makeSouthWallGrid()
{
  auto grid = makeOpenGrid(60, 30);
  blockRow(grid, 10, 0, 59);
  return grid;
}

TEST(GridSearchStartEscape, DisabledKeepsTheExpandedOneDeadEnd)
{
  const auto grid = makeSouthWallGrid();
  const auto start = poseAt(3.05, 1.25);  // 0.20 m from the wall; its neighbours reach only 0.30 m
  const auto goal = poseAt(3.05, 2.05);

  const auto result = minco_planner::GridAstar(escapeParams(0)).plan(grid, start, goal);
  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.reason, "no path");
  EXPECT_EQ(result.expanded_nodes, 1);
}

TEST(GridSearchStartEscape, ClimbsOutAlongNonDecreasingClearance)
{
  const auto grid = makeSouthWallGrid();
  const auto start = poseAt(3.05, 1.25);
  const auto goal = poseAt(3.05, 2.05);

  const auto result = minco_planner::GridAstar(escapeParams(2)).plan(grid, start, goal);
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GE(result.path.poses.size(), 3U);

  // Clearance never drops along the sub-floor stretch, and the path ends up at the search floor
  // or better after it leaves the start.
  double previous = -1.0;
  bool reached_floor = false;
  for (const auto & pose : result.path.poses) {
    const int x = static_cast<int>(std::floor(pose.pose.position.x / 0.1));
    const int y = static_cast<int>(std::floor(pose.pose.position.y / 0.1));
    const double clearance = minco_planner::measureGridClearance(grid, x, y, 2.0, policy());
    if (!reached_floor) {
      EXPECT_GE(clearance + 1e-9, previous) << "pose (" << x << "," << y << ") re-approaches the wall";
      previous = clearance;
    }
    reached_floor = reached_floor || clearance > kSearchFloor;
  }
  EXPECT_TRUE(reached_floor);
}

TEST(GridSearchStartEscape, StepBudgetIsAHardBound)
{
  const auto grid = makeSouthWallGrid();
  const auto start = poseAt(3.05, 1.15);  // 0.10 m from the wall: two sub-floor cells to cross
  const auto goal = poseAt(3.05, 2.05);

  EXPECT_FALSE(minco_planner::GridAstar(escapeParams(1)).plan(grid, start, goal).success);
  EXPECT_TRUE(minco_planner::GridAstar(escapeParams(2)).plan(grid, start, goal).success);
}

TEST(GridSearchStartEscape, DoesNotDescendOrTunnelThroughAClosedPocket)
{
  // Start in a strip between two walls whose clearance peaks at the start itself. Nothing is
  // "uphill" from there, so the escape must not wander through the plateau or across a wall.
  auto grid = makeOpenGrid(60, 30);
  blockRow(grid, 10, 0, 59);
  blockRow(grid, 14, 0, 59);
  for (int y = 10; y <= 14; ++y) {
    block(grid, 0, y);
    block(grid, 59, y);
  }
  const auto start = poseAt(3.05, 1.25);  // y index 12, the middle of the strip
  const auto goal = poseAt(3.05, 2.05);   // above the upper wall

  EXPECT_FALSE(minco_planner::GridAstar(escapeParams(8)).plan(grid, start, goal).success);
}

TEST(GridSearchStartEscape, NormalStartIsUntouchedByTheOption)
{
  const auto grid = makeSouthWallGrid();
  const auto start = poseAt(3.05, 1.65);  // 0.60 m: already above the floor
  const auto goal = poseAt(5.05, 2.05);

  const auto plain = minco_planner::GridAstar(escapeParams(0)).plan(grid, start, goal);
  const auto escape = minco_planner::GridAstar(escapeParams(4)).plan(grid, start, goal);
  ASSERT_TRUE(plain.success);
  ASSERT_TRUE(escape.success);
  ASSERT_EQ(plain.path.poses.size(), escape.path.poses.size());
  EXPECT_EQ(plain.expanded_nodes, escape.expanded_nodes);
  EXPECT_DOUBLE_EQ(plain.length, escape.length);
}

TEST(GridSearchStartEscape, StartCellPaintedOccupiedCanAlsoClimbOut)
{
  auto grid = makeSouthWallGrid();
  block(grid, 30, 12);  // the robot's own cell painted as a hard obstacle (transient fusion)
  const auto start = poseAt(3.05, 1.25);
  const auto goal = poseAt(3.05, 2.05);

  EXPECT_FALSE(minco_planner::GridAstar(escapeParams(0)).plan(grid, start, goal).success);
  EXPECT_TRUE(minco_planner::GridAstar(escapeParams(3)).plan(grid, start, goal).success);
}

// --- Defect C: a goal that no yaw-free disc fits, but the real body does ---

TEST(GridSearchClearance, GoalBelowCircumscribedRadiusIsReachedAtItsOwnClearance)
{
  const auto grid = makeWestCorridorGrid();
  const auto start = poseAt(5.05, 1.45);
  const auto goal = poseAt(0.45, 1.45);

  const double goal_clearance = minco_planner::measureGridClearance(grid, 4, 14, 1.0, policy());
  ASSERT_LT(goal_clearance, kCircumscribedRadius);
  ASSERT_GT(goal_clearance, kInscribedRadius);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kCircumscribedRadius;

  params.relax_endpoint_clearance = false;
  const auto rejected = minco_planner::GridJps(params).plan(grid, start, goal);
  ASSERT_FALSE(rejected.success);
  EXPECT_EQ(rejected.reason, "goal occupied");

  params.relax_endpoint_clearance = true;
  const auto relaxed = minco_planner::GridJps(params).plan(grid, start, goal);
  EXPECT_TRUE(relaxed.success) << relaxed.reason;
}

// --- Fix D: the endpoint floor is an isotropic proxy, the rectangular gate is exact ---
//
// domain 187 目标 9 `highland_ramp`:目标格净空约 0.26 m,footprint 一致下限 0.341 m,
// 图搜索直接报 goal occupied,7 个规划周期一条路径都产不出来直到看门狗耗尽。目标是
// 一个已知 yaw 的位姿,矩形足迹门禁能精确判定它;此时用各向同性代理量否决精确判定
// 只会让"窄终点"表现成规划失败。放宽只作用于目标格能否作为终点,搜索器随后按实测
// 目标净空重规划整条路径,轨迹安全性仍由矩形门禁裁定。
TEST(GridSearchClearance, GoalBelowTheFootprintConsistentFloorNeedsTheEndpointRelaxation)
{
  auto grid = makeOpenGrid(30, 20);
  for (int y = 0; y < 20; ++y) {
    block(grid, 15, y);
  }
  const auto start = poseAt(0.75, 0.95);
  const auto goal = poseAt(1.25, 0.95);

  const double search_floor = minco_planner::footprintConsistentClearanceFloor(
    kInscribedRadius, grid.info.resolution, kCircumscribedRadius);
  const double allowance = minco_planner::gridQuantizationAllowance(grid.info.resolution);
  const double goal_clearance = minco_planner::measureGridClearance(grid, 12, 9, 1.0, policy());
  ASSERT_LT(goal_clearance, search_floor);
  ASSERT_GT(goal_clearance, allowance);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.assume_start_traversable = true;
  params.relax_endpoint_clearance = true;
  params.safe_distance = kCircumscribedRadius;

  params.min_safe_distance = search_floor;
  const auto rejected =
    minco_planner::GridJps(params).planWithClearance(grid, start, goal, search_floor);
  ASSERT_FALSE(rejected.success);
  EXPECT_EQ(rejected.reason, "goal occupied");

  params.min_safe_distance = allowance;
  const auto relaxed =
    minco_planner::GridJps(params).planWithClearance(grid, start, goal, search_floor);
  EXPECT_TRUE(relaxed.success) << relaxed.reason;
}

TEST(GridSearchClearance, EndpointRelaxationStillRejectsAGoalOnAnOccupiedCell)
{
  auto grid = makeOpenGrid(30, 20);
  block(grid, 12, 9);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.assume_start_traversable = true;
  params.relax_endpoint_clearance = true;
  params.safe_distance = kCircumscribedRadius;
  params.min_safe_distance = minco_planner::gridQuantizationAllowance(grid.info.resolution);
  const auto result = minco_planner::GridJps(params).planWithClearance(
    grid, poseAt(0.75, 0.95), poseAt(1.25, 0.95), kInscribedRadius);
  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal occupied");
}

TEST(GridSearchClearance, GoalOnAnOccupiedCellIsStillRejected)
{
  auto grid = makeOpenGrid(30, 20);
  block(grid, 25, 15);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kCircumscribedRadius;
  const auto result =
    minco_planner::GridJps(params).plan(grid, poseAt(0.55, 0.55), poseAt(2.55, 1.55));
  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal occupied");
}

// --- the graduated ladder the node walks, exercised through the search API ---

TEST(GridSearchClearance, ClearanceLadderReachesGoalOnlyAtTheInscribedLevel)
{
  const auto grid = makeWestCorridorGrid();
  // A tight start pose is an interior problem, not an endpoint one: every cell
  // the robot could step to is itself below the circumscribed radius, so only
  // the ladder's lower rung produces a route.
  const auto start = poseAt(5.05, 1.05);  // 0.20 m clearance
  const auto goal = poseAt(3.05, 1.45);   // 0.60 m clearance, no relaxation needed

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kCircumscribedRadius;
  params.min_safe_distance = kInscribedRadius;
  const minco_planner::GridJps planner(params);

  const auto preferred = planner.planWithClearance(grid, start, goal, kCircumscribedRadius);
  EXPECT_FALSE(preferred.success);

  const auto floored = planner.planWithClearance(grid, start, goal, kInscribedRadius);
  EXPECT_TRUE(floored.success) << floored.reason;
}

TEST(GridSearchClearance, ClearanceOverrideDoesNotMutateConfiguredParams)
{
  const auto grid = makeWestCorridorGrid();
  const auto start = poseAt(5.05, 1.05);
  const auto goal = poseAt(3.05, 1.45);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kCircumscribedRadius;
  params.min_safe_distance = kInscribedRadius;
  const minco_planner::GridJps planner(params);

  ASSERT_TRUE(planner.planWithClearance(grid, start, goal, kInscribedRadius).success);
  const auto after = planner.planWithClearance(grid, start, goal, kCircumscribedRadius);
  EXPECT_FALSE(after.success) << "the relaxed attempt leaked into the configured clearance";
}

// --- the shared clearance primitive both searches now depend on ---

TEST(GridSearchClearance, MeasuredClearanceRoundTripsThroughTheRingTest)
{
  const auto grid = makeWestCorridorGrid();
  for (const auto & cell : {std::pair<int, int>{4, 14}, {50, 10}, {30, 14}, {50, 14}}) {
    const double measured =
      minco_planner::measureGridClearance(grid, cell.first, cell.second, 1.0, policy());
    EXPECT_TRUE(minco_planner::hasGridClearance(grid, cell.first, cell.second, measured, policy()))
      << "cell (" << cell.first << "," << cell.second << ") measured " << measured;
    EXPECT_FALSE(minco_planner::hasGridClearance(
      grid, cell.first, cell.second, measured + grid.info.resolution, policy()))
      << "cell (" << cell.first << "," << cell.second << ") exceeded " << measured;
  }
}

TEST(GridSearchClearance, OutOfBoundsCountsAsBlockedForClearance)
{
  const auto grid = makeOpenGrid(30, 20);
  EXPECT_FALSE(minco_planner::hasGridClearance(grid, 0, 0, 0.2, policy()));
  EXPECT_TRUE(minco_planner::hasGridClearance(grid, 15, 10, 0.2, policy()));
}

TEST(GridSearchClearance, EndpointRelaxationStopsAtTheInscribedFloor)
{
  auto grid = makeOpenGrid(40, 20);
  // Goal parked in a 0.10 m nook: no body yaw fits, so relaxing to it would only
  // re-create the zero-clearance fallback.
  blockRow(grid, 4, 20, 30);
  blockRow(grid, 6, 20, 30);

  minco_planner::GridJpsParams params;
  params.obstacle_value_threshold = kThreshold;
  params.unknown_is_obstacle = false;
  params.safe_distance = kCircumscribedRadius;
  params.min_safe_distance = kInscribedRadius;

  ASSERT_NEAR(minco_planner::measureGridClearance(grid, 25, 5, 1.0, policy()), 0.10, 1e-3);
  const auto result =
    minco_planner::GridJps(params).plan(grid, poseAt(0.55, 1.55), poseAt(2.55, 0.55));
  ASSERT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal occupied");
}

TEST(GridClearanceField, MatchesBruteForceClearanceRing)
{
  auto grid = makeOpenGrid(37, 23);
  // 不规则障碍 + 一个未知格，覆盖边界、斜向与孤立格。
  blockRow(grid, 5, 3, 20);
  block(grid, 30, 15);
  block(grid, 12, 18);
  block(grid, 13, 19);
  grid.data[static_cast<std::size_t>(2) * grid.info.width + 33] = -1;
  minco_planner::GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = kThreshold;
  policy.unknown_is_obstacle = true;
  const auto squared = minco_planner::computeBlockedSquaredDistanceCells(grid, policy);
  for (const double radius : {0.0, 0.15, 0.27, 0.3, 0.4187, 0.55}) {
    const double radius_cells = radius / grid.info.resolution;
    for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
      for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
        const bool expected = minco_planner::hasGridClearance(grid, x, y, radius, policy);
        const bool free = minco_planner::isGridCellFree(grid, x, y, policy);
        const bool from_field = free &&
          (!(radius > 0.0) || squared[static_cast<std::size_t>(y) * grid.info.width + x] >
          radius_cells * radius_cells);
        ASSERT_EQ(from_field, expected) << "x=" << x << " y=" << y << " r=" << radius;
      }
    }
  }
}

namespace
{
// 2.0 m 宽、6 m 长的东西向走廊，北墙在 y 格 0、南墙在 y 格 21（y=0.05..2.15）；
// 起终点都贴近北墙（y=1.75）。
nav_msgs::msg::OccupancyGrid makeCorridorGrid()
{
  auto grid = makeOpenGrid(60, 22);
  blockRow(grid, 0, 0, 59);
  blockRow(grid, 21, 0, 59);
  // 走廊中段一个从北墙伸出的凸角，最短路会贴着它的角点绕过。
  for (int y = 13; y <= 20; ++y) {
    block(grid, 30, y);
  }
  return grid;
}

double minimumPathClearance(
  const nav_msgs::msg::OccupancyGrid & grid, const nav_msgs::msg::Path & path)
{
  minco_planner::GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = kThreshold;
  const auto squared = minco_planner::computeBlockedSquaredDistanceCells(grid, policy);
  double minimum = 1e9;
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    minimum = std::min(
      minimum, minco_planner::blockedDistanceAt(
        grid, squared, path.poses[i].pose.position.x, path.poses[i].pose.position.y));
  }
  return minimum;
}
}  // namespace

TEST(GridAstarClearanceCost, CostKeepsPathAwayFromConvexCorner)
{
  const auto grid = makeCorridorGrid();
  minco_planner::GridAstarParams params;
  params.obstacle_value_threshold = kThreshold;
  params.safe_distance = 0.3;
  const auto start = poseAt(0.55, 1.55);
  const auto goal = poseAt(5.45, 1.55);

  const auto shortest = minco_planner::GridAstar(params).plan(grid, start, goal);
  ASSERT_TRUE(shortest.success);
  params.clearance_cost_weight = 4.0;
  params.clearance_cost_distance = 0.8;
  const auto centered = minco_planner::GridAstar(params).plan(grid, start, goal);
  ASSERT_TRUE(centered.success);

  const double shortest_clearance = minimumPathClearance(grid, shortest.path);
  const double centered_clearance = minimumPathClearance(grid, centered.path);
  // 最短路贴着凸角走到安全净空下限附近；离墙代价让它多留出空间。
  EXPECT_LT(shortest_clearance, 0.45);
  EXPECT_GT(centered_clearance, shortest_clearance + 0.15);
  // 只是更居中，不是绕远路。
  EXPECT_LT(centered.length, shortest.length * 1.3);
}

TEST(GridAstarClearanceCost, DisabledCostKeepsShortestPathLength)
{
  const auto grid = makeCorridorGrid();
  minco_planner::GridAstarParams params;
  params.obstacle_value_threshold = kThreshold;
  params.safe_distance = 0.3;
  const auto start = poseAt(0.55, 1.55);
  const auto goal = poseAt(5.45, 1.55);
  const auto baseline = minco_planner::GridAstar(params).plan(grid, start, goal);
  params.clearance_cost_distance = 0.8;  // 权重为 0：关闭
  const auto disabled = minco_planner::GridAstar(params).plan(grid, start, goal);
  ASSERT_TRUE(baseline.success);
  ASSERT_TRUE(disabled.success);
  EXPECT_DOUBLE_EQ(disabled.length, baseline.length);
}

namespace
{

double pathCost(
  const nav_msgs::msg::OccupancyGrid & grid, const nav_msgs::msg::Path & path,
  const std::vector<double> & blocked_squared, const minco_planner::GridAstarParams & params)
{
  const auto index = [&](const geometry_msgs::msg::PoseStamped & pose) {
      const auto & p = pose.pose.position;
      return std::make_pair(
        static_cast<int>(std::floor(p.x / grid.info.resolution)),
        static_cast<int>(std::floor(p.y / grid.info.resolution)));
    };
  double total = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto a = index(path.poses[i - 1]);
    const auto b = index(path.poses[i]);
    const int dx = std::abs(a.first - b.first);
    const int dy = std::abs(a.second - b.second);
    EXPECT_LE(dx, 1);
    EXPECT_LE(dy, 1);
    if (a == b) {
      continue;
    }
    const double distance = std::sqrt(
      blocked_squared[b.second * grid.info.width + b.first]) * grid.info.resolution;
    double factor = 1.0;
    if (distance < params.clearance_cost_distance && params.clearance_cost_weight > 0.0) {
      const double deficit =
        (params.clearance_cost_distance - distance) / params.clearance_cost_distance;
      factor += params.clearance_cost_weight * deficit * deficit;
    }
    total += (dx && dy ? params.diagonal_cost : 1.0) * factor;
  }
  return total;
}

void expectSameAstarPath(
  const minco_planner::GridAstarResult & previous,
  const minco_planner::GridSearchResult & unified)
{
  ASSERT_EQ(previous.success, unified.success);
  EXPECT_EQ(previous.reason, unified.reason);
  ASSERT_EQ(previous.path, unified.path);
  EXPECT_EQ(std::memcmp(&previous.length, &unified.length, sizeof(double)), 0);
  const auto components = [](const auto & pose) {
      return std::array<double, 7>{
        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
        pose.orientation.y, pose.orientation.z, pose.orientation.w};
    };
  for (std::size_t i = 0; i < previous.path.poses.size(); ++i) {
    const auto older = components(previous.path.poses[i].pose);
    const auto newer = components(unified.path.poses[i].pose);
    EXPECT_EQ(std::memcmp(older.data(), newer.data(), older.size() * sizeof(double)), 0);
  }
  if (previous.success) {
    EXPECT_EQ(previous.expanded_nodes, unified.expanded_nodes);
  }
}

}  // namespace

TEST(UnifiedGridSearch, JumpOffMatchesExistingCorridorsAndEndpointRelaxation)
{
  minco_planner::GridSearchParams params;
  params.jump_enabled = false;
  params.safe_distance = kCircumscribedRadius;
  params.min_safe_distance = kInscribedRadius;
  params.unknown_is_obstacle = true;
  params.start_escape_max_steps = 8;
  params.clearance_cost_weight = 4.0;
  params.clearance_cost_distance = 0.8;
  minco_planner::GridSearch search(params);
  minco_planner::GridAstar previous(params);
  for (const auto & grid : {makeWestCorridorGrid(), makeCorridorGrid(), makeOpenGrid(60, 30)}) {
    const auto field = minco_planner::computeBlockedSquaredDistanceCells(
      grid, {params.obstacle_value_threshold, params.unknown_is_obstacle});
    for (const auto & start : {poseAt(0.15, 1.55), poseAt(1.57, 1.55)}) {
      for (const auto & goal : {poseAt(5.43, 1.51), poseAt(0.24, 1.55), poseAt(6.1, 1.55)}) {
        for (const double clearance : {0.0, 0.28, kCircumscribedRadius}) {
          minco_planner::GridSearchOptions options;
          options.clearance = clearance;
          expectSameAstarPath(
            previous.planWithClearance(grid, start, goal, clearance, &field),
            search.plan(grid, start, goal, &field, options));
        }
      }
    }
  }
}

TEST(UnifiedGridSearch, JumpMatchesWeightedAstarOnTheSameRandomizedGrids)
{
  using Clock = std::chrono::steady_clock;
  std::mt19937 generator(0x20271119);
  minco_planner::GridSearch search;
  constexpr int kTrials = 480;
  int reachable = 0;
  int escaped_starts = 0;
  int previous_expanded = 0;
  int hybrid_expanded = 0;
  double previous_ms = 0.0;
  double hybrid_ms = 0.0;
  double max_cost_difference = 0.0;
  for (int trial = 0; trial < kTrials; ++trial) {
    SCOPED_TRACE(trial);
    const bool full_field = trial % 12 == 0;
    const int width = full_field ? 280 : 32 + trial % 7;
    const int height = full_field ? 160 : 22 + trial % 5;
    auto grid = makeOpenGrid(width, height);
    const auto set = [&](int x, int y, int value) {grid.data[y * width + x] = value;};
    for (int i = 0; i < (full_field ? 20 : 6); ++i) {
      const int x = 6 + generator() % (width - 12);
      const int y = 2 + generator() % (height - 5);
      const int value = i % 3 == 0 ? -1 : 100;
      for (int dy = 0; dy < 3; ++dy) {
        for (int dx = 0; dx < 3; ++dx) {
          set(x + dx, y + dy, value);
        }
      }
    }
    if (trial % 2 == 0 && !full_field) {
      const int gap = 4 + generator() % (height - 10);
      for (int y = 0; y < height; ++y) {
        if (y < gap || y >= gap + (trial % 20 == 0 ? 2 : 7)) {
          set(width / 2, y, 100);
        }
      }
    }
    minco_planner::GridSearchParams params;
    params.safe_distance = trial % 3 == 0 ? 0.21 : 0.0;
    params.unknown_is_obstacle = trial % 7 != 0;
    params.start_escape_max_steps = 6;
    params.clearance_cost_weight = trial % 5 == 0 ? 0.0 : 4.0;
    params.clearance_cost_distance = full_field ? 1.0 : 0.4;
    search.setParams(params);
    const auto field = minco_planner::computeBlockedSquaredDistanceCells(
      grid, {params.obstacle_value_threshold, params.unknown_is_obstacle});
    const int start_x = trial % 3 == 0 ? 0 : 3;
    const auto start = poseAt((start_x + 0.5) * grid.info.resolution,
        (5.5 + generator() % (height / 2)) * grid.info.resolution);
    const auto goal = poseAt((width - 4.5) * grid.info.resolution,
        (4.5 + generator() % (height - 8)) * grid.info.resolution);

    auto begin = Clock::now();
    const auto previous = minco_planner::GridAstar(params).planWithClearance(
      grid, start, goal, -1.0, &field);
    previous_ms += std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    begin = Clock::now();
    const auto hybrid = search.plan(grid, start, goal, &field);
    hybrid_ms += std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    ASSERT_EQ(previous.success, hybrid.success);
    previous_expanded += previous.expanded_nodes;
    hybrid_expanded += hybrid.expanded_nodes;

    params.jump_enabled = false;
    search.setParams(params);
    expectSameAstarPath(previous, search.plan(grid, start, goal, &field));
    if (previous.success) {
      ++reachable;
      escaped_starts += start_x == 0;
      const double expected = pathCost(grid, previous.path, field, params);
      EXPECT_NEAR(pathCost(grid, hybrid.path, field, params), hybrid.cost, 1e-9);
      EXPECT_NEAR(expected, hybrid.cost, 1e-9);
      max_cost_difference = std::max(max_cost_difference, std::abs(expected - hybrid.cost));
    }
  }
  EXPECT_GT(reachable, 250);
  EXPECT_GT(escaped_starts, 50);
  std::cout << "Unified search Release comparison: grids=" << kTrials << ", reachable=" <<
    reachable << ", escaped_starts=" << escaped_starts << ", max_cost_delta=" <<
    max_cost_difference << ", legacy_Astar_expanded=" << previous_expanded <<
    ", hybrid_expanded=" << hybrid_expanded << ", legacy_Astar_wall_ms=" << previous_ms <<
    ", hybrid_wall_ms=" << hybrid_ms << '\n';
}

TEST(UnifiedGridSearch, OutsideGoalReturnsTheClosestKnownFreeStop)
{
  auto grid = makeOpenGrid(40, 28);
  for (std::uint32_t y = 0; y < grid.info.height; ++y) {
    std::fill_n(grid.data.begin() + y * grid.info.width + 30, 10, -1);
  }
  minco_planner::GridSearchParams params;
  params.unknown_is_obstacle = true;
  params.safe_distance = 0.31;
  minco_planner::FootprintSafetyParams footprint;
  footprint.length = 0.38;
  footprint.width = 0.36;
  footprint.safety_margin = 0.02;
  footprint.unknown_is_obstacle = true;
  minco_planner::GridSearchOptions options;
  options.can_stop = [&](const auto & pose) {
      return minco_planner::goalPoseFootprintFree(
        pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation),
        grid, footprint);
    };
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  const auto start = poseAt(0.75, 1.45);
  const auto outside = poseAt(8.0, 1.45);
  for (const bool jump : {false, true}) {
    params.jump_enabled = jump;
    minco_planner::GridSearch search(params);
    EXPECT_FALSE(search.plan(grid, start, outside, &field).partial);
    const auto result = search.plan(grid, start, outside, &field, options);
    EXPECT_FALSE(result.success);
    ASSERT_TRUE(result.partial) << result.reason;
    EXPECT_EQ(result.reason, "partial: goal is outside grid");
    ASSERT_GT(result.path.poses.size(), 1U);
    const auto & end = result.path.poses.back();
    EXPECT_NEAR(end.pose.position.x, 2.65, 1e-6);
    EXPECT_NEAR(end.pose.position.y, 1.45, 1e-6);
    EXPECT_TRUE(options.can_stop(end));
    EXPECT_EQ(result.path.poses.front(), start);
  }
}

TEST(UnifiedGridSearch, DisconnectedGoalOnlyReturnsAnAdmittedPartialStop)
{
  auto grid = makeOpenGrid(36, 28);
  for (int y = 0; y < 28; ++y) {
    block(grid, 20, y);
  }
  const auto field = minco_planner::computeBlockedSquaredDistanceCells(grid, {50, true});
  minco_planner::GridSearchParams params;
  params.unknown_is_obstacle = true;
  params.jump_enabled = false;
  minco_planner::GridSearch search(params);
  minco_planner::GridSearchOptions options;
  const auto start = poseAt(0.55, 1.35);
  const auto goal = poseAt(2.85, 1.35);
  options.can_stop = [](const auto &) {return false;};
  const auto rejected = search.plan(grid, start, goal, &field, options);
  EXPECT_FALSE(rejected.success);
  EXPECT_FALSE(rejected.partial);
  EXPECT_TRUE(rejected.path.poses.empty());
  options.can_stop = [](const auto & pose) {return pose.pose.position.x < 1.7;};
  const auto partial = search.plan(grid, start, goal, &field, options);
  ASSERT_TRUE(partial.partial);
  EXPECT_FALSE(partial.success);
  EXPECT_EQ(partial.reason, "partial: no path");
  EXPECT_NEAR(partial.path.poses.back().pose.position.x, 1.65, 1e-6);
  EXPECT_NEAR(partial.path.poses.back().pose.position.y, 1.35, 1e-6);
  const auto complete = search.plan(grid, start, poseAt(1.45, 1.35), &field, options);
  EXPECT_TRUE(complete.success);
  EXPECT_FALSE(complete.partial);
}
