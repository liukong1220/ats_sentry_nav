// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_clearance.hpp"
#include "minco_planner/planning/grid_jps.hpp"

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
