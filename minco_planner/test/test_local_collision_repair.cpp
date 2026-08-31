// Copyright 2026

#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"

namespace
{

using minco_planner::FootprintSafetyChecker;
using minco_planner::FootprintSafetyParams;
using minco_planner::FootprintSafetyResult;
using minco_planner::LocalCollisionRepair;
using minco_planner::LocalCollisionRepairParams;
using minco_planner::ReferencePoint;
using minco_planner::ReferenceTrajectory;

// MuJoCo RMUC profile 的几何:0.60 x 0.50 加 0.02 余量,
// 全 yaw 外接半径 hypot(0.32, 0.27) = 0.4187 m。
constexpr double kAllYawRadius = 0.41869;
constexpr double kResolution = 0.1;
// 内切半宽 0.5*0.50 + 0.02 = 0.27,第二档下限 0.27 + 0.1*sqrt(1/2) = 0.340711 m。
constexpr double kInscribedRadius = 0.27;

nav_msgs::msg::OccupancyGrid makeGrid(std::uint32_t width = 40, std::uint32_t height = 40)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = kResolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

void fillColumn(nav_msgs::msg::OccupancyGrid & grid, int column, int8_t value)
{
  for (std::uint32_t row = 0; row < grid.info.height; ++row) {
    grid.data[static_cast<std::size_t>(row) * grid.info.width + column] = value;
  }
}

// 格心坐标,和 findNearestClearCell 的返回约定一致。
// 必须走 OccupancyGrid 自己的 float32 分辨率:直接用 double
// 常量会带进约 1.5e-8 的量化差,让"落在哪个格子"的断言变成
// 浮点公差之争,而不是几何判定。
double cellCenter(const nav_msgs::msg::OccupancyGrid & grid, int index)
{
  return (static_cast<double>(index) + 0.5) * static_cast<double>(grid.info.resolution);
}

ReferenceTrajectory makeStraightTrajectory(
  double x, double start_y, double end_y, double yaw)
{
  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  ReferencePoint first;
  first.x = x;
  first.y = start_y;
  first.yaw = yaw;
  ReferencePoint second = first;
  second.y = end_y;
  second.t = 1.0;
  second.s = std::abs(end_y - start_y);
  trajectory.points = {first, second};
  return trajectory;
}

// 端点保护之后,修复只对内部引导点生效:index 0 是机器人当前位姿,末点是
// action 下发的目标位姿。所以用例把待修复点放在中间两个采样上,首末点锚在
// "修复成功后应该落到的那一列",于是"修复成功"等价于整条轨迹被拉直。
ReferenceTrajectory makeGuidedTrajectory(
  double anchor_x, double point_x, double start_y, double end_y, double yaw)
{
  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  const double span = std::abs(end_y - start_y);
  ReferencePoint head;
  head.x = anchor_x;
  head.y = start_y - 0.2;
  head.yaw = yaw;
  ReferencePoint first = head;
  first.x = point_x;
  first.y = start_y;
  first.t = 1.0;
  first.s = 0.2;
  ReferencePoint second = first;
  second.y = end_y;
  second.t = 2.0;
  second.s = 0.2 + span;
  ReferencePoint tail = second;
  tail.x = anchor_x;
  tail.y = end_y + 0.2;
  tail.t = 3.0;
  tail.s = 0.4 + span;
  trajectory.points = {head, first, second, tail};
  return trajectory;
}

// 只标记内部采样为冲突。端点保护的行为由专门的用例覆盖,不应该混进
// "挑哪个候选格"的判据用例里。
FootprintSafetyResult collideInteriorPoints(const ReferenceTrajectory & trajectory)
{
  FootprintSafetyResult result;
  result.safe = false;
  for (std::size_t index = 1; index + 1 < trajectory.points.size(); ++index) {
    minco_planner::CollisionSample sample;
    sample.trajectory_index = index;
    sample.x = trajectory.points[index].x;
    sample.y = trajectory.points[index].y;
    result.collisions.push_back(sample);
  }
  return result;
}

FootprintSafetyResult collideEveryPoint(const ReferenceTrajectory & trajectory)
{
  FootprintSafetyResult result;
  result.safe = false;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    minco_planner::CollisionSample sample;
    sample.trajectory_index = index;
    sample.x = trajectory.points[index].x;
    sample.y = trajectory.points[index].y;
    result.collisions.push_back(sample);
  }
  return result;
}

LocalCollisionRepair makeRepair(
  double required_clearance_m, double search_radius = 0.35,
  double inscribed_radius_m = 0.0)
{
  LocalCollisionRepairParams params;
  params.enabled = true;
  params.search_radius = search_radius;
  params.obstacle_value_threshold = 100;
  params.unknown_is_obstacle = false;
  params.required_clearance_m = required_clearance_m;
  params.inscribed_radius_m = inscribed_radius_m;
  return LocalCollisionRepair(params);
}

TEST(LocalCollisionRepair, PointOccupancyOnlyRepairKeepsTheCollidingCell)
{
  // domain 193 目标 8 的形态:冲突点自身格子是空闲的,墙在西侧 4 格,
  // 矩形足迹却已经扫进墙里。只按点占据搜索时最近的"空闲格"就是原格,
  // 修复毫无位移,重新求解后必然被同一条门禁再拒一次。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(0.0);
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 8), 1e-9);
}

TEST(LocalCollisionRepair, RequiredClearanceMovesThePointOffTheWall)
{
  // 同一场景加上全 yaw 外接半径要求:候选格必须整个净空圆内无占据。
  // 半径 0.4187 m、分辨率 0.1 m 时圆盘横向覆盖到第 4 格,所以 i=9 仍不合格,
  // i=10 才是第一个合格候选,修复位移 0.20 m,仍在 search_radius 之内。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 10), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  ASSERT_TRUE(repair.repair(trajectory, collisions, grid));
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.x, cellCenter(grid, 10), 1e-9);
  }
}

TEST(LocalCollisionRepair, RepairedTrajectoryPassesTheRectangularFootprintGate)
{
  // 端到端:用真实门禁参数确认修复后的几何真能
  // 通过,而不是只"挪了一下"。这是本轮唯一能让
  // FAILURE_FOOTPRINT 收敛的路径;门禁复检仍在
  // 节点里无条件执行。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 10), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);

  FootprintSafetyParams gate_params;
  gate_params.length = 0.60;
  gate_params.width = 0.50;
  gate_params.safety_margin = 0.02;
  gate_params.obstacle_value_threshold = 100;
  gate_params.unknown_is_obstacle = false;
  gate_params.swept_max_corner_step_cells = 0.5;
  const FootprintSafetyChecker gate(gate_params);

  const auto before = gate.check(trajectory, grid);
  ASSERT_FALSE(before.safe);

  const auto repair = makeRepair(kAllYawRadius);
  ASSERT_TRUE(repair.repair(trajectory, before, grid));

  const auto after = gate.check(trajectory, grid);
  EXPECT_TRUE(after.safe);
  EXPECT_TRUE(after.collisions.empty());
}

TEST(LocalCollisionRepair, NarrowCorridorWithoutAnyClearCellStaysFailClosed)
{
  // 走廊窄到没有任何格子能容纳全 yaw 圆盘时,修复必须返回 false,
  // 让上层沿用今天的拒绝分支,而不是挪到一个照样过不了门禁的位置。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  fillColumn(grid, 11, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 8), 1e-9);
}

TEST(LocalCollisionRepair, OutsideTheGridCountsAsOccupied)
{
  // 越界必须按障碍处理,否则修复会把点推到地图
  // 边缘之外的"看起来很空"的地方。空地图里贴边
  // 的点因此只能向内挪,不能停在边上。
  auto grid = makeGrid();
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 4), cellCenter(grid, 1), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  ASSERT_TRUE(repair.repair(trajectory, collisions, grid));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 4), 1e-9);
}

TEST(LocalCollisionRepair, SearchRadiusTooSmallStaysFailClosed)
{
  // 合格候选存在但超出 search_radius 时同样保持拒绝,不放宽搜索窗口。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius, 0.1);
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 8), 1e-9);
}

TEST(LocalCollisionRepair, FallbackTierRepairsWhereTheAllYawTierIsEmpty)
{
  // 走廊放得下内切圆但放不下全 yaw 外接圆:严格档是空集,第二档能挑出
  // 走廊中心。这是 domain 169 目标 9 与 domain 171 目标 8 的形态——严格档
  // 恒空导致 89 次连续拒绝里修复一次都没动过轨迹。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  fillColumn(grid, 13, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 9), cellCenter(grid, 7), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto strict_only = makeRepair(kAllYawRadius);
  auto untouched = trajectory;
  EXPECT_FALSE(strict_only.repair(untouched, collisions, grid));
  EXPECT_NEAR(untouched.points[1].x, cellCenter(grid, 7), 1e-9);

  const auto two_tier = makeRepair(kAllYawRadius, 0.35, kInscribedRadius);
  minco_planner::LocalCollisionRepairStats stats;
  ASSERT_TRUE(two_tier.repair(trajectory, collisions, grid, &stats));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 9), 1e-9);
  EXPECT_EQ(stats.collision_points, 2U);
  EXPECT_EQ(stats.strict_repaired, 0U);
  EXPECT_EQ(stats.fallback_repaired, 2U);
  EXPECT_EQ(stats.no_candidate, 0U);
  EXPECT_NEAR(stats.fallback_clearance_m, 0.340711, 1e-6);
}

TEST(LocalCollisionRepair, StrictTierWinsWheneverItHasACandidate)
{
  // 第二档只在严格档挑不出候选时使用,不能把已经能用的严格档挤掉。
  auto grid = makeGrid();
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 4), cellCenter(grid, 1), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius, 0.35, kInscribedRadius);
  minco_planner::LocalCollisionRepairStats stats;
  ASSERT_TRUE(repair.repair(trajectory, collisions, grid, &stats));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 4), 1e-9);
  EXPECT_EQ(stats.strict_repaired, 2U);
  EXPECT_EQ(stats.fallback_repaired, 0U);
}

TEST(LocalCollisionRepair, BothTiersEmptyStaysFailClosedAndIsObservable)
{
  // 走廊连内切圆都放不下时两档都为空,行为与今天一致(拒绝),
  // 但现在 no_candidate 会把原因记下来,不再是无痕迹的静默失败。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  fillColumn(grid, 11, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(kAllYawRadius, 0.35, kInscribedRadius);
  minco_planner::LocalCollisionRepairStats stats;
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid, &stats));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 8), 1e-9);
  EXPECT_EQ(stats.collision_points, 2U);
  EXPECT_EQ(stats.no_candidate, 2U);
  EXPECT_EQ(stats.strict_repaired, 0U);
  EXPECT_EQ(stats.fallback_repaired, 0U);
}

TEST(LocalCollisionRepair, NegligibleShiftIsCountedSeparatelyFromNoCandidate)
{
  // 只按点占据修复时会挑回原格,位移可忽略。这与"挑不出候选格"是两种
  // 不同的失败,必须分开计数,否则日志无法定位是判据太严还是判据太松。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  const auto repair = makeRepair(0.0);
  minco_planner::LocalCollisionRepairStats stats;
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid, &stats));
  EXPECT_EQ(stats.collision_points, 2U);
  EXPECT_EQ(stats.negligible_shift, 2U);
  EXPECT_EQ(stats.no_candidate, 0U);
}

TEST(LocalCollisionRepair, DisabledRepairNeverTouchesTheTrajectory)
{
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideInteriorPoints(trajectory);

  LocalCollisionRepairParams params;
  params.enabled = false;
  params.required_clearance_m = kAllYawRadius;
  const LocalCollisionRepair repair(params);
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid));
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 8), 1e-9);
}

TEST(LocalCollisionRepair, StartAndTerminalPointsAreNeverRelocated)
{
  // 端点保护:index 0 是机器人当前位姿,末点是 action 下发的目标位姿。
  // 同样的几何在内部点上会被挪到第 10 格(见 RequiredClearanceMovesThePointOffTheWall),
  // 但作为端点时必须原地不动,并且这个跳过要可观测。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeStraightTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 10), cellCenter(grid, 12), 0.0);
  const auto collisions = collideEveryPoint(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  minco_planner::LocalCollisionRepairStats stats;
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid, &stats));
  EXPECT_NEAR(trajectory.points.front().x, cellCenter(grid, 8), 1e-9);
  EXPECT_NEAR(trajectory.points.back().x, cellCenter(grid, 8), 1e-9);
  EXPECT_EQ(stats.collision_points, 2U);
  EXPECT_EQ(stats.endpoint_protected, 2U);
  EXPECT_EQ(stats.strict_repaired, 0U);
  EXPECT_EQ(stats.fallback_repaired, 0U);
  EXPECT_EQ(stats.no_candidate, 0U);
}

TEST(LocalCollisionRepair, TerminalRotationSamplesAreNeverRelocated)
{
  // domain 173 目标 8 的实测形态:yaw 规划在终点位置追加了一串原地转向采样,
  // 它们与末点同坐标。旧行为只挪走其中一部分(另一部分因位移可忽略被跳过),
  // 引导点序列因此变成 目标->东侧->目标 的回头刺,重解出的末段反向切入目标,
  // 终端 yaw 被顶到墙上。整段与末点重合的采样必须一起保护。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  const double goal_x = cellCenter(grid, 8);
  const double goal_y = cellCenter(grid, 12);

  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  ReferencePoint head;
  head.x = goal_x;
  head.y = cellCenter(grid, 10);
  trajectory.points.push_back(head);
  ReferencePoint terminal = head;
  terminal.y = goal_y;
  terminal.t = 1.0;
  terminal.s = std::abs(goal_y - head.y);
  trajectory.points.push_back(terminal);
  for (int index = 1; index <= 5; ++index) {
    ReferencePoint sample = terminal;
    sample.t = 1.0 + 0.1 * index;
    sample.yaw = 0.2 * index;
    trajectory.points.push_back(sample);
  }
  ASSERT_TRUE(trajectory.valid());
  const auto collisions = collideEveryPoint(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  minco_planner::LocalCollisionRepairStats stats;
  EXPECT_FALSE(repair.repair(trajectory, collisions, grid, &stats));
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.x, goal_x, 1e-9);
  }
  EXPECT_EQ(stats.collision_points, 7U);
  // index 0 加上 6 个与末点重合的采样(平移末点自身也在内)。
  EXPECT_EQ(stats.endpoint_protected, 7U);
  EXPECT_EQ(stats.strict_repaired, 0U);
}

TEST(LocalCollisionRepair, InteriorPointsStillMoveWhileEndpointsStayPinned)
{
  // 端点保护不能顺手把修复本身关掉:同一条轨迹里内部点照常挪,端点照常钉住。
  auto grid = makeGrid();
  fillColumn(grid, 5, 100);
  auto trajectory = makeGuidedTrajectory(
    cellCenter(grid, 8), cellCenter(grid, 8), cellCenter(grid, 10),
    cellCenter(grid, 12), 0.0);
  const auto collisions = collideEveryPoint(trajectory);

  const auto repair = makeRepair(kAllYawRadius);
  minco_planner::LocalCollisionRepairStats stats;
  ASSERT_TRUE(repair.repair(trajectory, collisions, grid, &stats));
  EXPECT_NEAR(trajectory.points.front().x, cellCenter(grid, 8), 1e-9);
  EXPECT_NEAR(trajectory.points.back().x, cellCenter(grid, 8), 1e-9);
  EXPECT_NEAR(trajectory.points[1].x, cellCenter(grid, 10), 1e-9);
  EXPECT_NEAR(trajectory.points[2].x, cellCenter(grid, 10), 1e-9);
  EXPECT_EQ(stats.collision_points, 4U);
  EXPECT_EQ(stats.endpoint_protected, 2U);
  EXPECT_EQ(stats.strict_repaired, 2U);
}

}  // namespace
