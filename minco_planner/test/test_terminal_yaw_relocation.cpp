// Copyright 2026

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/terminal_yaw_relocation.hpp"

namespace
{

using minco_planner::FootprintSafetyChecker;
using minco_planner::FootprintSafetyParams;
using minco_planner::FootprintSafetyResult;
using minco_planner::ReferencePoint;
using minco_planner::ReferenceTrajectory;
using minco_planner::TerminalYawRelocationParams;
using minco_planner::collisionsConfinedToTerminalApproach;
using minco_planner::collisionsConfinedToTerminalRotation;
using minco_planner::relocateTerminalYawRotation;
using minco_planner::terminalApproachWindowStart;
using minco_planner::terminalCoincidentTailStart;
using minco_planner::terminalYawRelocationCandidates;

constexpr double kResolution = 0.1;

TerminalYawRelocationParams makeParams()
{
  TerminalYawRelocationParams params;
  params.yaw_rate_limit = 2.5;
  params.sample_period = 0.10;
  return params;
}

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = kResolution;
  grid.info.width = 40;
  grid.info.height = 40;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  return grid;
}

void fillColumn(nav_msgs::msg::OccupancyGrid & grid, int column, int8_t value)
{
  for (std::uint32_t row = 0; row < grid.info.height; ++row) {
    grid.data[static_cast<std::size_t>(row) * grid.info.width + column] = value;
  }
}

// MuJoCo RMUC profile 的门禁参数,与 rmuc_2025_navigation.yaml 一致。
FootprintSafetyChecker makeGate()
{
  FootprintSafetyParams params;
  params.length = 0.60;
  params.width = 0.50;
  params.safety_margin = 0.02;
  params.obstacle_value_threshold = 100;
  params.unknown_is_obstacle = false;
  params.swept_max_corner_step_cells = 0.5;
  return FootprintSafetyChecker(params);
}

// 一段向西的通道内平移,yaw 固定为通道切线(π)。窄通道 yaw 模式就是这个形态。
ReferenceTrajectory makeCorridorApproach(
  double start_x, double goal_x, double y, double corridor_yaw)
{
  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  constexpr int kCount = 7;
  for (int index = 0; index < kCount; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(kCount - 1);
    ReferencePoint point;
    point.x = start_x + (goal_x - start_x) * u;
    point.y = y;
    point.yaw = corridor_yaw;
    point.t = 0.5 * index;
    point.s = std::abs(point.x - start_x);
    point.v = (index + 1 < kCount) ? 0.5 : 0.0;
    point.vx = -point.v;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// 复刻 YawSplinePlanner::appendTerminalGoalYawTransition 的输出形态:
// 一串与末点同坐标、零平移、yaw 五次多项式过渡到 goal_yaw 的采样。
void appendTerminalRotation(
  ReferenceTrajectory & trajectory, double goal_yaw, const TerminalYawRelocationParams & params)
{
  const ReferencePoint terminal = trajectory.points.back();
  double delta = goal_yaw - terminal.yaw;
  while (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  while (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }
  const double duration = 1.875 * std::abs(delta) / params.yaw_rate_limit;
  const int sample_count = std::max(
    2, static_cast<int>(std::ceil(duration / params.sample_period)));
  for (int step = 1; step <= sample_count; ++step) {
    const double u = static_cast<double>(step) / static_cast<double>(sample_count);
    const double u3 = u * u * u;
    const double blend = 10.0 * u3 - 15.0 * u3 * u + 6.0 * u3 * u * u;
    ReferencePoint point = terminal;
    point.t = terminal.t + duration * u;
    point.s = terminal.s;
    point.v = 0.0;
    point.vx = 0.0;
    point.vy = 0.0;
    point.yaw = terminal.yaw + delta * blend;
    trajectory.points.push_back(point);
  }
}

// domain 173 目标 8 的最小复现几何:目标到墙的净空介于内切半宽 0.32 与全 yaw
// 外接半径 0.4187 之间,所以 yaw=0 可行,但在目标处原地转向必然扫进墙里。
ReferenceTrajectory makeGoalEightAnalogue(const TerminalYawRelocationParams & params)
{
  ReferenceTrajectory trajectory = makeCorridorApproach(1.60, 0.95, 1.05, M_PI);
  appendTerminalRotation(trajectory, 0.0, params);
  return trajectory;
}

// domain 189 目标 9 的最小复现几何:目标东侧有墙,墙面在 x=1.50 m。目标位姿
// (1.17, 1.05, yaw=0) 的前缘为 1.49 m,可行;但通道切线 yaw=-0.098 rad 让矩形
// 前向外扩从 e(0)=0.320 m 涨到 e(-0.098)=0.345 m,左前角越过墙面。实测越界量
// 只有 1 mm,冲突落在末点前的平移段而不是与末点重合的原地转向段,因此旧触发
// 条件一次都没触发过。
ReferenceTrajectory makeGoalNineAnalogue(const TerminalYawRelocationParams & params)
{
  ReferenceTrajectory trajectory;
  trajectory.header.frame_id = "map";
  constexpr int kCount = 9;
  for (int index = 0; index < kCount; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(kCount - 1);
    ReferencePoint point;
    point.x = 1.10 + 0.07 * u;
    point.y = 0.85 + 0.20 * u;
    point.yaw = -0.098;
    point.t = 0.5 * index;
    point.s = std::hypot(point.x - 1.10, point.y - 0.85);
    point.v = (index + 1 < kCount) ? 0.4 : 0.0;
    point.vy = point.v;
    trajectory.points.push_back(point);
  }
  appendTerminalRotation(trajectory, 0.0, params);
  return trajectory;
}

TEST(TerminalYawRelocation, ApproachWindowSkipsTheZeroLengthRotationBlock)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  // 7 个平移点跨 0.65 m,相邻间距 0.10833 m;原地转向段坐标重合,不消耗预算。
  // 0.30 m 预算可以退回到下标 4,再退一步累计 0.325 m 就超预算。
  EXPECT_EQ(terminalApproachWindowStart(trajectory, 0.30), 4U);
  EXPECT_EQ(terminalApproachWindowStart(trajectory, 5.00), 0U);
  EXPECT_EQ(
    terminalApproachWindowStart(trajectory, 0.0), trajectory.points.size() - 1U);
}

TEST(TerminalYawRelocation, CollisionsInTheTranslationalRunInNeedTheApproachWindow)
{
  const auto params = makeParams();
  auto grid = makeGrid();
  fillColumn(grid, 15, 100);
  fillColumn(grid, 16, 100);
  const auto trajectory = makeGoalNineAnalogue(params);
  const FootprintSafetyResult before = makeGate().check(trajectory, grid);
  ASSERT_FALSE(before.safe);
  ASSERT_FALSE(before.collisions.empty());
  std::size_t earliest = trajectory.points.size();
  for (const auto & collision : before.collisions) {
    earliest = std::min(earliest, collision.trajectory_index);
  }
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  // 最早的冲突在末点之前的平移段里,所以旧判据把整条轨迹判成"中途不可行"。
  EXPECT_LT(earliest, tail);
  EXPECT_FALSE(collisionsConfinedToTerminalRotation(trajectory, before));
  EXPECT_TRUE(collisionsConfinedToTerminalApproach(trajectory, before, 1.0));
  // 窗口是真实的边界:预算小到只覆盖末点时,同一组冲突仍然被交回拒绝路径。
  EXPECT_FALSE(collisionsConfinedToTerminalApproach(trajectory, before, 0.01));
}

TEST(TerminalYawRelocation, AnEarlierRotationClearsTheTangentYawApproachToAWall)
{
  const auto params = makeParams();
  auto grid = makeGrid();
  fillColumn(grid, 15, 100);
  fillColumn(grid, 16, 100);
  const auto gate = makeGate();
  const auto trajectory = makeGoalNineAnalogue(params);
  ASSERT_FALSE(gate.check(trajectory, grid).safe);
  bool cleared = false;
  std::size_t accepted_index = trajectory.points.size();
  for (const std::size_t rotation_index : terminalYawRelocationCandidates(trajectory, 6U)) {
    ReferenceTrajectory candidate;
    if (!relocateTerminalYawRotation(trajectory, 0.0, rotation_index, params, &candidate)) {
      continue;
    }
    if (gate.check(candidate, grid).safe) {
      cleared = true;
      accepted_index = rotation_index;
      // 末点位姿没有被改动:重定位只改到达目标的 yaw 时序。
      EXPECT_NEAR(candidate.points.back().x, trajectory.points.back().x, 1e-9);
      EXPECT_NEAR(candidate.points.back().y, trajectory.points.back().y, 1e-9);
      EXPECT_NEAR(candidate.points.back().yaw, 0.0, 1e-9);
      break;
    }
  }
  EXPECT_TRUE(cleared);
  EXPECT_LT(accepted_index, terminalCoincidentTailStart(trajectory));
}

TEST(TerminalYawRelocation, TailStartFindsTheAppendedRotationBlock)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  // 7 个平移点 + 追加的原地转向段,尾段起点就是最后一个平移点(下标 6)。
  EXPECT_EQ(terminalCoincidentTailStart(trajectory), 6U);
  EXPECT_GT(trajectory.points.size(), 7U);
}

TEST(TerminalYawRelocation, TailStartWithoutARotationBlockIsTheLastIndex)
{
  const auto trajectory = makeCorridorApproach(1.60, 0.95, 1.05, M_PI);
  EXPECT_EQ(terminalCoincidentTailStart(trajectory), trajectory.points.size() - 1U);
}

TEST(TerminalYawRelocation, TailStartOfAnEmptyTrajectoryIsZero)
{
  const ReferenceTrajectory trajectory;
  EXPECT_EQ(terminalCoincidentTailStart(trajectory), 0U);
}

TEST(TerminalYawRelocation, CollisionsInsideTheRotationBlockAreConfined)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  FootprintSafetyResult safety;
  safety.safe = false;
  minco_planner::CollisionSample sample;
  sample.trajectory_index = trajectory.points.size() - 3U;
  safety.collisions.push_back(sample);
  EXPECT_TRUE(collisionsConfinedToTerminalRotation(trajectory, safety));
}

TEST(TerminalYawRelocation, CollisionsBeforeTheRotationBlockAreNotConfined)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  FootprintSafetyResult safety;
  safety.safe = false;
  minco_planner::CollisionSample early;
  early.trajectory_index = 2U;
  minco_planner::CollisionSample late;
  late.trajectory_index = trajectory.points.size() - 2U;
  safety.collisions = {early, late};
  // 冲突出现在通道中段说明路径本身不可行,重定位不该介入掩盖问题。
  EXPECT_FALSE(collisionsConfinedToTerminalRotation(trajectory, safety));
}

TEST(TerminalYawRelocation, TrajectoryWithoutARotationBlockIsNotConfined)
{
  const auto trajectory = makeCorridorApproach(1.60, 0.95, 1.05, M_PI);
  FootprintSafetyResult safety;
  safety.safe = false;
  minco_planner::CollisionSample sample;
  sample.trajectory_index = trajectory.points.size() - 1U;
  safety.collisions.push_back(sample);
  // 末点后面没有追加转向段时,是目标位姿本身不可行,重定位无从下手。
  EXPECT_FALSE(collisionsConfinedToTerminalRotation(trajectory, safety));
}

TEST(TerminalYawRelocation, EmptyCollisionSetIsNotConfined)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const FootprintSafetyResult safety;
  EXPECT_FALSE(collisionsConfinedToTerminalRotation(trajectory, safety));
}

TEST(TerminalYawRelocation, RotationAtTheStartThenHoldsGoalYaw)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));
  ASSERT_TRUE(candidate.valid());

  // 第一个点保持机器人当前 yaw,之后原地拧到 goal_yaw,再全程保持。
  EXPECT_NEAR(candidate.points.front().yaw, M_PI, 1e-9);
  EXPECT_NEAR(candidate.points.front().x, trajectory.points.front().x, 1e-9);
  EXPECT_NEAR(candidate.points.back().yaw, 0.0, 1e-9);
  EXPECT_NEAR(candidate.points.back().x, trajectory.points.back().x, 1e-9);
  EXPECT_NEAR(candidate.points.back().y, trajectory.points.back().y, 1e-9);
  // 转向结束之后不允许再有 yaw 变化。
  std::size_t aligned = 0;
  for (const auto & point : candidate.points) {
    if (std::abs(point.yaw) <= 1e-9) {
      ++aligned;
    }
  }
  EXPECT_GE(aligned, 7U);
}

TEST(TerminalYawRelocation, RotationSamplesCommandNoTranslation)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));

  const double pivot_x = trajectory.points.front().x;
  const double pivot_y = trajectory.points.front().y;
  std::size_t rotation_samples = 0;
  for (const auto & point : candidate.points) {
    if (std::abs(point.x - pivot_x) > 1e-9 || std::abs(point.y - pivot_y) > 1e-9) {
      break;
    }
    EXPECT_NEAR(point.v, 0.0, 1e-12);
    EXPECT_NEAR(point.vx, 0.0, 1e-12);
    EXPECT_NEAR(point.vy, 0.0, 1e-12);
    EXPECT_NEAR(point.ax, 0.0, 1e-12);
    EXPECT_NEAR(point.ay, 0.0, 1e-12);
    ++rotation_samples;
  }
  // 1.875 * π / 2.5 = 2.356 s,按 0.10 s 采样至少 24 个,加上起点本身。
  EXPECT_GE(rotation_samples, 24U);
}

TEST(TerminalYawRelocation, TranslationGeometryAndGoalPositionAreUntouched)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));

  // 重定位只改 yaw 时序,不动任何一个引导点的坐标。
  std::vector<double> original_x;
  for (std::size_t index = 0; index <= tail; ++index) {
    original_x.push_back(trajectory.points[index].x);
  }
  std::vector<double> candidate_x;
  double previous = std::numeric_limits<double>::quiet_NaN();
  for (const auto & point : candidate.points) {
    if (!std::isfinite(previous) || std::abs(point.x - previous) > 1e-9) {
      candidate_x.push_back(point.x);
      previous = point.x;
    }
  }
  ASSERT_EQ(candidate_x.size(), original_x.size());
  for (std::size_t index = 0; index < original_x.size(); ++index) {
    EXPECT_NEAR(candidate_x[index], original_x[index], 1e-9);
  }
}

TEST(TerminalYawRelocation, TerminalPointIsAtRest)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));
  // goal manager 的成功判据同时要求位置、yaw 和速度收敛。
  EXPECT_NEAR(candidate.points.back().v, 0.0, 1e-12);
  EXPECT_NEAR(candidate.points.back().vx, 0.0, 1e-12);
  EXPECT_NEAR(candidate.points.back().vy, 0.0, 1e-12);
  EXPECT_NEAR(candidate.points.back().yaw_rate, 0.0, 1e-12);
}

TEST(TerminalYawRelocation, RelocationAtTheTailReproducesTheStatusQuoShape)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, tail, params, &candidate));
  ASSERT_TRUE(candidate.valid());
  // 候选族的最晚一个与现状等价:点数相同,末端 yaw 相同,坐标逐点相同。
  EXPECT_EQ(candidate.points.size(), trajectory.points.size());
  EXPECT_NEAR(candidate.points.back().yaw, 0.0, 1e-9);
  for (std::size_t index = 0; index < candidate.points.size(); ++index) {
    EXPECT_NEAR(candidate.points[index].x, trajectory.points[index].x, 1e-9);
    EXPECT_NEAR(candidate.points[index].y, trajectory.points[index].y, 1e-9);
  }
}

TEST(TerminalYawRelocation, RotationIndexBeyondTheTailIsRejected)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  ReferenceTrajectory candidate;
  EXPECT_FALSE(relocateTerminalYawRotation(trajectory, 0.0, tail + 1U, params, &candidate));
  EXPECT_FALSE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, nullptr));
}

TEST(TerminalYawRelocation, AlreadyAlignedYawInsertsNoRotationSamples)
{
  const auto params = makeParams();
  ReferenceTrajectory trajectory = makeCorridorApproach(1.60, 0.95, 1.05, 0.0);
  appendTerminalRotation(trajectory, 0.0, params);
  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));
  // yaw 已经对齐时不应该凭空插入原地转向段。
  EXPECT_EQ(candidate.points.size(), 7U);
  EXPECT_NEAR(candidate.points.back().yaw, 0.0, 1e-9);
}

TEST(TerminalYawRelocation, CandidatesSpanTheEarliestAndLatestRotationIndex)
{
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const std::size_t tail = terminalCoincidentTailStart(trajectory);
  const auto candidates = terminalYawRelocationCandidates(trajectory, 6U);
  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front(), 0U);
  EXPECT_EQ(candidates.back(), tail);
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    EXPECT_GT(candidates[index], candidates[index - 1]);
  }
}

TEST(TerminalYawRelocation, CandidateCountIsBounded)
{
  const auto params = makeParams();
  ReferenceTrajectory trajectory = makeCorridorApproach(3.00, 0.95, 1.05, M_PI);
  appendTerminalRotation(trajectory, 0.0, params);
  EXPECT_LE(terminalYawRelocationCandidates(trajectory, 3U).size(), 3U);
  EXPECT_EQ(terminalYawRelocationCandidates(trajectory, 1U).size(), 1U);
  EXPECT_EQ(terminalYawRelocationCandidates(trajectory, 1U).front(), 0U);
  EXPECT_TRUE(terminalYawRelocationCandidates(trajectory, 0U).empty());
}

TEST(TerminalYawRelocation, RelocationClearsTheTerminalRotationCollision)
{
  // 端到端:用真实矩形足迹门禁确认这条候选真的把 domain 173 目标 8 的形态解开,
  // 而不是只"换了个 yaw 时序"。目标处原地转向必然扫进墙,起点处转向则安全。
  auto grid = makeGrid();
  fillColumn(grid, 4, 100);
  fillColumn(grid, 5, 100);
  const auto gate = makeGate();
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);

  const auto before = gate.check(trajectory, grid);
  ASSERT_FALSE(before.safe);
  ASSERT_TRUE(collisionsConfinedToTerminalRotation(trajectory, before));

  ReferenceTrajectory candidate;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, 0U, params, &candidate));
  const auto after = gate.check(candidate, grid);
  EXPECT_TRUE(after.safe);
  EXPECT_TRUE(after.collisions.empty());
  // 目标位姿不变:位置逐位相同,末端 yaw 就是 goal_yaw。
  EXPECT_NEAR(candidate.points.back().x, trajectory.points.back().x, 1e-9);
  EXPECT_NEAR(candidate.points.back().y, trajectory.points.back().y, 1e-9);
  EXPECT_NEAR(candidate.points.back().yaw, 0.0, 1e-9);
}

TEST(TerminalYawRelocation, StatusQuoCandidateStillFailsTheSameGate)
{
  // 同一门禁下,候选族最晚的那个(等价于现状)必须仍然不安全。否则这个用例
  // 证明不了门禁在两个候选之间做了真实区分。
  auto grid = makeGrid();
  fillColumn(grid, 4, 100);
  fillColumn(grid, 5, 100);
  const auto gate = makeGate();
  const auto params = makeParams();
  const auto trajectory = makeGoalEightAnalogue(params);
  const std::size_t tail = terminalCoincidentTailStart(trajectory);

  ReferenceTrajectory status_quo;
  ASSERT_TRUE(relocateTerminalYawRotation(trajectory, 0.0, tail, params, &status_quo));
  EXPECT_FALSE(gate.check(status_quo, grid).safe);
}

}  // namespace
