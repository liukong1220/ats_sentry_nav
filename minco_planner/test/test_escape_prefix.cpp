// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "minco_planner/safety/escape_prefix.hpp"

namespace
{

// 每点间距 0.05 m 的直线轨迹，接近 RMUC 实测参考轨迹的采样密度（1.63 m / 38 点）。
std::vector<minco_planner::ReferencePoint> makeLine(std::size_t count, double spacing = 0.05)
{
  std::vector<minco_planner::ReferencePoint> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * static_cast<double>(i);
    point.s = spacing * static_cast<double>(i);
    point.x = spacing * static_cast<double>(i);
    point.y = 0.0;
    points.push_back(point);
  }
  return points;
}

// 原地转向轨迹：平移可忽略、yaw 逐点等量变化。复现 domain 195 goal 9 的实测形态
// （22 点、前 18 点仅 0.029 m 弧长、yaw 从 -0.744 rad 转回 0）。
std::vector<minco_planner::ReferencePoint> makeInPlaceRotation(
  std::size_t count, double yaw_start, double yaw_step, double spacing = 0.0016)
{
  std::vector<minco_planner::ReferencePoint> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * static_cast<double>(i);
    point.s = spacing * static_cast<double>(i);
    point.x = spacing * static_cast<double>(i);
    point.y = 0.0;
    point.yaw = yaw_start + yaw_step * static_cast<double>(i);
    points.push_back(point);
  }
  return points;
}

std::vector<minco_planner::CollisionSample> upTo(std::size_t last_index)
{
  std::vector<minco_planner::CollisionSample> collisions;
  for (std::size_t i = 0; i <= last_index; ++i) {
    minco_planner::CollisionSample sample;
    sample.trajectory_index = i;
    collisions.push_back(sample);
  }
  return collisions;
}

minco_planner::CollisionSample at(std::size_t index, bool swept = false)
{
  minco_planner::CollisionSample sample;
  sample.trajectory_index = index;
  sample.swept = swept;
  return sample;
}

minco_planner::EscapePrefixParams enabledParams()
{
  minco_planner::EscapePrefixParams params;
  params.enabled = true;
  return params;
}

}  // namespace

// 这是本修复要解决的实测死锁：domain 147 goal 2 在 176 s 内产生 89 次拒绝，全部落在
// index 0（47 次）或 index 1（42 次），位姿从 (3.696,-5.578) 只移动到 (3.702,-5.566)。
// 车已经在接触中，从当前位姿出发的每条轨迹都被 index 0/1 判负，因而永远无法被执行。
TEST(EscapePrefix, ContactAtTrajectoryStartIsAllowedToEscape)
{
  const auto points = makeLine(40);
  const auto params = enabledParams();

  const auto from_index_zero =
    minco_planner::evaluateEscapePrefix(points, {at(0), at(1)}, params);
  EXPECT_TRUE(from_index_zero.allowed);
  EXPECT_EQ(from_index_zero.prefix_end, 3U);
  EXPECT_NEAR(from_index_zero.head_offset_m, 0.0, 1e-9);
  EXPECT_NEAR(from_index_zero.prefix_length_m, 0.15, 1e-9);

  // domain 147 的另一半样本：当前格判定为空闲，但紧邻的下一点冲突。两种样本在同一位姿
  // 上交替出现，说明车压在栅格化的占据/空闲边界上，同属"从接触中逃逸"。
  const auto from_index_one = minco_planner::evaluateEscapePrefix(points, {at(1)}, params);
  EXPECT_TRUE(from_index_one.allowed);
  EXPECT_EQ(from_index_one.prefix_end, 3U);
  EXPECT_NEAR(from_index_one.head_offset_m, 0.05, 1e-9);
}

// 只有"贴着起点"的冲突才是逃逸。先自由行驶一段再撞上障碍的轨迹必须照旧拒绝，否则这个
// 放行就退化成允许规划器主动开进障碍。
TEST(EscapePrefix, CollisionAheadOfTheRobotIsStillRejected)
{
  const auto points = makeLine(40);
  const auto params = enabledParams();

  const auto decision = minco_planner::evaluateEscapePrefix(points, {at(6), at(7)}, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
  EXPECT_NEAR(decision.head_offset_m, 0.30, 1e-9);
}

// 前缀之后必须真的驶出障碍集并且不再驶回。冲突延伸到轨迹末端意味着整条轨迹都在障碍里，
// 这不是逃逸。
TEST(EscapePrefix, TrajectoryWithoutACollisionFreeRemainderIsRejected)
{
  const auto points = makeLine(6);
  const auto params = enabledParams();

  const auto decision =
    minco_planner::evaluateEscapePrefix(points, {at(0), at(1), at(2), at(3), at(4), at(5)}, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
}

// 驶出后又驶回：最后一个冲突把 prefix_end 推到剩余段之外，或推出弧长上界，都必须拒绝。
TEST(EscapePrefix, LeavingAndReenteringTheObstacleIsRejected)
{
  const auto points = makeLine(40);
  const auto params = enabledParams();

  // 冲突在 index 0 和 index 20：head_offset 合格，但 prefix_end=22 超出点数与弧长上界。
  const auto decision = minco_planner::evaluateEscapePrefix(points, {at(0), at(20)}, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
}

// 有界性：前缀的平移、旋转、点数各自独立封顶，任一超限即拒绝，避免把"穿过障碍"伪装成逃逸。
TEST(EscapePrefix, PrefixMotionAndPointBudgetsAreEachEnforced)
{
  auto params = enabledParams();

  // 平移超限而点数、yaw 合格：间距 0.10 m，冲突到 index 5 -> prefix_end=7，
  // 但前缀弧长 0.70 m > 0.40 m。
  const auto long_prefix = minco_planner::evaluateEscapePrefix(
    makeLine(40, 0.10), {at(0), at(5)}, params);
  EXPECT_FALSE(long_prefix.allowed);

  // 点数超限而平移、yaw 合格：间距 0.005 m，冲突到 index 70 -> prefix_end=72 > 64，
  // 前缀弧长仅 0.36 m。
  const auto dense = makeLine(100, 0.005);
  const auto many_points = minco_planner::evaluateEscapePrefix(dense, {at(0), at(70)}, params);
  EXPECT_FALSE(many_points.allowed);

  // 放宽点数上界后同一条轨迹通过，证明拒绝原因确实是点数预算而不是别的判据。
  params.max_prefix_points = 80U;
  const auto relaxed = minco_planner::evaluateEscapePrefix(dense, {at(0), at(70)}, params);
  EXPECT_TRUE(relaxed.allowed);
  EXPECT_EQ(relaxed.prefix_end, 72U);
  EXPECT_NEAR(relaxed.prefix_length_m, 0.36, 1e-9);
}

// domain 195 goal 9 实测：车停在 (9.256,-2.222,yaw=-0.744)，栅格过报使前缘落在占据格内，
// 每条候选轨迹都从 index 0 起冲突。唯一可行的脱离是原地转向到 e(yaw) 更小的相位——这条
// 轨迹平移只有 0.029 m，却要 18 个采样点，被旧的 12 点上界判负，于是 gate 无界拒绝、
// goal manager 的 no-executable-plan 预算耗尽，final_distance 0.029 m 仍然 ABORTED。
TEST(EscapePrefix, InPlaceRotationOutOfAQuantizedContactIsAllowedToEscape)
{
  const auto points = makeInPlaceRotation(22, -0.744, 0.744 / 18.0);
  const auto collisions = upTo(16U);
  const auto params = enabledParams();

  const auto decision = minco_planner::evaluateEscapePrefix(points, collisions, params);
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 18U);
  EXPECT_NEAR(decision.head_offset_m, 0.0, 1e-9);
  EXPECT_NEAR(decision.prefix_length_m, 18.0 * 0.0016, 1e-9);
  EXPECT_NEAR(decision.prefix_yaw_sweep_rad, 0.744, 1e-9);

  // 把点数上界调回旧的 12 会重现 livelock：这证明该形态下点数预算是唯一判负项，
  // 而平移与旋转两条运动上界都还有大量余量。
  minco_planner::EscapePrefixParams legacy = params;
  legacy.max_prefix_points = 12U;
  EXPECT_FALSE(minco_planner::evaluateEscapePrefix(points, collisions, legacy).allowed);
}

// 弧长只约束平移，原地旋转的弧长近似为 0，所以"在障碍里自转半圈以上"必须由 yaw 扫掠上界
// 拒绝，否则放宽点数预算就等于放行这一类轨迹。
TEST(EscapePrefix, RotatingThroughAnObstacleBeyondTheYawBudgetIsRejected)
{
  const auto points = makeInPlaceRotation(24, 0.0, 0.145);
  const auto collisions = upTo(19U);
  auto params = enabledParams();

  const auto decision = minco_planner::evaluateEscapePrefix(points, collisions, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
  EXPECT_NEAR(decision.prefix_yaw_sweep_rad, 21.0 * 0.145, 1e-9);

  // 放宽 yaw 上界后同一条轨迹通过，证明判负项确实是扫掠角而不是平移或点数。
  params.max_prefix_yaw_sweep_rad = M_PI;
  const auto relaxed = minco_planner::evaluateEscapePrefix(points, collisions, params);
  EXPECT_TRUE(relaxed.allowed);
  EXPECT_EQ(relaxed.prefix_end, 21U);
  EXPECT_LT(relaxed.prefix_length_m, 0.04);
}

// 往复摆动的净 yaw 变化为 0，但它同样是"压在障碍里反复转向"，扫掠角必须累加而不是相消。
TEST(EscapePrefix, OscillatingYawInsideTheObstacleAccumulatesTowardTheBudget)
{
  auto points = makeInPlaceRotation(24, 0.0, 0.0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].yaw = (i % 2U == 0U) ? 0.0 : 0.4;
  }
  const auto params = enabledParams();

  const auto decision = minco_planner::evaluateEscapePrefix(points, upTo(19U), params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NEAR(decision.prefix_yaw_sweep_rad, 21.0 * 0.4, 1e-9);
  // 前缀首末 yaw 之差最多 0.4 rad，只看净变化会把这条累计 8.4 rad 的轨迹判为"几乎没转"。
  EXPECT_LE(std::fabs(points[21].yaw - points.front().yaw), 0.4 + 1e-9);
  EXPECT_GT(decision.prefix_yaw_sweep_rad, 20.0 * std::fabs(points[21].yaw - points.front().yaw));
}

// yaw 跨 +-pi 的两点差必须归一化，否则一次 0.02 rad 的过零转向会被算成 6.26 rad。
TEST(EscapePrefix, YawSweepIsWrapNormalized)
{
  auto points = makeInPlaceRotation(22, 0.0, 0.0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].yaw = (i % 2U == 0U) ? (M_PI - 0.01) : (-M_PI + 0.01);
  }
  const auto params = enabledParams();
  const auto decision = minco_planner::evaluateEscapePrefix(points, upTo(16U), params);
  EXPECT_TRUE(decision.allowed);
  EXPECT_NEAR(decision.prefix_yaw_sweep_rad, 18.0 * 0.02, 1e-9);
}

// swept 冲突记录在线段起点下标上，所以 prefix_end 必须跨过 last_index+1，否则放行的
// 轨迹里仍可能藏着一段扫掠冲突。
TEST(EscapePrefix, SweptCollisionAtTheSegmentStartStillClearsTheFollowingPoint)
{
  const auto points = makeLine(40);
  const auto params = enabledParams();

  const auto decision =
    minco_planner::evaluateEscapePrefix(points, {at(0), at(1, true)}, params);
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 3U);
}

// 关闭开关必须完全恢复旧行为；无冲突时调用方走正常路径，这里不声明放行。
TEST(EscapePrefix, DisabledOrCollisionFreeInputsNeverClaimAnEscape)
{
  const auto points = makeLine(40);
  minco_planner::EscapePrefixParams disabled;
  disabled.enabled = false;
  EXPECT_FALSE(minco_planner::evaluateEscapePrefix(points, {at(0)}, disabled).allowed);

  const auto params = enabledParams();
  EXPECT_FALSE(minco_planner::evaluateEscapePrefix(points, {}, params).allowed);
  EXPECT_EQ(minco_planner::evaluateEscapePrefix(points, {}, params).collision_count, 0U);

  // 少于两点的轨迹本身就过不了 ReferenceTrajectory::valid()，不能借逃逸放行。
  EXPECT_FALSE(minco_planner::evaluateEscapePrefix(makeLine(1), {at(0)}, params).allowed);
}

// 冲突集与轨迹不同源（下标越界）属于调用方错误，必须拒绝而不是越界读取。
TEST(EscapePrefix, OutOfRangeCollisionIndexIsRejected)
{
  const auto points = makeLine(10);
  const auto params = enabledParams();

  const auto decision = minco_planner::evaluateEscapePrefix(points, {at(0), at(10)}, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
}

// 非有限坐标不能被当作零长度前缀放行。
TEST(EscapePrefix, NonFiniteGeometryIsRejected)
{
  const auto params = enabledParams();

  auto nan_position = makeLine(40);
  nan_position[2].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(minco_planner::evaluateEscapePrefix(nan_position, {at(0)}, params).allowed);

  // yaw 现在参与判据，非有限 yaw 同样不能被当作零扫掠前缀放行。
  auto nan_yaw = makeLine(40);
  nan_yaw[2].yaw = std::numeric_limits<double>::quiet_NaN();
  const auto decision = minco_planner::evaluateEscapePrefix(nan_yaw, {at(0)}, params);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.prefix_end, 0U);
}
