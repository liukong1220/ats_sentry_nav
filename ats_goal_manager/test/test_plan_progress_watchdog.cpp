// Copyright 2026

#include <chrono>
#include <limits>

#include "ats_goal_manager/plan_progress_watchdog.hpp"
#include "gtest/gtest.h"

namespace
{

using Watchdog = ats_goal_manager::PlanProgressWatchdog;
using Decision = ats_goal_manager::PlanProgressDecision;
using Clock = Watchdog::Clock;

ats_goal_manager::PlanProgressGate healthyGate(double distance)
{
  ats_goal_manager::PlanProgressGate gate;
  gate.goal_active = true;
  gate.emergency_stop = false;
  gate.map_fresh = true;
  gate.localization_fresh = true;
  gate.tf_healthy = true;
  gate.robot_inside_map = true;
  gate.robot_cell_free = true;
  gate.footprint_safe = true;
  gate.has_current_reference = true;
  gate.distance_to_goal_m = distance;
  gate.identity.goal_id = 7;
  gate.identity.localization_epoch = 3;
  gate.identity.map_publication_sequence = 11;
  gate.identity.map_generation = 19;
  gate.identity.plan_request_sequence = 2;
  return gate;
}

ats_goal_manager::SuspendedPlanGate healthySuspendedGate()
{
  ats_goal_manager::SuspendedPlanGate gate;
  gate.goal_active = true;
  gate.map_source_alive = true;
  gate.localization_fresh = true;
  gate.identity.goal_id = 7;
  gate.identity.localization_epoch = 3;
  return gate;
}

TEST(PlanProgressWatchdog, WaitsForStallAndHonorsProgressReset)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.progress_min_delta_m = 0.20;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 0.5;
  params.max_consecutive_replans = 2;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(5.0);
  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  gate.distance_to_goal_m = 4.7;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(500)), Decision::kNone);
  gate.distance_to_goal_m = 4.4;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(900)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(1800)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(1901)), Decision::kReplan);
  EXPECT_EQ(watchdog.consecutiveReplans(), 1U);
}

TEST(PlanProgressWatchdog, ReplansOnceThenUsesMinimumInterval)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 2.0;
  params.max_consecutive_replans = 2;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  const auto gate = healthyGate(5.0);

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(1)), Decision::kReplan);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(2100)), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(3001)), Decision::kReplan);
}

TEST(PlanProgressWatchdog, RejectsUnsafeGatesWithoutReplanning)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 0.1;
  // 本例只覆盖"有界等待窗口之内"的行为，
  // 因此把 ego 放行上限设得远大于时间轴；
  // 窗口耗尽之后的语义由 EgoBlockedEscapes* 用例单独锁定。
  params.ego_blocked_escape_timeout_sec = 30.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto gate = healthyGate(5.0);
  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);

  for (auto member : {0, 1, 2, 3, 4, 5}) {
    auto unsafe = gate;
    switch (member) {
      case 0: unsafe.map_fresh = false; break;
      case 1: unsafe.localization_fresh = false; break;
      case 2: unsafe.tf_healthy = false; break;
      case 3: unsafe.robot_inside_map = false; break;
      case 4: unsafe.robot_cell_free = false; break;
      default: unsafe.footprint_safe = false; break;
    }
    EXPECT_EQ(
      watchdog.evaluate(unsafe, t0 + std::chrono::seconds(2)), Decision::kUnsafe);
    EXPECT_EQ(watchdog.consecutiveReplans(), 0U);
  }
}

TEST(PlanProgressWatchdog, ExhaustsBoundedReplans)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 0.0;
  params.max_consecutive_replans = 1;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  const auto gate = healthyGate(5.0);

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(1)), Decision::kReplan);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(2)), Decision::kExhausted);
  EXPECT_EQ(watchdog.consecutiveReplans(), 1U);
}

ats_goal_manager::PlanProgressWatchdogParams escapeParams()
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.progress_min_delta_m = 0.10;
  params.replan_stall_timeout_sec = 1.0;
  params.replan_min_interval_sec = 0.0;
  params.max_consecutive_replans = 1;
  params.ego_blocked_escape_enabled = true;
  params.ego_blocked_escape_timeout_sec = 2.0;
  return params;
}

TEST(PlanProgressWatchdog, EgoBlockedEscapeIsDisabledByDefault)
{
  Watchdog watchdog;
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto blocked = healthyGate(5.0);
  blocked.footprint_safe = false;

  EXPECT_EQ(watchdog.evaluate(blocked, t0), Decision::kUnsafe);
  EXPECT_EQ(watchdog.evaluate(blocked, t0 + std::chrono::hours(1)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_EQ(watchdog.consecutiveReplans(), 0U);
}

TEST(PlanProgressWatchdog, InfrastructureUnsafeStaysUnboundedAndNeverEscapes)
{
  // 地图/定位/TF/出图这四个条件下等待本身可以恢复，
  // 而且这些条件不成立时连规划
  // 都不成立，所以必须保持无界 fail-closed，永远不进入 ego 放行。
  Watchdog watchdog(escapeParams());
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  for (auto member : {0, 1, 2, 3}) {
    auto unsafe = healthyGate(5.0);
    switch (member) {
      case 0: unsafe.map_fresh = false; break;
      case 1: unsafe.localization_fresh = false; break;
      case 2: unsafe.tf_healthy = false; break;
      default: unsafe.robot_inside_map = false; break;
    }
    for (int step = 0; step < 20; ++step) {
      EXPECT_EQ(
        watchdog.evaluate(unsafe, t0 + std::chrono::seconds(step)), Decision::kUnsafe);
      EXPECT_FALSE(watchdog.egoEscapeActive());
      EXPECT_EQ(watchdog.consecutiveReplans(), 0U);
    }
  }
}

TEST(PlanProgressWatchdog, EgoBlockedEscapesAfterBoundedWaitAndStaysBounded)
{
  // 机器人已经占据的位姿被判为占据时，
  // 唯一能挪走它的执行器正是这里要停掉的那
  // 一个。无界拒绝只会 livelock，所以有界等待耗尽后必须放行，
  // 并交回既有的
  // stall/replan 预算兜底：挪不出去就 kExhausted 干净失败。
  Watchdog watchdog(escapeParams());
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto blocked = healthyGate(5.0);
  blocked.footprint_safe = false;

  EXPECT_EQ(watchdog.evaluate(blocked, t0), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(1900)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());

  EXPECT_EQ(watchdog.evaluate(blocked, t0 + std::chrono::seconds(2)), Decision::kNone);
  EXPECT_TRUE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(3001)), Decision::kReplan);
  EXPECT_EQ(watchdog.consecutiveReplans(), 1U);
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(4002)), Decision::kExhausted);
}

TEST(PlanProgressWatchdog, EgoBlockedEscapeAlsoCoversOccupiedRobotCell)
{
  Watchdog watchdog(escapeParams());
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto blocked = healthyGate(5.0);
  blocked.robot_cell_free = false;

  EXPECT_EQ(watchdog.evaluate(blocked, t0), Decision::kUnsafe);
  EXPECT_EQ(watchdog.evaluate(blocked, t0 + std::chrono::seconds(2)), Decision::kNone);
  EXPECT_TRUE(watchdog.egoEscapeActive());
}

TEST(PlanProgressWatchdog, EgoBlockedEscapeClearsWhenPoseBecomesSafeAgain)
{
  Watchdog watchdog(escapeParams());
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto blocked = healthyGate(5.0);
  blocked.footprint_safe = false;

  EXPECT_EQ(watchdog.evaluate(blocked, t0), Decision::kUnsafe);
  EXPECT_EQ(watchdog.evaluate(blocked, t0 + std::chrono::seconds(2)), Decision::kNone);
  EXPECT_TRUE(watchdog.egoEscapeActive());

  // 位姿重新合法后放行状态必须收回，
  // 下一次占据要重新走完整的有界等待。
  EXPECT_EQ(
    watchdog.evaluate(healthyGate(5.0), t0 + std::chrono::milliseconds(2500)),
    Decision::kNone);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(2600)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(4500)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_TRUE(watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(4700)) !=
    Decision::kUnsafe);
  EXPECT_TRUE(watchdog.egoEscapeActive());
}

TEST(PlanProgressWatchdog, NeverCommittedReferenceFailsAfterABoundedWait)
{
  // domain 189 目标 9 的形态:基础设施健康、自身位姿安全,但规划器连续 91 次
  // FAILURE_FOOTPRINT,从未交付第一条参考。emergency_stop 在没有参考时必然为真,
  // 旧实现把它和 has_current_reference 放在同一个 kNone 分支上,于是 180 s 内
  // watchdog 零输出,只能由外部 runner 超时结束目标。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.has_current_reference = false;
  // 没有可执行参考时 goal manager 必然同时压着急停,
  // 判据不能依赖急停为假。
  gate.emergency_stop = true;

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(4999)), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(5000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, FirstReferenceClearsTheNoExecutablePlanBudget)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  params.replan_stall_timeout_sec = 100.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.has_current_reference = false;
  gate.emergency_stop = true;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(4000)), Decision::kNone);

  // 参考到达后预算必须清零,而不是继续累计到已经在推进的目标上。
  gate.has_current_reference = true;
  gate.emergency_stop = false;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(4500)), Decision::kNone);

  gate.has_current_reference = false;
  gate.emergency_stop = true;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(8000)), Decision::kNone);
  // 预算从"参考消失"那一刻重新起算,而不是沿用目标开始时的时间戳。
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(12999)), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(13000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, InfrastructureWaitIsNotChargedToTheNoExecutablePlanBudget)
{
  // 地图/定位预热期间没有参考是正常的,
  // 这段无界等待不能被记成"规划器交付不出轨迹",
  // 否则第一个目标会在建图完成之前就被判失败。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.has_current_reference = false;
  gate.emergency_stop = true;
  gate.map_fresh = false;
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(i)), Decision::kUnsafe);
  }

  gate.map_fresh = true;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(20)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(24)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(25)), Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, EgoBlockedPoseStillReachesTheNoExecutablePlanBudget)
{
  // 自身位姿被占据时的有界等待用尽后必须落到无参考预算上,
  // 而不是变成另一条无界路径。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  params.ego_blocked_escape_enabled = true;
  params.ego_blocked_escape_timeout_sec = 1.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.has_current_reference = false;
  gate.emergency_stop = true;
  gate.footprint_safe = false;

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kUnsafe);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(1001)), Decision::kNone);
  EXPECT_TRUE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(6002)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, HeldReferencePinnedByEmergencyStopFailsAfterBoundedWait)
{
  // domain 193 目标 8 的形态,和"从未拿到参考"是不同的入口:参考一直在
  // (观测到的 reference 恒为 1),但 MINCO 每 2 s 拒同一条轨迹,goal manager
  // 始终压着急停。旧实现只给 has_current_reference=false 记预算,急停这一条
  // 仍是无界 kNone,于是 152 s 内 watchdog 零输出,只能靠外部超时结束。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  params.replan_stall_timeout_sec = 100.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.has_current_reference = true;
  gate.emergency_stop = true;

  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(4999)), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(5000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, LegitimateEmergencyStopWindowsDoNotExhaustTheBudget)
{
  // 回归下界:domain 193 的 12 个目标内急停窗口最长 11.56 s,都属于正常
  // replan 节奏。30 s 预算必须让它们全部通过,并且在急停解除时清零,
  // 否则连续几段目标会被累计成误判。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 30.0;
  params.replan_stall_timeout_sec = 100.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(4.0);
  gate.emergency_stop = true;
  for (int ms = 0; ms <= 12000; ms += 2000) {
    EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(ms)), Decision::kNone);
  }

  // 急停解除且参考在手:这一拍可执行,预算清零。
  gate.emergency_stop = false;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::milliseconds(12500)), Decision::kNone);

  // 第二个同样长度的窗口从 20 s 重新起算,而不是接着第一个窗口累计。
  gate.emergency_stop = true;
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(20)), Decision::kNone);
  EXPECT_EQ(watchdog.evaluate(gate, t0 + std::chrono::seconds(31)), Decision::kNone);
  // 但它依然是有界的:从 20 s 起算满 30 s 就失败。
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::seconds(50)), Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, NonFiniteDistanceIsAlsoBounded)
{
  // 距离非有限时同样无法执行。这条以前也是一条无界 kNone 早退。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthyGate(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(watchdog.evaluate(gate, t0), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(gate, t0 + std::chrono::milliseconds(5000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, InfrastructureFaultRevokesEgoEscape)
{
  // 放行只在基础设施健康时成立；
  // 地图或定位掉了必须立刻退回无界 fail-closed。
  Watchdog watchdog(escapeParams());
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto blocked = healthyGate(5.0);
  blocked.footprint_safe = false;

  EXPECT_EQ(watchdog.evaluate(blocked, t0), Decision::kUnsafe);
  EXPECT_EQ(watchdog.evaluate(blocked, t0 + std::chrono::seconds(2)), Decision::kNone);
  EXPECT_TRUE(watchdog.egoEscapeActive());

  auto degraded = blocked;
  degraded.map_fresh = false;
  EXPECT_EQ(
    watchdog.evaluate(degraded, t0 + std::chrono::milliseconds(2100)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
  EXPECT_EQ(
    watchdog.evaluate(blocked, t0 + std::chrono::milliseconds(2200)), Decision::kUnsafe);
  EXPECT_FALSE(watchdog.egoEscapeActive());
}

TEST(PlanProgressWatchdog, SuspendedGoalWithHealthyInfrastructureIsBounded)
{
  // domain 168 目标 4(START_ENCLOSED)与 domain 169 目标 9(FOOTPRINT)的形态:
  // 瞬时故障把目标挂回 kWaitingForMap,dispatch->fail->suspend 每 2 s 一轮,
  // 每轮成功派发都会清掉 waiting_since,所以 map_wait_timeout_sec 的 5 s
  // 预算永远不到期;evaluate() 又只在 kTracking 调用。结果是 179.6/176.0 s
  // 全程零输出,只能由外部目标超时结束。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  const auto gate = healthySuspendedGate();
  EXPECT_EQ(watchdog.evaluateSuspended(gate, t0), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(4999)),
    Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(5000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, SuspendedInfrastructureWaitStaysUnbounded)
{
  // 启动预热与 adapter lease/service timeout/input stale 三个故障用例都落在
  // 这里:地图或定位没就绪时等待是刻意无界的,
  // 不能记成"规划器交付不出轨迹",
  // 否则会把 P2 故障注入的 fail-closed 停机改判成规划失败。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto gate = healthySuspendedGate();
  gate.map_source_alive = false;
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(60000)),
    Decision::kUnsafe);

  gate.map_source_alive = true;
  gate.localization_fresh = false;
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(120000)),
    Decision::kUnsafe);

  gate.localization_fresh = false;
  gate.map_source_alive = false;
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(180000)),
    Decision::kUnsafe);

  gate.map_source_alive = true;
  gate.localization_fresh = true;

  // 基础设施恢复后预算从此刻重新起算,不追认之前的无界等待。
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(180001)),
    Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(185001)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, SuspendedAndTrackingShareOneNoExecutablePlanBudget)
{
  // 关键不变量:churn 本身不能清零预算。挂起与 tracking 交替出现时,
  // 只有真正发布出参考才算"有可执行计划";否则 89 轮 2 s 的循环会
  // 因为每轮换一次状态而永远不到期。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  const auto suspended = healthySuspendedGate();
  auto tracking = healthyGate(4.0);
  tracking.has_current_reference = false;
  tracking.emergency_stop = true;

  EXPECT_EQ(watchdog.evaluateSuspended(suspended, t0), Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluate(tracking, t0 + std::chrono::milliseconds(2000)),
    Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(suspended, t0 + std::chrono::milliseconds(4000)),
    Decision::kNone);
  // 跨两次状态切换后预算仍按 t0 起算,在 5 s 处到期。
  EXPECT_EQ(
    watchdog.evaluate(tracking, t0 + std::chrono::milliseconds(5000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, PublishedReferenceClearsTheSuspendedBudget)
{
  // 成功的目标必须不受影响:domain 169 目标 1..8 的连续失败次数为 0,
  // 合法的连续挂起时长是 0 s。一旦参考发布出来预算就必须清零,
  // 否则正常推进的目标会被这条新预算误杀。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  const auto suspended = healthySuspendedGate();
  EXPECT_EQ(
    watchdog.evaluateSuspended(suspended, t0 + std::chrono::milliseconds(4000)),
    Decision::kNone);

  ats_goal_manager::PlanProgressIdentity identity;
  identity.goal_id = 7;
  identity.localization_epoch = 3;
  watchdog.observeReference(identity, 4.0, t0 + std::chrono::milliseconds(4500));

  EXPECT_EQ(
    watchdog.evaluateSuspended(suspended, t0 + std::chrono::milliseconds(9000)),
    Decision::kNone);
  // 预算从"参考消失后第一次观察"起算,即 9000,而不是沿用清零前的 4000。
  EXPECT_EQ(
    watchdog.evaluateSuspended(suspended, t0 + std::chrono::milliseconds(13999)),
    Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(suspended, t0 + std::chrono::milliseconds(14000)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, RecoverySnapshotChurnDoesNotRefundTheBudget)
{
  // domain 171 goal 8 的真实形态:adapter 心跳全程 ready=1(180/181),
  // 但每次瞬时规划失败都会就地清掉 goal manager 本地的 map ready 标志,
  // 并把 recovery snapshot 置为未就绪,约 2 s 后下一次心跳又恢复。
  // 早前的实现把这两个翻转信号当健康条件,于是预算每个心跳周期归零,
  // 179.6 s 里一次也没到期。这里锁定:只要地图源存活且定位新鲜,
  // 挂起/重派 churn 本身不再退还预算。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  const auto gate = healthySuspendedGate();

  // 每 500 ms 一个 tick,模拟 2 s 心跳周期内的反复挂起与重派。
  for (int tick = 1; tick <= 9; ++tick) {
    EXPECT_EQ(
      watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(500 * tick)),
      Decision::kNone) << "tick=" << tick;
  }
  // 预算锚定在首个 tick(t=500),不是 t0,所以到期在 5500。
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(5499)),
    Decision::kNone);
  EXPECT_EQ(
    watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(5500)),
    Decision::kNoExecutablePlan);
}

TEST(PlanProgressWatchdog, DeadMapSourceKeepsTheSuspendedWaitUnbounded)
{
  // 与上一条对偶:adapter_lease 让心跳租约过期、service_timeout 与 unknown
  // 让心跳自报 ready=0,两者都使 map_source_alive 为假。此时等待必须保持
  // 无界,否则注入故障后的 fail-closed 停机会被改判成规划失败。
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};
  auto gate = healthySuspendedGate();
  gate.map_source_alive = false;

  for (int tick = 1; tick <= 200; ++tick) {
    EXPECT_EQ(
      watchdog.evaluateSuspended(gate, t0 + std::chrono::milliseconds(500 * tick)),
      Decision::kUnsafe) << "tick=" << tick;
  }
}

TEST(PlanProgressWatchdog, SuspendedGateIgnoresForeignAndCanceledGoals)
{
  ats_goal_manager::PlanProgressWatchdogParams params;
  params.no_executable_plan_timeout_sec = 5.0;
  Watchdog watchdog(params);
  watchdog.resetGoal(7);
  const auto t0 = Clock::time_point{};

  auto foreign = healthySuspendedGate();
  foreign.identity.goal_id = 8;
  EXPECT_EQ(
    watchdog.evaluateSuspended(foreign, t0 + std::chrono::milliseconds(60000)),
    Decision::kNone);

  auto canceled = healthySuspendedGate();
  canceled.cancel_or_preempt = true;
  EXPECT_EQ(
    watchdog.evaluateSuspended(canceled, t0 + std::chrono::milliseconds(60000)),
    Decision::kNone);

  auto inactive = healthySuspendedGate();
  inactive.goal_active = false;
  EXPECT_EQ(
    watchdog.evaluateSuspended(inactive, t0 + std::chrono::milliseconds(60000)),
    Decision::kNone);
}

}  // namespace
