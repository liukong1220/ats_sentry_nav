// Copyright 2026 ATS 2026 Sentry Project
//
// 速度源仲裁判据测试。覆盖：
//  - 无链路心跳必须归零
//  - 新鲜手动优先
//  - 手动超时不得复活旧命令，可回收到合法自动源
//  - 自动源需要 ExecutionCommand，且授权必须发生在链路 UP 之后
//  - 急停两源归零
//  - 链路失效/心跳超时两源归零，重连不恢复旧命令

#include <gtest/gtest.h>

#include <chrono>

#include "ats_cmd_vel_arbiter/cmd_vel_arbiter.hpp"

using ats_cmd_vel_arbiter::CmdVelArbiter;
using ats_cmd_vel_arbiter::CmdVelArbiterConfig;
using ats_cmd_vel_arbiter::CmdVelArbiterReason;
using ats_cmd_vel_arbiter::CmdVelSource;
using std::chrono::milliseconds;

namespace
{

CmdVelArbiterConfig config()
{
  CmdVelArbiterConfig value;
  value.manual_timeout = milliseconds(300);
  value.auto_timeout = milliseconds(300);
  value.execution_command_timeout = milliseconds(500);
  value.link_timeout = milliseconds(300);
  return value;
}

void armLink(CmdVelArbiter & arbiter, std::chrono::steady_clock::time_point now)
{
  arbiter.onLinkHealth(true, now);
  arbiter.onMapReady(true, now);
  arbiter.onPlannerStatus(7, 11, 1, true, false, false);
}

constexpr uint64_t kManagerIncarnation = 100;

bool execute(
  CmdVelArbiter & arbiter, uint64_t sequence, milliseconds age,
  std::chrono::steady_clock::time_point now)
{
  return arbiter.onExecutionCommand(true, kManagerIncarnation, sequence, 7, 11, age, now);
}

}  // namespace

TEST(CmdVelArbiter, MissingLinkHeartbeatZerosOutput)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  arbiter.onManual(0.5, 0.0, 0.0, t0);
  const auto output = arbiter.tick(t0);
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.reason, CmdVelArbiterReason::LINK_DOWN);
}

TEST(CmdVelArbiter, ManualReceivedBeforeInitialLinkUpDoesNotRevive)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  arbiter.onManual(0.5, -0.2, 0.1, t0);
  EXPECT_EQ(arbiter.tick(t0).reason, CmdVelArbiterReason::LINK_DOWN);

  armLink(arbiter, t0 + milliseconds(10));
  const auto after_initial_up = arbiter.tick(t0 + milliseconds(10));
  EXPECT_TRUE(after_initial_up.isZero());
  EXPECT_EQ(after_initial_up.source, CmdVelSource::NONE);
  EXPECT_EQ(after_initial_up.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
}

TEST(CmdVelArbiter, FreshManualPreemptsAuthorizedAuto)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.5, -0.4, 0.2, t0);
  arbiter.onManual(0.3, 0.1, -0.2, t0 + milliseconds(10));
  const auto output = arbiter.tick(t0 + milliseconds(20));
  EXPECT_DOUBLE_EQ(output.vx, 0.3);
  EXPECT_DOUBLE_EQ(output.vy, 0.1);
  EXPECT_DOUBLE_EQ(output.wz, -0.2);
  EXPECT_EQ(output.source, CmdVelSource::MANUAL);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::MANUAL_FRESH);
}

TEST(CmdVelArbiter, ManualTimeoutDoesNotResurrectOldCommand)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  arbiter.onManual(0.8, 0.0, 0.0, t0);
  armLink(arbiter, t0 + milliseconds(301));
  const auto timed_out = arbiter.tick(t0 + milliseconds(301));
  EXPECT_TRUE(timed_out.isZero());
  EXPECT_EQ(timed_out.source, CmdVelSource::NONE);
  EXPECT_EQ(timed_out.reason, CmdVelArbiterReason::MANUAL_TIMEOUT);

  const auto later = arbiter.tick(t0 + milliseconds(350));
  EXPECT_TRUE(later.isZero());
  EXPECT_EQ(later.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
}

TEST(CmdVelArbiter, ManualTimeoutRecoversToAuthorizedAuto)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.2, 0.5, -0.3, t0);
  arbiter.onManual(0.4, 0.0, 0.0, t0 + milliseconds(10));
  arbiter.onAuto(1.2, 0.5, -0.3, t0 + milliseconds(250));
  armLink(arbiter, t0 + milliseconds(311));
  const auto recovered = arbiter.tick(t0 + milliseconds(311));
  EXPECT_DOUBLE_EQ(recovered.vx, 1.2);
  EXPECT_DOUBLE_EQ(recovered.vy, 0.5);
  EXPECT_DOUBLE_EQ(recovered.wz, -0.3);
  EXPECT_EQ(recovered.source, CmdVelSource::AUTO);
  EXPECT_EQ(recovered.reason, CmdVelArbiterReason::AUTO_AUTHORIZED);
}

TEST(CmdVelArbiter, AutoWithoutAuthorizationStaysZero)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  const auto output = arbiter.tick(t0);
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.reason, CmdVelArbiterReason::AUTO_UNAUTHORIZED);
}

TEST(CmdVelArbiter, AuthorizedAutoPassesThrough)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.5, -1.5, 2.0, t0);
  const auto output = arbiter.tick(t0);
  EXPECT_DOUBLE_EQ(output.vx, 1.5);
  EXPECT_DOUBLE_EQ(output.vy, -1.5);
  EXPECT_DOUBLE_EQ(output.wz, 2.0);
  EXPECT_EQ(output.source, CmdVelSource::AUTO);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::AUTO_AUTHORIZED);
}

TEST(CmdVelArbiter, ManualDoesNotRequireExecutionCommand)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  arbiter.onManual(0.5, -0.2, 0.1, t0);
  const auto output = arbiter.tick(t0);
  EXPECT_DOUBLE_EQ(output.vx, 0.5);
  EXPECT_EQ(output.source, CmdVelSource::MANUAL);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::MANUAL_FRESH);
}

TEST(CmdVelArbiter, EmergencyStopZerosBothSources)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  arbiter.onManual(0.4, 0.0, 0.0, t0);
  arbiter.onEmergencyStop(true);
  const auto output = arbiter.tick(t0 + milliseconds(5));
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.source, CmdVelSource::NONE);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::EMERGENCY_STOP);

  arbiter.onEmergencyStop(false);
  const auto after_clear = arbiter.tick(t0 + milliseconds(10));
  EXPECT_TRUE(after_clear.isZero());
  EXPECT_NE(after_clear.reason, CmdVelArbiterReason::MANUAL_FRESH);
  EXPECT_NE(after_clear.reason, CmdVelArbiterReason::AUTO_AUTHORIZED);
}

TEST(CmdVelArbiter, LinkDownZerosBothSources)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  arbiter.onManual(0.4, 0.0, 0.0, t0);
  arbiter.onSerialLinkDown();
  const auto output = arbiter.tick(t0 + milliseconds(5));
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.reason, CmdVelArbiterReason::LINK_DOWN);

  arbiter.onSerialLinkUp(t0 + milliseconds(8));
  const auto after_up = arbiter.tick(t0 + milliseconds(10));
  EXPECT_TRUE(after_up.isZero());
  EXPECT_FALSE(arbiter.autoAuthorized());
}

TEST(CmdVelArbiter, LinkHeartbeatTimeoutZerosOutput)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  arbiter.onManual(0.4, 0.0, 0.0, t0);
  const auto timed_out = arbiter.tick(t0 + milliseconds(301));
  EXPECT_TRUE(timed_out.isZero());
  EXPECT_EQ(timed_out.reason, CmdVelArbiterReason::LINK_DOWN);
  EXPECT_FALSE(arbiter.autoAuthorized());
}

TEST(CmdVelArbiter, AuthorizationDuringLinkDownIsNotHonored)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  arbiter.onMapReady(true, t0);
  arbiter.onPlannerStatus(7, 11, 1, true, false, false);
  EXPECT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  EXPECT_FALSE(arbiter.autoAuthorized());
  armLink(arbiter, t0);
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  const auto output = arbiter.tick(t0);
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.reason, CmdVelArbiterReason::AUTO_UNAUTHORIZED);
}

TEST(CmdVelArbiter, StaleOrReplayExecutionCommandIsRejected)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  EXPECT_FALSE(execute(arbiter, 1, milliseconds(501), t0));
  EXPECT_FALSE(arbiter.autoAuthorized());
  ASSERT_TRUE(execute(arbiter, 2, milliseconds(0), t0));
  EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), t0));
  EXPECT_FALSE(arbiter.autoAuthorized());
}

TEST(CmdVelArbiter, AutoTimeoutDoesNotResurrectOldCommand)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  armLink(arbiter, t0 + milliseconds(301));
  const auto timed_out = arbiter.tick(t0 + milliseconds(301));
  EXPECT_TRUE(timed_out.isZero());
  EXPECT_EQ(timed_out.reason, CmdVelArbiterReason::AUTO_TIMEOUT);
  const auto later = arbiter.tick(t0 + milliseconds(350));
  EXPECT_EQ(later.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
}

TEST(CmdVelArbiter, ExecutionLeaseExpiresWithoutNewCommand)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  armLink(arbiter, t0 + milliseconds(501));
  arbiter.onAuto(1.0, 0.0, 0.0, t0 + milliseconds(501));
  const auto expired = arbiter.tick(t0 + milliseconds(501));
  EXPECT_TRUE(expired.isZero());
  EXPECT_EQ(expired.reason, CmdVelArbiterReason::AUTO_UNAUTHORIZED);
  EXPECT_FALSE(arbiter.autoAuthorized());
}

TEST(CmdVelArbiter, StopRevokesAutoAndRequiresNewAutoSample)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  EXPECT_EQ(arbiter.tick(t0).source, CmdVelSource::AUTO);

  ASSERT_TRUE(arbiter.onExecutionCommand(false, kManagerIncarnation, 2, 7, 11, milliseconds(0), t0 + milliseconds(1)));
  EXPECT_TRUE(arbiter.tick(t0 + milliseconds(2)).isZero());
  EXPECT_FALSE(arbiter.autoAuthorized());

  ASSERT_TRUE(execute(arbiter, 3, milliseconds(0), t0 + milliseconds(3)));
  const auto without_new_auto = arbiter.tick(t0 + milliseconds(4));
  EXPECT_TRUE(without_new_auto.isZero());
  EXPECT_EQ(without_new_auto.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
}

TEST(CmdVelArbiter, StopDoesNotClearFreshManual)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  arbiter.onManual(0.4, -0.1, 0.2, t0);
  ASSERT_TRUE(arbiter.onExecutionCommand(false, kManagerIncarnation, 2, 7, 11, milliseconds(0), t0 + milliseconds(1)));

  const auto output = arbiter.tick(t0 + milliseconds(2));
  EXPECT_EQ(output.source, CmdVelSource::MANUAL);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::MANUAL_FRESH);
  EXPECT_DOUBLE_EQ(output.vx, 0.4);
}

TEST(CmdVelArbiter, NewIncarnationRequiresStopBeforeExecute)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 10, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);

  EXPECT_FALSE(arbiter.onExecutionCommand(true, kManagerIncarnation + 1, 1, 7, 11, milliseconds(0), t0 + milliseconds(1)));
  EXPECT_TRUE(arbiter.tick(t0 + milliseconds(2)).isZero());

  ASSERT_TRUE(arbiter.onExecutionCommand(false, kManagerIncarnation + 1, 1, 7, 11, milliseconds(0), t0 + milliseconds(3)));
  EXPECT_TRUE(arbiter.tick(t0 + milliseconds(4)).isZero());
  EXPECT_FALSE(arbiter.onExecutionCommand(true, kManagerIncarnation, 11, 7, 11, milliseconds(0), t0 + milliseconds(5)));

  ASSERT_TRUE(arbiter.onExecutionCommand(true, kManagerIncarnation + 1, 2, 7, 11, milliseconds(0), t0 + milliseconds(6)));
  arbiter.onAuto(0.7, 0.0, 0.0, t0 + milliseconds(6));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(7)).source, CmdVelSource::AUTO);
}

// 契约：无链路心跳期间缓存的两源命令，在首个 DOWN->UP 心跳后必须保持失效。
// 只有 UP 之后到达的新命令才允许成为 selected 的内容。
TEST(CmdVelArbiter, LinkDownToUpDiscardsCommandsBufferedWhileDown)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onManual(0.4, 0.0, 0.0, t0);
  ASSERT_EQ(arbiter.tick(t0).source, CmdVelSource::MANUAL);

  arbiter.onLinkHealth(false, t0 + milliseconds(10));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(11)).reason, CmdVelArbiterReason::LINK_DOWN);

  // 断链期间到达的两源命令只能进入缓存，不得在恢复后复活。
  arbiter.onManual(0.9, -0.3, 0.2, t0 + milliseconds(20));
  arbiter.onAuto(1.7, 0.6, -0.4, t0 + milliseconds(20));
  arbiter.onLinkHealth(true, t0 + milliseconds(30));
  const auto after_up = arbiter.tick(t0 + milliseconds(31));
  EXPECT_TRUE(after_up.isZero());
  EXPECT_EQ(after_up.source, CmdVelSource::NONE);
  EXPECT_EQ(after_up.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
  EXPECT_FALSE(arbiter.autoAuthorized());

  // UP 之后的新手动命令必须立刻恢复权威，证明失效不是永久闭锁。
  arbiter.onManual(0.5, 0.0, 0.0, t0 + milliseconds(40));
  const auto after_new_manual = arbiter.tick(t0 + milliseconds(41));
  EXPECT_DOUBLE_EQ(after_new_manual.vx, 0.5);
  EXPECT_EQ(after_new_manual.source, CmdVelSource::MANUAL);
  EXPECT_EQ(after_new_manual.reason, CmdVelArbiterReason::MANUAL_FRESH);
}

// 心跳超时导致的 DOWN 与显式 `false` 心跳走不同代码路径，
// 但 DOWN->UP 后旧缓存失效的结论必须一致。
TEST(CmdVelArbiter, HeartbeatTimeoutThenLinkUpDiscardsBufferedCommands)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(1.0, 0.0, 0.0, t0);
  ASSERT_EQ(arbiter.tick(t0).source, CmdVelSource::AUTO);

  // 心跳停发超过 link_timeout：这一拍即判链路失效。
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(301)).reason, CmdVelArbiterReason::LINK_DOWN);
  EXPECT_FALSE(arbiter.linkUp());

  arbiter.onManual(0.8, 0.0, 0.0, t0 + milliseconds(310));
  arbiter.onAuto(1.1, 0.0, 0.0, t0 + milliseconds(310));
  armLink(arbiter, t0 + milliseconds(320));
  const auto after_up = arbiter.tick(t0 + milliseconds(321));
  EXPECT_TRUE(after_up.isZero());
  EXPECT_EQ(after_up.reason, CmdVelArbiterReason::ZERO_NO_SOURCE);
  EXPECT_FALSE(arbiter.autoAuthorized());
}

// 契约：急停期间到达的 ExecutionCommand 不建立租约；
// 单独的 `emergency_stop=false` 不构成新授权，必须等急停解除后的新 EXECUTE。
TEST(CmdVelArbiter, ExecutionCommandDuringEmergencyStopIsNotHonored)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  arbiter.onEmergencyStop(true);

  // 样本本身合法（新鲜、序号递增），但急停期间不得转化为授权。
  EXPECT_TRUE(execute(arbiter, 1, milliseconds(0), t0 + milliseconds(1)));
  EXPECT_FALSE(arbiter.autoAuthorized());
  arbiter.onAuto(1.0, 0.0, 0.0, t0 + milliseconds(2));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(3)).reason, CmdVelArbiterReason::EMERGENCY_STOP);

  // 急停解除本身不恢复运动：自动源持续刷新也只能拿到未授权归零。
  arbiter.onEmergencyStop(false);
  arbiter.onAuto(1.0, 0.0, 0.0, t0 + milliseconds(10));
  const auto after_clear = arbiter.tick(t0 + milliseconds(11));
  EXPECT_TRUE(after_clear.isZero());
  EXPECT_EQ(after_clear.reason, CmdVelArbiterReason::AUTO_UNAUTHORIZED);

  // 解除后的新 EXECUTE 才重建租约。
  ASSERT_TRUE(execute(arbiter, 2, milliseconds(0), t0 + milliseconds(20)));
  arbiter.onAuto(1.0, 0.0, 0.0, t0 + milliseconds(20));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(21)).source, CmdVelSource::AUTO);
}

// 契约：MuJoCo / Gazebo / loopback 以 `require_serial_link=false` 启动，
// 此时 link_timeout=0，节点构造时一次性 arm 链路，之后没有任何心跳也不得误判 LINK_DOWN。
TEST(CmdVelArbiter, NoSerialLinkRequirementProfileKeepsManualAuthority)
{
  CmdVelArbiterConfig sim_config = config();
  sim_config.link_timeout = milliseconds(0);
  CmdVelArbiter arbiter(sim_config);
  const auto t0 = std::chrono::steady_clock::now();
  arbiter.onSerialLinkUp(t0);

  // 远超实机 link_timeout 的时刻，手动源仍然是权威。
  arbiter.onManual(0.3, 0.0, 0.0, t0 + std::chrono::seconds(10));
  const auto output = arbiter.tick(t0 + std::chrono::seconds(10));
  EXPECT_DOUBLE_EQ(output.vx, 0.3);
  EXPECT_EQ(output.source, CmdVelSource::MANUAL);
  EXPECT_EQ(output.reason, CmdVelArbiterReason::MANUAL_FRESH);

  // 该 profile 下急停仍然必须归零。
  arbiter.onEmergencyStop(true);
  EXPECT_EQ(
    arbiter.tick(t0 + std::chrono::seconds(10)).reason, CmdVelArbiterReason::EMERGENCY_STOP);
}

// 手动超时归零之后，新的手动命令必须能重新取得权威。
TEST(CmdVelArbiter, ManualTimeoutThenNewManualIsAccepted)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  arbiter.onManual(0.8, 0.0, 0.0, t0);
  armLink(arbiter, t0 + milliseconds(301));
  ASSERT_EQ(arbiter.tick(t0 + milliseconds(301)).reason, CmdVelArbiterReason::MANUAL_TIMEOUT);

  arbiter.onManual(-0.6, 0.2, -0.1, t0 + milliseconds(310));
  const auto resumed = arbiter.tick(t0 + milliseconds(311));
  EXPECT_DOUBLE_EQ(resumed.vx, -0.6);
  EXPECT_DOUBLE_EQ(resumed.vy, 0.2);
  EXPECT_DOUBLE_EQ(resumed.wz, -0.1);
  EXPECT_EQ(resumed.source, CmdVelSource::MANUAL);
}

// 契约：四舵轮命令是车体系 `[vx, vy, wz]`。
// arbiter 只做选择，不得做任何旋转、增益或差速/`vy=0` 约束。
TEST(CmdVelArbiter, BodyFrameHolonomicComponentsArePreserved)
{
  CmdVelArbiter arbiter(config());
  const auto t0 = std::chrono::steady_clock::now();
  armLink(arbiter, t0);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), t0));
  arbiter.onAuto(-1.25, 0.75, -2.5, t0);
  const auto auto_output = arbiter.tick(t0);
  EXPECT_DOUBLE_EQ(auto_output.vx, -1.25);
  EXPECT_DOUBLE_EQ(auto_output.vy, 0.75);
  EXPECT_DOUBLE_EQ(auto_output.wz, -2.5);

  arbiter.onManual(0.11, -0.22, 0.33, t0 + milliseconds(1));
  const auto manual_output = arbiter.tick(t0 + milliseconds(2));
  EXPECT_DOUBLE_EQ(manual_output.vx, 0.11);
  EXPECT_DOUBLE_EQ(manual_output.vy, -0.22);
  EXPECT_DOUBLE_EQ(manual_output.wz, 0.33);
}

namespace
{

struct RejectedCommand
{
  const char * name;
  uint8_t mode;
  uint64_t incarnation;
  uint64_t sequence;
  std::chrono::nanoseconds age;
};

class CmdVelArbiterRejection : public ::testing::TestWithParam<RejectedCommand> {};

TEST_P(CmdVelArbiterRejection, RevokesActiveAutoWithoutResettingReplayWatermarks)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  armLink(arbiter, now);
  ASSERT_TRUE(execute(arbiter, 10, milliseconds(0), now));
  arbiter.onAuto(1.0, -0.2, 0.3, now);
  ASSERT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);

  const auto & rejected = GetParam();
  EXPECT_FALSE(arbiter.onExecutionCommand(rejected.mode, rejected.incarnation, rejected.sequence, 7, 11, rejected.age, now));
  EXPECT_FALSE(arbiter.autoAuthorized());
  EXPECT_TRUE(arbiter.tick(now).isZero());

  // Invalid input must neither erase nor advance the accepted identity watermark.
  EXPECT_FALSE(execute(arbiter, 10, milliseconds(0), now));
  ASSERT_TRUE(execute(arbiter, 11, milliseconds(0), now));
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onAuto(0.6, 0.1, -0.2, now);
  const auto recovered = arbiter.tick(now);
  EXPECT_EQ(recovered.source, CmdVelSource::AUTO);
  EXPECT_DOUBLE_EQ(recovered.vx, 0.6);
}

TEST_P(CmdVelArbiterRejection, PreservesIndependentManualAuthority)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  armLink(arbiter, now);
  ASSERT_TRUE(execute(arbiter, 10, milliseconds(0), now));
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  arbiter.onManual(0.4, -0.1, 0.2, now);
  const auto & rejected = GetParam();
  EXPECT_FALSE(arbiter.onExecutionCommand(rejected.mode, rejected.incarnation, rejected.sequence, 7, 11, rejected.age, now));
  EXPECT_FALSE(arbiter.autoAuthorized());
  const auto output = arbiter.tick(now);
  EXPECT_EQ(output.source, CmdVelSource::MANUAL);
  EXPECT_DOUBLE_EQ(output.vx, 0.4);
  EXPECT_DOUBLE_EQ(output.vy, -0.1);
  EXPECT_DOUBLE_EQ(output.wz, 0.2);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidAuthorization, CmdVelArbiterRejection,
  ::testing::Values(
    RejectedCommand{"FutureNanosecond", 1, kManagerIncarnation, 99,
      std::chrono::nanoseconds(-1)},
    RejectedCommand{"StaleNanosecond", 1, kManagerIncarnation, 99,
      milliseconds(500) + std::chrono::nanoseconds(1)},
    RejectedCommand{"Replay", 1, kManagerIncarnation, 10, milliseconds(0)},
    RejectedCommand{"OlderSequence", 1, kManagerIncarnation, 9, milliseconds(0)},
    RejectedCommand{"OldIncarnation", 1, kManagerIncarnation - 1, 99, milliseconds(0)},
    RejectedCommand{"NewOwnerWithoutStop", 1, kManagerIncarnation + 1, 99, milliseconds(0)},
    RejectedCommand{"InvalidMode", 2, kManagerIncarnation + 1, 99, milliseconds(0)},
    RejectedCommand{"MissingIncarnation", 1, 0, 99, milliseconds(0)},
    RejectedCommand{"MissingSequence", 1, kManagerIncarnation, 0, milliseconds(0)}),
  [](const ::testing::TestParamInfo<RejectedCommand> & info) {return info.param.name;});

}  // namespace

TEST(CmdVelArbiterMapAuthority, UnknownOldFutureAndZeroIdentityConsumeSequence)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  arbiter.onSerialLinkUp(now);
  arbiter.onMapReady(true, now);
  EXPECT_FALSE(execute(arbiter, 1, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 11, 1, true, false, false);
  EXPECT_FALSE(execute(arbiter, 1, milliseconds(0), now));
  ASSERT_TRUE(execute(arbiter, 2, milliseconds(0), now));
  for (const uint64_t generation : {0U, 10U, 12U}) {
    arbiter.onAuto(1.0, 0.0, 0.0, now);
    const uint64_t sequence = 10 + generation;
    EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, sequence, 7, generation, milliseconds(0), now));
    EXPECT_TRUE(arbiter.tick(now).isZero());
    EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, sequence, 7, 11, milliseconds(0), now));
  }
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 30, 0, 11, milliseconds(0), now));
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 31, 6, 11, milliseconds(0), now));
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 32, 8, 11, milliseconds(0), now));
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 32, 7, 11, milliseconds(0), now));
  ASSERT_TRUE(execute(arbiter, 33, milliseconds(0), now));
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onAuto(0.7, -0.2, 0.3, now);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);
}

TEST(CmdVelArbiterMapAuthority, GenerationAndEpochAdvanceRevokeWithoutStatusOnlyResume)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  armLink(arbiter, now);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), now));
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  arbiter.onPlannerStatus(7, 11, 2, true, false, false);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);
  arbiter.onPlannerStatus(7, 12, 3, true, false, false);
  EXPECT_FALSE(arbiter.autoAuthorized());
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onPlannerStatus(7, 11, 100, true, false, false);
  EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), now));
  ASSERT_TRUE(arbiter.onExecutionCommand(1, 100, 3, 7, 12, milliseconds(0), now));
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);
  arbiter.onPlannerStatus(8, 1, 1, true, false, false);
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onPlannerStatus(7, 99, 1000, true, false, false);
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 4, 7, 99, milliseconds(0), now));
  ASSERT_TRUE(arbiter.onExecutionCommand(1, 100, 5, 8, 1, milliseconds(0), now));
  EXPECT_TRUE(arbiter.tick(now).isZero());
}

TEST(CmdVelArbiterMapAuthority, FailedStatusOrderingAndMapRetirement)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  armLink(arbiter, now);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), now));
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  arbiter.onPlannerStatus(7, 11, 10, false, true, false);
  EXPECT_TRUE(arbiter.tick(now).isZero());
  arbiter.onPlannerStatus(7, 11, 9, true, false, false);
  arbiter.onPlannerStatus(7, 11, 10, true, false, false);
  EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 11, 11, true, false, false);
  EXPECT_FALSE(arbiter.autoAuthorized());
  EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), now));
  ASSERT_TRUE(execute(arbiter, 3, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 11, 12, false, true, true);
  arbiter.onPlannerStatus(7, 11, 13, true, false, false);
  EXPECT_FALSE(execute(arbiter, 4, milliseconds(0), now));
  // SNAPSHOT_CHANGED names a new usable candidate, not a permanently retired map.
  arbiter.onPlannerStatus(7, 12, 14, false, true, false);
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 5, 7, 12, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 12, 15, true, false, false);
  EXPECT_FALSE(arbiter.autoAuthorized());
  ASSERT_TRUE(arbiter.onExecutionCommand(1, 100, 6, 7, 12, milliseconds(0), now));
}

TEST(CmdVelArbiterMapAuthority, AcceptedCannotEstablishAuthorityOrChurnHealthyGeneration)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  arbiter.onSerialLinkUp(now);
  arbiter.onMapReady(true, now);
  arbiter.onPlannerStatus(7, 0, 1, false, false, false);
  arbiter.onPlannerStatus(7, 11, 2, false, false, false);
  EXPECT_FALSE(execute(arbiter, 1, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 11, 3, true, false, false);
  ASSERT_TRUE(execute(arbiter, 2, milliseconds(0), now));
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  arbiter.onPlannerStatus(7, 11, 4, false, false, false);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);
  arbiter.onPlannerStatus(7, 12, 5, false, false, false);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::AUTO);
}

TEST(CmdVelArbiterMapAuthority, ReadyFalsePreservesManualAndRequiresNewGenerationAndAuthorization)
{
  CmdVelArbiter arbiter(config());
  const auto now = std::chrono::steady_clock::now();
  armLink(arbiter, now);
  ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), now));
  arbiter.onAuto(1.0, 0.0, 0.0, now);
  arbiter.onManual(0.4, -0.1, 0.2, now);
  arbiter.onMapReady(false, now);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::MANUAL);
  EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), now));
  arbiter.onPlannerStatus(7, 12, 2, true, false, false);
  EXPECT_EQ(arbiter.tick(now).source, CmdVelSource::MANUAL);
  arbiter.onMapReady(true, now);
  EXPECT_FALSE(arbiter.autoAuthorized());
  EXPECT_FALSE(arbiter.onExecutionCommand(1, 100, 2, 7, 12, milliseconds(0), now));
  ASSERT_TRUE(arbiter.onExecutionCommand(1, 100, 3, 7, 12, milliseconds(0), now));
  arbiter.onLinkHealth(true, now + milliseconds(301));
  EXPECT_TRUE(arbiter.tick(now + milliseconds(301)).isZero());
}

TEST(CmdVelArbiterMapAuthority, ReadyLeaseExpiryAndLateHeartbeatRetireGeneration)
{
  for (const bool tick_before_heartbeat : {false, true}) {
    auto settings = config();
    settings.map_ready_timeout = milliseconds(50);
    CmdVelArbiter arbiter(settings);
    const auto now = std::chrono::steady_clock::now();
    armLink(arbiter, now);
    ASSERT_TRUE(execute(arbiter, 1, milliseconds(0), now));
    arbiter.onAuto(1.0, 0.0, 0.0, now);
    EXPECT_EQ(arbiter.tick(now + milliseconds(50)).source, CmdVelSource::AUTO);
    if (tick_before_heartbeat) {
      EXPECT_TRUE(arbiter.tick(now + milliseconds(51)).isZero());
    }
    arbiter.onMapReady(true, now + milliseconds(51));
    arbiter.onPlannerStatus(7, 11, 2, true, false, false);
    EXPECT_FALSE(execute(arbiter, 2, milliseconds(0), now + milliseconds(51)));
    arbiter.onPlannerStatus(7, 12, 3, true, false, false);
    EXPECT_FALSE(arbiter.autoAuthorized());
    ASSERT_TRUE(arbiter.onExecutionCommand(
      1, 100, 3, 7, 12, milliseconds(0), now + milliseconds(51)));
    EXPECT_TRUE(arbiter.tick(now + milliseconds(51)).isZero());
    arbiter.onAuto(0.5, 0.0, 0.0, now + milliseconds(51));
    EXPECT_EQ(arbiter.tick(now + milliseconds(51)).source, CmdVelSource::AUTO);
  }
}
