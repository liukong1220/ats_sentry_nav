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
}

constexpr uint64_t kManagerIncarnation = 100;

bool execute(
  CmdVelArbiter & arbiter, uint64_t sequence, milliseconds age,
  std::chrono::steady_clock::time_point now)
{
  return arbiter.onExecutionCommand(true, kManagerIncarnation, sequence, age, now);
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
  EXPECT_TRUE(arbiter.autoAuthorized());
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

  ASSERT_TRUE(arbiter.onExecutionCommand(
    false, kManagerIncarnation, 2, milliseconds(0), t0 + milliseconds(1)));
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
  ASSERT_TRUE(arbiter.onExecutionCommand(
    false, kManagerIncarnation, 2, milliseconds(0), t0 + milliseconds(1)));

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

  EXPECT_FALSE(arbiter.onExecutionCommand(
    true, kManagerIncarnation + 1, 1, milliseconds(0), t0 + milliseconds(1)));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(2)).source, CmdVelSource::AUTO);

  ASSERT_TRUE(arbiter.onExecutionCommand(
    false, kManagerIncarnation + 1, 1, milliseconds(0), t0 + milliseconds(3)));
  EXPECT_TRUE(arbiter.tick(t0 + milliseconds(4)).isZero());
  EXPECT_FALSE(arbiter.onExecutionCommand(
    true, kManagerIncarnation, 11, milliseconds(0), t0 + milliseconds(5)));

  ASSERT_TRUE(arbiter.onExecutionCommand(
    true, kManagerIncarnation + 1, 2, milliseconds(0), t0 + milliseconds(6)));
  arbiter.onAuto(0.7, 0.0, 0.0, t0 + milliseconds(6));
  EXPECT_EQ(arbiter.tick(t0 + milliseconds(7)).source, CmdVelSource::AUTO);
}
