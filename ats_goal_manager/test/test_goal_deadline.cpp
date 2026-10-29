// Copyright 2026

#include "ats_goal_manager/goal_deadline.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace ats_goal_manager
{
namespace
{
constexpr std::int64_t kSecond = 1000000000LL;
}  // namespace

TEST(GoalDeadline, CountsNodeClockTimeNotWallTime)
{
  // 0.057 倍实时：600 s 的目标在仿真时间里要走满 600 仿真秒才超时，
  // 墙钟走了多少与这里无关（输入只有节点时钟）。
  GoalDeadline deadline(100 * kSecond, 600 * kSecond);
  EXPECT_TRUE(deadline.armed());
  EXPECT_FALSE(deadline.expired(100 * kSecond + 34 * kSecond));
  EXPECT_EQ(deadline.elapsed(100 * kSecond + 34 * kSecond), 34 * kSecond);
  EXPECT_FALSE(deadline.expired(100 * kSecond + 600 * kSecond - 1));
  EXPECT_TRUE(deadline.expired(100 * kSecond + 600 * kSecond));
}

TEST(GoalDeadline, WaitsForTheClockBeforeCounting)
{
  // use_sim_time 下目标先于第一条 /clock 到达：now() 为 0，不能把之后读到的整个
  // 仿真时间算成已用时间。
  GoalDeadline deadline(0, 10 * kSecond);
  EXPECT_FALSE(deadline.armed());
  EXPECT_FALSE(deadline.expired(0));
  EXPECT_EQ(deadline.elapsed(0), 0);
  EXPECT_FALSE(deadline.expired(5000 * kSecond));
  EXPECT_TRUE(deadline.armed());
  EXPECT_FALSE(deadline.expired(5009 * kSecond));
  EXPECT_TRUE(deadline.expired(5010 * kSecond));
}

TEST(GoalDeadline, RestartsWhenTheClockJumpsBackwards)
{
  GoalDeadline deadline(1000 * kSecond, 10 * kSecond);
  EXPECT_FALSE(deadline.expired(1005 * kSecond));
  // 仿真重启：时钟回到 2 s。从 2 s 重新计，既不永不超时，也不立刻超时。
  EXPECT_FALSE(deadline.expired(2 * kSecond));
  EXPECT_EQ(deadline.elapsed(2 * kSecond), 0);
  EXPECT_FALSE(deadline.expired(11 * kSecond));
  EXPECT_TRUE(deadline.expired(12 * kSecond));
}

TEST(GoalDeadline, NegativeTimeoutExpiresOnFirstArmedTick)
{
  GoalDeadline deadline(5 * kSecond, -1);
  EXPECT_EQ(deadline.timeoutNs(), 0);
  EXPECT_TRUE(deadline.expired(5 * kSecond));
}

}  // namespace ats_goal_manager
