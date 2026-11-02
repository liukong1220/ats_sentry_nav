// Copyright 2026

#include <cmath>

#include "ats_goal_manager/terminal_motion_window.hpp"
#include "gtest/gtest.h"

namespace
{

using ats_goal_manager::TerminalMotionWindow;

// 确定性的"位置噪声"：±3 mm 内的伪随机序列，零均值。
double jitter(int i, double amplitude)
{
  return amplitude * std::sin(12.9898 * i + 78.233 * std::sin(0.37 * i));
}

TEST(TerminalMotionWindow, DisabledByDefaultAndReportsNothing)
{
  TerminalMotionWindow window;
  EXPECT_FALSE(window.enabled());
  window.add(0.0, 0.0, 0.0, 0.0);
  window.add(1.0, 0.0, 0.0, 0.0);
  double linear = 0.0;
  double angular = 0.0;
  EXPECT_FALSE(window.averageSpeed(linear, angular));
}

TEST(TerminalMotionWindow, NeedsMostOfTheWindowBeforeAnswering)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  double linear = 0.0;
  double angular = 0.0;
  window.add(10.00, 1.0, 2.0, 0.1);
  EXPECT_FALSE(window.averageSpeed(linear, angular));
  window.add(10.20, 1.0, 2.0, 0.1);
  EXPECT_FALSE(window.averageSpeed(linear, angular));   // 0.2 s < 0.8 * 0.5 s
  window.add(10.41, 1.0, 2.0, 0.1);
  EXPECT_TRUE(window.averageSpeed(linear, angular));
  EXPECT_NEAR(linear, 0.0, 1e-12);
  EXPECT_NEAR(angular, 0.0, 1e-12);
}

TEST(TerminalMotionWindow, PositionNoiseAtStandstillStaysFarBelowTheToleranceThatTwistNoiseExceeds)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  double worst = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double t = 100.0 + 0.01 * i;   // 100 Hz 里程计
    window.add(t, 5.0 + jitter(i, 0.004), -2.0 + jitter(i + 1000, 0.004), 0.3 + jitter(i + 7, 0.002));
    double linear = 0.0;
    double angular = 0.0;
    if (window.averageSpeed(linear, angular)) {
      worst = std::max(worst, linear);
      EXPECT_LT(angular, 0.10);
    }
  }
  // 窗口 0.5 s、位置噪声 ±4 mm：平均速度 < 0.05 m/s 的 5 倍余量。
  EXPECT_LT(worst, 0.025);
}

TEST(TerminalMotionWindow, RealMotionAtTheToleranceSpeedIsNotCalledStopped)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  double linear = 0.0;
  double angular = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double t = 0.01 * i;
    window.add(t, 0.12 * t, 0.0, 0.0);   // 0.12 m/s
  }
  ASSERT_TRUE(window.averageSpeed(linear, angular));
  EXPECT_NEAR(linear, 0.12, 1e-6);
  EXPECT_GT(linear, 0.05);
}

TEST(TerminalMotionWindow, SlowYawDriftIsMeasuredAcrossTheWrap)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  constexpr double pi = 3.14159265358979323846;
  double linear = 0.0;
  double angular = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double t = 0.01 * i;
    window.add(t, 0.0, 0.0, std::atan2(std::sin(pi - 0.3 + 0.3 * t), std::cos(pi - 0.3 + 0.3 * t)));
  }
  ASSERT_TRUE(window.averageSpeed(linear, angular));
  EXPECT_NEAR(angular, 0.3, 1e-6);
  EXPECT_NEAR(linear, 0.0, 1e-12);
}

TEST(TerminalMotionWindow, ClockGoingBackwardsDiscardsTheOldWindow)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  double linear = 0.0;
  double angular = 0.0;
  for (int i = 0; i < 100; ++i) {
    window.add(50.0 + 0.01 * i, 0.5 * i, 0.0, 0.0);   // 快速运动
  }
  window.add(3.0, 9.0, 9.0, 0.0);   // 仿真重置，时间倒退
  EXPECT_FALSE(window.averageSpeed(linear, angular));
  for (int i = 1; i <= 60; ++i) {
    window.add(3.0 + 0.01 * i, 9.0, 9.0, 0.0);
  }
  ASSERT_TRUE(window.averageSpeed(linear, angular));
  EXPECT_NEAR(linear, 0.0, 1e-12);
}

TEST(TerminalMotionWindow, IgnoresRepeatedStampsAndNonFiniteInput)
{
  TerminalMotionWindow window;
  window.setWindow(0.5);
  double linear = 0.0;
  double angular = 0.0;
  window.add(1.0, 0.0, 0.0, 0.0);
  window.add(1.0, 9.0, 9.0, 1.0);   // 同一时间戳的重复消息不改变窗口
  window.add(std::nan(""), 0.0, 0.0, 0.0);
  window.add(1.6, 0.0, 0.0, 0.0);
  ASSERT_TRUE(window.averageSpeed(linear, angular));
  EXPECT_NEAR(linear, 0.0, 1e-12);
  EXPECT_NEAR(angular, 0.0, 1e-12);
}

}  // namespace
