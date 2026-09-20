// Copyright 2026 ATS 2026 Sentry Project

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include "rcl/time.h"

#define ATS_CMD_VEL_ARBITER_NO_MAIN
#include "../src/cmd_vel_arbiter_node.cpp"

namespace
{

using namespace std::chrono_literals;
using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
using Twist = geometry_msgs::msg::Twist;

enum class Rejection {FUTURE, STALE, REPLAY, OLD_INCARNATION, INVALID_MODE};

class CmdVelArbiterNodeRejection : public ::testing::TestWithParam<Rejection>
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("require_serial_link", false),
      rclcpp::Parameter("control_rate_hz", 1.0),
      rclcpp::Parameter("auto_timeout_ms", 5000),
      rclcpp::Parameter("execution_command_timeout", 5.0),
      rclcpp::Parameter("manual_cmd_vel_topic", "/test_arbiter/manual"),
      rclcpp::Parameter("autonomy_cmd_vel_topic", "/test_arbiter/auto"),
      rclcpp::Parameter("selected_cmd_vel_topic", "/test_arbiter/selected"),
      rclcpp::Parameter("execution_command_topic", "/test_arbiter/execution"),
      rclcpp::Parameter("emergency_stop_topic", "/test_arbiter/stop")});
    arbiter_ = std::make_shared<ats_cmd_vel_arbiter::CmdVelArbiterNode>(options);
    // Freeze ROS time so the +1 ns future boundary cannot disappear in DDS transit.
    ASSERT_EQ(rcl_enable_ros_time_override(arbiter_->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(
      rcl_set_ros_time_override(arbiter_->get_clock()->get_clock_handle(), 10000000000LL),
      RCL_RET_OK);
    driver_ = std::make_shared<rclcpp::Node>("test_arbiter_driver");
    execution_pub_ = driver_->create_publisher<ExecutionCommand>(
      "/test_arbiter/execution", rclcpp::QoS(1).reliable().transient_local());
    auto_pub_ = driver_->create_publisher<Twist>("/test_arbiter/auto", 10);
    selected_sub_ = driver_->create_subscription<Twist>(
      "/test_arbiter/selected", 10,
      [this](Twist::SharedPtr message) {
        selected_ = *message;
        ++selected_count_;
      });
    executor_.add_node(arbiter_);
    executor_.add_node(driver_);
    ASSERT_TRUE(spinUntil([this]() {
      return execution_pub_->get_subscription_count() == 1 &&
             auto_pub_->get_subscription_count() == 1 &&
             selected_sub_->get_publisher_count() == 1;
    }, 2s));
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(arbiter_);
  }

  bool spinUntil(const std::function<bool()> & predicate, std::chrono::steady_clock::duration timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  ExecutionCommand command(uint64_t sequence)
  {
    ExecutionCommand message;
    message.header.stamp = arbiter_->now();
    message.mode = ExecutionCommand::MODE_EXECUTE;
    message.manager_incarnation = 100;
    message.command_sequence = sequence;
    return message;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ats_cmd_vel_arbiter::CmdVelArbiterNode> arbiter_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<ExecutionCommand>::SharedPtr execution_pub_;
  rclcpp::Publisher<Twist>::SharedPtr auto_pub_;
  rclcpp::Subscription<Twist>::SharedPtr selected_sub_;
  Twist selected_;
  size_t selected_count_{0};
};

TEST_P(CmdVelArbiterNodeRejection, PublishesImmediateZeroAndRequiresFreshVelocityForRecovery)
{
  execution_pub_->publish(command(10));
  Twist velocity;
  velocity.linear.x = 0.8;
  velocity.linear.y = -0.2;
  velocity.angular.z = 0.3;
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 2s));

  auto rejected = command(99);
  switch (GetParam()) {
    case Rejection::FUTURE:
      rejected.header.stamp.nanosec = 1;
      break;
    case Rejection::STALE:
      rejected.header.stamp.sec = 4;
      rejected.header.stamp.nanosec = 999999999;
      break;
    case Rejection::REPLAY:
      rejected.command_sequence = 10;
      break;
    case Rejection::OLD_INCARNATION:
      rejected.manager_incarnation = 99;
      break;
    case Rejection::INVALID_MODE:
      rejected.mode = 255;
      break;
  }
  const auto before_rejection = selected_count_;
  execution_pub_->publish(rejected);
  // The 1 Hz timer just published motion. A zero within 200 ms must come from rejection.
  ASSERT_TRUE(spinUntil([this, before_rejection]() {
    return selected_count_ > before_rejection && selected_.linear.x == 0.0;
  }, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(selected_.angular.z, 0.0);

  execution_pub_->publish(command(11));
  const auto before_recovery = selected_count_;
  ASSERT_TRUE(spinUntil([this, before_recovery]() {
    return selected_count_ > before_recovery;
  }, 1500ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected_.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(selected_.angular.z, 0.0);
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
}

INSTANTIATE_TEST_SUITE_P(
  InvalidAuthorization, CmdVelArbiterNodeRejection,
  ::testing::Values(
    Rejection::FUTURE, Rejection::STALE, Rejection::REPLAY,
    Rejection::OLD_INCARNATION, Rejection::INVALID_MODE));

}  // namespace
