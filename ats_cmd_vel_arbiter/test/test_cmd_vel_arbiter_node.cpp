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
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
using Twist = geometry_msgs::msg::Twist;

enum class Rejection {
  FUTURE, STALE, REPLAY, OLD_INCARNATION, INVALID_MODE,
  OLD_GENERATION, FUTURE_GENERATION, ZERO_GENERATION, OLD_EPOCH, FUTURE_EPOCH
};

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
      rclcpp::Parameter("manual_timeout_ms", 5000),
      rclcpp::Parameter("map_ready_timeout", 30.0),
      rclcpp::Parameter("execution_command_timeout", 5.0),
      rclcpp::Parameter("manual_cmd_vel_topic", "/test_arbiter/manual"),
      rclcpp::Parameter("autonomy_cmd_vel_topic", "/test_arbiter/auto"),
      rclcpp::Parameter("selected_cmd_vel_topic", "/test_arbiter/selected"),
      rclcpp::Parameter("execution_command_topic", "/test_arbiter/execution"),
      rclcpp::Parameter("planner_status_topic", "/test_arbiter/status"),
      rclcpp::Parameter("map_ready_topic", "/test_arbiter/ready"),
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
    status_pub_ = driver_->create_publisher<PlannerStatus>(
      "/test_arbiter/status", rclcpp::QoS(1).reliable().transient_local());
    ready_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_arbiter/ready", rclcpp::QoS(1).reliable().transient_local());
    manual_pub_ = driver_->create_publisher<Twist>("/test_arbiter/manual", 10);
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
             status_pub_->get_subscription_count() == 1 &&
             ready_pub_->get_subscription_count() == 1 &&
             manual_pub_->get_subscription_count() == 1 &&
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
    message.localization_epoch = 7;
    message.map_generation = 11;
    return message;
  }

  PlannerStatus status(uint64_t generation = 11, uint32_t stamp_offset = 0)
  {
    PlannerStatus message;
    message.header.frame_id = "map";
    message.header.stamp = arbiter_->now();
    message.header.stamp.nanosec += stamp_offset;
    message.localization_epoch = 7;
    message.map_generation = generation;
    message.map_publication_sequence = 100 + stamp_offset;
    message.state = PlannerStatus::STATE_REFERENCE_READY;
    message.failure_reason = PlannerStatus::FAILURE_NONE;
    return message;
  }

  void establishAuthority()
  {
    const auto before = selected_count_;
    status_pub_->publish(status());
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
    ASSERT_TRUE(spinUntil([this, before]() {return selected_count_ >= before + 2;}, 500ms));
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ats_cmd_vel_arbiter::CmdVelArbiterNode> arbiter_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<ExecutionCommand>::SharedPtr execution_pub_;
  rclcpp::Publisher<PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<Twist>::SharedPtr manual_pub_;
  rclcpp::Publisher<Twist>::SharedPtr auto_pub_;
  rclcpp::Subscription<Twist>::SharedPtr selected_sub_;
  Twist selected_;
  size_t selected_count_{0};
};

TEST_P(CmdVelArbiterNodeRejection, PublishesImmediateZeroAndRequiresFreshVelocityForRecovery)
{
  establishAuthority();
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
    case Rejection::OLD_GENERATION:
      rejected.map_generation = 10;
      break;
    case Rejection::FUTURE_GENERATION:
      rejected.map_generation = 12;
      break;
    case Rejection::ZERO_GENERATION:
      rejected.map_generation = 0;
      break;
    case Rejection::OLD_EPOCH:
      rejected.localization_epoch = 6;
      break;
    case Rejection::FUTURE_EPOCH:
      rejected.localization_epoch = 8;
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

  execution_pub_->publish(command(100));
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
    Rejection::OLD_INCARNATION, Rejection::INVALID_MODE,
    Rejection::OLD_GENERATION, Rejection::FUTURE_GENERATION, Rejection::ZERO_GENERATION,
    Rejection::OLD_EPOCH, Rejection::FUTURE_EPOCH));

TEST_F(CmdVelArbiterNodeRejection, UnknownGenerationNeedsStatusThenHigherSequence)
{
  std_msgs::msg::Bool ready;
  ready.data = true;
  ready_pub_->publish(ready);
  ASSERT_TRUE(spinUntil([this]() {return selected_count_ > 0;}, 500ms));
  const auto before = selected_count_;
  execution_pub_->publish(command(10));
  ASSERT_TRUE(spinUntil([this, before]() {return selected_count_ > before;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  const auto status_received = selected_count_;
  status_pub_->publish(status());
  ASSERT_TRUE(spinUntil([this, status_received]() {return selected_count_ > status_received;}, 200ms));
  Twist velocity;
  velocity.linear.x = 0.8;
  auto_pub_->publish(velocity);
  execution_pub_->publish(command(10));
  const auto rejected = selected_count_;
  ASSERT_TRUE(spinUntil([this, rejected]() {return selected_count_ > rejected;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  execution_pub_->publish(command(11));
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
}

TEST_F(CmdVelArbiterNodeRejection, PublicationHeartbeatDoesNotChurnButNewGenerationZerosImmediately)
{
  establishAuthority();
  execution_pub_->publish(command(10));
  Twist velocity;
  velocity.linear.x = 0.8;
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
  const auto heartbeat = selected_count_;
  status_pub_->publish(status(11, 1));
  ASSERT_TRUE(spinUntil([this, heartbeat]() {return selected_count_ > heartbeat;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.8);
  const auto changed = selected_count_;
  status_pub_->publish(status(12, 2));
  ASSERT_TRUE(spinUntil([this, changed]() {
    return selected_count_ > changed && selected_.linear.x == 0.0;
  }, 200ms));
  const auto stale = selected_count_;
  status_pub_->publish(status(11, 3));
  ASSERT_TRUE(spinUntil([this, stale]() {return selected_count_ > stale;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  auto next = command(11);
  next.map_generation = 12;
  execution_pub_->publish(next);
  const auto authorized = selected_count_;
  ASSERT_TRUE(spinUntil([this, authorized]() {return selected_count_ > authorized;}, 1500ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
}

TEST_F(CmdVelArbiterNodeRejection, ReadyRecoveryCannotResumeCachedAuto)
{
  establishAuthority();
  execution_pub_->publish(command(10));
  Twist velocity;
  velocity.linear.x = 0.8;
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
  const auto stopped = selected_count_;
  ready_pub_->publish(std_msgs::msg::Bool{});
  ASSERT_TRUE(spinUntil([this, stopped]() {
    return selected_count_ > stopped && selected_.linear.x == 0.0;
  }, 200ms));
  const auto new_status = selected_count_;
  status_pub_->publish(status(12, 1));
  ASSERT_TRUE(spinUntil([this, new_status]() {return selected_count_ > new_status;}, 200ms));
  std_msgs::msg::Bool ready;
  ready.data = true;
  const auto recovered = selected_count_;
  ready_pub_->publish(ready);
  ASSERT_TRUE(spinUntil([this, recovered]() {return selected_count_ > recovered;}, 200ms));
  auto_pub_->publish(velocity);
  const auto no_authorization = selected_count_;
  ASSERT_TRUE(spinUntil([this, no_authorization]() {
    return selected_count_ > no_authorization;
  }, 1500ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  auto next = command(11);
  next.map_generation = 12;
  execution_pub_->publish(next);
  auto_pub_->publish(velocity);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.8;}, 1500ms));
}

TEST_F(CmdVelArbiterNodeRejection, MissingMapAuthorityRejectionPreservesManualSelection)
{
  Twist manual;
  manual.linear.x = 0.4;
  manual.linear.y = -0.1;
  manual.angular.z = 0.2;
  manual_pub_->publish(manual);
  ASSERT_TRUE(spinUntil([this]() {return selected_.linear.x == 0.4;}, 1500ms));
  const auto rejected = selected_count_;
  execution_pub_->publish(command(10));
  ASSERT_TRUE(spinUntil([this, rejected]() {return selected_count_ > rejected;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected_.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(selected_.angular.z, 0.2);
  const auto unready = selected_count_;
  ready_pub_->publish(std_msgs::msg::Bool{});
  ASSERT_TRUE(spinUntil([this, unready]() {return selected_count_ > unready;}, 200ms));
  EXPECT_DOUBLE_EQ(selected_.linear.x, 0.4);
}

TEST_F(CmdVelArbiterNodeRejection, MalformedStatusAndAcceptedTelemetryCannotAuthorizeAuto)
{
  std_msgs::msg::Bool ready;
  ready.data = true;
  ready_pub_->publish(ready);
  ASSERT_TRUE(spinUntil([this]() {return selected_count_ > 0;}, 500ms));
  for (int malformed = 0; malformed < 5; ++malformed) {
    auto message = status();
    switch (malformed) {
      case 0:
        message.header.frame_id.clear();
        break;
      case 1:
        message.header.stamp.sec = -1;
        break;
      case 2:
        message.header.stamp.nanosec = 1000000000U;
        break;
      case 3:
        message.header.stamp.sec = 0;
        message.header.stamp.nanosec = 0;
        break;
      case 4:
        message.state = PlannerStatus::STATE_ACCEPTED;
        break;
    }
    status_pub_->publish(message);
    const auto observed_status = selected_count_;
    ASSERT_TRUE(spinUntil([this, observed_status]() {
      return selected_count_ > observed_status;
    }, 1500ms));
    execution_pub_->publish(command(10 + malformed));
    Twist velocity;
    velocity.linear.x = 0.8;
    auto_pub_->publish(velocity);
    const auto before = selected_count_;
    ASSERT_TRUE(spinUntil([this, before]() {return selected_count_ > before;}, 1500ms));
    EXPECT_DOUBLE_EQ(selected_.linear.x, 0.0);
  }
}

}  // namespace
