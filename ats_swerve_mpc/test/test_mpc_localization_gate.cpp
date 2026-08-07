// Copyright 2026

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <thread>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using namespace std::chrono_literals;
using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;
using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
using GimbalYawStatus = ats_navigation_interfaces::msg::GimbalYawStatus;

class MpcLocalizationGateTest : public ::testing::Test
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
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        rclcpp::Parameter("odom_topic", "/test_mpc_gate/odom"),
        rclcpp::Parameter("trajectory_topic", "/test_mpc_gate/path"),
        rclcpp::Parameter("execution_command_topic", "/test_mpc_gate/execution"),
        rclcpp::Parameter("command_topic", "/test_mpc_gate/cmd"),
        rclcpp::Parameter("emergency_stop_topic", "/test_mpc_gate/stop"),
        rclcpp::Parameter("localization_status_topic", "/test_mpc_gate/status"),
        rclcpp::Parameter("gimbal_status_topic", "/test_mpc_gate/gimbal_status"),
        rclcpp::Parameter("require_localization_status", true),
        rclcpp::Parameter("require_gimbal_status", true),
        rclcpp::Parameter("gimbal_status_timeout", 0.1),
        rclcpp::Parameter("control_rate_hz", 50.0),
        rclcpp::Parameter("emergency_stop_timeout", 5.0),
        rclcpp::Parameter("trajectory_timeout", 2.0),
        rclcpp::Parameter("publish_debug_paths", false),
        rclcpp::Parameter("solver_mode", "qp_shadow"),
      });
    mpc_ = std::make_shared<ats_swerve_mpc::AtsSwerveMpcNode>(options);
    driver_ =
      std::make_shared<rclcpp::Node>("test_mpc_localization_gate_driver");
    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/test_mpc_gate/odom", rclcpp::SensorDataQoS());
    path_pub_ = driver_->create_publisher<nav_msgs::msg::Path>(
      "/test_mpc_gate/path", rclcpp::QoS(1).reliable());
    execution_pub_ = driver_->create_publisher<ExecutionCommand>(
      "/test_mpc_gate/execution", rclcpp::QoS(1).reliable().transient_local());
    stop_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_mpc_gate/stop", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = driver_->create_publisher<LocalizationStatus>(
      "/test_mpc_gate/status", rclcpp::QoS(1).reliable().transient_local());
    gimbal_status_pub_ = driver_->create_publisher<GimbalYawStatus>(
      "/test_mpc_gate/gimbal_status", rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = driver_->create_subscription<geometry_msgs::msg::Twist>(
      "/test_mpc_gate/cmd", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        command_norm_.store(
          std::hypot(message->linear.x, message->linear.y) +
          std::abs(message->angular.z));
      });
    telemetry_client_ = driver_->create_client<std_srvs::srv::Trigger>(
      "/ats_swerve_mpc/dump_control_telemetry");
    executor_.add_node(mpc_);
    executor_.add_node(driver_);
    ASSERT_TRUE(
      spinUntil(
        [this]() {
          return odom_pub_->get_subscription_count() == 1 &&
          path_pub_->get_subscription_count() == 1 &&
          execution_pub_->get_subscription_count() == 1 &&
          stop_pub_->get_subscription_count() == 1 &&
          status_pub_->get_subscription_count() == 1 &&
          gimbal_status_pub_->get_subscription_count() == 1 &&
          telemetry_client_->service_is_ready();
        },
        2s));
    // Production Goal Manager publishes a transient-local STOP at process
    // start. Establish the same handoff before this fixture sends EXECUTE.
    publishExecutionStop();
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(mpc_);
    command_sub_.reset();
    telemetry_client_.reset();
    status_pub_.reset();
    gimbal_status_pub_.reset();
    stop_pub_.reset();
    path_pub_.reset();
    execution_pub_.reset();
    odom_pub_.reset();
    driver_.reset();
    mpc_.reset();
  }

  bool spinUntil(
    const std::function<bool()> & predicate,
    std::chrono::steady_clock::duration timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    executor_.spin_some();
    return predicate();
  }

  void publishOdometry()
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = driver_->now();
    message.header.frame_id = "odom";
    message.child_frame_id = "gimbal_yaw_odom";
    message.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(message);
    publishGimbalStatus();
  }

  void publishGimbalStatus(bool locked = false)
  {
    GimbalYawStatus message;
    message.header.stamp = driver_->now();
    message.sequence = ++gimbal_sequence_;
    message.request_sequence = gimbal_request_sequence_;
    message.yaw_authority =
      GimbalYawStatus::YAW_AUTHORITY_GIMBAL_COMPENSATED;
    message.locked = locked;
    message.tf_healthy = true;
    gimbal_status_pub_->publish(message);
  }

  void publishStatus(std::uint8_t state, std::uint64_t epoch)
  {
    LocalizationStatus message;
    message.header.stamp = driver_->now();
    message.header.frame_id = "map";
    message.state = state;
    message.epoch = epoch;
    status_pub_->publish(message);
  }

  void publishStop(bool stop)
  {
    std_msgs::msg::Bool message;
    message.data = stop;
    stop_pub_->publish(message);
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    const rclcpp::Time start =
      driver_->now() + rclcpp::Duration::from_seconds(0.05);
    path.header.stamp = start;
    path.header.frame_id = "odom";
    for (int index = 0; index <= 20; ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.header.stamp = start + rclcpp::Duration::from_seconds(0.1 * index);
      pose.pose.position.x = 0.05 * index;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publishExecution(std::uint64_t epoch)
  {
    publishExecutionWithSequence(++execution_sequence_, epoch, 1);
  }

  void publishExecutionWithSequence(
    std::uint64_t sequence, std::uint64_t epoch, std::uint64_t map_generation,
    std::uint64_t gimbal_request_sequence = 0,
    std::uint64_t manager_incarnation = 0)
  {
    ExecutionCommand command;
    command.header.stamp = driver_->now();
    command.header.frame_id = "odom";
    command.mode = ExecutionCommand::MODE_EXECUTE;
    command.manager_incarnation = manager_incarnation == 0 ?
      manager_incarnation_ : manager_incarnation;
    command.command_sequence = sequence;
    command.goal_id = 1;
    command.localization_epoch = epoch;
    command.map_generation = map_generation;
    command.map_publication_sequence = 1;
    command.failure_reason = ExecutionCommand::FAILURE_NONE;
    command.yaw_authority =
      ExecutionCommand::YAW_AUTHORITY_GIMBAL_COMPENSATED;
    command.requires_gimbal_lock = false;
    command.gimbal_request_sequence = gimbal_request_sequence == 0 ?
      gimbal_request_sequence_ : gimbal_request_sequence;
    // The real Goal Manager serializes only a status it has already consumed.
    // This fixture publishes status and command from one executor, so the
    // preceding sequence is the latest one visible to the MPC callback.
    command.gimbal_feedback_sequence =
      gimbal_sequence_ > 0 ? gimbal_sequence_ - 1 : 0;
    const rclcpp::Time start =
      driver_->now() + rclcpp::Duration::from_seconds(0.05);
    command.reference.header.stamp = start;
    command.reference.header.frame_id = "odom";
    for (int index = 0; index <= 20; ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.header.stamp = start + rclcpp::Duration::from_seconds(0.1 * index);
      pose.pose.position.x = 0.05 * index;
      pose.pose.orientation.w = 1.0;
      command.reference.poses.push_back(pose);
    }
    execution_pub_->publish(command);
  }

  void publishExecutionStop(
    std::uint64_t manager_incarnation = 0,
    std::uint64_t sequence = 0)
  {
    ExecutionCommand command;
    command.header.stamp = driver_->now();
    command.header.frame_id = "odom";
    command.mode = ExecutionCommand::MODE_STOP;
    command.manager_incarnation = manager_incarnation == 0 ?
      manager_incarnation_ : manager_incarnation;
    command.command_sequence = sequence == 0 ? ++execution_sequence_ : sequence;
    command.goal_id = 1;
    command.localization_epoch = 1;
    command.failure_reason = ExecutionCommand::FAILURE_RUNTIME_UNSAFE;
    execution_pub_->publish(command);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ats_swerve_mpc::AtsSwerveMpcNode> mpc_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<ExecutionCommand>::SharedPtr execution_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<GimbalYawStatus>::SharedPtr gimbal_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr telemetry_client_;
  std::atomic<double> command_norm_{0.0};
  std::uint64_t execution_sequence_{0};
  std::uint64_t manager_incarnation_{1};
  std::uint64_t gimbal_sequence_{0};
  std::uint64_t gimbal_request_sequence_{1};
};

TEST_F(
  MpcLocalizationGateTest,
  RejectsReferenceReceivedWhileLocalizationIsUnhealthy) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  publishStop(false);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishStop(false);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));

  publishStatus(LocalizationStatus::STATE_DEGRADED, 1);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 1s));

  // 定位失效期间即使收到解除急停和新时间戳轨迹，也必须持续双零并丢弃轨迹。
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStop(false);
        publishPath();
        return command_norm_.load() < 1e-6;
      },
      300ms));

  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  publishStop(false);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        return command_norm_.load() < 1e-6;
      },
      300ms));

  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishStop(false);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));
}

TEST_F(MpcLocalizationGateTest, QpShadowRetainsTheSingleIlqrCommandPublisher) {
  ASSERT_EQ(command_sub_->get_publisher_count(), 1u);
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));
  EXPECT_EQ(command_sub_->get_publisher_count(), 1u);
}

TEST_F(MpcLocalizationGateTest, QpShadowTelemetryDumpIsReadOnlyAndDoesNotAddPublisher) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  ASSERT_TRUE(spinUntil(
    [this]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, 1);
      publishExecution(1);
      return command_norm_.load() > 0.02;
    },
    2s));

  const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = telemetry_client_->async_send_request(request);
  ASSERT_TRUE(spinUntil(
    [&result]() {
      return result.wait_for(0s) == std::future_status::ready;
    },
    1s));
  const auto response = result.get();
  ASSERT_TRUE(response->success);
  EXPECT_NE(response->message.find("\"schema_version\":2"), std::string::npos);
  EXPECT_NE(response->message.find("\"solver_mode\":\"qp_shadow\""),
            std::string::npos);
  EXPECT_EQ(command_sub_->get_publisher_count(), 1u);
}

TEST_F(MpcLocalizationGateTest, RejectsOldSequenceAndOldEpochExecutionCommands) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));

  const std::uint64_t old_sequence = execution_sequence_;
  publishExecutionStop();
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 1s));

  // The prior execute sample is older than the committed stop and cannot revive
  // the tracker even though its path timestamp is fresh.
  publishExecutionWithSequence(old_sequence, 1, 1);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        return command_norm_.load() < 1e-6;
      },
      300ms));

  publishStatus(LocalizationStatus::STATE_TRACKING, 2);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 2);
        return command_norm_.load() < 1e-6;
      },
      500ms));
  publishExecutionWithSequence(++execution_sequence_, 1, 2);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        return command_norm_.load() < 1e-6;
      },
      500ms));
}

TEST_F(MpcLocalizationGateTest, RequiresStopHandshakeForNewManagerIncarnation) {
  constexpr std::uint64_t kOldIncarnation = 100;
  constexpr std::uint64_t kNewIncarnation = 101;
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);

  publishExecutionStop(kOldIncarnation, 1);
  ASSERT_TRUE(spinUntil(
    [this]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, 1);
      publishExecutionWithSequence(2, 1, 1, 0, kOldIncarnation);
      return command_norm_.load() > 0.02;
    },
    2s));

  // A new process cannot authorize motion until it establishes an explicit
  // STOP handoff to the consumer.
  publishExecutionWithSequence(1, 1, 1, 0, kNewIncarnation);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 1s));

  publishExecutionStop(kNewIncarnation, 1);
  ASSERT_TRUE(spinUntil(
    [this]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, 1);
      publishExecutionWithSequence(2, 1, 1, 0, kNewIncarnation);
      return command_norm_.load() > 0.02;
    },
    2s));

  publishExecutionStop(kNewIncarnation, 3);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 1s));

  // A delayed command from the old manager cannot revive the stopped tracker.
  publishExecutionWithSequence(99, 1, 1, 0, kOldIncarnation);
  ASSERT_TRUE(spinUntil(
    [this]() {
      publishOdometry();
      return command_norm_.load() < 1e-6;
    },
    500ms));
}

TEST_F(MpcLocalizationGateTest, StopsWhenGimbalFeedbackLeaseExpires) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));

  ASSERT_TRUE(spinUntil(
    [this]() {return command_norm_.load() < 1e-6;}, 500ms));
}

TEST_F(MpcLocalizationGateTest, RejectsExecutionForDifferentGimbalRequest) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  publishExecutionWithSequence(
    ++execution_sequence_, 1, 1, gimbal_request_sequence_ + 1);
  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        return command_norm_.load() < 1e-6;
      },
      500ms));

  // Goal Manager continues publishing a structured STOP after it rejects an
  // authorization. Mirror that recovery boundary before a new execute lease.
  publishExecutionStop();
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 500ms));

  ASSERT_TRUE(
    spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return command_norm_.load() > 0.02;
      },
      2s));
}

} // namespace
