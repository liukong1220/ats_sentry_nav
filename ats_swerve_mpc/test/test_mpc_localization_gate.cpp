// Copyright 2026

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <limits>
#include <stdexcept>
#include <thread>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
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
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;

class MpcLocalizationGateTest : public ::testing::Test
{
protected:
  virtual bool initializePlannerAuthority() const {return true;}
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
        rclcpp::Parameter("planner_status_topic", "/test_mpc_gate/planner_status"),
        rclcpp::Parameter("map_ready_topic", "/test_mpc_gate/map_ready"),
        rclcpp::Parameter("map_ready_timeout_sec", 0.2),
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
    planner_status_pub_ = driver_->create_publisher<PlannerStatus>(
      "/test_mpc_gate/planner_status", rclcpp::QoS(1).reliable().transient_local());
    map_ready_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_mpc_gate/map_ready", rclcpp::QoS(1).reliable().transient_local());
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
          planner_status_pub_->get_subscription_count() == 1 &&
          map_ready_pub_->get_subscription_count() == 1 &&
          telemetry_client_->service_is_ready();
        },
        2s));
    // Production Goal Manager publishes a transient-local STOP at process
    // start. Establish the same handoff before this fixture sends EXECUTE.
    publishExecutionStop();
    publishMapReady(true);
    if (initializePlannerAuthority()) {
      publishPlannerStatus(1, 1);
    }
    executor_.spin_some();
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(mpc_);
    command_sub_.reset();
    telemetry_client_.reset();
    status_pub_.reset();
    gimbal_status_pub_.reset();
    planner_status_pub_.reset();
    map_ready_pub_.reset();
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
    if (renew_map_ready_) {
      publishMapReady(true);
    }
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

  void publishMapReady(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    map_ready_pub_->publish(message);
  }

  void publishPlannerStatus(
    std::uint64_t epoch, std::uint64_t generation,
    std::uint8_t state = PlannerStatus::STATE_REFERENCE_READY,
    std::uint8_t failure = PlannerStatus::FAILURE_NONE,
    std::uint64_t publication = 1, double stamp_offset_s = 0.0)
  {
    PlannerStatus message;
    message.header.stamp = driver_->now() +
      rclcpp::Duration::from_seconds(stamp_offset_s);
    message.header.frame_id = "map";
    message.goal_id = 1;
    message.plan_request_sequence = 1;
    message.localization_epoch = epoch;
    message.map_generation = generation;
    message.map_publication_sequence = publication;
    message.state = state;
    message.failure_reason = failure;
    planner_status_pub_->publish(message);
  }

  bool moveOnGeneration(std::uint64_t generation, std::uint64_t epoch = 1)
  {
    return spinUntil([this, generation, epoch]() {
      if (command_norm_.load() > 0.02) {
        return true;
      }
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, epoch);
      publishExecutionWithSequence(++execution_sequence_, epoch, generation);
      return false;
    }, 2s);
  }

  void expectRemainsStopped(std::uint64_t epoch = 1)
  {
    EXPECT_FALSE(spinUntil([this, epoch]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, epoch);
      return command_norm_.load() > 1e-6;
    }, 100ms));
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
    std::uint64_t manager_incarnation = 0,
    double command_stamp_offset_s = 0.0)
  {
    ExecutionCommand command;
    command.header.stamp = driver_->now() +
      rclcpp::Duration::from_seconds(command_stamp_offset_s);
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

  void expectRejectedExecutionStopsActiveTracker(bool old_incarnation, bool older_sequence)
  {
    manager_incarnation_ = 2;
    publishExecutionStop();
    ASSERT_TRUE(spinUntil(
      [this]() {
        if (command_norm_.load() > 0.02) {
          return true;
        }
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return false;
      },
      2s));

    const auto accepted_sequence = execution_sequence_;
    const auto rejected_sequence = old_incarnation ? accepted_sequence + 1000 :
      accepted_sequence - (older_sequence ? 1 : 0);
    publishExecutionWithSequence(
      rejected_sequence, 1, 1, 0, old_incarnation ? 1 : manager_incarnation_);
    // Keep every other gate healthy. Stop must precede the 500 ms execution
    // lease expiry, not merely wait for the old authorization to time out.
    ASSERT_TRUE(spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        return command_norm_.load() < 1e-6;
      },
      200ms));

    // Healthy feedback, legacy false and a fresh legacy path cannot revive
    // the cleared tracker. A replay with a freshly stamped path cannot either.
    publishExecutionWithSequence(accepted_sequence, 1, 1);
    EXPECT_FALSE(spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishStop(false);
        publishPath();
        EXPECT_LT(command_norm_.load(), 1e-6);
        return false;
      },
      100ms));

    // A fresh command at the next sequence restores motion; in particular an
    // old incarnation's large sequence must not advance the replay watermark.
    ASSERT_TRUE(spinUntil(
      [this]() {
        if (command_norm_.load() > 0.02) {
          return true;
        }
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishExecution(1);
        return false;
      },
      2s));
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
  rclcpp::Publisher<PlannerStatus>::SharedPtr planner_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_ready_pub_;
  bool renew_map_ready_{true};
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

TEST_F(MpcLocalizationGateTest, RejectsFutureAndStaleExecutionCommandTimestamps) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);

  // A command with a producer clock ahead of the consumer must not install a
  // reference, even though its manager identity and sequence are otherwise
  // valid.
  publishExecutionWithSequence(
    ++execution_sequence_, 1, 1, gimbal_request_sequence_, manager_incarnation_, 1.0);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 500ms));

  // Establish a fresh STOP handoff before testing the stale side of the same
  // contract.  The old timestamp must not be allowed to become a new lease.
  publishExecutionStop(manager_incarnation_, ++execution_sequence_);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 500ms));
  publishExecutionWithSequence(
    ++execution_sequence_, 1, 1, gimbal_request_sequence_, manager_incarnation_, -1.0);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 500ms));
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
  EXPECT_NE(response->message.find("\"schema_version\":4"), std::string::npos);
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

TEST_F(MpcLocalizationGateTest, OldIncarnationStopsActiveTrackerUntilFreshAuthorization) {
  expectRejectedExecutionStopsActiveTracker(true, false);
}

TEST_F(MpcLocalizationGateTest, DuplicateSequenceStopsActiveTrackerUntilFreshAuthorization) {
  expectRejectedExecutionStopsActiveTracker(false, false);
}

TEST_F(MpcLocalizationGateTest, OlderSequenceStopsActiveTrackerUntilFreshAuthorization) {
  expectRejectedExecutionStopsActiveTracker(false, true);
}

TEST_F(MpcLocalizationGateTest, RequiresStopHandshakeForNewManagerIncarnation) {
  constexpr std::uint64_t kOldIncarnation = 100;
  constexpr std::uint64_t kNewIncarnation = 101;
  std::uint64_t old_sequence = 1;
  std::uint64_t new_sequence = 1;
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);

  publishExecutionStop(kOldIncarnation, 1);
  ASSERT_TRUE(spinUntil(
    [this, &old_sequence]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, 1);
      publishExecutionWithSequence(++old_sequence, 1, 1, 0, kOldIncarnation);
      return command_norm_.load() > 0.02;
    },
    2s));

  // A new process cannot authorize motion until it establishes an explicit
  // STOP handoff to the consumer.
  publishExecutionWithSequence(1, 1, 1, 0, kNewIncarnation);
  ASSERT_TRUE(spinUntil([this]() {return command_norm_.load() < 1e-6;}, 1s));

  publishExecutionStop(kNewIncarnation, 1);
  ASSERT_TRUE(spinUntil(
    [this, &new_sequence]() {
      publishOdometry();
      publishStatus(LocalizationStatus::STATE_TRACKING, 1);
      publishExecutionWithSequence(++new_sequence, 1, 1, 0, kNewIncarnation);
      return command_norm_.load() > 0.02;
    },
    2s));

  publishExecutionStop(kNewIncarnation, ++new_sequence);
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

TEST_F(MpcLocalizationGateTest, RejectsZeroOldAndFutureLocalGenerations) {
  publishPlannerStatus(1, 7);
  for (const std::uint64_t rejected_generation : {0u, 6u, 8u}) {
    ASSERT_TRUE(moveOnGeneration(7));
    publishExecutionWithSequence(++execution_sequence_, 1, rejected_generation);
    ASSERT_TRUE(spinUntil([this]() {
      publishOdometry();
      return command_norm_.load() < 1e-6;
    }, 200ms));
    expectRemainsStopped();
  }
  ASSERT_TRUE(moveOnGeneration(7));
}

TEST_F(MpcLocalizationGateTest, NewGenerationRevokesAndLateOldStatusCannotRestore) {
  publishPlannerStatus(1, 7);
  ASSERT_TRUE(moveOnGeneration(7));
  publishPlannerStatus(1, 8, PlannerStatus::STATE_FAILED,
    PlannerStatus::FAILURE_SNAPSHOT_CHANGED);
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 200ms));
  publishPlannerStatus(1, 7);  // New timestamp must not lower the generation floor.
  publishExecutionWithSequence(++execution_sequence_, 1, 7);
  expectRemainsStopped();
  publishPlannerStatus(1, 8);
  expectRemainsStopped();  // A READY is map evidence, never execution permission.
  ASSERT_TRUE(moveOnGeneration(8));
}

TEST_F(MpcLocalizationGateTest, SameGenerationPublicationAndAcceptedStatusDoNotChurn) {
  publishPlannerStatus(1, 7);
  ASSERT_TRUE(moveOnGeneration(7));
  publishPlannerStatus(1, 7, PlannerStatus::STATE_REFERENCE_READY,
    PlannerStatus::FAILURE_NONE, 999);
  EXPECT_FALSE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 100ms));
  publishPlannerStatus(1, 0, PlannerStatus::STATE_ACCEPTED);
  EXPECT_FALSE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 100ms));
}

TEST_F(MpcLocalizationGateTest, FailedStatusRejectsDelayedReadyAndNeedsFreshAuthorization) {
  ASSERT_TRUE(moveOnGeneration(1));
  publishPlannerStatus(1, 1, PlannerStatus::STATE_FAILED,
    PlannerStatus::FAILURE_RUNTIME_UNSAFE);
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 200ms));
  publishPlannerStatus(1, 1, PlannerStatus::STATE_REFERENCE_READY,
    PlannerStatus::FAILURE_NONE, 1, -1.0);
  publishExecutionWithSequence(++execution_sequence_, 1, 1);
  expectRemainsStopped();
  publishPlannerStatus(1, 1);
  expectRemainsStopped();
  ASSERT_TRUE(moveOnGeneration(1));
}

TEST_F(MpcLocalizationGateTest, MapUnreadyRetiresGenerationAcrossReadyRecovery) {
  ASSERT_TRUE(moveOnGeneration(1));
  publishPlannerStatus(1, 1, PlannerStatus::STATE_FAILED,
    PlannerStatus::FAILURE_MAP_UNREADY);
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 200ms));
  publishPlannerStatus(1, 1);
  publishExecutionWithSequence(++execution_sequence_, 1, 1);
  expectRemainsStopped();
  publishPlannerStatus(1, 2);
  expectRemainsStopped();
  ASSERT_TRUE(moveOnGeneration(2));
}

TEST_F(MpcLocalizationGateTest, ReadyFalseStopsAndStatusBeforeReadyDoesNotResume) {
  ASSERT_TRUE(moveOnGeneration(1));
  renew_map_ready_ = false;
  publishMapReady(false);
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 200ms));
  publishPlannerStatus(1, 2);
  expectRemainsStopped();
  publishMapReady(true);
  renew_map_ready_ = true;
  expectRemainsStopped();
  ASSERT_TRUE(moveOnGeneration(2));
}

TEST_F(MpcLocalizationGateTest, ReadyLeaseExpiresBeforeCommandLeaseAndRetiresOldMap) {
  ASSERT_TRUE(moveOnGeneration(1));
  renew_map_ready_ = false;
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 350ms));
  renew_map_ready_ = true;
  publishMapReady(true);
  publishExecutionWithSequence(++execution_sequence_, 1, 1);
  expectRemainsStopped();
  publishPlannerStatus(1, 2);
  ASSERT_TRUE(moveOnGeneration(2));
}

TEST_F(MpcLocalizationGateTest, EpochChangeRevokesAndOldEpochStatusCannotRestore) {
  publishPlannerStatus(1, 7);
  ASSERT_TRUE(moveOnGeneration(7));
  publishStatus(LocalizationStatus::STATE_TRACKING, 2);
  publishPlannerStatus(2, 1);
  ASSERT_TRUE(spinUntil([this]() {
    publishOdometry();
    return command_norm_.load() < 1e-6;
  }, 200ms));
  publishPlannerStatus(1, 999);
  publishExecutionWithSequence(++execution_sequence_, 2, 7);
  expectRemainsStopped(2);
  ASSERT_TRUE(moveOnGeneration(1, 2));
}

class MpcUnknownMapGateTest : public MpcLocalizationGateTest
{
protected:
  bool initializePlannerAuthority() const override {return false;}
};

TEST_F(MpcUnknownMapGateTest, StatusAfterCommandRequiresNewSequenceNotReplay) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  const auto rejected_sequence = ++execution_sequence_;
  publishExecutionWithSequence(rejected_sequence, 1, 7);
  expectRemainsStopped();
  publishPlannerStatus(1, 7);
  expectRemainsStopped();
  publishExecutionWithSequence(rejected_sequence, 1, 7);
  expectRemainsStopped();
  ASSERT_TRUE(moveOnGeneration(7));
}

TEST_F(MpcUnknownMapGateTest, MalformedStatusHeaderCannotEstablishAuthority) {
  for (int defect = 0; defect < 4; ++defect) {
    PlannerStatus status;
    status.header.stamp = driver_->now();
    status.header.frame_id = "map";
    status.localization_epoch = 1;
    status.map_generation = 7;
    status.state = PlannerStatus::STATE_REFERENCE_READY;
    if (defect == 0) {
      status.header.frame_id.clear();
    } else if (defect == 1) {
      status.header.stamp.sec = -1;
    } else if (defect == 2) {
      status.header.stamp.nanosec = 1000000000u;
    } else {
      status.header.stamp.sec = 0;
      status.header.stamp.nanosec = 0;
    }
    planner_status_pub_->publish(status);
    publishExecutionWithSequence(++execution_sequence_, 1, 7);
    expectRemainsStopped();
  }
  publishPlannerStatus(1, 7);
  ASSERT_TRUE(moveOnGeneration(7));
}

TEST(AtsSwerveMpcNodeConstruction, RejectsOversizedHorizonBeforeControllerCreation) {
  const bool initialized_before = rclcpp::ok();
  if (!initialized_before) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  const auto expectRejected = [](int horizon) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("horizon", horizon),
        rclcpp::Parameter("solver_mode", "ilqr")});
    EXPECT_THROW(
        {
          auto node = std::make_shared<ats_swerve_mpc::AtsSwerveMpcNode>(options);
          (void)node;
        },
        std::invalid_argument);
  };
  expectRejected(ats_swerve_mpc::kLtvQpMaximumHorizon + 1);
  expectRejected(std::numeric_limits<int>::max());
  if (!initialized_before) {
    rclcpp::shutdown();
  }
}

} // namespace
