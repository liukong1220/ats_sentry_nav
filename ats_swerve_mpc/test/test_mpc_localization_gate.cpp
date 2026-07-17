// Copyright 2026

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace {

using namespace std::chrono_literals;
using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;

class MpcLocalizationGateTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("odom_topic", "/test_mpc_gate/odom"),
        rclcpp::Parameter("trajectory_topic", "/test_mpc_gate/path"),
        rclcpp::Parameter("command_topic", "/test_mpc_gate/cmd"),
        rclcpp::Parameter("emergency_stop_topic", "/test_mpc_gate/stop"),
        rclcpp::Parameter("localization_status_topic", "/test_mpc_gate/status"),
        rclcpp::Parameter("require_localization_status", true),
        rclcpp::Parameter("control_rate_hz", 50.0),
        rclcpp::Parameter("emergency_stop_timeout", 5.0),
        rclcpp::Parameter("trajectory_timeout", 2.0),
        rclcpp::Parameter("publish_debug_paths", false),
    });
    mpc_ = std::make_shared<ats_swerve_mpc::AtsSwerveMpcNode>(options);
    driver_ =
        std::make_shared<rclcpp::Node>("test_mpc_localization_gate_driver");
    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
        "/test_mpc_gate/odom", rclcpp::SensorDataQoS());
    path_pub_ = driver_->create_publisher<nav_msgs::msg::Path>(
        "/test_mpc_gate/path", rclcpp::QoS(1).reliable());
    stop_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
        "/test_mpc_gate/stop", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = driver_->create_publisher<LocalizationStatus>(
        "/test_mpc_gate/status", rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = driver_->create_subscription<geometry_msgs::msg::Twist>(
        "/test_mpc_gate/cmd", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          command_norm_.store(std::hypot(message->linear.x, message->linear.y) +
                              std::abs(message->angular.z));
        });
    executor_.add_node(mpc_);
    executor_.add_node(driver_);
    ASSERT_TRUE(spinUntil(
        [this]() {
          return odom_pub_->get_subscription_count() == 1 &&
                 path_pub_->get_subscription_count() == 1 &&
                 stop_pub_->get_subscription_count() == 1 &&
                 status_pub_->get_subscription_count() == 1;
        },
        2s));
  }

  void TearDown() override {
    executor_.remove_node(driver_);
    executor_.remove_node(mpc_);
    command_sub_.reset();
    status_pub_.reset();
    stop_pub_.reset();
    path_pub_.reset();
    odom_pub_.reset();
    driver_.reset();
    mpc_.reset();
  }

  bool spinUntil(const std::function<bool()> &predicate,
                 std::chrono::steady_clock::duration timeout) {
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

  void publishOdometry() {
    nav_msgs::msg::Odometry message;
    message.header.stamp = driver_->now();
    message.header.frame_id = "odom";
    message.child_frame_id = "gimbal_yaw_odom";
    message.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(message);
  }

  void publishStatus(std::uint8_t state, std::uint64_t epoch) {
    LocalizationStatus message;
    message.header.stamp = driver_->now();
    message.header.frame_id = "map";
    message.state = state;
    message.epoch = epoch;
    status_pub_->publish(message);
  }

  void publishStop(bool stop) {
    std_msgs::msg::Bool message;
    message.data = stop;
    stop_pub_->publish(message);
  }

  void publishPath() {
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

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ats_swerve_mpc::AtsSwerveMpcNode> mpc_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  std::atomic<double> command_norm_{0.0};
};

TEST_F(MpcLocalizationGateTest,
       RejectsReferenceReceivedWhileLocalizationIsUnhealthy) {
  publishOdometry();
  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  publishStop(false);
  ASSERT_TRUE(spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishStop(false);
        publishPath();
        return command_norm_.load() > 0.02;
      },
      2s));

  publishStatus(LocalizationStatus::STATE_DEGRADED, 1);
  ASSERT_TRUE(spinUntil([this]() { return command_norm_.load() < 1e-6; }, 1s));

  // 定位失效期间即使收到解除急停和新时间戳轨迹，也必须持续双零并丢弃轨迹。
  ASSERT_TRUE(spinUntil(
      [this]() {
        publishOdometry();
        publishStop(false);
        publishPath();
        return command_norm_.load() < 1e-6;
      },
      300ms));

  publishStatus(LocalizationStatus::STATE_TRACKING, 1);
  publishStop(false);
  ASSERT_TRUE(spinUntil(
      [this]() {
        publishOdometry();
        return command_norm_.load() < 1e-6;
      },
      300ms));

  ASSERT_TRUE(spinUntil(
      [this]() {
        publishOdometry();
        publishStatus(LocalizationStatus::STATE_TRACKING, 1);
        publishStop(false);
        publishPath();
        return command_norm_.load() > 0.02;
      },
      2s));
}

} // namespace
