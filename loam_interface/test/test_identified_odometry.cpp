// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "ats_navigation_interfaces/msg/identified_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "loam_interface/loam_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;  // NOLINT
using Identified = ats_navigation_interfaces::msg::IdentifiedOdometry;

TEST(LoamIdentifiedOdometry, TransformsAndRejectsRetiredInstance)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("state_estimation_topic", "test_state"),
    rclcpp::Parameter("registered_scan_topic", "test_scan"),
    rclcpp::Parameter("base_frame", "test_base"),
    rclcpp::Parameter("lidar_frame", "test_lidar"),
    rclcpp::Parameter("use_identified_odometry", true)});
  auto node = std::make_shared<loam_interface::LoamInterfaceNode>(options);
  auto driver = std::make_shared<rclcpp::Node>("loam_identified_driver");
  auto publisher = driver->create_publisher<Identified>("test_state_identified", 10);
  std::vector<Identified> received;
  auto subscriber = driver->create_subscription<Identified>(
    "lidar_odometry_identified", 10,
    [&received](Identified::ConstSharedPtr message) {received.push_back(*message);});
  tf2_ros::StaticTransformBroadcaster broadcaster(driver);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = driver->now();
  transform.header.frame_id = "test_base";
  transform.child_frame_id = "test_lidar";
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(driver);
  auto send = [&](std::uint64_t id) {
      Identified message;
      message.header.stamp = driver->now();
      message.header.frame_id = "camera_init";
      message.instance_id = id;
      message.odometry.header = message.header;
      message.odometry.pose.pose.orientation.w = 1.0;
      publisher->publish(message);
      for (int i = 0; i < 15; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(20ms);
      }
    };
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  send(101);
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back().instance_id, 101U);
  EXPECT_EQ(received.back().odometry.header.frame_id, "odom");
  send(202);
  ASSERT_EQ(received.back().instance_id, 202U);
  const auto count = received.size();
  send(101);
  // An unseen older ID is not a new process just because it was not cached.
  // Both old IDs must leave the transformed output untouched.
  send(50);
  send(0);  // zero never becomes an instance
  EXPECT_EQ(received.size(), count);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
