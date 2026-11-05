// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include "ats_navigation_interfaces/msg/identified_odometry.hpp"
#include "loam_interface/loam_interface.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

TEST(PointLioFrameContract, ImuPoseAndWorldCloudKeepBaseFixedDuringRotation)
{
  using Identified = ats_navigation_interfaces::msg::IdentifiedOdometry;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("state_estimation_topic", "imu_contract_state"),
    rclcpp::Parameter("registered_scan_topic", "imu_contract_scan"),
    rclcpp::Parameter("base_frame", "imu_contract_base"),
    rclcpp::Parameter("lidar_frame", "imu_contract_lidar"),
    rclcpp::Parameter("base_frame_height", 0.18),
    rclcpp::Parameter("state_to_lidar_translation", std::vector<double>{-0.011, -0.02329, 0.04412}),
    rclcpp::Parameter("use_identified_odometry", true)});
  auto converter = std::make_shared<loam_interface::LoamInterfaceNode>(options);
  auto driver = std::make_shared<rclcpp::Node>("point_lio_frame_contract_driver");
  auto input = driver->create_publisher<Identified>("imu_contract_state_identified", 10);
  auto clouds = driver->create_publisher<sensor_msgs::msg::PointCloud2>("imu_contract_scan", 10);
  std::vector<Identified> received;
  std::vector<sensor_msgs::msg::PointCloud2> received_clouds;
  auto subscription = driver->create_subscription<Identified>("lidar_odometry_identified", 10,
    [&](Identified::ConstSharedPtr msg) {received.push_back(*msg);});
  auto cloud_subscription = driver->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      received_clouds.push_back(*msg);});
  tf2::Quaternion tilt; tilt.setRPY(0.0, M_PI / 6.0, 0.0);
  const tf2::Transform base_from_lidar(tilt, {0.2405, 0.0, 0.02});
  const tf2::Transform imu_from_lidar(tf2::Quaternion::getIdentity(), {-0.011, -0.02329, 0.04412});
  const tf2::Transform initial_base(tf2::Quaternion::getIdentity(), {0.0, 0.0, 0.18});
  // Point-LIO's state world is anchored at the initial IMU, not the initial LiDAR.
  const auto odom_from_world = initial_base * base_from_lidar * imu_from_lidar.inverse();
  tf2_ros::StaticTransformBroadcaster broadcaster(driver);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = driver->now(); tf.header.frame_id = "imu_contract_base";
  tf.child_frame_id = "imu_contract_lidar"; tf.transform = tf2::toMsg(base_from_lidar);
  broadcaster.sendTransform(tf);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(converter); executor.add_node(driver);
  const auto spin = [&]() {
      for (int i = 0; i < 15; ++i) {
        executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(15));
      }
    };
  spin();
  for (const double heading : {0.0, M_PI / 2.0, M_PI}) {
    tf2::Quaternion q; q.setRPY(0, 0, heading);
    const tf2::Transform actual_base(q, {0.0, 0.0, 0.18});
    const auto expected_lidar = actual_base * base_from_lidar;
    const auto world_from_imu = odom_from_world.inverse() * expected_lidar * imu_from_lidar.inverse();
    Identified msg; msg.header.stamp = driver->now(); msg.header.frame_id = "camera_init";
    msg.instance_id = 100; msg.odometry.header = msg.header;
    tf2::toMsg(world_from_imu, msg.odometry.pose.pose);
    input->publish(msg); spin();
    ASSERT_FALSE(received.empty());
    tf2::Transform observed; tf2::fromMsg(received.back().odometry.pose.pose, observed);
    const auto base_error = (observed * base_from_lidar.inverse()).getOrigin() - actual_base.getOrigin();
    EXPECT_NEAR(base_error.length(), 0.0, 1e-6) << "heading=" << heading;
    // A fixed physical point is reported by Point-LIO in the same IMU-origin world.
    const tf2::Vector3 fixed(2.0, -1.0, 0.4);
    const auto raw = odom_from_world.inverse() * fixed;
    sensor_msgs::msg::PointCloud2 cloud; cloud.header = msg.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = raw.x();
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = raw.y();
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = raw.z();
    clouds->publish(cloud); spin();
    ASSERT_FALSE(received_clouds.empty());
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(received_clouds.back(), "x"), fixed.x(), 1e-5);
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(received_clouds.back(), "y"), fixed.y(), 1e-5);
    EXPECT_NEAR(*sensor_msgs::PointCloud2ConstIterator<float>(received_clouds.back(), "z"), fixed.z(), 1e-5);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv); rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS(); rclcpp::shutdown(); return result;
}
