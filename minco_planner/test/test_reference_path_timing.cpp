// Copyright 2026

#include "gtest/gtest.h"
#include "minco_planner/trajectory/reference_path_timing.hpp"

namespace
{

TEST(ReferencePathTiming, RebasesEveryStampAndPreservesRelativeTiming)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  for (const auto nanoseconds : {0, 250000000, 1000000000}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = rclcpp::Time(10000000000LL + nanoseconds, RCL_ROS_TIME);
    path.poses.push_back(pose);
  }

  minco_planner::rebasePathTimestamps(path, rclcpp::Time(42, 0, RCL_ROS_TIME));

  EXPECT_EQ(rclcpp::Time(path.header.stamp).nanoseconds(), 42000000000LL);
  EXPECT_EQ(rclcpp::Time(path.poses[0].header.stamp).nanoseconds(), 42000000000LL);
  EXPECT_EQ(rclcpp::Time(path.poses[1].header.stamp).nanoseconds(), 42250000000LL);
  EXPECT_EQ(rclcpp::Time(path.poses[2].header.stamp).nanoseconds(), 43000000000LL);
  EXPECT_EQ(path.header.frame_id, "odom");
  EXPECT_EQ(path.poses[1].header.frame_id, "odom");
}

}  // namespace
