// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__REFERENCE_PATH_TIMING_HPP_
#define MINCO_PLANNER__TRAJECTORY__REFERENCE_PATH_TIMING_HPP_

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace minco_planner
{

inline void rebasePathTimestamps(nav_msgs::msg::Path & path, const rclcpp::Time & new_start)
{
  const auto clock_type = new_start.get_clock_type();
  const rclcpp::Time old_start(path.header.stamp, clock_type);
  path.header.stamp = new_start;
  for (auto & pose : path.poses) {
    const rclcpp::Duration offset = rclcpp::Time(pose.header.stamp, clock_type) - old_start;
    pose.header.stamp = new_start + offset;
  }
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__REFERENCE_PATH_TIMING_HPP_
