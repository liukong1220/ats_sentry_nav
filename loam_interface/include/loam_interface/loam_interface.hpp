 

#ifndef LOAM_INTERFACE__LOAM_INTERFACE_HPP_
#define LOAM_INTERFACE__LOAM_INTERFACE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include "ats_navigation_interfaces/msg/identified_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace loam_interface
{

class LoamInterfaceNode : public rclcpp::Node
{
public:
  explicit LoamInterfaceNode(const rclcpp::NodeOptions & options);

private:
  struct OdomSample
  {
    rclcpp::Time stamp;
    tf2::Transform pose;
  };

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void identifiedOdometryCallback(
    const ats_navigation_interfaces::msg::IdentifiedOdometry::ConstSharedPtr msg);

  bool getClosestOdomSample(const rclcpp::Time & stamp, OdomSample & sample);

  nav_msgs::msg::Odometry buildOdometryMessage(
    const OdomSample & sample, const rclcpp::Time & stamp) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr
    identified_odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr
    identified_odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string state_estimation_topic_;
  bool use_identified_odometry_{false};
  std::uint64_t instance_id_{0};
  std::unordered_set<std::uint64_t> retired_instances_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string lidar_frame_;
  double base_frame_height_{0.0};
  std::string base_frame_;

  bool base_frame_to_lidar_initialized_;
  tf2::Transform tf_odom_to_state_world_;
  tf2::Transform tf_state_to_lidar_;
  std::deque<OdomSample> odom_buffer_;
  std::mutex odom_buffer_mutex_;
  static constexpr std::size_t kMaxOdomBufferSize = 4096;
};

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__LOAM_INTERFACE_HPP_
