 

#include "loam_interface/loam_interface.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<bool>("use_identified_odometry", false);
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");
  // Height of base_frame above the ground plane. The odom origin is anchored at
  // the ground point below base_frame, so odom z=0 is the floor rather than the
  // gimbal / lidar mount height consumed by ROGMap projection and the planners.
  this->declare_parameter<double>("base_frame_height", 0.0);
  // Point-LIO publishes the IMU state. Legacy lidar-pose producers keep the
  // identity defaults. These are T_state<-lidar, matching mapping.extrinsic_T/R.
  const auto state_to_lidar = this->declare_parameter<std::vector<double>>(
    "state_to_lidar_translation", {0.0, 0.0, 0.0});
  const auto state_to_lidar_rpy = this->declare_parameter<std::vector<double>>(
    "state_to_lidar_rpy", {0.0, 0.0, 0.0});
  const auto finite_three = [](const std::vector<double> & values) {
      return values.size() == 3 && std::all_of(values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
    };
  if (!finite_three(state_to_lidar) || !finite_three(state_to_lidar_rpy)) {
    throw std::invalid_argument("state-to-lidar extrinsics must contain three finite values");
  }
  tf2::Quaternion state_rotation;
  state_rotation.setRPY(state_to_lidar_rpy[0], state_to_lidar_rpy[1], state_to_lidar_rpy[2]);
  tf_state_to_lidar_ = tf2::Transform(state_rotation, tf2::Vector3(
    state_to_lidar[0], state_to_lidar[1], state_to_lidar[2]));

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("use_identified_odometry", use_identified_odometry_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame_height", base_frame_height_);

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);
  identified_odom_pub_ = this->create_publisher<
    ats_navigation_interfaces::msg::IdentifiedOdometry>("lidar_odometry_identified", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  if (use_identified_odometry_) {
    identified_odom_sub_ = this->create_subscription<
      ats_navigation_interfaces::msg::IdentifiedOdometry>(
      state_estimation_topic_ + "_identified", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&LoamInterfaceNode::identifiedOdometryCallback, this, std::placeholders::_1));
  } else {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_estimation_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
  }
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  OdomSample matched_odom;
  const auto cloud_stamp = rclcpp::Time(msg->header.stamp);
  if (!getClosestOdomSample(cloud_stamp, matched_odom)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping registered_scan publish because no lidar odometry sample is available yet.");
    return;
  }

  // NOTE: Input point cloud message is based on the `lidar_odom`
  // Here we transform it to the REAL `odom` frame
  // 以 unique_ptr 发布：开 intra-process 时 publish(const T &) 会整帧深拷贝一次。
  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_state_world_, *msg, *out);
  pcd_pub_->publish(std::move(out));
  if (!use_identified_odometry_) {
    odom_pub_->publish(
      std::make_unique<nav_msgs::msg::Odometry>(buildOdometryMessage(matched_odom, cloud_stamp)));
  }
}

void LoamInterfaceNode::identifiedOdometryCallback(
  const ats_navigation_interfaces::msg::IdentifiedOdometry::ConstSharedPtr msg)
{
  if (msg->instance_id == 0 || msg->instance_id < instance_id_ ||
    msg->header != msg->odometry.header ||
    retired_instances_.count(msg->instance_id) != 0U)
  {
    return;
  }
  if (instance_id_ != msg->instance_id) {
    if (instance_id_ != 0) {
      retired_instances_.insert(instance_id_);
    }
    instance_id_ = msg->instance_id;
    // A restarted Point-LIO has a new state world and a new first state.
    base_frame_to_lidar_initialized_ = false;
    std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
    odom_buffer_.clear();
  }
  odometryCallback(std::make_shared<nav_msgs::msg::Odometry>(msg->odometry));
  const rclcpp::Time stamp(msg->header.stamp);
  OdomSample sample;
  if (!getClosestOdomSample(stamp, sample) || sample.stamp != stamp) {
    return;
  }
  ats_navigation_interfaces::msg::IdentifiedOdometry identified;
  identified.instance_id = msg->instance_id;
  identified.odometry = buildOdometryMessage(sample, stamp);
  identified.header = identified.odometry.header;
  identified_odom_pub_->publish(identified);
  odom_pub_->publish(identified.odometry);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the `lidar_odom`
  // Here we transform it to the `odom` frame
  if (!base_frame_to_lidar_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);
      const tf2::Transform tf_odom_to_base_frame(
        tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.0, base_frame_height_));
      const tf2::Transform tf_odom_to_world =
        tf_odom_to_base_frame * tf_base_frame_to_lidar * tf_state_to_lidar_.inverse();
      // The first published state need not be the state world origin: Point-LIO's
      // first update can already move it. Anchor odom at the ground below the base
      // of this state; only position and yaw are removed, roll/pitch stay gravity's.
      tf2::Transform tf_world_to_first_state;
      tf2::fromMsg(msg->pose.pose, tf_world_to_first_state);
      const auto first_base = tf_odom_to_world * tf_world_to_first_state * tf_state_to_lidar_ *
        tf_base_frame_to_lidar.inverse();
      double roll, pitch, yaw;
      tf2::Matrix3x3(first_base.getRotation()).getRPY(roll, pitch, yaw);
      tf2::Quaternion unyaw;
      unyaw.setRPY(0.0, 0.0, -yaw);
      const tf2::Transform anchor(unyaw, tf2::quatRotate(unyaw, -first_base.getOrigin()));
      tf_odom_to_state_world_ = tf_odom_to_base_frame * anchor * tf_odom_to_world;
      base_frame_to_lidar_initialized_ = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
  }

  // Transform the odometry_msg (based lidar_odom) to the odom frame
  tf2::Transform tf_world_to_state;
  tf2::fromMsg(msg->pose.pose, tf_world_to_state);
  tf2::Transform tf_odom_to_lidar =
    tf_odom_to_state_world_ * tf_world_to_state * tf_state_to_lidar_;

  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  odom_buffer_.push_back(OdomSample{rclcpp::Time(msg->header.stamp), tf_odom_to_lidar});
  if (odom_buffer_.size() > kMaxOdomBufferSize) {
    odom_buffer_.pop_front();
  }
}

bool LoamInterfaceNode::getClosestOdomSample(const rclcpp::Time & stamp, OdomSample & sample)
{
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  if (odom_buffer_.empty()) {
    return false;
  }

  const auto newer_it = std::lower_bound(
    odom_buffer_.begin(), odom_buffer_.end(), stamp,
    [](const OdomSample & entry, const rclcpp::Time & target_stamp) {
      return entry.stamp < target_stamp;
    });

  if (newer_it == odom_buffer_.begin()) {
    sample = *newer_it;
    return true;
  }

  if (newer_it == odom_buffer_.end()) {
    sample = odom_buffer_.back();
    return true;
  }

  const auto older_it = std::prev(newer_it);
  const auto older_dt = (stamp - older_it->stamp).nanoseconds();
  const auto newer_dt = (newer_it->stamp - stamp).nanoseconds();
  sample = (older_dt <= newer_dt) ? *older_it : *newer_it;
  return true;
}

nav_msgs::msg::Odometry LoamInterfaceNode::buildOdometryMessage(
  const OdomSample & sample, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = sample.pose.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(sample.pose.getRotation());

  return out;
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
