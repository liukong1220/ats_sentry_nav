// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ats_navigation_interfaces/msg/relocalization_observation.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void publishObservation(
    bool accepted, std::uint8_t status, const std::string & message, std::size_t inliers,
    double error, std::size_t source_points, const Eigen::Isometry3d & map_to_robot_base,
    const std::array<double, 36> & covariance);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool shouldRunRegistration();
  double accumulatedCloudAgeSeconds() const;
  std::optional<Eigen::Isometry3d> getCurrentRobotBaseToOdom() const;
  std::optional<Eigen::Isometry3d> getOdomToRobotBase(const rclcpp::Time & stamp) const;
  bool confirmationConsistent(const Eigen::Isometry3d & candidate) const;
  double translationDeltaFromLastTrigger(
    const Eigen::Isometry3d & current_robot_base_to_odom) const;
  double yawDeltaFromLastTrigger(const Eigen::Isometry3d & current_robot_base_to_odom) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::RelocalizationObservation>::SharedPtr
    observation_pub_;

  int num_threads_;
  int num_neighbors_;
  int min_source_points_;
  int min_inliers_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  double max_registration_error_;
  bool log_registration_details_;
  bool publish_tf_;
  int confirmation_count_;
  double confirmation_translation_tolerance_;
  double confirmation_yaw_tolerance_;
  double registration_interval_s_;
  double max_accumulation_age_s_;
  double min_registration_translation_delta_;
  double min_registration_yaw_delta_;
  double initial_pose_force_registration_window_s_;
  double transform_future_offset_s_;
  double max_scan_stamp_lag_s_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  std::optional<rclcpp::Time> first_accumulated_scan_time_;
  std::optional<rclcpp::Time> initial_pose_override_time_;
  bool has_received_scan_{false};
  std::uint64_t observation_sequence_{0};
  int pending_confirmation_count_{0};
  std::optional<Eigen::Isometry3d> pending_confirmation_transform_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  std::optional<Eigen::Isometry3d> last_registration_robot_base_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
