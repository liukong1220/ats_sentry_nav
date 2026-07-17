

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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

namespace
{

double normalizedRegistrationError(double error, std::size_t inliers)
{
  return inliers > 0 && std::isfinite(error) ? error / static_cast<double>(inliers)
                                             : std::numeric_limits<double>::infinity();
}

double yawDistance(const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  const double delta =
    lhs.rotation().eulerAngles(0, 1, 2).z() - rhs.rotation().eulerAngles(0, 1, 2).z();
  return std::abs(std::atan2(std::sin(delta), std::cos(delta)));
}

std::array<double, 36> registrationCovariance(
  const Eigen::Matrix<double, 6, 6> & information, double error, std::size_t inliers)
{
  Eigen::Matrix<double, 6, 6> covariance_rt = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    0.5 * (information + information.transpose()));
  if (
    solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
    solver.eigenvectors().allFinite()) {
    const double degrees_of_freedom = std::max(1.0, 3.0 * static_cast<double>(inliers) - 6.0);
    const double residual_scale = std::clamp(2.0 * error / degrees_of_freedom, 1e-6, 1e3);
    Eigen::Matrix<double, 6, 1> variances;
    for (int index = 0; index < 6; ++index) {
      variances(index) =
        std::clamp(residual_scale / std::max(1e-9, solver.eigenvalues()(index)), 1e-6, 1e3);
    }
    covariance_rt =
      solver.eigenvectors() * variances.asDiagonal() * solver.eigenvectors().transpose();
  } else {
    covariance_rt.diagonal() << 1.0, 1.0, 1.0, 4.0, 4.0, 4.0;
  }

  // small_gicp 的李代数顺序是 [rx, ry, rz, x, y, z]，ROS PoseWithCovariance
  // 使用 [x, y, z, rx, ry, rz]。
  constexpr std::array<int, 6> kPoseToRegistration{{3, 4, 5, 0, 1, 2}};
  std::array<double, 36> covariance{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      covariance[static_cast<std::size_t>(6 * row + column)] =
        covariance_rt(kPoseToRegistration[row], kPoseToRegistration[column]);
    }
  }
  return covariance;
}

}  // namespace

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("min_source_points", 500);
  this->declare_parameter("min_inliers", 200);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_registration_error", -1.0);
  this->declare_parameter("log_registration_details", true);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("confirmation_count", 2);
  this->declare_parameter("confirmation_translation_tolerance", 0.15);
  this->declare_parameter("confirmation_yaw_tolerance", 0.10);
  this->declare_parameter("registration_interval_s", 0.25);
  this->declare_parameter("max_accumulation_age_s", 0.30);
  this->declare_parameter("min_registration_translation_delta", 0.10);
  this->declare_parameter("min_registration_yaw_delta", 0.12);
  this->declare_parameter("initial_pose_force_registration_window_s", 2.0);
  this->declare_parameter("transform_future_offset_s", 0.25);
  this->declare_parameter("max_scan_stamp_lag_s", 0.25);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_inliers", min_inliers_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_registration_error", max_registration_error_);
  this->get_parameter("log_registration_details", log_registration_details_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("confirmation_count", confirmation_count_);
  this->get_parameter("confirmation_translation_tolerance", confirmation_translation_tolerance_);
  this->get_parameter("confirmation_yaw_tolerance", confirmation_yaw_tolerance_);
  confirmation_count_ = std::max(1, confirmation_count_);
  confirmation_translation_tolerance_ = std::max(0.0, confirmation_translation_tolerance_);
  confirmation_yaw_tolerance_ = std::max(0.0, confirmation_yaw_tolerance_);
  this->get_parameter("registration_interval_s", registration_interval_s_);
  this->get_parameter("max_accumulation_age_s", max_accumulation_age_s_);
  this->get_parameter("min_registration_translation_delta", min_registration_translation_delta_);
  this->get_parameter("min_registration_yaw_delta", min_registration_yaw_delta_);
  this->get_parameter(
    "initial_pose_force_registration_window_s", initial_pose_force_registration_window_s_);
  this->get_parameter("transform_future_offset_s", transform_future_offset_s_);
  this->get_parameter("max_scan_stamp_lag_s", max_scan_stamp_lag_s_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;
  last_registration_robot_base_to_odom_ = Eigen::Isometry3d::Identity();

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  observation_pub_ =
    this->create_publisher<ats_navigation_interfaces::msg::RelocalizationObservation>(
      "relocalization_observation", rclcpp::QoS(10).reliable());

  register_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(std::max(0.05, registration_interval_s_))),
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  if (base_frame_.empty() || lidar_frame_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Skip global map frame conversion because base_frame or lidar_frame is empty.");
    return;
  }

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF lookup failed while waiting for %s -> %s. Retrying with latest available TF: %s",
        lidar_frame_.c_str(), base_frame_.c_str(), ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;
  has_received_scan_ = true;
  if (!first_accumulated_scan_time_) {
    first_accumulated_scan_time_ = msg->header.stamp;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (!shouldRunRegistration()) {
    return;
  }

  if (accumulated_cloud_->empty()) {
    return;
  }

  if (static_cast<int>(accumulated_cloud_->size()) < min_source_points_) {
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  const double registration_error = normalizedRegistrationError(result.error, result.num_inliers);
  const bool inlier_ok = static_cast<int>(result.num_inliers) >= min_inliers_;
  const bool error_ok =
    max_registration_error_ < 0.0 || registration_error <= max_registration_error_;
  const auto covariance = registrationCovariance(result.H, result.error, result.num_inliers);

  if (log_registration_details_) {
    RCLCPP_INFO(
      this->get_logger(),
      "GICP result: converged=%s iterations=%zu inliers=%zu error=%.6f source_points=%zu "
      "downsampled_source=%zu",
      result.converged ? "true" : "false", result.iterations, result.num_inliers,
      registration_error, accumulated_cloud_->size(), source_->size());
  }

  if (result.converged && inlier_ok && error_ok) {
    const Eigen::Isometry3d candidate = result.T_target_source;
    if (confirmation_count_ > 1) {
      if (confirmationConsistent(candidate)) {
        ++pending_confirmation_count_;
      } else {
        pending_confirmation_transform_ = candidate;
        pending_confirmation_count_ = 1;
      }
      if (pending_confirmation_count_ < confirmation_count_) {
        const auto odom_to_robot_base = getOdomToRobotBase(last_scan_time_);
        const Eigen::Isometry3d map_to_robot_base =
          odom_to_robot_base ? candidate * *odom_to_robot_base : Eigen::Isometry3d::Identity();
        publishObservation(
          false,
          ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_PENDING_CONFIRMATION,
          "awaiting consistent GICP confirmation", result.num_inliers, registration_error,
          source_->size(), map_to_robot_base, covariance);
        accumulated_cloud_->clear();
        first_accumulated_scan_time_.reset();
        return;
      }
    }

    const auto odom_to_robot_base = getOdomToRobotBase(last_scan_time_);
    if (!odom_to_robot_base) {
      publishObservation(
        false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_NO_ODOM,
        "odom->robot_base unavailable at observation time", result.num_inliers, registration_error,
        source_->size(), Eigen::Isometry3d::Identity(), covariance);
      accumulated_cloud_->clear();
      first_accumulated_scan_time_.reset();
      return;
    }

    result_t_ = previous_result_t_ = candidate;
    if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
      last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    }
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    publishObservation(
      true, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_ACCEPTED, "accepted",
      result.num_inliers, registration_error, source_->size(), result_t_ * *odom_to_robot_base,
      covariance);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject GICP result: converged=%s inliers=%zu/%d error=%.6f max_error=%.6f",
      result.converged ? "true" : "false", result.num_inliers, min_inliers_, registration_error,
      max_registration_error_);
    std::string reason = "rejected";
    if (!result.converged) {
      reason = "not converged";
    } else if (!inlier_ok) {
      reason = "insufficient inliers";
    } else if (!error_ok) {
      reason = "registration error too large";
    }
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    publishObservation(
      false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED, reason,
      result.num_inliers, registration_error, source_->size(), Eigen::Isometry3d::Identity(),
      covariance);
  }

  accumulated_cloud_->clear();
  first_accumulated_scan_time_.reset();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!publish_tf_) {
    return;
  }
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // Prefer scan time for consistency, but clamp stale scan stamps to avoid Nav2 asking
  // for transforms newer than the tf cache during simulation slowdowns.
  const auto current_time = now();
  rclcpp::Time tf_stamp = current_time;
  if (has_received_scan_) {
    tf_stamp = last_scan_time_ + rclcpp::Duration::from_seconds(transform_future_offset_s_);
    const double lag_s = (current_time - last_scan_time_).seconds();
    if (max_scan_stamp_lag_s_ > 0.0 && lag_s > max_scan_stamp_lag_s_) {
      tf_stamp = current_time + rclcpp::Duration::from_seconds(transform_future_offset_s_);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "small_gicp tf stamp falls behind current time by %.3fs, clamping map->odom stamp to "
        "now().",
        lag_s);
    }
  }
  transform_stamped.header.stamp = tf_stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::publishObservation(
  bool accepted, std::uint8_t status, const std::string & message, std::size_t inliers,
  double error, std::size_t source_points, const Eigen::Isometry3d & map_to_robot_base,
  const std::array<double, 36> & covariance)
{
  if (!observation_pub_) {
    return;
  }

  ats_navigation_interfaces::msg::RelocalizationObservation observation;
  observation.header.stamp = has_received_scan_ ? last_scan_time_ : now();
  observation.header.frame_id = map_frame_;
  observation.child_frame_id = robot_base_frame_;
  observation.sequence = ++observation_sequence_;
  observation.accepted = accepted;
  observation.status = status;
  observation.inlier_count = static_cast<std::uint32_t>(
    std::min<std::size_t>(inliers, std::numeric_limits<std::uint32_t>::max()));
  observation.source_points = static_cast<std::uint32_t>(
    std::min<std::size_t>(source_points, std::numeric_limits<std::uint32_t>::max()));
  observation.registration_error = std::isfinite(error) ? error : -1.0;
  const double inlier_ratio =
    source_points > 0 ? static_cast<double>(inliers) / static_cast<double>(source_points) : 0.0;
  const double error_quality =
    std::isfinite(error) && error >= 0.0 ? std::exp(-std::min(error, 10.0)) : 0.0;
  observation.quality = accepted ? std::clamp(inlier_ratio * error_quality, 0.0, 1.0) : 0.0;
  observation.message = message;

  const Eigen::Vector3d translation = map_to_robot_base.translation();
  const Eigen::Quaterniond rotation(map_to_robot_base.rotation());
  observation.pose.pose.position.x = translation.x();
  observation.pose.pose.position.y = translation.y();
  observation.pose.pose.position.z = translation.z();
  observation.pose.pose.orientation.x = rotation.x();
  observation.pose.pose.orientation.y = rotation.y();
  observation.pose.pose.orientation.z = rotation.z();
  observation.pose.pose.orientation.w = rotation.w();

  observation.pose.covariance = covariance;

  observation_pub_->publish(observation);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    accumulated_cloud_->clear();
    first_accumulated_scan_time_.reset();
    initial_pose_override_time_ = this->now();
    if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
      last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
  }
}

bool SmallGicpRelocalizationNode::shouldRunRegistration()
{
  if (!has_received_scan_ || accumulated_cloud_->empty()) {
    return false;
  }

  if (first_accumulated_scan_time_) {
    const double accumulation_age = accumulatedCloudAgeSeconds();
    if (max_accumulation_age_s_ > 0.0 && accumulation_age < max_accumulation_age_s_) {
      return false;
    }
  }

  const bool force_after_initial_pose =
    initial_pose_override_time_ && (this->now() - *initial_pose_override_time_).seconds() <=
                                     initial_pose_force_registration_window_s_;
  if (force_after_initial_pose) {
    return true;
  }

  if (pending_confirmation_transform_) {
    return true;
  }

  auto current_robot_base_to_odom = getCurrentRobotBaseToOdom();
  if (!current_robot_base_to_odom) {
    return false;
  }

  if (!last_registration_robot_base_to_odom_) {
    last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    return false;
  }

  const double translation_delta = translationDeltaFromLastTrigger(*current_robot_base_to_odom);
  const double yaw_delta = yawDeltaFromLastTrigger(*current_robot_base_to_odom);
  return translation_delta >= std::max(0.0, min_registration_translation_delta_) ||
         yaw_delta >= std::max(0.0, min_registration_yaw_delta_);
}

double SmallGicpRelocalizationNode::accumulatedCloudAgeSeconds() const
{
  if (!first_accumulated_scan_time_) {
    return 0.0;
  }
  return std::max(0.0, (last_scan_time_ - *first_accumulated_scan_time_).seconds());
}

std::optional<Eigen::Isometry3d> SmallGicpRelocalizationNode::getCurrentRobotBaseToOdom() const
{
  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Isometry3d> SmallGicpRelocalizationNode::getOdomToRobotBase(
  const rclcpp::Time & stamp) const
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

bool SmallGicpRelocalizationNode::confirmationConsistent(const Eigen::Isometry3d & candidate) const
{
  return pending_confirmation_transform_ &&
         (candidate.translation() - pending_confirmation_transform_->translation()).norm() <=
           confirmation_translation_tolerance_ &&
         yawDistance(candidate, *pending_confirmation_transform_) <= confirmation_yaw_tolerance_;
}

double SmallGicpRelocalizationNode::translationDeltaFromLastTrigger(
  const Eigen::Isometry3d & current_robot_base_to_odom) const
{
  if (!last_registration_robot_base_to_odom_) {
    return 0.0;
  }
  return (current_robot_base_to_odom.translation() -
          last_registration_robot_base_to_odom_->translation())
    .norm();
}

double SmallGicpRelocalizationNode::yawDeltaFromLastTrigger(
  const Eigen::Isometry3d & current_robot_base_to_odom) const
{
  if (!last_registration_robot_base_to_odom_) {
    return 0.0;
  }
  const double current_yaw = current_robot_base_to_odom.rotation().eulerAngles(0, 1, 2).z();
  const double previous_yaw =
    last_registration_robot_base_to_odom_->rotation().eulerAngles(0, 1, 2).z();
  return std::abs(
    std::atan2(std::sin(current_yaw - previous_yaw), std::cos(current_yaw - previous_yaw)));
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
