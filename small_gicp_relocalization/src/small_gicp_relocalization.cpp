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
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

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

void fillInformationMetrics(
  const Eigen::Matrix<double, 6, 6> & information, double & min_eigenvalue,
  double & condition_number)
{
  min_eigenvalue = 0.0;
  condition_number = std::numeric_limits<double>::infinity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
    0.5 * (information + information.transpose()));
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return;
  }
  min_eigenvalue = solver.eigenvalues().minCoeff();
  const double max_eigenvalue = solver.eigenvalues().maxCoeff();
  if (min_eigenvalue > 0.0 && std::isfinite(max_eigenvalue)) {
    condition_number = max_eigenvalue / min_eigenvalue;
  }
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
  this->declare_parameter("relax_convergence_for_sim", false);
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

  // Coarse-to-fine windowed registration (borrowed from BIT icp_relocalization).
  this->declare_parameter("registration_mode", "initial_guess");
  this->declare_parameter("accumulate_frames", 1);
  this->declare_parameter("fine_alignment.enable", true);
  this->declare_parameter("fine_alignment.coarse_first_window_only", true);
  this->declare_parameter("fine_alignment.max_correspondence_distance", 0.45);
  this->declare_parameter("coarse_max_iterations", 10);
  this->declare_parameter("fine_max_iterations", 16);
  // Quality gates (borrowed from HWSentry odom_localizer). 0 disables a gate.
  this->declare_parameter("min_overlap_ratio", 0.0);
  this->declare_parameter("min_information_eigenvalue", 0.0);
  this->declare_parameter("max_information_condition_number", 0.0);
  this->declare_parameter("multi_guess.search_half_xy", 2.0);
  this->declare_parameter("multi_guess.step_xy", 1.0);
  this->declare_parameter("multi_guess.step_yaw", 0.785398);
  this->declare_parameter("multi_guess.z_candidates", std::vector<double>{0.0});
  this->declare_parameter("follow_localization_status", true);
  this->declare_parameter("auto_multi_guess_on_lost", true);
  this->declare_parameter("force_registration_when_lost", true);
  this->declare_parameter("status_stale_skip_registration_s", 1.0);
  this->declare_parameter("height_filter.enable", false);
  this->declare_parameter("height_filter.min_z", -0.5);
  this->declare_parameter("height_filter.max_z", 2.5);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_inliers", min_inliers_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_registration_error", max_registration_error_);
  this->get_parameter("relax_convergence_for_sim", relax_convergence_for_sim_);
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

  this->get_parameter("registration_mode", registration_mode_);
  this->get_parameter("accumulate_frames", accumulate_frames_);
  this->get_parameter("fine_alignment.enable", fine_alignment_enabled_);
  this->get_parameter("fine_alignment.coarse_first_window_only", coarse_first_window_only_);
  double fine_max_correspondence_distance = 0.45;
  this->get_parameter(
    "fine_alignment.max_correspondence_distance", fine_max_correspondence_distance);
  this->get_parameter("coarse_max_iterations", coarse_max_iterations_);
  this->get_parameter("fine_max_iterations", fine_max_iterations_);
  this->get_parameter("min_overlap_ratio", min_overlap_ratio_);
  this->get_parameter("min_information_eigenvalue", min_information_eigenvalue_);
  this->get_parameter("max_information_condition_number", max_information_condition_number_);
  this->get_parameter("multi_guess.search_half_xy", multi_guess_search_half_xy_);
  this->get_parameter("multi_guess.step_xy", multi_guess_step_xy_);
  this->get_parameter("multi_guess.step_yaw", multi_guess_step_yaw_);
  this->get_parameter("multi_guess.z_candidates", multi_guess_z_candidates_);
  this->get_parameter("follow_localization_status", follow_localization_status_);
  this->get_parameter("auto_multi_guess_on_lost", auto_multi_guess_on_lost_);
  this->get_parameter("force_registration_when_lost", force_registration_when_lost_);
  this->get_parameter("status_stale_skip_registration_s", status_stale_skip_registration_s_);
  status_stale_skip_registration_s_ = std::max(0.0, status_stale_skip_registration_s_);
  double height_min_z = -0.5;
  double height_max_z = 2.5;
  this->get_parameter("height_filter.enable", height_filter_enable_);
  this->get_parameter("height_filter.min_z", height_min_z);
  this->get_parameter("height_filter.max_z", height_max_z);
  height_filter_min_z_ = static_cast<float>(height_min_z);
  height_filter_max_z_ = static_cast<float>(std::max(height_min_z + 1e-3, height_max_z));

  accumulate_frames_ = std::max(1, accumulate_frames_);
  coarse_max_iterations_ = std::max(1, coarse_max_iterations_);
  fine_max_iterations_ = std::max(1, fine_max_iterations_);
  fine_max_correspondence_distance = std::max(1e-3, fine_max_correspondence_distance);
  fine_max_dist_sq_ =
    static_cast<float>(fine_max_correspondence_distance * fine_max_correspondence_distance);
  min_overlap_ratio_ = std::clamp(min_overlap_ratio_, 0.0, 1.0);
  min_information_eigenvalue_ = std::max(0.0, min_information_eigenvalue_);
  max_information_condition_number_ = std::max(0.0, max_information_condition_number_);
  multi_guess_search_half_xy_ = std::max(0.0, multi_guess_search_half_xy_);
  multi_guess_step_xy_ = std::max(0.05, multi_guess_step_xy_);
  multi_guess_step_yaw_ = std::max(0.05, multi_guess_step_yaw_);
  if (multi_guess_z_candidates_.empty()) {
    multi_guess_z_candidates_ = {0.0};
  }
  if (registration_mode_ != "multi_guess" && registration_mode_ != "initial_guess") {
    RCLCPP_WARN(
      this->get_logger(), "Invalid registration_mode='%s'; defaulting to initial_guess",
      registration_mode_.c_str());
    registration_mode_ = "initial_guess";
  }

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
  need_coarse_alignment_ = true;
  has_accepted_alignment_ = false;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  localization_status_sub_ =
    this->create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &SmallGicpRelocalizationNode::localizationStatusCallback, this, std::placeholders::_1));
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

  RCLCPP_INFO(
    this->get_logger(),
    "GICP coarse-fine ready: mode=%s accumulate_frames=%d fine=%s coarse_first_only=%s "
    "fine_max_corr=%.3f min_overlap=%.3f follow_status=%s auto_multi_guess_on_lost=%s "
    "height_filter=%s",
    registration_mode_.c_str(), accumulate_frames_, fine_alignment_enabled_ ? "true" : "false",
    coarse_first_window_only_ ? "true" : "false", std::sqrt(static_cast<double>(fine_max_dist_sq_)),
    min_overlap_ratio_, follow_localization_status_ ? "true" : "false",
    auto_multi_guess_on_lost_ ? "true" : "false", height_filter_enable_ ? "true" : "false");
}

SmallGicpRelocalizationNode::~SmallGicpRelocalizationNode() { cancelAsyncMultiGuess(); }

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
  ++accumulated_frame_count_;
}

void SmallGicpRelocalizationNode::clearAccumulation()
{
  accumulated_cloud_->clear();
  first_accumulated_scan_time_.reset();
  accumulated_frame_count_ = 0;
}

bool SmallGicpRelocalizationNode::isRecoveringLocalization() const
{
  if (!follow_localization_status_) {
    return !has_accepted_alignment_ || need_coarse_alignment_;
  }
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  return localization_state_ == LS::STATE_UNINITIALIZED || localization_state_ == LS::STATE_LOST ||
         localization_state_ == LS::STATE_DEGRADED ||
         localization_state_ == LS::STATE_RELOCALIZING || !has_accepted_alignment_;
}

bool SmallGicpRelocalizationNode::preferMultiGuess() const
{
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  // Only suppress lattice search while the /initialpose force window is active.
  // A stale initial_pose_override_time_ must NOT permanently disable LOST recovery.
  const bool in_initial_pose_force_window =
    initial_pose_override_time_ && (this->now() - *initial_pose_override_time_).seconds() <=
                                     initial_pose_force_registration_window_s_;

  // Explicit multi_guess mode: lattice until first accept / while coarse needed.
  if (registration_mode_ == "multi_guess") {
    if (in_initial_pose_force_window) {
      return false;
    }
    return need_coarse_alignment_ || !has_accepted_alignment_;
  }
  // Auto multi_guess ONLY for LOST. DEGRADED keeps seeded coarse+fine.
  if (!auto_multi_guess_on_lost_ || !follow_localization_status_) {
    return false;
  }
  if (in_initial_pose_force_window) {
    return false;
  }
  // Silence alone must not fire on UNINITIALIZED cold start (silence may be inf).
  // Only reinforce LOST/DEGRADED when fusion already reports recovery health.
  if (
    observation_silence_sec_ >= 5.0 &&
    (localization_state_ == LS::STATE_LOST || localization_state_ == LS::STATE_DEGRADED)) {
    return localization_state_ == LS::STATE_LOST;
  }
  return localization_state_ == LS::STATE_LOST;
}

void SmallGicpRelocalizationNode::preprocessAccumulatedSource()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = accumulated_cloud_;
  if (height_filter_enable_ && filtered && !filtered->empty()) {
    auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped->points.reserve(filtered->points.size());
    for (const auto & point : filtered->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < height_filter_min_z_ || point.z > height_filter_max_z_) {
        continue;
      }
      cropped->points.push_back(point);
    }
    cropped->width = static_cast<uint32_t>(cropped->points.size());
    cropped->height = 1;
    cropped->is_dense = false;
    filtered = cropped;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *filtered, registered_leaf_size_);
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));
}

SmallGicpRelocalizationNode::RegistrationAttempt SmallGicpRelocalizationNode::alignOnce(
  const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations) const
{
  if (!register_) {
    RegistrationAttempt attempt;
    attempt.reject_reason = "registration inputs unavailable";
    return attempt;
  }
  return alignOnceOn(initial_guess, max_dist_sq, max_iterations, source_, source_tree_, *register_);
}

SmallGicpRelocalizationNode::RegistrationAttempt SmallGicpRelocalizationNode::alignOnceOn(
  const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations,
  const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
  GicpRegistration & registration) const
{
  RegistrationAttempt attempt;
  if (!source || !source_tree || !target_ || !target_tree_) {
    attempt.reject_reason = "registration inputs unavailable";
    return attempt;
  }

  registration.reduction.num_threads = num_threads_;
  registration.rejector.max_dist_sq = max_dist_sq;
  registration.optimizer.max_iterations = max_iterations;

  const auto result = registration.align(*target_, *source, *target_tree_, initial_guess);
  attempt.converged = result.converged;
  attempt.iterations = result.iterations;
  attempt.num_inliers = result.num_inliers;
  attempt.registration_error = normalizedRegistrationError(result.error, result.num_inliers);
  attempt.transform = result.T_target_source;
  attempt.information = result.H;
  attempt.overlap_ratio =
    source->empty() ? 0.0
                    : static_cast<double>(result.num_inliers) / static_cast<double>(source->size());
  fillInformationMetrics(
    result.H, attempt.min_information_eigenvalue, attempt.information_condition_number);
  return attempt;
}

bool SmallGicpRelocalizationNode::passesQualityGates(RegistrationAttempt & attempt) const
{
  const bool inlier_ok = static_cast<int>(attempt.num_inliers) >= min_inliers_;
  const bool error_ok =
    max_registration_error_ < 0.0 || attempt.registration_error <= max_registration_error_;
  const bool transform_finite = attempt.transform.matrix().allFinite();
  const bool has_initial_pose_seed = static_cast<bool>(initial_pose_override_time_);
  const bool sim_relaxed_ok = relax_convergence_for_sim_ && has_initial_pose_seed && inlier_ok &&
                              transform_finite &&
                              static_cast<int>(attempt.num_inliers) >= (min_inliers_ * 2);

  if (!(attempt.converged && inlier_ok && error_ok) && !sim_relaxed_ok) {
    if (!attempt.converged) {
      attempt.reject_reason = "not converged";
    } else if (!inlier_ok) {
      attempt.reject_reason = "insufficient inliers";
    } else if (!error_ok) {
      attempt.reject_reason = "registration error too large";
    } else {
      attempt.reject_reason = "rejected";
    }
    return false;
  }

  if (min_overlap_ratio_ > 0.0 && attempt.overlap_ratio < min_overlap_ratio_ && !sim_relaxed_ok) {
    attempt.reject_reason = "overlap ratio below threshold";
    return false;
  }
  if (
    min_information_eigenvalue_ > 0.0 &&
    attempt.min_information_eigenvalue < min_information_eigenvalue_ && !sim_relaxed_ok) {
    attempt.reject_reason = "minimum information eigenvalue below threshold";
    return false;
  }
  if (
    max_information_condition_number_ > 0.0 &&
    attempt.information_condition_number > max_information_condition_number_ && !sim_relaxed_ok) {
    attempt.reject_reason = "information matrix is too ill-conditioned";
    return false;
  }

  if (sim_relaxed_ok && !(attempt.converged && error_ok)) {
    attempt.stage += "+sim_relax";
  }
  attempt.ok = true;
  return true;
}

std::vector<Eigen::Isometry3d> SmallGicpRelocalizationNode::buildMultiGuessCandidates() const
{
  return buildMultiGuessCandidatesFrom(previous_result_t_);
}

std::vector<Eigen::Isometry3d> SmallGicpRelocalizationNode::buildMultiGuessCandidatesFrom(
  const Eigen::Isometry3d & seed) const
{
  std::vector<Eigen::Isometry3d> candidates;
  const double seed_yaw = seed.rotation().eulerAngles(0, 1, 2).z();
  const double cx = seed.translation().x();
  const double cy = seed.translation().y();
  const double half = multi_guess_search_half_xy_;
  const double step = multi_guess_step_xy_;

  for (double z : multi_guess_z_candidates_) {
    for (double x = cx - half; x <= cx + half + 1e-9; x += step) {
      for (double y = cy - half; y <= cy + half + 1e-9; y += step) {
        for (double yaw = seed_yaw - M_PI; yaw < seed_yaw + M_PI - 1e-9;
             yaw += multi_guess_step_yaw_) {
          Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
          guess.translation() << x, y, seed.translation().z() + z;
          guess.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          candidates.push_back(guess);
        }
      }
    }
  }
  if (candidates.empty()) {
    candidates.push_back(seed);
  }
  return candidates;
}

SmallGicpRelocalizationNode::RegistrationAttempt
SmallGicpRelocalizationNode::runMultiGuessAlignment()
{
  std::atomic<bool> never_cancel{false};
  return runMultiGuessAlignmentOn(source_, source_tree_, previous_result_t_, never_cancel);
}

SmallGicpRelocalizationNode::RegistrationAttempt
SmallGicpRelocalizationNode::runMultiGuessAlignmentOn(
  const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
  const Eigen::Isometry3d & seed, const std::atomic<bool> & cancel_flag) const
{
  RegistrationAttempt best;
  best.reject_reason = "multi_guess produced no valid candidate";
  if (!source || !source_tree || !target_ || !target_tree_) {
    best.reject_reason = "registration inputs unavailable";
    return best;
  }

  GicpRegistration local_register;
  const auto candidates = buildMultiGuessCandidatesFrom(seed);
  RCLCPP_WARN(
    this->get_logger(), "Starting async multi_guess recovery with %zu candidates (state=%u)",
    candidates.size(), static_cast<unsigned>(localization_state_));
  constexpr std::size_t kMaxMultiGuessCandidates = 48;
  const std::size_t limit = std::min(candidates.size(), kMaxMultiGuessCandidates);
  for (std::size_t i = 0; i < limit; ++i) {
    if (cancel_flag.load()) {
      best.reject_reason = "multi_guess cancelled";
      best.ok = false;
      return best;
    }
    const auto & guess = candidates[i];
    auto coarse =
      alignOnceOn(guess, max_dist_sq_, coarse_max_iterations_, source, source_tree, local_register);
    coarse.stage = "multi_guess_coarse";
    if (!passesQualityGates(coarse)) {
      continue;
    }
    RegistrationAttempt refined = coarse;
    if (fine_alignment_enabled_) {
      refined = alignOnceOn(
        coarse.transform, fine_max_dist_sq_, fine_max_iterations_, source, source_tree,
        local_register);
      refined.stage = "multi_guess_fine";
      if (!passesQualityGates(refined)) {
        continue;
      }
    }
    if (!best.ok || refined.registration_error < best.registration_error) {
      best = refined;
    }
  }
  if (candidates.size() > kMaxMultiGuessCandidates) {
    RCLCPP_WARN(
      this->get_logger(), "multi_guess truncated %zu -> %zu candidates to protect latency",
      candidates.size(), kMaxMultiGuessCandidates);
  }
  return best;
}

SmallGicpRelocalizationNode::RegistrationAttempt
SmallGicpRelocalizationNode::runCoarseFineAlignment(const Eigen::Isometry3d & initial_guess)
{
  const bool coarse_required = !fine_alignment_enabled_ || !coarse_first_window_only_ ||
                               need_coarse_alignment_ || !has_accepted_alignment_;

  RegistrationAttempt attempt;
  if (coarse_required || !fine_alignment_enabled_) {
    attempt = alignOnce(initial_guess, max_dist_sq_, coarse_max_iterations_);
    attempt.stage = fine_alignment_enabled_ ? "coarse" : "single";
    if (!passesQualityGates(attempt)) {
      return attempt;
    }
    if (fine_alignment_enabled_) {
      auto fine = alignOnce(attempt.transform, fine_max_dist_sq_, fine_max_iterations_);
      fine.stage = "coarse+fine";
      if (!passesQualityGates(fine)) {
        return fine;
      }
      return fine;
    }
    return attempt;
  }

  attempt = alignOnce(initial_guess, fine_max_dist_sq_, fine_max_iterations_);
  attempt.stage = "fine_only";
  passesQualityGates(attempt);
  return attempt;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  drainAsyncMultiGuessResult();

  if (!shouldRunRegistration()) {
    return;
  }

  if (accumulated_cloud_->empty()) {
    return;
  }

  if (static_cast<int>(accumulated_cloud_->size()) < min_source_points_) {
    return;
  }

  // Preprocess the accumulated window once; coarse and fine reuse the same source.
  preprocessAccumulatedSource();
  if (!source_ || !source_tree_ || source_->empty()) {
    return;
  }

  if (isRecoveringLocalization()) {
    need_coarse_alignment_ = true;
  }

  // LOST multi_guess must not block the ROS callback thread (/initialpose, status, scans).
  if (preferMultiGuess()) {
    startAsyncMultiGuess();
    clearAccumulation();
    return;
  }

  RegistrationAttempt attempt = runCoarseFineAlignment(previous_result_t_);
  handleRegistrationAttempt(attempt, last_scan_time_, source_->size());
  clearAccumulation();
}

void SmallGicpRelocalizationNode::startAsyncMultiGuess()
{
  bool expected = false;
  if (!multi_guess_running_.compare_exchange_strong(expected, true)) {
    return;
  }
  cancel_multi_guess_.store(false);

  const auto source = source_;
  const auto source_tree = source_tree_;
  const Eigen::Isometry3d seed = previous_result_t_;
  const rclcpp::Time scan_time = last_scan_time_;
  const std::size_t source_points = source ? source->size() : 0;

  if (multi_guess_thread_.joinable()) {
    multi_guess_thread_.join();
  }

  multi_guess_thread_ = std::thread([this, source, source_tree, seed, scan_time, source_points]() {
    RegistrationAttempt attempt =
      runMultiGuessAlignmentOn(source, source_tree, seed, cancel_multi_guess_);
    {
      std::lock_guard<std::mutex> lock(async_result_mutex_);
      async_result_ = attempt;
      async_result_scan_time_ = scan_time;
      async_result_source_points_ = source_points;
      async_result_ready_ = true;
    }
    multi_guess_running_.store(false);
  });
}

void SmallGicpRelocalizationNode::cancelAsyncMultiGuess()
{
  cancel_multi_guess_.store(true);
  if (multi_guess_thread_.joinable()) {
    multi_guess_thread_.join();
  }
  multi_guess_running_.store(false);
  std::lock_guard<std::mutex> lock(async_result_mutex_);
  async_result_ready_ = false;
}

void SmallGicpRelocalizationNode::drainAsyncMultiGuessResult()
{
  RegistrationAttempt attempt;
  rclcpp::Time scan_time;
  std::size_t source_points = 0;
  {
    std::lock_guard<std::mutex> lock(async_result_mutex_);
    if (!async_result_ready_) {
      return;
    }
    attempt = async_result_;
    scan_time = async_result_scan_time_;
    source_points = async_result_source_points_;
    async_result_ready_ = false;
  }
  handleRegistrationAttempt(attempt, scan_time, source_points);
}

void SmallGicpRelocalizationNode::handleRegistrationAttempt(
  RegistrationAttempt attempt, const rclcpp::Time & scan_time, std::size_t source_points)
{
  const auto covariance =
    registrationCovariance(attempt.information, attempt.registration_error, attempt.num_inliers);

  if (log_registration_details_) {
    RCLCPP_INFO(
      this->get_logger(),
      "GICP result: stage=%s ok=%s converged=%s iterations=%zu inliers=%zu error=%.6f "
      "overlap=%.3f source_points=%zu",
      attempt.stage.c_str(), attempt.ok ? "true" : "false", attempt.converged ? "true" : "false",
      attempt.iterations, attempt.num_inliers, attempt.registration_error, attempt.overlap_ratio,
      source_points);
  }

  if (attempt.ok) {
    const Eigen::Isometry3d candidate = attempt.transform;
    if (confirmation_count_ > 1) {
      if (confirmationConsistent(candidate)) {
        ++pending_confirmation_count_;
      } else {
        pending_confirmation_transform_ = candidate;
        pending_confirmation_count_ = 1;
      }
      if (pending_confirmation_count_ < confirmation_count_) {
        const auto odom_to_robot_base = getOdomToRobotBase(scan_time);
        const Eigen::Isometry3d map_to_robot_base =
          odom_to_robot_base ? candidate * *odom_to_robot_base : Eigen::Isometry3d::Identity();
        publishObservation(
          false,
          ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_PENDING_CONFIRMATION,
          "awaiting consistent GICP confirmation", attempt.num_inliers, attempt.registration_error,
          source_points, map_to_robot_base, covariance);
        if (coarse_first_window_only_ && fine_alignment_enabled_) {
          need_coarse_alignment_ = false;
        }
        return;
      }
    }

    const auto odom_to_robot_base = getOdomToRobotBase(scan_time);
    if (!odom_to_robot_base) {
      publishObservation(
        false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_NO_ODOM,
        "odom->robot_base unavailable at observation time", attempt.num_inliers,
        attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
      return;
    }

    result_t_ = previous_result_t_ = candidate;
    if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
      last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    }
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    has_accepted_alignment_ = true;
    if (coarse_first_window_only_ && fine_alignment_enabled_) {
      using LS = ats_navigation_interfaces::msg::LocalizationStatus;
      if (!follow_localization_status_) {
        need_coarse_alignment_ = false;
      } else {
        need_coarse_alignment_ = localization_state_ == LS::STATE_LOST ||
                                 localization_state_ == LS::STATE_DEGRADED ||
                                 localization_state_ == LS::STATE_RELOCALIZING;
      }
    }

    publishObservation(
      true, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_ACCEPTED,
      "accepted:" + attempt.stage, attempt.num_inliers, attempt.registration_error, source_points,
      result_t_ * *odom_to_robot_base, covariance);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject GICP result: stage=%s reason=%s converged=%s inliers=%zu/%d error=%.6f "
      "overlap=%.3f max_error=%.6f",
      attempt.stage.c_str(), attempt.reject_reason.c_str(), attempt.converged ? "true" : "false",
      attempt.num_inliers, min_inliers_, attempt.registration_error, attempt.overlap_ratio,
      max_registration_error_);
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    need_coarse_alignment_ = true;
    publishObservation(
      false, ats_navigation_interfaces::msg::RelocalizationObservation::STATUS_REJECTED,
      attempt.reject_reason.empty() ? "rejected" : attempt.reject_reason, attempt.num_inliers,
      attempt.registration_error, source_points, Eigen::Isometry3d::Identity(), covariance);
  }
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
  // Non-finite GICP error (common under sim covariance collapse) must not force
  // quality=0, or localization_fusion rejects with min_observation_quality.
  observation.registration_error = std::isfinite(error) ? error : 0.0;
  const double inlier_ratio =
    source_points > 0 ? static_cast<double>(inliers) / static_cast<double>(source_points) : 0.0;
  const double error_quality =
    std::isfinite(error) && error >= 0.0 ? std::exp(-std::min(error, 10.0)) : 1.0;
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
    clearAccumulation();
    initial_pose_override_time_ = this->now();
    need_coarse_alignment_ = true;
    has_accepted_alignment_ = false;
    // Drop any in-flight LOST lattice search; seeded coarse+fine owns recovery now.
    cancel_multi_guess_.store(true);
    if (auto current_robot_base_to_odom = getCurrentRobotBaseToOdom()) {
      last_registration_robot_base_to_odom_ = *current_robot_base_to_odom;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
  }
}

void SmallGicpRelocalizationNode::localizationStatusCallback(
  const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::uint8_t previous = localization_state_;
  localization_state_ = msg->state;
  localization_epoch_ = msg->epoch;
  observation_silence_sec_ = msg->observation_silence_sec;
  last_status_receive_time_ = this->now();

  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  const bool entered_recovery =
    previous == LS::STATE_TRACKING &&
    (localization_state_ == LS::STATE_LOST || localization_state_ == LS::STATE_DEGRADED);
  if (entered_recovery) {
    need_coarse_alignment_ = true;
    pending_confirmation_transform_.reset();
    pending_confirmation_count_ = 0;
    RCLCPP_WARN(
      this->get_logger(),
      "Localization left TRACKING (state=%u epoch=%s silence=%.2f) - forcing coarse recovery",
      static_cast<unsigned>(localization_state_), std::to_string(localization_epoch_).c_str(),
      observation_silence_sec_);
  }
}

bool SmallGicpRelocalizationNode::shouldRunRegistration()
{
  if (!has_received_scan_ || accumulated_cloud_->empty()) {
    return false;
  }

  if (accumulated_frame_count_ < accumulate_frames_) {
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

  // After a process stall, cached TRACKING can be older than fusion's LOST.
  // Wait for a fresh /localization/status before registering.
  if (
    follow_localization_status_ && status_stale_skip_registration_s_ > 0.0 &&
    last_status_receive_time_ &&
    (this->now() - *last_status_receive_time_).seconds() > status_stale_skip_registration_s_) {
    return false;
  }

  if (pending_confirmation_transform_) {
    return true;
  }

  // Until the first accepted alignment, keep trying (needed for multi_guess / cold start).
  if (!has_accepted_alignment_) {
    return true;
  }

  // LOST/DEGRADED: correctness over frequency — keep attempting recovery windows.
  if (force_registration_when_lost_ && isRecoveringLocalization()) {
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
