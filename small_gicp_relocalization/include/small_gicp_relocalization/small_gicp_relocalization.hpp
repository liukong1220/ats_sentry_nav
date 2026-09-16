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
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
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
  ~SmallGicpRelocalizationNode() override;

private:
  using PointCovarianceCloud = pcl::PointCloud<pcl::PointCovariance>;
  using PointKdTree = small_gicp::KdTree<PointCovarianceCloud>;
  using GicpRegistration =
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;

  struct RegistrationAttempt
  {
    bool ok{false};
    bool converged{false};
    std::size_t iterations{0};
    std::size_t num_inliers{0};
    double registration_error{std::numeric_limits<double>::infinity()};
    double overlap_ratio{0.0};
    double min_information_eigenvalue{0.0};
    double information_condition_number{std::numeric_limits<double>::infinity()};
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Zero()};
    std::string stage{"none"};
    std::string reject_reason;
  };

  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void publishObservation(
    bool accepted, std::uint8_t status, const std::string & message, std::size_t inliers,
    double error, std::size_t source_points, const Eigen::Isometry3d & map_to_robot_base,
    const std::array<double, 36> & covariance);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void localizationStatusCallback(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr msg);
  bool shouldRunRegistration();
  bool isRecoveringLocalization() const;
  bool preferMultiGuess() const;
  void clearAccumulation();
  void preprocessAccumulatedSource();
  double accumulatedCloudAgeSeconds() const;
  std::optional<Eigen::Isometry3d> getCurrentRobotBaseToOdom() const;
  std::optional<Eigen::Isometry3d> getOdomToRobotBase(const rclcpp::Time & stamp) const;
  bool confirmationConsistent(const Eigen::Isometry3d & candidate) const;
  double translationDeltaFromLastTrigger(
    const Eigen::Isometry3d & current_robot_base_to_odom) const;
  double yawDeltaFromLastTrigger(const Eigen::Isometry3d & current_robot_base_to_odom) const;

  RegistrationAttempt alignOnce(
    const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations) const;
  RegistrationAttempt alignOnceOn(
    const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations,
    const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
    GicpRegistration & registration) const;
  RegistrationAttempt runCoarseFineAlignment(const Eigen::Isometry3d & initial_guess);
  RegistrationAttempt runMultiGuessAlignment();
  RegistrationAttempt runMultiGuessAlignmentOn(
    const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
    const Eigen::Isometry3d & seed, const std::atomic<bool> & cancel_flag) const;
  bool passesQualityGates(RegistrationAttempt & attempt) const;
  std::vector<Eigen::Isometry3d> buildMultiGuessCandidates() const;
  std::vector<Eigen::Isometry3d> buildMultiGuessCandidatesFrom(
    const Eigen::Isometry3d & seed) const;
  void startAsyncMultiGuess();
  void cancelAsyncMultiGuess();
  void drainAsyncMultiGuessResult();
  void handleRegistrationAttempt(
    RegistrationAttempt attempt, const rclcpp::Time & scan_time, std::size_t source_points);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_status_sub_;
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
  bool relax_convergence_for_sim_{false};
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

  // Coarse-to-fine / windowed alignment (BIT icp_relocalization + HWSentry quality gates).
  std::string registration_mode_{"initial_guess"};
  int accumulate_frames_{1};
  int accumulated_frame_count_{0};
  bool fine_alignment_enabled_{true};
  bool coarse_first_window_only_{true};
  float fine_max_dist_sq_{0.2025f};  // 0.45 m
  int coarse_max_iterations_{10};
  int fine_max_iterations_{16};
  double min_overlap_ratio_{0.0};
  double min_information_eigenvalue_{0.0};
  double max_information_condition_number_{0.0};
  double multi_guess_search_half_xy_{2.0};
  double multi_guess_step_xy_{1.0};
  double multi_guess_step_yaw_{0.785398};
  std::vector<double> multi_guess_z_candidates_{0.0};
  bool need_coarse_alignment_{true};
  bool has_accepted_alignment_{false};

  // Status-driven recovery: TRACKING keeps fine-only; LOST opens async multi_guess.
  bool follow_localization_status_{true};
  bool auto_multi_guess_on_lost_{true};
  bool force_registration_when_lost_{true};
  std::uint8_t localization_state_{
    ats_navigation_interfaces::msg::LocalizationStatus::STATE_UNINITIALIZED};
  std::uint64_t localization_epoch_{0};
  double observation_silence_sec_{0.0};
  std::optional<rclcpp::Time> last_status_receive_time_;
  // After a stall (e.g. SIGSTOP), skip registration until a fresh status sample arrives
  // so we do not act on a ghost TRACKING state that missed LOST transitions.
  double status_stale_skip_registration_s_{1.0};
  bool height_filter_enable_{false};
  float height_filter_min_z_{-0.5f};
  float height_filter_max_z_{2.5f};

  // LOST multi_guess runs off the ROS callback thread so /initialpose stays responsive.
  std::atomic<bool> multi_guess_running_{false};
  std::atomic<bool> cancel_multi_guess_{false};
  std::mutex async_result_mutex_;
  bool async_result_ready_{false};
  RegistrationAttempt async_result_;
  rclcpp::Time async_result_scan_time_;
  std::size_t async_result_source_points_{0};
  std::thread multi_guess_thread_;

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
  PointCovarianceCloud::Ptr target_;
  PointCovarianceCloud::Ptr source_;

  std::shared_ptr<PointKdTree> target_tree_;
  std::shared_ptr<PointKdTree> source_tree_;
  std::shared_ptr<GicpRegistration> register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
