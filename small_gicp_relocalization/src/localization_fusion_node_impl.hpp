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

#ifndef SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_NODE_IMPL_HPP_
#define SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_NODE_IMPL_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "ats_navigation_interfaces/msg/identified_odometry.hpp"
#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/relocalization_observation.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "small_gicp_relocalization/localization_fusion_core.hpp"
#include "small_gicp_relocalization/odom_instance_tracker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace small_gicp_relocalization
{

using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;
using RelocalizationObservation = ats_navigation_interfaces::msg::RelocalizationObservation;
using SteadyTime = std::chrono::steady_clock::time_point;

inline bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const double q_norm =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && q_norm > 1e-8;
}

inline bool finiteCovariance(const std::array<double, 36> & covariance)
{
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
    if (covariance[index] < 0.0) {
      return false;
    }
  }
  return true;
}

inline tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  transform.getRotation().normalize();
  return transform;
}

inline double steadySecondsSince(const SteadyTime & stamp)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
}

class LocalizationFusionNode : public rclcpp::Node
{
public:
  explicit LocalizationFusionNode(const rclcpp::NodeOptions & options);

private:
  std::uint64_t instanceUpperBound() const;
  void onIdentityMonitor(
    const ats_navigation_interfaces::msg::IdentifiedOdometry::SharedPtr message);
  void onIdentifiedOdometry(
    const ats_navigation_interfaces::msg::IdentifiedOdometry::SharedPtr message);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onObservation(const RelocalizationObservation::SharedPtr message);
  void onStatusTimer();
  void setStatus(std::uint8_t status, const std::string & message);
  void recordRejectionLocked(const std::string & message, bool count_failure);
  double observationSilenceLocked() const;
  void updateHealthLocked();
  void enterLostLocked(const std::string & message, bool initial_odom_timeout = false);
  void pruneHistoryLocked();
  void publishTfLocked();
  void publishStatusLocked();

  std::string odom_topic_;
  std::string localization_topic_;
  std::string observation_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  bool publish_tf_{true};
  bool allow_initial_identity_{false};
  bool use_initial_map_to_odom_{false};
  double initial_map_to_odom_x_{0.0};
  double initial_map_to_odom_y_{0.0};
  double initial_map_to_odom_z_{0.0};
  double initial_map_to_odom_roll_{0.0};
  double initial_map_to_odom_pitch_{0.0};
  double initial_map_to_odom_yaw_{0.0};
  double odom_timeout_s_{0.5};
  double observation_degraded_timeout_s_{3.0};
  double observation_lost_timeout_s_{10.0};
  double history_duration_s_{5.0};
  std::size_t max_odom_history_samples_{2000};
  std::uint64_t odom_history_count_drops_{0};
  double history_boundary_tolerance_s_{0.1};
  double maximum_interpolation_gap_s_{0.2};
  double transform_future_offset_s_{0.05};
  double relocalizing_hold_s_{0.2};
  int max_consecutive_rejections_{3};
  double epoch_translation_threshold_{0.05};
  double epoch_yaw_threshold_{0.05};
  double continuous_correction_max_translation_{0.0};
  double continuous_correction_max_yaw_{0.0};
  double max_correction_translation_{2.0};
  double max_correction_yaw_{1.0};
  double lost_max_correction_translation_{5.0};
  double lost_max_correction_yaw_{1.5};
  double min_observation_quality_{0.0};
  std::uint32_t min_observation_inliers_{1};
  double max_registration_error_{-1.0};
  double status_publish_rate_hz_{10.0};
  bool degrade_on_out_of_order_odom_{false};
  DirectionalFusionConfig directional_fusion_config_;
  TrackingFilterConfig tracking_filter_config_;

  std::mutex mutex_;
  std::optional<tf2::Transform> last_accepted_odom_to_base_;
  bool tracking_lpf_settling_{false};
  std::optional<tf2::Transform> pending_jump_candidate_;
  int pending_jump_count_{0};
  OdomInstanceTracker odom_instance_tracker_;
  builtin_interfaces::msg::Time odom_epoch_start_stamp_;
  double identity_future_tolerance_s_{5.0};
  double identity_max_age_s_{0.5};
  std::deque<OdomPoseSample> odom_history_;
  std::optional<rclcpp::Time> latest_odom_stamp_;
  std::optional<SteadyTime> last_odom_receive_time_;
  std::optional<rclcpp::Time> last_input_observation_stamp_;
  std::optional<std::uint64_t> last_input_observation_sequence_;
  std::optional<rclcpp::Time> last_observation_stamp_;
  std::optional<std::uint64_t> last_observation_sequence_;
  std::optional<SteadyTime> last_observation_receive_time_;
  const SteadyTime startup_time_{std::chrono::steady_clock::now()};
  // First valid odometry arms bootstrap even when there is no map->odom seed.
  std::optional<SteadyTime> observation_clock_start_;
  std::optional<SteadyTime> relocalizing_until_;
  std::optional<SteadyTime> recovery_until_;
  std::optional<rclcpp::Time> recovery_observation_floor_;
  // After a recovery episode times out, pending alone cannot reopen it.
  bool recovery_episode_closed_{false};
  // Survives RECOVERING so only LOST-origin recovery receives the wider gate.
  bool lost_latched_{false};
  bool initial_odom_timeout_only_{false};
  tf2::Transform map_to_odom_{tf2::Transform::getIdentity()};
  bool has_map_to_odom_{false};
  bool has_accepted_observation_{false};
  std::uint64_t epoch_{0};
  double last_correction_translation_{0.0};
  double last_correction_yaw_{0.0};
  std::uint32_t consecutive_rejections_{0};
  std::uint8_t status_{LocalizationStatus::STATE_UNINITIALIZED};
  std::int64_t last_status_stamp_ns_{0};
  std::string status_message_{"waiting for odometry"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr
    identified_odom_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::IdentifiedOdometry>::SharedPtr
    identity_monitor_sub_;
  rclcpp::Subscription<RelocalizationObservation>::SharedPtr observation_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization


#endif  // SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_NODE_IMPL_HPP_
