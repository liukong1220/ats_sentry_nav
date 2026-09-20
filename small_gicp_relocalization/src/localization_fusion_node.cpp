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

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/relocalization_observation.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "small_gicp_relocalization/localization_fusion_core.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace small_gicp_relocalization
{

namespace
{
using LocalizationStatus = ats_navigation_interfaces::msg::LocalizationStatus;
using RelocalizationObservation = ats_navigation_interfaces::msg::RelocalizationObservation;
using SteadyTime = std::chrono::steady_clock::time_point;

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const double q_norm =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && q_norm > 1e-8;
}

bool finiteCovariance(const std::array<double, 36> & covariance)
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

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  transform.getRotation().normalize();
  return transform;
}

double steadySecondsSince(const SteadyTime & stamp)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
}

}  // namespace

class LocalizationFusionNode : public rclcpp::Node
{
public:
  explicit LocalizationFusionNode(const rclcpp::NodeOptions & options)
  : Node("localization_fusion", options)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry");
    localization_topic_ = declare_parameter<std::string>("localization_topic", "/localization");
    observation_topic_ =
      declare_parameter<std::string>("observation_topic", "relocalization_observation");
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/status");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "gimbal_yaw_odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    allow_initial_identity_ = declare_parameter<bool>("allow_initial_identity", false);
    use_initial_map_to_odom_ = declare_parameter<bool>("use_initial_map_to_odom", false);
    initial_map_to_odom_x_ = declare_parameter<double>("initial_map_to_odom_x", 0.0);
    initial_map_to_odom_y_ = declare_parameter<double>("initial_map_to_odom_y", 0.0);
    initial_map_to_odom_z_ = declare_parameter<double>("initial_map_to_odom_z", 0.0);
    initial_map_to_odom_roll_ = declare_parameter<double>("initial_map_to_odom_roll", 0.0);
    initial_map_to_odom_pitch_ = declare_parameter<double>("initial_map_to_odom_pitch", 0.0);
    initial_map_to_odom_yaw_ = declare_parameter<double>("initial_map_to_odom_yaw", 0.0);
    odom_timeout_s_ = std::max(0.05, declare_parameter<double>("odom_timeout_s", 0.5));
    observation_degraded_timeout_s_ =
      std::max(0.0, declare_parameter<double>("observation_timeout_s", 3.0));
    observation_lost_timeout_s_ = std::max(
      observation_degraded_timeout_s_,
      declare_parameter<double>("observation_lost_timeout_s", 10.0));
    if (!std::isfinite(observation_lost_timeout_s_) || observation_lost_timeout_s_ <= 0.0) {
      throw std::invalid_argument("observation_lost_timeout_s must be positive and finite");
    }
    observation_stamp_max_age_s_ =
      std::max(0.0, declare_parameter<double>("observation_stamp_max_age_s", 1.0));
    observation_stamp_max_future_s_ =
      std::max(0.0, declare_parameter<double>("observation_stamp_max_future_s", 0.25));
    history_duration_s_ = std::max(0.5, declare_parameter<double>("history_duration_s", 5.0));
    const auto max_history = declare_parameter<std::int64_t>("max_odom_history_samples", 2000);
    if (max_history < 2) {
      throw std::invalid_argument("max_odom_history_samples must be at least 2");
    }
    max_odom_history_samples_ = static_cast<std::size_t>(max_history);
    history_boundary_tolerance_s_ =
      std::max(0.0, declare_parameter<double>("history_boundary_tolerance_s", 0.10));
    maximum_interpolation_gap_s_ =
      std::max(0.0, declare_parameter<double>("maximum_interpolation_gap_s", 0.20));
    transform_future_offset_s_ =
      std::max(0.0, declare_parameter<double>("transform_future_offset_s", 0.05));
    relocalizing_hold_s_ = std::max(0.0, declare_parameter<double>("relocalizing_hold_s", 0.20));
    max_consecutive_rejections_ = static_cast<int>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("max_consecutive_rejections", 3)));
    epoch_translation_threshold_ =
      std::max(0.0, declare_parameter<double>("epoch_translation_threshold", 0.05));
    epoch_yaw_threshold_ = std::max(0.0, declare_parameter<double>("epoch_yaw_threshold", 0.05));
    max_correction_translation_ =
      std::max(0.0, declare_parameter<double>("max_correction_translation", 2.0));
    max_correction_yaw_ = std::max(0.0, declare_parameter<double>("max_correction_yaw", 1.0));
    // LOST-only wider gates for global recovery; TRACKING stays fail-closed at the base limits.
    lost_max_correction_translation_ = std::max(
      max_correction_translation_,
      declare_parameter<double>("lost_max_correction_translation", 5.0));
    lost_max_correction_yaw_ =
      std::max(max_correction_yaw_, declare_parameter<double>("lost_max_correction_yaw", 1.5));
    min_observation_quality_ =
      std::clamp(declare_parameter<double>("min_observation_quality", 0.0), 0.0, 1.0);
    min_observation_inliers_ = static_cast<std::uint32_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("min_observation_inliers", 1)));
    max_registration_error_ = declare_parameter<double>("max_registration_error", -1.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    localization_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(localization_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<LocalizationStatus>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationFusionNode::onOdometry, this, std::placeholders::_1));
    observation_sub_ = create_subscription<RelocalizationObservation>(
      observation_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalizationFusionNode::onObservation, this, std::placeholders::_1));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&LocalizationFusionNode::onStatusTimer, this));

    const std::array<double, 6> initial_map_to_odom{
      initial_map_to_odom_x_,    initial_map_to_odom_y_,     initial_map_to_odom_z_,
      initial_map_to_odom_roll_, initial_map_to_odom_pitch_, initial_map_to_odom_yaw_};
    if (std::any_of(initial_map_to_odom.begin(), initial_map_to_odom.end(), [](double value) {
          return !std::isfinite(value);
        })) {
      throw std::invalid_argument("initial map->odom parameter contains a non-finite value");
    }
    if (use_initial_map_to_odom_ && allow_initial_identity_) {
      throw std::invalid_argument(
        "use_initial_map_to_odom and allow_initial_identity are mutually exclusive");
    }
    if (use_initial_map_to_odom_) {
      tf2::Quaternion rotation;
      rotation.setRPY(
        initial_map_to_odom_roll_, initial_map_to_odom_pitch_, initial_map_to_odom_yaw_);
      rotation.normalize();
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_.setOrigin(
        tf2::Vector3(initial_map_to_odom_x_, initial_map_to_odom_y_, initial_map_to_odom_z_));
      map_to_odom_.setRotation(rotation);
      has_map_to_odom_ = true;
      epoch_ = 1;
      status_message_ = "map->odom initialized by explicit parameter";
    } else if (allow_initial_identity_) {
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_.setIdentity();
      has_map_to_odom_ = true;
      epoch_ = 1;
      status_message_ = "identity map->odom initialized by parameter";
    }
    RCLCPP_INFO(
      get_logger(),
      "Localization fusion ready: odom='%s' localization='%s' observation='%s' status='%s'",
      odom_topic_.c_str(), localization_topic_.c_str(), observation_topic_.c_str(),
      status_topic_.c_str());
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finitePose(message->pose.pose)) {
      setStatus(LocalizationStatus::STATE_LOST, "received non-finite odometry");
      return;
    }
    nav_msgs::msg::Odometry localization = *message;
    if (localization.header.frame_id.empty()) {
      localization.header.frame_id = odom_frame_;
    }
    if (localization.child_frame_id.empty()) {
      localization.child_frame_id = robot_base_frame_;
    }
    if (
      localization.header.frame_id != odom_frame_ ||
      localization.child_frame_id != robot_base_frame_) {
      setStatus(LocalizationStatus::STATE_LOST, "odometry frame contract mismatch");
      return;
    }

    if (localization.header.stamp.sec < 0 || localization.header.stamp.nanosec >= 1000000000U ||
        (localization.header.stamp.sec == 0 && localization.header.stamp.nanosec == 0)) {
      setStatus(LocalizationStatus::STATE_LOST, "invalid odometry stamp");
      return;
    }
    const rclcpp::Time stamp(localization.header.stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Observe an expired lease before this input can renew its receive time.
      updateHealthLocked();
      if (latest_odom_stamp_ && stamp < *latest_odom_stamp_) {
        recordRejectionLocked("dropped out-of-order odometry", true);
        if (!lost_latched_) {
          status_ = LocalizationStatus::STATE_DEGRADED;
          relocalizing_until_.reset();
        }
        publishStatusLocked();
        return;
      }
      if (latest_odom_stamp_ && stamp == *latest_odom_stamp_) {
        return;
      }
      latest_odom_stamp_ = stamp;
      last_odom_receive_time_ = std::chrono::steady_clock::now();
      if (!observation_clock_start_) {
        observation_clock_start_ = last_odom_receive_time_;
      }
      odom_history_.push_back(OdomPoseSample{stamp, poseToTransform(localization.pose.pose)});
      pruneHistoryLocked();
      updateHealthLocked();
      publishStatusLocked();
    }
    // /localization 保持连续 odom 状态；全局修正只通过唯一 map->odom 表达。
    localization_pub_->publish(localization);
  }

  void onObservation(const RelocalizationObservation::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool was_recovering = recovery_until_.has_value();
    updateHealthLocked();
    if (was_recovering && !recovery_until_) {
      recordRejectionLocked("relocalization recovery episode expired", true);
      publishStatusLocked();
      return;
    }
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U ||
        (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)) {
      recordRejectionLocked("invalid relocalization observation stamp", true);
      publishStatusLocked();
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (recovery_observation_floor_ && stamp <= *recovery_observation_floor_) {
      recordRejectionLocked("relocalization observation predates recovery boundary", true);
      publishStatusLocked();
      return;
    }
    if (!observationIsNewer(
          stamp, message->sequence, last_input_observation_stamp_,
          last_input_observation_sequence_)) {
      recordRejectionLocked("dropped delayed or duplicate relocalization observation", true);
      publishStatusLocked();
      return;
    }

    const std::string stamp_error = validateObservationStamp(
      stamp, now(), observation_stamp_max_age_s_, observation_stamp_max_future_s_);
    if (!stamp_error.empty()) {
      recordRejectionLocked(stamp_error, true);
      publishStatusLocked();
      return;
    }

    const bool pending = message->status == RelocalizationObservation::STATUS_PENDING_CONFIRMATION;
    if (!pending &&
        (!message->accepted || message->status != RelocalizationObservation::STATUS_ACCEPTED)) {
      last_input_observation_stamp_ = stamp;
      last_input_observation_sequence_ = message->sequence;
      recordRejectionLocked("GICP rejected: " + message->message, true);
      publishStatusLocked();
      return;
    }
    if (message->header.frame_id != map_frame_ || message->child_frame_id != robot_base_frame_) {
      recordRejectionLocked("relocalization observation frame mismatch", true);
      publishStatusLocked();
      return;
    }
    if (
      (pending && message->accepted) || !finitePose(message->pose.pose) ||
      !finiteCovariance(message->pose.covariance) || !std::isfinite(message->quality) ||
      message->quality < 0.0 || (!pending && message->quality < min_observation_quality_) ||
      message->quality > 1.0 || message->inlier_count < min_observation_inliers_ ||
      message->source_points < message->inlier_count ||
      !std::isfinite(message->registration_error) || message->registration_error < 0.0 ||
      (max_registration_error_ >= 0.0 && message->registration_error > max_registration_error_)) {
      recordRejectionLocked("relocalization observation failed quality validation", true);
      publishStatusLocked();
      return;
    }

    const auto odom_to_base = interpolateOdomPose(
      odom_history_, stamp, history_boundary_tolerance_s_, maximum_interpolation_gap_s_);
    if (!odom_to_base || !last_odom_receive_time_ ||
        steadySecondsSince(*last_odom_receive_time_) > odom_timeout_s_) {
      recordRejectionLocked("relocalization observation is outside fresh odometry history", true);
      publishStatusLocked();
      return;
    }
    last_input_observation_stamp_ = stamp;
    last_input_observation_sequence_ = message->sequence;
    // Pending carries zero accepted-quality, but must pass evidence/frame/time gates.
    // Repeated pending messages never renew the episode deadline, and a closed
    // episode cannot be reopened by pending alone.
    if (pending) {
      if (recovery_episode_closed_) {
        recordRejectionLocked("relocalization recovery episode closed", true);
        publishStatusLocked();
        return;
      }
      // Incremental GICP confirmation while already map-locked must not demote
      // TRACKING/CONFIRMED to RELOCALIZING: that races DualMap ready and clears
      // EXECUTE within ~1s of every reference commit (recovery229).
      if (has_accepted_observation_ && !lost_latched_) {
        publishStatusLocked();
        return;
      }
      if (!recovery_until_) {
        recovery_until_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(observation_lost_timeout_s_));
      }
      status_ = LocalizationStatus::STATE_RELOCALIZING;
      status_message_ = "waiting for GICP confirmation";
      publishStatusLocked();
      return;
    }


    const tf2::Transform candidate_map_to_odom =
      poseToTransform(message->pose.pose) * odom_to_base->inverse();
    const std::optional<tf2::Transform> current_map_to_odom =
      has_map_to_odom_ ? std::optional<tf2::Transform>(map_to_odom_) : std::nullopt;
    const CorrectionUpdate correction = selectCorrectionUpdate(
      candidate_map_to_odom, current_map_to_odom, epoch_translation_threshold_,
      epoch_yaw_threshold_);
    {
      const bool lost = lost_latched_;
      const double max_xy = lost ? lost_max_correction_translation_ : max_correction_translation_;
      const double max_yaw = lost ? lost_max_correction_yaw_ : max_correction_yaw_;
      if (
        has_map_to_odom_ && ((max_xy > 0.0 && correction.delta.translation > max_xy) ||
                             (max_yaw > 0.0 && correction.delta.yaw > max_yaw))) {
        recordRejectionLocked(
          lost ? "LOST relocalization correction exceeded lost plausibility gate"
               : "relocalization correction exceeded plausibility gate",
          true);
        publishStatusLocked();
        return;
      }
    }

    const bool requires_hold = correction.advance_epoch ||
      status_ != LocalizationStatus::STATE_TRACKING;
    // 阈值内观测只刷新健康状态，不改 TF；一旦改 TF 就必须同步推进 epoch。
    map_to_odom_ = correction.map_to_odom;
    has_map_to_odom_ = true;
    has_accepted_observation_ = true;
    lost_latched_ = false;
    recovery_episode_closed_ = false;
    recovery_until_.reset();
    last_observation_stamp_ = stamp;
    last_observation_sequence_ = message->sequence;
    last_observation_receive_time_ = std::chrono::steady_clock::now();
    last_correction_translation_ = correction.delta.translation;
    last_correction_yaw_ = correction.delta.yaw;
    consecutive_rejections_ = 0;
    if (correction.advance_epoch) {
      ++epoch_;
    }
    if (requires_hold && (!relocalizing_until_ || correction.advance_epoch)) {
      relocalizing_until_ = std::chrono::steady_clock::now() +
                            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(relocalizing_hold_s_));
    }
    status_ = requires_hold ? LocalizationStatus::STATE_CONFIRMED : LocalizationStatus::STATE_TRACKING;
    updateHealthLocked();
    publishStatusLocked();
  }

  void onStatusTimer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateHealthLocked();
    if (publish_tf_ && has_map_to_odom_) {
      publishTfLocked();
    }
    publishStatusLocked();
  }

  void setStatus(std::uint8_t status, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status == LocalizationStatus::STATE_LOST) {
      enterLostLocked(message);
    } else {
      status_ = status;
      status_message_ = message;
    }
    publishStatusLocked();
  }

  void recordRejectionLocked(const std::string & message, bool count_failure)
  {
    if (count_failure) {
      ++consecutive_rejections_;
    }
    if (recovery_until_) {
      enterLostLocked(message);
    } else {
      updateHealthLocked();
    }
    status_message_ = message;
  }

  /// Steady silence starts at the first valid odometry, regardless of seed/TF.
  /// Pending and rejected observations never renew this accepted-evidence lease.
  double observationSilenceLocked() const
  {
    const std::optional<SteadyTime> & reference =
      last_observation_receive_time_ ? last_observation_receive_time_ : observation_clock_start_;
    if (!reference) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, steadySecondsSince(*reference));
  }

  void updateHealthLocked()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!last_odom_receive_time_) {
      if (steadySecondsSince(startup_time_) > odom_timeout_s_) {
        enterLostLocked("initial odometry input timed out");
      } else if (!lost_latched_) {
        status_ = LocalizationStatus::STATE_UNINITIALIZED;
        status_message_ = "waiting for odometry";
      }
      return;
    }
    if (steadySecondsSince(*last_odom_receive_time_) > odom_timeout_s_) {
      enterLostLocked("odometry input stale");
      return;
    }
    if (recovery_until_ && steady_now >= *recovery_until_) {
      enterLostLocked("relocalization recovery episode timed out");
      return;
    }
    if (lost_latched_) {
      status_ = recovery_until_ ? LocalizationStatus::STATE_RELOCALIZING
                                : LocalizationStatus::STATE_LOST;
      return;
    }
    const double silence = observationSilenceLocked();
    if (observation_lost_timeout_s_ > 0.0 && silence > observation_lost_timeout_s_) {
      enterLostLocked(has_accepted_observation_
        ? "accepted relocalization observation timed out"
        : "no accepted relocalization observation since first odometry");
      return;
    }
    if (recovery_until_) {
      status_ = LocalizationStatus::STATE_RELOCALIZING;
      return;
    }
    if (status_ == LocalizationStatus::STATE_DEGRADED ||
        (observation_degraded_timeout_s_ > 0.0 && silence > observation_degraded_timeout_s_) ||
        consecutive_rejections_ >= static_cast<std::uint32_t>(max_consecutive_rejections_)) {
      status_ = LocalizationStatus::STATE_DEGRADED;
      relocalizing_until_.reset();
      status_message_ = has_accepted_observation_ ? "accepted relocalization observation stale"
        : "awaiting first accepted relocalization observation";
      return;
    }
    if (!has_accepted_observation_) {
      status_ = LocalizationStatus::STATE_BOOTSTRAP;
      status_message_ = "valid odometry; awaiting confirmed map-frame evidence";
      return;
    }
    if (relocalizing_until_ && steady_now < *relocalizing_until_) {
      status_ = LocalizationStatus::STATE_CONFIRMED;
      status_message_ = "accepted relocalization; postcommit hold";
      return;
    }
    relocalizing_until_.reset();
    status_ = LocalizationStatus::STATE_TRACKING;
    status_message_ = "tracking";
  }

  void enterLostLocked(const std::string & message)
  {
    if (recovery_until_) {
      // Pending-only traffic must not arm another episode after this timeout.
      recovery_episode_closed_ = true;
    }
    if (!lost_latched_ || recovery_until_) {
      recovery_observation_floor_ = now();
    }
    lost_latched_ = true;
    recovery_until_.reset();
    relocalizing_until_.reset();
    status_ = LocalizationStatus::STATE_LOST;
    status_message_ = message;
  }

  void pruneHistoryLocked()
  {
    if (!latest_odom_stamp_) {
      return;
    }
    const rclcpp::Time cutoff =
      *latest_odom_stamp_ - rclcpp::Duration::from_seconds(history_duration_s_);
    const auto dropped = pruneOdomHistory(odom_history_, cutoff, max_odom_history_samples_);
    if (dropped > 0) {
      odom_history_count_drops_ += dropped;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry history sample cap reached: retained=%zu count_cap_drops=%llu",
        odom_history_.size(), static_cast<unsigned long long>(odom_history_count_drops_));
    }
  }

  void publishTfLocked()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now() + rclcpp::Duration::from_seconds(transform_future_offset_s_);
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_->sendTransform(transform);
  }

  void publishStatusLocked()
  {
    LocalizationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.state = status_;
    status.epoch = epoch_;
    status.observation_sequence = last_observation_sequence_.value_or(0);
    status.odometry_silence_sec = last_odom_receive_time_
                                    ? std::max(0.0, steadySecondsSince(*last_odom_receive_time_))
                                    : std::numeric_limits<double>::infinity();
    status.observation_silence_sec = observationSilenceLocked();
    if (last_observation_stamp_) {
      const auto stamp_ns = last_observation_stamp_->nanoseconds();
      status.last_observation_stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
      status.last_observation_stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
      status.observation_age_sec = std::max(0.0, (now() - *last_observation_stamp_).seconds());
    } else {
      status.observation_age_sec = std::numeric_limits<double>::infinity();
    }
    status.last_correction_translation = last_correction_translation_;
    status.last_correction_yaw = last_correction_yaw_;
    status.consecutive_rejections = consecutive_rejections_;
    status.message = status_message_;
    status_pub_->publish(status);
  }

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
  double observation_stamp_max_age_s_{1.0};
  double observation_stamp_max_future_s_{0.25};
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
  double max_correction_translation_{2.0};
  double max_correction_yaw_{1.0};
  double lost_max_correction_translation_{5.0};
  double lost_max_correction_yaw_{1.5};
  double min_observation_quality_{0.0};
  std::uint32_t min_observation_inliers_{1};
  double max_registration_error_{-1.0};

  std::mutex mutex_;
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
  tf2::Transform map_to_odom_{tf2::Transform::getIdentity()};
  bool has_map_to_odom_{false};
  bool has_accepted_observation_{false};
  std::uint64_t epoch_{0};
  double last_correction_translation_{0.0};
  double last_correction_yaw_{0.0};
  std::uint32_t consecutive_rejections_{0};
  std::uint8_t status_{LocalizationStatus::STATE_UNINITIALIZED};
  std::string status_message_{"waiting for odometry"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<RelocalizationObservation>::SharedPtr observation_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::LocalizationFusionNode)
