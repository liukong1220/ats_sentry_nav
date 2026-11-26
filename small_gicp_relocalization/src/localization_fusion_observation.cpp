#include "localization_fusion_node_impl.hpp"

namespace small_gicp_relocalization
{

void LocalizationFusionNode::onObservation(const RelocalizationObservation::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (get_parameter("use_identified_odometry").as_bool() && !observationMatchesFrontend(
      message->odom_instance_id, message->odom_epoch, odom_instance_tracker_.instance(),
      odom_instance_tracker_.epoch()))
  {
    recordRejectionLocked("relocalization observation from obsolete frontend epoch", true);
    publishStatusLocked();
    return;
  }
  if (odom_instance_tracker_.latched()) {
    enterLostLocked("concurrent odometry instances; restart fusion required");
    publishStatusLocked();
    return;
  }
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


  const tf2::Transform observed_map_to_base = poseToTransform(message->pose.pose);
  tf2::Transform candidate_map_to_odom = observed_map_to_base * odom_to_base->inverse();
  if (has_map_to_odom_ && !lost_latched_ && directional_fusion_config_.enable) {
    const DirectionalFusionResult dir_result = applyDirectionalDegeneracyFusion(
      map_to_odom_, observed_map_to_base, *odom_to_base, message->pose.covariance,
      message->degeneracy_mask, message->weak_direction_x, message->weak_direction_y,
      directional_fusion_config_);
    if (!dir_result.accepted) {
      recordRejectionLocked(dir_result.reject_reason, true);
      publishStatusLocked();
      return;
    }
    candidate_map_to_odom = dir_result.filtered_map_to_odom;
    if (dir_result.damped_weak_axis) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "【退化方向抑制】mask=%u weak_dir=(%.2f,%.2f) strong_nis=%.2f，仅融合可观测子空间修正。",
        static_cast<unsigned>(message->degeneracy_mask),
        message->weak_direction_x, message->weak_direction_y, dir_result.strong_nis);
    }
  }
  const std::optional<tf2::Transform> current_map_to_odom =
    has_map_to_odom_ ? std::optional<tf2::Transform>(map_to_odom_) : std::nullopt;
  const CorrectionDelta raw_delta = current_map_to_odom
    ? correctionDelta(candidate_map_to_odom, *current_map_to_odom)
    : CorrectionDelta{};
  {
    const bool lost = lost_latched_;
    const double max_xy = lost ? lost_max_correction_translation_ : max_correction_translation_;
    const double max_yaw = lost ? lost_max_correction_yaw_ : max_correction_yaw_;
    if (has_map_to_odom_ && !correctionWithinLimits(raw_delta, max_xy, max_yaw)) {
      recordRejectionLocked(
        lost ? "LOST relocalization correction exceeded lost plausibility gate"
             : "relocalization correction exceeded plausibility gate",
        true);
      publishStatusLocked();
      return;
    }
  }

  const bool in_steady_tracking =
    has_map_to_odom_ && has_accepted_observation_ && !lost_latched_ &&
    status_ == LocalizationStatus::STATE_TRACKING && tracking_filter_config_.enable;
  if (in_steady_tracking && tracking_filter_config_.motion_consistency_base_m > 0.0) {
    double odom_travel = 0.0;
    double odom_rot = 0.0;
    if (last_accepted_odom_to_base_) {
      const CorrectionDelta odom_step =
        correctionDelta(*odom_to_base, *last_accepted_odom_to_base_);
      odom_travel = odom_step.translation;
      odom_rot = odom_step.yaw;
    }
    const double allowed_trans =
      tracking_filter_config_.motion_consistency_base_m +
      tracking_filter_config_.motion_consistency_ratio * odom_travel;
    const double allowed_yaw =
      tracking_filter_config_.motion_consistency_base_yaw +
      tracking_filter_config_.motion_consistency_ratio * odom_rot;
    if (raw_delta.translation > allowed_trans || raw_delta.yaw > allowed_yaw) {
      if (pending_jump_candidate_) {
        const CorrectionDelta jump_diff =
          correctionDelta(candidate_map_to_odom, *pending_jump_candidate_);
        if (jump_diff.translation <= tracking_filter_config_.motion_jump_confirm_tolerance_m &&
          jump_diff.yaw <= tracking_filter_config_.motion_jump_confirm_tolerance_yaw)
        {
          ++pending_jump_count_;
          pending_jump_candidate_ = candidate_map_to_odom;
        } else {
          pending_jump_candidate_ = candidate_map_to_odom;
          pending_jump_count_ = 1;
        }
      } else {
        pending_jump_candidate_ = candidate_map_to_odom;
        pending_jump_count_ = 1;
      }
      if (pending_jump_count_ < tracking_filter_config_.motion_jump_confirm_count) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "【运动一致性门控】跟踪期异常跳变 待确认=%d/%d delta=(%.3f m, %.3f rad) "
          "allowed=(%.3f m, %.3f rad) odom_travel=%.2f m，暂不改写 map->odom。",
          pending_jump_count_, tracking_filter_config_.motion_jump_confirm_count,
          raw_delta.translation, raw_delta.yaw, allowed_trans, allowed_yaw, odom_travel);
        publishStatusLocked();
        return;
      }
    }
    pending_jump_candidate_.reset();
    pending_jump_count_ = 0;
  } else {
    pending_jump_candidate_.reset();
    pending_jump_count_ = 0;
  }

  CorrectionUpdate correction = selectCorrectionUpdate(
    candidate_map_to_odom, current_map_to_odom, epoch_translation_threshold_,
    epoch_yaw_threshold_, continuous_correction_max_translation_,
    continuous_correction_max_yaw_);
  bool damped_applied = false;
  const bool continuous_band_enabled =
    continuous_correction_max_translation_ > 0.0 && continuous_correction_max_yaw_ > 0.0;
  const bool within_continuous_band =
    continuous_band_enabled &&
    raw_delta.translation <= continuous_correction_max_translation_ &&
    raw_delta.yaw <= continuous_correction_max_yaw_;
  if (in_steady_tracking && within_continuous_band) {
    const bool beyond_deadband =
      (raw_delta.translation > 0.0 &&
       raw_delta.translation >= tracking_filter_config_.deadband_translation) ||
      (raw_delta.yaw > 0.0 &&
       raw_delta.yaw >= tracking_filter_config_.deadband_yaw);
    if (beyond_deadband) {
      tracking_lpf_settling_ = true;
    }
    if (tracking_lpf_settling_) {
      tf2::Transform filtered = interpolateTransform(
        map_to_odom_, candidate_map_to_odom,
        tracking_filter_config_.lpf_alpha_translation,
        tracking_filter_config_.lpf_alpha_yaw);
      const CorrectionDelta remaining = correctionDelta(candidate_map_to_odom, filtered);
      if (remaining.translation <= tracking_filter_config_.settle_translation_tolerance &&
        remaining.yaw <= tracking_filter_config_.settle_yaw_tolerance)
      {
        filtered = candidate_map_to_odom;
        tracking_lpf_settling_ = false;
      }
      correction.map_to_odom = filtered;
      correction.delta = correctionDelta(filtered, map_to_odom_);
      correction.apply = true;
      correction.continuous = true;
      correction.advance_epoch = false;
      damped_applied = true;
    } else {
      correction.map_to_odom = map_to_odom_;
      correction.apply = false;
      correction.continuous = false;
      correction.advance_epoch = false;
    }
  } else if (correction.advance_epoch) {
    tracking_lpf_settling_ = false;
  }

  const bool requires_hold = correction.advance_epoch ||
    status_ != LocalizationStatus::STATE_TRACKING;
  // 死区内观测只刷新健康状态不改 TF；连续带内采用指数低通阻尼吸收；超界跳变推进 epoch。
  map_to_odom_ = correction.map_to_odom;
  has_map_to_odom_ = true;
  has_accepted_observation_ = true;
  last_accepted_odom_to_base_ = *odom_to_base;
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
  } else if (correction.continuous) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "【连续%s修正】map->odom 步进=(%.3f m, %.3f rad) 原始新息=(%.3f m, %.3f rad)，epoch=%llu 保持。",
      damped_applied ? "阻尼" : "",
      correction.delta.translation, correction.delta.yaw,
      raw_delta.translation, raw_delta.yaw,
      static_cast<unsigned long long>(epoch_));
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

void LocalizationFusionNode::onStatusTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateHealthLocked();
  if (publish_tf_ && has_map_to_odom_) {
    publishTfLocked();
  }
  publishStatusLocked();
}

void LocalizationFusionNode::setStatus(std::uint8_t status, const std::string & message)
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

void LocalizationFusionNode::recordRejectionLocked(const std::string & message, bool count_failure)
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
double LocalizationFusionNode::observationSilenceLocked() const
{
  const std::optional<SteadyTime> & reference =
    last_observation_receive_time_ ? last_observation_receive_time_ : observation_clock_start_;
  if (!reference) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, steadySecondsSince(*reference));
}

void LocalizationFusionNode::updateHealthLocked()
{
  const auto steady_now = std::chrono::steady_clock::now();
  if (!last_odom_receive_time_) {
    if (steadySecondsSince(startup_time_) > odom_timeout_s_) {
      enterLostLocked("initial odometry input timed out", true);
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

void LocalizationFusionNode::enterLostLocked(const std::string & message, bool initial_odom_timeout)
{
  // An initial timeout cannot replace a different latched failure. Repeated
  // no-input ticks only preserve this marker until the first valid input.
  initial_odom_timeout_only_ = initial_odom_timeout &&
    (!lost_latched_ || initial_odom_timeout_only_);
  if (recovery_until_) {
    // Pending-only traffic must not arm another episode after this timeout.
    recovery_episode_closed_ = true;
  }
  if (!lost_latched_ || recovery_until_) {
    recovery_observation_floor_ = now();
  }
  lost_latched_ = true;
  tracking_lpf_settling_ = false;
  pending_jump_candidate_.reset();
  pending_jump_count_ = 0;
  recovery_until_.reset();
  relocalizing_until_.reset();
  status_ = LocalizationStatus::STATE_LOST;
  status_message_ = message;
}

void LocalizationFusionNode::pruneHistoryLocked()
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
      "【定位历史裁剪】里程计历史达到样本上限：保留=%zu 按数量丢弃=%llu",
      odom_history_.size(), static_cast<unsigned long long>(odom_history_count_drops_));
  }
}

void LocalizationFusionNode::publishTfLocked()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now() + rclcpp::Duration::from_seconds(transform_future_offset_s_);
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform = tf2::toMsg(map_to_odom_);
  tf_broadcaster_->sendTransform(transform);
}

void LocalizationFusionNode::publishStatusLocked()
{
  LocalizationStatus status;
  // Several state transitions can be committed from one callback while the
  // simulated ROS clock is between ticks.  The consumer intentionally has a
  // strict monotonic admission gate, so publishing the same `now()` stamp
  // twice would make a valid status stream look like a replay.  Preserve the
  // ROS clock domain and advance by one nanosecond only within that tick.
  const auto wall_stamp = now();
  const auto wall_stamp_ns = wall_stamp.nanoseconds();
  const auto status_stamp_ns = std::max(wall_stamp_ns, last_status_stamp_ns_ + 1);
  status.header.stamp = rclcpp::Time(status_stamp_ns, RCL_ROS_TIME);
  last_status_stamp_ns_ = status_stamp_ns;
  status.header.frame_id = map_frame_;
  status.state = status_;
  status.epoch = epoch_;
  status.odom_instance_id = odom_instance_tracker_.instance();
  status.odom_epoch = odom_instance_tracker_.epoch();
  status.odom_epoch_start_stamp = odom_epoch_start_stamp_;
  status.odom_identity_latched = odom_instance_tracker_.latched();
  status.observation_sequence = last_observation_sequence_.value_or(0);
  status.map_to_odom_valid = has_map_to_odom_;
  status.map_to_odom.header = status.header;
  status.map_to_odom.child_frame_id = odom_frame_;
  status.map_to_odom.transform = tf2::toMsg(map_to_odom_);
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

}  // namespace small_gicp_relocalization
