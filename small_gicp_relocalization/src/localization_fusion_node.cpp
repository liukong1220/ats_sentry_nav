#include "localization_fusion_node_impl.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace small_gicp_relocalization
{

LocalizationFusionNode::LocalizationFusionNode(const rclcpp::NodeOptions & options)
: Node("localization_fusion", options)
{
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry");
  const bool use_identified_odometry =
    declare_parameter<bool>("use_identified_odometry", false);
  const auto identified_odom_topic = declare_parameter<std::string>(
    "identified_odom_topic", "/odometry_identified");
  const auto identity_monitor_topic = declare_parameter<std::string>(
    "identity_monitor_topic", "/aft_mapped_to_init_identified");
  if (use_identified_odometry && identity_monitor_topic == identified_odom_topic) {
    throw std::invalid_argument("identity monitor must use the unfiltered frontend topic");
  }
  const auto initial_samples = declare_parameter<int>("identity_initial_samples", 3);
  const auto retired_limit = declare_parameter<int>("identity_retired_limit", 5);
  identity_max_age_s_ = declare_parameter<double>("identity_max_age_s", 0.5);
  identity_future_tolerance_s_ = declare_parameter<double>("identity_future_tolerance_s", 5.0);
  if (initial_samples < 1 || retired_limit < 0 || !std::isfinite(identity_max_age_s_) ||
    identity_max_age_s_ <= 0.0 || !std::isfinite(identity_future_tolerance_s_) ||
    identity_future_tolerance_s_ < 0.0)
  {
    throw std::invalid_argument("invalid identity safety parameters");
  }
  odom_instance_tracker_ = OdomInstanceTracker(
    static_cast<std::size_t>(initial_samples), static_cast<std::size_t>(retired_limit), 1.0);
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
  // 连续修正带上界；0 关闭（保持原行为：超过 epoch 阈值即推进 epoch）。
  continuous_correction_max_translation_ = std::max(
    0.0, declare_parameter<double>("continuous_correction_max_translation", 0.0));
  continuous_correction_max_yaw_ =
    std::max(0.0, declare_parameter<double>("continuous_correction_max_yaw", 0.0));
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
  status_publish_rate_hz_ = std::max(
    1.0, declare_parameter<double>("status_publish_rate_hz", 10.0));
  degrade_on_out_of_order_odom_ =
    declare_parameter<bool>("degrade_on_out_of_order_odom", false);
  directional_fusion_config_.enable =
    declare_parameter<bool>("directional_fusion.enable", true);
  directional_fusion_config_.weak_axis_damping = std::clamp(
    declare_parameter<double>("directional_fusion.weak_axis_damping", 0.0), 0.0, 1.0);
  directional_fusion_config_.weak_yaw_damping = std::clamp(
    declare_parameter<double>("directional_fusion.weak_yaw_damping", 0.2), 0.0, 1.0);
  directional_fusion_config_.max_strong_nis = std::max(
    0.0, declare_parameter<double>("directional_fusion.max_strong_nis", 36.0));
  tracking_filter_config_.enable =
    declare_parameter<bool>("tracking_filter.enable", true);
  tracking_filter_config_.deadband_translation = std::max(
    0.0, declare_parameter<double>("tracking_filter.deadband_translation", 0.04));
  tracking_filter_config_.deadband_yaw = std::max(
    0.0, declare_parameter<double>("tracking_filter.deadband_yaw", 0.03));
  tracking_filter_config_.lpf_alpha_translation = std::clamp(
    declare_parameter<double>("tracking_filter.lpf_alpha_translation", 0.25), 0.01, 1.0);
  tracking_filter_config_.lpf_alpha_yaw = std::clamp(
    declare_parameter<double>("tracking_filter.lpf_alpha_yaw", 0.20), 0.01, 1.0);
  tracking_filter_config_.settle_translation_tolerance = std::max(
    1e-4, declare_parameter<double>("tracking_filter.settle_translation_tolerance", 0.001));
  tracking_filter_config_.settle_yaw_tolerance = std::max(
    1e-4, declare_parameter<double>("tracking_filter.settle_yaw_tolerance", 0.002));
  tracking_filter_config_.motion_consistency_base_m = std::max(
    0.0, declare_parameter<double>("tracking_filter.motion_consistency_base_m", 0.12));
  tracking_filter_config_.motion_consistency_ratio = std::max(
    0.0, declare_parameter<double>("tracking_filter.motion_consistency_ratio", 0.05));
  tracking_filter_config_.motion_consistency_base_yaw = std::max(
    0.0, declare_parameter<double>("tracking_filter.motion_consistency_base_yaw", 0.08));
  tracking_filter_config_.motion_jump_confirm_count = static_cast<int>(std::max<std::int64_t>(
    1, declare_parameter<std::int64_t>("tracking_filter.motion_jump_confirm_count", 3)));
  tracking_filter_config_.motion_jump_confirm_tolerance_m = std::max(
    0.005, declare_parameter<double>("tracking_filter.motion_jump_confirm_tolerance_m", 0.04));
  tracking_filter_config_.motion_jump_confirm_tolerance_yaw = std::max(
    0.005, declare_parameter<double>("tracking_filter.motion_jump_confirm_tolerance_yaw", 0.04));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  localization_pub_ =
    create_publisher<nav_msgs::msg::Odometry>(localization_topic_, rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<LocalizationStatus>(
    status_topic_, rclcpp::QoS(1).reliable().transient_local());
  if (use_identified_odometry) {
    identity_monitor_sub_ = create_subscription<
      ats_navigation_interfaces::msg::IdentifiedOdometry>(
      identity_monitor_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationFusionNode::onIdentityMonitor, this, std::placeholders::_1));
    identified_odom_sub_ = create_subscription<
      ats_navigation_interfaces::msg::IdentifiedOdometry>(
      identified_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationFusionNode::onIdentifiedOdometry, this, std::placeholders::_1));
  } else {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationFusionNode::onOdometry, this, std::placeholders::_1));
  }
  observation_sub_ = create_subscription<RelocalizationObservation>(
    observation_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&LocalizationFusionNode::onObservation, this, std::placeholders::_1));
  const auto status_period = std::chrono::milliseconds(
    std::max(1, static_cast<int>(std::lround(1000.0 / status_publish_rate_hz_))));
  status_timer_ = create_wall_timer(
    status_period, std::bind(&LocalizationFusionNode::onStatusTimer, this));

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
    "【定位融合就绪】里程计='%s' 定位='%s' 重定位观测='%s' 状态='%s'",
    odom_topic_.c_str(), localization_topic_.c_str(), observation_topic_.c_str(),
    status_topic_.c_str());
}

// Ids are steady_clock start nanoseconds on this host and boot; none may lie in
// the receiver's future beyond the configured tolerance.
std::uint64_t LocalizationFusionNode::instanceUpperBound() const
{
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto tolerance_ns = static_cast<std::int64_t>(identity_future_tolerance_s_ * 1e9);
  return static_cast<std::uint64_t>(std::max<std::int64_t>(0, now_ns + tolerance_ns));
}

void LocalizationFusionNode::onIdentityMonitor(
  const ats_navigation_interfaces::msg::IdentifiedOdometry::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (message->instance_id == 0 || message->header != message->odometry.header ||
    odom_instance_tracker_.instance() == 0 ||
    message->instance_id >= odom_instance_tracker_.instance())
  {
    return;
  }
  const double steady_now = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto result = odom_instance_tracker_.observe(
    message->instance_id, false, steady_now, instanceUpperBound());
  if (result == OdomInstanceTracker::Result::kLatched) {
    enterLostLocked("concurrent odometry instances; restart fusion required");
  } else {
    recordRejectionLocked("retired frontend instance", true);
  }
  publishStatusLocked();
}

void LocalizationFusionNode::onIdentifiedOdometry(
  const ats_navigation_interfaces::msg::IdentifiedOdometry::SharedPtr message)
{
  if (message->header != message->odometry.header) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & stamp = message->header.stamp;
    const bool valid_stamp = stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
      (stamp.sec != 0 || stamp.nanosec != 0);
    const double age = valid_stamp ? (now() - rclcpp::Time(stamp)).seconds() : -1.0;
    const bool fresh = valid_stamp && age >= 0.0 && age <= identity_max_age_s_;
    const double steady_now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto result = odom_instance_tracker_.observe(
      message->instance_id, fresh, steady_now, instanceUpperBound());
    using Result = OdomInstanceTracker::Result;
    if (result == Result::kLatched) {
      enterLostLocked("concurrent odometry instances; restart fusion required");
      publishStatusLocked();
      return;
    }
    if (result == Result::kRetired) {
      recordRejectionLocked("retired odometry instance", true);
      publishStatusLocked();
      return;
    }
    if (result == Result::kInvalid || result == Result::kPending) {
      return;
    }
    if (result == Result::kFirst || result == Result::kChanged) {
      odom_epoch_start_stamp_ = message->header.stamp;
    }
    if (result == Result::kChanged) {
      odom_history_.clear();
      latest_odom_stamp_.reset();
      last_odom_receive_time_.reset();
      last_input_observation_stamp_.reset();
      last_input_observation_sequence_.reset();
      last_observation_stamp_.reset();
      last_observation_sequence_.reset();
      last_observation_receive_time_.reset();
      observation_clock_start_.reset();
      has_map_to_odom_ = false;
      has_accepted_observation_ = false;
      last_accepted_odom_to_base_.reset();
      tracking_lpf_settling_ = false;
      pending_jump_candidate_.reset();
      pending_jump_count_ = 0;
      ++epoch_;
      enterLostLocked("odometry source instance changed");
      publishStatusLocked();
    }
  }
  onOdometry(std::make_shared<nav_msgs::msg::Odometry>(message->odometry));
}

void LocalizationFusionNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
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
    if (!last_odom_receive_time_ && initial_odom_timeout_only_) {
      // Loading sensors/initializing the frontend may outlive the initial
      // receive deadline. Its first validated input starts BOOTSTRAP, never
      // TRACKING. Established-input loss, malformed data and identity changes
      // still require accepted relocalization evidence to clear LOST.
      lost_latched_ = false;
      initial_odom_timeout_only_ = false;
    }
    if (latest_odom_stamp_ && stamp < *latest_odom_stamp_) {
      if (degrade_on_out_of_order_odom_) {
        recordRejectionLocked("dropped out-of-order odometry", true);
        if (!lost_latched_) {
          status_ = LocalizationStatus::STATE_DEGRADED;
          relocalizing_until_.reset();
        }
        publishStatusLocked();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Dropped isolated out-of-order odometry frame without degrading tracking.");
      }
      return;
    }
    if (latest_odom_stamp_ && stamp == *latest_odom_stamp_) {
      return;
    }
    const bool is_first_odom = !last_odom_receive_time_.has_value();
    const auto prev_status = status_;
    const auto prev_epoch = epoch_;
    const auto prev_odom_instance = odom_instance_tracker_.instance();
    const auto prev_odom_epoch = odom_instance_tracker_.epoch();
    const bool prev_latched = odom_instance_tracker_.latched();
    latest_odom_stamp_ = stamp;
    last_odom_receive_time_ = std::chrono::steady_clock::now();
    if (!observation_clock_start_) {
      observation_clock_start_ = last_odom_receive_time_;
    }
    odom_history_.push_back(OdomPoseSample{stamp, poseToTransform(localization.pose.pose)});
    pruneHistoryLocked();
    updateHealthLocked();
    if (is_first_odom || status_ != prev_status || epoch_ != prev_epoch ||
      odom_instance_tracker_.instance() != prev_odom_instance ||
      odom_instance_tracker_.epoch() != prev_odom_epoch ||
      odom_instance_tracker_.latched() != prev_latched)
    {
      publishStatusLocked();
    }
  }
  // /localization 保持连续 odom 状态；全局修正只通过唯一 map->odom 表达。
  localization_pub_->publish(localization);
}


}  // namespace small_gicp_relocalization

RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::LocalizationFusionNode)
