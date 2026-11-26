#include "ats_rog_map_node_impl.hpp"

namespace ats_rog_map
{

namespace
{


template<std::size_t Size>
std::array<double, Size> declareFixedArray(
  rclcpp::Node & node, const std::string & name, const std::array<double, Size> & defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, std::vector<double>(defaults.begin(), defaults.end()));
  if (values.size() != Size) {
    throw std::invalid_argument(name + " must contain exactly " + std::to_string(Size) + " values");
  }
  std::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

RogMapCoreParameters declareCoreParameters(rclcpp::Node & node)
{
  RogMapCoreParameters parameters;
  parameters.esdf_enable = node.declare_parameter<bool>("core.esdf.enable", parameters.esdf_enable);
  parameters.esdf_resolution = node.declare_parameter<double>("core.esdf.resolution", parameters.esdf_resolution);
  parameters.esdf_local_update_box = declareFixedArray(
    node, "core.esdf.local_update_box", parameters.esdf_local_update_box);
  parameters.esdf_update_interval_updates = node.declare_parameter<int>(
    "core.esdf.update_interval_updates", parameters.esdf_update_interval_updates);
  parameters.load_pcd_enable = node.declare_parameter<bool>("core.load_pcd_enable", parameters.load_pcd_enable);
  parameters.pcd_name = node.declare_parameter<std::string>("core.pcd_name", parameters.pcd_name);
  parameters.map_sliding_enable = node.declare_parameter<bool>(
    "core.map_sliding.enable", parameters.map_sliding_enable);
  parameters.map_sliding_threshold = node.declare_parameter<double>(
    "core.map_sliding.threshold", parameters.map_sliding_threshold);
  parameters.map_sliding_max_recenter_jump = node.declare_parameter<double>(
    "core.map_sliding.max_recenter_jump", parameters.map_sliding_max_recenter_jump);
  parameters.fix_map_origin = declareFixedArray(node, "core.fix_map_origin", parameters.fix_map_origin);
  parameters.frontier_extraction_enable = node.declare_parameter<bool>(
    "core.frontier_extraction_enable", parameters.frontier_extraction_enable);
  parameters.ros_callback_enable = node.declare_parameter<bool>(
    "core.ros_callback.enable", parameters.ros_callback_enable);
  parameters.ros_callback_cloud_topic = node.declare_parameter<std::string>(
    "core.ros_callback.cloud_topic", parameters.ros_callback_cloud_topic);
  parameters.ros_callback_odom_topic = node.declare_parameter<std::string>(
    "core.ros_callback.odom_topic", parameters.ros_callback_odom_topic);
  parameters.ros_callback_odom_timeout = node.declare_parameter<double>(
    "core.ros_callback.odom_timeout", parameters.ros_callback_odom_timeout);
  parameters.visualization_enable = node.declare_parameter<bool>(
    "core.visualization.enable", parameters.visualization_enable);
  parameters.visualization_publish_unknown = node.declare_parameter<bool>(
    "core.visualization.publish_unknown", parameters.visualization_publish_unknown);
  parameters.visualization_frame_id = node.declare_parameter<std::string>(
    "core.visualization.frame_id", parameters.visualization_frame_id);
  parameters.visualization_time_rate = node.declare_parameter<double>(
    "core.visualization.time_rate", parameters.visualization_time_rate);
  parameters.visualization_frame_rate = node.declare_parameter<int>(
    "core.visualization.frame_rate", parameters.visualization_frame_rate);
  parameters.visualization_range = declareFixedArray(
    node, "core.visualization.range", parameters.visualization_range);
  parameters.resolution = node.declare_parameter<double>("core.resolution", parameters.resolution);
  parameters.inflation_resolution = node.declare_parameter<double>(
    "core.inflation_resolution", parameters.inflation_resolution);
  parameters.unknown_inflation_enable = node.declare_parameter<bool>(
    "core.unknown_inflation.enable", parameters.unknown_inflation_enable);
  parameters.unknown_inflation_step = node.declare_parameter<int>(
    "core.unknown_inflation.step", parameters.unknown_inflation_step);
  parameters.inflation_step = node.declare_parameter<int>("core.inflation_step", parameters.inflation_step);
  parameters.intensity_threshold = node.declare_parameter<int>(
    "core.intensity_threshold", parameters.intensity_threshold);
  parameters.map_size = declareFixedArray(node, "core.map_size", parameters.map_size);
  parameters.point_filter_count = node.declare_parameter<int>(
    "core.point_filter_count", parameters.point_filter_count);
  parameters.raycasting_enable = node.declare_parameter<bool>(
    "core.raycasting.enable", parameters.raycasting_enable);
  parameters.raycasting_batch_update_size = node.declare_parameter<int>(
    "core.raycasting.batch_update_size", parameters.raycasting_batch_update_size);
  parameters.raycasting_unknown_threshold = node.declare_parameter<double>(
    "core.raycasting.unknown_threshold", parameters.raycasting_unknown_threshold);
  parameters.raycasting_p_hit = node.declare_parameter<double>(
    "core.raycasting.p_hit", parameters.raycasting_p_hit);
  parameters.raycasting_p_miss = node.declare_parameter<double>(
    "core.raycasting.p_miss", parameters.raycasting_p_miss);
  parameters.raycasting_p_min = node.declare_parameter<double>(
    "core.raycasting.p_min", parameters.raycasting_p_min);
  parameters.raycasting_p_max = node.declare_parameter<double>(
    "core.raycasting.p_max", parameters.raycasting_p_max);
  parameters.raycasting_p_occupied = node.declare_parameter<double>(
    "core.raycasting.p_occupied", parameters.raycasting_p_occupied);
  parameters.raycasting_p_free = node.declare_parameter<double>(
    "core.raycasting.p_free", parameters.raycasting_p_free);
  parameters.raycasting_range = declareFixedArray(
    node, "core.raycasting.range", parameters.raycasting_range);
  parameters.raycasting_local_update_box = declareFixedArray(
    node, "core.raycasting.local_update_box", parameters.raycasting_local_update_box);
  parameters.stale_decay_enable = node.declare_parameter<bool>(
    "core.raycasting.stale_decay.enable", parameters.stale_decay_enable);
  parameters.stale_decay_soft_ttl_updates = node.declare_parameter<int>(
    "core.raycasting.stale_decay.soft_ttl_updates", parameters.stale_decay_soft_ttl_updates);
  parameters.stale_decay_hard_ttl_updates = node.declare_parameter<int>(
    "core.raycasting.stale_decay.hard_ttl_updates", parameters.stale_decay_hard_ttl_updates);
  parameters.stale_decay_log_odds_step = node.declare_parameter<double>(
    "core.raycasting.stale_decay.log_odds_step", parameters.stale_decay_log_odds_step);
  parameters.stale_decay_sweep_interval_updates = node.declare_parameter<int>(
    "core.raycasting.stale_decay.sweep_interval_updates", parameters.stale_decay_sweep_interval_updates);
  parameters.stale_decay_local_update_box_only = node.declare_parameter<bool>(
    "core.raycasting.stale_decay.local_update_box_only", parameters.stale_decay_local_update_box_only);
  parameters.clear_clipped_endpoint = node.declare_parameter<bool>(
    "core.raycasting.clear_clipped_endpoint", parameters.clear_clipped_endpoint);
  parameters.virtual_ground_height = node.declare_parameter<double>(
    "core.virtual_ground_height", parameters.virtual_ground_height);
  parameters.virtual_ceil_height = node.declare_parameter<double>(
    "core.virtual_ceil_height", parameters.virtual_ceil_height);
  return parameters;
}

rog_map::Pose poseFromTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  return {
    rog_map::Vec3f(translation.x, translation.y, translation.z),
    super_utils::Quatf(rotation.w, rotation.x, rotation.y, rotation.z)};
}

}  // namespace

AtsRogMapNode::AtsRogMapNode()
: Node("ats_rog_map"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "gimbal_yaw_odom");
  sensor_frame_ = declare_parameter<std::string>("sensor_frame", "front_mid360");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/registered_scan");
  require_odom_identity_ = declare_parameter<bool>("require_odom_identity", false);
  identity_timeout_sec_ = declare_parameter<double>("identity_timeout_sec", 1.0);
  cloud_timeout_sec_ = std::max(0.0, declare_parameter<double>("cloud_timeout_sec", 0.5));
  odom_timeout_sec_ = std::max(0.0, declare_parameter<double>("odom_timeout_sec", 0.5));
  tf_timeout_sec_ = std::max(0.0, declare_parameter<double>("tf_timeout_sec", 0.1));
  const double health_publish_rate_hz = std::max(
    1.0, declare_parameter<double>("health_publish_rate_hz", 10.0));
  const int cloud_pending_poll_ms = static_cast<int>(std::max<int64_t>(
    1, declare_parameter<int>("cloud_pending_poll_ms", 1)));
  debug_rate_hz_ = std::max(0.0, declare_parameter<double>("debug_rate_hz", 2.0));
  map_update_trace_enabled_ = declare_parameter<bool>("map_update_trace_enabled", false);
  esdf_visualization_height_ = declare_parameter<double>("esdf_visualization_height", 0.15);
  debug_viz_topic_ = declare_parameter<std::string>("debug_viz_topic", "rog_map/viz");
  debug_viz_stride_ = std::max<int>(
    1, static_cast<int>(declare_parameter<int>("debug_viz_stride", 1)));
  debug_viz_include_unknown_ = declare_parameter<bool>("debug_viz_include_unknown", false);
  debug_bounds_topic_ = declare_parameter<std::string>("debug_bounds_topic", "rog_map/bounds");
  debug_bounds_show_labels_ = declare_parameter<bool>("debug_bounds_show_labels", true);
  self_filter_radius_ = std::max(0.0, declare_parameter<double>("self_filter_radius", 0.45));
  debug_qos_depth_ = std::max<int>(
    1, static_cast<int>(declare_parameter<int>("debug_qos_depth", 1)));
  input_qos_reliable_ = declare_parameter<bool>("input_qos_reliable", false);
  debug_qos_reliable_ = declare_parameter<bool>("debug_qos_reliable", false);
  // rog_map: this process also runs the planning-map owner (ats_rog_map_adapter).
  // rc_esdf: it does not, so a different owner can publish the planning grid.
  planning_grid_owner_ = declare_parameter<std::string>("planning_grid_owner", "rog_map");
  if (planning_grid_owner_ != "rog_map" && planning_grid_owner_ != "rc_esdf") {
    throw std::invalid_argument("planning_grid_owner must be 'rog_map' or 'rc_esdf'");
  }

  const auto core_parameters = declareCoreParameters(*this);
  intensity_required_ = core_parameters.intensity_threshold > 0;
  map_ = std::make_unique<RogMapEngine>(get_clock(), makeRogMapConfig(core_parameters));
  OdomLayerConfig layer_config;
  layer_config.origin_x = declare_parameter<double>("odom_layer.origin_x", layer_config.origin_x);
  layer_config.origin_y = declare_parameter<double>("odom_layer.origin_y", layer_config.origin_y);
  layer_config.resolution = declare_parameter<double>("odom_layer.resolution", layer_config.resolution);
  layer_config.cloud_resolution = layer_config.resolution;
  layer_config.width = declare_parameter<int>("odom_layer.width", layer_config.width);
  layer_config.height = declare_parameter<int>("odom_layer.height", layer_config.height);
  layer_config.static_promotion_seconds = declare_parameter<double>(
    "odom_layer.static_promotion_seconds", layer_config.static_promotion_seconds);
  layer_config.dynamic_hold_seconds = declare_parameter<double>(
    "odom_layer.dynamic_hold_seconds", layer_config.dynamic_hold_seconds);
  layer_config.max_frame_dt_seconds = declare_parameter<double>(
    "odom_layer.max_frame_dt_seconds", layer_config.max_frame_dt_seconds);
  layer_config.save_persistence_ratio = declare_parameter<double>(
    "odom_layer.save_persistence_ratio", layer_config.save_persistence_ratio);
  layer_config.cloud_clear_ray_radius = declare_parameter<double>(
    "odom_layer.cloud_clear_ray_radius", layer_config.cloud_clear_ray_radius);
  layer_config.cloud_clear_endpoint_margin = declare_parameter<double>(
    "odom_layer.cloud_clear_endpoint_margin", layer_config.cloud_clear_endpoint_margin);
  layer_config.tunnel_free_enabled = declare_parameter<bool>(
    "odom_layer.tunnel_free_enabled", layer_config.tunnel_free_enabled);
  layer_config.tunnel_free_confirm_frames = std::max(
    1, static_cast<int>(declare_parameter<int>(
      "odom_layer.tunnel_free_confirm_frames", layer_config.tunnel_free_confirm_frames)));
  odom_layer_ = std::make_unique<OdomLayerStore>(layer_config);
  GroundColumnConfig column_config;
  column_config.surface_max = declare_parameter<double>(
    "odom_layer.surface_max", column_config.surface_max);
  column_config.robot_height = declare_parameter<double>(
    "odom_layer.robot_height", column_config.robot_height);
  column_config.top_margin = declare_parameter<double>(
    "odom_layer.top_margin", column_config.top_margin);
  column_config.max_ground_slope = declare_parameter<double>(
    "odom_layer.max_ground_slope", column_config.max_ground_slope);
  column_config.plane_residual_max = declare_parameter<double>(
    "odom_layer.plane_residual_max", column_config.plane_residual_max);
  column_config.ceiling_plane_enabled = declare_parameter<bool>(
    "odom_layer.ceiling_plane_enabled", column_config.ceiling_plane_enabled);
  column_config.ceiling_plane_band = declare_parameter<double>(
    "odom_layer.ceiling_plane_band", column_config.ceiling_plane_band);
  column_config.floor_plane_enabled = declare_parameter<bool>(
    "odom_layer.floor_plane_enabled", column_config.floor_plane_enabled);
  column_config.floor_plane_band = declare_parameter<double>(
    "odom_layer.floor_plane_band", column_config.floor_plane_band);
  column_config.voxel_resolution = layer_config.resolution;
  column_config.tunnel_free_enabled = layer_config.tunnel_free_enabled;
  column_config.tunnel_free_confirm_frames = layer_config.tunnel_free_confirm_frames;
  ground_columns_ = std::make_unique<GroundColumnGrid>(
    layer_config.width, layer_config.height, layer_config.resolution,
    layer_config.origin_x, layer_config.origin_y, column_config);
  projection_follow_measured_ground_ = declare_parameter<bool>(
    "projection_follow_measured_ground", false);
  base_ground_offset_ = declare_parameter<double>("odom_layer.base_ground_offset", 0.0);
  layer_window_x_ = declare_parameter<double>("odom_layer.window_x", 12.0);
  layer_window_y_ = declare_parameter<double>("odom_layer.window_y", 12.0);
  layer_global_coarse_resolution_ = std::max(
    layer_config.resolution,
    declare_parameter<double>("odom_layer.global_coarse_resolution", layer_config.resolution));
  mapping_mode_ = declare_parameter<bool>("mapping_mode", false);
  if (!std::isfinite(base_ground_offset_) || !std::isfinite(layer_window_x_) ||
    !std::isfinite(layer_window_y_) || !std::isfinite(layer_global_coarse_resolution_) ||
    layer_window_x_ <= 0 || layer_window_y_ <= 0)
  {throw std::invalid_argument("invalid ground offset or computation window");}
  odom_layer_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/rog_map/odom_layer", rclcpp::QoS(1).reliable().transient_local());
  if (!intensity_required_) {
    RCLCPP_INFO(
      get_logger(),
      "ROGMap intensity filter disabled; XYZ-only clouds are accepted with unavailable intensity.");
  }

  odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  debug_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto input_qos = rclcpp::QoS(rclcpp::KeepLast(debug_qos_depth_));
  if (input_qos_reliable_) {
    input_qos.reliable();
  } else {
    input_qos.best_effort();
  }
  auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(debug_qos_depth_));
  if (debug_qos_reliable_) {
    debug_qos.reliable();
  } else {
    debug_qos.best_effort();
  }

  occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/occ", debug_qos);
  inflated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", debug_qos);
  unknown_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/unk", debug_qos);
  esdf_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf", debug_qos);
  viz_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_viz_topic_, debug_qos);
  bounds_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    debug_bounds_topic_, debug_qos);
  stale_pub_ = create_publisher<std_msgs::msg::Bool>("rog_map/stale", rclcpp::QoS(1).reliable());
  layer_updated_pub_ = create_publisher<std_msgs::msg::UInt64>(
    "rog_map/odom_layer_updated", rclcpp::QoS(1).reliable());
  save_service_ = create_service<ats_rog_map_interfaces::srv::SaveOdomMap>(
    "rog_map/save_map", std::bind(&AtsRogMapNode::saveMap, this,
      std::placeholders::_1, std::placeholders::_2));

  rclcpp::SubscriptionOptions odom_options;
  odom_options.callback_group = odom_callback_group_;
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, input_qos,
    std::bind(&AtsRogMapNode::odomCallback, this, std::placeholders::_1), odom_options);
  rclcpp::SubscriptionOptions cloud_options;
  cloud_options.callback_group = cloud_callback_group_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, input_qos,
    std::bind(&AtsRogMapNode::deferCloud, this, std::placeholders::_1), cloud_options);
  cloud_pending_timer_ = create_wall_timer(
    std::chrono::milliseconds(cloud_pending_poll_ms),
    std::bind(&AtsRogMapNode::tryProcessCloud, this), cloud_callback_group_);
  localization_sub_ = create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
    "/localization/status", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&AtsRogMapNode::onLocalizationStatus, this, std::placeholders::_1));

  const auto health_period = std::chrono::milliseconds(
    std::max(1, static_cast<int>(std::lround(1000.0 / health_publish_rate_hz))));
  health_timer_ = create_wall_timer(
    health_period, std::bind(&AtsRogMapNode::publishHealth, this), health_callback_group_);
  if (debug_rate_hz_ > 0.0) {
    const auto debug_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / debug_rate_hz_));
    debug_timer_ = create_wall_timer(
      std::max(debug_period, std::chrono::milliseconds(1)),
      std::bind(&AtsRogMapNode::publishDebug, this), debug_callback_group_);
  }

  RCLCPP_INFO(
    get_logger(),
    "ROGMap ready: odom='%s' cloud='%s' map='%s' base='%s' sensor='%s' core=ROS-parameters",
    odom_topic_.c_str(), cloud_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
    sensor_frame_.c_str());
}


void AtsRogMapNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const SteadyTime & trace_begin)
{
  std::uint64_t cloud_revision;
  {
    std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
    if (require_odom_identity_ &&
      !odom_epoch_guard_.accepts(rclcpp::Time(msg->header.stamp).nanoseconds())) {return;}
    cloud_revision = odom_epoch_guard_.revision();
  }
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_cloud_receive_time_ = trace_begin;
  }
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "ROGMap ignored a cloud without frame_id.");
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  geometry_msgs::msg::TransformStamped map_from_base;
  geometry_msgs::msg::TransformStamped map_from_sensor;
  geometry_msgs::msg::TransformStamped map_from_cloud;
  if (!lookupTransform(base_frame_, stamp, map_from_base) ||
    !lookupTransform(sensor_frame_, stamp, map_from_sensor) ||
    !lookupTransform(msg->header.frame_id, stamp, map_from_cloud))
  {
    return;
  }
  const auto trace_tf = std::chrono::steady_clock::now();

  auto decoded = decodePointCloudForRogMap(*msg, intensity_required_);
  if (decoded.status != PointCloudInputStatus::kAccepted) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ROGMap rejected point cloud: %s. No map update is committed; stale-map safety applies.",
      pointCloudInputStatusString(decoded.status));
    return;
  }

  rog_map::PointCloud input_cloud = std::move(decoded.points);
  rog_map::PointCloud cloud;
  cloud.reserve(input_cloud.size());
  tf2::Transform cloud_transform;
  tf2::fromMsg(map_from_cloud.transform, cloud_transform);
  const auto & base_translation = map_from_base.transform.translation;
  const double base_yaw = tf2::getYaw(map_from_base.transform.rotation);
  const double cos_by = std::cos(base_yaw);
  const double sin_by = std::sin(base_yaw);
  // Filter self-hits strictly inside the oriented robot chassis envelope (<= self_filter_radius_)
  // rather than an oversized circle, so 5 cm ground cells right in front of the chassis
  // (0.33..0.45 m) stay observable instead of forming an artificial unknown moat.
  const double self_box_half = std::min(self_filter_radius_, 0.34);
  for (auto point : input_cloud) {
    const tf2::Vector3 transformed = cloud_transform * tf2::Vector3(point.x, point.y, point.z);
    point.x = static_cast<float>(transformed.x());
    point.y = static_cast<float>(transformed.y());
    point.z = static_cast<float>(transformed.z());
    const double dx = point.x - base_translation.x;
    const double dy = point.y - base_translation.y;
    if (dx * dx + dy * dy < self_filter_radius_ * self_filter_radius_) {
      const double bx = cos_by * dx + sin_by * dy;
      const double by = -sin_by * dx + cos_by * dy;
      if (std::abs(bx) <= self_box_half && std::abs(by) <= self_box_half) {
        continue;
      }
    }
    cloud.push_back(point);
  }

  const auto trace_prepared = std::chrono::steady_clock::now();
  bool map_updated = false;
  std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
  const auto trace_epoch = std::chrono::steady_clock::now();
  if (cloud_revision != odom_epoch_guard_.revision() ||
    (require_odom_identity_ && !odom_epoch_guard_.accepts(stamp.nanoseconds()))) {return;}
  auto trace_map_locked = trace_epoch;
  const bool need_3d_prob_update = !mapping_mode_ ||
    occupied_pub_->get_subscription_count() > 0U ||
    inflated_pub_->get_subscription_count() > 0U ||
    unknown_pub_->get_subscription_count() > 0U ||
    esdf_pub_->get_subscription_count() > 0U ||
    viz_pub_->get_subscription_count() > 0U;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    trace_map_locked = std::chrono::steady_clock::now();
    if (need_3d_prob_update) {
      const bool need_3d_esdf = esdf_pub_->get_subscription_count() > 0U;
      map_updated = map_->update(
        cloud, poseFromTransform(map_from_base), poseFromTransform(map_from_sensor), need_3d_esdf);
      if (map_updated) {
        const auto & stats = map_->getLastRaycastDebugStats();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ROGMap update=%llu snapshot=%llu input=%d endpoints=%d hits=%d occupied=%d stale=%d",
          static_cast<unsigned long long>(stats.update_index),
          static_cast<unsigned long long>(map_->generation()), stats.input_points,
          stats.raycast_endpoint_points, stats.hit_endpoint_points, stats.occupied_touched_cells,
          stats.stale_decayed_cells);
      }
    } else {
      map_->advanceOdomLayerOnly(poseFromTransform(map_from_base));
      map_updated = true;
    }
    if (map_updated) {
      // The published grid is an immutable ROGMap snapshot assembled at
      // commit time.  Sensor header time remains the TF lookup key above;
      // using it as the grid stamp would make a healthy queued cloud appear
      // seconds older than terrain/slope inputs during projection.
      last_map_stamp_ = now();
      has_map_data_ = true;
    }
  }
  const auto trace_map_done = std::chrono::steady_clock::now();
  if (map_updated) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_map_update_time_ = std::chrono::steady_clock::now();
  }
  if (map_updated) {
    accumulateOdomLayer(cloud, map_from_base.transform.translation,
      map_from_sensor.transform.translation, stamp);
    std_msgs::msg::UInt64 update;
    update.data = map_->generation();
    layer_updated_pub_->publish(update);
  }
  if (map_update_trace_enabled_) {
    const auto trace_end = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
      "S3 cloud_commit stamp_ns=%lld received_ns=%lld points=%zu tf_ms=%.3f "
      "prepare_ms=%.3f epoch_wait_ms=%.3f map_wait_ms=%.3f map_update_ms=%.3f "
      "layer_ms=%.3f total_ms=%.3f",
      static_cast<long long>(stamp.nanoseconds()), static_cast<long long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(trace_begin.time_since_epoch()).count()),
      cloud.size(),
      1000.0 * std::chrono::duration<double>(trace_tf - trace_begin).count(),
      1000.0 * std::chrono::duration<double>(trace_prepared - trace_tf).count(),
      1000.0 * std::chrono::duration<double>(trace_epoch - trace_prepared).count(),
      1000.0 * std::chrono::duration<double>(trace_map_locked - trace_epoch).count(),
      1000.0 * std::chrono::duration<double>(trace_map_done - trace_map_locked).count(),
      1000.0 * std::chrono::duration<double>(trace_end - trace_map_done).count(),
      1000.0 * std::chrono::duration<double>(trace_end - trace_begin).count());
  }
}

sensor_msgs::msg::PointCloud2 AtsRogMapNode::makeCloud(
  const rog_map::vec_E<rog_map::Vec3f> & points, const rclcpp::Time & stamp) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto & point : points) {
    cloud.emplace_back(point.x(), point.y(), point.z());
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp = stamp;
  return message;
}

void AtsRogMapNode::collectEsdfDebugPoints(
  const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max,
  pcl::PointCloud<pcl::PointXYZI> & cloud) const
{
  cloud.clear();
  const double resolution = std::max(map_->getResolution(), 0.01);
  const float z = static_cast<float>(std::clamp(
    esdf_visualization_height_, static_cast<double>(box_min.z()), static_cast<double>(box_max.z())));
  const auto axis_count = [resolution](const float min_value, const float max_value) {
      return static_cast<std::size_t>(std::floor((max_value - min_value) / resolution + 1.0));
    };
  cloud.reserve(axis_count(box_min.x(), box_max.x()) * axis_count(box_min.y(), box_max.y()));
  for (double x = box_min.x(); x <= box_max.x(); x += resolution) {
    for (double y = box_min.y(); y <= box_max.y(); y += resolution) {
      const rog_map::Vec3f point(static_cast<float>(x), static_cast<float>(y), z);
      const double distance = map_->getESDFDistance(point);
      if (std::isfinite(distance)) {
        pcl::PointXYZI esdf_point;
        esdf_point.x = point.x();
        esdf_point.y = point.y();
        esdf_point.z = point.z();
        esdf_point.intensity = static_cast<float>(distance);
        cloud.push_back(esdf_point);
      }
    }
  }
}

sensor_msgs::msg::PointCloud2 AtsRogMapNode::makeEsdfCloud(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp = stamp;
  return message;
}

sensor_msgs::msg::PointCloud2 AtsRogMapNode::makeVoxelDebugCloud(
  const std::vector<rog_map::ProbMap::VoxelDebugCell> & cells,
  const rclcpp::Time & stamp) const
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(cells.size());
  for (const auto & cell : cells) {
    pcl::PointXYZRGB point;
    point.x = cell.center.x();
    point.y = cell.center.y();
    point.z = cell.center.z();
    const VoxelDebugRgb color = voxelDebugRgb(cell.type);
    point.r = color.red;
    point.g = color.green;
    point.b = color.blue;
    cloud.push_back(point);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp = stamp;
  return message;
}

bool AtsRogMapNode::validBounds(const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max)
{
  return box_min.array().isFinite().all() && box_max.array().isFinite().all() &&
         ((box_max - box_min).array() > 1e-6F).all();
}

void AtsRogMapNode::appendBoundsMarker(
  visualization_msgs::msg::MarkerArray & markers, const rog_map::Vec3f & box_min,
  const rog_map::Vec3f & box_max, const std::string & ns, const std::string & label,
  int id, float red, float green, float blue, float alpha, float line_width,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = validBounds(box_min, box_max) ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;

  const auto point = [](const rog_map::Vec3f & value) {
      geometry_msgs::msg::Point result;
      result.x = value.x();
      result.y = value.y();
      result.z = value.z();
      return result;
    };
  const std::array<geometry_msgs::msg::Point, 8> corners = {
    point(rog_map::Vec3f(box_min.x(), box_min.y(), box_min.z())),
    point(rog_map::Vec3f(box_max.x(), box_min.y(), box_min.z())),
    point(rog_map::Vec3f(box_max.x(), box_max.y(), box_min.z())),
    point(rog_map::Vec3f(box_min.x(), box_max.y(), box_min.z())),
    point(rog_map::Vec3f(box_min.x(), box_min.y(), box_max.z())),
    point(rog_map::Vec3f(box_max.x(), box_min.y(), box_max.z())),
    point(rog_map::Vec3f(box_max.x(), box_max.y(), box_max.z())),
    point(rog_map::Vec3f(box_min.x(), box_max.y(), box_max.z()))};
  constexpr std::array<std::array<std::size_t, 2>, 12> kEdges = {{
    {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
    {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
    {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
  if (marker.action == visualization_msgs::msg::Marker::ADD) {
    marker.points.reserve(2U * kEdges.size());
    for (const auto & edge : kEdges) {
      marker.points.push_back(corners[edge[0]]);
      marker.points.push_back(corners[edge[1]]);
    }
  }
  markers.markers.push_back(std::move(marker));

  visualization_msgs::msg::Marker text;
  text.header.frame_id = map_frame_;
  text.header.stamp = stamp;
  text.ns = ns + "_label";
  text.id = id + 100;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = (debug_bounds_show_labels_ && validBounds(box_min, box_max)) ?
    visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  text.pose.orientation.w = 1.0;
  text.pose.position.x = 0.5 * (box_min.x() + box_max.x());
  text.pose.position.y = 0.5 * (box_min.y() + box_max.y());
  text.pose.position.z = box_max.z() + 0.12;
  text.scale.z = 0.18;
  text.color.r = red;
  text.color.g = green;
  text.color.b = blue;
  text.color.a = alpha;
  text.text = label;
  markers.markers.push_back(std::move(text));
}

visualization_msgs::msg::MarkerArray AtsRogMapNode::makeBoundsMarkers(
  const rog_map::Vec3f & local_min, const rog_map::Vec3f & local_max,
  const rog_map::Vec3f & visualization_min, const rog_map::Vec3f & visualization_max,
  const rog_map::Vec3f & update_min, const rog_map::Vec3f & update_max, const double resolution,
  const rclcpp::Time & stamp, const bool show_visualization_range)
{
  visualization_msgs::msg::MarkerArray markers;
  const float line_width = static_cast<float>(std::max(0.02, 0.4 * resolution));
  appendBoundsMarker(
    markers, local_min, local_max, "rog_map_local_map", "Local Map Range", 0,
    1.0F, 0.50F, 0.0F, 0.95F, line_width, stamp);
  // The purple box only describes the /rog_map/viz voxel window. Without a
  // voxel subscriber it would sit on top of the green update box, so emit
  // DELETE (inverted box) to keep the bounds display to what is drawn.
  const rog_map::Vec3f hidden_min = rog_map::Vec3f::Constant(1.0);
  const rog_map::Vec3f hidden_max = rog_map::Vec3f::Constant(-1.0);
  appendBoundsMarker(
    markers, show_visualization_range ? visualization_min : hidden_min,
    show_visualization_range ? visualization_max : hidden_max,
    "rog_map_visualization", "Visualization Range", 1,
    0.50F, 0.0F, 1.0F, 0.90F, line_width, stamp);
  appendBoundsMarker(
    markers, update_min, update_max, "rog_map_local_update", "Raycast Update Range", 2,
    0.0F, 1.0F, 0.0F, 0.90F, line_width, stamp);
  return markers;
}

struct DebugSnapshot
{
  std::uint64_t source_generation{0U};
  rclcpp::Time map_stamp{0, 0, RCL_ROS_TIME};
  bool publish_occupied{false};
  bool publish_inflated{false};
  bool publish_unknown{false};
  bool publish_esdf{false};
  bool publish_viz{false};
  bool publish_bounds{false};
  rog_map::vec_E<rog_map::Vec3f> occupied;
  rog_map::vec_E<rog_map::Vec3f> inflated;
  rog_map::vec_E<rog_map::Vec3f> unknown;
  std::vector<rog_map::ProbMap::VoxelDebugCell> viz_cells;
  rog_map::ProbMap::VoxelDebugStats viz_stats;
  pcl::PointCloud<pcl::PointXYZI> esdf_points;
  visualization_msgs::msg::MarkerArray bounds_markers;
  double map_lock_wait_ms{0.0};
  double map_lock_hold_ms{0.0};
  double viz_collect_ms{0.0};
  double viz_serialize_ms{0.0};
  double viz_publish_ms{0.0};
};

void AtsRogMapNode::publishDebug()
{
  const auto debug_started = std::chrono::steady_clock::now();
  DebugSnapshot snapshot;
  snapshot.publish_occupied = occupied_pub_->get_subscription_count() > 0U;
  snapshot.publish_inflated = inflated_pub_->get_subscription_count() > 0U;
  snapshot.publish_unknown = unknown_pub_->get_subscription_count() > 0U;
  snapshot.publish_viz = shouldBuildDebugVisualization(viz_pub_->get_subscription_count());
  snapshot.publish_bounds = bounds_pub_->get_subscription_count() > 0U;
  if (
    !snapshot.publish_occupied && !snapshot.publish_inflated && !snapshot.publish_unknown &&
    !snapshot.publish_viz && !snapshot.publish_bounds && esdf_pub_->get_subscription_count() == 0U)
  {
    return;
  }

  const auto map_lock_wait_started = std::chrono::steady_clock::now();
  {
    std::unique_lock<std::mutex> lock(map_mutex_);
    snapshot.map_lock_wait_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - map_lock_wait_started).count();
    const auto map_lock_hold_started = std::chrono::steady_clock::now();
    if (!has_map_data_) {
      return;
    }
    snapshot.source_generation = map_->generation();
    snapshot.map_stamp = last_map_stamp_;
    snapshot.publish_esdf = esdf_pub_->get_subscription_count() > 0U && map_->hasESDF();

    const rog_map::Vec3f map_center = map_->getLocalMapOrigin();
    const rog_map::Vec3f half_map_size = 0.5F * map_->getLocalMapSize();
    rog_map::Vec3f local_box_min = map_center - half_map_size;
    rog_map::Vec3f local_box_max = map_center + half_map_size;
    map_->boundBoxByLocalMap(local_box_min, local_box_max);

    rog_map::Vec3f visualization_box_min = local_box_min;
    rog_map::Vec3f visualization_box_max = local_box_max;
    const auto config = map_->getMapConfig();
    if ((config.visualization_range.array() > 0.0F).all()) {
      const rog_map::Vec3f robot_position = map_->getRobotState().p;
      const rog_map::Vec3f half_visualization_range = 0.5F * config.visualization_range;
      visualization_box_min = robot_position - half_visualization_range;
      visualization_box_max = robot_position + half_visualization_range;
      map_->boundBoxByLocalMap(visualization_box_min, visualization_box_max);
    }

    rog_map::Vec3f update_box_min;
    rog_map::Vec3f update_box_max;
    map_->getRaycastLocalUpdateBox(update_box_min, update_box_max);
    map_->boundBoxByLocalMap(update_box_min, update_box_max);
    const rog_map::Vec3f box_min = visualization_box_min;
    const rog_map::Vec3f box_max = visualization_box_max;
    if (snapshot.publish_bounds) {
      snapshot.bounds_markers = makeBoundsMarkers(
        local_box_min, local_box_max, visualization_box_min, visualization_box_max,
        update_box_min, update_box_max, map_->getResolution(), snapshot.map_stamp,
        snapshot.publish_viz);
    }
    if (snapshot.publish_viz) {
      const auto viz_collect_started = std::chrono::steady_clock::now();
      map_->collectVoxelDebugInBox(
        box_min, box_max, snapshot.viz_cells, snapshot.viz_stats, debug_viz_stride_,
        debug_viz_include_unknown_);
      snapshot.viz_collect_ms = 1000.0 * std::chrono::duration<double>(
        std::chrono::steady_clock::now() - viz_collect_started).count();
    }
    if (snapshot.publish_occupied) {
      map_->boxSearch(box_min, box_max, super_utils::OCCUPIED, snapshot.occupied);
    }
    if (snapshot.publish_inflated) {
      map_->boxSearchInflate(box_min, box_max, super_utils::OCCUPIED, snapshot.inflated);
    }
    if (snapshot.publish_unknown) {
      map_->boxSearch(box_min, box_max, super_utils::UNKNOWN, snapshot.unknown);
    }
    if (snapshot.publish_esdf && map_->ensureCurrentEsdf()) {
      rog_map::Vec3f esdf_box_min;
      rog_map::Vec3f esdf_box_max;
      if (map_->getCurrentEsdfBounds(esdf_box_min, esdf_box_max)) {
        esdf_box_min = esdf_box_min.cwiseMax(box_min);
        esdf_box_max = esdf_box_max.cwiseMin(box_max);
        if (validBounds(esdf_box_min, esdf_box_max)) {
          collectEsdfDebugPoints(esdf_box_min, esdf_box_max, snapshot.esdf_points);
        }
      }
    }
    snapshot.map_lock_hold_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - map_lock_hold_started).count();
  }

  if (snapshot.publish_bounds) {
    bounds_pub_->publish(snapshot.bounds_markers);
  }
  if (snapshot.publish_viz) {
    const auto viz_serialize_started = std::chrono::steady_clock::now();
    const auto message = makeVoxelDebugCloud(snapshot.viz_cells, snapshot.map_stamp);
    snapshot.viz_serialize_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - viz_serialize_started).count();
    const auto viz_publish_started = std::chrono::steady_clock::now();
    viz_pub_->publish(message);
    snapshot.viz_publish_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - viz_publish_started).count();
  }
  if (snapshot.publish_occupied) {
    occupied_pub_->publish(makeCloud(snapshot.occupied, snapshot.map_stamp));
  }
  if (snapshot.publish_inflated) {
    inflated_pub_->publish(makeCloud(snapshot.inflated, snapshot.map_stamp));
  }
  if (snapshot.publish_unknown) {
    unknown_pub_->publish(makeCloud(snapshot.unknown, snapshot.map_stamp));
  }
  if (snapshot.publish_esdf && !snapshot.esdf_points.empty()) {
    esdf_pub_->publish(makeEsdfCloud(snapshot.esdf_points, snapshot.map_stamp));
  }
  const double debug_ms = 1000.0 * std::chrono::duration<double>(
    std::chrono::steady_clock::now() - debug_started).count();
  const double viz_build_ms =
    snapshot.viz_collect_ms + snapshot.viz_serialize_ms + snapshot.viz_publish_ms;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "P2 debug build generation=%llu occupied=%d inflated=%d unknown=%d esdf=%d viz=%d "
    "viz_cells=%zu viz_unknown=%d viz_build_ms=%.1f viz_collect_ms=%.1f "
    "viz_serialize_ms=%.1f viz_publish_ms=%.1f map_lock_wait_ms=%.1f "
    "map_lock_hold_ms=%.1f bounds=%d total_ms=%.1f",
    static_cast<unsigned long long>(snapshot.source_generation), snapshot.publish_occupied ? 1 : 0,
    snapshot.publish_inflated ? 1 : 0, snapshot.publish_unknown ? 1 : 0,
    snapshot.publish_esdf ? 1 : 0, snapshot.publish_viz ? 1 : 0, snapshot.viz_cells.size(),
    snapshot.viz_stats.unknown_cells, viz_build_ms, snapshot.viz_collect_ms,
    snapshot.viz_serialize_ms, snapshot.viz_publish_ms, snapshot.map_lock_wait_ms,
    snapshot.map_lock_hold_ms, snapshot.publish_bounds ? 1 : 0, debug_ms);
}


}  // namespace ats_rog_map
