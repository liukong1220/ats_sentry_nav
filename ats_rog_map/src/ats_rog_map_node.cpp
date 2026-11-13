// Copyright 2026

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ats_rog_map/debug_viz.hpp"
#include "ats_rog_map/odom_layer.hpp"
#include "ats_rog_map/ground_column.hpp"
#include "ats_rog_map/odom_epoch_guard.hpp"
#include "ats_rog_map/odom_layer_integration.hpp"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_rog_map/point_cloud_input.hpp"
#include "ats_rog_map/rog_map_core_parameters.hpp"
#include "ats_rog_map/rog_map_engine.hpp"
#include "ats_rog_map/test_fault_authorization.hpp"
#include "ats_rog_map/unknown_projection_audit.hpp"
#include "ats_rog_map/ground_projection.hpp"
#include "ats_rog_map/planning/planning_map_publisher.hpp"
#include "ats_rog_map_interfaces/srv/save_odom_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_types.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rog_map/rog_map.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ats_rog_map
{

namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

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

class AtsRogMapNode final : public rclcpp::Node
{
public:
  AtsRogMapNode()
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
    // Startup-only authorization gate.  It is latched once here and never
    // re-read, so no live client can raise it.  Default false keeps nominal,
    // real-vehicle and default-launch runs from ever clearing the map.
    test_fault_injection_enabled_ =
      declare_parameter<bool>("enable_test_fault_injection", false);
    // Test-only edge-triggered source fixture.  It never changes default map
    // semantics and is intentionally distinct from an input-stale fault.
    declare_parameter<bool>("test_reset_to_unknown", false);
    RCLCPP_INFO(
      get_logger(), "P2 ROGMap test fault injection gate enable_test_fault_injection=%d",
      test_fault_injection_enabled_ ? 1 : 0);
    test_fault_parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        std::vector<std::string> names;
        names.reserve(parameters.size());
        for (const auto & parameter : parameters) {
          names.push_back(parameter.get_name());
        }
        const auto verdict = screenTestFaultParameters(
          test_fault_injection_enabled_, names, {"test_reset_to_unknown"});
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = verdict.accepted;
        result.reason = verdict.reason;
        if (!verdict.accepted) {
          RCLCPP_ERROR(
            get_logger(), "P2 ROGMap test fault request rejected: %s", verdict.reason.c_str());
        }
        return result;
      });

    const auto core_parameters = declareCoreParameters(*this);
    intensity_required_ = core_parameters.intensity_threshold > 0;
    map_ = std::make_unique<RogMapEngine>(get_clock(), makeRogMapConfig(core_parameters));
    OdomLayerConfig layer_config;
    layer_config.origin_x = declare_parameter<double>("odom_layer.origin_x", layer_config.origin_x);
    layer_config.origin_y = declare_parameter<double>("odom_layer.origin_y", layer_config.origin_y);
    layer_config.resolution = declare_parameter<double>("odom_layer.resolution", layer_config.resolution);
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
    ground_columns_ = std::make_unique<GroundColumnGrid>(
      layer_config.width, layer_config.height, layer_config.resolution,
      layer_config.origin_x, layer_config.origin_y, column_config);
    base_ground_offset_ = declare_parameter<double>("odom_layer.base_ground_offset", 0.0);
    layer_window_x_ = declare_parameter<double>("odom_layer.window_x", 12.0);
    layer_window_y_ = declare_parameter<double>("odom_layer.window_y", 12.0);
    if (!std::isfinite(base_ground_offset_) || !std::isfinite(layer_window_x_) ||
      !std::isfinite(layer_window_y_) || layer_window_x_ <= 0 || layer_window_y_ <= 0)
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
      std::chrono::milliseconds(1),
      std::bind(&AtsRogMapNode::tryProcessCloud, this), cloud_callback_group_);
    localization_sub_ = create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AtsRogMapNode::onLocalizationStatus, this, std::placeholders::_1));

    const auto health_period = std::chrono::milliseconds(100);
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

private:
  void onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr message)
  {
    if (!require_odom_identity_) {return;}
    std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
    const auto signal_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (signal_ns <= last_identity_signal_ns_) {return;}
    last_identity_signal_ns_ = signal_ns;
    last_identity_receive_ = std::chrono::steady_clock::now();
    has_accepted_map_transform_ = rclcpp::Time(message->last_observation_stamp).nanoseconds() > 0 &&
      !message->odom_identity_latched;
    const bool changed = odom_epoch_guard_.update(
      message->odom_instance_id, message->odom_epoch,
      rclcpp::Time(message->odom_epoch_start_stamp).nanoseconds(), signal_ns,
      message->odom_identity_latched);
    if (!changed) {return;}
    // epoch_mutex_ is always acquired before map/layer/input. A cloud decoded
    // outside it must re-check its revision before either store can commit.
    {
      std::lock_guard<std::mutex> map_lock(map_mutex_);
      map_->resetForOdomEpoch();
      has_map_data_ = false;
      last_map_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    {
      std::lock_guard<std::mutex> layer_lock(layer_mutex_);
      odom_layer_->clear();
      ground_columns_->clear();
      last_layer_stamp_seconds_ = 0.0;
      last_layer_source_stamp_ = builtin_interfaces::msg::Time{};
    }
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      last_map_update_time_.reset();
      last_cloud_receive_time_.reset();
      last_odom_receive_time_.reset();
    }
    RCLCPP_WARN(get_logger(),
      "S3 odom epoch reset instance=%llu epoch=%llu: revoked ROGMap, columns and persistent cloud",
      static_cast<unsigned long long>(odom_epoch_guard_.instance()),
      static_cast<unsigned long long>(odom_epoch_guard_.epoch()));
  }

  void saveMap(
    const ats_rog_map_interfaces::srv::SaveOdomMap::Request::SharedPtr request,
    ats_rog_map_interfaces::srv::SaveOdomMap::Response::SharedPtr response)
  {
    try {
      std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
      if (require_odom_identity_ &&
        (!odom_epoch_guard_.valid() || !has_accepted_map_transform_ ||
        !last_identity_receive_ || std::chrono::duration<double>(
          std::chrono::steady_clock::now() - *last_identity_receive_).count() > identity_timeout_sec_ ||
        inputHealth().map_update_stale || inputHealth().odom_stale))
      {
        response->message = "map save requires a healthy frontend epoch and an accepted map transform";
        return;
      }
      response->map_from_odom = tf_buffer_.lookupTransform(
        "map", map_frame_, tf2::TimePointZero);
      const auto transform_stamp = rclcpp::Time(response->map_from_odom.header.stamp);
      if (require_odom_identity_ && transform_stamp.nanoseconds() != 0 &&
        ((now() - transform_stamp).seconds() > identity_timeout_sec_ ||
        (now() - transform_stamp).seconds() < -0.2))
      {
        response->message = "map save transform is stale or from the future";
        return;
      }
      OdomLayerTransform transform;
      const auto & t = response->map_from_odom.transform;
      transform.x = t.translation.x;
      transform.y = t.translation.y;
      transform.z = t.translation.z;
      transform.yaw = tf2::getYaw(t.rotation);
      tf2::Quaternion rotation;
      tf2::fromMsg(t.rotation, rotation);
      tf2::Matrix3x3(rotation).getRPY(transform.roll, transform.pitch, transform.yaw);
      std::unique_ptr<OdomLayerStore> snapshot;
      {
        std::lock_guard<std::mutex> lock(layer_mutex_);
        snapshot = std::make_unique<OdomLayerStore>(*odom_layer_);
      }
      OdomLayerSaveStats stats;
      response->success = snapshot->save(request->output_prefix, "map", transform, &stats);
      response->occupied_cells = stats.occupied_cells;
      response->persistent_cells = stats.persistent_cells;
      response->persistent_voxels = stats.persistent_voxels;
      response->raster_occupied_cells = stats.raster_occupied_cells;
      response->message = response->success ? "saved transformed immutable snapshot" : "save failed";
      RCLCPP_INFO(
        get_logger(), "S3 save prefix='%s' success=%d occupied_cells=%zu persistent_cells=%zu "
        "persistent_voxels=%zu cells_pcd_points=%zu pcd_points=%zu raster_occupied_cells=%zu", request->output_prefix.c_str(),
        response->success, stats.occupied_cells, stats.persistent_cells,
        stats.persistent_voxels, stats.persistent_cells, stats.persistent_voxels,
        stats.raster_occupied_cells);
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "S3 save failed: %s", error.what());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
    if (require_odom_identity_ &&
      !odom_epoch_guard_.accepts(rclcpp::Time(message->header.stamp).nanoseconds())) {return;}
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_odom_receive_time_ = std::chrono::steady_clock::now();
  }

  bool lookupTransform(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, source_frame, stamp, rclcpp::Duration::from_seconds(0.0));
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ROGMap waits for TF %s -> %s at cloud time: %s",
        source_frame.c_str(), map_frame_.c_str(), exception.what());
      return false;
    }
  }

  void deferCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // This subscription and the retry timer share a mutually exclusive group.
    // Keep only the newest cloud while TF is pending, just as the input QoS
    // bounds queued work. TF listener callbacks remain free to fill the buffer.
    pending_cloud_ = msg;
    pending_cloud_receive_time_ = std::chrono::steady_clock::now();
    tryProcessCloud();
  }

  void tryProcessCloud()
  {
    if (!pending_cloud_) {return;}
    const auto received = pending_cloud_receive_time_;
    if (tf_timeout_sec_ > 0.0 && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - received).count() >= tf_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap discarded a cloud after its TF wait deadline.");
      pending_cloud_.reset();
      return;
    }
    const auto & frame = pending_cloud_->header.frame_id;
    const rclcpp::Time stamp(pending_cloud_->header.stamp);
    if (!frame.empty() &&
      (!tf_buffer_.canTransform(map_frame_, base_frame_, stamp) ||
      !tf_buffer_.canTransform(map_frame_, sensor_frame_, stamp) ||
      !tf_buffer_.canTransform(map_frame_, frame, stamp)))
    {
      if (tf_timeout_sec_ == 0.0) {pending_cloud_.reset();}
      return;
    }
    auto cloud = std::move(pending_cloud_);
    cloudCallback(cloud, received);
  }

  void cloudCallback(
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

    // The isolated all-unknown fixture keeps sensor/TF cadence healthy while
    // withholding every numeric occupancy observation.  It is an explicit
    // substitute for MuJoCo's zero-return LiDAR fixture; normal profiles can
    // never enter this branch because the startup authorization gate is false.
    const bool hold_all_unknown = test_fault_injection_enabled_ &&
      get_parameter("test_reset_to_unknown").as_bool();
    if (hold_all_unknown) {
      std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
      if (cloud_revision != odom_epoch_guard_.revision()) {return;}
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->resetToUnknownForTest();
        last_map_stamp_ = now();
        has_map_data_ = true;
        test_reset_to_unknown_active_ = true;
      }
      {
        std::lock_guard<std::mutex> lock(layer_mutex_);
        odom_layer_->clear();
        ground_columns_->clear();
        last_layer_stamp_seconds_ = 0.0;
        last_layer_source_stamp_ = msg->header.stamp;
      }
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        last_map_update_time_ = std::chrono::steady_clock::now();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "P2 all-unknown fixture holds numeric occupancy unknown while cloud and TF remain fresh.");
      return;
    }

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
    for (auto point : input_cloud) {
      const tf2::Vector3 transformed = cloud_transform * tf2::Vector3(point.x, point.y, point.z);
      point.x = static_cast<float>(transformed.x());
      point.y = static_cast<float>(transformed.y());
      point.z = static_cast<float>(transformed.z());
      if (std::hypot(point.x - base_translation.x, point.y - base_translation.y) <
        self_filter_radius_)
      {
        continue;
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
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      trace_map_locked = std::chrono::steady_clock::now();
      // Second, independent enforcement point: even if a value slipped in via a
      // parameter file, an unauthorized fixture stays ineffective here.
      test_reset_to_unknown_active_ = false;
      map_updated = map_->update(
        cloud, poseFromTransform(map_from_base), poseFromTransform(map_from_sensor));
      if (map_updated) {
        const auto & stats = map_->getLastRaycastDebugStats();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ROGMap update=%llu snapshot=%llu input=%d endpoints=%d hits=%d occupied=%d stale=%d",
          static_cast<unsigned long long>(stats.update_index),
          static_cast<unsigned long long>(map_->generation()), stats.input_points,
          stats.raycast_endpoint_points, stats.hit_endpoint_points, stats.occupied_touched_cells,
          stats.stale_decayed_cells);
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

  // The global odom layer has independent synchronization. Its obstacle hits
  // use ground-relative columns, including the headroom check, not base z.
  void accumulateOdomLayer(
    const rog_map::PointCloud & cloud, const geometry_msgs::msg::Vector3 & base,
    const geometry_msgs::msg::Vector3 & sensor, const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(layer_mutex_);
    const double stamp_seconds = stamp.seconds();
    const double frame_dt = last_layer_stamp_seconds_ > 0.0 ?
      std::max(0.0, stamp_seconds - last_layer_stamp_seconds_) : 0.0;
    std::vector<std::array<double, 3>> points;
    points.reserve(cloud.size());
    for (const auto & p : cloud) {points.push_back({p.x, p.y, p.z});}
    const auto stats = integrateOdomLayerFrame(*odom_layer_, *ground_columns_, points,
      {sensor.x, sensor.y, sensor.z}, {base.x, base.y, base.z - base_ground_offset_},
      stamp_seconds, frame_dt);
    if (map_update_trace_enabled_) {
      RCLCPP_INFO(get_logger(),
        "S3 layer_integrate prepare_ms=%.3f ray_ms=%.3f ground_ms=%.3f evidence_ms=%.3f",
        stats.prepare_ms, stats.ray_ms, stats.ground_ms, stats.evidence_ms);
    }
    layer_height_rejected_ += stats.non_obstacle;
    layer_out_of_range_ += stats.out_of_range;
    if (stats.out_of_range_sample) {
      const auto & p = *stats.out_of_range_sample;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "S3 global odom layer out_of_range=%llu frame_count=%zu sample_odom=(%.3f,%.3f,%.3f)",
        static_cast<unsigned long long>(layer_out_of_range_), stats.out_of_range, p[0], p[1], p[2]);
    }
    layer_center_x_ = base.x;
    layer_center_y_ = base.y;
    last_layer_stamp_seconds_ = stamp_seconds;
    last_layer_source_stamp_ = stamp;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom layer: column_non_obstacle=%llu out_of_range=%llu ray_cleared=%zu ray_voxels=%zu",
      static_cast<unsigned long long>(layer_height_rejected_),
      static_cast<unsigned long long>(layer_out_of_range_), stats.ray_cleared_cells, stats.ray_voxels);
  }

  nav_msgs::msg::OccupancyGrid layerGrid(
    double origin_x, double origin_y, std::uint32_t width, std::uint32_t height,
    double resolution) const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame_;
    grid.header.stamp = last_layer_source_stamp_;
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<std::size_t>(width) * height, -1);
    for (std::uint32_t y = 0; y < height; ++y) {
      for (std::uint32_t x = 0; x < width; ++x) {
        grid.data[static_cast<std::size_t>(y) * width + x] = odom_layer_->sampleFootprint(
          origin_x + x * resolution, origin_y + y * resolution,
          origin_x + (x + 1) * resolution, origin_y + (y + 1) * resolution);
      }
    }
    return grid;
  }

  void getOdomLayerProjection(
    float requested_resolution, GroundProjection & response)
  {
    const auto trace_begin = std::chrono::steady_clock::now();
    // epoch_mutex_ fences the map generation and the layer commit together.
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      response.generation = map_->generation();
      response.ready = has_map_data_ && map_->ensureCurrentEsdf();
    }
    const auto trace_esdf = std::chrono::steady_clock::now();
    const auto health = inputHealth();
    response.stale = health.map_update_stale || health.odom_stale;
    if (!response.ready) {return;}
    auto trace_grid = trace_esdf;
    auto trace_debug = trace_esdf;
    // The full-layer debug grid is copied once under layer_mutex_ and only while
    // someone subscribes; serialization and publish happen after the lock is
    // released so RViz cannot stall cloud integration (report 2.2.1 H).
    std::optional<nav_msgs::msg::OccupancyGrid> odom_layer_debug;
    {
      std::lock_guard<std::mutex> lock(layer_mutex_);
      const auto & c = odom_layer_->config();
      const double resolution = std::max(c.resolution, static_cast<double>(requested_resolution));
      response.occupancy_grid = layerGrid(
        std::floor((layer_center_x_ - 0.5 * layer_window_x_) / resolution) * resolution,
        std::floor((layer_center_y_ - 0.5 * layer_window_y_) / resolution) * resolution,
        static_cast<std::uint32_t>(std::ceil(layer_window_x_ / resolution)),
        static_cast<std::uint32_t>(std::ceil(layer_window_y_ / resolution)), resolution);
      trace_grid = std::chrono::steady_clock::now();
      if (odom_layer_pub_->get_subscription_count() > 0) {
        odom_layer_debug = layerGrid(c.origin_x, c.origin_y, c.width, c.height, c.resolution);
      }
    }
    if (odom_layer_debug) {
      odom_layer_pub_->publish(*odom_layer_debug);
    }
    trace_debug = std::chrono::steady_clock::now();
    ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
    esdf.updateGrid(response.occupancy_grid, 50, true);
    std::vector<double> field;
    if (!esdf.copySignedDistanceField(field)) {response.ready = false; return;}
    const auto & grid = response.occupancy_grid;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    response.signed_distance.resize(field.size(), nan);
    response.gradient_x.resize(field.size(), nan);
    response.gradient_y.resize(field.size(), nan);
    for (std::size_t i = 0; i < field.size(); ++i) {
      if (grid.data[i] < 0) {continue;}
      response.signed_distance[i] = static_cast<float>(field[i]);
      const auto gradient = esdf.getGradient(
        grid.info.origin.position.x + (i % grid.info.width + 0.5) * grid.info.resolution,
        grid.info.origin.position.y + (i / grid.info.width + 0.5) * grid.info.resolution);
      response.gradient_x[i] = static_cast<float>(gradient.x());
      response.gradient_y[i] = static_cast<float>(gradient.y());
    }
    response.odom_layer_included = true;
    if (map_update_trace_enabled_) {
      const auto trace_end = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(),
        "S3 layer_projection generation=%llu esdf3d_ms=%.3f grid_ms=%.3f debug_ms=%.3f "
        "esdf2d_ms=%.3f total_ms=%.3f",
        static_cast<unsigned long long>(response.generation),
        1000.0 * std::chrono::duration<double>(trace_esdf - trace_begin).count(),
        1000.0 * std::chrono::duration<double>(trace_grid - trace_esdf).count(),
        1000.0 * std::chrono::duration<double>(trace_debug - trace_grid).count(),
        1000.0 * std::chrono::duration<double>(trace_end - trace_debug).count(),
        1000.0 * std::chrono::duration<double>(trace_end - trace_begin).count());
    }
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
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

  void collectEsdfDebugPoints(
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

  sensor_msgs::msg::PointCloud2 makeEsdfCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud, const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    return message;
  }

  sensor_msgs::msg::PointCloud2 makeVoxelDebugCloud(
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

  static bool validBounds(const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max)
  {
    return box_min.array().isFinite().all() && box_max.array().isFinite().all() &&
           ((box_max - box_min).array() > 1e-6F).all();
  }

  void appendBoundsMarker(
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

  visualization_msgs::msg::MarkerArray makeBoundsMarkers(
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

  void publishDebug()
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

  void publishHealth()
  {
    bool map_update_stale = true;
    bool odom_stale = true;
    bool raw_cloud_stale = true;
    getHealthStaleness(map_update_stale, odom_stale, raw_cloud_stale);
    std_msgs::msg::Bool message;
    message.data = map_update_stale || odom_stale;
    stale_pub_->publish(message);
    if (message.data) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap stale: map_update=%s odom=%s raw_cloud=%s",
        map_update_stale ? "true" : "false", odom_stale ? "true" : "false",
        raw_cloud_stale ? "true" : "false");
    }
  }

  void getHealthStaleness(
    bool & map_update_stale, bool & odom_stale, bool & raw_cloud_stale)
  {
    const InputHealth health = inputHealth();
    map_update_stale = health.map_update_stale;
    odom_stale = health.odom_stale;
    raw_cloud_stale = health.raw_cloud_stale;
  }

  struct InputHealth
  {
    double map_update_age_sec{std::numeric_limits<double>::infinity()};
    double odom_age_sec{std::numeric_limits<double>::infinity()};
    double cloud_age_sec{std::numeric_limits<double>::infinity()};
    bool map_update_stale{true};
    bool odom_stale{true};
    bool raw_cloud_stale{true};
  };

  InputHealth inputHealth()
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    InputHealth health;
    if (last_cloud_receive_time_) {
      health.cloud_age_sec =
        std::chrono::duration<double>(now - *last_cloud_receive_time_).count();
    }
    if (last_map_update_time_) {
      health.map_update_age_sec =
        std::chrono::duration<double>(now - *last_map_update_time_).count();
    }
    if (last_odom_receive_time_) {
      health.odom_age_sec =
        std::chrono::duration<double>(now - *last_odom_receive_time_).count();
    }
    health.raw_cloud_stale = health.cloud_age_sec > cloud_timeout_sec_;
    health.map_update_stale = health.map_update_age_sec > cloud_timeout_sec_;
    health.odom_stale = health.odom_age_sec > odom_timeout_sec_;
    return health;
  }

public:
  /// Computes one ground projection for the in-process planning publisher.
  /// Calls must not overlap; the publisher runs them on one worker thread.
  bool ownsPlanningGrid() const {return planning_grid_owner_ == "rog_map";}

  void getGroundProjection(const GroundProjectionRequest & request_in, GroundProjection & response_out)
  {
    const GroundProjectionRequest * const request = &request_in;
    GroundProjection * const response = &response_out;
    const auto trace_begin = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
    const auto trace_epoch = std::chrono::steady_clock::now();
    response->odom_instance_id = odom_epoch_guard_.instance();
    response->odom_epoch = odom_epoch_guard_.epoch();
    if (require_odom_identity_ && (!odom_epoch_guard_.valid() || !last_identity_receive_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
      *last_identity_receive_).count() > identity_timeout_sec_))
    {response->stale = true; return;}
    if ((request->expected_odom_instance_id != 0 &&
      request->expected_odom_instance_id != response->odom_instance_id) ||
      (request->expected_odom_epoch != 0 && request->expected_odom_epoch != response->odom_epoch))
    {return;}
    if (request->use_odom_layer) {
      getOdomLayerProjection(request->resolution, *response);
      if (map_update_trace_enabled_) {
        RCLCPP_INFO(get_logger(), "S3 projection_dispatch epoch_wait_ms=%.3f total_ms=%.3f",
          1000.0 * std::chrono::duration<double>(trace_epoch - trace_begin).count(),
          1000.0 * std::chrono::duration<double>(
            std::chrono::steady_clock::now() - trace_begin).count());
      }
      return;
    }
    const auto projection_started = std::chrono::steady_clock::now();
    const rclcpp::Time projection_start_stamp = now();
    const std::uint64_t request_sequence = ++projection_request_sequence_;
    const InputHealth health_at_start = inputHealth();
    double map_lock_wait_ms = 0.0;
    double esdf_refresh_ms = 0.0;
    double sample_ms = 0.0;
    // sample_ms 内部再拆成两个查询族，用来区分“占用类型查询”与“ESDF 距离查询”
    // 各自的成本。每个 z 步只额外读取 3 次 steady_clock（约 20 ns/次），在
    // 数十万次迭代下总开销 <1%，不足以改变归因结论。
    double grid_type_query_ms = 0.0;
    double esdf_query_ms = 0.0;
    std::uint64_t grid_type_queries = 0;
    std::uint64_t esdf_queries = 0;
    double gradient_ms = 0.0;
    // serialize_ms 是 response payload 的构建/分配开销（栅格元数据 + occupancy
    // 与三个数值数组的 assign）。DDS 线上序列化发生在 rclcpp 内部，本节点无法
    // 直接测量，因此该项只声明为 payload 构建，不冒充线上序列化耗时。
    double serialize_ms = 0.0;
    // lock_hold_ms 是持有 map_mutex_ 的时长；与 map_lock_wait_ms 一起区分
    // "等锁" 与 "占锁"，避免把 contention 与计算混为一谈。锁在整段计算中持续
    // 持有，故在每个记录点按当前时刻刷新。
    double lock_hold_ms = 0.0;
    std::optional<std::chrono::steady_clock::time_point> map_lock_acquired_at;
    bool map_lock_held = false;
    response->stale = health_at_start.map_update_stale || health_at_start.odom_stale;
    RCLCPP_INFO(
      get_logger(),
      "P2 projection begin request=%llu start_ns=%lld map_age=%.3f odom_age=%.3f cloud_age=%.3f "
      "map_stale=%d odom_stale=%d cloud_stale=%d",
      static_cast<unsigned long long>(request_sequence),
      static_cast<long long>(projection_start_stamp.nanoseconds()),
      health_at_start.map_update_age_sec, health_at_start.odom_age_sec,
      health_at_start.cloud_age_sec, health_at_start.map_update_stale ? 1 : 0,
      health_at_start.odom_stale ? 1 : 0, health_at_start.raw_cloud_stale ? 1 : 0);
    const auto log_projection_end = [&]() {
        if (map_lock_held && map_lock_acquired_at.has_value()) {
          lock_hold_ms = 1000.0 * std::chrono::duration<double>(
            std::chrono::steady_clock::now() - *map_lock_acquired_at).count();
        }
        const InputHealth health_at_end = inputHealth();
        const rclcpp::Time projection_end_stamp = now();
        const double projection_ms = 1000.0 * std::chrono::duration<double>(
          std::chrono::steady_clock::now() - projection_started).count();
        RCLCPP_INFO(
          get_logger(),
          "P2 projection end request=%llu start_ns=%lld end_ns=%lld source_generation=%llu "
          "source_stamp_ns=%lld ready=%d stale=%d compute_ms=%.1f "
          "map_lock_wait_ms=%.1f map_lock_hold_ms=%.1f esdf_refresh_ms=%.1f sample_ms=%.1f "
          "grid_type_query_ms=%.1f esdf_query_ms=%.1f grid_type_queries=%llu esdf_queries=%llu "
          "gradient_ms=%.1f serialize_ms=%.1f total_ms=%.1f accounted_ms=%.1f "
          "unaccounted_ms=%.1f "
          "map_age_start=%.3f map_age_end=%.3f odom_age_start=%.3f odom_age_end=%.3f "
          "cloud_age_start=%.3f cloud_age_end=%.3f",
          static_cast<unsigned long long>(request_sequence),
          static_cast<long long>(projection_start_stamp.nanoseconds()),
          static_cast<long long>(projection_end_stamp.nanoseconds()),
          static_cast<unsigned long long>(response->generation),
          static_cast<long long>(rclcpp::Time(response->occupancy_grid.header.stamp).nanoseconds()),
          response->ready ? 1 : 0, response->stale ? 1 : 0, projection_ms,
          map_lock_wait_ms, lock_hold_ms, esdf_refresh_ms, sample_ms,
          grid_type_query_ms, esdf_query_ms,
          static_cast<unsigned long long>(grid_type_queries),
          static_cast<unsigned long long>(esdf_queries), gradient_ms,
          serialize_ms, projection_ms,
          map_lock_wait_ms + esdf_refresh_ms + sample_ms + gradient_ms + serialize_ms,
          projection_ms - (map_lock_wait_ms + esdf_refresh_ms + sample_ms + gradient_ms +
          serialize_ms),
          health_at_start.map_update_age_sec, health_at_end.map_update_age_sec,
          health_at_start.odom_age_sec, health_at_end.odom_age_sec,
          health_at_start.cloud_age_sec, health_at_end.cloud_age_sec);
      };

    const auto map_lock_wait_started = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(map_mutex_);
    map_lock_acquired_at = std::chrono::steady_clock::now();
    map_lock_held = true;
    map_lock_wait_ms = 1000.0 * std::chrono::duration<double>(
      *map_lock_acquired_at - map_lock_wait_started).count();
    response->generation = map_->generation();
    const bool publish_unknown_projection_audit = test_fault_injection_enabled_ &&
      test_reset_to_unknown_active_ && unknown_pub_->get_subscription_count() > 0U;
    const auto esdf_refresh_started = std::chrono::steady_clock::now();
    response->ready = has_map_data_ && map_->ensureCurrentEsdf();
    esdf_refresh_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - esdf_refresh_started).count();
    if (!response->ready) {
      log_projection_end();
      return;
    }
    rog_map::Vec3f esdf_box_min;
    rog_map::Vec3f esdf_box_max;
    if (!map_->getCurrentEsdfBounds(esdf_box_min, esdf_box_max)) {
      response->ready = false;
      log_projection_end();
      return;
    }

    const rog_map::Vec3f map_center = map_->getLocalMapOrigin();
    const rog_map::Vec3f half_map_size = 0.5F * map_->getLocalMapSize();
    rog_map::Vec3f box_min = map_center - half_map_size;
    rog_map::Vec3f box_max = map_center + half_map_size;
    map_->boundBoxByLocalMap(box_min, box_max);
    const double resolution = std::max(
      map_->getResolution(), request->resolution > 0.0F ?
      static_cast<double>(request->resolution) : map_->getResolution());
    const double z_min = std::max(
      static_cast<double>(box_min.z()),
      std::min(static_cast<double>(request->min_height), static_cast<double>(request->max_height)));
    const double z_max = std::min(
      static_cast<double>(box_max.z()),
      std::max(static_cast<double>(request->min_height), static_cast<double>(request->max_height)));
    if (z_max < z_min) {
      response->ready = false;
      log_projection_end();
      return;
    }

    const auto serialize_started = std::chrono::steady_clock::now();
    auto & grid = response->occupancy_grid;
    grid.header.frame_id = map_frame_;
    grid.header.stamp = last_map_stamp_;
    grid.info.map_load_time = last_map_stamp_;
    grid.info.resolution = resolution;
    grid.info.width = static_cast<std::uint32_t>(std::ceil(
      (static_cast<double>(box_max.x()) - static_cast<double>(box_min.x())) / resolution));
    grid.info.height = static_cast<std::uint32_t>(std::ceil(
      (static_cast<double>(box_max.y()) - static_cast<double>(box_min.y())) / resolution));
    grid.info.origin.position.x = box_min.x();
    grid.info.origin.position.y = box_min.y();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const std::size_t cell_count =
      static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    grid.data.assign(cell_count, -1);
    response->signed_distance.assign(cell_count, nan);
    response->gradient_x.assign(cell_count, nan);
    response->gradient_y.assign(cell_count, nan);
    serialize_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - serialize_started).count();

    const double z_step = std::max(map_->getResolution(), 0.01);
    const double esdf_distance_limit = (esdf_box_max - esdf_box_min).norm() + resolution;
    const auto inside_esdf_box = [&esdf_box_min, &esdf_box_max](const rog_map::Vec3f & point) {
        constexpr double tolerance = 1e-6;
        return (point.array() >= (esdf_box_min.array() - tolerance)).all() &&
               (point.array() <= (esdf_box_max.array() + tolerance)).all();
      };
    const auto sample_started = std::chrono::steady_clock::now();
    for (std::uint32_t my = 0; my < grid.info.height; ++my) {
      for (std::uint32_t mx = 0; mx < grid.info.width; ++mx) {
        const std::size_t index =
          static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) + mx;
        const float x = static_cast<float>(
          grid.info.origin.position.x + (static_cast<double>(mx) + 0.5) * resolution);
        const float y = static_cast<float>(
          grid.info.origin.position.y + (static_cast<double>(my) + 0.5) * resolution);
        bool occupied = false;
        bool known_free = false;
        double minimum_distance = std::numeric_limits<double>::infinity();
        for (double z = z_min + 0.5 * z_step; z <= z_max + 1e-6; z += z_step) {
          const rog_map::Vec3f point(x, y, static_cast<float>(std::min(z, z_max)));
          // Downstream JPS and footprint gates apply their own metric clearance on this snapshot.
          // Project raw probability occupancy here to avoid double-inflating the robot start cell.
          const auto grid_type_started = std::chrono::steady_clock::now();
          const rog_map::GridType cell_type = map_->getGridType(point);
          const auto grid_type_finished = std::chrono::steady_clock::now();
          grid_type_query_ms += 1000.0 * std::chrono::duration<double>(
            grid_type_finished - grid_type_started).count();
          ++grid_type_queries;
          occupied = occupied || cell_type == super_utils::OCCUPIED;
          known_free = known_free || cell_type == super_utils::KNOWN_FREE;
          if (inside_esdf_box(point) &&
            (cell_type == super_utils::OCCUPIED || cell_type == super_utils::KNOWN_FREE))
          {
            const auto esdf_query_started = std::chrono::steady_clock::now();
            const double distance = map_->getESDFDistance(point);
            esdf_query_ms += 1000.0 * std::chrono::duration<double>(
              std::chrono::steady_clock::now() - esdf_query_started).count();
            ++esdf_queries;
            if (std::isfinite(distance) && std::abs(distance) <= esdf_distance_limit) {
              minimum_distance = std::min(minimum_distance, std::abs(distance));
            }
          }
        }
        if (occupied) {
          grid.data[index] = 100;
          if (std::isfinite(minimum_distance)) {
            response->signed_distance[index] = static_cast<float>(
              -std::max(0.5 * resolution, minimum_distance));
          }
        } else if (known_free) {
          grid.data[index] = 0;
          if (std::isfinite(minimum_distance)) {
            response->signed_distance[index] = static_cast<float>(minimum_distance);
          }
        }
      }
    }
    sample_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - sample_started).count();

    // Implicit unknown cells cannot be enumerated through ProbMap::boxSearch().  During the
    // explicitly authorized reset fixture, publish an audit-only cloud from the authoritative
    // numerical response after it has proved strictly all-unknown.  It is never a safety input.
    rog_map::vec_E<rog_map::Vec3f> unknown_projection_audit;
    if (publish_unknown_projection_audit) {
      unknown_projection_audit = makeAllUnknownProjectionAudit(grid);
    }

    const auto index_of = [&grid](std::uint32_t x, std::uint32_t y) {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.info.width) + x;
      };
    const auto gradient_started = std::chrono::steady_clock::now();
    for (std::uint32_t my = 0; my < grid.info.height; ++my) {
      for (std::uint32_t mx = 0; mx < grid.info.width; ++mx) {
        const std::size_t index = index_of(mx, my);
        const float center = response->signed_distance[index];
        if (!std::isfinite(center)) {
          continue;
        }
        const float left = mx > 0 ? response->signed_distance[index_of(mx - 1, my)] : nan;
        const float right = mx + 1 < grid.info.width ?
          response->signed_distance[index_of(mx + 1, my)] : nan;
        const float down = my > 0 ? response->signed_distance[index_of(mx, my - 1)] : nan;
        const float up = my + 1 < grid.info.height ?
          response->signed_distance[index_of(mx, my + 1)] : nan;
        if (std::isfinite(left) && std::isfinite(right)) {
          response->gradient_x[index] = static_cast<float>((right - left) / (2.0 * resolution));
        } else if (std::isfinite(right)) {
          response->gradient_x[index] = static_cast<float>((right - center) / resolution);
        } else if (std::isfinite(left)) {
          response->gradient_x[index] = static_cast<float>((center - left) / resolution);
        } else {
          response->gradient_x[index] = 0.0F;
        }
        if (std::isfinite(down) && std::isfinite(up)) {
          response->gradient_y[index] = static_cast<float>((up - down) / (2.0 * resolution));
        } else if (std::isfinite(up)) {
          response->gradient_y[index] = static_cast<float>((up - center) / resolution);
        } else if (std::isfinite(down)) {
          response->gradient_y[index] = static_cast<float>((center - down) / resolution);
        } else {
          response->gradient_y[index] = 0.0F;
        }
      }
    }
    gradient_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - gradient_started).count();

    const InputHealth health_at_end = inputHealth();
    response->stale = health_at_end.map_update_stale || health_at_end.odom_stale;
    const double projection_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - projection_started).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ROGMap projection generation=%llu cells=%zu compute=%.1f ms stale=%s",
      static_cast<unsigned long long>(response->generation), cell_count, projection_ms,
      response->stale ? "true" : "false");
    log_projection_end();
    lock.unlock();
    if (!unknown_projection_audit.empty()) {
      unknown_pub_->publish(
        makeCloud(unknown_projection_audit, rclcpp::Time(grid.header.stamp, RCL_ROS_TIME)));
    }
  }

private:
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string odom_topic_;
  std::string cloud_topic_;
  double cloud_timeout_sec_{0.5};
  double odom_timeout_sec_{0.5};
  double tf_timeout_sec_{0.1};
  double debug_rate_hz_{2.0};
  bool map_update_trace_enabled_{false};
  double esdf_visualization_height_{0.15};
  std::string debug_viz_topic_;
  int debug_viz_stride_{1};
  bool debug_viz_include_unknown_{false};
  std::string debug_bounds_topic_;
  bool debug_bounds_show_labels_{true};
  double self_filter_radius_{0.45};
  int debug_qos_depth_{1};
  bool input_qos_reliable_{false};
  bool debug_qos_reliable_{false};
  bool test_reset_to_unknown_active_{false};
  bool test_fault_injection_enabled_{false};
  std::string planning_grid_owner_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    test_fault_parameter_callback_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<RogMapEngine> map_;
  std::unique_ptr<OdomLayerStore> odom_layer_;
  std::unique_ptr<GroundColumnGrid> ground_columns_;
  double base_ground_offset_{0.0};
  double layer_window_x_{12.0}, layer_window_y_{12.0};
  double layer_center_x_{0.0}, layer_center_y_{0.0};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr odom_layer_pub_;
  std::mutex layer_mutex_;
  std::uint64_t layer_height_rejected_{0};
  std::uint64_t layer_out_of_range_{0};
  std::mutex map_mutex_;
  std::mutex input_mutex_;
  std::optional<SteadyTime> last_cloud_receive_time_;
  std::optional<SteadyTime> last_map_update_time_;
  std::optional<SteadyTime> last_odom_receive_time_;
  std::uint64_t projection_request_sequence_{0};
  rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
  double last_layer_stamp_seconds_{0.0};
  builtin_interfaces::msg::Time last_layer_source_stamp_;
  bool has_map_data_{false};
  bool intensity_required_{false};
  bool require_odom_identity_{false};
  bool has_accepted_map_transform_{false};
  double identity_timeout_sec_{1.0};
  std::int64_t last_identity_signal_ns_{0};
  std::optional<SteadyTime> last_identity_receive_;
  OdomEpochGuard odom_epoch_guard_;
  std::mutex epoch_mutex_;

  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;
  rclcpp::CallbackGroup::SharedPtr debug_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;
  SteadyTime pending_cloud_receive_time_{};
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounds_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stale_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr layer_updated_pub_;
  rclcpp::Service<ats_rog_map_interfaces::srv::SaveOdomMap>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr cloud_pending_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace ats_rog_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ats_rog_map::AtsRogMapNode>();
  // The planning-map owner lives in this process and calls the projection
  // directly; there is no projection service. Its node name is pinned locally so
  // a launch-level `__node` remap for this executable cannot rename it as well.
  rclcpp::NodeOptions planning_options;
  planning_options.arguments({"--ros-args", "-r", "__node:=ats_rog_map_adapter"});
  ats_rog_map::AtsRogMapNode * const map_node = node.get();
  std::shared_ptr<rclcpp::Node> planning_node;
  if (node->ownsPlanningGrid()) {
    planning_node = ats_rog_map::planning::makePlanningMapPublisher(
      [map_node](const ats_rog_map::GroundProjectionRequest & request,
      ats_rog_map::GroundProjection & response) {
        map_node->getGroundProjection(request, response);
      }, planning_options);
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  if (planning_node) {
    executor.add_node(planning_node);
  }
  executor.spin();
  // Stop the projection worker before the map node it calls is destroyed.
  planning_node.reset();
  rclcpp::shutdown();
  return 0;
}
