#ifndef ATS_ROG_MAP__ATS_ROG_MAP_NODE_IMPL_HPP_
#define ATS_ROG_MAP__ATS_ROG_MAP_NODE_IMPL_HPP_

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

using SteadyTime = std::chrono::steady_clock::time_point;

class AtsRogMapNode final : public rclcpp::Node
{
public:
  AtsRogMapNode();
  bool ownsPlanningGrid() const {return planning_grid_owner_ == "rog_map";}
  void getGroundProjection(
    const GroundProjectionRequest & request_in, GroundProjection & response_out);

private:
  struct InputHealth
  {
    double map_update_age_sec{std::numeric_limits<double>::infinity()};
    double odom_age_sec{std::numeric_limits<double>::infinity()};
    double cloud_age_sec{std::numeric_limits<double>::infinity()};
    bool map_update_stale{true};
    bool odom_stale{true};
    bool raw_cloud_stale{true};
  };

  void onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr message);
  void saveMap(
    const ats_rog_map_interfaces::srv::SaveOdomMap::Request::SharedPtr request,
    ats_rog_map_interfaces::srv::SaveOdomMap::Response::SharedPtr response);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  bool lookupTransform(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform);
  void deferCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void tryProcessCloud();
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const SteadyTime & trace_begin);
  void accumulateOdomLayer(
    const rog_map::PointCloud & cloud, const geometry_msgs::msg::Vector3 & base,
    const geometry_msgs::msg::Vector3 & sensor, const rclcpp::Time & stamp);
  nav_msgs::msg::OccupancyGrid sampleLayerBuffer(
    const std::vector<std::int8_t> & full_cells, const OdomLayerConfig & c,
    double origin_x, double origin_y, std::uint32_t width, std::uint32_t height,
    double resolution) const;
  void getOdomLayerProjection(float requested_resolution, GroundProjection & response);
  sensor_msgs::msg::PointCloud2 makeCloud(
    const rog_map::vec_E<rog_map::Vec3f> & points, const rclcpp::Time & stamp) const;
  void collectEsdfDebugPoints(
    const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max,
    pcl::PointCloud<pcl::PointXYZI> & cloud) const;
  sensor_msgs::msg::PointCloud2 makeEsdfCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud, const rclcpp::Time & stamp) const;
  sensor_msgs::msg::PointCloud2 makeVoxelDebugCloud(
    const std::vector<rog_map::ProbMap::VoxelDebugCell> & cells,
    const rclcpp::Time & stamp) const;
  static bool validBounds(const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max);
  void appendBoundsMarker(
    visualization_msgs::msg::MarkerArray & markers, const rog_map::Vec3f & box_min,
    const rog_map::Vec3f & box_max, const std::string & ns, const std::string & label,
    int id, float red, float green, float blue, float alpha, float line_width,
    const rclcpp::Time & stamp);
  visualization_msgs::msg::MarkerArray makeBoundsMarkers(
    const rog_map::Vec3f & local_min, const rog_map::Vec3f & local_max,
    const rog_map::Vec3f & visualization_min, const rog_map::Vec3f & visualization_max,
    const rog_map::Vec3f & update_min, const rog_map::Vec3f & update_max, const double resolution,
    const rclcpp::Time & stamp, const bool show_visualization_range);
  void publishDebug();
  void publishHealth();
  void getHealthStaleness(
    bool & map_update_stale, bool & odom_stale, bool & raw_cloud_stale);
  InputHealth inputHealth();

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
  std::string planning_grid_owner_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<RogMapEngine> map_;
  std::unique_ptr<OdomLayerStore> odom_layer_;
  std::unique_ptr<GroundColumnGrid> ground_columns_;
  bool projection_follow_measured_ground_{false};
  double base_ground_offset_{0.0};
  double layer_window_x_{12.0}, layer_window_y_{12.0};
  double layer_global_coarse_resolution_{0.05};
  bool mapping_mode_{false};
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


#endif  // ATS_ROG_MAP__ATS_ROG_MAP_NODE_IMPL_HPP_
