#ifndef ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_IMPL_HPP_
#define ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_IMPL_HPP_

// Copyright 2026

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_rog_map/planning/mapping_health.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "ats_navigation_interfaces/msg/planning_map_status.hpp"
#include "ats_rog_map/odom_layer.hpp"
#include "ats_rog_map/planning/distance_viz.hpp"
#include "ats_rog_map/planning/ground_projection_fusion.hpp"
#include "ats_rog_map/planning/planning_map_snapshot.hpp"
#include "ats_rog_map/planning/planning_map_publisher.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"

namespace ats_rog_map::planning
{

class PlanningMapPublisher final : public rclcpp::Node
{
public:
  PlanningMapPublisher(GroundProjectionProvider provider, const rclcpp::NodeOptions & options);
  ~PlanningMapPublisher() override;

private:
  struct ProjectionJob
  {
    GroundProjectionRequest request;
    std::uint64_t epoch{0};
    std::uint64_t localization_epoch{0};
    rclcpp::Time start_stamp;
  };

  static bool localizationStateAllowsPlanning(std::uint8_t state);
  void onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr message);
  bool localizationReady() const;
  bool inputFresh(const nav_msgs::msg::OccupancyGrid & grid) const;
  bool projectionFresh(const nav_msgs::msg::OccupancyGrid & grid) const;
  bool inputSynchronized(
    const nav_msgs::msg::OccupancyGrid & grid,
    const builtin_interfaces::msg::Time & projection_stamp) const;
  void requestProjection();
  void runProjectionWorker();
  bool workerStopping();
  void completeProjection(
    const ProjectionJob & job, const GroundProjection & response, bool computed);
  void processProjection(
    const GroundProjection & response,
    std::uint64_t localization_epoch, std::uint64_t request_epoch);
  sensor_msgs::msg::PointCloud2 encodeEsdfCloud(
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<double> & distance_field) const;
  static std::array<double, 2> cellToFrame(
    const nav_msgs::msg::MapMetaData & info, double gx, double gy);
  visualization_msgs::msg::MarkerArray encodeClearanceContours(
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<double> & distance_field,
    double inscribed_radius, double circumscribed_radius) const;
  bool planningGridVizWanted() const;
  void publishPlanningGridViz(const nav_msgs::msg::OccupancyGrid & grid);
  void publishBlockedGrid(const nav_msgs::msg::OccupancyGrid & source);
  void publishUnavailable(const char * reason, bool include_blocking_grid = false);
  void publishMapStatus(bool ready, std::uint64_t rog_generation, const std::string & reason);

  std::string static_map_topic_;
  std::string traversability_grid_topic_;
  std::string slope_grid_topic_;
  std::string planning_grid_viz_topic_;
  std::string planning_grid_owner_;
  std::string esdf_cloud_topic_;
  std::string clearance_contour_topic_;
  std::string ready_topic_;
  std::string generation_topic_;
  std::string map_status_topic_;
  std::string planning_snapshot_topic_;
  std::string localization_status_topic_;
  std::string robot_frame_;
  std::optional<std::chrono::steady_clock::time_point> last_projection_complete_time_;
  double projection_rate_hz_{2.0};
  double projection_request_timeout_sec_{2.0};
  double projection_snapshot_timeout_sec_{2.0};
  double projection_min_height_{0.10};
  double projection_max_height_{0.80};
  double planning_resolution_{0.05};
  double input_timeout_sec_{2.0};
  double input_sync_tolerance_sec_{1.0};
  double future_stamp_tolerance_sec_{0.1};
  double tf_lookup_timeout_sec_{0.1};
  double tf_latest_fallback_max_skew_sec_{0.1};
  double signed_distance_max_m_{2.0};
  double footprint_length_{0.70};
  double footprint_width_{0.55};
  double footprint_safety_margin_{0.05};
  double robot_unknown_clear_radius_{0.0};
  double localization_status_timeout_sec_{1.0};
  bool require_localization_status_{false};
  bool mapping_mode_{false};
  bool use_odom_layer_{false};
  bool robot_unknown_clear_footprint_{false};
  double robot_clear_length_{0.70}, robot_clear_width_{0.55}, robot_clear_margin_{0.10};
  bool require_odom_identity_{false};
  bool odom_identity_stable_{false};
  bool odom_identity_latched_{false};
  std::uint64_t odom_instance_id_{0}, odom_epoch_{0};
  std::int64_t last_identity_signal_ns_{0};
  double odometry_silence_sec_{std::numeric_limits<double>::infinity()};
  bool require_terrain_inputs_{true};
  GroundProjectionFusionParams fusion_params_;
  bool allow_missing_static_map_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex input_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_grid_;
  TerrainSpikeFilter terrain_spike_filter_;
  std::size_t terrain_suppressed_cells_{0};
  nav_msgs::msg::OccupancyGrid::SharedPtr slope_grid_;
  nav_msgs::msg::OccupancyGrid last_blocking_grid_;
  RogMapEsdfSnapshot last_numeric_snapshot_;
  ats_rc_esdf::RcTraversabilityEsdfProvider planning_esdf_;
  // Guards every field below that callbacks and the projection worker share.
  std::recursive_mutex state_mutex_;
  bool request_pending_{false};
  std::uint64_t next_request_epoch_{0};
  std::uint64_t active_request_epoch_{0};
  std::uint64_t localization_epoch_{0};
  std::uint64_t last_rog_generation_{0};
  std::uint64_t map_status_sequence_{0};
  std::uint8_t localization_state_{
      ats_navigation_interfaces::msg::LocalizationStatus::STATE_UNINITIALIZED};
  bool has_localization_status_{false};
  std::optional<std::chrono::steady_clock::time_point>
      last_localization_status_signal_;
  std::chrono::steady_clock::time_point active_request_sent_time_{};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr layer_updated_sub_;
  bool mapping_update_pending_{false};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slope_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::
      SharedPtr localization_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_grid_viz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_contour_pub_;
  double esdf_viz_max_distance_{2.0};
  std::size_t esdf_viz_max_points_{250000U};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generation_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlanningMapStatus>::
      SharedPtr map_status_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlanningMapSnapshot>::
      SharedPtr planning_snapshot_pub_;
  rclcpp::TimerBase::SharedPtr projection_timer_;

  GroundProjectionProvider projection_provider_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::optional<ProjectionJob> pending_job_;
  bool worker_busy_{false};
  bool worker_stop_{false};
  std::thread projection_worker_;
};


}  // namespace ats_rog_map::planning

#endif  // ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_IMPL_HPP_
