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
  PlanningMapPublisher(GroundProjectionProvider provider, const rclcpp::NodeOptions & options)
  : Node("ats_rog_map_adapter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    projection_provider_(std::move(provider))
  {
    if (!projection_provider_) {
      throw std::invalid_argument("planning map publisher needs a ground projection provider");
    }
    projection_rate_hz_ = std::max(0.1, declare_parameter<double>("projection_rate_hz", 2.0));
    projection_request_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("projection_request_timeout_sec", 2.0));
    projection_snapshot_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("projection_snapshot_timeout_sec", 2.0));
    projection_min_height_ = declare_parameter<double>("projection_min_height", 0.10);
    projection_max_height_ = declare_parameter<double>("projection_max_height", 0.80);
    planning_resolution_ = std::max(0.01, declare_parameter<double>("planning_resolution", 0.10));
    planning_grid_owner_ = declare_parameter<std::string>("planning_grid_owner", "rog_map");
    if (planning_grid_owner_ != "rog_map" && planning_grid_owner_ != "rc_esdf") {
      throw std::invalid_argument(
        "planning_grid_owner must be 'rog_map' or 'rc_esdf'");
    }
    if (planning_grid_owner_ == "rc_esdf") {
      throw std::runtime_error(
        "rc_esdf planning-grid owner has no active publisher in this launch; refusing duplicate ownership");
    }
    static_map_topic_ = declare_parameter<std::string>("static_map_topic", "/map");
    traversability_grid_topic_ = declare_parameter<std::string>(
      "traversability_grid_topic", "/traversability_grid");
    slope_grid_topic_ = declare_parameter<std::string>(
      "traversability_slope_grid_topic", "/traversability_slope_grid");
    // M4: planning_snapshot is the only planning-map contract. The two topics
    // below are RViz-only views of the same publication, encoded only while
    // someone subscribes; nothing on the planning or safety path reads them.
    // An empty string disables a view.
    planning_grid_viz_topic_ = declare_parameter<std::string>(
      "planning_grid_viz_topic", "/rog_map_adapter/viz/planning_grid");
    // 5-field XYZ + distance + intensity ESDF 距离场可视化点云（仅订阅时编码，跳过未知格，
    // 配合 RViz Intensity Rainbow Flat Squares 呈现无遮挡平滑距离场）与两条净空等值线。
    esdf_cloud_topic_ = declare_parameter<std::string>(
      "esdf_cloud_topic", "/rog_map_adapter/viz/esdf_cloud");
    clearance_contour_topic_ = declare_parameter<std::string>(
      "clearance_contour_topic", "/rog_map_adapter/viz/clearance_contours");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/rog_map_adapter/ready");
    generation_topic_ = declare_parameter<std::string>(
      "generation_topic", "/rog_map_adapter/generation");
    map_status_topic_ = declare_parameter<std::string>(
        "map_status_topic", "/rog_map_adapter/status");
    planning_snapshot_topic_ = declare_parameter<std::string>(
      "planning_snapshot_topic", "/rog_map_adapter/planning_snapshot");
    localization_status_topic_ = declare_parameter<std::string>(
        "localization_status_topic", "/localization/status");
    require_localization_status_ =
        declare_parameter<bool>("require_localization_status", false);
    mapping_mode_ = declare_parameter<bool>("mapping_mode", false);
    const double mapping_rate = std::max(
      0.1, declare_parameter<double>("mapping_projection_rate_hz", 40.0));
    if (mapping_mode_) {projection_rate_hz_ = mapping_rate;}
    use_odom_layer_ = declare_parameter<bool>("use_odom_layer", false) || mapping_mode_;
    require_odom_identity_ = declare_parameter<bool>("require_odom_identity", false);
    // When false (Gazebo GT sim), missing/stale terrain+slope become all-unknown
    // placeholders so static+ROGMap projection can still fuse. Production keeps
    // true: terrain sync failure remains fail-closed.
    require_terrain_inputs_ =
        declare_parameter<bool>("require_terrain_inputs", true);
    localization_status_timeout_sec_ = std::max(
        0.1, declare_parameter<double>("localization_status_timeout_sec", 1.0));
    fusion_params_.static_obstacle_value_threshold = declare_parameter<int>(
      "static_obstacle_value_threshold", 50);
    fusion_params_.terrain_obstacle_value_threshold = declare_parameter<int>(
      "terrain_obstacle_value_threshold", 50);
    // Plan A: a terrain hard obstacle needs this many consecutive frames with distinct
    // stamps before it blocks planning.  1 keeps the legacy single-frame behaviour.
    terrain_spike_filter_ = TerrainSpikeFilter(
      declare_parameter<int>("terrain_confirm_frames", 1),
      fusion_params_.terrain_obstacle_value_threshold);
    fusion_params_.slope_grid_max_degrees = declare_parameter<double>(
      "slope_grid_max_degrees", 45.0);
    fusion_params_.slope_obstacle_degrees = declare_parameter<double>(
      "slope_obstacle_degrees", 28.0);
    fusion_params_.planning_resolution = planning_resolution_;
    fusion_params_.unknown_is_obstacle = declare_parameter<bool>("unknown_is_obstacle", true);
    fusion_params_.output_in_projection_frame =
      declare_parameter<bool>("output_in_projection_frame", false) || mapping_mode_;
    fusion_params_.use_static_map_obstacles = declare_parameter<bool>(
      "use_static_map_obstacles", true);
    fusion_params_.use_static_map_geofence_only = declare_parameter<bool>(
      "use_static_map_geofence_only", false);
    fusion_params_.static_map_geofence_border_m = std::max(
      0.0, declare_parameter<double>("static_map_geofence_border_m", 0.35));
    fusion_params_.terrain_free_override_rects = declare_parameter<std::vector<double>>(
      "terrain_free_override_rects", std::vector<double>{});
    fusion_params_.mapping_mode = mapping_mode_;
    fusion_params_.terrain_free_override_frame = declare_parameter<std::string>(
      "terrain_free_override_frame", "map");
    fusion_params_.diagnostic_allow_terrain_free_override = declare_parameter<bool>(
      "diagnostic_allow_terrain_free_override", false);
    fusion_params_.tunnel_free_suppresses_terrain = declare_parameter<bool>(
      "tunnel_free_suppresses_terrain", true);
    if (mapping_mode_ && !fusion_params_.terrain_free_override_rects.empty()) {
      RCLCPP_WARN(get_logger(), "Terrain rectangle prior in mapping mode: %s",
        fusion_params_.diagnostic_allow_terrain_free_override ?
        "diagnostic A/B enabled; not a prior-free acceptance run" : "disabled");
    }
    if (fusion_params_.terrain_free_override_rects.size() % 4 != 0) {
      RCLCPP_ERROR(get_logger(), "terrain_free_override_rects 长度不是 4 的倍数，已忽略。");
      fusion_params_.terrain_free_override_rects.clear();
    }
    allow_missing_static_map_ = declare_parameter<bool>("allow_missing_static_map", false);
    input_timeout_sec_ = std::max(0.0, declare_parameter<double>("input_timeout_sec", 2.0));
    input_sync_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("input_sync_tolerance_sec", 1.0));
    future_stamp_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("future_stamp_tolerance_sec", 0.1));
    tf_lookup_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("tf_lookup_timeout_sec", 0.1));
    tf_latest_fallback_max_skew_sec_ = std::max(
      0.0, declare_parameter<double>("tf_latest_fallback_max_skew_sec", 0.1));
    signed_distance_max_m_ = std::max(
      1e-3, declare_parameter<double>("signed_distance_max_m", 2.0));
    esdf_viz_max_distance_ = std::max(
      0.1, declare_parameter<double>("esdf_viz_max_distance", signed_distance_max_m_));
    esdf_viz_max_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1000, declare_parameter<std::int64_t>("esdf_viz_max_points", 250000)));
    footprint_length_ = std::max(0.0, declare_parameter<double>("footprint_length", 0.58));
    footprint_width_ = std::max(0.0, declare_parameter<double>("footprint_width", 0.58));
    footprint_safety_margin_ = std::max(
      0.0, declare_parameter<double>("footprint_safety_margin", 0.05));
    robot_frame_ = declare_parameter<std::string>("robot_frame", "gimbal_yaw_odom");
    robot_unknown_clear_radius_ = std::max(
      0.0, declare_parameter<double>("robot_unknown_clear_radius", 0.0));
    robot_unknown_clear_footprint_ = declare_parameter<bool>("robot_unknown_clear_footprint", false);
    robot_clear_length_ = std::max(
      0.0, declare_parameter<double>("robot_unknown_clear_length", footprint_length_));
    robot_clear_width_ = std::max(
      0.0, declare_parameter<double>("robot_unknown_clear_width", footprint_width_));
    // 建图模式（robot_unknown_clear_footprint=true）下，ROGMap self_filter_radius(0.45 m)
    // 与倾斜雷达近场盲区会在车身周围形成未观测未知环带，且图搜索最低净空为内切半径加量化余量；
    // 默认外扩 0.14 m（仅清理未知格并经视线遮挡检查，不穿透/改写占据墙体）以衔接近场已知空闲地面。
    robot_clear_margin_ = std::max(
      0.0, declare_parameter<double>(
        "robot_unknown_clear_margin", std::max(footprint_safety_margin_, 0.14)));

    static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        static_map_ = std::move(msg);
      });
    traversability_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      traversability_grid_topic_, rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        // Every terrain frame passes through the spike filter, not only the one the
        // projection timer happens to sample, so consecutive-frame counts are exact.
        if (terrain_spike_filter_.confirmFrames() > 1) {
          msg = std::make_shared<nav_msgs::msg::OccupancyGrid>(
            terrain_spike_filter_.process(*msg));
          terrain_suppressed_cells_ = terrain_spike_filter_.suppressedCells();
        }
        traversability_grid_ = std::move(msg);
      });
    slope_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      slope_grid_topic_, rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        slope_grid_ = std::move(msg);
      });
    localization_status_sub_ =
        create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
            localization_status_topic_,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&PlanningMapPublisher::onLocalizationStatus, this,
                      std::placeholders::_1));

    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    if (!planning_grid_viz_topic_.empty()) {
      planning_grid_viz_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        planning_grid_viz_topic_, output_qos);
    }
    if (!esdf_cloud_topic_.empty()) {
      esdf_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        esdf_cloud_topic_, rclcpp::QoS(1).reliable().transient_local());
    }
    if (!clearance_contour_topic_.empty()) {
      clearance_contour_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        clearance_contour_topic_, rclcpp::QoS(1).reliable().transient_local());
    }
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, output_qos);
    generation_pub_ = create_publisher<std_msgs::msg::UInt64>(generation_topic_, output_qos);
    map_status_pub_ =
        create_publisher<ats_navigation_interfaces::msg::PlanningMapStatus>(
            map_status_topic_, output_qos);
    planning_snapshot_pub_ = create_publisher<
      ats_navigation_interfaces::msg::PlanningMapSnapshot>(planning_snapshot_topic_, output_qos);
    projection_worker_ = std::thread([this]() {runProjectionWorker();});
    if (mapping_mode_) {
      layer_updated_sub_ = create_subscription<std_msgs::msg::UInt64>(
        "/rog_map/odom_layer_updated", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::UInt64::ConstSharedPtr) {
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);
          if (request_pending_) {mapping_update_pending_ = true; return;}
          requestProjection();
        });
    }

    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      publishMapStatus(false, 0, "waiting for first projection");
    }
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / projection_rate_hz_));
    projection_timer_ = create_wall_timer(
      std::max(period, std::chrono::milliseconds(1)),
      [this]() {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        requestProjection();
      });
    RCLCPP_INFO(
      get_logger(),
      "ROGMap ground adapter ready: in-process projection planning_snapshot='%s' height=[%.2f, %.2f] m",
      planning_snapshot_topic_.c_str(), projection_min_height_, projection_max_height_);
  }

  ~PlanningMapPublisher() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_stop_ = true;
    }
    worker_cv_.notify_all();
    if (projection_worker_.joinable()) {
      projection_worker_.join();
    }
  }

private:
  static bool localizationStateAllowsPlanning(std::uint8_t state)
  {
    using LS = ats_navigation_interfaces::msg::LocalizationStatus;
    // CONFIRMED: post-accept hold with map-frame evidence.
    // DEGRADED: soft quality loss / consecutive rejects; map-odom still held.
    // LOST/BOOTSTRAP/RELOCALIZING/UNINITIALIZED remain fail-closed.
    return state == LS::STATE_TRACKING || state == LS::STATE_CONFIRMED ||
           state == LS::STATE_DEGRADED;
  }


  void onLocalizationStatus(
      const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
          message) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    // Always bookkeep epoch/state so map_status.localization_epoch stays
    // aligned with /localization/status. Gazebo may disable
    // require_localization_status to keep planning through GICP flicker;
    // skipping this update left map_status_epoch=0 while fusion published
    // epoch>=1 and permanently failed the health probe (stable=0/3).
    const bool epoch_changed =
        has_localization_status_ && message->epoch != localization_epoch_;
    const auto signal_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if ((mapping_mode_ || require_odom_identity_) && signal_ns <= last_identity_signal_ns_) {return;}
    const bool odom_changed = message->odom_instance_id != odom_instance_id_ ||
      message->odom_epoch != odom_epoch_;
    last_identity_signal_ns_ = signal_ns;
    odom_identity_stable_ = !odom_changed && message->odom_instance_id != 0 &&
      message->odom_epoch != 0;
    odom_instance_id_ = message->odom_instance_id;
    odom_epoch_ = message->odom_epoch;
    odom_identity_latched_ = message->odom_identity_latched;
    odometry_silence_sec_ = message->odometry_silence_sec;
    has_localization_status_ = true;
    localization_epoch_ = message->epoch;
    localization_state_ = message->state;
    last_localization_status_signal_ = std::chrono::steady_clock::now();
    if (!require_localization_status_ && !mapping_mode_ && !require_odom_identity_) {
      return;
    }
    if (epoch_changed || odom_changed || !localizationReady()) {
      // Bumping the epoch makes an in-flight projection result unusable.
      request_pending_ = false;
      ++active_request_epoch_;
      RogMapEsdfSnapshot invalidated_snapshot;
      last_numeric_snapshot_ = std::move(invalidated_snapshot);
      publishUnavailable(
          epoch_changed
              ? "localization epoch changed; invalidating planning snapshot"
              : "localization is not tracking");
    }
  }

  bool localizationReady() const {
    if (mapping_mode_ || require_odom_identity_) {
      const double status_age = last_localization_status_signal_ ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *last_localization_status_signal_).count() :
        std::numeric_limits<double>::infinity();
      if (!odom_identity_stable_ || odom_identity_latched_ || !mappingInputHealthy(
          odom_instance_id_, odom_epoch_, odometry_silence_sec_, status_age,
          localization_status_timeout_sec_)) {return false;}
      // Growing-map registration can build while global localization is in
      // BOOTSTRAP/LOST. Goal dispatch still requires accepted map evidence.
      if (mapping_mode_) {return true;}
    }
    if (!require_localization_status_) {
      return true;
    }
    return has_localization_status_ && last_localization_status_signal_ &&
           localizationStateAllowsPlanning(localization_state_) &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         *last_localization_status_signal_)
                   .count() <= localization_status_timeout_sec_;
  }

  bool inputFresh(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    if (input_timeout_sec_ <= 0.0) {
      return true;
    }
    const rclcpp::Time stamp(grid.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    const double age = (now() - stamp).seconds();
    return age >= -future_stamp_tolerance_sec_ && age <= input_timeout_sec_;
  }

  bool projectionFresh(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    const rclcpp::Time stamp(grid.header.stamp);
    // ROGMap Gazebo projections often omit stamps (source_stamp_ns=0). This
    // helper is only called on a just-received service response, so a zero
    // stamp means "fresh at receipt" rather than "infinitely stale". Domain26
    // saw 177 ready=0 flaps from the old stamp<=0 => false rule despite 30s
    // projection_snapshot_timeout_sec.
    if (stamp.nanoseconds() <= 0) {
      return true;
    }
    const double age = (now() - stamp).seconds();
    return age >= -future_stamp_tolerance_sec_ && age <= projection_snapshot_timeout_sec_;
  }

  bool inputSynchronized(
    const nav_msgs::msg::OccupancyGrid & grid,
    const builtin_interfaces::msg::Time & projection_stamp) const
  {
    if (input_sync_tolerance_sec_ <= 0.0) {
      return true;
    }
    const rclcpp::Time input_stamp(grid.header.stamp);
    const rclcpp::Time rog_stamp(projection_stamp);
    return input_stamp.nanoseconds() > 0 && rog_stamp.nanoseconds() > 0 &&
           std::abs((input_stamp - rog_stamp).seconds()) <= input_sync_tolerance_sec_;
  }

  struct ProjectionJob
  {
    GroundProjectionRequest request;
    std::uint64_t epoch{0};
    std::uint64_t localization_epoch{0};
    rclcpp::Time start_stamp;
  };

  // Caller holds state_mutex_.
  void requestProjection()
  {
    if (!localizationReady()) {
      publishUnavailable(
          "localization status is missing, stale, or not tracking");
      return;
    }
    if (request_pending_) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - active_request_sent_time_).count();
      if (elapsed <= projection_request_timeout_sec_) {
        return;
      }
      // Deadline expired: drop the request and fence its late result by epoch.
      request_pending_ = false;
      ++active_request_epoch_;
      publishUnavailable("ROGMap projection request timed out");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_busy_ || pending_job_) {
        // A timed-out projection is still running; it cannot be preempted.
        publishUnavailable("ROGMap projection worker is still busy");
        return;
      }
    }
    ProjectionJob job;
    job.request.min_height = static_cast<float>(projection_min_height_);
    job.request.max_height = static_cast<float>(projection_max_height_);
    job.request.resolution = static_cast<float>(planning_resolution_);
    job.request.use_odom_layer = use_odom_layer_;
    if (mapping_mode_ || require_odom_identity_) {
      job.request.expected_odom_instance_id = odom_instance_id_;
      job.request.expected_odom_epoch = odom_epoch_;
    }
    job.epoch = ++next_request_epoch_;
    job.localization_epoch = localization_epoch_;
    job.start_stamp = now();
    active_request_epoch_ = job.epoch;
    active_request_sent_time_ = std::chrono::steady_clock::now();
    request_pending_ = true;
    RCLCPP_INFO(
      get_logger(),
      "P2 adapter projection begin request=%llu request_id=%llu start_ns=%lld "
      "localization_epoch=%llu deadline_sec=%.3f",
      static_cast<unsigned long long>(job.epoch), static_cast<unsigned long long>(job.epoch),
      static_cast<long long>(job.start_stamp.nanoseconds()),
      static_cast<unsigned long long>(job.localization_epoch), projection_request_timeout_sec_);
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_job_ = std::move(job);
    }
    worker_cv_.notify_one();
  }

  void runProjectionWorker()
  {
    while (true) {
      ProjectionJob job;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_cv_.wait(lock, [this]() {return worker_stop_ || pending_job_.has_value();});
        if (worker_stop_) {return;}
        job = std::move(*pending_job_);
        pending_job_.reset();
        worker_busy_ = true;
      }
      GroundProjection response;
      bool computed = true;
      try {
        projection_provider_(job.request, response);
      } catch (const std::exception & exception) {
        computed = false;
        RCLCPP_ERROR(get_logger(), "ROGMap projection failed: %s", exception.what());
      }
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_busy_ = false;
      }
      if (workerStopping()) {return;}
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      completeProjection(job, response, computed);
    }
  }

  bool workerStopping()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    return worker_stop_;
  }

  // Caller holds state_mutex_. Same checks as the former service response callback.
  void completeProjection(
    const ProjectionJob & job, const GroundProjection & response, bool computed)
  {
    if (!request_pending_ || job.epoch != active_request_epoch_) {
      return;
    }
    request_pending_ = false;
    if (!computed) {
      publishUnavailable("ROGMap projection response failed");
      return;
    }
    try {
      if (!localizationReady() || job.localization_epoch != localization_epoch_) {
        publishUnavailable("discarded projection from an obsolete localization epoch");
        return;
      }
      const rclcpp::Time response_stamp = now();
      const double round_trip_ms = 1000.0 * std::chrono::duration<double>(
        std::chrono::steady_clock::now() - active_request_sent_time_).count();
      RCLCPP_INFO(
        get_logger(),
        "P2 adapter projection end request=%llu start_ns=%lld end_ns=%lld "
        "round_trip_ms=%.1f source_generation=%llu source_stamp_ns=%lld ready=%d stale=%d "
        "odom_instance_id=%llu odom_epoch=%llu odom_layer_included=%d frame=%s size=%ux%u",
        static_cast<unsigned long long>(job.epoch),
        static_cast<long long>(job.start_stamp.nanoseconds()),
        static_cast<long long>(response_stamp.nanoseconds()), round_trip_ms,
        static_cast<unsigned long long>(response.generation),
        static_cast<long long>(rclcpp::Time(response.occupancy_grid.header.stamp).nanoseconds()),
        response.ready ? 1 : 0, response.stale ? 1 : 0,
        static_cast<unsigned long long>(response.odom_instance_id),
        static_cast<unsigned long long>(response.odom_epoch),
        response.odom_layer_included ? 1 : 0, response.occupancy_grid.header.frame_id.c_str(),
        response.occupancy_grid.info.width, response.occupancy_grid.info.height);
      processProjection(response, job.localization_epoch, job.epoch);
      if (mapping_update_pending_) {
        mapping_update_pending_ = false;
        requestProjection();
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "ROGMap projection response failed: %s", exception.what());
      publishUnavailable("ROGMap projection response failed");
    }
  }

  void processProjection(
      const GroundProjection &response,
      std::uint64_t localization_epoch, std::uint64_t request_epoch) {
    if (use_odom_layer_ && response.ready && !response.odom_layer_included) {
      publishUnavailable("global odom layer was requested but missing from projection");
      return;
    }
    if ((mapping_mode_ || require_odom_identity_) && !projectionIdentityMatches(
        odom_instance_id_, odom_epoch_, response.odom_instance_id, response.odom_epoch))
    {
      publishUnavailable("projection belongs to a different frontend epoch");
      return;
    }
    const RogMapEsdfSnapshot numeric_snapshot = RogMapEsdfSnapshot::fromResponse(response);
    if (response.occupancy_grid.info.width > 0 && response.occupancy_grid.info.height > 0 &&
      !response.occupancy_grid.data.empty())
    {
      last_blocking_grid_ = response.occupancy_grid;
    }
    // Keep every ready=false heartbeat tied to the latest numeric response;
    // unavailable paths must not report an older ROGMap source generation.
    last_rog_generation_ = response.generation;
    nav_msgs::msg::OccupancyGrid::SharedPtr static_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr traversability;
    nav_msgs::msg::OccupancyGrid::SharedPtr slope;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      static_map = static_map_;
      traversability = traversability_grid_;
      slope = slope_grid_;
    }
    const auto message_stamp_ns = [](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid) {
        return grid ? static_cast<long long>(rclcpp::Time(grid->header.stamp).nanoseconds()) : 0LL;
      };
    const auto message_age = [this](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid) {
        if (!grid || rclcpp::Time(grid->header.stamp).nanoseconds() <= 0) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return (now() - rclcpp::Time(grid->header.stamp)).seconds();
      };
    const auto timestamp_delta = [](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
                                    const builtin_interfaces::msg::Time & projection_stamp) {
        if (!grid || rclcpp::Time(grid->header.stamp).nanoseconds() <= 0 ||
          rclcpp::Time(projection_stamp).nanoseconds() <= 0)
        {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return std::abs((rclcpp::Time(grid->header.stamp) -
          rclcpp::Time(projection_stamp)).seconds());
      };
    RCLCPP_INFO(
      get_logger(),
      "P2 adapter projection inputs request=%llu localization_epoch=%llu source_generation=%llu "
      "projection_stamp_ns=%lld terrain_stamp_ns=%lld terrain_age=%.3f terrain_delta=%.3f "
      "slope_stamp_ns=%lld slope_age=%.3f slope_delta=%.3f",
      static_cast<unsigned long long>(request_epoch),
      static_cast<unsigned long long>(localization_epoch),
      static_cast<unsigned long long>(response.generation),
      static_cast<long long>(rclcpp::Time(response.occupancy_grid.header.stamp).nanoseconds()),
      message_stamp_ns(traversability), message_age(traversability),
      timestamp_delta(traversability, response.occupancy_grid.header.stamp),
      message_stamp_ns(slope), message_age(slope),
      timestamp_delta(slope, response.occupancy_grid.header.stamp));
    // Mutable copy: Gazebo ROGMap often returns stamp=0. Fill receipt time so
    // projectionFresh + terrain sync/TF share a coherent clock (d26: 177
    // ready=0 from zero-stamp before terrain could even be evaluated).
    nav_msgs::msg::OccupancyGrid projection_grid = response.occupancy_grid;
    if (rclcpp::Time(projection_grid.header.stamp).nanoseconds() <= 0) {
      projection_grid.header.stamp = now();
    }
    if (!numeric_snapshot.available() || !projectionFresh(projection_grid)) {
      publishUnavailable("ROGMap projection is unavailable or stale");
      return;
    }
    const bool traversability_fresh = traversability && inputFresh(*traversability);
    const bool slope_fresh = slope && inputFresh(*slope);
    const bool traversability_synchronized = traversability && inputSynchronized(
      *traversability, projection_grid.header.stamp);
    const bool slope_synchronized = slope && inputSynchronized(
      *slope, projection_grid.header.stamp);
    // Missing prior maps are allowed only by the explicit mapping-mode override.
    bool static_map_is_live_fallback = false;
    if (!static_map && !allow_missing_static_map_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Static map missing; cannot fuse planning grid");
      publishUnavailable("Terrain inputs are missing, stale, or unsynchronized");
      return;
    }
    if (!static_map) {
      static_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(projection_grid);
      std::fill(static_map->data.begin(), static_map->data.end(), static_cast<std::int8_t>(-1));
      static_map_is_live_fallback = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Static PGM unavailable; continuing with odom live projection only");
    }
    if (!traversability_fresh || !slope_fresh ||
      !traversability_synchronized || !slope_synchronized)
    {
      if (require_terrain_inputs_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Terrain rejected: static=%d traversal fresh=%d sync=%d age=%.3f delta=%.3f; "
          "slope fresh=%d sync=%d age=%.3f delta=%.3f",
          1, traversability_fresh ? 1 : 0,
          traversability_synchronized ? 1 : 0, message_age(traversability),
          timestamp_delta(traversability, projection_grid.header.stamp),
          slope_fresh ? 1 : 0, slope_synchronized ? 1 : 0, message_age(slope),
          timestamp_delta(slope, projection_grid.header.stamp));
        publishUnavailable("Terrain inputs are missing, stale, or unsynchronized");
        return;
      }
      const auto make_unknown_like =
        [](const nav_msgs::msg::OccupancyGrid & source) {
          auto masked = std::make_shared<nav_msgs::msg::OccupancyGrid>();
          masked->header = source.header;
          masked->info = source.info;
          masked->data.assign(
            source.info.width * source.info.height, static_cast<std::int8_t>(-1));
          return masked;
        };
      if (!traversability_fresh || !traversability_synchronized) {
        traversability = make_unknown_like(projection_grid);
      }
      if (!slope_fresh || !slope_synchronized) {
        slope = make_unknown_like(projection_grid);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "require_terrain_inputs=false: substituting unknown terrain/slope "
        "(traversal fresh=%d sync=%d; slope fresh=%d sync=%d)",
        traversability_fresh ? 1 : 0, traversability_synchronized ? 1 : 0,
        slope_fresh ? 1 : 0, slope_synchronized ? 1 : 0);
    }

    const auto t_tf_start = std::chrono::steady_clock::now();
    const auto lookupAtProjectionStamp = [this](
      const std::string & target_frame, const std::string & source_frame,
      const builtin_interfaces::msg::Time & requested_stamp,
      geometry_msgs::msg::TransformStamped & transform) {
        try {
          transform = tf_buffer_.lookupTransform(
            target_frame, source_frame, rclcpp::Time(requested_stamp),
            rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
          return true;
        } catch (const tf2::TransformException & exception) {
          // Gazebo can publish a complete sensor slot just before the next
          // localization TF slot. Preserve the bounded fallback contract: a
          // latest-TF fallback is allowed only when it is older than the
          // requested sample by at most tf_latest_fallback_max_skew_sec_.
          try {
            const auto latest = tf_buffer_.lookupTransform(
              target_frame, source_frame, tf2::TimePointZero,
              tf2::durationFromSec(0.0));
            const double skew =
              (rclcpp::Time(requested_stamp) - rclcpp::Time(latest.header.stamp)).seconds();
            if (skew >= 0.0 && skew <= tf_latest_fallback_max_skew_sec_) {
              transform = latest;
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Using latest %s <- %s TF %.3f s behind projection stamp after bounded "
                "future extrapolation: %s",
                target_frame.c_str(), source_frame.c_str(), skew, exception.what());
              return true;
            }
          } catch (const tf2::TransformException &) {
            // Keep the original failure below; no usable latest transform exists.
          }
          return false;
        }
      };

    geometry_msgs::msg::TransformStamped static_from_projection;
    geometry_msgs::msg::TransformStamped static_from_robot;
    if (!static_map_is_live_fallback && !lookupAtProjectionStamp(
        static_map->header.frame_id, projection_grid.header.frame_id,
        projection_grid.header.stamp, static_from_projection)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap adapter waits for %s <- %s TF at the projection stamp",
        static_map->header.frame_id.c_str(), projection_grid.header.frame_id.c_str());
      publishUnavailable("ROGMap projection transform is unavailable");
      return;
    }
    if (static_map_is_live_fallback) {
      static_from_projection.header = projection_grid.header;
      static_from_projection.header.frame_id = projection_grid.header.frame_id;
      static_from_projection.child_frame_id = projection_grid.header.frame_id;
      static_from_projection.transform.rotation.w = 1.0;
    }
    // The robot pose only centres the ego unknown-clearing circle. With the radius at its
    // default 0.0 it is unused, and requiring it made the grid unavailable whenever
    // odom -> robot TF lagged the projection stamp by more than 0.1 s: in MuJoCo lio runs that
    // was 106 of 161 moving stops (report 5.1.10 item 2), each a ready=false heartbeat that the
    // planner turns into FAILURE_MAP_UNREADY and the MPC into a fail-stop.
    if ((robot_unknown_clear_radius_ > 0.0 || robot_unknown_clear_footprint_) &&
      !lookupAtProjectionStamp(
        fusion_params_.output_in_projection_frame ? projection_grid.header.frame_id :
        static_map->header.frame_id, robot_frame_, projection_grid.header.stamp,
        static_from_robot)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap adapter waits for %s <- %s TF at the projection stamp",
        static_map->header.frame_id.c_str(), robot_frame_.c_str());
      publishUnavailable("ROGMap projection transform is unavailable");
      return;
    }

    // This prior has its own strict, timestamped TF. Never borrow the static-map
    // identity fallback or the bounded latest-TF policy above. Missing TF only
    // disables this optional prior; the planning grid must still be produced.
    auto current_fusion_params = fusion_params_;
    current_fusion_params.has_static_map = !static_map_is_live_fallback;
    geometry_msgs::msg::TransformStamped override_from_projection;
    const geometry_msgs::msg::TransformStamped * override_transform = nullptr;
    if (!fusion_params_.terrain_free_override_rects.empty() &&
      (!(mapping_mode_ || static_map_is_live_fallback) ||
      fusion_params_.diagnostic_allow_terrain_free_override) &&
      !fusion_params_.terrain_free_override_frame.empty() &&
      rclcpp::Time(projection_grid.header.stamp).nanoseconds() > 0)
    {
      try {
        override_from_projection = tf_buffer_.lookupTransform(
          fusion_params_.terrain_free_override_frame, projection_grid.header.frame_id,
          rclcpp::Time(projection_grid.header.stamp),
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
        // Static TFs may report stamp zero; lookup has already evaluated the
        // requested time, so record that query time for the fusion contract.
        override_from_projection.header.stamp = projection_grid.header.stamp;
        override_transform = &override_from_projection;
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Terrain rectangle disabled: strict projection-time TF unavailable: %s",
          exception.what());
      }
    }
    const auto t_fuse_start = std::chrono::steady_clock::now();
    GroundProjectionFusionResult fusion;
    const nav_msgs::msg::OccupancyGrid * tunnel_free_ptr =
      response.tunnel_free_grid.data.empty() ? nullptr : &response.tunnel_free_grid;
    if (!GroundProjectionFusion::fuse(
        projection_grid, *traversability, *slope, *static_map,
        static_from_projection, current_fusion_params, fusion, override_transform,
        tunnel_free_ptr))
    {
      publishUnavailable("ROGMap terrain fusion failed");
      return;
    }
    std::size_t ego_unknown_cleared = GroundProjectionFusion::clearUnknownCircle(
      fusion, static_from_robot.transform.translation.x, static_from_robot.transform.translation.y,
      robot_unknown_clear_radius_);
    if (robot_unknown_clear_footprint_) {
      ego_unknown_cleared += GroundProjectionFusion::clearUnknownFootprint(
        fusion, static_from_robot.transform.translation.x, static_from_robot.transform.translation.y,
        tf2::getYaw(static_from_robot.transform.rotation), robot_clear_length_, robot_clear_width_,
        robot_clear_margin_, fusion_params_.terrain_obstacle_value_threshold);
    }
    last_blocking_grid_ = fusion.planning_grid;
    if (fusion.known_free_cells == 0) {
      publishUnavailable("ROGMap terrain fusion produced no known-free cells", true);
      return;
    }

    const auto t_esdf_start = std::chrono::steady_clock::now();
    planning_esdf_.updateGrid(
      fusion.planning_grid, fusion_params_.terrain_obstacle_value_threshold,
      fusion_params_.unknown_is_obstacle, 100, nullptr, nullptr, nullptr, nullptr, false);
    std::vector<double> signed_distance;
    if (!planning_esdf_.copySignedDistanceField(signed_distance)) {
      last_blocking_grid_ = fusion.planning_grid;
      publishUnavailable("Fused RC-ESDF construction failed");
      return;
    }

    if (!localizationReady() || localization_epoch != localization_epoch_) {
      publishUnavailable(
          "localization changed before planning snapshot commit");
      return;
    }
    const auto t_snap_start = std::chrono::steady_clock::now();
    last_numeric_snapshot_ = numeric_snapshot;
    last_blocking_grid_ = fusion.planning_grid;
    const builtin_interfaces::msg::Time publication_stamp = now();
    const auto publication_sequence = map_status_sequence_ + 1U;
    const auto planning_snapshot = makePlanningMapSnapshot(
      fusion.planning_grid, signed_distance, publication_stamp, localization_epoch,
      response.generation, publication_sequence, fusion_params_.unknown_is_obstacle,
      fusion_params_.terrain_obstacle_value_threshold);
    if (!planning_snapshot.ready) {
      publishUnavailable("fused planning snapshot serialization failed");
      return;
    }
    const auto t_pub_start = std::chrono::steady_clock::now();
    publishPlanningGridViz(fusion.planning_grid);
    const double footprint_radius = std::hypot(
      0.5 * footprint_length_ + footprint_safety_margin_,
      0.5 * footprint_width_ + footprint_safety_margin_);
    if (esdf_cloud_pub_ && esdf_cloud_pub_->get_subscription_count() > 0U) {
      esdf_cloud_pub_->publish(
        encodeEsdfCloud(fusion.planning_grid, signed_distance));
    }
    if (clearance_contour_pub_ && clearance_contour_pub_->get_subscription_count() > 0U) {
      const double inscribed_radius =
        0.5 * std::min(footprint_length_, footprint_width_) + footprint_safety_margin_;
      clearance_contour_pub_->publish(encodeClearanceContours(
        fusion.planning_grid, signed_distance, inscribed_radius, footprint_radius));
    }
    if (generation_pub_ && generation_pub_->get_subscription_count() > 0U) {
      std_msgs::msg::UInt64 generation;
      generation.data = response.generation;
      generation_pub_->publish(generation);
    }
    last_rog_generation_ = response.generation;
    planning_snapshot_pub_->publish(planning_snapshot);
    publishMapStatus(true, response.generation, "planning snapshot ready");
    const auto t_done = std::chrono::steady_clock::now();
    const auto ms_between = [](auto a, auto b) {
      return 1000.0 * std::chrono::duration<double>(b - a).count();
    };
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "P2 snapshot generation=%llu free=%zu occupied=%zu unknown=%zu ego_clear=%zu numeric_esdf=%zu "
      "stage_ms(tf/fuse/esdf/snap/pub)=%.1f/%.1f/%.1f/%.1f/%.1f",
      static_cast<unsigned long long>(response.generation), fusion.known_free_cells,
      fusion.occupied_cells, fusion.unknown_cells, ego_unknown_cleared,
      numeric_snapshot.signed_distance.size(),
      ms_between(t_tf_start, t_fuse_start), ms_between(t_fuse_start, t_esdf_start),
      ms_between(t_esdf_start, t_snap_start), ms_between(t_snap_start, t_pub_start),
      ms_between(t_pub_start, t_done));
  }

  // unix_global_esdf 风格 5-field XYZ+distance+intensity 距离场点云：
  // 1) 跳过未知格（grid.data < 0）与非有限距离，消除场地外大块灰/红实体遮挡；
  // 2) 二分查找自适应步长控制点数上限 esdf_viz_max_points_；
  // 3) 保留原始有符号 distance，同时将 clamp(distance, 0, esdf_viz_max_distance_) 写入 intensity
  //    供 RViz Intensity Rainbow Flat Squares 渲染；z 置于栅格平面下方 0.02 m 避免遮挡路径。
  sensor_msgs::msg::PointCloud2 encodeEsdfCloud(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & distance_field) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = grid.header;
    const auto & info = grid.info;
    const std::size_t width = static_cast<std::size_t>(info.width);
    const std::size_t height = static_cast<std::size_t>(info.height);
    if (width == 0U || height == 0U || grid.data.size() < width * height ||
      distance_field.size() < width * height)
    {
      return cloud;
    }
    const auto gridCount = [&](std::size_t stride) {
        return ((width - 1U) / stride + 1U) * ((height - 1U) / stride + 1U);
      };
    std::size_t low = 1U;
    std::size_t high = std::max(width, height);
    while (low < high) {
      const std::size_t mid = low + (high - low) / 2U;
      if (gridCount(mid) > esdf_viz_max_points_) {
        low = mid + 1U;
      } else {
        high = mid;
      }
    }
    const std::size_t stride = low;

    std::vector<std::size_t> valid_indices;
    valid_indices.reserve(gridCount(stride));
    for (std::size_t row = 0U; row < height; row += stride) {
      const std::size_t row_offset = row * width;
      for (std::size_t col = 0U; col < width; col += stride) {
        const std::size_t idx = row_offset + col;
        if (grid.data[idx] < 0 || !std::isfinite(distance_field[idx])) {
          continue;
        }
        valid_indices.push_back(idx);
      }
    }

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(valid_indices.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_dist(cloud, "distance");
    sensor_msgs::PointCloud2Iterator<float> it_int(cloud, "intensity");
    const float z_plane = static_cast<float>(info.origin.position.z - 0.02);
    for (const std::size_t idx : valid_indices) {
      const double column = static_cast<double>(idx % width) + 0.5;
      const double row = static_cast<double>(idx / width) + 0.5;
      const auto point = cellToFrame(info, column, row);
      const double d = distance_field[idx];
      *it_x = static_cast<float>(point[0]);
      *it_y = static_cast<float>(point[1]);
      *it_z = z_plane;
      *it_dist = static_cast<float>(d);
      *it_int = static_cast<float>(std::clamp(d, 0.0, esdf_viz_max_distance_));
      ++it_x; ++it_y; ++it_z; ++it_dist; ++it_int;
    }
    cloud.is_dense = true;
    return cloud;
  }

  // (column, row) in cell units -> grid frame, honouring the origin yaw.
  static std::array<double, 2> cellToFrame(
    const nav_msgs::msg::MapMetaData & info, double column, double row)
  {
    const double yaw = tf2::getYaw(info.origin.orientation);
    const double lx = column * info.resolution;
    const double ly = row * info.resolution;
    return {
      info.origin.position.x + std::cos(yaw) * lx - std::sin(yaw) * ly,
      info.origin.position.y + std::sin(yaw) * lx + std::cos(yaw) * ly};
  }

  // Two iso-distance lines: inscribed half-width (any yaw collides inside it)
  // and circumscribed radius (no yaw collides outside it).
  visualization_msgs::msg::MarkerArray encodeClearanceContours(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & distance_field,
    double inscribed_radius, double circumscribed_radius) const
  {
    visualization_msgs::msg::MarkerArray markers;
    const struct
    {
      const char * ns;
      double level;
      float r, g, b;
    } lines[] = {
      {"inscribed", inscribed_radius, 0.90F, 0.10F, 0.10F},
      {"circumscribed", circumscribed_radius, 1.00F, 0.65F, 0.00F}};
    for (const auto & line : lines) {
      visualization_msgs::msg::Marker marker;
      marker.header = grid.header;
      marker.ns = line.ns;
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.color.r = line.r;
      marker.color.g = line.g;
      marker.color.b = line.b;
      marker.color.a = 1.0F;
      const auto segments = distanceContourSegments(
        distance_field, grid.data, grid.info.width, grid.info.height, line.level);
      marker.action = segments.empty() ?
        visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
      marker.points.reserve(2U * segments.size());
      for (const auto & segment : segments) {
        for (int end = 0; end < 2; ++end) {
          const auto point = cellToFrame(grid.info, segment[2 * end], segment[2 * end + 1]);
          geometry_msgs::msg::Point p;
          p.x = point[0];
          p.y = point[1];
          p.z = grid.info.origin.position.z + 0.01;
          marker.points.push_back(p);
        }
      }
      markers.markers.push_back(std::move(marker));
    }
    return markers;
  }

  bool planningGridVizWanted() const
  {
    return planning_grid_viz_pub_ && planning_grid_viz_pub_->get_subscription_count() > 0U;
  }

  void publishPlanningGridViz(const nav_msgs::msg::OccupancyGrid & grid)
  {
    if (planningGridVizWanted()) {
      planning_grid_viz_pub_->publish(grid);
    }
  }

  void publishBlockedGrid(const nav_msgs::msg::OccupancyGrid & source)
  {
    if (!planningGridVizWanted() || source.info.width == 0 || source.info.height == 0 ||
      source.data.empty())
    {
      return;
    }
    nav_msgs::msg::OccupancyGrid blocked = source;
    blocked.data.assign(source.data.size(), -1);
    planning_grid_viz_pub_->publish(blocked);
  }

  void publishUnavailable(const char * reason, bool include_blocking_grid = false)
  {
    const auto publication_sequence = map_status_sequence_ + 1U;
    const builtin_interfaces::msg::Time publication_stamp = now();
    if (include_blocking_grid) {
      planning_snapshot_pub_->publish(makeBlockedUnavailablePlanningMapSnapshot(
        last_blocking_grid_, publication_stamp, localization_epoch_, last_rog_generation_,
        publication_sequence, fusion_params_.unknown_is_obstacle,
        fusion_params_.terrain_obstacle_value_threshold));
    } else {
      planning_snapshot_pub_->publish(makeUnavailablePlanningMapSnapshot(
        last_blocking_grid_, publication_stamp, localization_epoch_, last_rog_generation_,
        publication_sequence, fusion_params_.unknown_is_obstacle,
        fusion_params_.terrain_obstacle_value_threshold));
    }
    publishMapStatus(false, last_rog_generation_, reason);
    publishBlockedGrid(last_blocking_grid_);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason);
  }

  void publishMapStatus(bool ready, std::uint64_t rog_generation,
                        const std::string &reason) {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_pub_->publish(message);

    ats_navigation_interfaces::msg::PlanningMapStatus status;
    status.header.stamp = now();
    status.header.frame_id = last_blocking_grid_.header.frame_id;
    status.ready = ready;
    status.localization_epoch = localization_epoch_;
    status.rog_generation = rog_generation;
    status.publication_sequence = ++map_status_sequence_;
    status.message = reason;
    map_status_pub_->publish(status);
    RCLCPP_INFO(
      get_logger(),
      "P2 adapter heartbeat ready=%d publication_sequence=%llu source_generation=%llu "
      "localization_epoch=%llu stamp_ns=%lld reason=%s",
      ready ? 1 : 0, static_cast<unsigned long long>(status.publication_sequence),
      static_cast<unsigned long long>(status.rog_generation),
      static_cast<unsigned long long>(status.localization_epoch),
      static_cast<long long>(rclcpp::Time(status.header.stamp).nanoseconds()), reason.c_str());
  }

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
  double projection_rate_hz_{2.0};
  double projection_request_timeout_sec_{2.0};
  double projection_snapshot_timeout_sec_{2.0};
  double projection_min_height_{0.10};
  double projection_max_height_{0.80};
  double planning_resolution_{0.10};
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

std::shared_ptr<rclcpp::Node> makePlanningMapPublisher(
  GroundProjectionProvider provider, const rclcpp::NodeOptions & options)
{
  return std::make_shared<PlanningMapPublisher>(std::move(provider), options);
}

}  // namespace ats_rog_map::planning
