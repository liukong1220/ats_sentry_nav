#include "planning_map_publisher_impl.hpp"

namespace ats_rog_map::planning
{

PlanningMapPublisher::PlanningMapPublisher(GroundProjectionProvider provider, const rclcpp::NodeOptions & options)
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
  planning_resolution_ = std::max(0.01, declare_parameter<double>("planning_resolution", 0.05));
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
  // In mapping_mode, /rog_map/odom_layer_updated triggers projection immediately on every
  // committed cloud; keep the periodic timer as a low-overhead keepalive (4 Hz) so it never
  // saturates CPU or contends with event-driven cloud updates.
  const double mapping_rate = std::max(
    0.1, declare_parameter<double>("mapping_projection_rate_hz", 4.0));
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
  esdf_viz_use_global_in_mapping_ = declare_parameter<bool>(
    "esdf_viz_use_global_in_mapping", true);
  esdf_viz_global_rate_hz_ = std::max(
    0.2, declare_parameter<double>("esdf_viz_global_rate_hz", 2.0));
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
  const auto snapshot_qos = rclcpp::QoS(4).reliable().transient_local();
  planning_snapshot_pub_ = create_publisher<
    ats_navigation_interfaces::msg::PlanningMapSnapshot>(planning_snapshot_topic_, snapshot_qos);
  projection_worker_ = std::thread([this]() {runProjectionWorker();});
  if (mapping_mode_ && esdf_viz_use_global_in_mapping_ &&
    (esdf_cloud_pub_ || clearance_contour_pub_))
  {
    global_viz_worker_ = std::thread([this]() {runGlobalEsdfVizWorker();});
  }
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
      if (mapping_mode_ && last_projection_complete_time_.has_value()) {
        const double since_last = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - *last_projection_complete_time_).count();
        if (since_last < 0.20) {
          return;
        }
      }
      requestProjection();
    });
  RCLCPP_INFO(
    get_logger(),
    "ROGMap ground adapter ready: in-process projection planning_snapshot='%s' height=[%.2f, %.2f] m",
    planning_snapshot_topic_.c_str(), projection_min_height_, projection_max_height_);
}

PlanningMapPublisher::~PlanningMapPublisher()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stop_ = true;
  }
  worker_cv_.notify_all();
  if (projection_worker_.joinable()) {
    projection_worker_.join();
  }
  {
    std::lock_guard<std::mutex> lock(global_viz_mutex_);
    global_viz_stop_ = true;
  }
  global_viz_cv_.notify_all();
  if (global_viz_worker_.joinable()) {
    global_viz_worker_.join();
  }
}

bool PlanningMapPublisher::localizationStateAllowsPlanning(std::uint8_t state)
{
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  // CONFIRMED: post-accept hold with map-frame evidence.
  // DEGRADED: soft quality loss / consecutive rejects; map-odom still held.
  // LOST/BOOTSTRAP/RELOCALIZING/UNINITIALIZED remain fail-closed.
  return state == LS::STATE_TRACKING || state == LS::STATE_CONFIRMED ||
         state == LS::STATE_DEGRADED;
}


void PlanningMapPublisher::onLocalizationStatus(
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

bool PlanningMapPublisher::localizationReady() const {
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

bool PlanningMapPublisher::inputFresh(const nav_msgs::msg::OccupancyGrid & grid) const
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

bool PlanningMapPublisher::projectionFresh(const nav_msgs::msg::OccupancyGrid & grid) const
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

bool PlanningMapPublisher::inputSynchronized(
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

// Caller holds state_mutex_.
void PlanningMapPublisher::requestProjection()
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

void PlanningMapPublisher::runProjectionWorker()
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

bool PlanningMapPublisher::workerStopping()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return worker_stop_;
}

// Caller holds state_mutex_. Same checks as the former service response callback.
void PlanningMapPublisher::completeProjection(
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
    last_projection_complete_time_ = std::chrono::steady_clock::now();
    if (mapping_update_pending_) {
      mapping_update_pending_ = false;
      requestProjection();
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "ROGMap projection response failed: %s", exception.what());
    publishUnavailable("ROGMap projection response failed");
  }
}

std::shared_ptr<rclcpp::Node> makePlanningMapPublisher(
  GroundProjectionProvider provider, const rclcpp::NodeOptions & options)
{
  return std::make_shared<PlanningMapPublisher>(std::move(provider), options);
}

}  // namespace ats_rog_map::planning
