#include "ats_rog_map_node_impl.hpp"

namespace ats_rog_map
{

void AtsRogMapNode::onLocalizationStatus(
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

void AtsRogMapNode::saveMap(
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

void AtsRogMapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::lock_guard<std::mutex> epoch_lock(epoch_mutex_);
  if (require_odom_identity_ &&
    !odom_epoch_guard_.accepts(rclcpp::Time(message->header.stamp).nanoseconds())) {return;}
  std::lock_guard<std::mutex> lock(input_mutex_);
  last_odom_receive_time_ = std::chrono::steady_clock::now();
}

bool AtsRogMapNode::lookupTransform(
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

void AtsRogMapNode::deferCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // This subscription and the retry timer share a mutually exclusive group.
  // Keep only the newest cloud while TF is pending, just as the input QoS
  // bounds queued work. TF listener callbacks remain free to fill the buffer.
  pending_cloud_ = msg;
  pending_cloud_receive_time_ = std::chrono::steady_clock::now();
  tryProcessCloud();
}

void AtsRogMapNode::tryProcessCloud()
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


// The global odom layer has independent synchronization. Its obstacle hits
// use ground-relative columns, including the headroom check, not base z.
void AtsRogMapNode::accumulateOdomLayer(
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
  // Mark unknown LiDAR blind-spot cells directly underneath the robot's physical
  // footprint (never overwriting occupied cells) so saved partial maps remain
  // navigable from the start pose upon reload.
  if (!points.empty()) {
    odom_layer_->markUnknownFootprintFree(base.x, base.y, 0.36, 0.36, stamp_seconds);
  }
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

nav_msgs::msg::OccupancyGrid AtsRogMapNode::sampleLayerBuffer(
  const std::vector<std::int8_t> & full_cells, const OdomLayerConfig & c,
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

  // Fast path when output aligns with the odom layer lattice (full grid or 1:1 sub-window).
  // Use 1e-5 / 1e-4 tolerances so float32 OccupancyGrid::info.resolution (0.05f = 0.050000000745)
  // never drops out of the 1:1 copy path into a 2x2 dilating fallback.
  if (std::abs(resolution - c.resolution) <= 1e-5) {
    const double fx_d = (origin_x - c.origin_x) / c.resolution;
    const double fy_d = (origin_y - c.origin_y) / c.resolution;
    const int sx = static_cast<int>(std::llround(fx_d));
    const int sy = static_cast<int>(std::llround(fy_d));
    if (std::abs(fx_d - static_cast<double>(sx)) <= 1e-4 &&
      std::abs(fy_d - static_cast<double>(sy)) <= 1e-4)
    {
      if (sx == 0 && sy == 0 && width == c.width && height == c.height) {
        grid.data = full_cells;
        return grid;
      }
      const int base_w = static_cast<int>(c.width);
      const int base_h = static_cast<int>(c.height);
      const int y_beg = std::max(0, -sy);
      const int y_end = std::min(static_cast<int>(height), base_h - sy);
      const int x_beg = std::max(0, -sx);
      const int x_end = std::min(static_cast<int>(width), base_w - sx);
      if (x_beg < x_end && y_beg < y_end) {
        const std::size_t copy_len = static_cast<std::size_t>(x_end - x_beg);
        for (int y = y_beg; y < y_end; ++y) {
          const std::size_t src_off =
            static_cast<std::size_t>(sy + y) * static_cast<std::size_t>(base_w) +
            static_cast<std::size_t>(sx + x_beg);
          const std::size_t dst_off =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x_beg);
          std::copy_n(full_cells.data() + src_off, copy_len, grid.data.data() + dst_off);
        }
      }
      return grid;
    }
  }

  const double inv_base = 1.0 / c.resolution;
  const int base_w = static_cast<int>(c.width);
  const int base_h = static_cast<int>(c.height);
  for (std::uint32_t y = 0; y < height; ++y) {
    const double min_y = origin_y + y * resolution;
    const double max_y = origin_y + (y + 1U) * resolution;
    const double y0 = (min_y - c.origin_y) * inv_base;
    const double y1 = (max_y - c.origin_y) * inv_base;
    const int fy = static_cast<int>(std::floor(std::clamp(y0 + 1e-4, 0.0, static_cast<double>(base_h))));
    const int ly = static_cast<int>(std::ceil(std::clamp(y1 - 1e-4, 0.0, static_cast<double>(base_h)))) - 1;
    const bool y_in_bounds = y0 >= -1e-4 && y1 <= static_cast<double>(base_h) + 1e-4;
    for (std::uint32_t x = 0; x < width; ++x) {
      const double min_x = origin_x + x * resolution;
      const double max_x = origin_x + (x + 1U) * resolution;
      const double x0 = (min_x - c.origin_x) * inv_base;
      const double x1 = (max_x - c.origin_x) * inv_base;
      const int fx = static_cast<int>(std::floor(std::clamp(x0 + 1e-4, 0.0, static_cast<double>(base_w))));
      const int lx = static_cast<int>(std::ceil(std::clamp(x1 - 1e-4, 0.0, static_cast<double>(base_w)))) - 1;
      if (fx > lx || fy > ly) {
        grid.data[static_cast<std::size_t>(y) * width + x] = -1;
        continue;
      }
      bool all_free = y_in_bounds && x0 >= -1e-4 && x1 <= static_cast<double>(base_w) + 1e-4;
      std::int8_t out = -1;
      bool hit_occ = false;
      for (int by = fy; by <= ly && !hit_occ; ++by) {
        const std::size_t row = static_cast<std::size_t>(by) * static_cast<std::size_t>(base_w);
        for (int bx = fx; bx <= lx; ++bx) {
          const std::int8_t v = full_cells[row + static_cast<std::size_t>(bx)];
          if (v >= 50) {
            out = 100;
            hit_occ = true;
            break;
          }
          if (v < 0) {
            all_free = false;
          }
        }
      }
      if (!hit_occ) {
        out = all_free ? 0 : -1;
      }
      grid.data[static_cast<std::size_t>(y) * width + x] = out;
    }
  }
  return grid;
}

void AtsRogMapNode::getOdomLayerProjection(
  float requested_resolution, GroundProjection & response)
{
  const auto trace_begin = std::chrono::steady_clock::now();
  // epoch_mutex_ fences the map generation and the layer commit together.
  // Odom-layer projection reads odom_layer_ directly and does not use 3D ROGMap ESDF.
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    response.generation = map_->generation();
    response.ready = has_map_data_;
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
    const auto & cells = odom_layer_->cells();
    std::vector<std::int8_t> full_cells(cells.size(), -1);
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const auto s = cells[i].state;
      if (s == OdomCellState::kOccupied) {
        full_cells[i] = 100;
      } else if (s == OdomCellState::kFree || s == OdomCellState::kTunnelFree) {
        full_cells[i] = 0;
      }
    }
    const double req_res = static_cast<double>(requested_resolution);
    const double resolution =
      (std::abs(req_res - c.resolution) <= 1e-5) ? c.resolution : std::max(c.resolution, req_res);
    const double win_ox = c.origin_x +
      std::floor((layer_center_x_ - 0.5 * layer_window_x_ - c.origin_x) / resolution) * resolution;
    const double win_oy = c.origin_y +
      std::floor((layer_center_y_ - 0.5 * layer_window_y_ - c.origin_y) / resolution) * resolution;
    const auto win_w = static_cast<std::uint32_t>(std::ceil(layer_window_x_ / resolution));
    const auto win_h = static_cast<std::uint32_t>(std::ceil(layer_window_y_ / resolution));
    response.occupancy_grid = sampleLayerBuffer(
      full_cells, c, win_ox, win_oy, win_w, win_h, resolution);
    // Remove 1..2 cell floating LiDAR speckles strictly enclosed by known-free floor
    // and conservatively bridge tiny isolated unknown scan gaps on the local window.
    OdomLayerStore::filterIsolatedOccupiedSpeckles(
      response.occupancy_grid.data, response.occupancy_grid.info.width,
      response.occupancy_grid.info.height);
    OdomLayerStore::fillIsolatedUnknownGaps(
      response.occupancy_grid.data, response.occupancy_grid.info.width,
      response.occupancy_grid.info.height);

    // Export global persistent odom layer (40 x 30 m at unified 5 cm resolution, no global ESDF)
    // atomically with the local window for two-layer optimistic-global + conservative-local planning.
    const double coarse_res = std::max(c.resolution, layer_global_coarse_resolution_);
    const auto coarse_w = static_cast<std::uint32_t>(
      std::ceil((static_cast<double>(c.width) * c.resolution) / coarse_res));
    const auto coarse_h = static_cast<std::uint32_t>(
      std::ceil((static_cast<double>(c.height) * c.resolution) / coarse_res));
    response.global_occupancy_grid = sampleLayerBuffer(
      full_cells, c, c.origin_x, c.origin_y, coarse_w, coarse_h, coarse_res);
    OdomLayerStore::filterIsolatedOccupiedSpeckles(
      response.global_occupancy_grid.data, response.global_occupancy_grid.info.width,
      response.global_occupancy_grid.info.height);

    if (c.tunnel_free_enabled) {
      response.tunnel_free_grid.header = response.occupancy_grid.header;
      response.tunnel_free_grid.info = response.occupancy_grid.info;
      response.tunnel_free_grid.data.assign(
        static_cast<std::size_t>(win_w) * win_h, 0);
      for (std::uint32_t y = 0; y < win_h; ++y) {
        for (std::uint32_t x = 0; x < win_w; ++x) {
          if (odom_layer_->sampleTunnelFree(
              win_ox + x * resolution, win_oy + y * resolution,
              win_ox + (x + 1) * resolution, win_oy + (y + 1) * resolution))
          {
            response.tunnel_free_grid.data[static_cast<std::size_t>(y) * win_w + x] = 100;
          }
        }
      }
    }
    trace_grid = std::chrono::steady_clock::now();
    if (odom_layer_pub_->get_subscription_count() > 0) {
      odom_layer_debug = sampleLayerBuffer(
        full_cells, c, c.origin_x, c.origin_y, c.width, c.height, c.resolution);
    }
  }
  if (odom_layer_debug) {
    odom_layer_pub_->publish(*odom_layer_debug);
  }
  trace_debug = std::chrono::steady_clock::now();
  // PlanningMapPublisher computes the authoritative 2D RC-ESDF after terrain/slope fusion,
  // so skip redundant throwaway 2D ESDF + per-cell gradient computation here.
  response.signed_distance.clear();
  response.gradient_x.clear();
  response.gradient_y.clear();
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


void AtsRogMapNode::publishHealth()
{
  bool map_update_stale = true;
  bool odom_stale = true;
  bool raw_cloud_stale = true;
  getHealthStaleness(map_update_stale, odom_stale, raw_cloud_stale);
  if (stale_pub_ && stale_pub_->get_subscription_count() > 0U) {
    std_msgs::msg::Bool message;
    message.data = map_update_stale || odom_stale;
    stale_pub_->publish(message);
  }
  if (map_update_stale || odom_stale) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ROGMap stale: map_update=%s odom=%s raw_cloud=%s",
      map_update_stale ? "true" : "false", odom_stale ? "true" : "false",
      raw_cloud_stale ? "true" : "false");
  }
}

void AtsRogMapNode::getHealthStaleness(
  bool & map_update_stale, bool & odom_stale, bool & raw_cloud_stale)
{
  const InputHealth health = inputHealth();
  map_update_stale = health.map_update_stale;
  odom_stale = health.odom_stale;
  raw_cloud_stale = health.raw_cloud_stale;
}


AtsRogMapNode::InputHealth AtsRogMapNode::inputHealth()
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


void AtsRogMapNode::getGroundProjection(const GroundProjectionRequest & request_in, GroundProjection & response_out)
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
  const bool use_ground = projection_follow_measured_ground_;
  const bool export_tunnel_free = odom_layer_->config().tunnel_free_enabled;
  std::unique_lock<std::mutex> layer_lock(layer_mutex_, std::defer_lock);
  if (use_ground || export_tunnel_free) {
    layer_lock.lock();
  }
  const auto & layer_config = odom_layer_->config();
  const auto & column_config = ground_columns_->config();
  if (export_tunnel_free) {
    response->tunnel_free_grid.header = grid.header;
    response->tunnel_free_grid.info = grid.info;
    response->tunnel_free_grid.data.assign(cell_count, 0);
  }
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
      double cell_z_min = z_min;
      double cell_z_max = z_max;
      const double x0 = grid.info.origin.position.x + mx * resolution;
      const double y0 = grid.info.origin.position.y + my * resolution;
      if (export_tunnel_free &&
        odom_layer_->sampleTunnelFree(x0, y0, x0 + resolution, y0 + resolution))
      {
        response->tunnel_free_grid.data[index] = 100;
      }
      if (use_ground) {
        const int value = odom_layer_->sampleFootprint(x0, y0, x0 + resolution, y0 + resolution);
        occupied = value == 100;
        if (value == 0) {
          const int gx = static_cast<int>(std::floor((x - layer_config.origin_x) /
            layer_config.resolution));
          const int gy = static_cast<int>(std::floor((y - layer_config.origin_y) /
            layer_config.resolution));
          if (const auto ground = ground_columns_->ground(gx, gy)) {
            // Observed connected support establishes the swept height band. The
            // numerical map can still block a column with a new return in that band.
            cell_z_min = std::max(static_cast<double>(box_min.z()),
              *ground + column_config.surface_max);
            cell_z_max = std::min(static_cast<double>(box_max.z()),
              *ground + column_config.robot_height + column_config.top_margin);
            if (ground_columns_->state(gx, gy) == ColumnState::kTunnelFree) {
              if (const auto headroom = ground_columns_->ceilingHeadroom(gx, gy)) {
                const double ceiling_voxel_bottom =
                  std::floor(headroom->ceiling_low / z_step) * z_step - 1e-4;
                cell_z_max = std::min(cell_z_max, ceiling_voxel_bottom);
              }
            }
            known_free = true;
          }
        }
      }
      double minimum_distance = std::numeric_limits<double>::infinity();
      for (double z = cell_z_min + 0.5 * z_step; z <= cell_z_max + 1e-6; z += z_step) {
        const rog_map::Vec3f point(x, y, static_cast<float>(std::min(z, cell_z_max)));
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
  if (layer_lock.owns_lock()) {
    layer_lock.unlock();
  }
  sample_ms = 1000.0 * std::chrono::duration<double>(
    std::chrono::steady_clock::now() - sample_started).count();

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
}


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
