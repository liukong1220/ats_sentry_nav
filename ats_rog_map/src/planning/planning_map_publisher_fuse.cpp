#include "planning_map_publisher_impl.hpp"

namespace ats_rog_map::planning
{

void PlanningMapPublisher::processProjection(
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
  const bool projection_ready = response.odom_layer_included
    ? (response.ready && !response.stale &&
       response.occupancy_grid.info.resolution > 0.0 &&
       response.occupancy_grid.info.width > 0U && response.occupancy_grid.info.height > 0U &&
       response.occupancy_grid.data.size() ==
         static_cast<std::size_t>(response.occupancy_grid.info.width) *
         static_cast<std::size_t>(response.occupancy_grid.info.height))
    : numeric_snapshot.available();
  if (!projection_ready || !projectionFresh(projection_grid)) {
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
  if (fusion.known_free_cells == 0) {
    last_blocking_grid_ = fusion.planning_grid;
    publishUnavailable("ROGMap terrain fusion produced no known-free cells", true);
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
  } else if (!static_map_is_live_fallback) {
    // Partial-map reload support: if a reloaded prior map has known-free space elsewhere
    // but leaves an unobserved LiDAR blind spot directly under the robot's spawn pose,
    // clear only unoccluded unknown cells inside the robot footprint (never on all-unknown
    // grids and zero-op on full prior maps where the robot cell is already known-free).
    try {
      const auto robot_tf = tf_buffer_.lookupTransform(
        fusion.planning_grid.header.frame_id, robot_frame_, tf2::TimePointZero);
      const double rx = robot_tf.transform.translation.x;
      const double ry = robot_tf.transform.translation.y;
      const auto & g = fusion.planning_grid;
      const double dx = rx - g.info.origin.position.x;
      const double dy = ry - g.info.origin.position.y;
      const double oyaw = tf2::getYaw(g.info.origin.orientation);
      const double lx = std::cos(oyaw) * dx + std::sin(oyaw) * dy;
      const double ly = -std::sin(oyaw) * dx + std::cos(oyaw) * dy;
      const int ix = static_cast<int>(std::floor(lx / g.info.resolution));
      const int iy = static_cast<int>(std::floor(ly / g.info.resolution));
      const int r_fp = std::max(1, static_cast<int>(std::ceil(0.35 / g.info.resolution)));
      bool footprint_has_unknown = false;
      for (int dy = -r_fp; dy <= r_fp && !footprint_has_unknown; ++dy) {
        const int ny = iy + dy;
        if (ny < 0 || ny >= static_cast<int>(g.info.height)) {continue;}
        for (int dx = -r_fp; dx <= r_fp; ++dx) {
          const int nx = ix + dx;
          if (nx < 0 || nx >= static_cast<int>(g.info.width)) {continue;}
          if (g.data[static_cast<std::size_t>(ny) * g.info.width + static_cast<std::size_t>(nx)] < 0) {
            footprint_has_unknown = true;
            break;
          }
        }
      }
      if (footprint_has_unknown) {
        ego_unknown_cleared += GroundProjectionFusion::clearUnknownFootprint(
          fusion, rx, ry, tf2::getYaw(robot_tf.transform.rotation),
          robot_clear_length_, robot_clear_width_, robot_clear_margin_,
          fusion_params_.terrain_obstacle_value_threshold);
      }
    } catch (const tf2::TransformException &) {
    }
  }
  if (mapping_mode_ && !fusion.planning_grid.data.empty()) {
    const std::size_t despeckled = OdomLayerStore::filterIsolatedOccupiedSpeckles(
      fusion.planning_grid.data, fusion.planning_grid.info.width,
      fusion.planning_grid.info.height, 2,
      fusion_params_.terrain_obstacle_value_threshold);
    if (despeckled > 0U) {
      fusion.known_free_cells += despeckled;
    }
    const std::size_t bridged = OdomLayerStore::fillIsolatedUnknownGaps(
      fusion.planning_grid.data, fusion.planning_grid.info.width,
      fusion.planning_grid.info.height, 9, 3,
      fusion_params_.terrain_obstacle_value_threshold);
    if (bridged > 0U) {
      fusion.known_free_cells += bridged;
      fusion.unknown_cells = fusion.unknown_cells >= bridged ?
        fusion.unknown_cells - bridged : 0U;
    }
  }
  last_blocking_grid_ = fusion.planning_grid;

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
  std::optional<nav_msgs::msg::OccupancyGrid> global_grid;
  if (response.odom_layer_included && !response.global_occupancy_grid.data.empty() &&
    response.global_occupancy_grid.header.frame_id == fusion.planning_grid.header.frame_id &&
    response.global_occupancy_grid.info.resolution > 0.0)
  {
    global_grid = response.global_occupancy_grid;
    // Overlay live local fused obstacles (from fusion.planning_grid) onto the same-generation
    // global layer inside the current 12x12 m window so global and local views are consistent,
    // while letting odom_layer (with GroundColumnGrid wall/step/drop-off/rough-bump & tunnel
    // headroom evidence) remain the authoritative persistent 40x30 m obstacle map without
    // permanently latching distant terrain_analysis_ext overhang false positives.
    const auto & fg = fusion.planning_grid;
    const double f_res = fg.info.resolution;
    const double f_ox = fg.info.origin.position.x;
    const double f_oy = fg.info.origin.position.y;
    const double f_yaw = tf2::getYaw(fg.info.origin.orientation);
    const double f_cos = std::cos(f_yaw);
    const double f_sin = std::sin(f_yaw);
    const double g_inv_res = 1.0 / global_grid->info.resolution;
    const double g_ox = global_grid->info.origin.position.x;
    const double g_oy = global_grid->info.origin.position.y;
    const int g_w = static_cast<int>(global_grid->info.width);
    const int g_h = static_cast<int>(global_grid->info.height);
    const int occ_thr = fusion_params_.terrain_obstacle_value_threshold;
    const bool aligned_same_res =
      std::abs(f_yaw) <= 1e-6 && std::abs(f_res - global_grid->info.resolution) <= 1e-6;
    const int aligned_sx = aligned_same_res
      ? static_cast<int>(std::llround((f_ox - g_ox) * g_inv_res)) : 0;
    const int aligned_sy = aligned_same_res
      ? static_cast<int>(std::llround((f_oy - g_oy) * g_inv_res)) : 0;
    for (std::uint32_t fy = 0; fy < fg.info.height; ++fy) {
      const double ly = (static_cast<double>(fy) + 0.5) * f_res;
      const std::size_t frow = static_cast<std::size_t>(fy) * fg.info.width;
      for (std::uint32_t fx = 0; fx < fg.info.width; ++fx) {
        const std::int8_t fv = fg.data[frow + fx];
        if (fv < occ_thr) {continue;}
        int gx = 0;
        int gy = 0;
        if (aligned_same_res) {
          gx = aligned_sx + static_cast<int>(fx);
          gy = aligned_sy + static_cast<int>(fy);
        } else {
          const double lx = (static_cast<double>(fx) + 0.5) * f_res;
          const double wx = f_ox + f_cos * lx - f_sin * ly;
          const double wy = f_oy + f_sin * lx + f_cos * ly;
          gx = static_cast<int>(std::floor((wx - g_ox) * g_inv_res));
          gy = static_cast<int>(std::floor((wy - g_oy) * g_inv_res));
        }
        if (gx >= 0 && gy >= 0 && gx < g_w && gy < g_h) {
          const std::size_t gidx =
            static_cast<std::size_t>(gy) * static_cast<std::size_t>(g_w) +
            static_cast<std::size_t>(gx);
          global_grid->data[gidx] = 100;
        }
      }
    }
    OdomLayerStore::filterIsolatedOccupiedSpeckles(
      global_grid->data, global_grid->info.width, global_grid->info.height, 2, occ_thr);
  }
  const builtin_interfaces::msg::Time publication_stamp = now();
  const auto publication_sequence = map_status_sequence_ + 1U;
  const auto planning_snapshot = makePlanningMapSnapshot(
    fusion.planning_grid, signed_distance, publication_stamp, localization_epoch,
    response.generation, publication_sequence, fusion_params_.unknown_is_obstacle,
    fusion_params_.terrain_obstacle_value_threshold,
    global_grid ? &(*global_grid) : nullptr);
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
sensor_msgs::msg::PointCloud2 PlanningMapPublisher::encodeEsdfCloud(
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
std::array<double, 2> PlanningMapPublisher::cellToFrame(
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
visualization_msgs::msg::MarkerArray PlanningMapPublisher::encodeClearanceContours(
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

bool PlanningMapPublisher::planningGridVizWanted() const
{
  return planning_grid_viz_pub_ && planning_grid_viz_pub_->get_subscription_count() > 0U;
}

void PlanningMapPublisher::publishPlanningGridViz(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (planningGridVizWanted()) {
    planning_grid_viz_pub_->publish(grid);
  }
}

void PlanningMapPublisher::publishBlockedGrid(const nav_msgs::msg::OccupancyGrid & source)
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

void PlanningMapPublisher::publishUnavailable(const char * reason, bool include_blocking_grid)
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

void PlanningMapPublisher::publishMapStatus(bool ready, std::uint64_t rog_generation,
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

}  // namespace ats_rog_map::planning
