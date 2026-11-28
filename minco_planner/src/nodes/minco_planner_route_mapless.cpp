// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

GridSearchResult MincoPlannerNode::planMaplessRoute(
  const std::shared_ptr<const PlanningMapSnapshot> & snapshot,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const GoalUuid & goal_uuid,
  std::uint64_t goal_id, std::uint64_t localization_epoch, bool goal_pose_footprint_verified,
  double & used_clearance,
  nav_msgs::msg::Path * global_guide_out,
  std::optional<geometry_msgs::msg::PoseStamped> * local_anchor_out)
{
  // Two-layer mapless planning (Wave 6B):
  // Layer 1 (Global, 40x30 m @ 0.20 m): optimistic hybrid JPS/A* (unknown traversable with
  //   gentle penalty) + global_route_ caching + new-obstacle replanning.
  // Layer 2 (Local, 12x12 m @ 0.05-0.10 m): strictly known-free conservative anchor & JPS/A*
  //   -> MINCO S3 + swept rectangular footprint gate.
  const auto & grid = snapshot->grid;
  const auto * squared = snapshot->blocked_squared_cells.get();
  const auto now_steady = std::chrono::steady_clock::now();
  const double cur_start_yaw = tf2::getYaw(start.pose.orientation);
  if (frontier_taboo_goal_id_ != goal_id || frontier_taboo_epoch_ != localization_epoch) {
    if (!frontier_taboo_.empty()) {
      frontier_taboo_.clear();
      ++frontier_taboo_version_;
    }
    last_partial_anchor_.reset();
    last_partial_start_.reset();
    last_unknown_frontier_.reset();
    mapless_stall_origin_ = start.pose.position;
    mapless_long_stall_origin_ = start.pose.position;
    mapless_stall_yaw_ = cur_start_yaw;
    mapless_stall_time_ = now_steady;
    mapless_long_stall_time_ = now_steady;
    mapless_consecutive_rejects_ = 0;
    mapless_optimistic_obstructed_count_ = 0;
    frontier_taboo_goal_id_ = goal_id;
    frontier_taboo_epoch_ = localization_epoch;
  } else {
    const std::size_t taboo_before = frontier_taboo_.size();
    frontier_taboo_.erase(
      std::remove_if(
        frontier_taboo_.begin(), frontier_taboo_.end(),
        [&](const FrontierTabooEntry & e) {
          return std::chrono::duration<double>(now_steady - e.added_at).count() >
                 frontier_taboo_ttl_sec_;
        }),
      frontier_taboo_.end());
    if (frontier_taboo_.size() != taboo_before) {
      ++frontier_taboo_version_;
    }
    // Two-horizon stall detector:
    // 1) Short horizon (>= 0.35 m): lets slow narrow-tunnel pre-alignment/transit (~0.15-0.30 m/s)
    //    proceed without false taboo while catching stationary cliff stalls within 4 s.
    // 2) Long horizon (>= 1.10 m): catches back-and-forth ~0.4 m cliff oscillation while never
    //    prematurely banning an uninspected frontier while the LiDAR is still turning (facing=0).
    const bool moved_short = !mapless_stall_origin_ ||
      std::hypot(
        start.pose.position.x - mapless_stall_origin_->x,
        start.pose.position.y - mapless_stall_origin_->y) >= 0.35;
    const bool rotated_short =
      std::abs(std::remainder(cur_start_yaw - mapless_stall_yaw_, 2.0 * M_PI)) >= 0.35;
    if (moved_short || rotated_short) {
      if (moved_short) {
        mapless_stall_origin_ = start.pose.position;
      }
      mapless_stall_yaw_ = cur_start_yaw;
      mapless_stall_time_ = now_steady;
      mapless_consecutive_rejects_ = 0;
    }
    const bool moved_long = !mapless_long_stall_origin_ ||
      std::hypot(
        start.pose.position.x - mapless_long_stall_origin_->x,
        start.pose.position.y - mapless_long_stall_origin_->y) >= 1.10;
    if (moved_long) {
      mapless_long_stall_origin_ = start.pose.position;
      mapless_long_stall_time_ = now_steady;
    }
    const auto lidar_fov_offset = resolveLidarFovYawOffsetRad();
    bool facing_frontier = true;
    bool frontier_near = false;
    if (last_unknown_frontier_) {
      const double f_dist = std::hypot(
        last_unknown_frontier_->x - start.pose.position.x,
        last_unknown_frontier_->y - start.pose.position.y);
      frontier_near = f_dist <= 2.5;
      if (lidar_fov_offset.has_value()) {
        const double f_yaw = std::atan2(
          last_unknown_frontier_->y - start.pose.position.y,
          last_unknown_frontier_->x - start.pose.position.x);
        const double lidar_look_yaw = std::remainder(
          cur_start_yaw + *lidar_fov_offset, 2.0 * M_PI);
        // A 30-deg tilted Mid-360 only has dense near-floor coverage within ~25 deg of its
        // primary observation bearing; outside ~0.45 rad, side-lobe floor blind spots exceed 1 m.
        facing_frontier = std::abs(std::remainder(f_yaw - lidar_look_yaw, 2.0 * M_PI)) <= 0.45;
      }
    }
    // Do not accumulate the short frontier-stall timer while the tilted LiDAR is still
    // turning toward the unknown frontier; require a full observation dwell while facing it.
    if (!facing_frontier) {
      mapless_stall_time_ = now_steady;
      mapless_stall_yaw_ = cur_start_yaw;
    }
    const double stall_elapsed =
      std::chrono::duration<double>(now_steady - mapless_stall_time_).count();
    const double long_stall_elapsed =
      std::chrono::duration<double>(now_steady - mapless_long_stall_time_).count();
    // Never ban a frontier prematurely on stall=0.00s / facing=0 while the robot is still
    // turning to inspect it; only hard-escape after 24 s of zero net 1.1 m displacement.
    if ((stall_elapsed > 4.0 && facing_frontier && frontier_near) ||
      (long_stall_elapsed > 14.0 && facing_frontier && frontier_near) ||
      long_stall_elapsed > 24.0 ||
      (mapless_consecutive_rejects_ >= 6 && stall_elapsed >= 4.0 &&
        facing_frontier && frontier_near))
    {
      const auto addTaboo = [&](double tx, double ty) {
          if (!std::isfinite(tx) || !std::isfinite(ty)) {return;}
          for (const auto & existing : frontier_taboo_) {
            if (std::hypot(existing.x - tx, existing.y - ty) < 0.60) {return;}
          }
          frontier_taboo_.push_back(FrontierTabooEntry{tx, ty, now_steady});
          ++frontier_taboo_version_;
        };
      if (last_unknown_frontier_) {
        addTaboo(last_unknown_frontier_->x, last_unknown_frontier_->y);
      }
      if (global_route_ && global_route_->path && global_route_->path->poses.size() >= 2U) {
        const auto rem = remainingGuide(*global_route_->path, start, 2.5);
        double s_acc = 0.0;
        double unknown_start_s = -1.0;
        int added_along_unknown = 0;
        for (std::size_t idx = 1U; idx < rem.poses.size(); ++idx) {
          const auto & p0 = rem.poses[idx - 1U].pose.position;
          const auto & p1 = rem.poses[idx].pose.position;
          s_acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
          std::int8_t lv = -1;
          const bool is_unknown = !sampleOccupancyGridCell(grid, p1.x, p1.y, lv) || lv < 0;
          if (is_unknown && unknown_start_s < 0.0) {
            unknown_start_s = s_acc;
          }
          if (unknown_start_s >= 0.0) {
            const double u_s = s_acc - unknown_start_s;
            // Offset taboo centers slightly inside the unknown zone (+0.35 m) so sealing a
            // cliff edge never spills over into a parallel adjacent narrow corridor.
            if (u_s >= 0.35 + static_cast<double>(added_along_unknown) * 0.70) {
              addTaboo(p1.x, p1.y);
              if (++added_along_unknown >= 4) {
                break;
              }
            }
          }
        }
        // Fallback if global route had no unknown sample within local grid
        if (added_along_unknown == 0 && !last_unknown_frontier_) {
          double fb_s = 0.0;
          for (std::size_t idx = 1U; idx < rem.poses.size(); ++idx) {
            const auto & p0 = rem.poses[idx - 1U].pose.position;
            const auto & p1 = rem.poses[idx].pose.position;
            fb_s += std::hypot(p1.x - p0.x, p1.y - p0.y);
            if (fb_s >= 1.2) {
              addTaboo(p1.x, p1.y);
              break;
            }
          }
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "TRACE mapless_frontier_taboo stall=%.2fs rejects=%d facing=%d total_taboo=%zu",
        stall_elapsed, mapless_consecutive_rejects_, facing_frontier ? 1 : 0,
        frontier_taboo_.size());
      last_partial_anchor_.reset();
      last_partial_start_.reset();
      last_unknown_frontier_.reset();
      global_route_.reset();
      mapless_stall_origin_ = start.pose.position;
      mapless_long_stall_origin_ = start.pose.position;
      mapless_stall_yaw_ = cur_start_yaw;
      mapless_stall_time_ = now_steady;
      mapless_long_stall_time_ = now_steady;
      mapless_consecutive_rejects_ = 0;
    }
  }

  // (1) If the goal is already inside the 12x12 m local window and known-free, try direct
  // conservative local search first (reaches the exact final goal without partial hop).
  std::int8_t local_goal_val = -1;
  const bool goal_in_local = sampleOccupancyGridCell(
    grid, goal.pose.position.x, goal.pose.position.y, local_goal_val);
  if (goal_in_local && local_goal_val >= 0 && local_goal_val < obstacle_value_threshold_) {
    double direct_clearance = used_clearance;
    GridSearchResult direct = runGraphSearch(
      grid, start, goal, goal_pose_footprint_verified, direct_clearance, squared, false);
    if (direct.success && goal_pose_footprint_verified) {
      global_route_.reset();
      used_clearance = direct.clearance;
      if (global_guide_out) {
        *global_guide_out = direct.path;
      }
      return direct;
    }
    if (direct.success && !goal_pose_footprint_verified) {
      // A connected known-free corridor exists to the goal inside the local window, yet the
      // goal pose itself is not yet footprint-safe (e.g. its far corner still touches unknown
      // or distant grazing-angle speckles). Approach the farthest footprint-safe point along
      // direct.path as a partial subgoal so close-range LiDAR scans the goal cleanly.
      const double dist_to_goal = std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y);
      if (dist_to_goal > 1.0 && direct.path.poses.size() >= 2U) {
        const auto lidar_fov_offset = resolveLidarFovYawOffsetRad();
        const double start_yaw_local = tf2::getYaw(start.pose.orientation);
        for (int idx = static_cast<int>(direct.path.poses.size()) - 2; idx >= 1; --idx) {
          const auto & pp = direct.path.poses[static_cast<std::size_t>(idx)].pose.position;
          const double adv = std::hypot(
            pp.x - start.pose.position.x, pp.y - start.pose.position.y);
          if (adv < partial_subgoal_min_advance_m_) {
            break;
          }
          const double look_yaw = std::atan2(
            goal.pose.position.y - pp.y, goal.pose.position.x - pp.x);
          const double aim_yaw = lidar_fov_offset.has_value()
            ? std::remainder(look_yaw - *lidar_fov_offset, 2.0 * M_PI)
            : look_yaw;
          double chosen_yaw = aim_yaw;
          bool ok = goalPoseFootprintFree(pp.x, pp.y, aim_yaw, grid, footprint_params_);
          if (!ok) {
            chosen_yaw = look_yaw;
            ok = goalPoseFootprintFree(pp.x, pp.y, look_yaw, grid, footprint_params_);
          }
          if (!ok) {
            chosen_yaw = start_yaw_local;
            ok = goalPoseFootprintFree(pp.x, pp.y, start_yaw_local, grid, footprint_params_);
          }
          if (!ok) {
            continue;
          }
          GridSearchResult approach;
          approach.success = true;
          approach.partial = true;
          approach.clearance = direct.clearance;
          approach.expanded_nodes = direct.expanded_nodes;
          approach.reason = "ok";
          approach.path.header = direct.path.header;
          approach.path.poses.assign(
            direct.path.poses.begin(),
            direct.path.poses.begin() + static_cast<std::ptrdiff_t>(idx + 1));
          approach.path.poses.back().pose.orientation.x = 0.0;
          approach.path.poses.back().pose.orientation.y = 0.0;
          approach.path.poses.back().pose.orientation.z = std::sin(chosen_yaw * 0.5);
          approach.path.poses.back().pose.orientation.w = std::cos(chosen_yaw * 0.5);
          approach.length = pathArcLength(approach.path);
          used_clearance = approach.clearance;
          if (local_anchor_out) {
            *local_anchor_out = approach.path.poses.back();
          }
          if (global_guide_out) {
            *global_guide_out = direct.path;
          }
          return approach;
        }
      }
      // Robot is already close to an observed goal whose tolerance disk has no footprint-safe pose.
      global_route_.reset();
      direct.success = false;
      direct.partial = false;
      direct.reason = "no path";
      direct.expanded_nodes = std::max(direct.expanded_nodes, 2);
      return direct;
    }
    // Only short-circuit to deterministic FAILURE_NO_PATH when the local topology confirms either
    // start or goal is physically sealed inside the 12x12 m window; otherwise fall through to the
    // 40x30 m global search so detours that leave the local window (e.g. descending a bridge ramp)
    // still succeed.
    if ((direct.reason == "no path" || direct.reason == "goal is occupied") &&
      direct.expanded_nodes > 1)
    {
      const double local_floor = search_clearance_floor_configured_
        ? search_clearance_floor_
        : footprintConsistentClearanceFloor(
          inscribed_footprint_radius_, grid.info.resolution, preferred_search_clearance_);
      if (isLocalDirectNoPathSealed(
          grid, squared, start, goal, local_floor, obstacle_value_threshold_))
      {
        global_route_.reset();
        direct.success = false;
        direct.partial = false;
        direct.reason = "no path";
        direct.expanded_nodes = std::max(direct.expanded_nodes, 2);
        return direct;
      }
    }
  }

  // (2) If the goal lies inside the local window and its entire tolerance disk is confirmed
  // occupied (>= threshold, not -1 unknown), fail deterministically without exploration.
  if (goal_in_local && local_goal_val >= obstacle_value_threshold_) {
    const int rad_cells = std::max(
      0, static_cast<int>(std::ceil(
        std::max(0.0, goal_pose_admission_params_.position_tolerance_m) /
        std::max(1e-3, static_cast<double>(grid.info.resolution)))));
    int gix = 0;
    int giy = 0;
    sampleOccupancyGridCell(
      grid, goal.pose.position.x, goal.pose.position.y, local_goal_val, &gix, &giy);
    bool all_known_occ = true;
    for (int oy = -rad_cells; oy <= rad_cells && all_known_occ; ++oy) {
      for (int ox = -rad_cells; ox <= rad_cells && all_known_occ; ++ox) {
        const int ix = gix + ox;
        const int iy = giy + oy;
        if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(grid.info.width) ||
          iy >= static_cast<int>(grid.info.height))
        {
          all_known_occ = false;
          break;
        }
        if (grid.data[static_cast<std::size_t>(iy) * grid.info.width + ix] <
          obstacle_value_threshold_)
        {
          all_known_occ = false;
        }
      }
    }
    if (all_known_occ) {
      global_route_.reset();
      GridSearchResult failed;
      failed.reason = "goal is occupied";
      return failed;
    }
  }

  // (3) 40x30 m Optimistic Global Search (unknown traversable with gentle penalty) + caching.
  auto buildTabooedGlobalGrid = [&](bool apply_taboo) {
      nav_msgs::msg::OccupancyGrid out = *snapshot->global_grid;
      if (!apply_taboo || frontier_taboo_.empty() || out.info.resolution <= 0.0) {
        return out;
      }
      const double gres = out.info.resolution;
      const double gox = out.info.origin.position.x;
      const double goy = out.info.origin.position.y;
      const int gw = static_cast<int>(out.info.width);
      const int gh = static_cast<int>(out.info.height);
      const double r_taboo = frontier_taboo_radius_m_;
      const int r_cells = std::max(1, static_cast<int>(std::ceil(r_taboo / gres)));
      for (const auto & e : frontier_taboo_) {
        const int cx = static_cast<int>(std::floor((e.x - gox) / gres));
        const int cy = static_cast<int>(std::floor((e.y - goy) / gres));
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
          const int iy = cy + dy;
          if (iy < 0 || iy >= gh) {continue;}
          for (int dx = -r_cells; dx <= r_cells; ++dx) {
            const int ix = cx + dx;
            if (ix < 0 || ix >= gw) {continue;}
            const double wx = gox + (static_cast<double>(ix) + 0.5) * gres;
            const double wy = goy + (static_cast<double>(iy) + 0.5) * gres;
            if (std::hypot(wx - e.x, wy - e.y) > r_taboo) {continue;}
            // Never stamp over the robot's immediate exit zone or the final goal.
            if (std::hypot(wx - start.pose.position.x, wy - start.pose.position.y) < 0.85 ||
              std::hypot(wx - goal.pose.position.x, wy - goal.pose.position.y) < 0.65)
            {
              continue;
            }
            // Taboo only masks unobservable unknown (-1) cliff/void cells so optimistic
            // global search never routes off arena edges or drop-offs, while never sealing
            // known-free (0) corridors or tunnels.
            auto & cell = out.data[static_cast<std::size_t>(iy) * static_cast<std::size_t>(gw) +
              static_cast<std::size_t>(ix)];
            if (cell < 0) {
              cell = 100;
            }
          }
        }
      }
      return out;
    };
  const GridOccupancyPolicy optimistic_policy{obstacle_value_threshold_, false};
  const double start_to_goal_m = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  const double cache_goal_tol = start_to_goal_m > 1.5 ? 0.25 : 0.05;
  const bool same_global_goal = global_route_ && global_route_->optimistic &&
    global_route_->goal_uuid == goal_uuid &&
    global_route_->goal_id == goal_id &&
    global_route_->localization_epoch == localization_epoch &&
    global_route_->path->header.frame_id == grid.header.frame_id &&
    std::hypot(global_route_->goal.position.x - goal.pose.position.x,
    global_route_->goal.position.y - goal.pose.position.y) <= cache_goal_tol &&
    std::abs(std::remainder(tf2::getYaw(global_route_->goal.orientation) -
    tf2::getYaw(goal.pose.orientation), 2.0 * M_PI)) <= 0.10;

  nav_msgs::msg::Path global_path;
  double global_clearance = preferred_search_clearance_;
  bool reused_global_cache = false;
  if (same_global_goal) {
    nav_msgs::msg::Path candidate = remainingGuide(
      *global_route_->path, start, std::max(replan_start_params_.deviation_max_m, 1.0));
    if (candidate.poses.size() >= 2U) {
      // Fast O(path_len) check directly against raw global_grid + taboo circles + local grid,
      // avoiding a 480k-cell full-map EDT copy/recompute whenever the cached guide stays clear.
      bool blocked = false;
      const auto & raw_g = *snapshot->global_grid;
      for (std::size_t i = 1U; i < candidate.poses.size() && !blocked; ++i) {
        const auto & p = candidate.poses[i].pose.position;
        int gx = 0, gy = 0;
        std::int8_t gval = -1;
        if (sampleOccupancyGridCell(raw_g, p.x, p.y, gval, &gx, &gy)) {
          if (gval >= obstacle_value_threshold_ ||
            !hasGridClearance(raw_g, gx, gy, inscribed_footprint_radius_, optimistic_policy))
          {
            blocked = true;
            break;
          }
        }
        for (const auto & te : frontier_taboo_) {
          if (std::hypot(p.x - te.x, p.y - te.y) <= frontier_taboo_radius_m_ + 0.05 &&
            gval < 0)
          {
            blocked = true;
            break;
          }
        }
        std::int8_t lval = -1;
        if (!blocked && sampleOccupancyGridCell(grid, p.x, p.y, lval) &&
          lval >= obstacle_value_threshold_)
        {
          blocked = true;
        }
      }
      if (!blocked) {
        global_path = std::move(candidate);
        global_clearance = global_route_->clearance;
        reused_global_cache = true;
      } else if (!global_route_->known_only) {
        ++mapless_optimistic_obstructed_count_;
      }
    }
  }

  GridSearchResult optimistic_result;
  if (!reused_global_cache) {
    if (cached_global_blocked_pub_seq_ != snapshot->publication_sequence ||
      cached_global_blocked_taboo_version_ != frontier_taboo_version_ ||
      cached_global_blocked_sq_.size() != snapshot->global_grid->data.size())
    {
      cached_tabooed_global_grid_ = buildTabooedGlobalGrid(true);
      cached_global_blocked_sq_ = computeBlockedSquaredDistanceCells(
        cached_tabooed_global_grid_, optimistic_policy);
      cached_global_blocked_pub_seq_ = snapshot->publication_sequence;
      cached_global_blocked_taboo_version_ = frontier_taboo_version_;
    }
    const auto & global_grid = cached_tabooed_global_grid_;
    const auto & global_blocked_sq = cached_global_blocked_sq_;
    const double coarse_res = global_grid.info.resolution;
    const double global_floor = footprintConsistentClearanceFloor(
      inscribed_footprint_radius_, coarse_res, preferred_search_clearance_);
    std::vector<double> global_ladder{preferred_search_clearance_};
    if (clearance_relaxation_enabled_ && global_floor + 1e-6 < preferred_search_clearance_) {
      global_ladder.push_back(global_floor);
    }
    const auto runGlobalSearch = [&](double clr, double endpoint_min, bool known_only) {
        GridSearchOptions opts;
        opts.clearance = clr;
        opts.endpoint_minimum = endpoint_min;
        opts.unknown_is_obstacle_override = known_only;
        opts.unknown_cost_multiplier = optimistic_global_unknown_cost_;
        return graph_search_.plan(global_grid, start, goal, &global_blocked_sq, opts);
      };
    const auto knownPrefixArcLength = [&](const nav_msgs::msg::Path & path) {
        double known_s = 0.0;
        for (std::size_t i = 0; i < path.poses.size(); ++i) {
          const auto & p = path.poses[i].pose.position;
          if (i > 0U) {
            const auto & prev = path.poses[i - 1U].pose.position;
            known_s += std::hypot(p.x - prev.x, p.y - prev.y);
          }
          std::int8_t lv = -1;
          const bool in_local = sampleOccupancyGridCell(grid, p.x, p.y, lv);
          if (!in_local || lv < 0 || lv >= obstacle_value_threshold_) {
            if (i > 0U) {
              break;
            }
          }
          if (known_s >= 5.5) {
            break;
          }
        }
        return known_s;
      };
    const auto selectGlobalLadder = [&](bool known_only) {
        const double straight_dist = std::hypot(
          goal.pose.position.x - start.pose.position.x,
          goal.pose.position.y - start.pose.position.y);
        GridSearchResult best;
        double best_known = 0.0;
        for (std::size_t lvl = 0; lvl < global_ladder.size(); ++lvl) {
          const double ep_min = (lvl + 1U == global_ladder.size())
            ? std::min(global_floor, coarse_res + 0.01) : -1.0;
          GridSearchResult attempt = runGlobalSearch(global_ladder[lvl], ep_min, known_only);
          if (!attempt.success) {
            if (!best.success) {
              best = std::move(attempt);
            }
            continue;
          }
          const double attempt_known = knownPrefixArcLength(attempt.path);
          if (!best.success) {
            best = std::move(attempt);
            best_known = attempt_known;
            // Always evaluate the footprint-consistent floor (0.341 m) on global maps when
            // 0.436 m deviates from straight-line distance OR runs into unknown space near
            // the robot: narrow 0.75-0.80 m tunnels are impassable at 0.436 m but passable
            // at 0.341 m.
            const bool check_shorter_corridor =
              lvl + 1U < global_ladder.size() &&
              (best.length > straight_dist * 1.03 || best_known < 1.5);
            if (!check_shorter_corridor) {
              break;
            }
          } else {
            const bool much_shorter =
              best.length > attempt.length * 1.06 || best.length > attempt.length + 1.0;
            const bool opens_known_corridor =
              best_known < 1.5 && attempt_known >= best_known + 0.80 &&
              attempt.length <= best.length * 1.25;
            if (much_shorter || opens_known_corridor) {
              best = std::move(attempt);
              best_known = attempt_known;
            }
            break;
          }
        }
        if (known_only && clearance_relaxation_enabled_ &&
          inscribed_footprint_radius_ + 1e-6 < global_floor &&
          (!best.success || best.length > straight_dist * 1.50 + 2.0))
        {
          // 5 cm grid 0.341 m needs >= 7 cells (0.35 m); 1 grazing pixel on a 0.795 m tunnel
          // drops to 0.335 m. Allow known-free guide fallback to inscribed_footprint_radius_
          // (min pass 0.316 m); local swept footprint still gates every executed meter.
          const double ep_min = std::min(inscribed_footprint_radius_, coarse_res + 0.01);
          GridSearchResult tight = runGlobalSearch(inscribed_footprint_radius_, ep_min, true);
          if (tight.success &&
            (!best.success || tight.length * 1.15 + 1.5 < best.length))
          {
            best = std::move(tight);
          }
        }
        return best;
      };
    const auto g_begin = std::chrono::steady_clock::now();
    // Try known-free global search first when goal is mapped; fall back to optimistic if unmapped.
    const double straight_dist = std::hypot(
      goal.pose.position.x - start.pose.position.x,
      goal.pose.position.y - start.pose.position.y);
    std::int8_t goal_gval = -1;
    if (sampleOccupancyGridCell(
        global_grid, goal.pose.position.x, goal.pose.position.y, goal_gval) &&
      goal_gval >= 0 && goal_gval < obstacle_value_threshold_)
    {
      optimistic_result = selectGlobalLadder(true);
    }
    bool route_is_known_only = optimistic_result.success;
    // Keep a verified known-free route once frontier taboo or >= 2 optimistic obstructions hit
    // (preventing outer-wall gap ping-pong); otherwise allow much shorter unknown shortcuts.
    if (!optimistic_result.success ||
      (frontier_taboo_.empty() && mapless_optimistic_obstructed_count_ < 2 &&
        optimistic_result.length > straight_dist * 1.65 + 2.0))
    {
      GridSearchResult opt_candidate = selectGlobalLadder(false);
      if (opt_candidate.success &&
        (!optimistic_result.success ||
          opt_candidate.length * 1.80 + 3.0 < optimistic_result.length))
      {
        optimistic_result = std::move(opt_candidate);
        route_is_known_only = false;
      }
    }
    if (optimistic_result.success) {
      global_clearance = optimistic_result.clearance;
      global_path = std::move(optimistic_result.path);
    }
    // If taboo circles accidentally sealed off all routes from the robot, clear taboo
    // and retry on the raw global grid so stale taboo never causes a false NO_PATH stall.
    if (global_path.poses.size() < 2U && !frontier_taboo_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "TRACE mapless_frontier_taboo cleared=%zu reason=global_sealed_retry",
        frontier_taboo_.size());
      frontier_taboo_.clear();
      ++frontier_taboo_version_;
      cached_tabooed_global_grid_ = buildTabooedGlobalGrid(false);
      cached_global_blocked_sq_ = computeBlockedSquaredDistanceCells(
        cached_tabooed_global_grid_, optimistic_policy);
      cached_global_blocked_pub_seq_ = snapshot->publication_sequence;
      cached_global_blocked_taboo_version_ = frontier_taboo_version_;
      optimistic_result = selectGlobalLadder(false);
      route_is_known_only = false;
      if (optimistic_result.success) {
        global_clearance = optimistic_result.clearance;
        global_path = std::move(optimistic_result.path);
      }
    }
    const double g_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - g_begin).count();
    RCLCPP_INFO(
      get_logger(),
      "TRACE mapless_global_route goal=%llu publication=%llu cached=0 success=%d "
      "known_only=%d opt_obs=%d points=%zu length=%.2f clearance=%.3f expanded=%d wall_ms=%.2f reason='%s'",
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(snapshot->publication_sequence),
      optimistic_result.success ? 1 : 0, route_is_known_only ? 1 : 0,
      mapless_optimistic_obstructed_count_, global_path.poses.size(),
      optimistic_result.length, global_clearance, optimistic_result.expanded_nodes,
      g_ms, optimistic_result.reason.c_str());
    if (global_path.poses.size() < 2U) {
      // Even optimistic search over the full 40x30 m global layer found no path:
      // goal or start is physically enclosed by confirmed obstacles -> deterministic NO_PATH!
      global_route_.reset();
      optimistic_result.partial = false;
      optimistic_result.success = false;
      if (optimistic_result.reason.empty()) {
        optimistic_result.reason = "no path";
      }
      return optimistic_result;
    }
    global_route_ = GlobalRoute{
      goal_uuid, goal_id, localization_epoch, snapshot, goal.pose,
      std::make_shared<const nav_msgs::msg::Path>(global_path), global_clearance, true,
      route_is_known_only};
  }

  if (global_guide_out) {
    *global_guide_out = global_path;
  }

  // (4) Extract conservative known-free local rolling anchor along global_path inside the
  // 12x12 m local window and optimize strictly on known-free space.
  struct SampledGlobalPoint
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double s{0.0};
  };
  std::vector<SampledGlobalPoint> known_prefix;
  std::optional<geometry_msgs::msg::PoseStamped> first_unknown_pose;
  const double anchor_cap_m = mapless_consecutive_rejects_ >= 2 ? 1.6 :
    (mapless_consecutive_rejects_ == 1 ? 2.8 : 4.6);
  const double max_local_anchor_arc = std::min(rolling_horizon_length_m_, anchor_cap_m);
  double accum_s = 0.0;
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const auto computeTangent = [&](std::size_t idx) {
      const std::size_t i0 = idx > 0U ? idx - 1U : 0U;
      const std::size_t i1 = std::min(idx + 1U, global_path.poses.size() - 1U);
      const auto & p0 = global_path.poses[i0].pose.position;
      const auto & p1 = global_path.poses[i1].pose.position;
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      return std::hypot(dx, dy) > 1e-6 ? std::atan2(dy, dx) : start_yaw;
    };
  const auto lidar_fov_offset = resolveLidarFovYawOffsetRad();
  const auto computeLookaheadHeading = [&](std::size_t idx) {
      const auto & base_p = global_path.poses[idx].pose.position;
      double s_ahead = 0.0;
      std::size_t target_idx = idx;
      for (std::size_t j = idx + 1U; j < global_path.poses.size(); ++j) {
        const auto & a = global_path.poses[j - 1U].pose.position;
        const auto & b = global_path.poses[j].pose.position;
        s_ahead += std::hypot(b.x - a.x, b.y - a.y);
        target_idx = j;
        if (s_ahead >= 1.2) {
          break;
        }
      }
      const auto & tp = global_path.poses[target_idx].pose.position;
      const double dx = tp.x - base_p.x;
      const double dy = tp.y - base_p.y;
      return std::hypot(dx, dy) > 1e-4 ? std::atan2(dy, dx) : computeTangent(idx);
    };
  for (std::size_t i = 0; i < global_path.poses.size(); ++i) {
    const auto & p = global_path.poses[i].pose.position;
    if (i > 0U) {
      const auto & prev = global_path.poses[i - 1U].pose.position;
      accum_s += std::hypot(p.x - prev.x, p.y - prev.y);
    }
    std::int8_t lval = -1;
    const bool in_local = sampleOccupancyGridCell(grid, p.x, p.y, lval);
    if (!in_local || lval < 0) {
      if (!first_unknown_pose) {
        geometry_msgs::msg::PoseStamped fp;
        fp.header = grid.header;
        fp.pose.position = p;
        const double tyaw = computeTangent(i);
        fp.pose.orientation.z = std::sin(tyaw * 0.5);
        fp.pose.orientation.w = std::cos(tyaw * 0.5);
        first_unknown_pose = fp;
      }
      if (i > 0U) {
        break;
      }
      continue;
    }
    if (lval >= obstacle_value_threshold_) {
      if (i > 0U) {
        break;
      }
      continue;
    }
    if (accum_s > max_local_anchor_arc && !known_prefix.empty()) {
      break;
    }
    known_prefix.push_back(SampledGlobalPoint{p.x, p.y, computeLookaheadHeading(i), accum_s});
  }

  if (first_unknown_pose) {
    last_unknown_frontier_ = first_unknown_pose->pose.position;
  } else {
    last_unknown_frontier_.reset();
  }

  auto isYawTurnFootprintFree = [&](double tx, double ty, double from_yaw, double to_yaw,
      const FootprintSafetyParams & check_fp) -> bool {
      const double delta = std::remainder(to_yaw - from_yaw, 2.0 * M_PI);
      const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / 0.25)));
      for (int s = 1; s <= steps; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(steps);
        const double y_sample = std::remainder(from_yaw + delta * ratio, 2.0 * M_PI);
        if (!goalPoseFootprintFree(tx, ty, y_sample, grid, check_fp)) {
          return false;
        }
      }
      return true;
    };

  auto tryLidarBlindSpotScanTurn = [&]() -> std::optional<GridSearchResult> {
      if (!lidar_fov_offset.has_value()) {
        return std::nullopt;
      }
      double dep_x = goal.pose.position.x;
      double dep_y = goal.pose.position.y;
      double dep_s = 0.0;
      for (std::size_t i = 1U; i < global_path.poses.size(); ++i) {
        const auto & p0 = global_path.poses[i - 1U].pose.position;
        const auto & p1 = global_path.poses[i].pose.position;
        dep_s += std::hypot(p1.x - p0.x, p1.y - p0.y);
        dep_x = p1.x;
        dep_y = p1.y;
        if (dep_s >= 1.2) {
          break;
        }
      }
      const double dep_yaw = std::atan2(
        dep_y - start.pose.position.y, dep_x - start.pose.position.x);
      const double target_scan_yaw = std::remainder(
        dep_yaw - *lidar_fov_offset, 2.0 * M_PI);
      const double yaw_err = std::remainder(target_scan_yaw - start_yaw, 2.0 * M_PI);
      const double yaw_diff = std::abs(yaw_err);
      if (yaw_diff <= 0.35) {
        return std::nullopt;
      }
      const double step_yaw = std::remainder(
        start_yaw + std::clamp(yaw_err, -1.8, 1.8), 2.0 * M_PI);
      // Prefer stepping slightly into open known space (away from or sideways to the frontier)
      // where the full rotational sweep from start_yaw to step_yaw clears walls with extra margin
      // (> 1 planning cell) so 5 cm grid quantization never lets a blind-spot spin graze a wall.
      FootprintSafetyParams scan_turn_fp = footprint_params_;
      scan_turn_fp.safety_margin += 0.06;
      const double step_radii[4] = {0.20, 0.35, 0.50, 0.16};
      const double angle_offsets[8] = {
        M_PI, 0.75 * M_PI, -0.75 * M_PI, 0.5 * M_PI, -0.5 * M_PI,
        0.25 * M_PI, -0.25 * M_PI, 0.0};
      for (double sr : step_radii) {
        for (double off : angle_offsets) {
          const double th = dep_yaw + off;
          const double tx = start.pose.position.x + sr * std::cos(th);
          const double ty = start.pose.position.y + sr * std::sin(th);
          std::int8_t lv = -1;
          if (!sampleOccupancyGridCell(grid, tx, ty, lv) || lv < 0 ||
            lv >= obstacle_value_threshold_)
          {
            continue;
          }
          if (!isYawTurnFootprintFree(tx, ty, start_yaw, step_yaw, scan_turn_fp)) {
            continue;
          }
          geometry_msgs::msg::PoseStamped turn_pose = start;
          turn_pose.header = grid.header;
          turn_pose.pose.position.x = tx;
          turn_pose.pose.position.y = ty;
          turn_pose.pose.orientation.x = 0.0;
          turn_pose.pose.orientation.y = 0.0;
          turn_pose.pose.orientation.z = std::sin(step_yaw * 0.5);
          turn_pose.pose.orientation.w = std::cos(step_yaw * 0.5);
          double local_clr = used_clearance;
          GridSearchResult turn_attempt = runGraphSearch(
            grid, start, turn_pose, true, local_clr, squared, false);
          if (turn_attempt.success && turn_attempt.path.poses.size() >= 2U) {
            turn_attempt.partial = true;
            turn_attempt.scan_turn = true;
            turn_attempt.path.poses.back().pose.orientation = turn_pose.pose.orientation;
            used_clearance = turn_attempt.clearance;
            if (local_anchor_out) {
              *local_anchor_out = turn_pose;
            }
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), log_throttle_ms_,
              "【无图盲区对准】沿全局路径起始方向短步转向 yaw=%.2f->%.2f rad (target=%.2f) 以扫描未知出口。",
              start_yaw, step_yaw, dep_yaw);
            return turn_attempt;
          }
        }
      }
      return std::nullopt;
    };

  // If known_prefix hits an unknown frontier within 1.2 m while the LiDAR's actual TF-derived
  // observation direction is misaligned (> 26 deg) with the global departure heading, execute
  // a clean open-space scan-turn to aim the LiDAR straight into the unknown corridor.
  const bool blocked_near_start_by_unknown =
    first_unknown_pose.has_value() &&
    (known_prefix.empty() || known_prefix.back().s < 1.2);
  if (blocked_near_start_by_unknown && lidar_fov_offset.has_value()) {
    const double stall_now = mapless_stall_origin_
      ? std::chrono::duration<double>(now_steady - mapless_stall_time_).count()
      : 0.0;
    if (mapless_consecutive_rejects_ >= 1 || stall_now >= 1.2) {
      if (auto scan_turn = tryLidarBlindSpotScanTurn()) {
        return *scan_turn;
      }
    }
  }

  FootprintSafetyParams wide_anchor_fp = footprint_params_;
  wide_anchor_fp.safety_margin += 0.12;
  FootprintSafetyParams mid_anchor_fp = footprint_params_;
  mid_anchor_fp.safety_margin += 0.04;
  auto findSafeAnchorNear = [&](double cx, double cy, double pref_yaw,
      const FootprintSafetyParams & check_fp, bool require_lidar_aim,
      geometry_msgs::msg::PoseStamped & out_pose) -> bool {
      // Aim the actual TF-derived LiDAR observation axis (+Z horizontal projection) along
      // the global route continuation heading so the swerve robot scans ahead into unknown
      // corridors while cruising. Never accept a radial offset that backtracks behind start.
      const double lidar_aim_yaw = lidar_fov_offset.has_value()
        ? std::remainder(pref_yaw - *lidar_fov_offset, 2.0 * M_PI)
        : pref_yaw;
      std::vector<double> yaws;
      yaws.push_back(lidar_aim_yaw);
      if (!require_lidar_aim) {
        yaws.push_back(pref_yaw);
        yaws.push_back(start_yaw);
      }
      const double sp_dx = cx - start.pose.position.x;
      const double sp_dy = cy - start.pose.position.y;
      const double sp_dist = std::hypot(sp_dx, sp_dy);
      const double radii[4] = {0.0, 0.15, 0.30, 0.45};
      for (double r : radii) {
        const int angles = (r <= 1e-6) ? 1 : 8;
        for (int a = 0; a < angles; ++a) {
          const double th = (2.0 * M_PI * a) / static_cast<double>(angles);
          const double tx = cx + r * std::cos(th);
          const double ty = cy + r * std::sin(th);
          const double dx = tx - start.pose.position.x;
          const double dy = ty - start.pose.position.y;
          const double dist = std::hypot(dx, dy);
          if (dist < partial_subgoal_min_advance_m_) {
            continue;
          }
          // Reject radial offsets that bounce backward behind start (prevents ping-ponging
          // at narrow corridor mouths when r=0.45 finds open space behind the robot).
          if (r > 1e-6 && sp_dist > 1e-3 &&
            (dx * sp_dx + dy * sp_dy) < 0.5 * dist * sp_dist)
          {
            continue;
          }
          std::int8_t lv = -1;
          if (!sampleOccupancyGridCell(grid, tx, ty, lv) || lv < 0 ||
            lv >= obstacle_value_threshold_)
          {
            continue;
          }
          for (double yv : yaws) {
            // Inside a narrow corridor far ahead (dist >= 0.65 m), the robot pre-aligns to yv
            // in open space BEFORE entering; requiring an in-place spin sweep at (tx, ty) would
            // falsely reject every valid anchor inside a 0.80 m tunnel. Full trajectory swept
            // safety is always verified downstream by FootprintSafetyChecker.
            const bool need_inplace_sweep =
              dist < 0.65 && std::abs(std::remainder(yv - start_yaw, 2.0 * M_PI)) > 0.20;
            const bool yaw_safe = need_inplace_sweep
              ? isYawTurnFootprintFree(tx, ty, start_yaw, yv, check_fp)
              : goalPoseFootprintFree(tx, ty, yv, grid, check_fp);
            if (yaw_safe) {
              out_pose.header = grid.header;
              out_pose.pose.position.x = tx;
              out_pose.pose.position.y = ty;
              out_pose.pose.position.z = start.pose.position.z;
              out_pose.pose.orientation.x = 0.0;
              out_pose.pose.orientation.y = 0.0;
              out_pose.pose.orientation.z = std::sin(yv * 0.5);
              out_pose.pose.orientation.w = std::cos(yv * 0.5);
              return true;
            }
          }
        }
      }
      return false;
    };

  // Walk backward from the farthest known-free point along global_path so we maximize
  // safe known-free horizon (and enter narrow 0.80 m corridors directly instead of
  // short-circuiting to a 0.4 m open-space stub outside the corridor mouth). At each
  // distance, prefer wide margin (+12 cm) first, then mid (+4 cm), then nominal footprint.
  const bool prefer_lidar_aim_first = first_unknown_pose.has_value() && lidar_fov_offset.has_value();
  const FootprintSafetyParams anchor_fps[3] = {wide_anchor_fp, mid_anchor_fp, footprint_params_};
  double last_tried_s = std::numeric_limits<double>::infinity();
  for (int idx = static_cast<int>(known_prefix.size()) - 1; idx >= 0; --idx) {
    const auto & sp = known_prefix[static_cast<std::size_t>(idx)];
    if (sp.s < partial_subgoal_min_advance_m_) {
      break;
    }
    if (last_tried_s - sp.s < 0.25 && idx + 1 < static_cast<int>(known_prefix.size())) {
      continue;
    }
    geometry_msgs::msg::PoseStamped anchor_pose;
    bool found_anchor = false;
    for (int pass = 0; pass < 3; ++pass) {
      const bool require_aim = prefer_lidar_aim_first && (pass < 2);
      if (findSafeAnchorNear(sp.x, sp.y, sp.yaw, anchor_fps[pass], require_aim, anchor_pose)) {
        found_anchor = true;
        break;
      }
    }
    if (!found_anchor) {
      continue;
    }
    last_tried_s = sp.s;
    double local_clr = used_clearance;
    GridSearchResult local_attempt = runGraphSearch(
      grid, start, anchor_pose, true, local_clr, squared, false);
    if (local_attempt.success) {
      const double dist_to_final = std::hypot(
        anchor_pose.pose.position.x - goal.pose.position.x,
        anchor_pose.pose.position.y - goal.pose.position.y);
      local_attempt.partial = dist_to_final > 0.05;
      if (local_attempt.partial && !local_attempt.path.poses.empty()) {
        local_attempt.path.poses.back().pose.orientation = anchor_pose.pose.orientation;
      }
      used_clearance = local_attempt.clearance;
      if (local_anchor_out && local_attempt.partial) {
        *local_anchor_out = anchor_pose;
      }
      if (local_attempt.partial) {
        if (!last_partial_anchor_ ||
          std::hypot(anchor_pose.pose.position.x - last_partial_anchor_->x,
            anchor_pose.pose.position.y - last_partial_anchor_->y) > 0.50)
        {
          last_partial_anchor_ = anchor_pose.pose.position;
          last_partial_start_ = start.pose.position;
          last_partial_anchor_time_ = now_steady;
        }
      }
      return local_attempt;
    }
  }

  // Fallback A: tilted LiDAR blind spot right behind/beside start.
  if (auto scan_turn = tryLidarBlindSpotScanTurn()) {
    return *scan_turn;
  }

  // Fallback B: conservative local frontier search toward the global path's frontier entry
  // (NOT the raw Euclidean final goal, avoiding concave pocket deadlocks).
  geometry_msgs::msg::PoseStamped frontier_target =
    first_unknown_pose.value_or(global_path.poses.back());
  frontier_target.header = grid.header;
  GridSearchResult local_partial = runGraphSearch(
    grid, start, frontier_target, false, used_clearance, squared, true);
  if (local_partial.success || local_partial.partial) {
    local_partial.partial = true;
    used_clearance = local_partial.clearance;
    if (local_anchor_out && !local_partial.path.poses.empty()) {
      *local_anchor_out = local_partial.path.poses.back();
    }
    return local_partial;
  }

  return local_partial;
}

}  // namespace minco_planner
