// Copyright 2026

#include "minco_planner_node_helpers.hpp"

#include <queue>

namespace minco_planner
{

namespace node_detail
{

// Check whether a local conservative direct-search "no path" failure is genuinely sealed inside
// the 12x12 m local window (e.g. an enclosed room/barrier or sealed obstacle pocket), versus a
// detour that simply leaves the 12x12 m local window (such as descending a bridge ramp via the
// 40x30 m global layer).
bool isLocalDirectNoPathSealed(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<double> * conservative_squared,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double floor_clearance_m,
  int obstacle_value_threshold)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double res = grid.info.resolution;
  if (w <= 0 || h <= 0 || !(res > 0.0) || !conservative_squared ||
    conservative_squared->size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h))
  {
    return false;
  }
  int sx = 0, sy = 0, gx = 0, gy = 0;
  std::int8_t sv = -1, gv = -1;
  if (!sampleOccupancyGridCell(grid, start.pose.position.x, start.pose.position.y, sv, &sx, &sy) ||
    !sampleOccupancyGridCell(grid, goal.pose.position.x, goal.pose.position.y, gv, &gx, &gy))
  {
    return false;
  }
  const double r_cells = std::max(0.0, floor_clearance_m) / res;
  const double r_sq = r_cells * r_cells;
  const int border_margin = static_cast<int>(std::ceil(r_cells)) + 2;
  const int seed_rad = std::max(1, static_cast<int>(std::ceil(r_cells)) + 1);
  const auto onBorder = [&](int x, int y) {
      return x <= border_margin || x >= w - 1 - border_margin ||
             y <= border_margin || y >= h - 1 - border_margin;
    };
  const double seg_dx = goal.pose.position.x - start.pose.position.x;
  const double seg_dy = goal.pose.position.y - start.pose.position.y;
  const double seg_len = std::hypot(seg_dx, seg_dy);
  const double ux = seg_len > 1e-6 ? (seg_dx / seg_len) : 1.0;
  const double uy = seg_len > 1e-6 ? (seg_dy / seg_len) : 0.0;
  const auto inDirectCorridor = [&](int x, int y) {
      const double wx = grid.info.origin.position.x + (static_cast<double>(x) + 0.5) * res;
      const double wy = grid.info.origin.position.y + (static_cast<double>(y) + 0.5) * res;
      const double rx = wx - start.pose.position.x;
      const double ry = wy - start.pose.position.y;
      const double proj = rx * ux + ry * uy;
      const double lat = std::abs(-rx * uy + ry * ux);
      return proj >= -1.50 && proj <= seg_len + 1.50 && lat <= 3.50;
    };
  const auto bfsComponent = [&](
    const std::vector<double> & sq_field, bool allow_unknown, bool corridor_only,
    int seed_x, int seed_y, int target_x, int target_y,
    bool & reached_border, bool & reached_target)
    {
      reached_border = false;
      reached_target = false;
      const auto cellPassable = [&](int x, int y) {
          if (x < 0 || y < 0 || x >= w || y >= h) {return false;}
          if (corridor_only && !inDirectCorridor(x, y)) {return false;}
          const auto idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
            static_cast<std::size_t>(x);
          const std::int8_t val = grid.data[idx];
          if (val >= obstacle_value_threshold || (!allow_unknown && val < 0)) {return false;}
          return sq_field[idx] > r_sq;
        };
      std::vector<std::uint8_t> visited(
        static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0U);
      std::queue<std::pair<int, int>> q;
      const auto pushCell = [&](int x, int y) {
          const auto idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
            static_cast<std::size_t>(x);
          if (!visited[idx]) {
            visited[idx] = 1U;
            q.emplace(x, y);
          }
        };
      if (cellPassable(seed_x, seed_y)) {
        pushCell(seed_x, seed_y);
      } else {
        for (int dy = -seed_rad; dy <= seed_rad; ++dy) {
          for (int dx = -seed_rad; dx <= seed_rad; ++dx) {
            if (dx * dx + dy * dy > seed_rad * seed_rad) {continue;}
            if (cellPassable(seed_x + dx, seed_y + dy)) {
              pushCell(seed_x + dx, seed_y + dy);
            }
          }
        }
      }
      if (q.empty()) {
        return;
      }
      const int kDx[4] = {1, -1, 0, 0};
      const int kDy[4] = {0, 0, 1, -1};
      while (!q.empty()) {
        const auto cur = q.front();
        q.pop();
        if (onBorder(cur.first, cur.second)) {
          reached_border = true;
        }
        if (std::abs(cur.first - target_x) <= seed_rad &&
          std::abs(cur.second - target_y) <= seed_rad)
        {
          reached_target = true;
        }
        if (reached_border && reached_target) {
          return;
        }
        for (int k = 0; k < 4; ++k) {
          const int nx = cur.first + kDx[k];
          const int ny = cur.second + kDy[k];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
          const auto nidx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) +
            static_cast<std::size_t>(nx);
          if (!visited[nidx] && cellPassable(nx, ny)) {
            visited[nidx] = 1U;
            q.emplace(nx, ny);
          }
        }
      }
    };

  const GridOccupancyPolicy opt_policy{obstacle_value_threshold, false};
  const std::vector<double> opt_sq = computeBlockedSquaredDistanceCells(grid, opt_policy);

  // (A) If start's known-free traversable component does NOT reach the 12x12 m border (e.g.
  // an enclosed 10x10 m room with a wall-to-wall barrier, or a robot at an arena edge),
  // distinguish a confirmed-obstacle barrier from an unmapped interior corridor (like a bridge
  // ahead in a LiDAR blind spot): if even optimistic BFS (unknown passable, only confirmed
  // obstacles block) inside the +/-1.5 m direct corridor between start and goal cannot connect
  // them, then start and goal are separated by a confirmed barrier with no local escape.
  bool conservative_start_border = false;
  bool conservative_start_goal = false;
  bfsComponent(
    *conservative_squared, false, false, sx, sy, gx, gy,
    conservative_start_border, conservative_start_goal);
  if (!conservative_start_border) {
    bool opt_corridor_border = false;
    bool opt_corridor_goal = false;
    bfsComponent(
      opt_sq, true, true, sx, sy, gx, gy,
      opt_corridor_border, opt_corridor_goal);
    if (!opt_corridor_goal) {
      return true;
    }
  }

  // (B) Even if start reaches the 12x12 m border, check under optimistic occupancy whether
  // either start or goal is physically sealed inside a confirmed-obstacle pocket.
  bool opt_goal_border = false;
  bool opt_goal_start = false;
  bfsComponent(opt_sq, true, false, gx, gy, sx, sy, opt_goal_border, opt_goal_start);
  if (!opt_goal_border && !opt_goal_start) {
    return true;
  }
  bool opt_start_border = false;
  bool opt_start_goal = false;
  bfsComponent(opt_sq, true, false, sx, sy, gx, gy, opt_start_border, opt_start_goal);
  if (!opt_start_border && !opt_start_goal) {
    return true;
  }
  return false;
}

}  // namespace node_detail

using namespace node_detail;

GridSearchResult MincoPlannerNode::planRoute(
  const std::shared_ptr<const PlanningMapSnapshot> & snapshot,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const GoalUuid & goal_uuid,
  std::uint64_t goal_id, std::uint64_t localization_epoch, bool goal_pose_footprint_verified,
  double & used_clearance,
  nav_msgs::msg::Path * global_guide_out,
  std::optional<geometry_msgs::msg::PoseStamped> * local_anchor_out)
{
  if (global_guide_out) {
    global_guide_out->poses.clear();
  }
  if (local_anchor_out) {
    local_anchor_out->reset();
  }
  const auto & grid = snapshot->grid;
  const auto * squared = snapshot->blocked_squared_cells.get();
  const bool managed_goal = std::any_of(goal_uuid.begin(), goal_uuid.end(),
      [](std::uint8_t byte) {return byte != 0;});
  const bool prior_mode = managed_goal && prior_guide_map_ &&
    prior_guide_map_->header.frame_id == grid.header.frame_id;
  const bool mapless_global_mode = !prior_mode && snapshot->global_grid.has_value() &&
    snapshot->global_grid->header.frame_id == grid.header.frame_id;
  GridSearchResult result;
  if (prior_mode) {
    const bool same_goal = global_route_ && !global_route_->optimistic &&
      global_route_->goal_uuid == goal_uuid &&
      global_route_->goal_id == goal_id && global_route_->localization_epoch == localization_epoch &&
      global_route_->path->header.frame_id == grid.header.frame_id &&
      std::hypot(global_route_->goal.position.x - goal.pose.position.x,
      global_route_->goal.position.y - goal.pose.position.y) <= 0.001 &&
      std::abs(std::remainder(tf2::getYaw(global_route_->goal.orientation) -
      tf2::getYaw(goal.pose.orientation), 2.0 * M_PI)) <= 0.001;
    nav_msgs::msg::Path remaining;
    double clearance = 0.0;
    if (same_goal) {
      remaining = remainingGuide(*global_route_->path, start, replan_start_params_.deviation_max_m);
      clearance = global_route_->clearance;
    } else {
      auto prior = runGraphSearch(
        *prior_guide_map_, start, goal, goal_pose_footprint_verified, clearance,
        &prior_blocked_squared_);
      RCLCPP_INFO(
        get_logger(), "TRACE global_route source=prior goal=%llu generation=%llu "
        "publication=%llu source_generation=%llu points=%zu length=%.3f result='%s'",
        static_cast<unsigned long long>(goal_id), static_cast<unsigned long long>(snapshot->generation),
        static_cast<unsigned long long>(snapshot->publication_sequence),
        static_cast<unsigned long long>(snapshot->source_generation),
        prior.path.poses.size(), prior.length, prior.reason.c_str());
      remaining = std::move(prior.path);
    }
    if (!remaining.poses.empty()) {
      remaining.header = grid.header;
      std::optional<geometry_msgs::msg::Point> reference_conflict;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (reference_conflict_ && reference_conflict_->goal_id == goal_id &&
          reference_conflict_->localization_epoch == localization_epoch &&
          reference_conflict_->snapshot_generation == snapshot->generation)
        {
          reference_conflict = reference_conflict_->position;
          reference_conflict_.reset();
        }
      }
      const auto begin = std::chrono::steady_clock::now();
      const auto * accepted_grid = same_goal ? &global_route_->snapshot->grid : prior_guide_map_.get();
      auto local = rejoinGlobalRoute(
        remaining, grid, *squared, clearance, safety_checker_, graph_search_, reference_conflict,
        accepted_grid, local_route_params_);
      if (local.obstruction) {
        const double wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();
        const auto rejoin = local.rejoin.value_or(geometry_msgs::msg::Point());
        RCLCPP_INFO(
          get_logger(), "TRACE local_rejoin goal=%llu generation=%llu publication=%llu "
          "source_generation=%llu cached_publication=%llu cached_source_generation=%llu "
          "blocked=%zu..%zu expanded=%d wall_ms=%.3f bypass_m=%.3f rejoin=(%.4f,%.4f) result='%s'",
          static_cast<unsigned long long>(goal_id),
          static_cast<unsigned long long>(snapshot->generation),
          static_cast<unsigned long long>(snapshot->publication_sequence),
          static_cast<unsigned long long>(snapshot->source_generation),
          static_cast<unsigned long long>(same_goal ? global_route_->snapshot->publication_sequence : 0),
          static_cast<unsigned long long>(same_goal ? global_route_->snapshot->source_generation : 0),
          local.obstruction->first, local.obstruction->last, local.route.expanded_nodes,
          wall_ms, local.bypass_length, rejoin.x, rejoin.y, local.route.reason.c_str());
      }
      result = std::move(local.route);
    }
  } else if (mapless_global_mode) {
    return planMaplessRoute(
      snapshot, start, goal, goal_uuid, goal_id, localization_epoch,
      goal_pose_footprint_verified, used_clearance, global_guide_out, local_anchor_out);
  }
  if (!result.success) {
    const bool allow_partial = partial_subgoal_continuation_enabled_ &&
      (!partial_subgoal_only_without_prior_ || !prior_mode);
    result = runGraphSearch(
      grid, start, goal, goal_pose_footprint_verified, used_clearance, squared, allow_partial);
    if (prior_mode) {
      RCLCPP_INFO(
        get_logger(), "TRACE global_route source=fused goal=%llu publication=%llu result='%s'",
        static_cast<unsigned long long>(goal_id),
        static_cast<unsigned long long>(snapshot->publication_sequence), result.reason.c_str());
    }
  }
  global_route_.reset();
  if (result.success && prior_mode) {
    global_route_ = GlobalRoute{
      goal_uuid, goal_id, localization_epoch, snapshot, goal.pose,
      std::make_shared<const nav_msgs::msg::Path>(result.path), result.clearance, false};
  }
  if (result.success || result.partial) {
    used_clearance = result.clearance;
    if (global_guide_out && global_guide_out->poses.empty()) {
      *global_guide_out = result.path;
    }
  }
  return result;
}

}  // namespace minco_planner
