// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

GridSearchResult MincoPlannerNode::runGraphSearch(
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool goal_pose_footprint_verified,
  double & used_clearance,
  const std::vector<double> * blocked_squared,
  bool allow_partial_subgoal)
{
  // 显式配置的下限按用户意图使用;自动下限必须补上栅格量化余量,否则梯子会稳定产出
  // footprint gate 必然拒绝的路径,把"窄通道不可通行"表现成目标超时。
  const double effective_floor = search_clearance_floor_configured_
    ? search_clearance_floor_
    : footprintConsistentClearanceFloor(
      inscribed_footprint_radius_, planning_grid.info.resolution,
      preferred_search_clearance_);
  RCLCPP_INFO_ONCE(
    get_logger(),
    "【图搜索净空生效】下限=%.3f m（内切半径=%.3f m 栅格分辨率=%.3f m 量化余量=%.3f m）；"
    "低于此值的路径无法通过矩形足迹门禁。",
    effective_floor, inscribed_footprint_radius_, planning_grid.info.resolution,
    gridQuantizationAllowance(planning_grid.info.resolution));
  std::vector<double> ladder {preferred_search_clearance_};
  if (clearance_relaxation_enabled_ && effective_floor + 1e-6 < preferred_search_clearance_) {
    ladder.push_back(effective_floor);
  }

  bool enable_partial = allow_partial_subgoal && partial_subgoal_continuation_enabled_;
  if (enable_partial && planning_grid.info.resolution > 0.0 && !planning_grid.data.empty()) {
    // 若目标点已在当前窗口内且整个容差圆都被实测确认占据（>= obstacle_value_threshold，而非 -1 未知），
    // 则属于明确物理不可达目标，禁止生成 partial 子目标，立即走确定性急停拒绝。
    const double g_yaw = tf2::getYaw(planning_grid.info.origin.orientation);
    const double cg = std::cos(g_yaw);
    const double sg = std::sin(g_yaw);
    const double dx = goal.pose.position.x - planning_grid.info.origin.position.x;
    const double dy = goal.pose.position.y - planning_grid.info.origin.position.y;
    const double lx = (cg * dx + sg * dy) / planning_grid.info.resolution;
    const double ly = (-sg * dx + cg * dy) / planning_grid.info.resolution;
    if (lx >= 0.0 && ly >= 0.0 &&
      lx < static_cast<double>(planning_grid.info.width) &&
      ly < static_cast<double>(planning_grid.info.height))
    {
      const int rad_cells = std::max(
        0, static_cast<int>(std::ceil(
          std::max(0.0, goal_pose_admission_params_.position_tolerance_m) /
          planning_grid.info.resolution)));
      const int cx = static_cast<int>(std::floor(lx));
      const int cy = static_cast<int>(std::floor(ly));
      bool all_known_occupied = true;
      for (int oy = -rad_cells; oy <= rad_cells && all_known_occupied; ++oy) {
        for (int ox = -rad_cells; ox <= rad_cells && all_known_occupied; ++ox) {
          const int ix = cx + ox;
          const int iy = cy + oy;
          if (ix < 0 || iy < 0 ||
            ix >= static_cast<int>(planning_grid.info.width) ||
            iy >= static_cast<int>(planning_grid.info.height))
          {
            all_known_occupied = false;
            break;
          }
          const int8_t val = planning_grid.data[
            static_cast<std::size_t>(iy) * planning_grid.info.width +
            static_cast<std::size_t>(ix)];
          if (val < obstacle_value_threshold_) {
            all_known_occupied = false;
          }
        }
      }
      if (all_known_occupied) {
        enable_partial = false;
      }
    }
  }

  const double start_goal_dist = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  const auto search_at = [&](double clearance, double endpoint_minimum = -1.0) {
    const auto begin = std::chrono::steady_clock::now();
    GridSearchOptions options;
    options.clearance = clearance;
    options.endpoint_minimum = endpoint_minimum;
    if (enable_partial) {
      options.can_stop = [&](const geometry_msgs::msg::PoseStamped & cand) {
        const double step_dist = std::hypot(
          cand.pose.position.x - start.pose.position.x,
          cand.pose.position.y - start.pose.position.y);
        if (step_dist < partial_subgoal_min_advance_m_) {
          return false;
        }
        const double cand_goal_dist = std::hypot(
          goal.pose.position.x - cand.pose.position.x,
          goal.pose.position.y - cand.pose.position.y);
        if (cand_goal_dist + 0.15 > start_goal_dist) {
          return false;
        }
        const auto taboo_now = std::chrono::steady_clock::now();
        for (const auto & entry : frontier_taboo_) {
          if (std::chrono::duration<double>(taboo_now - entry.added_at).count() <=
              frontier_taboo_ttl_sec_ &&
            std::hypot(cand.pose.position.x - entry.x, cand.pose.position.y - entry.y) <
              frontier_taboo_radius_m_)
          {
            return false;
          }
        }
        return goalPoseFootprintFree(
          cand.pose.position.x, cand.pose.position.y,
          tf2::getYaw(cand.pose.orientation), planning_grid, footprint_params_);
      };
    }
    GridSearchResult attempt = graph_search_.plan(
      planning_grid, start, goal, blocked_squared, options);
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    RCLCPP_INFO(
      get_logger(),
      "TRACE graph_search mode=%s requested_clearance=%.3f clearance=%.3f "
      "expanded=%d wall_ms=%.3f cost=%.9f length=%.3f partial=%d result='%s'",
      search_algorithm_.c_str(), clearance, attempt.clearance, attempt.expanded_nodes,
      wall_ms, attempt.cost, attempt.length, attempt.partial ? 1 : 0, attempt.reason.c_str());
    return attempt;
  };

  GridSearchResult last_result;
  GridSearchResult best_partial;
  double best_partial_goal_dist = std::numeric_limits<double>::infinity();
  for (std::size_t level = 0; level < ladder.size(); ++level) {
    const double clearance = ladder[level];
    if (last_result.expanded_nodes > 0 && last_result.clearance <= clearance + 1e-9) {
      break;  // An endpoint relaxation already searched this effective clearance.
    }
    GridSearchResult attempt = search_at(clearance);
    if (attempt.success) {
      // 首选档比窄通道半宽大时,绕远路也能在这一档成功,梯子就永远不降档。
      // 路径相对直线明显绕行时,用下限档再搜一次,明显更短才取代;
      // 安全仍由矩形足迹门禁与局部修复裁定。
      if (level == 0 && ladder.size() > 1 && attempt.clearance > ladder[1] + 1e-9) {
        const double straight = std::hypot(
          goal.pose.position.x - start.pose.position.x,
          goal.pose.position.y - start.pose.position.y);
        if (shouldProbeRelaxedClearance(attempt.length, straight, clearance_detour_ratio_)) {
          GridSearchResult relaxed = search_at(ladder[1]);
          if (relaxed.success &&
            shouldTakeRelaxedRoute(attempt.length, relaxed.length, clearance_detour_ratio_))
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), log_throttle_ms_,
              "【绕行回退】首选净空 %.3f m 路径 %.2f m（直线 %.2f m）明显绕行，"
              "改用净空 %.3f m 路径 %.2f m；最终仍以航向感知足迹门禁为准。",
              clearance, attempt.length, straight, ladder[1], relaxed.length);
            used_clearance = relaxed.clearance;
            return relaxed;
          }
        }
      }
      used_clearance = attempt.clearance;
      if (clearance + 1e-6 < preferred_search_clearance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【规划净空放宽】图搜索从 %.3f m 放宽到 %.3f m 后成功；最终仍以航向感知足迹门禁为准。",
          preferred_search_clearance_, clearance);
      }
      return attempt;
    }
    if (attempt.partial && !attempt.path.poses.empty()) {
      const auto & p = attempt.path.poses.back().pose.position;
      const double rem_dist = std::hypot(
        goal.pose.position.x - p.x, goal.pose.position.y - p.y);
      if (!best_partial.partial || rem_dist + 0.35 < best_partial_goal_dist) {
        best_partial = attempt;
        best_partial_goal_dist = rem_dist;
      }
    }
    last_result = attempt;
  }

  // 最后一档:目标格净空低于 footprint 一致下限,但目标位姿本身已通过同一套矩形
  // 足迹门禁。目标格净空是各向同性代理量,矩形门禁是精确判定;此时继续用代理量
  // 否决精确判定,结果是一条路径都产不出来(domain 187 目标 9:目标格净空约 0.26 m
  // 对下限 0.341 m,图搜索报 goal occupied,7 个规划周期零可执行计划直到看门狗耗尽)。
  // 放宽只作用于"目标格是否可作为终点",搜索器随后按实测目标净空重规划整条路径,
  // 安全性仍由重解 MINCO 之后的矩形足迹门禁与局部修复裁定,不是靠放宽判据换成功。
  const double relaxed_endpoint_min = gridQuantizationAllowance(planning_grid.info.resolution);
  if (goal_pose_footprint_verified && endpoint_clearance_relaxation_enabled_ &&
    (last_result.expanded_nodes == 0 || last_result.clearance > effective_floor + 1e-9) &&
    !last_result.success && relaxed_endpoint_min + 1e-6 < effective_floor)
  {
    GridSearchResult attempt = search_at(effective_floor, relaxed_endpoint_min);
    if (attempt.success) {
      used_clearance = attempt.clearance;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终点净空放宽】终点下限从 %.3f m 放宽到 %.3f m 后成功；目标已通过矩形足迹门禁，"
        "轨迹安全仍由该门禁最终裁定。",
        effective_floor, relaxed_endpoint_min);
      return attempt;
    }
    last_result = attempt;
  }

  if (best_partial.partial) {
    used_clearance = best_partial.clearance;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【远目标局部前沿接续】目标超出当前观测连通域，推进至安全前沿子目标 (%.3f, %.3f) "
      "length=%.2f m clearance=%.3f m reason='%s'。",
      best_partial.path.poses.back().pose.position.x,
      best_partial.path.poses.back().pose.position.y,
      best_partial.length, best_partial.clearance, best_partial.reason.c_str());
    return best_partial;
  }

  used_clearance = ladder.back();
  return last_result;
}

void MincoPlannerNode::publishRollingAnchor(
  const std::optional<geometry_msgs::msg::PoseStamped> & anchor,
  const std::string & frame_id, const rclcpp::Time & stamp, const char * label)
{
  if (!rolling_anchor_pub_ || rolling_anchor_pub_->get_subscription_count() == 0U) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "minco_rolling_anchor";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  if (anchor.has_value()) {
    const std::string active_frame =
      anchor->header.frame_id.empty() ? frame_id : anchor->header.frame_id;
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = active_frame;
    sphere.header.stamp = stamp;
    sphere.ns = "minco_rolling_anchor";
    sphere.id = 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = anchor->pose;
    sphere.pose.position.z = 0.14;
    sphere.scale.x = 0.28;
    sphere.scale.y = 0.28;
    sphere.scale.z = 0.28;
    sphere.color.r = 0.0F;
    sphere.color.g = 0.96F;
    sphere.color.b = 1.0F;
    sphere.color.a = 0.95F;
    markers.markers.push_back(sphere);

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = active_frame;
    arrow.header.stamp = stamp;
    arrow.ns = "minco_rolling_anchor";
    arrow.id = 2;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = anchor->pose;
    arrow.pose.position.z = 0.14;
    arrow.scale.x = 0.55;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.08;
    arrow.color.r = 1.0F;
    arrow.color.g = 1.0F;
    arrow.color.b = 1.0F;
    arrow.color.a = 0.95F;
    markers.markers.push_back(arrow);

    visualization_msgs::msg::Marker ring;
    ring.header.frame_id = active_frame;
    ring.header.stamp = stamp;
    ring.ns = "minco_rolling_anchor";
    ring.id = 3;
    ring.type = visualization_msgs::msg::Marker::CYLINDER;
    ring.action = visualization_msgs::msg::Marker::ADD;
    ring.pose = anchor->pose;
    ring.pose.orientation.x = 0.0;
    ring.pose.orientation.y = 0.0;
    ring.pose.orientation.z = 0.0;
    ring.pose.orientation.w = 1.0;
    ring.pose.position.z = 0.05;
    ring.scale.x = 0.44;
    ring.scale.y = 0.44;
    ring.scale.z = 0.03;
    ring.color.r = 1.0F;
    ring.color.g = 0.84F;
    ring.color.b = 0.0F;
    ring.color.a = 0.85F;
    markers.markers.push_back(ring);

    if (label && label[0] != '\0') {
      visualization_msgs::msg::Marker text;
      text.header.frame_id = active_frame;
      text.header.stamp = stamp;
      text.ns = "minco_rolling_anchor";
      text.id = 4;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = anchor->pose;
      text.pose.orientation.x = 0.0;
      text.pose.orientation.y = 0.0;
      text.pose.orientation.z = 0.0;
      text.pose.orientation.w = 1.0;
      text.pose.position.z = 0.44;
      text.scale.z = 0.22;
      text.color.r = 0.0F;
      text.color.g = 0.98F;
      text.color.b = 1.0F;
      text.color.a = 0.95F;
      text.text = label;
      markers.markers.push_back(text);
    }
  }
  rolling_anchor_pub_->publish(markers);
}


}  // namespace minco_planner
