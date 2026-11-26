// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

void MincoPlannerNode::applyTerminalYawRelocationAndSidestep(
  const std::shared_ptr<const PlanningMapSnapshot> & map_snapshot,
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> & clearance_esdf,
  double active_goal_yaw, bool active_is_final_goal,
  ReferenceTrajectory & reference, FootprintSafetyResult & safety)
{
  // 终端原地转向重定位与全向侧移掉头：
  // 1) 当冲突全部落在终端近域窗口（含起点即终点的纯原地转向）时，先尝试沿路径提前转向；
  // 2) 当沿路径候选全被拒，或虽安全但原地大角度转向的足迹净空极薄（< 6 cm，易被厘米级
  //    轮系滑移推入墙壁）时，利用四舵轮全向能力先保持当前航向侧移到开阔 ESDF 区域掉头，
  //    再保持目标航向平移至终点。所有候选仍过同一套零冲突矩形足迹、扫掠与洞口对齐门禁。
  const double comfortable_rotation_clearance_m = terminal_rotation_comfortable_clearance_m_;
  const std::size_t relocation_tail_start = terminalCoincidentTailStart(reference);
  const bool unsafe_terminal_rotation = !safety.safe && (
    collisionsConfinedToTerminalApproach(
      reference, safety, terminal_yaw_relocation_params_.terminal_window_length) ||
    (relocation_tail_start == 0U && reference.points.size() >= 2U));
  const bool tight_terminal_rotation = active_is_final_goal && safety.safe &&
    hasTightInPlaceTerminalRotation(
      reference, terminal_rotation_tight_yaw_span_rad_, comfortable_rotation_clearance_m);
  if (terminal_yaw_relocation_enabled_ &&
    (unsafe_terminal_rotation || tight_terminal_rotation))
  {
    const auto minRotationClearanceOf = [](const ReferenceTrajectory & traj) {
      double min_rot = std::numeric_limits<double>::infinity();
      double min_all = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0U; i < traj.points.size(); ++i) {
        const auto & pt = traj.points[i];
        if (!std::isfinite(pt.clearance)) {
          continue;
        }
        min_all = std::min(min_all, pt.clearance);
        bool rotating = std::abs(pt.yaw_rate) > 0.10;
        if (!rotating && i + 1U < traj.points.size()) {
          const double dyaw = std::atan2(
            std::sin(traj.points[i + 1U].yaw - pt.yaw),
            std::cos(traj.points[i + 1U].yaw - pt.yaw));
          rotating = std::abs(dyaw) > 0.01;
        }
        if (!rotating && i > 0U) {
          const double dyaw = std::atan2(
            std::sin(pt.yaw - traj.points[i - 1U].yaw),
            std::cos(pt.yaw - traj.points[i - 1U].yaw));
          rotating = std::abs(dyaw) > 0.01;
        }
        if (rotating) {
          min_rot = std::min(min_rot, pt.clearance);
        }
      }
      const double result = std::isfinite(min_rot) ? min_rot : min_all;
      return std::isfinite(result) ? result : -std::numeric_limits<double>::infinity();
    };

    const std::size_t tail_start = relocation_tail_start;
    const std::size_t window_start = std::min(
      tail_start,
      terminalApproachWindowStart(
        reference, terminal_yaw_relocation_params_.terminal_window_length));
    const std::size_t original_collisions = safety.collisions.size();
    const double original_min_clearance = safety.safe ?
      minRotationClearanceOf(reference) : -std::numeric_limits<double>::infinity();
    const std::vector<std::size_t> candidates = tail_start > 0U ?
      terminalYawRelocationCandidates(
        reference, static_cast<std::size_t>(terminal_yaw_relocation_max_candidates_)) :
      std::vector<std::size_t>{};
    std::ostringstream candidate_list;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (i != 0U) {
        candidate_list << ',';
      }
      candidate_list << candidates[i];
    }
    std::size_t first_conflict = 0;
    std::size_t last_conflict = 0;
    if (!safety.collisions.empty()) {
      first_conflict = safety.collisions.front().trajectory_index;
      last_conflict = first_conflict;
      for (const CollisionSample & collision : safety.collisions) {
        first_conflict = std::min(first_conflict, collision.trajectory_index);
        last_conflict = std::max(last_conflict, collision.trajectory_index);
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【终端航向重定位】尝试：tail_start=%zu window_start=%zu candidates=[%s] "
      "conflict_span=[%zu,%zu] collisions=%zu tight=%d min_clearance=%.3f.",
      tail_start, window_start, candidate_list.str().c_str(), first_conflict, last_conflict,
      original_collisions, tight_terminal_rotation ? 1 : 0,
      std::isfinite(original_min_clearance) ? original_min_clearance : -1.0);
    std::size_t built = 0;
    std::ostringstream rejected;
    std::optional<ReferenceTrajectory> best_candidate;
    FootprintSafetyResult best_safety;
    double best_clearance = original_min_clearance;
    std::size_t best_rotation_index = 0U;
    bool best_is_sidestep = false;
    double best_turn_x = 0.0;
    double best_turn_y = 0.0;

    auto evaluateCandidate = [&](ReferenceTrajectory & candidate, const std::string & label,
      FootprintSafetyResult & out_safety, double & out_clearance) -> bool {
      candidate.header.stamp = now();
      applyWheelSpeedTimeScaling(candidate, wheel_speed_time_scaling_params_);
      // 只补净空标注,绝不调用 planYaw:planYaw 会在末点再追加一段原地转向,
      // 把刚刚移开的问题原样搬回来。
      annotateClearance(candidate, *map_snapshot);
      out_safety = safety_checker_.check(candidate, planning_grid, clearance_esdf.get());
      if (!out_safety.safe) {
        rejected << label << ":footprint;";
        return false;
      }
      if (tunnel_yaw_params_.enabled && clearance_esdf) {
        auto candidate_center = candidate;
        annotatePositionClearance(candidate_center, *map_snapshot);
        auto candidate_tunnels = detectTunnelSegments(
          candidate_center, *clearance_esdf, tunnel_yaw_params_);
        resolveTerminalTunnelGoalAlignment(
          candidate_tunnels, candidate_center, active_goal_yaw,
          active_is_final_goal, tunnel_yaw_params_);
        if (!forwardTunnelEntriesAligned(candidate, candidate_tunnels, tunnel_yaw_params_)) {
          rejected << label << ":tunnel_align;";
          return false;
        }
      }
      out_clearance = minRotationClearanceOf(candidate);
      return true;
    };

    for (const std::size_t rotation_index : candidates) {
      ReferenceTrajectory candidate;
      if (!relocateTerminalYawRotation(
          reference, active_goal_yaw, rotation_index, terminal_yaw_relocation_params_, &candidate))
      {
        rejected << rotation_index << ":build_failed;";
        continue;
      }
      ++built;
      FootprintSafetyResult candidate_safety;
      double candidate_clearance = -std::numeric_limits<double>::infinity();
      if (!evaluateCandidate(
          candidate, std::to_string(rotation_index), candidate_safety, candidate_clearance))
      {
        continue;
      }
      if (!best_candidate || candidate_clearance > best_clearance) {
        best_candidate = std::move(candidate);
        best_safety = std::move(candidate_safety);
        best_clearance = candidate_clearance;
        best_rotation_index = rotation_index;
        best_is_sidestep = false;
      }
      if (best_clearance >= comfortable_rotation_clearance_m) {
        break;
      }
    }

    // 沿路径重定位全部失败或净空仍不足 comfortable_rotation_clearance_m 时，搜索附近开阔 ESDF 点进行全向侧移掉头。
    // 若终点位于窄通道内（如贴壁掉头或进洞反转），额外尝试在起点静止/低速附近（anchor=0）先侧移掉头再沿原安全轨迹进洞。
    if ((!best_candidate || best_clearance < comfortable_rotation_clearance_m) &&
        clearance_esdf && clearance_esdf->available() && !reference.points.empty())
    {
      std::vector<std::size_t> anchor_indices;
      if (tail_start == 0U ||
          (reference.points[tail_start].s <= 0.60 && reference.points.front().v <= 0.15))
      {
        anchor_indices.push_back(0U);
      } else {
        anchor_indices.push_back(tail_start);
        if (reference.points.front().v <= 0.25) {
          anchor_indices.push_back(0U);
        }
      }
      for (const std::size_t anchor_index : anchor_indices) {
        if (best_candidate && best_clearance >= sidestep_accept_clearance_m_) {
          break;
        }
        const ReferencePoint & pivot = reference.points[anchor_index];
        const ReferencePoint & goal_pt = reference.points.back();
        const double yaw_diff = std::abs(std::atan2(
          std::sin(active_goal_yaw - pivot.yaw), std::cos(active_goal_yaw - pivot.yaw)));
        if (yaw_diff <= 0.25 && safety.safe) {
          continue;
        }
        const double half_l = 0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_;
        const double half_w = 0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_;
        const double circumradius = std::hypot(half_l, half_w);
        const double min_esdf = circumradius + sidestep_min_esdf_margin_m_;
        const double desired_esdf = circumradius + sidestep_desired_esdf_margin_m_;

        struct SidestepSpot
        {
          double x;
          double y;
          double esdf;
          double detour;
          bool preferred;
        };
        std::vector<SidestepSpot> spots;
        const double radius_scale = sidestep_max_radius_m_ / 0.45;
        const std::array<double, 6> radii{
          0.12 * radius_scale, 0.18 * radius_scale, 0.24 * radius_scale,
          0.30 * radius_scale, 0.38 * radius_scale, sidestep_max_radius_m_};
        std::vector<Eigen::Vector2d> dirs;
        dirs.reserve(17U);
        const Eigen::Vector2d grad = clearance_esdf->getGradient(pivot.x, pivot.y);
        if (grad.allFinite() && grad.norm() > 1e-4) {
          dirs.push_back(grad.normalized());
        }
        for (int k = 0; k < 16; ++k) {
          const double angle = (2.0 * M_PI * static_cast<double>(k)) / 16.0;
          dirs.emplace_back(std::cos(angle), std::sin(angle));
        }
        for (double r : radii) {
          for (const auto & d : dirs) {
            const double tx = pivot.x + r * d.x();
            const double ty = pivot.y + r * d.y();
            const double dist = clearance_esdf->getDistance(tx, ty);
            if (!std::isfinite(dist) || dist < min_esdf) {
              continue;
            }
            bool duplicate = false;
            for (const auto & existing : spots) {
              if (std::hypot(existing.x - tx, existing.y - ty) < 0.04) {
                duplicate = true;
                break;
              }
            }
            if (duplicate) {
              continue;
            }
            const double detour =
              anchor_index == 0U
                ? (2.0 * std::hypot(tx - pivot.x, ty - pivot.y))
                : (std::hypot(tx - pivot.x, ty - pivot.y) +
                   std::hypot(goal_pt.x - tx, goal_pt.y - ty));
            spots.push_back({tx, ty, dist, detour, dist >= desired_esdf});
          }
        }
        std::sort(
          spots.begin(), spots.end(),
          [](const SidestepSpot & a, const SidestepSpot & b) {
            if (a.preferred != b.preferred) {
              return a.preferred;
            }
            if (std::abs(a.detour - b.detour) > 0.04) {
              return a.detour < b.detour;
            }
            return a.esdf > b.esdf;
          });
        const std::size_t max_sidestep_tries = std::min<std::size_t>(
          spots.size(), static_cast<std::size_t>(sidestep_max_tries_));
        for (std::size_t s_idx = 0U; s_idx < max_sidestep_tries; ++s_idx) {
          const auto & spot = spots[s_idx];
          ReferenceTrajectory candidate;
          if (!buildSidestepTerminalYawTurnaround(
              reference, active_goal_yaw, anchor_index, spot.x, spot.y,
              terminal_yaw_relocation_params_, &candidate))
          {
            rejected << "sidestep" << anchor_index << "_" << s_idx << ":build_failed;";
            continue;
          }
          ++built;
          FootprintSafetyResult candidate_safety;
          double candidate_clearance = -std::numeric_limits<double>::infinity();
          if (!evaluateCandidate(
              candidate, "sidestep" + std::to_string(anchor_index) + "_" + std::to_string(s_idx),
              candidate_safety, candidate_clearance))
          {
            continue;
          }
          if (!best_candidate || candidate_clearance > best_clearance + 0.02) {
            best_candidate = std::move(candidate);
            best_safety = std::move(candidate_safety);
            best_clearance = candidate_clearance;
            best_is_sidestep = true;
            best_turn_x = spot.x;
            best_turn_y = spot.y;
          }
          if (best_clearance >= sidestep_accept_clearance_m_) {
            break;
          }
        }
      }
    }

    const bool accept_relocated = best_candidate.has_value() &&
      (!safety.safe || best_clearance > original_min_clearance + sidestep_min_improvement_m_);
    if (accept_relocated) {
      if (best_is_sidestep) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【终端航向侧移掉头成功】turn=(%.3f,%.3f) tail_start=%zu "
          "cleared_collisions=%zu clearance=%.3f->%.3f points=%zu。",
          best_turn_x, best_turn_y, tail_start, original_collisions,
          std::isfinite(original_min_clearance) ? original_min_clearance : -1.0,
          best_clearance, best_candidate->points.size());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【终端航向重定位成功】rotation_index=%zu tail_start=%zu "
          "window_start=%zu candidates=%zu built=%zu cleared_collisions=%zu "
          "clearance=%.3f points=%zu rejected=[%s]。",
          best_rotation_index, tail_start, window_start, candidates.size(), built,
          original_collisions, best_clearance, best_candidate->points.size(),
          rejected.str().c_str());
      }
      reference = std::move(*best_candidate);
      safety = std::move(best_safety);
    } else if (!safety.safe) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终端航向重定位失败】tail_start=%zu window_start=%zu candidates=[%s] "
        "built=%zu collisions=%zu rejected=%s；保留现有足迹拒绝路径。",
        tail_start, window_start, candidate_list.str().c_str(), built, original_collisions,
        rejected.str().c_str());
    }
  }
}

void MincoPlannerNode::tryEscapePrefixAndLocalReverse(
  const std::shared_ptr<const PlanningMapSnapshot> & map_snapshot,
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> & clearance_esdf,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double start_yaw, double goal_yaw,
  double & active_goal_yaw, bool & active_is_final_goal,
  const std::optional<double> & exploration_lidar_fov,
  const InitialKinematicState * seed_ptr, double used_clearance,
  bool goal_pose_footprint_verified,
  const std::function<bool()> & rolling_budget_exceeded,
  ReferenceTrajectory & reference, FootprintSafetyResult & safety,
  MincoOptimizationTrace & selected_trace, bool & has_spliced_tail,
  EscapePrefixDecision & escape_decision)
{
  // 贴障起步时 yaw 规划在起点就转向，角点扫进障碍，逃逸前缀的深度/扫掠两道上界都拒绝它。
  // 换成"先保持起点 yaw 平移驶离再追回原 yaw"的候选，重新过同一套门禁与逃逸判据；
  // 几何路径和终端 yaw 不变。只在逃逸开关打开且冲突紧贴起点时尝试。
  if (!safety.safe && !escape_decision.allowed && escape_prefix_params_.enabled &&
    !safety.collisions.empty() &&
    std::min_element(
      safety.collisions.begin(), safety.collisions.end(),
      [](const CollisionSample & a, const CollisionSample & b) {
        return a.trajectory_index < b.trajectory_index;
      })->trajectory_index == 0U)
  {
    std::ostringstream tried;
    std::vector<double> hold_lengths {0.20, escape_prefix_params_.max_prefix_length_m};
    if (escape_prefix_params_.monotone_max_contact_depth_m > 0.0 &&
      escape_prefix_params_.monotone_max_prefix_length_m > escape_prefix_params_.max_prefix_length_m)
    {
      hold_lengths.push_back(escape_prefix_params_.monotone_max_prefix_length_m);
    }
    for (const double hold_length : hold_lengths) {
      ReferenceTrajectory candidate;
      if (!holdStartYawForEscape(
          reference, hold_length, terminal_yaw_relocation_params_.yaw_rate_limit, &candidate))
      {
        tried << hold_length << ":build_failed;";
        continue;
      }
      annotateClearance(candidate, *map_snapshot);
      const FootprintSafetyResult candidate_safety =
        safety_checker_.check(candidate, planning_grid);
      EscapePrefixDecision candidate_escape = candidate_safety.safe ?
        EscapePrefixDecision{} :
        evaluateEscapePrefix(candidate.points, candidate_safety.collisions, escape_prefix_params_);
      if (candidate_escape.allowed &&
        !escapePrefixContactAdmitted(
          candidate, candidate_escape, footprint_params_, planning_grid, escape_prefix_params_))
      {
        candidate_escape.prefix_end = 0U;
        candidate_escape.allowed = false;
      }
      if (!candidate_safety.safe && !candidate_escape.allowed) {
        tried << hold_length << ":collisions=" << candidate_safety.collisions.size() <<
          ",prefix_end=" << candidate_escape.candidate_prefix_end <<
          ",yaw_sweep=" << candidate_escape.prefix_yaw_sweep_rad << ';';
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【脱困航向保持】接受：长度=%.2f m safe=%d 冲突=%zu 前缀末端=%zu。",
        hold_length, candidate_safety.safe ? 1 : 0, candidate_safety.collisions.size(),
        candidate_escape.prefix_end);
      reference = std::move(candidate);
      safety = candidate_safety;
      escape_decision = candidate_escape;
      break;
    }
    if (!safety.safe && !escape_decision.allowed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【脱困航向保持失败】已尝试：%s", tried.str().c_str());
    }
  }
  // 局部倒退脱困：当近端（如窄通道/隧道口附近贴障起步或前向拐弯角点擦墙）存在足迹冲突且零位点尚未被完全锁死时，
  // 优先尝试保持当前航向直线倒退（v_x < 0）或沿局部 ESDF 梯度平移 0.25~0.65 m 到更高净空的局部锚点，
  // 再从局部锚点重接剩余路径，整条组合候选必须严格通过同一零冲突矩形足迹与扫掠门禁。
  if (!safety.safe && !safety.collisions.empty() && !rolling_budget_exceeded() &&
    (!seed_ptr || !seed_ptr->valid || seed_ptr->velocity.norm() <= 0.20))
  {
    const std::size_t earliest_conflict = std::min_element(
      safety.collisions.begin(), safety.collisions.end(),
      [](const CollisionSample & a, const CollisionSample & b) {
        return a.trajectory_index < b.trajectory_index;
      })->trajectory_index;
    if (earliest_conflict <= 12U && clearance_esdf && clearance_esdf->available()) {
      const double sx = start.pose.position.x;
      const double sy = start.pose.position.y;
      const double start_d = clearance_esdf->getDistance(sx, sy);
      std::vector<Eigen::Vector2d> dirs{
        Eigen::Vector2d(-std::cos(start_yaw), -std::sin(start_yaw)),
        Eigen::Vector2d(std::cos(start_yaw), std::sin(start_yaw))};
      const Eigen::Vector2d grad = clearance_esdf->getGradient(sx, sy);
      if (grad.norm() > 0.20) {
        dirs.push_back(grad.normalized());
      }
      const std::array<double, 3> backoff_dists{0.25, 0.45, 0.65};
      bool escaped = false;
      for (const auto & dir : dirs) {
        if (escaped || rolling_budget_exceeded()) {
          break;
        }
        for (const double dist : backoff_dists) {
          if (rolling_budget_exceeded()) {
            break;
          }
          const double ax = sx + dir.x() * dist;
          const double ay = sy + dir.y() * dist;
          const double ad = clearance_esdf->getDistance(ax, ay);
          if (!std::isfinite(ad) || ad <= std::max(0.32, start_d + 0.02)) {
            continue;
          }
          ReferenceTrajectory backoff;
          backoff.header = reference.header;
          const double v_cap = std::max(0.25, terminal_yaw_relocation_params_.sidestep_max_speed);
          const double a_cap = std::max(0.40, terminal_yaw_relocation_params_.sidestep_max_accel);
          const double dur = std::max({
            0.35, 1.875 * dist / v_cap, std::sqrt(5.7735026919 * dist / a_cap)});
          const double sample_dt = std::max(0.02, terminal_yaw_relocation_params_.sample_period);
          const int steps = std::max(4, static_cast<int>(std::ceil(dur / sample_dt)));
          backoff.points.reserve(static_cast<std::size_t>(steps + 1));
          for (int k = 0; k <= steps; ++k) {
            const double u = static_cast<double>(k) / static_cast<double>(steps);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double b = u3 * (10.0 - 15.0 * u + 6.0 * u2);
            const double db = (30.0 * u2 - 60.0 * u3 + 30.0 * u2 * u2) / dur;
            const double ddb = (60.0 * u - 180.0 * u2 + 120.0 * u3) / (dur * dur);
            ReferencePoint pt;
            pt.t = dur * u;
            pt.s = dist * b;
            pt.x = sx + dir.x() * dist * b;
            pt.y = sy + dir.y() * dist * b;
            pt.vx = dir.x() * dist * db;
            pt.vy = dir.y() * dist * db;
            pt.v = std::hypot(pt.vx, pt.vy);
            pt.ax = dir.x() * dist * ddb;
            pt.ay = dir.y() * dist * ddb;
            pt.yaw = start_yaw;
            pt.yaw_rate = 0.0;
            backoff.points.push_back(pt);
          }
          if (!safety_checker_.check(backoff, planning_grid, clearance_esdf.get()).safe) {
            continue;
          }
          geometry_msgs::msg::PoseStamped anchor_pose = start;
          anchor_pose.pose.position.x = ax;
          anchor_pose.pose.position.y = ay;
          double local_clearance = used_clearance;
          const GridSearchResult tail_search = runGraphSearch(
            planning_grid, anchor_pose, goal, goal_pose_footprint_verified,
            local_clearance, map_snapshot->blocked_squared_cells.get());
          if (!tail_search.success || tail_search.path.poses.size() < 2U) {
            continue;
          }
          nav_msgs::msg::Path tail_guide_path = tail_search.path;
          double tail_goal_yaw = goal_yaw;
          bool tail_is_final = !tail_search.partial;
          bool tail_spliced = false;
          if (rolling_horizon_enabled_ &&
            pathArcLength(tail_guide_path) >
              rolling_horizon_length_m_ + rolling_min_tail_length_m_)
          {
            SplitGuideResult tail_split = splitPathAtArcLength(
              tail_guide_path, rolling_horizon_length_m_, rolling_min_tail_length_m_);
            if (tail_split.valid) {
              tail_guide_path = std::move(tail_split.prefix);
              tail_goal_yaw = std::atan2(
                tail_split.junction_tangent.y(), tail_split.junction_tangent.x());
              tail_is_final = false;
              tail_spliced = true;
            }
          }
          MincoOptimizationTrace tail_trace;
          ReferenceTrajectory tail_traj = optimizer_.optimize(
            tail_guide_path, clearance_esdf.get(), nullptr, nullptr,
            &planning_grid, &safety_checker_, &tail_trace, nullptr,
            map_snapshot->blocked_squared_cells.get());
          if (!tail_traj.valid()) {
            continue;
          }
          tail_traj.header.stamp = now();
          planYaw(
            tail_traj, *map_snapshot, start_yaw, tail_goal_yaw, tail_is_final,
            exploration_lidar_fov);
          ReferenceTrajectory combined = spliceReferenceTrajectories(
            backoff, tail_traj, backoff.totalTime());
          if (!combined.valid()) {
            continue;
          }
          annotateClearance(combined, *map_snapshot);
          const FootprintSafetyResult combined_safety =
            safety_checker_.check(combined, planning_grid, clearance_esdf.get());
          if (!combined_safety.safe) {
            continue;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "【局部倒退脱困成功】anchor=(%.3f,%.3f) dist=%.2f m clearance=%.3f->%.3f points=%zu。",
            ax, ay, dist, start_d, ad, combined.points.size());
          reference = std::move(combined);
          safety = combined_safety;
          if (tail_spliced) {
            has_spliced_tail = true;
            active_goal_yaw = tail_goal_yaw;
            active_is_final_goal = false;
          }
          selected_trace = std::move(tail_trace);
          escaped = true;
          break;
        }
      }
    }
  }
}

void MincoPlannerNode::finalizeAndCommitReference(
  PlanGoalCommitContext & ctx, const EscapePrefixDecision & escape_decision)
{
  const std::uint64_t goal_id = ctx.goal_id;
  const std::uint64_t localization_epoch = ctx.localization_epoch;
  const std::uint64_t plan_request_sequence = ctx.plan_request_sequence;
  const std::uint64_t map_publication_sequence = ctx.map_publication_sequence;
  const bool report_status = ctx.report_status;
  const bool keep_active_reference = ctx.keep_active_reference;
  const auto & plan_wall_begin = ctx.plan_wall_begin;
  const auto & plan_begin_time = ctx.plan_begin_time;
  const auto & committed_for_start = ctx.committed_for_start;
  const auto & map_snapshot = ctx.map_snapshot;
  const std::uint64_t map_health_epoch = ctx.map_health_epoch;
  const auto & planning_grid = ctx.planning_grid;
  const auto & clearance_esdf = ctx.clearance_esdf;
  const auto & start = ctx.start;
  const double active_goal_yaw = ctx.active_goal_yaw;
  const bool active_is_final_goal = ctx.active_is_final_goal;
  const auto & replan_start = ctx.replan_start;
  const bool replan_frame_matches = ctx.replan_frame_matches;
  const double replan_elapsed_sec = ctx.replan_elapsed_sec;
  const double measured_start_x = ctx.measured_start_x;
  const double measured_start_y = ctx.measured_start_y;
  const auto & search_result = ctx.search_result;
  const double search_wall_ms = ctx.search_wall_ms;
  const double minco_wall_ms = ctx.minco_wall_ms;
  const auto & exploration_lidar_fov = ctx.exploration_lidar_fov;
  auto & reference = ctx.reference;
  auto & safety = ctx.safety;
  const auto & selected_trace = ctx.selected_trace;
  const bool has_spliced_tail = ctx.has_spliced_tail;
  const bool esdf_fallback_used = ctx.esdf_fallback_used;
  const bool local_repair_used = ctx.local_repair_used;
  const auto & elapsedWallMs = ctx.elapsed_wall_ms;
  const auto & fail = ctx.fail;

  TrajectoryQualityMetrics quality = quality_evaluator_.evaluate(
    reference, clearance_esdf.get(), footprintSamples(planning_grid),
    selected_trace.segment_durations);
  for (const CollisionSample & collision : safety.collisions) {
    if (collision.swept) {
      ++quality.swept_collision_count;
    } else {
      ++quality.footprint_collision_count;
    }
  }
  if (!quality.finite || !quality.strictly_monotonic_time) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】导数包含非有限值或时间不单调。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  // Nominal commits only: escape-from-contact may need a short lateral shove.
  // d21 gen173 committed length_ratio=5.496 lateral=7.424 and drove north of spawn.
  if (safety.safe && !admitsCommitGeometry(quality, commit_geometry_limits_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】绕行轨迹提交前未通过几何约束（length_ratio=%.3f max=%.3f "
      "lateral=%.3f m max=%.3f m path_length=%.2f direct=%.2f）。",
      quality.length_ratio, commit_geometry_limits_.max_length_ratio,
      quality.max_lateral_deviation, commit_geometry_limits_.max_lateral_deviation_m,
      quality.path_length, quality.direct_length);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NO_PATH,
      map_snapshot->generation);
    return;
  }
  // 小陀螺：开启时对整条参考生成自旋 yaw（开阔段自旋，窄段前 spin_align_lookahead 起回到规划 yaw，
  // 窄段后 spin_resume_after 再自旋），重做轮速时间缩放和足迹门禁；门禁不过就沿用规划 yaw，
  // 授权按原规则判定。
  std::uint8_t yaw_authority = selectYawAuthority(
    reference, force_body_yaw_follow_, body_yaw_follow_clearance_);
  std::vector<TunnelSegment> forward_tunnels;
  if (tunnel_yaw_params_.enabled && clearance_esdf) {
    auto center = reference;
    annotatePositionClearance(center, *map_snapshot);
    forward_tunnels = detectTunnelSegments(center, *clearance_esdf, tunnel_yaw_params_);
    resolveTerminalTunnelGoalAlignment(
      forward_tunnels, center, active_goal_yaw, active_is_final_goal, tunnel_yaw_params_);
  }
  if (!forwardTunnelEntriesAligned(reference, forward_tunnels, tunnel_yaw_params_)) {
    std::size_t bad_index = 0U;
    double bad_yaw = 0.0;
    double bad_target = 0.0;
    double bad_width = 0.0;
    for (const auto & tunnel : forward_tunnels) {
      if (!tunnel.forward_alignment || tunnel.axis_yaw.empty()) {
        continue;
      }
      const double entry_s = reference.points[tunnel.begin].s - tunnel_yaw_params_.front_extent;
      std::size_t first = tunnel.begin;
      while (first > 1U && reference.points[first - 1U].s >= entry_s) {
        --first;
      }
      for (std::size_t idx = first; idx <= tunnel.end; ++idx) {
        const auto local_index = idx > tunnel.begin ? idx - tunnel.begin : 0U;
        const double target = tunnel.axis_yaw[local_index] + tunnel_yaw_params_.yaw_offset;
        if (std::abs(std::atan2(
            std::sin(reference.points[idx].yaw - target),
            std::cos(reference.points[idx].yaw - target))) > 0.20)
        {
          bad_index = idx;
          bad_yaw = reference.points[idx].yaw;
          bad_target = target;
          bad_width = tunnel.width_estimate;
          break;
        }
      }
      if (bad_width > 0.0) {
        break;
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "【洞口对齐拒绝】目标=%llu 快照=%llu，候选进入窄通道前未对向入口 "
      "(idx=%zu/%zu yaw=%.3f target=%.3f width=%.3f)。",
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(map_snapshot->publication_sequence),
      bad_index, reference.points.size(), bad_yaw, bad_target, bad_width);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  bool robot_in_or_arming_tunnel = false;
  if (tunnel_lock_gimbal_ && !forward_tunnels.empty() && !reference.points.empty()) {
    for (const auto & tunnel : forward_tunnels) {
      const double dyn_pre = computeDynamicPreAlignDistance(
        reference, tunnel, reference.points.front().yaw,
        replan_start.yaw_rate, tunnel_yaw_params_,
        spin_yaw_params_.blend_rate_limit, spin_yaw_params_.angular_accel);
      const TunnelTraversalPhase start_phase = classifyTunnelTraversalPhase(
        reference, 0U, tunnel, dyn_pre, tunnel_yaw_params_);
      if (start_phase == TunnelTraversalPhase::kArmed ||
        start_phase == TunnelTraversalPhase::kCommitted ||
        start_phase == TunnelTraversalPhase::kExit)
      {
        robot_in_or_arming_tunnel = true;
        break;
      }
    }
  }
  if (robot_in_or_arming_tunnel || search_result.scan_turn || exploration_lidar_fov.has_value()) {
    // 隧道内（含洞口锁定段与整车退洞段）以及盲区原地扫描段严格采用 BODY_YAW_FOLLOW；
    // 无图探索段默认先置 BODY_YAW_FOLLOW（面向雷达视野），开阔安全区若小陀螺通过门禁则升为 SPIN。
    yaw_authority = ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
  }
  if (spin_policy_.enabled && !force_body_yaw_follow_ && !robot_in_or_arming_tunnel &&
    !search_result.scan_turn)
  {
    ReferenceTrajectory spun = reference;
    SpinYawParams spin_params = spin_yaw_params_;
    spin_params.initial_rate = replan_start.yaw_rate;
    spin_params.stop_before_narrow_m = spin_policy_.align_lookahead_m;
    bool spin_ok = applySpinYaw(
      spun, spin_params,
      forward_tunnels.empty() ? nullptr : &forward_tunnels,
      &tunnel_yaw_params_);
    const bool attempted = spin_ok;
    FootprintSafetyResult spun_safety;
    if (spin_ok) {
      applyWheelSpeedTimeScaling(spun, wheel_speed_time_scaling_params_);
      annotateClearance(spun, *map_snapshot);
      spun_safety = safety_checker_.check(spun, planning_grid, clearance_esdf.get());
      spin_ok = spun.valid() && spun_safety.safe &&
        forwardTunnelEntriesAligned(spun, forward_tunnels, tunnel_yaw_params_);
    }
    if (spin_ok) {
      reference = std::move(spun);
      safety = std::move(spun_safety);
      quality = quality_evaluator_.evaluate(
        reference, clearance_esdf.get(), footprintSamples(planning_grid),
        selected_trace.segment_durations);
      yaw_authority = ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_SPIN;
    } else if (attempted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【小陀螺回退】自旋 yaw 未通过足迹门禁或序列无效，沿用规划 yaw。");
    }
  }
  // Escape-prefix geometry/depth is diagnostic, not an execution permission.
  // All final references, including recovery and debug candidates, obey the
  // same zero-collision contract. Keep the candidate machinery for diagnosis.
  if (!safety.safe) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【最终提交拒绝】参考仍有 %zu 个足迹冲突；脱困前缀不豁免，保持急停。",
      safety.collisions.size());
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  nav_msgs::msg::Path control_reference;
  geometry_msgs::msg::TransformStamped control_from_grid;
  control_from_grid.header.frame_id.clear();
  if (!transformPathToGlobal(toPath(reference), control_reference, &control_from_grid)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考发布失败】控制坐标系转换失败，未发布 MINCO 参考。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REFERENCE_TF,
      map_snapshot->generation);
    return;
  }
  const double total_plan_wall_ms = elapsedWallMs();
  if (plan_hard_wall_budget_ms_ > 0.0 && total_plan_wall_ms > plan_hard_wall_budget_ms_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划墙钟超预算】goal=%llu total_wall_ms=%.2f > hard_budget=%.2f，按契约拒绝提交。",
      static_cast<unsigned long long>(goal_id), total_plan_wall_ms, plan_hard_wall_budget_ms_);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  PlannerCommitTelemetry telemetry;
  telemetry.grid_topic = planning_snapshot_topic_.empty() ? grid_topic_ : planning_snapshot_topic_;
  telemetry.validation_frame = planning_grid.header.frame_id.empty() ?
    global_frame_ : planning_grid.header.frame_id;
  telemetry.gate_collision_count =
    static_cast<std::uint32_t>(safety.collisions.size());
  telemetry.gate_collision_indices = formatCollisionIndices(safety.collisions);
  telemetry.occupancy_digest = planning_digest::occupancyDigest(planning_grid);
  telemetry.content_digest = planning_digest::pathContentDigest(control_reference);
  telemetry.reference_point_count =
    static_cast<std::uint32_t>(reference.points.size());
  telemetry.is_partial_subgoal = search_result.partial;
  telemetry.has_spliced_tail = has_spliced_tail;
  telemetry.search_wall_ms = static_cast<float>(search_wall_ms);
  telemetry.minco_wall_ms = static_cast<float>(minco_wall_ms);
  telemetry.total_plan_wall_ms = static_cast<float>(total_plan_wall_ms);
  if (control_reference.poses.size() == reference.points.size()) {
    double rot_cos = 1.0;
    double rot_sin = 0.0;
    if (!control_from_grid.header.frame_id.empty()) {
      tf2::Transform tf_ctrl_from_grid;
      tf2::fromMsg(control_from_grid.transform, tf_ctrl_from_grid);
      const auto & basis = tf_ctrl_from_grid.getBasis();
      const double delta_yaw = std::atan2(basis[1][0], basis[0][0]);
      rot_cos = std::cos(delta_yaw);
      rot_sin = std::sin(delta_yaw);
    }
    const std::size_t n_pts = reference.points.size();
    telemetry.reference_vx.reserve(n_pts);
    telemetry.reference_vy.reserve(n_pts);
    telemetry.reference_ax.reserve(n_pts);
    telemetry.reference_ay.reserve(n_pts);
    telemetry.reference_yaw_rate.reserve(n_pts);
    for (const auto & pt : reference.points) {
      telemetry.reference_vx.push_back(rot_cos * pt.vx - rot_sin * pt.vy);
      telemetry.reference_vy.push_back(rot_sin * pt.vx + rot_cos * pt.vy);
      telemetry.reference_ax.push_back(rot_cos * pt.ax - rot_sin * pt.ay);
      telemetry.reference_ay.push_back(rot_sin * pt.ax + rot_cos * pt.ay);
      telemetry.reference_yaw_rate.push_back(pt.yaw_rate);
    }
  }
  {
    std::lock_guard<std::mutex> stats_lock(plan_wall_stats_.mutex);
    constexpr std::size_t kMaxSamples = 2048U;
    ++plan_wall_stats_.calls;
    if (minco_wall_ms > minco_stage_budget_ms_) {
      ++plan_wall_stats_.minco_over_budget;
    }
    if (total_plan_wall_ms > plan_wall_budget_ms_) {
      ++plan_wall_stats_.plan_over_budget;
    }
    if (plan_wall_stats_.minco_samples_ms.size() < kMaxSamples) {
      plan_wall_stats_.minco_samples_ms.push_back(minco_wall_ms);
      plan_wall_stats_.total_samples_ms.push_back(total_plan_wall_ms);
    } else {
      const std::size_t idx = plan_wall_stats_.ring_cursor % kMaxSamples;
      plan_wall_stats_.minco_samples_ms[idx] = minco_wall_ms;
      plan_wall_stats_.total_samples_ms[idx] = total_plan_wall_ms;
      ++plan_wall_stats_.ring_cursor;
    }
    const double minco_p50 = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 0.50);
    const double minco_p95 = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 0.95);
    const double minco_max = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 1.00);
    const double plan_p50 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.50);
    const double plan_p95 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.95);
    const double plan_p99 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.99);
    const double plan_max = empiricalPercentile(plan_wall_stats_.total_samples_ms, 1.00);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TRACE plan_wall_stats calls=%llu search_ms=%.2f minco_ms=%.2f total_ms=%.2f "
      "minco_p50_ms=%.2f minco_p95_ms=%.2f minco_max_ms=%.2f "
      "plan_p50_ms=%.2f plan_p95_ms=%.2f plan_p99_ms=%.2f plan_max_ms=%.2f "
      "minco_over_budget=%llu plan_over_budget=%llu spliced=%d partial=%d",
      static_cast<unsigned long long>(plan_wall_stats_.calls),
      search_wall_ms, minco_wall_ms, total_plan_wall_ms,
      minco_p50, minco_p95, minco_max,
      plan_p50, plan_p95, plan_p99, plan_max,
      static_cast<unsigned long long>(plan_wall_stats_.minco_over_budget),
      static_cast<unsigned long long>(plan_wall_stats_.plan_over_budget),
      has_spliced_tail ? 1 : 0, search_result.partial ? 1 : 0);
  }
  if (!publishReferenceIfCurrent(map_snapshot, map_health_epoch,
                                 control_reference, reference, goal_id, localization_epoch,
                                 plan_request_sequence, map_publication_sequence, yaw_authority,
                                 report_status, telemetry,
                                 control_from_grid.header.frame_id.empty() ?
                                 nullptr : &control_from_grid,
                                 replan_start.start_delay_sec,
                                 keep_active_reference,
                                 search_result.scan_turn)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考提交丢弃】generation=%llu 因规划地图变化或过期而丢弃，等待重规划。",
      static_cast<unsigned long long>(map_snapshot->generation));
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
      map_snapshot->generation);
    return;
  }
  mapless_consecutive_rejects_ = 0;
  if (committed_for_start) {
    // 只有无停车重规划才有这一行（不节流，每次建议重规划一行）。source=committed 表示新起点
    // 取自旧参考，measured 表示退回实测，reason 说明原因。start_gap 是新起点到请求时实测
    // 位姿的距离，约等于 车速*plan_lead；拿它和提交时刻车的实际位置比，就是新参考采用瞬间
    // 的起点间隙。plan_ms 为进入 planGoal 到提交的节点时钟耗时，plan_wall_ms 为同段墙钟耗时。
    const double plan_ms = (now() - plan_begin_time).seconds() * 1000.0;
    const double plan_wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - plan_wall_begin).count();
    const double start_gap = std::hypot(
      start.pose.position.x - measured_start_x, start.pose.position.y - measured_start_y);
    RCLCPP_INFO(
      get_logger(),
      "【重规划起点】目标=%llu 请求=%llu source=%s reason=%s frame_ok=%d elapsed=%.3f "
      "progress=%.3f lag=%.3f t_ref=%.3f origin_delay=%.3f deviation=%.3f plan_lead=%.3f "
      "deviation_max=%.3f measured=(%.3f, %.3f) start=(%.3f, %.3f, yaw=%.3f) start_gap=%.3f "
      "v=(%.3f, %.3f) a=(%.3f, %.3f) plan_ms=%.1f plan_wall_ms=%.1f",
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(plan_request_sequence),
      replan_start.source == ReplanStartSource::kCommitted ? "committed" : "measured",
      replanStartReasonName(replan_start.reason), replan_frame_matches ? 1 : 0,
      replan_elapsed_sec, replan_start.progress, replan_elapsed_sec - replan_start.progress,
      replan_start.t_ref, replan_start.start_delay_sec, replan_start.deviation,
      replan_start_params_.plan_lead_sec, replan_start_params_.deviation_max_m,
      measured_start_x, measured_start_y, start.pose.position.x, start.pose.position.y,
      tf2::getYaw(start.pose.orientation), start_gap, replan_start.vx, replan_start.vy,
      replan_start.ax, replan_start.ay, plan_ms, plan_wall_ms);
  }
  if (marker_pub_ && marker_pub_->get_subscription_count() > 0) {
    // Marker publication follows the immutable-snapshot commit, so the red
    // curve is exactly the safety-validated reference handed to MPC.
    marker_pub_->publish(visualizer_.buildMarkers(
      selected_trace.preprocessed_guide, reference, safety));
  }

  std::ostringstream durations;
  durations.setf(std::ios::fixed);
  durations.precision(3);
  for (std::size_t index = 0; index < quality.segment_durations.size(); ++index) {
    if (index > 0U) {
      durations << ',';
    }
    durations << quality.segment_durations[index];
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), log_throttle_ms_,
    "【规划完成】planned generation=%llu snapshot_publication=%llu "
    "raw_points=%zu preprocessed_points=%zu "
    "esdf_refined_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu "
    "footprint_collisions=%zu swept_collisions=%zu "
    "collision_indices=%s occupancy_digest=%s content_digest=%s grid_topic=%s validation_frame=%s "
    "fallback=%d local_repair=%d optimization_failure_reason=%s escape_prefix_end=%zu "
    "expanded=%d yaw_authority=%u center_clearance=%.3f footprint_clearance=%.3f "
    "length_ratio=%.3f lateral=%.3f curvature_max=%.3f curvature_p95=%.3f turn=%.3f "
    "curvature_tv=%.3f curvature_sign_changes=%zu local_scaled=%d uniform_scaled=%d "
    "peak_v=%.3f peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f seed_scale=%.2f "
    "joint_pieces=%d joint_iterations=%d joint_evaluations=%d joint_termination=%s "
    "joint_cost=%.3f->%.3f joint_wall_ms=%.3f joint_satisfied=%d joint_center_violation=%.3f "
    "joint_footprint_violation=%.3f joint_guide_excess=%.3f joint_v_excess=%.3f "
    "joint_a_excess=%.3f "
    "joint_lat_excess=%.3f segment_durations=[%s]",
    static_cast<unsigned long long>(map_snapshot->generation),
    static_cast<unsigned long long>(map_publication_sequence), search_result.path.poses.size(),
    selected_trace.preprocessed_guide.poses.size(), selected_trace.esdf_refined_guide.poses.size(),
    reference.points.size(), reference.totalLength(), reference.totalTime(), safety.collisions.size(),
    quality.footprint_collision_count, quality.swept_collision_count,
    telemetry.gate_collision_indices.c_str(),
    telemetry.occupancy_digest.c_str(), telemetry.content_digest.c_str(), telemetry.grid_topic.c_str(),
    telemetry.validation_frame.c_str(),
    esdf_fallback_used ? 1 : 0, local_repair_used ? 1 : 0,
    selected_trace.failure_reason.empty() ? "none" : selected_trace.failure_reason.c_str(),
    escape_decision.prefix_end,
    search_result.expanded_nodes, static_cast<unsigned int>(yaw_authority),
    quality.minimum_center_clearance, quality.minimum_footprint_clearance, quality.length_ratio,
    quality.max_lateral_deviation, quality.max_geometric_curvature,
    quality.p95_geometric_curvature, quality.total_turning_angle,
    quality.curvature_total_variation, quality.curvature_sign_changes,
    selected_trace.local_time_scaled ? 1 : 0, selected_trace.uniform_time_scaled ? 1 : 0,
    quality.peak_velocity, quality.peak_acceleration, quality.peak_jerk,
    selected_trace.solver_wall_time_ms, selected_trace.seed_scale,
    selected_trace.joint_piece_count,
    selected_trace.joint_iterations, selected_trace.joint_evaluations,
    selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
    selected_trace.joint_initial_cost, selected_trace.joint_final_cost,
    selected_trace.joint_wall_time_ms, selected_trace.joint_constraints_satisfied ? 1 : 0,
    selected_trace.joint_max_center_violation, selected_trace.joint_max_footprint_violation,
    selected_trace.joint_max_guide_excess, selected_trace.joint_max_velocity_excess,
    selected_trace.joint_max_acceleration_excess,
    selected_trace.joint_max_lateral_excess, durations.str().c_str());
}

}  // namespace minco_planner
