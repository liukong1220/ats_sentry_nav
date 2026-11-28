// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

void MincoPlannerNode::planGoal(
    const geometry_msgs::msg::PoseStamped &input_goal, std::uint64_t goal_id,
    std::uint64_t localization_epoch, std::uint64_t plan_request_sequence,
    std::uint64_t map_publication_sequence,
    bool report_status,
    const geometry_msgs::msg::PoseStamped * frozen_start,
    const InitialKinematicState * initial_state, double position_tolerance,
    const GoalUuid & goal_uuid, bool preserve_active_reference) {
  const auto plan_wall_begin = std::chrono::steady_clock::now();
  const rclcpp::Time plan_begin_time = now();
  bool keep_active_reference = false;
  // 无停车重规划时拷一份正在执行的已提交参考，用来取新参考的起点（附录 A.3）。
  std::optional<ActiveSafetyReference> committed_for_start;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    // 由【建议重规划】或目标管理器连续终点修正（preserve_active_reference）触发的同目标
    // 非停车替换请求：旧参考仍在执行，保留它的运行期复检守护与 plan_safe，直到新参考提交
    // （publishReferenceIfCurrent 替换）或本次规划以 STATE_ACCEPTED 报告失败。
    const bool advised_match = replan_advised_reference_stamp_ &&
      active_safety_reference_ &&
      active_safety_reference_->trajectory.header.stamp == *replan_advised_reference_stamp_;
    keep_active_reference = active_safety_reference_ &&
      safety_state_.plan_safe && active_safety_reference_->goal_id == goal_id &&
      active_safety_reference_->localization_epoch == localization_epoch &&
      active_safety_reference_->plan_request_sequence < plan_request_sequence &&
      (preserve_active_reference || advised_match);
    if (keep_active_reference) {
      committed_for_start = active_safety_reference_;
    } else {
      active_safety_reference_.reset();
    }
  }
  if (!keep_active_reference) {
    setPlanSafe(false);
  }
  const auto fail = [this, goal_id, localization_epoch, plan_request_sequence,
                     map_publication_sequence, report_status, keep_active_reference](
                        std::uint8_t failure_reason, std::uint64_t generation) {
    if (keep_active_reference) {
      // 无停车重规划失败：旧参考仍在执行且受 10 Hz 运行期复检守护，不撤销它。失败以
      // STATE_ACCEPTED 发布（MPC/arbiter 忽略，不停车），GoalManager 清除替换标记；下一次
      // 地图变化若仍远端冲突会再次建议重规划。旧参考近端失效时运行期复检照常急停。
      bool still_guarded = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        still_guarded = active_safety_reference_.has_value() && safety_state_.plan_safe;
        replan_advised_reference_stamp_.reset();
        if (still_guarded && planner_status_pub_ && report_status) {
          ats_navigation_interfaces::msg::PlannerStatus status;
          status.header.stamp = now();
          status.header.frame_id = global_frame_;
          status.goal_id = goal_id;
          status.localization_epoch = localization_epoch;
          status.plan_request_sequence = plan_request_sequence;
          status.map_generation = generation != 0 ? generation : next_map_generation_;
          status.map_publication_sequence = map_publication_sequence;
          status.state = ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED;
          status.failure_reason = failure_reason;
          planner_status_pub_->publish(status);
        }
      }
      if (still_guarded) {
        if (global_route_ && global_route_->optimistic &&
          (failure_reason ==
            ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT ||
           failure_reason ==
            ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER))
        {
          ++mapless_consecutive_rejects_;
        }
        RCLCPP_WARN(
          get_logger(),
          "【无停车重规划失败】目标=%llu 请求=%llu 原因=%u，继续执行受运行期复检守护的旧参考。",
          static_cast<unsigned long long>(goal_id),
          static_cast<unsigned long long>(plan_request_sequence),
          static_cast<unsigned>(failure_reason));
        return;
      }
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      active_safety_reference_.reset();
    }
    setPlanSafe(false);
    if (report_status) {
      publishPlannerStatus(
          goal_id, localization_epoch, plan_request_sequence, generation,
          map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED, failure_reason);
    }
  };
  std::shared_ptr<const PlanningMapSnapshot> map_snapshot;
  std::uint64_t map_health_epoch = 0;
  bool map_ready = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_snapshot = latest_map_snapshot_;
    map_health_epoch = map_health_epoch_;
    map_ready = map_snapshot && safety_state_.mapSnapshotUsable(map_snapshot->generation);
  }
  if (!map_ready || !map_snapshot ||
    (localization_epoch != 0 && map_snapshot->localization_epoch != 0 &&
    map_snapshot->localization_epoch != localization_epoch))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【等待地图】尚未收到可通行栅格，规划保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY, 0);
    return;
  }
  const auto & planning_grid = map_snapshot->grid;
  const auto & clearance_esdf = map_snapshot->clearance_esdf;
  InitialKinematicState frozen_seed;
  const InitialKinematicState * seed_ptr = nullptr;
  if (initial_state && initial_state->valid) {
    frozen_seed = *initial_state;
    seed_ptr = &frozen_seed;
  }

  geometry_msgs::msg::PoseStamped start;
  if (!resolveStartPose(planning_grid, frozen_start, start)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】无法获取起点位姿，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_START_TF,
      map_snapshot->generation);
    return;
  }

  // 无停车重规划的起点（调研报告附录 A.3）。请求里冻结的实测位姿在规划耗时内已落到车后面
  // （domain 111 目标 69965bd0 第二条参考的起点落后车 0.26 m，车速 0.96 m/s），所以新参考
  // 改从旧参考上"车的推进位置 + plan_lead"处取 p/v/a/yaw。车离旧参考 >= deviation_max、
  // 定位 epoch 不同、旧参考已不在执行（含急停撤销）时仍用实测，行为与此前一致。
  ReplanStart replan_start;
  bool replan_frame_matches = true;
  double replan_elapsed_sec = 0.0;
  const double measured_start_x = start.pose.position.x;
  const double measured_start_y = start.pose.position.y;
  if (committed_for_start) {
    MeasuredStart measured;
    measured.x = measured_start_x;
    measured.y = measured_start_y;
    measured.yaw = tf2::getYaw(start.pose.orientation);
    if (seed_ptr) {
      measured.vx = seed_ptr->velocity.x();
      measured.vy = seed_ptr->velocity.y();
    }
    // 旧参考按当前 map->odom 修正后再和实测比较，两者才在同一栅格坐标系里。
    const ReferenceTrajectory aligned = alignedToCurrentTf(*committed_for_start);
    replan_frame_matches = aligned.header.frame_id == planning_grid.header.frame_id;
    replan_elapsed_sec = std::max(0.0, (now() - rclcpp::Time(aligned.header.stamp)).seconds());
    replan_start = selectReplanStart(
      replan_frame_matches ? &aligned : nullptr, committed_for_start->localization_epoch,
      localization_epoch, replan_elapsed_sec, measured, replan_start_params_);
    if (replan_start.source == ReplanStartSource::kCommitted) {
      start.pose.position.x = replan_start.x;
      start.pose.position.y = replan_start.y;
      start.pose.orientation.x = 0.0;
      start.pose.orientation.y = 0.0;
      start.pose.orientation.z = std::sin(replan_start.yaw * 0.5);
      start.pose.orientation.w = std::cos(replan_start.yaw * 0.5);
      frozen_seed.valid = true;
      frozen_seed.velocity = Eigen::Vector2d(replan_start.vx, replan_start.vy);
      frozen_seed.acceleration = Eigen::Vector2d(replan_start.ax, replan_start.ay);
      seed_ptr = &frozen_seed;
    }
  }
  // Cold-start plans (no active trajectory being preserved) run while emergency_stop
  // holds zero command. Measured Point-LIO twist at standstill carries 0.03-0.20 m/s
  // IMU jitter; seeding that sideways noise as an exact MINCO head_state velocity
  // launches narrow-tunnel starts laterally across floor seams.
  if (!keep_active_reference && seed_ptr != nullptr &&
    seed_ptr->velocity.norm() <= 0.25 && seed_ptr->acceleration.norm() <= 1e-6)
  {
    frozen_seed.valid = false;
    frozen_seed.velocity.setZero();
    frozen_seed.acceleration.setZero();
    seed_ptr = nullptr;
  }

  geometry_msgs::msg::PoseStamped goal = input_goal;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id =
      planning_grid.header.frame_id.empty() ? global_frame_ : planning_grid.header.frame_id;
  }
  geometry_msgs::msg::PoseStamped goal_in_grid;
  if (!transformGoalToGrid(planning_grid, goal, goal_in_grid)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】目标位姿转换失败，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_GOAL_TF,
      map_snapshot->generation);
    return;
  }
  goal = goal_in_grid;

  // 目标位姿准入。action 的成功判据是一个容差域，而不是一个点：目标点足迹与栅格墙面
  // 重叠几毫米时，容差域内往往仍有大量完全可行的位姿。此前规划器只认那个点，于是提交门
  // 正确拒绝每一条轨迹，goal manager 反复重试直到预算耗尽（domain 175 目标 8 实测：
  // 车已停在目标 0.16 m 处，只差一个可行的终点标注）。这里在容差域内挑一个可行终点，
  // 候选过的是与提交门完全相同的足迹判定，偏移严格小于容差，所以 SUCCEEDED 依然由原目标
  // 的判据给出，不是靠放宽判据换来的。找不到可行位姿时保持原目标不变，走既有失败路径。
  // 终点净空放宽档只有在目标位姿确实过了矩形足迹门禁时才允许启用;准入关闭时
  // 保持 false,行为与今天完全一致。
  bool goal_pose_footprint_verified = false;
  auto admission_params = goal_pose_admission_params_;
  if (position_tolerance > 0.0) {
    // Per-request copy: a waypoint override must not alter later/final goals.
    admission_params.position_tolerance_m = position_tolerance;
  }
  if (admission_params.enabled) {
    const double commanded_goal_yaw = tf2::getYaw(goal.pose.orientation);
    const GoalPoseAdmissionResult admission = admitGoalPose(
      goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
      planning_grid, footprint_params_, admission_params);
    if (!admission.feasible) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "【目标准入】成功容差内没有可行位姿：目标=(%.3f, %.3f, yaw=%.3f) "
        "候选数=%zu；保留原目标。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.candidates_checked);
    }
    goal_pose_footprint_verified = admission.feasible;
    if (admission.feasible && admission.relocated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【目标准入】在成功容差内找到可行位姿：原目标=(%.3f, %.3f, yaw=%.3f) "
        "采用=(%.3f, %.3f, yaw=%.3f) 位置偏差=%.3f m 航向偏差=%.3f rad "
        "额外净空=%d 候选数=%zu。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.x, admission.y, admission.yaw, admission.position_deviation_m,
        admission.yaw_deviation_rad, admission.used_preferred_margin ? 1 : 0,
        admission.candidates_checked);
      goal.pose.position.x = admission.x;
      goal.pose.position.y = admission.y;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = std::sin(admission.yaw * 0.5);
      goal.pose.orientation.w = std::cos(admission.yaw * 0.5);
    }
  }

  // 起点来自 TF、终点来自 goal_topic，因此此分支不依赖 Smac 的路径几何。
  double used_clearance = preferred_search_clearance_;
  nav_msgs::msg::Path global_guide_path;
  std::optional<geometry_msgs::msg::PoseStamped> mapless_local_anchor;
  const auto search_wall_begin = std::chrono::steady_clock::now();
  const GridSearchResult search_result = planRoute(
    map_snapshot, start, goal, goal_uuid, goal_id, localization_epoch,
    goal_pose_footprint_verified, used_clearance, &global_guide_path, &mapless_local_anchor);
  const double search_wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_wall_begin).count();
  if (!search_result.success && !search_result.partial) {
    publishRollingAnchor(std::nullopt, planning_grid.header.frame_id, now(), "");
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划失败】%s 失败（failed: %s expanded=%d clearance=%.3f m） "
      "goal=%llu localization_epoch=%llu plan_request_sequence=%llu",
      search_algorithm_.c_str(), search_result.reason.c_str(),
      search_result.expanded_nodes, used_clearance,
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(localization_epoch),
      static_cast<unsigned long long>(plan_request_sequence));
    const std::uint8_t failure_reason =
      classifyGraphSearchFailure(search_result.reason, search_result.expanded_nodes);
    fail(failure_reason, map_snapshot->generation);
    return;
  }

  // The raw JPS route, guide stages and final candidate all come from this
  // immutable snapshot.  The guide shortcut delegates collision semantics to
  // the exact oriented footprint/swept checker used by the final gate.
  // These are diagnostic-only products. They deliberately publish before the
  // candidate gate so a fail-closed rejection can still be attributed to a
  // geometry, ESDF, time-allocation, or dynamic-limit stage.
  if (raw_path_pub_ && raw_path_pub_->get_subscription_count() > 0) {
    raw_path_pub_->publish(
      !global_guide_path.poses.empty() ? global_guide_path : search_result.path);
  }
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = (search_result.partial && !search_result.path.poses.empty()) ?
    tf2::getYaw(search_result.path.poses.back().pose.orientation) :
    tf2::getYaw(goal.pose.orientation);
  const std::optional<double> exploration_lidar_fov =
    (search_result.partial && !search_result.scan_turn)
      ? resolveLidarFovYawOffsetRad()
      : std::nullopt;
  std::vector<TunnelSegment> guide_tunnels;
  ReferenceTrajectory traced_guide;
  if (tunnel_yaw_params_.enabled && clearance_esdf) {
    traced_guide = traceGuide(search_result.path);
    annotatePositionClearance(traced_guide, *map_snapshot);
    guide_tunnels = detectTunnelSegments(traced_guide, *clearance_esdf, tunnel_yaw_params_);
    resolveTerminalTunnelGoalAlignment(
      guide_tunnels, traced_guide, goal_yaw, !search_result.partial, tunnel_yaw_params_);
    for (const auto & tunnel : guide_tunnels) {
      if (tunnel.forward_alignment) {
        const auto & entry = traced_guide.points[tunnel.begin];
        RCLCPP_INFO(
          get_logger(), "TRACE guide_tunnel publication=%llu width_m=%.3f "
          "entry=(%.3f,%.3f) yaw=%.3f exit=(%.3f,%.3f)",
          static_cast<unsigned long long>(map_snapshot->publication_sequence),
          tunnel.width_estimate, entry.x, entry.y, tunnel.axis_yaw.front(),
          traced_guide.points[tunnel.end].x, traced_guide.points[tunnel.end].y);
      }
    }
  }

  double minco_wall_ms = 0.0;
  const auto elapsedWallMs = [&plan_wall_begin]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_wall_begin).count();
    };
  const auto rollingBudgetExceeded = [&]() {
      return keep_active_reference && rolling_replan_wall_budget_ms_ > 0.0 &&
             elapsedWallMs() >= rolling_replan_wall_budget_ms_;
    };
  const auto runOptimize = [&](
    const nav_msgs::msg::Path & guide_path,
    const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf_ptr,
    const ReferenceTrajectory * yaw_ref,
    MincoOptimizationTrace * out_trace,
    const TerminalKinematicState * tail_ptr = nullptr,
    bool allow_shortcut = true) {
      const auto t0 = std::chrono::steady_clock::now();
      ReferenceTrajectory out = optimizer_.optimize(
        guide_path, esdf_ptr, yaw_ref, seed_ptr, &planning_grid, &safety_checker_,
        out_trace, tail_ptr, map_snapshot->blocked_squared_cells.get(), allow_shortcut);
      minco_wall_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return out;
    };
  const auto minTrajectoryClearance = [](const ReferenceTrajectory & traj) {
      double min_c = std::numeric_limits<double>::infinity();
      for (const auto & pt : traj.points) {
        if (std::isfinite(pt.clearance)) {
          min_c = std::min(min_c, pt.clearance);
        }
      }
      return std::isfinite(min_c) ? min_c : -std::numeric_limits<double>::infinity();
    };

  bool has_spliced_tail = false;
  bool used_old_trajectory_splice = false;
  nav_msgs::msg::Path active_guide_path = search_result.path;
  double active_goal_yaw = goal_yaw;
  bool active_is_final_goal = !search_result.partial || search_result.scan_turn;
  MincoOptimizationTrace selected_trace;
  ReferenceTrajectory center_reference;

  // O2 真局部滚动地平线 + C2 复用拼接：
  // 当路径总长 > rolling_horizon_length_m + rolling_min_tail_length_m 时：
  //   (A) 若已提交安全参考直达同一终点且后段仍安全，优先 C2 拼接至旧参考后段；
  //   (B) 否则在隧道外且足迹可停的滚动地平线处截断为局部窗口（约 8~12 m，零速确定性停车），
  //       仅对局部窗口跑完整 ESDF/足迹联合优化（~8-12 段，< 15 ms），同时保留全局引导供绕障
  //       检测，由 GoalManager 在进入制动尾段前无停车滚动刷新下一窗口。
  const double total_guide_length = pathArcLength(search_result.path);
  if (rolling_horizon_enabled_ &&
    total_guide_length > rolling_horizon_length_m_ + rolling_min_tail_length_m_)
  {
    auto avoidTunnels = [&](double split_s) {
        for (const auto & tunnel : guide_tunnels) {
          if (tunnel.end < traced_guide.points.size()) {
            const double entry_s = traced_guide.points[tunnel.begin].s - 1.5;
            const double exit_s = traced_guide.points[tunnel.end].s + 1.5;
            if (split_s >= entry_s && split_s <= exit_s) {
              if (exit_s + rolling_min_tail_length_m_ < total_guide_length &&
                exit_s <= rolling_horizon_length_m_ + 4.0)
              {
                split_s = exit_s;
              } else if (entry_s >= 4.0) {
                split_s = entry_s;
              } else if (exit_s + rolling_min_tail_length_m_ < total_guide_length) {
                split_s = exit_s;
              }
            }
          }
        }
        return split_s;
      };
    const double base_split_s = avoidTunnels(rolling_horizon_length_m_);
    SplitGuideResult split;
    const double offsets[] = {0.0, -0.5, 0.5, -1.0, 1.0, -1.5, 1.5};
    for (const double delta : offsets) {
      const double try_s = avoidTunnels(base_split_s + delta);
      if (try_s < 3.5 || try_s + rolling_min_tail_length_m_ >= total_guide_length) {
        continue;
      }
      SplitGuideResult cand = splitPathAtArcLength(
        search_result.path, try_s, rolling_min_tail_length_m_);
      if (!cand.valid) {
        continue;
      }
      const double cand_yaw = std::atan2(cand.junction_tangent.y(), cand.junction_tangent.x());
      if (goalPoseFootprintFree(
          cand.junction_pos.x(), cand.junction_pos.y(), cand_yaw,
          planning_grid, footprint_params_))
      {
        split = std::move(cand);
        break;
      }
    }
    if (split.valid) {
      // (A) 优先尝试直接 C2 拼接到仍在执行且直达最终目标的安全旧参考轨迹。
      if (committed_for_start && replan_frame_matches &&
        committed_for_start->trajectory.valid() &&
        !search_result.path.poses.empty())
      {
        const auto & old_traj = committed_for_start->trajectory;
        const auto & goal_pt = search_result.path.poses.back().pose.position;
        const double goal_match_dist = std::hypot(
          old_traj.points.back().x - goal_pt.x, old_traj.points.back().y - goal_pt.y);
        const double t_rejoin = projectedReferenceTime(
          old_traj, split.junction_pos.x(), split.junction_pos.y(), old_traj.totalTime());
        const double min_rejoin_t = std::isfinite(replan_start.t_ref) ?
          replan_start.t_ref + 0.35 : 0.35;
        if (goal_match_dist <= 0.05 && t_rejoin > min_rejoin_t &&
          t_rejoin + 0.35 < old_traj.totalTime())
        {
          const ReferencePoint ahead_j = interpolateReferencePoint(old_traj, t_rejoin);
          if (std::hypot(ahead_j.x - split.junction_pos.x(), ahead_j.y - split.junction_pos.y()) <=
            0.15)
          {
            const ReferenceTrajectory old_tail = remainingReferenceWindow(
              old_traj, t_rejoin, std::numeric_limits<double>::infinity());
            if (old_tail.points.size() >= 2U &&
              safety_checker_.check(old_tail, planning_grid, clearance_esdf.get()).safe)
            {
              nav_msgs::msg::Path exact_prefix = split.prefix;
              exact_prefix.poses.back().pose.position.x = ahead_j.x;
              exact_prefix.poses.back().pose.position.y = ahead_j.y;
              TerminalKinematicState exact_tail;
              exact_tail.valid = true;
              exact_tail.velocity = Eigen::Vector2d(ahead_j.vx, ahead_j.vy);
              exact_tail.acceleration = Eigen::Vector2d(ahead_j.ax, ahead_j.ay);
              exact_tail.exact_splice = true;
              MincoOptimizationTrace splice_trace;
              ReferenceTrajectory local_prefix = runOptimize(
                exact_prefix, clearance_esdf.get(), nullptr, &splice_trace, &exact_tail);
              if (local_prefix.valid() && std::abs(splice_trace.seed_scale - 1.0) <= 1e-6) {
                ReferenceTrajectory spliced = spliceReferenceTrajectories(
                  local_prefix, old_tail, local_prefix.totalTime() - t_rejoin);
                if (spliced.valid()) {
                  spliced.header.stamp = now();
                  planYaw(
                    spliced, *map_snapshot, start_yaw, goal_yaw, !search_result.partial,
                    exploration_lidar_fov);
                  if (safety_checker_.check(spliced, planning_grid, clearance_esdf.get()).safe) {
                    center_reference = std::move(spliced);
                    selected_trace = std::move(splice_trace);
                    has_spliced_tail = true;
                    used_old_trajectory_splice = true;
                  }
                }
              }
            }
          }
        }
      }
      // (B) 真局部滚动窗口：仅对近端局部窗口跑完整 ESDF/足迹优化（末端安全零速停车），
      // 保留完整全局引导用于远端绕障，由 GoalManager 在进入制动区前无停车滚动推进。
      if (!center_reference.valid()) {
        active_guide_path = std::move(split.prefix);
        active_goal_yaw = std::atan2(split.junction_tangent.y(), split.junction_tangent.x());
        active_is_final_goal = false;
        has_spliced_tail = true;
      }
      if (!mapless_local_anchor.has_value()) {
        geometry_msgs::msg::PoseStamped split_anchor;
        split_anchor.header = planning_grid.header;
        split_anchor.pose.position.x = split.junction_pos.x();
        split_anchor.pose.position.y = split.junction_pos.y();
        split_anchor.pose.position.z = start.pose.position.z;
        const double syaw = std::atan2(split.junction_tangent.y(), split.junction_tangent.x());
        split_anchor.pose.orientation.z = std::sin(syaw * 0.5);
        split_anchor.pose.orientation.w = std::cos(syaw * 0.5);
        publishRollingAnchor(
          split_anchor, planning_grid.header.frame_id, now(),
          used_old_trajectory_splice ? "SPLICE ANCHOR" : "ROLLING ANCHOR");
      }
    }
  }
  if (mapless_local_anchor.has_value()) {
    publishRollingAnchor(
      mapless_local_anchor, planning_grid.header.frame_id, now(), "LOCAL ANCHOR");
  } else if (!has_spliced_tail) {
    publishRollingAnchor(std::nullopt, planning_grid.header.frame_id, now(), "");
  }

  FootprintSafetyResult center_safety;
  if (center_reference.valid()) {
    center_safety.safe = true;
  } else {
    center_reference = runOptimize(
      active_guide_path, clearance_esdf.get(), nullptr, &selected_trace, nullptr);
    if (!center_reference.valid() && has_spliced_tail && !used_old_trajectory_splice) {
      // 局部窗口求解失败时回退至整条路径一次，保证不因局部裁剪引入额外失败。
      has_spliced_tail = false;
      active_guide_path = search_result.path;
      active_goal_yaw = goal_yaw;
      active_is_final_goal = !search_result.partial;
      center_reference = runOptimize(
        active_guide_path, clearance_esdf.get(), nullptr, &selected_trace, nullptr);
    }
  }
  if (preprocessed_guide_pub_ && preprocessed_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.preprocessed_guide.poses.empty())
  {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && esdf_refined_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.esdf_refined_guide.poses.empty())
  {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!center_reference.valid()) {
    std::ostringstream durations;
    durations.setf(std::ios::fixed);
    durations.precision(3);
    for (std::size_t index = 0; index < selected_trace.segment_durations.size(); ++index) {
      if (index > 0U) {
        durations << ',';
      }
      durations << selected_trace.segment_durations[index];
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】候选被拒绝（candidate rejected generation=%llu snapshot_publication=%llu stage=%s "
      "raw_points=%zu preprocessed_points=%zu esdf_refined_points=%zu peak_v=%.3f "
      "peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f joint_termination=%s joint_iterations=%d "
      "joint_wall_ms=%.3f seed_scale=%.2f segment_durations=[%s]）",
      static_cast<unsigned long long>(map_snapshot->generation),
      static_cast<unsigned long long>(map_publication_sequence),
      selected_trace.failure_reason.empty() ? "unknown" : selected_trace.failure_reason.c_str(),
      search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
      selected_trace.esdf_refined_guide.poses.size(), selected_trace.peak_velocity,
      selected_trace.peak_acceleration, selected_trace.peak_jerk,
      selected_trace.solver_wall_time_ms,
      selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
      selected_trace.joint_iterations, selected_trace.joint_wall_time_ms,
      selected_trace.seed_scale, durations.str().c_str());
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  if (!used_old_trajectory_splice) {
    center_reference.header.stamp = now();
    planYaw(
      center_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
      exploration_lidar_fov, search_result.scan_turn);
    center_safety = safety_checker_.check(
      center_reference, planning_grid, clearance_esdf.get());
  }
  ReferenceTrajectory reference = center_reference;
  FootprintSafetyResult safety = center_safety;
  // 足迹罚项用的是上一条轨迹的 yaw，而 planYaw 会按新轨迹重算 yaw：两者不一致时，
  // 角点可落进薄墙尖（优化器看不到、门禁拒绝）。用刚算出的 yaw 作参考再解，
  // 直到门禁通过或轮数用尽；每轮结果都过同一矩形门禁，不放宽任何判据，
  // 碰撞数变多即停并保留上一轮。
  const auto refineFootprintYaw = [&](
    const nav_msgs::msg::Path & guide, ReferenceTrajectory & candidate,
    FootprintSafetyResult & candidate_safety, MincoOptimizationTrace & candidate_trace) {
      for (int round = 0; !candidate_safety.safe && round < footprint_yaw_refinement_rounds_ &&
        !rollingBudgetExceeded();
        ++round)
      {
        MincoOptimizationTrace refined_trace;
        ReferenceTrajectory refined = runOptimize(
          guide, clearance_esdf.get(), &candidate, &refined_trace, nullptr);
        if (!refined.valid()) {
          return;
        }
        refined.header.stamp = now();
        planYaw(
          refined, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
          exploration_lidar_fov, search_result.scan_turn);
        FootprintSafetyResult refined_safety = safety_checker_.check(
          refined, planning_grid, clearance_esdf.get());
        if (refined_safety.collisions.size() > candidate_safety.collisions.size()) {
          return;
        }
        candidate = std::move(refined);
        candidate_safety = std::move(refined_safety);
        candidate_trace = std::move(refined_trace);
      }
    };
  const double center_min_clearance = minTrajectoryClearance(center_reference);
  const bool need_footprint_refinement = !center_safety.safe ||
    !guide_tunnels.empty() ||
    center_min_clearance < footprint_refinement_skip_clearance_m_;
  if (optimizer_.esdfFootprintOptimizationEnabled() && need_footprint_refinement &&
    !used_old_trajectory_splice && !rollingBudgetExceeded())
  {
    MincoOptimizationTrace footprint_trace;
    ReferenceTrajectory footprint_reference = runOptimize(
      active_guide_path, clearance_esdf.get(), &center_reference, &footprint_trace, nullptr);
    if (footprint_reference.valid()) {
      footprint_reference.header.stamp = now();
      planYaw(
        footprint_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
        exploration_lidar_fov, search_result.scan_turn);
      FootprintSafetyResult footprint_safety = safety_checker_.check(
        footprint_reference, planning_grid, clearance_esdf.get());
      refineFootprintYaw(
        active_guide_path, footprint_reference, footprint_safety, footprint_trace);
      if (footprint_safety.safe || !center_safety.safe) {
        reference = std::move(footprint_reference);
        safety = std::move(footprint_safety);
        selected_trace = std::move(footprint_trace);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【足迹候选回退】RC-ESDF 足迹候选有 %zu 个冲突，改用安全的中心 ESDF 候选。",
          footprint_safety.collisions.size());
      }
    }
  }
  bool esdf_fallback_used = false;
  if (!safety.safe && optimizer_.esdfObstacleOptimizationEnabled() && !rollingBudgetExceeded()) {
    // 外推候选仍碰撞时回到不做 ESDF 位移的 JPS-MINCO（先常规捷径，若仍在窄道弓出擦墙则关闭捷径贴紧中心线）。
    for (const bool allow_sc : {true, false}) {
      if (safety.safe || rollingBudgetExceeded()) {
        break;
      }
      MincoOptimizationTrace fallback_trace;
      ReferenceTrajectory fallback_reference = runOptimize(
        active_guide_path, nullptr, nullptr, &fallback_trace, nullptr, allow_sc);
      if (fallback_reference.valid()) {
        fallback_reference.header.stamp = now();
        planYaw(
          fallback_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
          exploration_lidar_fov, search_result.scan_turn);
        const FootprintSafetyResult fallback_safety = safety_checker_.check(
          fallback_reference, planning_grid, clearance_esdf.get());
        if (fallback_safety.safe) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "【足迹候选回退】RC-ESDF 外层候选有 %zu 个足迹冲突，改用安全的 JPS-MINCO 基线 (shortcut=%d)。",
            safety.collisions.size(), allow_sc ? 1 : 0);
          esdf_fallback_used = true;
          reference = std::move(fallback_reference);
          safety = fallback_safety;
          selected_trace = std::move(fallback_trace);
          break;
        }
      }
    }
  }
  LocalCollisionRepairStats repair_stats;
  // repair() 就地改引导点,所以要先留一份修复前的轨迹:重解 MINCO 失败时
  // 必须能退回原状。以前这条路走不到(严格档恒为空集),现在会真的走到,
  // 而 FAILURE_REPAIR 在 goal manager 里不是瞬时故障,会立刻判死目标。
  // 保证"尝试修复"永远不比"不尝试"更差。
  ReferenceTrajectory pre_repair_reference;
  if (!safety.safe) {
    pre_repair_reference = reference;
  }
  const bool repair_changed = !safety.safe &&
    collision_repair_.repair(reference, safety, planning_grid, &repair_stats);
  if (!safety.safe) {
    // repair() 自身没有 logger。没有这一行时无法区分"没尝试""挑不出候选格"
    // 和"挑出来但位移可忽略",domain 169 的 89 次连续拒绝就完全没有痕迹。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【局部碰撞修复】changed=%d points=%zu strict=%zu fallback=%zu "
      "no_candidate=%zu negligible=%zu endpoint_protected=%zu strict_clearance=%.3f "
      "fallback_clearance=%.3f search_radius=%.3f m.",
      repair_changed ? 1 : 0, repair_stats.collision_points,
      repair_stats.strict_repaired, repair_stats.fallback_repaired,
      repair_stats.no_candidate, repair_stats.negligible_shift,
      repair_stats.endpoint_protected,
      repair_stats.strict_clearance_m, repair_stats.fallback_clearance_m,
      repair_stats.effective_search_radius_m);
  }
  bool local_repair_used = false;
  if (repair_changed) {
    // 局部修复只改变几何引导线，必须重新求 MINCO、yaw 和最终矩形足迹安全性。
    // 关闭几何捷径，避免 PathGeometryPreprocessor 把刚修复的隧道入口绕离点重新拉直。
    const nav_msgs::msg::Path repair_guide = toPath(reference);
    MincoOptimizationTrace repair_trace;
    ReferenceTrajectory repaired = runOptimize(
      repair_guide, clearance_esdf.get(), &reference, &repair_trace, nullptr,
      /*allow_shortcut=*/false);
    if (!repaired.valid()) {
      // 退回修复前轨迹,交给下面既有的 footprint 拒绝路径。那条路径报
      // FAILURE_FOOTPRINT(瞬时,等下一次 snapshot 重规划),而 FAILURE_REPAIR
      // 会让目标直接失败——修复只是一次尝试,失败不该比没尝试更严重。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【局部碰撞修复失败】修复后 MINCO 轨迹无效，保留修复前轨迹并进入既有足迹拒绝路径。");
      reference = std::move(pre_repair_reference);
    } else {
      local_repair_used = true;
      reference = std::move(repaired);
      reference.header.stamp = now();
      planYaw(
        reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
        exploration_lidar_fov, search_result.scan_turn);
      safety = safety_checker_.check(reference, planning_grid, clearance_esdf.get());
      refineFootprintYaw(repair_guide, reference, safety, repair_trace);
      selected_trace = std::move(repair_trace);
    }
    // If ESDF re-optimization fell back into the same corner-clipping local minimum
    // (because joint_obstacle ignores dense_guide), re-solve strictly following the
    // repaired+shouldered guide polyline and verify with the exact same footprint gate.
    if (!safety.safe && !rollingBudgetExceeded()) {
      MincoOptimizationTrace guide_repair_trace;
      ReferenceTrajectory guide_repaired = runOptimize(
        repair_guide, nullptr, nullptr, &guide_repair_trace, nullptr,
        /*allow_shortcut=*/false);
      if (guide_repaired.valid()) {
        guide_repaired.header.stamp = now();
        planYaw(
          guide_repaired, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal,
          exploration_lidar_fov, search_result.scan_turn);
        FootprintSafetyResult guide_safety = safety_checker_.check(
          guide_repaired, planning_grid, clearance_esdf.get());
        if (!guide_safety.safe) {
          refineFootprintYaw(repair_guide, guide_repaired, guide_safety, guide_repair_trace);
        }
        if (guide_safety.safe) {
          local_repair_used = true;
          reference = std::move(guide_repaired);
          safety = std::move(guide_safety);
          selected_trace = std::move(guide_repair_trace);
        }
      }
    }
  }

  applyTerminalYawRelocationAndSidestep(
    map_snapshot, planning_grid, clearance_esdf,
    active_goal_yaw, active_is_final_goal, reference, safety);

  // When a trajectory is collision-free for a substantial prefix (>= 0.45 m on partial subgoals,
  // or >= 0.80 m on full local goals) and only clips the wall/unknown border far downstream
  // (e.g. a narrow-corridor entrance or exit kink), truncate the guide strictly before the first
  // collision and publish it as a partial subgoal (triggering rolling replanning as the robot
  // advances) instead of discarding the entire safe prefix with FAILURE_FOOTPRINT. Still verified
  // by the exact same zero-collision FootprintSafetyChecker.
  const double min_trim_prefix_s =
    (search_result.partial || !active_is_final_goal) ? 0.45 : 0.80;
  const bool tail_trim_budget_ok = !keep_active_reference ||
    rolling_replan_wall_budget_ms_ <= 0.0 ||
    elapsedWallMs() < rolling_replan_wall_budget_ms_ + 20.0;
  if (!safety.safe && !search_result.scan_turn &&
    !safety.collisions.empty() && reference.points.size() >= 2U && tail_trim_budget_ok)
  {
    std::size_t first_col_idx = reference.points.size();
    for (const auto & col : safety.collisions) {
      first_col_idx = std::min(first_col_idx, col.segment_index);
    }
    if (first_col_idx < reference.points.size() &&
      reference.points[first_col_idx].s >= min_trim_prefix_s)
    {
      const double first_col_s = reference.points[first_col_idx].s;
      const double margins[3] = {0.30, 0.55, 0.85};
      bool trimmed_safe = false;
      for (double margin : margins) {
        if (trimmed_safe) {
          break;
        }
        const double cut_s = first_col_s - margin;
        if (cut_s < 0.25) {
          continue;
        }
        nav_msgs::msg::Path short_guide;
        short_guide.header = active_guide_path.header;
        short_guide.poses.push_back(active_guide_path.poses.front());
        double acc_s = 0.0;
        for (std::size_t gi = 1U; gi < active_guide_path.poses.size(); ++gi) {
          const auto & prev_p = active_guide_path.poses[gi - 1U].pose.position;
          const auto & cur_p = active_guide_path.poses[gi].pose.position;
          const double seg = std::hypot(cur_p.x - prev_p.x, cur_p.y - prev_p.y);
          if (acc_s + seg >= cut_s) {
            const double u = seg > 1e-6 ? std::clamp((cut_s - acc_s) / seg, 0.0, 1.0) : 0.0;
            geometry_msgs::msg::PoseStamped end_pose = active_guide_path.poses[gi];
            end_pose.pose.position.x = prev_p.x + u * (cur_p.x - prev_p.x);
            end_pose.pose.position.y = prev_p.y + u * (cur_p.y - prev_p.y);
            short_guide.poses.push_back(end_pose);
            break;
          }
          acc_s += seg;
          short_guide.poses.push_back(active_guide_path.poses[gi]);
        }
        if (short_guide.poses.size() < 2U) {
          continue;
        }
        // Hold the already-verified collision-free yaw at cut_s instead of swinging corners
        // into the unknown frontier border with exploration_lidar_fov.
        double short_goal_yaw = active_goal_yaw;
        for (const auto & pt : reference.points) {
          if (pt.s >= cut_s) {
            short_goal_yaw = pt.yaw;
            break;
          }
        }
        for (int opt_pass = 0; opt_pass < 2; ++opt_pass) {
          const bool use_esdf = (opt_pass == 0);
          MincoOptimizationTrace short_trace;
          ReferenceTrajectory short_ref = runOptimize(
            short_guide, use_esdf ? clearance_esdf.get() : nullptr, nullptr, &short_trace,
            nullptr, /*allow_shortcut=*/use_esdf);
          if (!short_ref.valid()) {
            continue;
          }
          short_ref.header.stamp = now();
          planYaw(
            short_ref, *map_snapshot, start_yaw, short_goal_yaw, false,
            std::nullopt, false);
          const FootprintSafetyResult short_safety = safety_checker_.check(
            short_ref, planning_grid, clearance_esdf.get());
          if (short_safety.safe) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), log_throttle_ms_,
              "【部分子目标尾段裁剪】原首冲突在 s=%.2f m，安全裁剪至 s=%.2f m (esdf=%d) 后零冲突通过。",
              first_col_s, cut_s, use_esdf ? 1 : 0);
            active_guide_path = std::move(short_guide);
            active_goal_yaw = short_goal_yaw;
            active_is_final_goal = false;
            reference = std::move(short_ref);
            safety = short_safety;
            selected_trace = std::move(short_trace);
            trimmed_safe = true;
            break;
          }
        }
      }
    }
  }

  if (preprocessed_guide_pub_ && preprocessed_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.preprocessed_guide.poses.empty())
  {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && esdf_refined_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.esdf_refined_guide.poses.empty())
  {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!reference.valid()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】参考轨迹无效，保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  EscapePrefixDecision escape_decision = safety.safe ?
    EscapePrefixDecision{} :
    evaluateEscapePrefix(reference.points, safety.collisions, escape_prefix_params_);
  if (escape_decision.allowed &&
    !escapePrefixContactAdmitted(
      reference, escape_decision, footprint_params_, planning_grid, escape_prefix_params_))
  {
    // 前缀压进占据格超过深度上界：不是擦边接触，照旧 fail-closed。
    escape_decision.prefix_end = 0U;
    escape_decision.allowed = false;
  }

  const geometry_msgs::msg::PoseStamped local_escape_goal =
    (!active_is_final_goal && !active_guide_path.poses.empty()) ?
    active_guide_path.poses.back() : goal;
  const double local_escape_goal_yaw = active_is_final_goal ? goal_yaw : active_goal_yaw;
  const bool local_escape_goal_verified =
    active_is_final_goal ? goal_pose_footprint_verified : false;
  tryEscapePrefixAndLocalReverse(
    map_snapshot, planning_grid, clearance_esdf, start, local_escape_goal,
    start_yaw, local_escape_goal_yaw, active_goal_yaw, active_is_final_goal,
    exploration_lidar_fov, seed_ptr, used_clearance,
    local_escape_goal_verified, rollingBudgetExceeded,
    reference, safety, selected_trace, has_spliced_tail, escape_decision);

  if (!safety.safe && !publish_unsafe_trajectory_ && escape_decision.allowed) {
    // 车此刻就压在冲突区里，冲突集证明这条轨迹在有界前缀内驶出障碍并且不再驶回。
    // 继续拒绝只会让唯一能挪走车的执行器拿不到轨迹，参见 escape_prefix.hpp 的说明。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【脱困候选】MINCO 轨迹驶离接触区，仍须通过最终零冲突门：冲突=%zu 起始偏移=%.3f m "
      "prefix_end=%zu prefix_length=%.3f m prefix_yaw_sweep=%.3f rad.",
      escape_decision.collision_count, escape_decision.head_offset_m,
      escape_decision.prefix_end, escape_decision.prefix_length_m,
      escape_decision.prefix_yaw_sweep_rad);
  } else if (!safety.safe && !publish_unsafe_trajectory_) {
    if (safety.collisions.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】安全校验失败，未发布不安全轨迹。");
    } else {
      const CollisionSample & first_collision = safety.collisions.front();
      // 只报冲突数和一个 footprint 采样坐标无法区分三种完全不同的失败:车此刻就压在
      // 障碍上、轨迹先走一段再撞墙、以及轨迹整条贴着障碍蹭行。这三种的修复位置分别在
      // 逃逸放行、图搜索净空和栅格量化一致性上,所以这里把判据本身打全:冲突的离散/
      // 扫掠构成、首末冲突下标、首个冲突点的轨迹中心位姿(注意 first_collision 的坐标
      // 是矩形上的采样点,不是中心),以及逃逸判据为什么没有放行。
      std::size_t discrete_collisions = 0;
      std::size_t swept_collisions = 0;
      std::size_t last_collision_index = 0;
      for (const CollisionSample & collision : safety.collisions) {
        if (collision.swept) {
          ++swept_collisions;
        } else {
          ++discrete_collisions;
        }
        last_collision_index = std::max(last_collision_index, collision.trajectory_index);
      }
      const ReferencePoint & first_center =
        reference.points[std::min(first_collision.trajectory_index, reference.points.size() - 1U)];
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【MINCO拒绝】足迹冲突=%zu，未发布不安全轨迹 "
        "（discrete=%zu swept=%zu）first_index=%zu last_index=%zu points=%zu "
        "first_sample=(%.3f, %.3f) first_center=(%.3f, %.3f, yaw=%.3f) "
        "start=(%.3f, %.3f, yaw=%.3f) raw_points=%zu preprocessed_points=%zu "
        "esdf_refined_points=%zu escape_allowed=%d "
        "escape_head_offset=%.3f m escape_prefix_end=%zu escape_candidate_prefix_end=%zu "
        "escape_prefix_length=%.3f m escape_prefix_yaw_sweep=%.3f rad）",
        safety.collisions.size(), discrete_collisions, swept_collisions,
        first_collision.trajectory_index, last_collision_index, reference.points.size(),
        first_collision.x, first_collision.y,
        first_center.x, first_center.y, first_center.yaw,
        reference.points.front().x, reference.points.front().y, reference.points.front().yaw,
        search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
        selected_trace.esdf_refined_guide.poses.size(), static_cast<int>(escape_decision.allowed),
        escape_decision.head_offset_m, escape_decision.prefix_end,
        escape_decision.candidate_prefix_end,
        escape_decision.prefix_length_m, escape_decision.prefix_yaw_sweep_rad);
      // 上面的数字只说明整条轨迹都不安全,不说明是哪
      // 一层把格子打成了障碍。静态层已离线排除:停车位
      // 姿到最近静态占据格心约 0.61 m,大于 0.4187 m 的
      // 全 yaw 半径。所以必须把实际栅格打出来,否则无法
      // 区分 ROG 投影、terrain 硬障碍与 slope 障碍。两个
      // 窗口分别覆盖轨迹起点和首个冲突点。
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】起点：%s",
        describeGridWindow(
          planning_grid, reference.points.front().x, reference.points.front().y, 0.6,
          obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      if (first_collision.trajectory_index != 0U) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】首个冲突点：%s",
          describeGridWindow(
            planning_grid, first_center.x, first_center.y, 0.6,
            obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      }
    }
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }

  PlanGoalCommitContext commit_ctx{
    goal_id, localization_epoch, plan_request_sequence, map_publication_sequence,
    report_status, keep_active_reference, plan_wall_begin, plan_begin_time,
    committed_for_start, map_snapshot, map_health_epoch, planning_grid, clearance_esdf,
    start, goal, active_goal_yaw, active_is_final_goal, replan_start, replan_frame_matches,
    replan_elapsed_sec, measured_start_x, measured_start_y, search_result, search_wall_ms,
    minco_wall_ms, exploration_lidar_fov, reference, safety, selected_trace,
    has_spliced_tail, esdf_fallback_used, local_repair_used, elapsedWallMs, fail};
  finalizeAndCommitReference(commit_ctx, escape_decision);
}

}  // namespace minco_planner
