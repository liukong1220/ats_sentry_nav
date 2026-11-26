// Copyright 2026

#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "minco_planner/trajectory/local_time_scaling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace minco_planner
{

YawSplinePlanner::YawSplinePlanner(YawSplinePlannerParams params) : params_(params) {}

void YawSplinePlanner::setParams(const YawSplinePlannerParams & params) { params_ = params; }

void YawSplinePlanner::apply(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw,
  const std::vector<TunnelSegment> * tunnels,
  bool is_final_goal,
  std::optional<double> preferred_lidar_fov_yaw_offset,
  bool force_goal_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }
  if (force_goal_yaw) {
    // Dedicated LiDAR blind-spot scan-turn: smoothly rotate from initial_yaw to goal_yaw
    // starting immediately at t = 0 so MPC executes nonzero wz right away rather than waiting
    // for the 1.5 cm terminal position handoff disk.
    const double start = normalizeAngle(initial_yaw);
    const double delta = shortestAngularDistance(start, goal_yaw);
    double required_duration = params_.yaw_rate_limit > 1e-6
      ? 1.875 * std::abs(delta) / params_.yaw_rate_limit
      : std::max(1e-3, trajectory.totalTime());
    if (params_.yaw_acceleration_limit > 0.0) {
      required_duration = std::max(
        required_duration,
        std::sqrt(5.7735 * std::abs(delta) / params_.yaw_acceleration_limit));
    }
    const double available = trajectory.totalTime();
    if (required_duration > available && available > 1e-6) {
      const double time_scale = required_duration / available;
      for (auto & point : trajectory.points) {
        point.t *= time_scale;
        point.vx /= time_scale;
        point.vy /= time_scale;
        point.v /= time_scale;
        point.ax /= time_scale * time_scale;
        point.ay /= time_scale * time_scale;
      }
    }
    const double duration = std::max(1e-3, trajectory.totalTime());
    for (auto & point : trajectory.points) {
      const double u = std::max(0.0, std::min(1.0, point.t / duration));
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      const double u5 = u4 * u;
      const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
      point.yaw = normalizeAngle(start + delta * blend);
      point.yaw_rate = delta * blend_rate;
    }
    return;
  }
  if (params_.mode == "path_tangent") {
    applyPathTangent(trajectory, initial_yaw);
    return;
  }
  if (params_.mode == "clearance_aware") {
    applyClearanceAware(
      trajectory, initial_yaw, goal_yaw, tunnels, is_final_goal,
      preferred_lidar_fov_yaw_offset);
    return;
  }
  if (params_.mode == "hold") {
    for (auto & point : trajectory.points) {
      point.yaw = normalizeAngle(initial_yaw);
      point.yaw_rate = 0.0;
    }
    return;
  }
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);
}

void YawSplinePlanner::applyClearanceAware(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw,
  const std::vector<TunnelSegment> * tunnels,
  bool is_final_goal,
  std::optional<double> preferred_lidar_fov_yaw_offset) const
{
  if (trajectory.points.size() < 2) {
    applyGoalHeading(trajectory, initial_yaw, goal_yaw);
    return;
  }

  // 若从洞外近距离接近需要车头正向对准的窄通道，先保证到达洞口前有足够转向时间：
  // 静止起步优先在洞外起点插入平滑原地对准段；运动中则等比放长轨迹时间，避免未转到位进洞。
  std::vector<TunnelSegment> adjusted_tunnels;
  const std::vector<TunnelSegment> * active_tunnels = tunnels;
  if (tunnels != nullptr && !tunnels->empty()) {
    adjusted_tunnels = *tunnels;
    active_tunnels = &adjusted_tunnels;
    const double rate_limit = std::max(1e-3, params_.yaw_rate_limit);
    const double acceleration_limit = std::max(0.0, params_.yaw_acceleration_limit);
    for (const auto & tunnel : adjusted_tunnels) {
      if (!tunnel.forward_alignment || tunnel.axis_yaw.empty() ||
        tunnel.begin >= trajectory.points.size())
      {
        continue;
      }
      const double target_yaw = normalizeAngle(tunnel.axis_yaw.front() + params_.tunnel.yaw_offset);
      const double delta = std::abs(shortestAngularDistance(initial_yaw, target_yaw));
      if (delta <= 0.10) {
        break;
      }
      const double entry_s = trajectory.points[tunnel.begin].s - params_.tunnel.front_extent;
      std::size_t first_at_mouth = tunnel.begin;
      while (first_at_mouth > 1U && trajectory.points[first_at_mouth - 1U].s >= entry_s) {
        --first_at_mouth;
      }
      double required_time = 1.875 * delta / rate_limit;
      if (acceleration_limit > 0.0) {
        const double bang_bang_time = (delta >= rate_limit * rate_limit / acceleration_limit) ?
          (delta / rate_limit + rate_limit / acceleration_limit) :
          (2.0 * std::sqrt(delta / acceleration_limit));
        required_time = std::max(
          required_time,
          std::max(std::sqrt(5.7735 * delta / acceleration_limit), bang_bang_time + 0.15));
      }
      const double available_time =
        trajectory.points[first_at_mouth].t - trajectory.points.front().t;
      if (available_time + 1e-6 < required_time) {
        const ReferencePoint start_point = trajectory.points.front();
        if (start_point.v < 0.15 || available_time <= 1e-3) {
          const double sample_period = std::max(0.01, params_.terminal_yaw_sample_period);
          const int sample_count = std::max(
            2, static_cast<int>(std::ceil(required_time / sample_period)));
          std::vector<ReferencePoint> prefix;
          prefix.reserve(static_cast<std::size_t>(sample_count + 1));
          prefix.push_back(start_point);
          for (int step = 1; step <= sample_count; ++step) {
            ReferencePoint point = start_point;
            point.t = start_point.t + required_time *
              static_cast<double>(step) / static_cast<double>(sample_count);
            point.v = 0.0;
            point.vx = 0.0;
            point.vy = 0.0;
            point.ax = 0.0;
            point.ay = 0.0;
            prefix.push_back(point);
          }
          for (std::size_t idx = 1; idx < trajectory.points.size(); ++idx) {
            trajectory.points[idx].t += required_time;
          }
          prefix.insert(
            prefix.end(), trajectory.points.begin() + 1, trajectory.points.end());
          trajectory.points = std::move(prefix);
          for (auto & segment : adjusted_tunnels) {
            segment.begin += static_cast<std::size_t>(sample_count);
            segment.end += static_cast<std::size_t>(sample_count);
          }
        } else {
          const double time_scale = required_time / available_time;
          if (time_scale <= 2.0) {
            const double t0 = start_point.t;
            for (auto & point : trajectory.points) {
              point.t = t0 + (point.t - t0) * time_scale;
              point.vx /= time_scale;
              point.vy /= time_scale;
              point.v /= time_scale;
              point.ax /= time_scale * time_scale;
              point.ay /= time_scale * time_scale;
            }
          } else {
            // Only slow down the pre-mouth prefix instead of stretching the entire trajectory
            // by 8-10x (which would crush cruising speed to ~0.1 m/s and trip progress watchdogs).
            std::vector<double> factors(trajectory.points.size(), 1.0);
            const double prefix_factor = std::clamp(available_time / required_time, 0.12, 1.0);
            for (std::size_t idx = 0; idx <= first_at_mouth && idx < factors.size(); ++idx) {
              factors[idx] = prefix_factor;
            }
            applyLocalTimeScaling(trajectory, std::move(factors), 1.5);
          }
        }
      }
      break;
    }
  }

  // 开阔区域先平滑朝向目标；只有窄通道才强制朝向路径，保留舵轮横移能力。
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);

  std::vector<double> open_area_yaws;
  open_area_yaws.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    open_area_yaws.push_back(point.yaw);
  }

  const double enter_clearance = std::max(0.0, params_.narrow_clearance_enter);
  const double exit_clearance = std::max(enter_clearance, params_.narrow_clearance_exit);
  // 只接受 2（矩形）与 4（正方形）；其它值按矩形处理，保持保守。
  const int symmetry_order = params_.tangent_symmetry_order == 4 ? 4 : 2;
  bool narrow = false;
  for (const auto & point : trajectory.points) {
    if (std::isfinite(point.clearance) && point.clearance <= enter_clearance) {
      // A path that enters a narrow section must start rotating before the
      // vehicle reaches the wall; delaying tangent alignment creates the
      // large yaw error seen in red-box goal 8.
      narrow = true;
      break;
    }
  }
  const std::size_t point_count = trajectory.points.size();
  std::vector<bool> narrow_flags(point_count, false);
  for (std::size_t i = 1; i < point_count; ++i) {
    const auto & current = trajectory.points[i];
    if (std::isfinite(current.clearance)) {
      // 进入和退出使用不同阈值，避免净空在边界附近时航向模式来回切换。
      narrow = narrow ? current.clearance < exit_clearance : current.clearance <= enter_clearance;
    }
    narrow_flags[i] = narrow;
  }
  // 两段窄通道之间的短开阔间隙按窄通道处理：否则参考 yaw 会在间隙内先回摆到目标朝向，
  // 下一段再转回切线，MPC 跟随这种来回摆动时误差叠加，足迹角点会蹭上通道口
  // （红框斜坡口即此情形）。只桥接两侧都是窄通道的内部间隙，末端开阔段仍回到目标朝向。
  const double bridge_time = std::max(0.0, params_.narrow_gap_bridge_time);
  if (bridge_time > 0.0) {
    std::size_t i = 1;
    while (i < point_count) {
      if (narrow_flags[i]) {
        ++i;
        continue;
      }
      std::size_t gap_end = i;
      while (gap_end < point_count && !narrow_flags[gap_end]) {
        ++gap_end;
      }
      const bool interior = narrow_flags[i - 1] && gap_end < point_count;
      if (interior && trajectory.points[gap_end].t - trajectory.points[i - 1].t <= bridge_time) {
        std::fill(narrow_flags.begin() + i, narrow_flags.begin() + gap_end, true);
      }
      i = gap_end;
    }
  }

  // 每点的切线朝向（前后差分）；切线退化的点记为 NaN。
  std::vector<double> tangent_yaws(point_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 1; i < point_count; ++i) {
    const std::size_t next_index = std::min(i + 1, point_count - 1);
    const double tangent_x = trajectory.points[next_index].x - trajectory.points[i - 1].x;
    const double tangent_y = trajectory.points[next_index].y - trajectory.points[i - 1].y;
    if (std::hypot(tangent_x, tangent_y) > 1e-6) {
      tangent_yaws[i] = std::atan2(tangent_y, tangent_x);
    }
  }
  // 在所有等价朝向中选与 reference_yaw 转角最小的一个；转角相同时保留靠前的候选。
  const auto nearest_aligned_yaw = [](double tangent, double reference_yaw, int order) {
      const double local_period = 2.0 * M_PI / static_cast<double>(order);
      double best = tangent;
      double best_turn = std::abs(shortestAngularDistance(reference_yaw, tangent));
      for (int k = 1; k < order; ++k) {
        const double candidate = normalizeAngle(tangent + local_period * k);
        const double turn = std::abs(shortestAngularDistance(reference_yaw, candidate));
        if (turn < best_turn) {
          best_turn = turn;
          best = candidate;
        }
      }
      return best;
    };
  const bool minimal_rotation = params_.open_area_yaw_mode == "minimal_rotation";
  const double narrow_start_hold = std::max(0.0, params_.narrow_start_hold_length);
  const bool start_in_narrow = narrow_start_hold > 0.0 && point_count > 1U && narrow_flags[1];
  const double start_s = trajectory.points.front().s;
  // 当前开阔段的过渡：yaw 从 open_start_yaw 经五次 smoothstep 到 open_start_yaw + open_delta。
  double open_start_yaw = 0.0;
  double open_delta = 0.0;
  double open_t0 = 0.0;
  double open_t1 = 0.0;

  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  double previous_rate = 0.0;
  const double acceleration_limit = std::max(0.0, params_.yaw_acceleration_limit);
  const double rate_limit = std::max(0.0, params_.yaw_rate_limit);
  // Start before the front reaches the entry. Allow enough trajectory time for
  // even a half-turn, including deceleration into the directed tunnel heading.
  const double alignment_time = std::max(
    1.875 * M_PI / std::max(1e-3, rate_limit),
    acceleration_limit > 0.0 ? std::sqrt(5.7735 * M_PI / acceleration_limit) : 0.0);
  std::vector<double> tunnel_turn_start;
  std::vector<double> tunnel_dynamic_pre;
  std::vector<double> tunnel_locked_branch_offset;
  if (active_tunnels) {
    tunnel_turn_start.reserve(active_tunnels->size());
    tunnel_dynamic_pre.reserve(active_tunnels->size());
    tunnel_locked_branch_offset.reserve(active_tunnels->size());
    for (const auto & tunnel : *active_tunnels) {
      std::size_t entry = tunnel.begin;
      const double front_s = trajectory.points[entry].s - params_.tunnel.front_extent;
      while (entry > 0U && trajectory.points[entry].s > front_s) {
        --entry;
      }
      tunnel_turn_start.push_back(trajectory.points[entry].t - alignment_time);
      tunnel_dynamic_pre.push_back(computeDynamicPreAlignDistance(
        trajectory, tunnel, initial_yaw, 0.0, params_.tunnel, rate_limit, acceleration_limit));
      // 起点已在隧道内/洞口时（如隧道内后退脱困/退出），锁定当前与隧道正方向对齐的同轴分支（0 或 pi），
      // 全程保持正方向沿隧道轴线直到整车完全退出隧道口，绝不在隧道内尝试 180° 掉头。
      double branch_shift = 0.0;
      if (!tunnel.forward_alignment && !tunnel.axis_yaw.empty()) {
        const double aligned_init =
          tunnelAlignedYaw(tunnel.axis_yaw.front(), params_.tunnel.yaw_offset, initial_yaw);
        const double base_target =
          normalizeAngle(tunnel.axis_yaw.front() + params_.tunnel.yaw_offset);
        if (std::abs(shortestAngularDistance(aligned_init, base_target)) > 0.5 * M_PI) {
          branch_shift = M_PI;
        }
      }
      tunnel_locked_branch_offset.push_back(branch_shift);
    }
  }
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;
  const auto preferred_corridor_yaw = [&](double tangent_yaw, double ref_yaw) -> double {
      if (preferred_lidar_fov_yaw_offset.has_value()) {
        // Keep the rectangular footprint aligned with the corridor axis while selecting
        // the branch whose actual TF-derived LiDAR observation direction points forward
        // along the exploration tangent (prevents blind backwards entry into unknown corridors).
        const double lidar_forward_body_yaw = normalizeAngle(
          tangent_yaw - *preferred_lidar_fov_yaw_offset);
        return nearest_aligned_yaw(tangent_yaw, lidar_forward_body_yaw, symmetry_order);
      }
      return nearest_aligned_yaw(tangent_yaw, ref_yaw, symmetry_order);
    };

  for (std::size_t i = 1; i < point_count; ++i) {
    auto & current = trajectory.points[i];
    double desired_yaw = open_area_yaws[i];
    bool tunnel_alignment = false;
    if (active_tunnels != nullptr) {
      for (std::size_t j = 0; j < active_tunnels->size(); ++j) {
        const auto & tunnel = (*active_tunnels)[j];
        if (tunnel.axis_yaw.empty()) {
          continue;
        }
        const TunnelTraversalPhase phase = classifyTunnelTraversalPhase(
          trajectory, i, tunnel, tunnel_dynamic_pre[j], params_.tunnel);
        const bool prealign = tunnel.forward_alignment && current.t >= tunnel_turn_start[j] &&
          current.s <= trajectory.points[tunnel.end].s + params_.tunnel.front_extent +
            params_.tunnel.post_release_distance;
        if (phase != TunnelTraversalPhase::kOpen || prealign) {
          const std::size_t local_index =
            std::min(tunnel.axis_yaw.size() - 1, i >= tunnel.begin ? i - tunnel.begin : 0U);
          if (tunnel.forward_alignment) {
            desired_yaw = normalizeAngle(tunnel.axis_yaw[local_index] + params_.tunnel.yaw_offset);
          } else if (trajectory.points[tunnel.begin].s - start_s <=
            params_.tunnel.front_extent + 1e-6)
          {
            // 已在隧道中：严格锁定进入时的隧道轴线方向（允许 v_x < 0 直线倒车退出隧道）。
            desired_yaw = normalizeAngle(
              tunnel.axis_yaw[local_index] + params_.tunnel.yaw_offset +
              tunnel_locked_branch_offset[j]);
          } else {
            const double tunnel_ref = preferred_lidar_fov_yaw_offset.has_value()
              ? normalizeAngle(tunnel.axis_yaw[local_index] - *preferred_lidar_fov_yaw_offset)
              : previous_yaw;
            desired_yaw = tunnelAlignedYaw(
              tunnel.axis_yaw[local_index], params_.tunnel.yaw_offset, tunnel_ref);
          }
          tunnel_alignment = true;
          break;
        }
      }
    }
    if (minimal_rotation && !tunnel_alignment && !narrow_flags[i]) {
      if (i == 1 || narrow_flags[i - 1]) {
        // 进入开阔段：找下一段窄通道入口，目标取其切线的对齐朝向；没有则为目标朝向。
        std::size_t run_end = i;
        while (run_end < point_count && !narrow_flags[run_end]) {
          ++run_end;
        }
        double target = goal_yaw;
        std::size_t tangent_index = run_end;
        while (tangent_index < point_count && !std::isfinite(tangent_yaws[tangent_index])) {
          ++tangent_index;
        }
        if (run_end < point_count && tangent_index < point_count) {
          target = preferred_corridor_yaw(tangent_yaws[tangent_index], previous_yaw);
        }
        open_start_yaw = previous_yaw;
        open_delta = shortestAngularDistance(previous_yaw, target);
        open_t0 = trajectory.points[i - 1].t;
        open_t1 = trajectory.points[std::min(run_end, point_count - 1)].t;
      }
      // During mapless exploration in a pure open-space run (not pre-aligning for an upcoming
      // narrow corridor on this trajectory), actively steer the TF-derived LiDAR observation
      // axis toward the local travel / frontier heading immediately (rate+accel limited below)
      // instead of deferring rotation across a slow quintic blend that rolling replans reset.
      const bool has_upcoming_narrow = open_t1 < trajectory.points.back().t - 1e-6;
      if (preferred_lidar_fov_yaw_offset.has_value() && !has_upcoming_narrow) {
        const double rem_s = trajectory.points.back().s - current.s;
        if (rem_s <= 0.80 || !std::isfinite(tangent_yaws[i])) {
          desired_yaw = goal_yaw;
        } else {
          desired_yaw = normalizeAngle(tangent_yaws[i] - *preferred_lidar_fov_yaw_offset);
        }
      } else {
        const double span = open_t1 - open_t0;
        const double u = span > 1e-6 ?
          std::max(0.0, std::min(1.0, (current.t - open_t0) / span)) : 1.0;
        const double blend = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        desired_yaw = normalizeAngle(open_start_yaw + open_delta * blend);
      }
    }
    if (!tunnel_alignment && narrow_flags[i] && std::isfinite(tangent_yaws[i])) {
      desired_yaw = preferred_corridor_yaw(tangent_yaws[i], previous_yaw);
      if (start_in_narrow && current.s - start_s <= narrow_start_hold) {
        desired_yaw = normalizeAngle(initial_yaw);
      }
    }

    // 最后按 yaw_rate_limit 裁剪，避免参考轨迹要求舵轮瞬时转向。
    const double dt = std::max(1e-3, current.t - previous_t);
    const double desired_delta = shortestAngularDistance(previous_yaw, desired_yaw);
    double rate = std::max(-rate_limit, std::min(rate_limit, desired_delta / dt));
    if (acceleration_limit > 0.0) {
      // 目标角速度取"走完本步后剩余角差还能以 acceleration_limit 刹停"的上界
      // （离散形式 v^2/(2a) + v*dt <= |delta|，避免越过期望 yaw），再限制相邻点的
      // 角速度变化量，使参考 yaw 的角加速度不超过 acceleration_limit。
      const double braking_rate = acceleration_limit * (
        std::sqrt(dt * dt + 2.0 * std::abs(desired_delta) / acceleration_limit) - dt);
      rate = std::copysign(std::min(std::abs(rate), braking_rate), desired_delta);
      const double max_rate_change = acceleration_limit * dt;
      rate = std::max(previous_rate - max_rate_change, std::min(previous_rate + max_rate_change, rate));
      rate = std::max(-rate_limit, std::min(rate_limit, rate));
    }
    current.yaw = normalizeAngle(previous_yaw + rate * dt);
    current.yaw_rate = rate;
    previous_yaw = current.yaw;
    previous_rate = rate;
    previous_t = current.t;
  }
  // 窄通道可能延续到最终位置。此时保留通道内的切线 yaw，再追加原地、
  // 限速的目标朝向过渡，避免 Navigate action 在位置到达后永久 tracking。
  // 对于非最终目标的滚动局部子目标，绝不追加原地转向停顿。
  if (is_final_goal) {
    appendTerminalGoalYawTransition(trajectory, goal_yaw);
  }
}

void YawSplinePlanner::appendTerminalGoalYawTransition(
  ReferenceTrajectory & trajectory, double goal_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }
  const ReferencePoint terminal = trajectory.points.back();
  const double delta = shortestAngularDistance(terminal.yaw, goal_yaw);
  if (std::abs(delta) <= 1e-8) {
    trajectory.points.back().yaw = normalizeAngle(goal_yaw);
    trajectory.points.back().yaw_rate = 0.0;
    return;
  }

  const double yaw_rate_limit = std::max(1e-3, params_.yaw_rate_limit);
  double duration = 1.875 * std::abs(delta) / yaw_rate_limit;
  if (params_.yaw_acceleration_limit > 0.0) {
    // 五次 smoothstep 的峰值角加速度为 5.7735 * |delta| / T^2；小角度时只按角速度
    // 定时长会让角加速度远超 MPC max_awz。
    duration = std::max(
      duration, std::sqrt(5.7735 * std::abs(delta) / params_.yaw_acceleration_limit));
  }
  const double sample_period = std::max(0.01, params_.terminal_yaw_sample_period);
  const int sample_count = std::max(
    2, static_cast<int>(std::ceil(duration / sample_period)));

  trajectory.points.reserve(trajectory.points.size() + static_cast<std::size_t>(sample_count));
  for (int index = 1; index <= sample_count; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(sample_count);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;

    ReferencePoint point = terminal;
    point.t = terminal.t + duration * u;
    point.s = terminal.s;
    point.v = 0.0;
    point.vx = 0.0;
    point.vy = 0.0;
    point.ax = 0.0;
    point.ay = 0.0;
    point.yaw = normalizeAngle(terminal.yaw + delta * blend);
    point.yaw_rate = delta * blend_rate;
    trajectory.points.push_back(point);
  }
}

void YawSplinePlanner::applyGoalHeading(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  const double start = normalizeAngle(initial_yaw);
  const double delta = shortestAngularDistance(start, goal_yaw);
  double available_duration = std::max(1e-3, trajectory.totalTime());
  const double rate_limited_duration = params_.yaw_rate_limit > 1e-6
                                         ? 1.875 * std::abs(delta) / params_.yaw_rate_limit
                                         : available_duration;
  if (rate_limited_duration > available_duration && trajectory.totalTime() > 1e-6) {
    const double time_scale = rate_limited_duration / available_duration;
    for (auto & point : trajectory.points) {
      point.t *= time_scale;
      point.vx /= time_scale;
      point.vy /= time_scale;
      point.v /= time_scale;
      point.ax /= time_scale * time_scale;
      point.ay /= time_scale * time_scale;
    }
    available_duration = rate_limited_duration;
  }
  double blend_duration = rate_limited_duration;
  if (params_.yaw_acceleration_limit > 0.0) {
    // 小角度转向只按角速度定时长会过短，五次曲线峰值角加速度 5.7735 * |delta| / T^2
    // 远超 MPC max_awz；在轨迹时长内按角加速度上限放长。不改变平移时间参数化。
    blend_duration = std::max(
      blend_duration, std::sqrt(5.7735 * std::abs(delta) / params_.yaw_acceleration_limit));
  }
  const double duration = std::max(1e-3, std::min(available_duration, blend_duration));

  for (auto & point : trajectory.points) {
    const double u = std::max(0.0, std::min(1.0, point.t / duration));
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
    point.yaw = normalizeAngle(start + delta * blend);
    point.yaw_rate = delta * blend_rate;
  }
}

void YawSplinePlanner::applyPathTangent(ReferenceTrajectory & trajectory, double initial_yaw) const
{
  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & prev = trajectory.points[i - 1];
    auto & current = trajectory.points[i];
    const double tangent_yaw = std::atan2(current.y - prev.y, current.x - prev.x);
    const double dt = std::max(1e-3, current.t - previous_t);
    const double max_delta = std::max(0.0, params_.yaw_rate_limit) * dt;
    const double desired_delta = shortestAngularDistance(previous_yaw, tangent_yaw);
    const double clamped_delta = std::max(-max_delta, std::min(max_delta, desired_delta));

    current.yaw = normalizeAngle(previous_yaw + clamped_delta);
    current.yaw_rate = clamped_delta / dt;
    previous_yaw = current.yaw;
    previous_t = current.t;
  }
}

double YawSplinePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double YawSplinePlanner::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

}  // namespace minco_planner
