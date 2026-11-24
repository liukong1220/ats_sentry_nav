// Copyright 2026

#include "minco_planner/trajectory/tunnel_yaw_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool chordTangent(
  const ReferenceTrajectory & trajectory, std::size_t index, double half_window,
  std::size_t min_index, std::size_t max_index,
  double & tx, double & ty)
{
  const auto & points = trajectory.points;
  const std::size_t count = points.size();
  if (count < 2U || index >= count || min_index > max_index || max_index >= count) {
    return false;
  }
  const double cx = points[index].x;
  const double cy = points[index].y;
  std::size_t lo = index;
  while (lo > min_index && std::hypot(cx - points[lo].x, cy - points[lo].y) < half_window) {
    --lo;
  }
  std::size_t hi = index;
  while (hi < max_index && std::hypot(points[hi].x - cx, points[hi].y - cy) < half_window) {
    ++hi;
  }
  const double dx = points[hi].x - points[lo].x;
  const double dy = points[hi].y - points[lo].y;
  const double norm = std::hypot(dx, dy);
  if (norm <= 1e-4) {
    return false;
  }
  tx = dx / norm;
  ty = dy / norm;
  return true;
}

double distanceToWall(
  const ats_rc_esdf::EsdfProvider & esdf, double x, double y, double nx, double ny,
  double max_distance, double step)
{
  double last = 0.0;
  for (double d = step; d <= max_distance + 1e-9; d += step) {
    const double value = esdf.getDistance(x + nx * d, y + ny * d);
    if (!std::isfinite(value) || value <= step * 0.75) {
      return d;
    }
    last = d;
  }
  return last;
}
}  // namespace

double tunnelAlignedYaw(double axis_yaw, double yaw_offset, double reference_yaw)
{
  const double target = axis_yaw + yaw_offset;
  double best = target;
  double best_error = std::abs(normalize(reference_yaw - target));
  // 矩形足迹在隧道内只允许 2 阶对称：车头（+x，0.571 m 短边）沿隧道轴前进，
  // 倒车朝向 target + pi 等价，+-pi/2 的朝向会让 0.581 m 长边正对隧道口，不可取。
  for (int k = 1; k < 2; ++k) {
    const double candidate = target + k * kPi;
    const double error = std::abs(normalize(reference_yaw - candidate));
    if (error < best_error) {
      best = candidate;
      best_error = error;
    }
  }
  return normalize(best);
}

std::vector<TunnelSegment> detectTunnelSegments(
  const ReferenceTrajectory & trajectory, const ats_rc_esdf::EsdfProvider & esdf,
  const TunnelYawParams & params)
{
  std::vector<TunnelSegment> segments;
  if (!params.enabled || trajectory.points.size() < 3 || !esdf.available()) {
    return segments;
  }
  if (trajectory.points.back().s - trajectory.points.front().s <= 1e-3) {
    return segments;
  }

  const double enter = std::max(0.0, params.clearance_enter);
  const double exit = std::max(enter, params.clearance_exit);
  const double probe_step = 0.5 * std::max(1e-3, params.axis_window / 6.0);
  const double half_window = std::max(0.25, params.axis_window);
  const double smooth_window = std::max(0.60, 2.0 * params.axis_window);
  std::vector<bool> candidates(trajectory.points.size(), false);
  bool in_candidate = false;
  for (std::size_t i = 1; i + 1 < trajectory.points.size(); ++i) {
    const double d = trajectory.points[i].clearance;
    if (!std::isfinite(d)) {
      in_candidate = false;
      continue;
    }
    if (!in_candidate) {
      in_candidate = d <= enter;
    } else if (d > exit) {
      in_candidate = false;
    }
    candidates[i] = in_candidate;
  }

  std::size_t i = 0;
  while (i < candidates.size()) {
    if (!candidates[i]) {
      ++i;
      continue;
    }
    const std::size_t begin = i;
    while (i + 1 < candidates.size() && candidates[i + 1]) {
      ++i;
    }
    const std::size_t end = i;
    if (end <= begin || end - begin + 1 < 2) {
      ++i;
      continue;
    }

    const std::size_t count = end - begin + 1U;
    TunnelSegment segment;
    segment.begin = begin;
    segment.end = end;
    segment.axis_yaw.resize(count, 0.0);
    std::vector<double> tangent_yaws(count, 0.0);
    std::vector<double> sample_cos2(count, 0.0);
    std::vector<double> sample_sin2(count, 0.0);
    struct WallSample
    {
      bool valid = false;
      double lx = 0.0;
      double ly = 0.0;
      double rx = 0.0;
      double ry = 0.0;
    };
    std::vector<WallSample> wall_samples(count);
    double cos2 = 0.0;
    double sin2 = 0.0;
    double weight_sum = 0.0;
    double width_sum = 0.0;
    std::size_t width_count = 0;
    bool has_valid_tangent = false;
    const bool use_segment_bounds =
      (trajectory.points[end].s - trajectory.points[begin].s) >= 0.20;
    const std::size_t chord_min = use_segment_bounds ? begin : 0U;
    const std::size_t chord_max = use_segment_bounds ? end : (trajectory.points.size() - 1U);
    for (std::size_t k = begin; k <= end; ++k) {
      const std::size_t idx = k - begin;
      const auto & point = trajectory.points[k];
      double tx = 1.0;
      double ty = 0.0;
      const bool got_tangent =
        chordTangent(trajectory, k, half_window, chord_min, chord_max, tx, ty) ||
        (use_segment_bounds &&
         chordTangent(trajectory, k, half_window, 0U, trajectory.points.size() - 1U, tx, ty));
      if (!got_tangent) {
        if (idx > 0U && has_valid_tangent) {
          tx = std::cos(tangent_yaws[idx - 1U]);
          ty = std::sin(tangent_yaws[idx - 1U]);
        } else {
          continue;
        }
      } else {
        has_valid_tangent = true;
      }
      const double axis = std::atan2(ty, tx);
      tangent_yaws[idx] = axis;
      const double nx = -ty;
      const double ny = tx;
      const double plus = distanceToWall(esdf, point.x, point.y, nx, ny, params.max_width, probe_step);
      const double minus = distanceToWall(esdf, point.x, point.y, -nx, -ny, params.max_width, probe_step);
      if (plus > 0.0 && minus > 0.0 && plus + minus <= params.max_width + probe_step) {
        width_sum += plus + minus;
        ++width_count;
        wall_samples[idx] = WallSample{
          true,
          point.x + nx * plus,
          point.y + ny * plus,
          point.x - nx * minus,
          point.y - ny * minus};
      }

      // 仅采信指向两侧平行壁面的 ESDF 梯度：中线梯度趋零或洞口/拐角梯度偏离通道切线
      // 超过 ~16°（cos(2*delta) < 0.85）时回退到平滑弦切线，避免角点梯度把轴线拉偏导致假拒。
      const Eigen::Vector2d gradient = esdf.getGradient(point.x, point.y);
      const double gradient_norm = gradient.norm();
      const double wall_axis = gradient_norm > 1e-6 ?
        std::atan2(gradient.y(), gradient.x()) + kPi / 2.0 : axis;
      const bool valid_side_wall =
        gradient_norm >= params.side_wall_gradient_min &&
        std::cos(2.0 * (wall_axis - axis)) >= params.side_wall_cos2_min;
      const double sample_axis = valid_side_wall ? wall_axis : axis;
      const double weight = valid_side_wall ? gradient_norm : 0.5;
      sample_cos2[idx] = weight * std::cos(2.0 * sample_axis);
      sample_sin2[idx] = weight * std::sin(2.0 * sample_axis);
      cos2 += sample_cos2[idx];
      sin2 += sample_sin2[idx];
      weight_sum += weight;
    }
    segment.coherence = weight_sum > 1e-6 ? std::hypot(cos2, sin2) / weight_sum : 0.0;
    if (width_count == 0 || !has_valid_tangent) {
      ++i;
      continue;
    }
    segment.width_estimate = width_sum / static_cast<double>(width_count);
    double pca_coherence_sum = 0.0;
    std::size_t pca_count = 0U;
    for (std::size_t k = begin; k <= end; ++k) {
      const std::size_t idx = k - begin;
      const double center_s = trajectory.points[k].s;
      bool used_pca = false;
      if (params.sidewall_pca_enabled) {
        double mean_lx = 0.0, mean_ly = 0.0, mean_rx = 0.0, mean_ry = 0.0;
        std::size_t valid_pairs = 0U;
        for (std::size_t m = begin; m <= end; ++m) {
          const auto & ws = wall_samples[m - begin];
          if (ws.valid && std::abs(trajectory.points[m].s - center_s) <= smooth_window) {
            mean_lx += ws.lx;
            mean_ly += ws.ly;
            mean_rx += ws.rx;
            mean_ry += ws.ry;
            ++valid_pairs;
          }
        }
        if (valid_pairs >= 3U) {
          const double inv_n = 1.0 / static_cast<double>(valid_pairs);
          mean_lx *= inv_n;
          mean_ly *= inv_n;
          mean_rx *= inv_n;
          mean_ry *= inv_n;
          double sxx = 0.0, sxy = 0.0, syy = 0.0;
          for (std::size_t m = begin; m <= end; ++m) {
            const auto & ws = wall_samples[m - begin];
            if (ws.valid && std::abs(trajectory.points[m].s - center_s) <= smooth_window) {
              const double dlx = ws.lx - mean_lx;
              const double dly = ws.ly - mean_ly;
              const double drx = ws.rx - mean_rx;
              const double dry = ws.ry - mean_ry;
              sxx += dlx * dlx + drx * drx;
              sxy += dlx * dly + drx * dry;
              syy += dly * dly + dry * dry;
            }
          }
          const double trace = sxx + syy;
          if (trace > 1e-5) {
            const double diff = sxx - syy;
            const double disc = std::hypot(diff, 2.0 * sxy);
            const double anisotropy = disc / (trace + 1e-9);
            const double pca_axis = 0.5 * std::atan2(2.0 * sxy, diff);
            if (anisotropy >= params.min_coherence &&
              std::cos(2.0 * (pca_axis - tangent_yaws[idx])) >= params.side_wall_cos2_min)
            {
              segment.axis_yaw[idx] = tunnelAlignedYaw(pca_axis, 0.0, tangent_yaws[idx]);
              pca_coherence_sum += anisotropy;
              ++pca_count;
              used_pca = true;
            }
          }
        }
      }
      if (!used_pca) {
        if (segment.coherence < params.min_coherence) {
          segment.axis_yaw[idx] = tangent_yaws[idx];
        } else {
          double win_cos2 = 0.0;
          double win_sin2 = 0.0;
          for (std::size_t m = begin; m <= end; ++m) {
            if (std::abs(trajectory.points[m].s - center_s) <= smooth_window) {
              win_cos2 += sample_cos2[m - begin];
              win_sin2 += sample_sin2[m - begin];
            }
          }
          segment.axis_yaw[idx] = (std::hypot(win_cos2, win_sin2) > 1e-6) ?
            normalize(std::atan2(win_sin2, win_cos2) / 2.0) :
            tangent_yaws[idx];
        }
      }
    }
    if (pca_count > 0U) {
      segment.coherence = std::max(
        segment.coherence, pca_coherence_sum / static_cast<double>(pca_count));
    }
    // 对齐前按弧长平滑 2 阶轴向，消除 PCA 开关跳变或洞口梯度切换引入的单步角度阶跃。
    if (count >= 3U) {
      const double axis_smooth_window = std::max(0.45, smooth_window);
      std::vector<double> smoothed_axis(count, 0.0);
      for (std::size_t k = begin; k <= end; ++k) {
        const std::size_t idx = k - begin;
        const double center_s = trajectory.points[k].s;
        double sum_c2 = 0.0;
        double sum_s2 = 0.0;
        for (std::size_t m = begin; m <= end; ++m) {
          const double ds = std::abs(trajectory.points[m].s - center_s);
          if (ds <= axis_smooth_window) {
            const double w = 1.0 - 0.5 * (ds / axis_smooth_window);
            const double a = segment.axis_yaw[m - begin];
            sum_c2 += w * std::cos(2.0 * a);
            sum_s2 += w * std::sin(2.0 * a);
          }
        }
        smoothed_axis[idx] = std::hypot(sum_c2, sum_s2) > 1e-6 ?
          normalize(0.5 * std::atan2(sum_s2, sum_c2)) :
          segment.axis_yaw[idx];
      }
      segment.axis_yaw = std::move(smoothed_axis);
    }
    const double start_s = trajectory.points.front().s;
    const bool approaching_from_outside =
      (trajectory.points[begin].s - start_s) > params.front_extent + 1e-6;
    segment.forward_alignment =
      segment.width_estimate < params.forward_width_limit && approaching_from_outside;
    if (segment.forward_alignment) {
      for (std::size_t k = begin; k <= end; ++k) {
        const std::size_t idx = k - begin;
        const double branch_ref = idx == 0U ? tangent_yaws[idx] : segment.axis_yaw[idx - 1U];
        segment.axis_yaw[idx] = tunnelAlignedYaw(segment.axis_yaw[idx], 0.0, branch_ref);
      }
    }
    segments.push_back(std::move(segment));
    ++i;
  }
  return segments;
}

void resolveTerminalTunnelGoalAlignment(
  std::vector<TunnelSegment> & tunnels,
  const ReferenceTrajectory & trajectory,
  double goal_yaw,
  bool is_final_goal,
  const TunnelYawParams & params)
{
  if (!params.enabled || !params.terminal_goal_axial_override_enabled || !is_final_goal ||
    tunnels.empty() || trajectory.points.size() < 2U)
  {
    return;
  }
  const double total_end_s = trajectory.points.back().s;
  const double start_s = trajectory.points.front().s;
  const double narrow_turn_diameter = 2.0 * std::max(0.30, params.circumscribed_radius) + 0.04;
  for (auto & segment : tunnels) {
    if (segment.axis_yaw.empty() || segment.end >= trajectory.points.size()) {
      continue;
    }
    // 仅当最终目标点本身位于不能原地 180° 掉头的窄通道内部/口部时触发：
    // 若 goal_yaw 与隧道轴线同轴（含反向同轴，如 V1 在东口外侧对齐 yaw=0 后倒车 0.5 m 入位），
    // 则在洞外预先将车头正方向对齐到该同轴方向并全程保持轴向滑入目标，避免洞内 180° 原地扫掠碰撞。
    const bool ends_inside_tunnel =
      (total_end_s - trajectory.points[segment.end].s) <=
      std::max(0.25, params.front_extent + 0.15);
    if (!ends_inside_tunnel || segment.width_estimate >= narrow_turn_diameter) {
      continue;
    }
    const double nearest_axial =
      tunnelAlignedYaw(segment.axis_yaw.back(), params.yaw_offset, goal_yaw);
    if (std::abs(normalize(goal_yaw - nearest_axial)) > std::max(1e-3, params.axis_tolerance)) {
      continue;
    }
    segment.terminal_goal_axial_override = true;
    const bool approaching_from_outside =
      (trajectory.points[segment.begin].s - start_s) > params.front_extent + 1e-6;
    if (approaching_from_outside) {
      segment.forward_alignment = true;
    }
    const double desired_raw_axis = normalize(nearest_axial - params.yaw_offset);
    for (double & axis_val : segment.axis_yaw) {
      axis_val = tunnelAlignedYaw(axis_val, 0.0, desired_raw_axis);
    }
  }
}

double computeDynamicPreAlignDistance(
  const ReferenceTrajectory & trajectory,
  const TunnelSegment & tunnel,
  double initial_yaw,
  double initial_yaw_rate,
  const TunnelYawParams & params,
  double yaw_rate_limit,
  double yaw_accel_limit)
{
  const double base_pre = std::max(0.0, params.pre_align_distance);
  if (!params.dynamic_pre_align_enabled || tunnel.axis_yaw.empty() ||
    tunnel.begin >= trajectory.points.size() || trajectory.points.size() < 2U)
  {
    return base_pre;
  }
  const double target_yaw = tunnel.forward_alignment ?
    normalize(tunnel.axis_yaw.front() + params.yaw_offset) :
    tunnelAlignedYaw(tunnel.axis_yaw.front(), params.yaw_offset, initial_yaw);
  const double delta = std::abs(normalize(target_yaw - initial_yaw));
  const double w_max = std::max(0.1, yaw_rate_limit);
  const double a_max = std::max(0.1, yaw_accel_limit > 0.0 ? yaw_accel_limit : 2.0);
  const double brake_time = std::abs(initial_yaw_rate) / a_max;
  const double turn_time = std::max(
    1.875 * delta / w_max,
    std::sqrt(5.7735026919 * delta / a_max));
  const double total_required_time =
    brake_time + turn_time + std::max(0.0, params.pre_align_time_margin);

  const double armed_s =
    trajectory.points[tunnel.begin].s - params.front_extent - std::max(0.0, params.armed_distance);
  std::size_t armed_idx = tunnel.begin;
  while (armed_idx > 0U && trajectory.points[armed_idx].s > armed_s) {
    --armed_idx;
  }
  const double t_armed = trajectory.points[armed_idx].t;
  const double t_start = t_armed - total_required_time;
  std::size_t start_idx = armed_idx;
  while (start_idx > 0U && trajectory.points[start_idx].t > t_start) {
    --start_idx;
  }
  const double dynamic_pre = std::max(
    0.0, trajectory.points[tunnel.begin].s - trajectory.points[start_idx].s);
  return std::max(base_pre, dynamic_pre);
}

TunnelTraversalPhase classifyTunnelTraversalPhase(
  const ReferenceTrajectory & trajectory,
  std::size_t point_index,
  const TunnelSegment & tunnel,
  double dynamic_pre_align_distance,
  const TunnelYawParams & params)
{
  if (!params.enabled || tunnel.axis_yaw.empty() || point_index >= trajectory.points.size() ||
    tunnel.begin >= trajectory.points.size() || tunnel.end >= trajectory.points.size())
  {
    return TunnelTraversalPhase::kOpen;
  }
  const double s = trajectory.points[point_index].s;
  const double half_len = std::max(0.0, params.front_extent);
  const double s_mouth = trajectory.points[tunnel.begin].s - half_len;
  const double s_armed = s_mouth - std::max(0.0, params.armed_distance);
  const double s_prepare = std::min(
    s_armed, trajectory.points[tunnel.begin].s - std::max(params.pre_align_distance, dynamic_pre_align_distance));
  // Full-vehicle exit: rear bumper (or front bumper when reversing out) clears tunnel.end by half_len.
  const double s_rear_clear = trajectory.points[tunnel.end].s + half_len;
  const double s_exit_done = s_rear_clear + std::max(0.0, params.post_release_distance);
  if (s < s_prepare || s > s_exit_done) {
    return TunnelTraversalPhase::kOpen;
  }
  if (s < s_armed) {
    return TunnelTraversalPhase::kPrepare;
  }
  if (s < s_mouth) {
    return TunnelTraversalPhase::kArmed;
  }
  if (s <= s_rear_clear) {
    return TunnelTraversalPhase::kCommitted;
  }
  return TunnelTraversalPhase::kExit;
}

bool forwardTunnelEntriesAligned(
  const ReferenceTrajectory & trajectory, const std::vector<TunnelSegment> & tunnels,
  const TunnelYawParams & params)
{
  const double axis_tolerance = std::max(1e-3, params.axis_tolerance);
  for (const auto & tunnel : tunnels) {
    if (!tunnel.forward_alignment || tunnel.axis_yaw.empty()) {
      continue;
    }
    const double entry_s = trajectory.points[tunnel.begin].s - params.front_extent;
    // Check every pose once the front reaches the mouth and all poses inside the passage.
    // Never reject index 0 (initial measured yaw) when the robot starts outside the mouth.
    std::size_t first = tunnel.begin;
    while (first > 1U && trajectory.points[first - 1U].s >= entry_s) {
      --first;
    }
    for (std::size_t i = first; i <= tunnel.end; ++i) {
      const auto local_index = i > tunnel.begin ? i - tunnel.begin : 0U;
      const double target = tunnel.axis_yaw[local_index] + params.yaw_offset;
      if (std::abs(normalize(trajectory.points[i].yaw - target)) > axis_tolerance) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace minco_planner
