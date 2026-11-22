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
    if (segment.coherence < params.min_coherence) {
      segment.axis_yaw = tangent_yaws;
    } else {
      for (std::size_t k = begin; k <= end; ++k) {
        const std::size_t idx = k - begin;
        const double center_s = trajectory.points[k].s;
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
    const double start_s = trajectory.points.front().s;
    const bool approaching_from_outside =
      (trajectory.points[begin].s - start_s) > params.front_extent + 1e-6;
    segment.forward_alignment =
      segment.width_estimate < params.forward_width_limit && approaching_from_outside;
    if (segment.forward_alignment) {
      for (std::size_t k = begin; k <= end; ++k) {
        const std::size_t idx = k - begin;
        segment.axis_yaw[idx] = tunnelAlignedYaw(segment.axis_yaw[idx], 0.0, tangent_yaws[idx]);
      }
    }
    segments.push_back(std::move(segment));
    ++i;
  }
  return segments;
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
