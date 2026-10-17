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
  // 矩形足迹在隧道内只允许 2 阶对称，避免 0.581 m 面被当成等价朝向。
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

  const double enter = std::max(0.0, params.clearance_enter);
  const double exit = std::max(enter, params.clearance_exit);
  const double probe_step = 0.5 * std::max(1e-3, params.axis_window / 6.0);
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

    TunnelSegment segment;
    segment.begin = begin;
    segment.end = end;
    segment.axis_yaw.reserve(end - begin + 1);
    double cos2 = 0.0;
    double sin2 = 0.0;
    double weight_sum = 0.0;
    double width_sum = 0.0;
    std::size_t width_count = 0;
    for (std::size_t k = begin; k <= end; ++k) {
      const auto & point = trajectory.points[k];
      const std::size_t prev = k == 0 ? k : k - 1;
      const std::size_t next = std::min(k + 1, trajectory.points.size() - 1);
      double tx = trajectory.points[next].x - trajectory.points[prev].x;
      double ty = trajectory.points[next].y - trajectory.points[prev].y;
      const double tangent_norm = std::hypot(tx, ty);
      if (tangent_norm < 1e-6) {
        tx = 1.0;
        ty = 0.0;
      } else {
        tx /= tangent_norm;
        ty /= tangent_norm;
      }
      const double nx = -ty;
      const double ny = tx;
      const double plus = distanceToWall(esdf, point.x, point.y, nx, ny, params.max_width, probe_step);
      const double minus = distanceToWall(esdf, point.x, point.y, -nx, -ny, params.max_width, probe_step);
      if (plus > 0.0 && minus > 0.0 && plus + minus <= params.max_width + probe_step) {
        width_sum += plus + minus;
        ++width_count;
      }

      const Eigen::Vector2d gradient = esdf.getGradient(point.x, point.y);
      const double gradient_norm = gradient.norm();
      const double axis = std::atan2(ty, tx);
      const double wall_axis = gradient_norm > 1e-6 ? std::atan2(gradient.y(), gradient.x()) + kPi / 2.0 : axis;
      const double weight = gradient_norm > 1e-6 ? gradient_norm : 1.0;
      cos2 += weight * std::cos(2.0 * wall_axis);
      sin2 += weight * std::sin(2.0 * wall_axis);
      weight_sum += weight;
      segment.axis_yaw.push_back(normalize(std::atan2(sin2, cos2) / 2.0));
    }
    segment.coherence = weight_sum > 1e-6 ? std::hypot(cos2, sin2) / weight_sum : 0.0;
    if (width_count == 0) {
      // ESDF 查询在中线可能没有稳定梯度，但 path tangent 仍给出可用轴线。
      ++i;
      continue;
    }
    segment.width_estimate = width_sum / static_cast<double>(width_count);
    if (segment.coherence < params.min_coherence) {
      segment.axis_yaw.assign(segment.axis_yaw.size(),
        std::atan2(trajectory.points[end].y - trajectory.points[begin].y,
          trajectory.points[end].x - trajectory.points[begin].x));
    }
    segments.push_back(std::move(segment));
    ++i;
  }
  return segments;
}

}  // namespace minco_planner
