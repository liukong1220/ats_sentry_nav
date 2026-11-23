// Copyright 2026

#include "minco_planner/safety/footprint_safety_checker.hpp"

#include <algorithm>
#include <cmath>

#include "minco_planner/safety/footprint_samples.hpp"

namespace minco_planner
{

FootprintSafetyChecker::FootprintSafetyChecker(FootprintSafetyParams params)
: params_(params)
{
}

void FootprintSafetyChecker::setParams(const FootprintSafetyParams & params)
{
  params_ = params;
}

FootprintSafetyResult FootprintSafetyChecker::check(
  const ReferenceTrajectory & trajectory,
  const nav_msgs::msg::OccupancyGrid & grid,
  const ats_rc_esdf::EsdfProvider * esdf) const
{
  FootprintSafetyResult result;
  if (!trajectory.valid() || grid.data.empty()) {
    result.safe = false;
    return result;
  }

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    ++result.discrete_samples_checked;
    double collision_x = 0.0;
    double collision_y = 0.0;
    if (!sampleFootprintOccupied(trajectory.points[i], grid, collision_x, collision_y)) {
      continue;
    }
    result.safe = false;
    CollisionSample sample;
    sample.trajectory_index = i;
    sample.segment_index = i;
    sample.x = collision_x;
    sample.y = collision_y;
    result.collisions.push_back(sample);
  }

  // A reference point check alone can skip a wall between two safe poses.  The
  // same oriented rectangular rasterization is therefore evaluated at
  // adaptively subdivided SE(2) poses.  The bound uses actual corner motion,
  // so pure rotation, lateral translation, and diagonal motion share one rule.
  const double base_step = std::max(
    1e-6, static_cast<double>(grid.info.resolution) *
    std::max(1e-3, params_.swept_max_corner_step_cells));
  const double margin_half = params_.safety_margin > 0.0 ?
    std::max(0.005, 0.5 * params_.safety_margin) : base_step;
  const double fine_step_m = std::min(
    base_step,
    std::min(margin_half, std::max(0.005, params_.swept_min_corner_step_m)));

  for (std::size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    ++result.swept_segments_checked;
    if (params_.adaptive_lipschitz_swept_enabled) {
      if (segmentSweptAabbClear(trajectory.points[i], trajectory.points[i + 1], grid)) {
        continue;
      }
      checkSegmentAdaptiveLipschitz(
        trajectory.points[i], trajectory.points[i + 1], i, 0.0, 1.0,
        fine_step_m, grid, esdf, result);
      continue;
    }
    const std::size_t subdivisions = sweptSubdivisions(
      trajectory.points[i], trajectory.points[i + 1], grid);
    for (std::size_t step = 1; step < subdivisions; ++step) {
      const double fraction = static_cast<double>(step) /
        static_cast<double>(subdivisions);
      const ReferencePoint point = interpolate(
        trajectory.points[i], trajectory.points[i + 1], fraction);
      ++result.swept_samples_checked;
      double collision_x = 0.0;
      double collision_y = 0.0;
      if (!sampleFootprintOccupied(point, grid, collision_x, collision_y)) {
        continue;
      }
      result.safe = false;
      CollisionSample sample;
      sample.trajectory_index = i;
      sample.segment_index = i;
      sample.segment_fraction = fraction;
      sample.swept = true;
      sample.x = collision_x;
      sample.y = collision_y;
      result.collisions.push_back(sample);
    }
  }
  return result;
}

bool FootprintSafetyChecker::segmentSweptAabbClear(
  const ReferencePoint & start,
  const ReferencePoint & end,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const double resolution = grid.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }
  const auto & origin = grid.info.origin;
  const double origin_yaw = std::atan2(
    2.0 * (origin.orientation.w * origin.orientation.z +
      origin.orientation.x * origin.orientation.y),
    1.0 - 2.0 * (origin.orientation.y * origin.orientation.y +
      origin.orientation.z * origin.orientation.z));
  const double co = std::cos(origin_yaw);
  const double so = std::sin(origin_yaw);
  const double cx0 = (co * (start.x - origin.position.x) +
    so * (start.y - origin.position.y)) / resolution;
  const double cy0 = (-so * (start.x - origin.position.x) +
    co * (start.y - origin.position.y)) / resolution;
  const double cx1 = (co * (end.x - origin.position.x) +
    so * (end.y - origin.position.y)) / resolution;
  const double cy1 = (-so * (end.x - origin.position.x) +
    co * (end.y - origin.position.y)) / resolution;

  const double hx = (0.5 * std::max(0.0, params_.length) +
    std::max(0.0, params_.safety_margin)) / resolution;
  const double hy = (0.5 * std::max(0.0, params_.width) +
    std::max(0.0, params_.safety_margin)) / resolution;
  const double dyaw = std::abs(normalizeAngle(end.yaw - start.yaw));
  double pad_x = 0.0;
  double pad_y = 0.0;
  if (dyaw <= 1e-4) {
    const double c = std::cos(start.yaw - origin_yaw);
    const double s = std::sin(start.yaw - origin_yaw);
    const double ac = std::abs(c);
    const double as = std::abs(s);
    pad_x = ac * hx + as * hy + 0.5 * (ac + as);
    pad_y = as * hx + ac * hy + 0.5 * (ac + as);
  } else {
    // Exact conservative superset over all orientations yaw(u): circumscribed disc + cell half-diag.
    const double r_diag = std::hypot(hx, hy) + 0.7071067812;
    pad_x = r_diag;
    pad_y = r_diag;
  }
  const double min_x = std::min(cx0, cx1) - pad_x;
  const double max_x = std::max(cx0, cx1) + pad_x;
  const double min_y = std::min(cy0, cy1) - pad_y;
  const double max_y = std::max(cy0, cy1) + pad_y;
  if (min_x < 0.0 || min_y < 0.0 ||
    max_x >= static_cast<double>(grid.info.width) ||
    max_y >= static_cast<double>(grid.info.height))
  {
    return false;
  }
  const int ix0 = static_cast<int>(std::floor(min_x));
  const int ix1 = static_cast<int>(std::floor(max_x));
  const int iy0 = static_cast<int>(std::floor(min_y));
  const int iy1 = static_cast<int>(std::floor(max_y));
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      if (isOccupied(grid, GridIndex{ix, iy})) {
        return false;
      }
    }
  }
  return true;
}

void FootprintSafetyChecker::checkSegmentAdaptiveLipschitz(
  const ReferencePoint & seg_start,
  const ReferencePoint & seg_end,
  std::size_t seg_index,
  double u_start,
  double u_end,
  double fine_step_m,
  const nav_msgs::msg::OccupancyGrid & grid,
  const ats_rc_esdf::EsdfProvider * esdf,
  FootprintSafetyResult & result) const
{
  const ReferencePoint pa = interpolate(seg_start, seg_end, u_start);
  const ReferencePoint pb = interpolate(seg_start, seg_end, u_end);
  const double half_length = 0.5 * std::max(0.0, params_.length) +
    std::max(0.0, params_.safety_margin);
  const double half_width = 0.5 * std::max(0.0, params_.width) +
    std::max(0.0, params_.safety_margin);
  const double r_diag_m = std::hypot(half_length, half_width);
  const double trans_disp = std::hypot(pb.x - pa.x, pb.y - pa.y);
  const double rot_disp = r_diag_m * std::abs(normalizeAngle(pb.yaw - pa.yaw));
  // True continuous path-length upper bound of any point in the body rectangle over [u_start, u_end].
  const double corner_arc_upper = trans_disp + rot_disp;
  if (corner_arc_upper <= fine_step_m || (u_end - u_start) <= 1.0 / 512.0) {
    return;
  }

  if (segmentSweptAabbClear(pa, pb, grid)) {
    return;
  }

  const double u_mid = 0.5 * (u_start + u_end);
  const ReferencePoint p_mid = interpolate(seg_start, seg_end, u_mid);
  ++result.swept_samples_checked;
  double collision_x = 0.0;
  double collision_y = 0.0;
  if (sampleFootprintOccupied(p_mid, grid, collision_x, collision_y)) {
    result.safe = false;
    CollisionSample sample;
    sample.trajectory_index = seg_index;
    sample.segment_index = seg_index;
    sample.segment_fraction = u_mid;
    sample.swept = true;
    sample.x = collision_x;
    sample.y = collision_y;
    result.collisions.push_back(sample);
    // Once a swept collision is recorded on this sub-interval, still check siblings only down
    // to coarse resolution so collision count remains representative without O(512) blow-up.
    if (corner_arc_upper <= std::max(fine_step_m, 0.05)) {
      return;
    }
  } else if (esdf != nullptr && esdf->available()) {
    const double cell_diag_half = 0.7071067812 * grid.info.resolution;
    const double d_center = esdf->getDistance(p_mid.x, p_mid.y);
    if (std::isfinite(d_center) &&
      d_center > r_diag_m + 0.5 * trans_disp + cell_diag_half)
    {
      return;
    }
    // Oriented perimeter 1-Lipschitz lower bound certificate around p_mid.
    constexpr double kPerimSpacing = 0.08;
    const int nx = std::max(1, static_cast<int>(std::ceil(2.0 * half_length / kPerimSpacing)));
    const int ny = std::max(1, static_cast<int>(std::ceil(2.0 * half_width / kPerimSpacing)));
    const double seg_chord_half = 0.5 * std::max(
      2.0 * half_length / static_cast<double>(nx),
      2.0 * half_width / static_cast<double>(ny));
    const double required_d = seg_chord_half + 0.5 * corner_arc_upper + cell_diag_half;
    const double cm = std::cos(p_mid.yaw);
    const double sm = std::sin(p_mid.yaw);
    bool certified = true;
    auto check_local = [&](double lx, double ly) {
      const double wx = p_mid.x + cm * lx - sm * ly;
      const double wy = p_mid.y + sm * lx + cm * ly;
      const double d = esdf->getDistance(wx, wy);
      return std::isfinite(d) && d > required_d;
    };
    for (int k = 0; k <= nx && certified; ++k) {
      const double lx = -half_length + 2.0 * half_length * static_cast<double>(k) / nx;
      certified = check_local(lx, -half_width) && check_local(lx, half_width);
    }
    for (int k = 1; k < ny && certified; ++k) {
      const double ly = -half_width + 2.0 * half_width * static_cast<double>(k) / ny;
      certified = check_local(-half_length, ly) && check_local(half_length, ly);
    }
    if (certified) {
      return;
    }
  }

  checkSegmentAdaptiveLipschitz(
    seg_start, seg_end, seg_index, u_start, u_mid, fine_step_m, grid, esdf, result);
  checkSegmentAdaptiveLipschitz(
    seg_start, seg_end, seg_index, u_mid, u_end, fine_step_m, grid, esdf, result);
}

bool FootprintSafetyChecker::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double wx,
  double wy,
  GridIndex & out) const
{
  if (grid.info.resolution <= 0.0) {
    return false;
  }
  const double yaw = std::atan2(
    2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
      grid.info.origin.orientation.x * grid.info.origin.orientation.y),
    1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
      grid.info.origin.orientation.z * grid.info.origin.orientation.z));
  const double dx = wx - grid.info.origin.position.x;
  const double dy = wy - grid.info.origin.position.y;
  // OccupancyGrid cells live in the origin pose's local axes.  Rotate the
  // query into those axes before converting to an index.
  const double gx = (std::cos(yaw) * dx + std::sin(yaw) * dy) / grid.info.resolution;
  const double gy = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / grid.info.resolution;
  if (gx < 0.0 || gy < 0.0 ||
    gx >= static_cast<double>(grid.info.width) ||
    gy >= static_cast<double>(grid.info.height))
  {
    return false;
  }
  out.x = static_cast<int>(std::floor(gx));
  out.y = static_cast<int>(std::floor(gy));
  return true;
}

bool FootprintSafetyChecker::isOccupied(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  if (index.x < 0 || index.y < 0 ||
    index.x >= static_cast<int>(grid.info.width) ||
    index.y >= static_cast<int>(grid.info.height))
  {
    return true;
  }
  const std::size_t linear =
    static_cast<std::size_t>(index.y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(index.x);
  const int8_t value = grid.data[linear];
  if (value < 0) {
    return params_.unknown_is_obstacle;
  }
  return value >= params_.obstacle_value_threshold;
}

bool FootprintSafetyChecker::sampleFootprintOccupied(
  const ReferencePoint & point,
  const nav_msgs::msg::OccupancyGrid & grid,
  double & collision_x,
  double & collision_y) const
{
  // Match the execution authority: a lattice of points can miss a cell
  // intersected only by a rotated corner (d115 terminal unknown-cell failure).
  // Enumerate the grid bounding box and test the separating axes of the
  // rectangle and each cell. Keep the physical dimensions and margin intact.
  GridIndex center;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
    !std::isfinite(point.yaw) || !worldToGrid(grid, point.x, point.y, center))
  {
    collision_x = point.x;
    collision_y = point.y;
    return true;
  }
  const double resolution = grid.info.resolution;
  const auto & origin = grid.info.origin;
  const double origin_yaw = std::atan2(
    2.0 * (origin.orientation.w * origin.orientation.z +
      origin.orientation.x * origin.orientation.y),
    1.0 - 2.0 * (origin.orientation.y * origin.orientation.y +
      origin.orientation.z * origin.orientation.z));
  const double co = std::cos(origin_yaw);
  const double so = std::sin(origin_yaw);
  const double dx = point.x - origin.position.x;
  const double dy = point.y - origin.position.y;
  const double cx = (co * dx + so * dy) / resolution;
  const double cy = (-so * dx + co * dy) / resolution;
  const double hx = (0.5 * std::max(0.0, params_.length) +
    std::max(0.0, params_.safety_margin)) / resolution;
  const double hy = (0.5 * std::max(0.0, params_.width) +
    std::max(0.0, params_.safety_margin)) / resolution;
  const double c = std::cos(point.yaw - origin_yaw);
  const double s = std::sin(point.yaw - origin_yaw);
  const double ac = std::abs(c);
  const double as = std::abs(s);
  const double extent_x = ac * hx + as * hy;
  const double extent_y = as * hx + ac * hy;
  if (cx - extent_x < 0.0 || cy - extent_y < 0.0 ||
    cx + extent_x >= grid.info.width || cy + extent_y >= grid.info.height)
  {
    collision_x = point.x;
    collision_y = point.y;
    return true;
  }
  for (int iy = static_cast<int>(std::floor(cy - extent_y));
    iy <= static_cast<int>(std::floor(cy + extent_y)); ++iy)
  {
    for (int ix = static_cast<int>(std::floor(cx - extent_x));
      ix <= static_cast<int>(std::floor(cx + extent_x)); ++ix)
    {
      const double cell_dx = ix + 0.5 - cx;
      const double cell_dy = iy + 0.5 - cy;
      if (std::abs(c * cell_dx + s * cell_dy) > hx + 0.5 * (ac + as) ||
        std::abs(-s * cell_dx + c * cell_dy) > hy + 0.5 * (ac + as))
      {
        continue;
      }
      if (isOccupied(grid, GridIndex{ix, iy})) {
        const double local_x = (ix + 0.5) * resolution;
        const double local_y = (iy + 0.5) * resolution;
        collision_x = origin.position.x + co * local_x - so * local_y;
        collision_y = origin.position.y + so * local_x + co * local_y;
        return true;
      }
    }
  }
  return false;
}

std::size_t FootprintSafetyChecker::sweptSubdivisions(
  const ReferencePoint & start, const ReferencePoint & end,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const double half_length = 0.5 * std::max(0.0, params_.length) +
    std::max(0.0, params_.safety_margin);
  const double half_width = 0.5 * std::max(0.0, params_.width) +
    std::max(0.0, params_.safety_margin);
  const std::vector<Eigen::Vector2d> corners = {
    Eigen::Vector2d(-half_length, -half_width),
    Eigen::Vector2d(-half_length, half_width),
    Eigen::Vector2d(half_length, -half_width),
    Eigen::Vector2d(half_length, half_width)};
  const double start_cos = std::cos(start.yaw);
  const double start_sin = std::sin(start.yaw);
  const double end_cos = std::cos(end.yaw);
  const double end_sin = std::sin(end.yaw);
  double maximum_corner_displacement = 0.0;
  for (const auto & corner : corners) {
    const Eigen::Vector2d start_corner(
      start.x + start_cos * corner.x() - start_sin * corner.y(),
      start.y + start_sin * corner.x() + start_cos * corner.y());
    const Eigen::Vector2d end_corner(
      end.x + end_cos * corner.x() - end_sin * corner.y(),
      end.y + end_sin * corner.x() + end_cos * corner.y());
    maximum_corner_displacement = std::max(
      maximum_corner_displacement, (end_corner - start_corner).norm());
  }
  const double maximum_step = std::max(
    1e-6, static_cast<double>(grid.info.resolution) *
    std::max(1e-3, params_.swept_max_corner_step_cells));
  return std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(maximum_corner_displacement / maximum_step)));
}

ReferencePoint FootprintSafetyChecker::interpolate(
  const ReferencePoint & start, const ReferencePoint & end, double fraction)
{
  const double clamped_fraction = std::max(0.0, std::min(1.0, fraction));
  ReferencePoint result = start;
  result.x = start.x + (end.x - start.x) * clamped_fraction;
  result.y = start.y + (end.y - start.y) * clamped_fraction;
  result.yaw = normalizeAngle(
    start.yaw + normalizeAngle(end.yaw - start.yaw) * clamped_fraction);
  result.t = start.t + (end.t - start.t) * clamped_fraction;
  result.s = start.s + (end.s - start.s) * clamped_fraction;
  return result;
}

double FootprintSafetyChecker::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace minco_planner
