// Copyright 2026

#include "minco_planner/safety/local_collision_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "minco_planner/planning/clearance_ladder.hpp"

namespace minco_planner
{

LocalCollisionRepair::LocalCollisionRepair(LocalCollisionRepairParams params)
: params_(params)
{
}

void LocalCollisionRepair::setParams(const LocalCollisionRepairParams & params)
{
  params_ = params;
}

bool LocalCollisionRepair::repair(
  ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & collisions,
  const nav_msgs::msg::OccupancyGrid & grid,
  LocalCollisionRepairStats * stats) const
{
  if (!params_.enabled || collisions.collisions.empty() || grid.info.resolution <= 0.0) {
    return false;
  }

  // 第二档只在严格档挑不出候选格时使用,且永远不高于严格档。
  const double fallback_clearance = params_.inscribed_radius_m > 0.0
    ? footprintConsistentClearanceFloor(
      params_.inscribed_radius_m, grid.info.resolution, params_.required_clearance_m)
    : 0.0;
  // search_radius 是用户配置的引导点位移上界,这里不覆盖它。但它必须不小于
  // 所要求的净空:贴着障碍的点至少要挪出一个净空距离才可能合格,
  // 0.35 m 搜索半径配 0.419 m 严格净空必然是空集(domain 169/171 的实测形态)。
  // 配置不足时由调用方按 stats 里的两个字段判断,而不是在这里悄悄放宽。
  if (stats != nullptr) {
    stats->strict_clearance_m = params_.required_clearance_m;
    stats->fallback_clearance_m = fallback_clearance;
    stats->effective_search_radius_m = params_.search_radius;
  }

  // 端点保护:index 0 是机器人当前位姿,轨迹末点是 action 下发的目标位姿。
  // 二者都不是可以自由挪动的引导点。yaw 规划会在终点位置追加一段原地转向
  // 采样,这些采样与末点同坐标,所以"与末点重合"必须整段一起保护,
  // 否则只有部分采样被挪走,引导点序列会变成 目标->东侧->目标 的回头刺,
  // 重解出的末段反向切入目标,终端 yaw 被顶到墙上(domain 173 目标 8 实测)。
  const bool has_terminal = !trajectory.points.empty();
  const double terminal_x = has_terminal ? trajectory.points.back().x : 0.0;
  const double terminal_y = has_terminal ? trajectory.points.back().y : 0.0;
  const double coincident_eps = 1e-6;
  const std::size_t n_pts = trajectory.points.size();
  std::vector<double> orig_x(n_pts, 0.0);
  std::vector<double> orig_y(n_pts, 0.0);
  std::vector<double> arc_s(n_pts, 0.0);
  for (std::size_t i = 0; i < n_pts; ++i) {
    orig_x[i] = trajectory.points[i].x;
    orig_y[i] = trajectory.points[i].y;
    if (i > 0U) {
      arc_s[i] = arc_s[i - 1U] + std::hypot(
        orig_x[i] - orig_x[i - 1U], orig_y[i] - orig_y[i - 1U]);
    }
  }
  std::vector<std::uint8_t> repaired_mask(n_pts, 0U);

  bool changed = false;
  for (const auto & collision : collisions.collisions) {
    if (collision.trajectory_index >= trajectory.points.size()) {
      continue;
    }
    auto & point = trajectory.points[collision.trajectory_index];
    if (stats != nullptr) {
      ++stats->collision_points;
    }
    const bool is_terminal_position = has_terminal &&
      std::abs(point.x - terminal_x) <= coincident_eps &&
      std::abs(point.y - terminal_y) <= coincident_eps;
    if (collision.trajectory_index == 0U || is_terminal_position) {
      if (stats != nullptr) {
        ++stats->endpoint_protected;
      }
      continue;
    }
    double repaired_x = point.x;
    double repaired_y = point.y;
    bool used_fallback = false;
    // Require the candidate cell to move strictly farther from the exact obstacle cell
    // that the rectangular footprint hit (collision.x, collision.y). Otherwise a tunnel-mouth
    // or narrow-corridor point whose isotropic circular clearance already meets 0.341..0.436 m
    // gets mapped right back onto itself (negligible_shift) while its rectangular corner still clips.
    const double min_repulsion_gain = 0.5 * grid.info.resolution;
    bool found = findNearestClearCell(
      grid, point.x, point.y, collision.x, collision.y, min_repulsion_gain,
      params_.required_clearance_m, params_.search_radius, repaired_x, repaired_y);
    if (!found && fallback_clearance > 0.0 &&
      fallback_clearance + 1e-9 < params_.required_clearance_m)
    {
      found = findNearestClearCell(
        grid, point.x, point.y, collision.x, collision.y, min_repulsion_gain,
        fallback_clearance, params_.search_radius, repaired_x, repaired_y);
      used_fallback = found;
    }
    if (!found) {
      if (stats != nullptr) {
        ++stats->no_candidate;
      }
      continue;
    }
    // 没有真正挪动就不要报告已修复:上层会白跑一次
    // MINCO 求解,再拒同一条轨迹。这里跳过,让它走既
    // 有的 fail-closed 拒绝分支。
    if (std::hypot(repaired_x - point.x, repaired_y - point.y) <
      0.25 * grid.info.resolution)
    {
      if (stats != nullptr) {
        ++stats->negligible_shift;
      }
      continue;
    }
    point.x = repaired_x;
    point.y = repaired_y;
    repaired_mask[collision.trajectory_index] = 1U;
    changed = true;
    if (stats != nullptr) {
      if (used_fallback) {
        ++stats->fallback_repaired;
      } else {
        ++stats->strict_repaired;
      }
    }
  }

  // Blend a smooth C1 cosine-tapered shoulder (+/- 0.65 m along arc length) around
  // shifted colliding points so MINCO re-optimization does not smooth or resample
  // across an isolated 10-20 cm spike at a tunnel mouth or narrow corner.
  if (changed && n_pts > 2U) {
    constexpr double kShoulderRadiusM = 0.65;
    const double grid_yaw = std::atan2(
      2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
        grid.info.origin.orientation.x * grid.info.origin.orientation.y),
      1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
        grid.info.origin.orientation.z * grid.info.origin.orientation.z));
    const double cos_y = std::cos(grid_yaw);
    const double sin_y = std::sin(grid_yaw);
    std::vector<std::size_t> repaired_indices;
    repaired_indices.reserve(n_pts);
    for (std::size_t i = 0; i < n_pts; ++i) {
      if (repaired_mask[i] != 0U) {
        repaired_indices.push_back(i);
      }
    }
    for (std::size_t k = 1U; k + 1U < n_pts; ++k) {
      if (repaired_mask[k] != 0U) {
        continue;
      }
      const bool is_terminal_position =
        std::abs(orig_x[k] - terminal_x) <= coincident_eps &&
        std::abs(orig_y[k] - terminal_y) <= coincident_eps;
      if (is_terminal_position) {
        continue;
      }
      std::size_t nearest_r = repaired_indices.front();
      double min_ds = std::abs(arc_s[k] - arc_s[nearest_r]);
      for (std::size_t r : repaired_indices) {
        const double ds = std::abs(arc_s[k] - arc_s[r]);
        if (ds < min_ds) {
          min_ds = ds;
          nearest_r = r;
        }
      }
      if (min_ds >= kShoulderRadiusM) {
        continue;
      }
      const double w = 0.5 * (1.0 + std::cos(M_PI * min_ds / kShoulderRadiusM));
      const double shift_x = trajectory.points[nearest_r].x - orig_x[nearest_r];
      const double shift_y = trajectory.points[nearest_r].y - orig_y[nearest_r];
      double scale = w;
      for (int attempt = 0; attempt < 3; ++attempt) {
        const double cand_x = orig_x[k] + scale * shift_x;
        const double cand_y = orig_y[k] + scale * shift_y;
        const double dx0 = cand_x - grid.info.origin.position.x;
        const double dy0 = cand_y - grid.info.origin.position.y;
        const double lx = cos_y * dx0 + sin_y * dy0;
        const double ly = -sin_y * dx0 + cos_y * dy0;
        const int mx = static_cast<int>(std::floor(lx / grid.info.resolution));
        const int my = static_cast<int>(std::floor(ly / grid.info.resolution));
        if (isFree(grid, mx, my)) {
          trajectory.points[k].x = cand_x;
          trajectory.points[k].y = cand_y;
          break;
        }
        scale *= 0.5;
      }
    }
  }
  return changed;
}

bool LocalCollisionRepair::findNearestClearCell(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x,
  double y,
  double obs_x,
  double obs_y,
  double min_repulsion_gain_m,
  double required_clearance_m,
  double search_radius_m,
  double & repaired_x,
  double & repaired_y) const
{
  const double yaw = std::atan2(
    2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
      grid.info.origin.orientation.x * grid.info.origin.orientation.y),
    1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
      grid.info.origin.orientation.z * grid.info.origin.orientation.z));
  const double dx_from_origin = x - grid.info.origin.position.x;
  const double dy_from_origin = y - grid.info.origin.position.y;
  const double local_x = std::cos(yaw) * dx_from_origin +
    std::sin(yaw) * dy_from_origin;
  const double local_y = -std::sin(yaw) * dx_from_origin +
    std::cos(yaw) * dy_from_origin;
  const int center_x = static_cast<int>(
    std::floor(local_x / grid.info.resolution));
  const int center_y = static_cast<int>(
    std::floor(local_y / grid.info.resolution));
  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(search_radius_m / grid.info.resolution)));
  const double current_obs_dist = std::hypot(x - obs_x, y - obs_y);
  const bool enforce_repulsion = std::isfinite(obs_x) && std::isfinite(obs_y) &&
    min_repulsion_gain_m > 0.0;

  double best_distance_sq = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = center_x + dx;
      const int my = center_y + dy;
      if (!isFree(grid, mx, my) || !hasClearance(grid, mx, my, required_clearance_m)) {
        continue;
      }
      const double cell_local_x = (static_cast<double>(mx) + 0.5) * grid.info.resolution;
      const double cell_local_y = (static_cast<double>(my) + 0.5) * grid.info.resolution;
      const double wx = grid.info.origin.position.x +
        std::cos(yaw) * cell_local_x - std::sin(yaw) * cell_local_y;
      const double wy = grid.info.origin.position.y +
        std::sin(yaw) * cell_local_x + std::cos(yaw) * cell_local_y;
      if (enforce_repulsion &&
        std::hypot(wx - obs_x, wy - obs_y) < current_obs_dist + min_repulsion_gain_m)
      {
        continue;
      }
      const double distance_sq = (wx - x) * (wx - x) + (wy - y) * (wy - y);
      if (distance_sq >= best_distance_sq) {
        continue;
      }
      best_distance_sq = distance_sq;
      repaired_x = wx;
      repaired_y = wy;
      found = true;
    }
  }
  return found;
}

bool LocalCollisionRepair::hasClearance(
  const nav_msgs::msg::OccupancyGrid & grid, int mx, int my,
  double required_clearance_m) const
{
  if (required_clearance_m <= 0.0) {
    return true;
  }
  const double resolution = grid.info.resolution;
  const int span = static_cast<int>(std::ceil(required_clearance_m / resolution));
  const double limit_sq = required_clearance_m * required_clearance_m;
  for (int dy = -span; dy <= span; ++dy) {
    for (int dx = -span; dx <= span; ++dx) {
      const double offset_x = static_cast<double>(dx) * resolution;
      const double offset_y = static_cast<double>(dy) * resolution;
      if (offset_x * offset_x + offset_y * offset_y > limit_sq) {
        continue;
      }
      // isFree 对越界返回 false,越界因此等价于障碍,方向是保守的。
      if (!isFree(grid, mx + dx, my + dy)) {
        return false;
      }
    }
  }
  return true;
}

bool LocalCollisionRepair::isFree(const nav_msgs::msg::OccupancyGrid & grid, int mx, int my) const
{
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(grid.info.width) ||
    my >= static_cast<int>(grid.info.height))
  {
    return false;
  }
  const std::size_t linear =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(mx);
  const int8_t value = grid.data[linear];
  if (value < 0) {
    return !params_.unknown_is_obstacle;
  }
  return value < params_.obstacle_value_threshold;
}

}  // namespace minco_planner
