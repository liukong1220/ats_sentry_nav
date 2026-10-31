// Copyright 2026

#include "minco_planner/planning/grid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace minco_planner
{

namespace
{

struct QueueNode
{
  int x = 0;
  int y = 0;
  double f = 0.0;
  double g = 0.0;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

}  // namespace

GridAstar::GridAstar(GridAstarParams params)
: params_(params)
{
}

void GridAstar::setParams(const GridAstarParams & params)
{
  params_ = params;
}

GridAstarResult GridAstar::plan(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return planWithClearance(grid, start, goal, -1.0);
}

GridOccupancyPolicy GridAstar::occupancyPolicy() const
{
  GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = params_.obstacle_value_threshold;
  policy.unknown_is_obstacle = params_.unknown_is_obstacle;
  return policy;
}

GridAstarResult GridAstar::planWithClearance(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double clearance_override) const
{
  if (clearance_override >= 0.0 &&
    std::abs(clearance_override - params_.safe_distance) > 1e-9)
  {
    GridAstarParams relaxed = params_;
    relaxed.safe_distance = clearance_override;
    return GridAstar(relaxed).planWithClearance(grid, start, goal, -1.0);
  }

  GridAstarResult result;
  result.path.header = grid.header;

  if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 ||
    grid.data.empty())
  {
    result.reason = "grid is empty";
    return result;
  }

  GridIndex start_idx;
  GridIndex goal_idx;
  if (!worldToGrid(grid, start.pose.position.x, start.pose.position.y, start_idx)) {
    result.reason = "start is outside grid";
    return result;
  }
  if (!worldToGrid(grid, goal.pose.position.x, goal.pose.position.y, goal_idx)) {
    result.reason = "goal is outside grid";
    return result;
  }
  const GridOccupancyPolicy policy = occupancyPolicy();
  // The robot already stands on the start cell. Rejecting it cannot move the
  // robot, so the only outcome of a fail-closed rejection here is a livelock.
  if (!params_.assume_start_traversable && !isTraversable(grid, start_idx)) {
    result.reason = "start is occupied";
    return result;
  }
  if (!isTraversable(grid, goal_idx)) {
    const double goal_clearance = params_.relax_endpoint_clearance
      ? measureGridClearance(grid, goal_idx.x, goal_idx.y, params_.safe_distance, policy)
      : 0.0;
    if (!(goal_clearance > 0.0) || goal_clearance + 1e-9 < params_.min_safe_distance) {
      result.reason = "goal is occupied";
      return result;
    }
    GridAstarParams relaxed = params_;
    relaxed.safe_distance = goal_clearance;
    relaxed.relax_endpoint_clearance = false;
    GridAstarResult relaxed_result =
      GridAstar(relaxed).planWithClearance(grid, start, goal, -1.0);
    if (relaxed_result.success) {
      return relaxed_result;
    }
    result.reason = "goal is occupied";
    return result;
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  // 一次精确距离变换，同时用于可通行判定（与 hasGridClearance 等价）和离墙代价。
  const std::vector<double> blocked_squared = computeBlockedSquaredDistanceCells(grid, policy);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<uint8_t> closed(cell_count, 0);
  // 起点脱出：只有起点自己不满足净空时才启用。escape_steps 记录到达该格之前连续经过的
  // 低于净空的格数（走到达标格就清零），超过 start_escape_max_steps 不再外扩。
  const bool start_escape = params_.start_escape_max_steps > 0 &&
    !isTraversable(grid, blocked_squared, start_idx);
  std::vector<uint16_t> escape_steps(start_escape ? cell_count : 0U, 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open;

  const std::size_t start_linear = linearIndex(grid, start_idx);
  const std::size_t goal_linear = linearIndex(grid, goal_idx);
  g_score[start_linear] = 0.0;
  open.push(QueueNode {
      start_idx.x,
      start_idx.y,
      heuristic(start_idx, goal_idx),
      0.0});

  static constexpr int kDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static constexpr int kDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  static constexpr int kDx4[4] = {1, 0, -1, 0};
  static constexpr int kDy4[4] = {0, 1, 0, -1};
  const int neighbor_count = params_.allow_diagonal ? 8 : 4;

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();
    GridIndex current_idx {current.x, current.y};
    const std::size_t current_linear = linearIndex(grid, current_idx);
    if (closed[current_linear]) {
      continue;
    }
    closed[current_linear] = 1;
    ++result.expanded_nodes;

    if (current_linear == goal_linear) {
      result.success = true;
      break;
    }

    // 当前格自己低于净空（只可能是起点或从起点爬出来的格）。
    const bool current_below_clearance =
      start_escape && !isTraversable(grid, blocked_squared, current_idx);
    for (int i = 0; i < neighbor_count; ++i) {
      const int dx = params_.allow_diagonal ? kDx8[i] : kDx4[i];
      const int dy = params_.allow_diagonal ? kDy8[i] : kDy4[i];
      GridIndex next {current.x + dx, current.y + dy};
      if (next.x < 0 || next.y < 0 ||
        next.x >= static_cast<int>(grid.info.width) ||
        next.y >= static_cast<int>(grid.info.height))
      {
        continue;
      }
      bool escape_edge = false;
      if (!isTraversable(grid, blocked_squared, next)) {
        // 唯一的例外：低于净空的当前格 -> 净空不减的空闲邻格，且累计步数未超上限。
        if (!current_below_clearance ||
          static_cast<int>(escape_steps[linearIndex(grid, current_idx)]) >=
          params_.start_escape_max_steps ||
          !isGridCellFree(grid, next.x, next.y, policy) ||
          blocked_squared[linearIndex(grid, next)] + 1e-9 <
          blocked_squared[linearIndex(grid, current_idx)])
        {
          continue;
        }
        escape_edge = true;
      }
      if (params_.allow_diagonal && dx != 0 && dy != 0) {
        // Avoid diagonal corner cutting through two blocked cells. The later
        // footprint checker still verifies body clearance, but the graph search
        // should not create topologically invalid shortcuts.
        // 脱出段上两个正交邻格只要求空闲（不能夹着阻塞格斜穿），不要求达标。
        const GridIndex side_a {current.x + dx, current.y};
        const GridIndex side_b {current.x, current.y + dy};
        const bool corner_ok = current_below_clearance ?
          (isGridCellFree(grid, side_a.x, side_a.y, policy) &&
          isGridCellFree(grid, side_b.x, side_b.y, policy)) :
          (isTraversable(grid, blocked_squared, side_a) &&
          isTraversable(grid, blocked_squared, side_b));
        if (!corner_ok) {
          continue;
        }
      }

      const std::size_t next_linear = linearIndex(grid, next);
      if (closed[next_linear]) {
        continue;
      }

      const double step_cost =
        ((dx != 0 && dy != 0) ? params_.diagonal_cost : 1.0) *
        clearanceCostFactor(grid, blocked_squared, next);
      const double tentative_g = g_score[current_linear] + step_cost;
      if (tentative_g + 1e-9 >= g_score[next_linear]) {
        continue;
      }

      parent[next_linear] = static_cast<int>(current_linear);
      g_score[next_linear] = tentative_g;
      if (start_escape) {
        escape_steps[next_linear] = escape_edge ?
          static_cast<uint16_t>(escape_steps[current_linear] + 1U) : static_cast<uint16_t>(0U);
      }
      open.push(QueueNode {
          next.x,
          next.y,
          tentative_g + heuristic(next, goal_idx),
          tentative_g});
    }
  }

  if (!result.success) {
    result.reason = "no path";
    return result;
  }

  std::vector<GridIndex> reversed;
  for (int cursor = static_cast<int>(goal_linear); cursor >= 0; cursor = parent[cursor]) {
    const int width = static_cast<int>(grid.info.width);
    reversed.push_back(GridIndex {cursor % width, cursor / width});
    if (static_cast<std::size_t>(cursor) == start_linear) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());

  result.path.poses.reserve(std::max<std::size_t>(2U, reversed.size()));
  for (const auto & index : reversed) {
    auto pose = gridToPose(grid, index);
    pose.header = result.path.header;
    result.path.poses.push_back(pose);
  }

  // Search runs on cell indices, but MINCO and the footprint checker must start
  // and end at the requested continuous poses. Returning cell centres here can
  // displace a 0.4 m RC-ESDF path by 0.2 m before any trajectory is generated.
  auto exact_start = start;
  exact_start.header = result.path.header;
  auto exact_goal = goal;
  exact_goal.header = result.path.header;
  if (result.path.poses.size() == 1U) {
    result.path.poses.front() = exact_start;
    if (std::hypot(
        exact_goal.pose.position.x - exact_start.pose.position.x,
        exact_goal.pose.position.y - exact_start.pose.position.y) > 1e-6)
    {
      result.path.poses.push_back(exact_goal);
    }
  } else {
    result.path.poses.front() = exact_start;
    result.path.poses.back() = exact_goal;
  }

  for (std::size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto & a = result.path.poses[i - 1].pose.position;
    const auto & b = result.path.poses[i].pose.position;
    result.length += std::hypot(b.x - a.x, b.y - a.y);
  }
  result.reason = "ok";
  return result;
}

bool GridAstar::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double wx,
  double wy,
  GridIndex & out) const
{
  const double gx = (wx - grid.info.origin.position.x) / grid.info.resolution;
  const double gy = (wy - grid.info.origin.position.y) / grid.info.resolution;
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

geometry_msgs::msg::PoseStamped GridAstar::gridToPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = grid.header;
  pose.pose.position.x =
    grid.info.origin.position.x + (static_cast<double>(index.x) + 0.5) * grid.info.resolution;
  pose.pose.position.y =
    grid.info.origin.position.y + (static_cast<double>(index.y) + 0.5) * grid.info.resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool GridAstar::isTraversable(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  return hasGridClearance(grid, index.x, index.y, params_.safe_distance, occupancyPolicy());
}

bool GridAstar::isTraversable(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & blocked_squared,
  const GridIndex & index) const
{
  if (!isGridCellFree(grid, index.x, index.y, occupancyPolicy())) {
    return false;
  }
  if (!(params_.safe_distance > 0.0)) {
    return true;
  }
  const double radius_cells = params_.safe_distance / grid.info.resolution;
  return blocked_squared[linearIndex(grid, index)] > radius_cells * radius_cells;
}

double GridAstar::clearanceCostFactor(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & blocked_squared,
  const GridIndex & index) const
{
  const double range = params_.clearance_cost_distance;
  if (!(params_.clearance_cost_weight > 0.0) || !(range > 0.0)) {
    return 1.0;
  }
  const double distance =
    std::sqrt(blocked_squared[linearIndex(grid, index)]) * grid.info.resolution;
  if (!(distance < range)) {
    return 1.0;
  }
  const double deficit = (range - distance) / range;
  return 1.0 + params_.clearance_cost_weight * deficit * deficit;
}

std::size_t GridAstar::linearIndex(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  return static_cast<std::size_t>(index.y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(index.x);
}

double GridAstar::heuristic(const GridIndex & from, const GridIndex & to) const
{
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return params_.allow_diagonal ? std::hypot(dx, dy) : (std::abs(dx) + std::abs(dy));
}

}  // namespace minco_planner
