// Copyright 2026

#include "minco_planner/planning/grid_jps.hpp"

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
    if (std::abs(lhs.f - rhs.f) <= 1e-9) {
      return lhs.g < rhs.g;
    }
    return lhs.f > rhs.f;
  }
};

int sign(int value)
{
  return (value > 0) - (value < 0);
}

}  // namespace

GridJps::GridJps(GridJpsParams params)
: params_(params)
{
}

void GridJps::setParams(const GridJpsParams & params)
{
  params_ = params;
}

GridAstarResult GridJps::plan(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return planWithClearance(grid, start, goal, -1.0);
}

GridOccupancyPolicy GridJps::occupancyPolicy() const
{
  GridOccupancyPolicy policy;
  policy.obstacle_value_threshold = params_.obstacle_value_threshold;
  policy.unknown_is_obstacle = params_.unknown_is_obstacle;
  return policy;
}

GridAstarResult GridJps::planWithClearance(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double clearance_override) const
{
  if (clearance_override >= 0.0 &&
    std::abs(clearance_override - params_.safe_distance) > 1e-9)
  {
    GridJpsParams relaxed = params_;
    relaxed.safe_distance = clearance_override;
    return GridJps(relaxed).planWithClearance(grid, start, goal, -1.0);
  }

  GridAstarResult result;
  result.path.header = grid.header;
  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 ||
    grid.data.size() < cell_count)
  {
    result.reason = "invalid grid";
    return result;
  }

  GridIndex start_index;
  GridIndex goal_index;
  if (!worldToGrid(grid, start.pose.position.x, start.pose.position.y, start_index)) {
    result.reason = "start outside grid";
    return result;
  }
  if (!worldToGrid(grid, goal.pose.position.x, goal.pose.position.y, goal_index)) {
    result.reason = "goal outside grid";
    return result;
  }
  // The robot already stands on the start cell, so a fail-closed rejection here
  // deadlocks: the only thing that could move it is the planner that refuses to
  // run. The yaw-aware footprint gate and local repair stay authoritative.
  if (!params_.assume_start_traversable && !isTraversable(grid, start_index.x, start_index.y)) {
    result.reason = "start occupied";
    return result;
  }
  if (!isTraversable(grid, goal_index.x, goal_index.y)) {
    const double goal_clearance = params_.relax_endpoint_clearance
      ? measureGridClearance(
      grid, goal_index.x, goal_index.y, params_.safe_distance, occupancyPolicy())
      : 0.0;
    if (!(goal_clearance > 0.0) || goal_clearance + 1e-9 < params_.min_safe_distance) {
      result.reason = "goal occupied";
      return result;
    }
    GridJpsParams relaxed = params_;
    relaxed.safe_distance = goal_clearance;
    relaxed.relax_endpoint_clearance = false;
    GridAstarResult relaxed_result =
      GridJps(relaxed).planWithClearance(grid, start, goal, -1.0);
    if (relaxed_result.success) {
      return relaxed_result;
    }
    result.reason = "goal occupied";
    return result;
  }

  const std::size_t start_linear = linearIndex(grid, start_index.x, start_index.y);
  const std::size_t goal_linear = linearIndex(grid, goal_index.x, goal_index.y);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open;
  auto heuristic = [&goal_index](int x, int y) {
      return std::hypot(
        static_cast<double>(goal_index.x - x),
        static_cast<double>(goal_index.y - y));
    };

  g_score[start_linear] = 0.0;
  open.push(QueueNode {start_index.x, start_index.y, heuristic(start_index.x, start_index.y), 0.0});
  while (!open.empty()) {
    const QueueNode current_node = open.top();
    open.pop();
    const std::size_t current_linear = linearIndex(grid, current_node.x, current_node.y);
    if (closed[current_linear] || current_node.g > g_score[current_linear] + 1e-9) {
      continue;
    }
    closed[current_linear] = true;
    ++result.expanded_nodes;
    if (current_linear == goal_linear) {
      result.success = true;
      break;
    }
    if (params_.max_expanded_nodes > 0 && result.expanded_nodes >= params_.max_expanded_nodes) {
      result.reason = "expansion limit reached";
      return result;
    }

    Direction incoming;
    if (parent[current_linear] >= 0) {
      const int width = static_cast<int>(grid.info.width);
      incoming.dx = sign(current_node.x - parent[current_linear] % width);
      incoming.dy = sign(current_node.y - parent[current_linear] / width);
    }
    const GridIndex current {current_node.x, current_node.y};
    for (const auto & direction : prunedDirections(grid, current, incoming)) {
      GridIndex successor;
      if (!jump(grid, current.x, current.y, direction.dx, direction.dy, goal_index, successor)) {
        continue;
      }
      const std::size_t successor_linear = linearIndex(grid, successor.x, successor.y);
      if (closed[successor_linear]) {
        continue;
      }
      const double step_cost = std::hypot(
        static_cast<double>(successor.x - current.x),
        static_cast<double>(successor.y - current.y));
      const double tentative_g = g_score[current_linear] + step_cost;
      if (tentative_g + 1e-9 >= g_score[successor_linear]) {
        continue;
      }
      parent[successor_linear] = static_cast<int>(current_linear);
      g_score[successor_linear] = tentative_g;
      open.push(QueueNode {
          successor.x, successor.y,
          tentative_g + heuristic(successor.x, successor.y), tentative_g});
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
  if (reversed.empty() || reversed.back().x != start_index.x || reversed.back().y != start_index.y) {
    result.success = false;
    result.reason = "broken parent chain";
    return result;
  }
  std::reverse(reversed.begin(), reversed.end());
  result.path.poses.reserve(std::max<std::size_t>(2U, reversed.size()));
  for (const auto & index : reversed) {
    auto pose = gridToPose(grid, index);
    pose.header = result.path.header;
    result.path.poses.push_back(pose);
  }

  // JPS uses the cells only for graph connectivity. Preserve the true poses at
  // both ends so a coarse RC-ESDF cannot shift the robot or target by half a cell.
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
    const auto & previous = result.path.poses[i - 1].pose.position;
    const auto & current = result.path.poses[i].pose.position;
    result.length += std::hypot(
      current.x - previous.x,
      current.y - previous.y);
  }
  result.reason = "ok";
  return result;
}

bool GridJps::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double wx,
  double wy,
  GridIndex & out) const
{
  const double gx = (wx - grid.info.origin.position.x) / grid.info.resolution;
  const double gy = (wy - grid.info.origin.position.y) / grid.info.resolution;
  if (gx < 0.0 || gy < 0.0 || gx >= grid.info.width || gy >= grid.info.height) {
    return false;
  }
  out.x = static_cast<int>(std::floor(gx));
  out.y = static_cast<int>(std::floor(gy));
  return true;
}

geometry_msgs::msg::PoseStamped GridJps::gridToPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = grid.header;
  pose.pose.position.x =
    grid.info.origin.position.x + (static_cast<double>(index.x) + 0.5) * grid.info.resolution;
  pose.pose.position.y =
    grid.info.origin.position.y + (static_cast<double>(index.y) + 0.5) * grid.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool GridJps::isTraversable(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
{
  return hasGridClearance(grid, x, y, params_.safe_distance, occupancyPolicy());
}

bool GridJps::isCellFree(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
{
  return isGridCellFree(grid, x, y, occupancyPolicy());
}

std::size_t GridJps::linearIndex(
  const nav_msgs::msg::OccupancyGrid & grid,
  int x,
  int y) const
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(x);
}

bool GridJps::hasForcedNeighbor(
  const nav_msgs::msg::OccupancyGrid & grid,
  int x,
  int y,
  int dx,
  int dy) const
{
  if (dx != 0 && dy != 0) {
    return
      (!isTraversable(grid, x - dx, y) && isTraversable(grid, x - dx, y + dy)) ||
      (!isTraversable(grid, x, y - dy) && isTraversable(grid, x + dx, y - dy));
  }
  if (dx != 0) {
    return
      (!isTraversable(grid, x, y + 1) && isTraversable(grid, x + dx, y + 1)) ||
      (!isTraversable(grid, x, y - 1) && isTraversable(grid, x + dx, y - 1));
  }
  return
    (!isTraversable(grid, x + 1, y) && isTraversable(grid, x + 1, y + dy)) ||
    (!isTraversable(grid, x - 1, y) && isTraversable(grid, x - 1, y + dy));
}

bool GridJps::jump(
  const nav_msgs::msg::OccupancyGrid & grid,
  int x,
  int y,
  int dx,
  int dy,
  const GridIndex & goal,
  GridIndex & jump_point) const
{
  const int next_x = x + dx;
  const int next_y = y + dy;
  if (!isTraversable(grid, next_x, next_y)) {
    return false;
  }
  if (next_x == goal.x && next_y == goal.y) {
    jump_point = GridIndex {next_x, next_y};
    return true;
  }
  if (hasForcedNeighbor(grid, next_x, next_y, dx, dy)) {
    jump_point = GridIndex {next_x, next_y};
    return true;
  }
  if (dx != 0 && dy != 0) {
    GridIndex ignored;
    if (jump(grid, next_x, next_y, dx, 0, goal, ignored) ||
      jump(grid, next_x, next_y, 0, dy, goal, ignored))
    {
      jump_point = GridIndex {next_x, next_y};
      return true;
    }
  }
  return jump(grid, next_x, next_y, dx, dy, goal, jump_point);
}

std::vector<GridJps::Direction> GridJps::prunedDirections(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & current,
  const Direction & incoming) const
{
  std::vector<Direction> directions;
  auto add = [this, &directions](int dx, int dy) {
      if (dx == 0 && dy == 0) {
        return;
      }
      if (!params_.allow_diagonal && dx != 0 && dy != 0) {
        return;
      }
      const auto duplicate = std::find_if(
        directions.begin(), directions.end(), [dx, dy](const Direction & candidate) {
          return candidate.dx == dx && candidate.dy == dy;
        });
      if (duplicate == directions.end()) {
        directions.push_back(Direction {dx, dy});
      }
    };

  const int dx = sign(incoming.dx);
  const int dy = sign(incoming.dy);
  if (dx == 0 && dy == 0) {
    for (int nx = -1; nx <= 1; ++nx) {
      for (int ny = -1; ny <= 1; ++ny) {
        add(nx, ny);
      }
    }
    return directions;
  }
  if (dx != 0 && dy != 0) {
    add(dx, dy);
    add(dx, 0);
    add(0, dy);
    if (!isTraversable(grid, current.x - dx, current.y) &&
      isTraversable(grid, current.x - dx, current.y + dy))
    {
      add(-dx, dy);
    }
    if (!isTraversable(grid, current.x, current.y - dy) &&
      isTraversable(grid, current.x + dx, current.y - dy))
    {
      add(dx, -dy);
    }
  } else if (dx != 0) {
    add(dx, 0);
    if (!isTraversable(grid, current.x, current.y + 1) &&
      isTraversable(grid, current.x + dx, current.y + 1))
    {
      add(dx, 1);
    }
    if (!isTraversable(grid, current.x, current.y - 1) &&
      isTraversable(grid, current.x + dx, current.y - 1))
    {
      add(dx, -1);
    }
  } else {
    add(0, dy);
    if (!isTraversable(grid, current.x + 1, current.y) &&
      isTraversable(grid, current.x + 1, current.y + dy))
    {
      add(1, dy);
    }
    if (!isTraversable(grid, current.x - 1, current.y) &&
      isTraversable(grid, current.x - 1, current.y + dy))
    {
      add(-1, dy);
    }
  }
  return directions;
}

}  // namespace minco_planner
