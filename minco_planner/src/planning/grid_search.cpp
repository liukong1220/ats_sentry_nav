// Copyright 2026

#include "minco_planner/planning/grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{
namespace
{

struct Index
{
  int x;
  int y;
};

bool operator==(Index a, Index b)
{
  return a.x == b.x && a.y == b.y;
}

int sign(int value)
{
  return (value > 0) - (value < 0);
}

constexpr Index kNeighbors8[] = {
  {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
constexpr Index kNeighbors4[] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

}  // namespace

GridSearch::GridSearch(GridSearchParams params)
: params_(params)
{
}

GridSearchResult GridSearch::plan(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<double> * blocked_squared,
  const GridSearchOptions & options)
{
  GridSearchResult result;
  result.path.header = grid.header;
  const std::size_t cell_count = static_cast<std::size_t>(grid.info.width) * grid.info.height;
  const double resolution = grid.info.resolution;
  if (cell_count == 0 || cell_count > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    grid.data.size() != cell_count || !(resolution > 0.0) || !std::isfinite(resolution))
  {
    result.reason = "grid is empty";
    return result;
  }
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const auto inside = [width, height](Index index) {
      return index.x >= 0 && index.y >= 0 && index.x < width && index.y < height;
    };
  const auto linear = [width](Index index) {return index.y * width + index.x;};
  const auto from_linear = [width](int id) {return Index{id % width, id / width};};
  const auto & origin = grid.info.origin;
  const double yaw = tf2::getYaw(origin.orientation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const auto cell_center = [&](Index index) {
      geometry_msgs::msg::Point point;
      const double x = (static_cast<double>(index.x) + 0.5) * resolution;
      const double y = (static_cast<double>(index.y) + 0.5) * resolution;
      point.x = origin.position.x + (cosine * x - sine * y);
      point.y = origin.position.y + (sine * x + cosine * y);
      return point;
    };
  const auto in_search = [&](Index index) {
      if (!inside(index)) {
        return false;
      }
      if (!options.bounds) {
        return true;
      }
      const auto p = cell_center(index);
      const auto & b = *options.bounds;
      return p.x >= b.min_x && p.x <= b.max_x && p.y >= b.min_y && p.y <= b.max_y;
    };
  const auto local_coordinates = [&](const geometry_msgs::msg::Point & point) {
      const double dx = point.x - origin.position.x;
      const double dy = point.y - origin.position.y;
      return std::make_pair((cosine * dx + sine * dy) / resolution,
        (-sine * dx + cosine * dy) / resolution);
    };
  const auto world_to_grid = [&](const geometry_msgs::msg::Point & point, Index & index) {
      const auto [x, y] = local_coordinates(point);
      if (!(x >= 0.0 && x < width && y >= 0.0 && y < height)) {
        return false;
      }
      index = {static_cast<int>(x), static_cast<int>(y)};
      return true;
    };
  Index start_index, goal_index{-1, -1};
  if (!world_to_grid(start.pose.position, start_index) || !in_search(start_index)) {
    result.reason = "start is outside grid";
    return result;
  }
  const auto [goal_x, goal_y] = local_coordinates(goal.pose.position);
  const double heuristic_x = std::floor(goal_x);
  const double heuristic_y = std::floor(goal_y);
  const bool allow_partial = options.can_stop && params_.unknown_is_obstacle &&
    std::isfinite(goal_x) && std::isfinite(goal_y) &&
    grid.data[linear(start_index)] >= 0 &&
    grid.data[linear(start_index)] < params_.obstacle_value_threshold;
  bool has_goal = world_to_grid(goal.pose.position, goal_index) && in_search(goal_index);
  if (!has_goal && !allow_partial) {
    result.reason = "goal is outside grid";
    return result;
  }

  const GridOccupancyPolicy policy{params_.obstacle_value_threshold, params_.unknown_is_obstacle};
  std::vector<double> owned_field;
  if (!blocked_squared || blocked_squared->size() != cell_count) {
    owned_field = computeBlockedSquaredDistanceCells(grid, policy);
    blocked_squared = &owned_field;
  }
  const auto & field = *blocked_squared;
  const auto free = [&](Index index) {
      return in_search(index) && isGridCellFree(grid, index.x, index.y, policy);
    };
  result.clearance = options.clearance >= 0.0 &&
    std::abs(options.clearance - params_.safe_distance) > 1e-9 ?
    options.clearance : params_.safe_distance;
  const auto radius_squared = [resolution](double clearance) {
      const double radius = std::max(0.0, clearance) / resolution;
      return radius * radius;
    };
  double minimum_squared = radius_squared(result.clearance);
  const auto traversable = [&](Index index) {
      return free(index) && (minimum_squared == 0.0 || field[linear(index)] > minimum_squared);
    };
  if ((options.strict_endpoints || !params_.assume_start_traversable) && !traversable(start_index)) {
    result.reason = "start is occupied";
    return result;
  }
  const bool relaxed_goal = has_goal && !traversable(goal_index);
  std::string failure = has_goal ? "no path" : "goal is outside grid";
  if (relaxed_goal) {
    const double endpoint_minimum = options.endpoint_minimum >= 0.0 ?
      options.endpoint_minimum : params_.min_safe_distance;
    const double goal_clearance =
      !options.strict_endpoints && params_.relax_endpoint_clearance && free(goal_index) ?
      std::nextafter(std::sqrt(field[linear(goal_index)]) * resolution, 0.0) : 0.0;
    failure = "goal is occupied";
    if (goal_clearance > 0.0 && goal_clearance + 1e-9 >= endpoint_minimum) {
      result.clearance = std::min(result.clearance, goal_clearance);
      minimum_squared = radius_squared(result.clearance);
    }
    if (!traversable(goal_index)) {
      has_goal = false;
      if (!allow_partial) {
        result.reason = failure;
        return result;
      }
    }
  }

  if (width_ != width || height_ != height) {
    width_ = width;
    height_ = height;
    cells_.assign(cell_count, Cell{});
  }
  if (++generation_ == 0) {
    std::fill(cells_.begin(), cells_.end(), Cell{});
    ++generation_;
  }

  const bool jump = params_.jump_enabled && params_.allow_diagonal &&
    params_.diagonal_cost > 1.0 && params_.diagonal_cost < 2.0;
  const double range = params_.clearance_cost_distance;
  const double weight = params_.clearance_cost_weight;
  const bool weighted = weight > 0.0 && range > 0.0;
  const auto cost_factor = [&](int id) {
      if (weighted) {
        const double distance = std::sqrt(field[id]) * resolution;
        if (distance < range) {
          const double deficit = (range - distance) / range;
          return 1.0 + weight * deficit * deficit;
        }
      }
      return 1.0;
    };
  const auto uniform = [&](Index index) {
      if (!in_search(index)) {
        return false;
      }
      const int id = linear(index);
      return grid.data[id] >= 0 && grid.data[id] < policy.obstacle_value_threshold &&
             (minimum_squared == 0.0 || field[id] > minimum_squared) &&
             (!weighted || std::sqrt(field[id]) * resolution >= range);
    };
  const auto uniform_neighborhood = [&](Index index) {
      auto & cell = cells_[linear(index)];
      if (cell.neighborhood_checked != generation_) {
        cell.uniform_neighborhood = uniform(index) && std::all_of(
          std::begin(kNeighbors8), std::end(kNeighbors8), [&](Index offset) {
            return uniform({index.x + offset.x, index.y + offset.y});
          });
        cell.neighborhood_checked = generation_;
      }
      return cell.uniform_neighborhood;
    };
  const auto heuristic = [&](Index index) {
      const double dx = std::abs(static_cast<double>(index.x) - heuristic_x);
      const double dy = std::abs(static_cast<double>(index.y) - heuristic_y);
      if (jump) {
        return std::max(dx, dy) + (params_.diagonal_cost - 1.0) * std::min(dx, dy);
      }
      return params_.allow_diagonal ? std::hypot(dx, dy) : dx + dy;
    };

  const bool start_escape = params_.start_escape_max_steps > 0 && !traversable(start_index);
  const int start_id = linear(start_index);
  auto & first = cells_[start_id];
  first.visited = generation_;
  first.parent = -1;
  first.g = 0.0;
  first.escape_steps = 0;
  open_buffer_.clear();
  const auto compare_open = [jump](const OpenNode & a, const OpenNode & b) {
      return jump && a.f == b.f ? a.g < b.g : a.f > b.f;
    };
  open_buffer_.push_back({start_index.x, start_index.y, heuristic(start_index), 0.0});
  int best_stop = -1;
  double best_remaining = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::PoseStamped stop_pose;

  while (!open_buffer_.empty()) {
    std::pop_heap(open_buffer_.begin(), open_buffer_.end(), compare_open);
    const OpenNode top = open_buffer_.back();
    open_buffer_.pop_back();
    const Index current{top.x, top.y};
    const int current_id = linear(current);
    auto & cell = cells_[current_id];
    if (cell.closed == generation_) {
      continue;
    }
    cell.closed = generation_;
    ++result.expanded_nodes;
    if (has_goal && current == goal_index) {
      result.success = true;
      break;
    }
    if (allow_partial && current_id != start_id && traversable(current)) {
      const double remaining = heuristic(current);
      if (best_stop < 0 || remaining < best_remaining ||
        (remaining == best_remaining && cell.g < cells_[best_stop].g))
      {
        const auto parent = from_linear(cell.parent);
        const double terminal_yaw = yaw + std::atan2(current.y - parent.y, current.x - parent.x);
        geometry_msgs::msg::PoseStamped candidate;
        candidate.header = grid.header;
        candidate.pose.position = cell_center(current);
        candidate.pose.orientation.z = std::sin(terminal_yaw * 0.5);
        candidate.pose.orientation.w = std::cos(terminal_yaw * 0.5);
        if (options.can_stop(candidate)) {
          stop_pose = candidate;
          best_stop = current_id;
          best_remaining = remaining;
        }
      }
    }
    if (jump && params_.max_expanded_nodes > 0 &&
      result.expanded_nodes >= params_.max_expanded_nodes)
    {
      failure = "expanded node limit";
      break;
    }

    const bool below_clearance = start_escape && !traversable(current);
    const bool jump_from_current = jump && uniform(current);
    const bool prune = jump_from_current && cell.parent >= 0 && uniform_neighborhood(current);
    const Index previous = prune ? from_linear(cell.parent) : current;
    const int incoming_x = sign(current.x - previous.x);
    const int incoming_y = sign(current.y - previous.y);
    const auto * neighbors = params_.allow_diagonal ? kNeighbors8 : kNeighbors4;
    const int neighbor_count = params_.allow_diagonal ? 8 : 4;
    for (int i = 0; i < neighbor_count; ++i) {
      const int dx = neighbors[i].x;
      const int dy = neighbors[i].y;
      // Forced turns and weighted transitions lie on the fringe, where all directions
      // remain available. Inside a uniform neighborhood, diagonal-first paths dominate.
      if (prune && ((dx != 0 && dx != incoming_x) || (dy != 0 && dy != incoming_y))) {
        continue;
      }
      Index next{current.x + dx, current.y + dy};
      if (!in_search(next)) {
        continue;
      }
      bool escape_edge = false;
      if (!traversable(next)) {
        if (!below_clearance || cell.escape_steps >= params_.start_escape_max_steps ||
          !free(next) || field[linear(next)] + 1e-9 < field[current_id])
        {
          continue;
        }
        escape_edge = true;
      }
      const bool diagonal = dx != 0 && dy != 0;
      if (diagonal) {
        const Index side_a{current.x + dx, current.y};
        const Index side_b{current.x, current.y + dy};
        if (below_clearance ? !free(side_a) || !free(side_b) :
          !traversable(side_a) || !traversable(side_b))
        {
          continue;
        }
      }

      int steps = 1;
      // In the interior both straight components of a diagonal can reach a
      // frontier jump point, so each diagonal landing is itself a jump point.
      // Cardinal rays contract all intervening uniform cells in one edge.
      if (jump_from_current && !diagonal) {
        while (!(has_goal && next == goal_index) && uniform_neighborhood(next)) {
          next.x += dx;
          next.y += dy;
          ++steps;
        }
      }
      const int next_id = linear(next);
      auto & successor = cells_[next_id];
      if (successor.closed == generation_) {
        continue;
      }
      const double step_cost = (diagonal ? params_.diagonal_cost : 1.0) * cost_factor(next_id);
      const double tentative_g = cell.g + steps * step_cost;
      if (successor.visited == generation_ && tentative_g + 1e-9 >= successor.g) {
        continue;
      }
      successor.visited = generation_;
      successor.parent = current_id;
      successor.g = tentative_g;
      successor.escape_steps = escape_edge ? cell.escape_steps + 1 : 0;
      open_buffer_.push_back({next.x, next.y, tentative_g + heuristic(next), tentative_g});
      std::push_heap(open_buffer_.begin(), open_buffer_.end(), compare_open);
    }
  }

  if (!result.success && best_stop < 0) {
    result.reason = failure;
    return result;
  }
  const int terminal_id = result.success ? linear(goal_index) : best_stop;
  result.partial = !result.success;
  result.cost = cells_[terminal_id].g;

  std::vector<Index> reversed;
  for (int id = terminal_id; id != start_id; id = cells_[id].parent) {
    const auto parent = from_linear(cells_[id].parent);
    auto index = from_linear(id);
    const int dx = sign(index.x - parent.x);
    const int dy = sign(index.y - parent.y);
    // MINCO and the local rejoin logic receive every cell, including those skipped by jumps.
    while (!(index == parent)) {
      reversed.push_back(index);
      index.x -= dx;
      index.y -= dy;
    }
  }
  reversed.push_back(start_index);
  result.path.poses.reserve(std::max<std::size_t>(2U, reversed.size()));
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.path.header;
    pose.pose.position = cell_center(*it);
    pose.pose.orientation.w = 1.0;
    result.path.poses.push_back(pose);
  }
  auto exact_start = start;
  exact_start.header = result.path.header;
  auto exact_goal = result.partial ? stop_pose : goal;
  exact_goal.header = result.path.header;
  result.path.poses.front() = exact_start;
  if (result.path.poses.size() > 1U) {
    result.path.poses.back() = exact_goal;
  } else if (std::hypot(
      exact_goal.pose.position.x - exact_start.pose.position.x,
      exact_goal.pose.position.y - exact_start.pose.position.y) > 1e-6)
  {
    result.path.poses.push_back(exact_goal);
  }
  for (std::size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto & a = result.path.poses[i - 1].pose.position;
    const auto & b = result.path.poses[i].pose.position;
    result.length += std::hypot(b.x - a.x, b.y - a.y);
  }
  result.reason = result.partial ? "partial: " + failure : "ok";
  return result;
}

}  // namespace minco_planner
