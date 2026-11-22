// Copyright 2026

#ifndef MINCO_PLANNER__PLANNING__LOCAL_ROUTE_HPP_
#define MINCO_PLANNER__PLANNING__LOCAL_ROUTE_HPP_

#include "minco_planner/planning/grid_search.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"

namespace minco_planner
{

struct RouteObstruction
{
  std::size_t first;
  std::size_t last;
};

struct LocalRouteParams
{
  double anchor_offset = 1.0;
  double bypass_padding = 2.0;
  double max_local_length = 7.0;
};

struct LocalRouteResult
{
  GridSearchResult route;
  std::optional<RouteObstruction> obstruction;
  std::optional<geometry_msgs::msg::Point> rejoin;
  double bypass_length = 0.0;
};

// A geometric trace used for collision checks and early tunnel detection.
ReferenceTrajectory traceGuide(const nav_msgs::msg::Path & path);

nav_msgs::msg::Path remainingGuide(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & start,
  double max_attachment_distance);

std::optional<RouteObstruction> obstructedGuide(
  const ReferenceTrajectory & guide, const nav_msgs::msg::OccupancyGrid & grid,
  const FootprintSafetyChecker & checker,
  const nav_msgs::msg::OccupancyGrid * accepted_grid = nullptr);

LocalRouteResult rejoinGlobalRoute(
  const nav_msgs::msg::Path & remaining, const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<double> & blocked_squared, double clearance,
  const FootprintSafetyChecker & checker, GridSearch & search,
  const std::optional<geometry_msgs::msg::Point> & reference_conflict = std::nullopt,
  const nav_msgs::msg::OccupancyGrid * accepted_grid = nullptr,
  const LocalRouteParams & params = LocalRouteParams{});

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING__LOCAL_ROUTE_HPP_
