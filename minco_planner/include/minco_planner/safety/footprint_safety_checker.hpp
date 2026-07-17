// Copyright 2026

#ifndef MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_

#include <vector>

#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

struct FootprintSafetyParams
{
  double length = 0.70;
  double width = 0.55;
  double safety_margin = 0.05;
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  // The maximum displacement of the farthest rectangular corner between two
  // swept samples, expressed in planning-grid cells.
  double swept_max_corner_step_cells = 0.5;
};

struct CollisionSample
{
  std::size_t trajectory_index = 0;
  std::size_t segment_index = 0;
  double segment_fraction = 0.0;
  bool swept = false;
  double x = 0.0;
  double y = 0.0;
};

struct FootprintSafetyResult
{
  bool safe = true;
  std::vector<CollisionSample> collisions;
  std::size_t discrete_samples_checked = 0;
  std::size_t swept_samples_checked = 0;
  std::size_t swept_segments_checked = 0;
};

class FootprintSafetyChecker
{
public:
  explicit FootprintSafetyChecker(FootprintSafetyParams params = FootprintSafetyParams());

  void setParams(const FootprintSafetyParams & params);
  FootprintSafetyResult check(
    const ReferenceTrajectory & trajectory,
    const nav_msgs::msg::OccupancyGrid & grid) const;

private:
  struct GridIndex
  {
    int x = 0;
    int y = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx,
    double wy,
    GridIndex & out) const;
  bool isOccupied(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  bool sampleFootprintOccupied(
    const ReferencePoint & point,
    const nav_msgs::msg::OccupancyGrid & grid,
    double & collision_x,
    double & collision_y) const;
  std::size_t sweptSubdivisions(
    const ReferencePoint & start, const ReferencePoint & end,
    const nav_msgs::msg::OccupancyGrid & grid) const;
  static ReferencePoint interpolate(
    const ReferencePoint & start, const ReferencePoint & end, double fraction);
  static double normalizeAngle(double angle);

  FootprintSafetyParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_
