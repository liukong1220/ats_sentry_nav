// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_

#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"

namespace trajectory_optimizer
{
class RcTraversabilityEsdfProvider;
}

namespace minco_planner
{

struct MincoTrajectoryOptimizerParams
{
  double reference_speed = 1.5;
  double min_segment_time = 0.05;
  double sample_spacing = 0.12;
  double max_velocity = 2.0;
  double max_acceleration = 2.5;
  int max_time_scaling_iterations = 5;
  double time_scaling_factor = 1.25;

  // Keep MINCO's interpolation from cutting into obstacles between JPS nodes.
  // The correction moves only inner control points and re-solves MINCO after
  // each update; endpoints and the JPS route topology remain fixed.
  bool esdf_obstacle_optimization_enabled = true;
  double esdf_obstacle_clearance = 0.45;
  int esdf_obstacle_max_iterations = 6;
  double esdf_obstacle_control_point_spacing = 0.30;
  double esdf_obstacle_max_step = 0.10;
  double esdf_obstacle_max_deviation = 0.50;
};

class MincoTrajectoryOptimizer
{
public:
  explicit MincoTrajectoryOptimizer(
    MincoTrajectoryOptimizerParams params = MincoTrajectoryOptimizerParams());

  void setParams(const MincoTrajectoryOptimizerParams & params);
  bool esdfObstacleOptimizationEnabled() const;
  ReferenceTrajectory optimize(
    const nav_msgs::msg::Path & raw_path,
    const trajectory_optimizer::RcTraversabilityEsdfProvider * esdf = nullptr) const;

private:
  MincoTrajectoryOptimizerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
