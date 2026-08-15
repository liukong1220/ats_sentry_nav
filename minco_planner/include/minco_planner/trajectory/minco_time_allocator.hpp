// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__MINCO_TIME_ALLOCATOR_HPP_
#define MINCO_PLANNER__TRAJECTORY__MINCO_TIME_ALLOCATOR_HPP_

#include <vector>

#include <Eigen/Core>

namespace minco_planner
{

struct MincoTimeAllocatorParams
{
  double reference_speed = 1.5;
  double max_velocity = 2.0;
  double max_acceleration = 2.5;
  double max_lateral_acceleration = 1.5;
  double min_segment_time = 0.05;
};

struct MincoTimeAllocation
{
  Eigen::VectorXd durations;
  std::vector<double> waypoint_speeds;
  std::vector<double> signed_curvatures;
  bool valid = false;
};

class MincoTimeAllocator
{
public:
  explicit MincoTimeAllocator(MincoTimeAllocatorParams params = MincoTimeAllocatorParams());

  void setParams(const MincoTimeAllocatorParams & params);
  MincoTimeAllocation allocate(
    const std::vector<Eigen::Vector2d> & waypoints,
    double initial_speed = 0.0) const;
  bool applyLocalDynamicScaling(
    Eigen::VectorXd & durations,
    const std::vector<double> & segment_peak_velocities,
    const std::vector<double> & segment_peak_accelerations,
    const std::vector<double> & segment_peak_jerks,
    double max_velocity,
    double max_acceleration,
    double max_jerk) const;

private:
  MincoTimeAllocatorParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__MINCO_TIME_ALLOCATOR_HPP_
