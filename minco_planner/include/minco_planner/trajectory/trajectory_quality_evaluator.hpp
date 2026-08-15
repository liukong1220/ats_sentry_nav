// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__TRAJECTORY_QUALITY_EVALUATOR_HPP_
#define MINCO_PLANNER__TRAJECTORY__TRAJECTORY_QUALITY_EVALUATOR_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace ats_rc_esdf
{
class RcTraversabilityEsdfProvider;
}

namespace minco_planner
{

struct TrajectoryQualityMetrics
{
  double path_length = 0.0;
  double direct_length = 0.0;
  double length_ratio = 1.0;
  double max_lateral_deviation = 0.0;
  double max_geometric_curvature = 0.0;
  double p95_geometric_curvature = 0.0;
  double total_turning_angle = 0.0;
  double curvature_total_variation = 0.0;
  std::size_t curvature_sign_changes = 0;
  double minimum_center_clearance = 0.0;
  double minimum_footprint_clearance = 0.0;
  std::size_t footprint_collision_count = 0;
  std::size_t swept_collision_count = 0;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_jerk = 0.0;
  bool finite = false;
  bool strictly_monotonic_time = false;
  std::vector<double> segment_durations;
};

class TrajectoryQualityEvaluator
{
public:
  TrajectoryQualityMetrics evaluate(
    const ReferenceTrajectory & trajectory,
    const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf = nullptr,
    const std::vector<Eigen::Vector2d> & footprint_samples = std::vector<Eigen::Vector2d>(),
    const std::vector<double> & segment_durations = std::vector<double>()) const;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__TRAJECTORY_QUALITY_EVALUATOR_HPP_
