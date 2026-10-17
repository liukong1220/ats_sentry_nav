// Copyright 2026

#ifndef MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_
#define MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_

#include <cstddef>
#include <vector>

#include "ats_rc_esdf/esdf/esdf_provider.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

struct TunnelYawParams
{
  bool enabled = false;
  double clearance_enter = 0.45;
  double clearance_exit = 0.50;
  double max_width = 0.90;
  double axis_window = 0.30;
  double min_coherence = 0.80;
  double yaw_offset = 1.5707963267948966;
  double pre_align_distance = 0.53;
  double post_release_distance = 0.31;
};

struct TunnelSegment
{
  std::size_t begin = 0;
  std::size_t end = 0;
  std::vector<double> axis_yaw;
  double width_estimate = 0.0;
  double coherence = 0.0;
};

std::vector<TunnelSegment> detectTunnelSegments(
  const ReferenceTrajectory & trajectory, const ats_rc_esdf::EsdfProvider & esdf,
  const TunnelYawParams & params);

double tunnelAlignedYaw(double axis_yaw, double yaw_offset, double reference_yaw);

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_
