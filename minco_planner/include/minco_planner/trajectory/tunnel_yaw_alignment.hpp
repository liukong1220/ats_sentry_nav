// Copyright 2026

#ifndef MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_
#define MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ats_rc_esdf/esdf/esdf_provider.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

enum class TunnelTraversalPhase : std::uint8_t
{
  kOpen = 0,
  kPrepare = 1,
  kArmed = 2,
  kCommitted = 3,
  kExit = 4
};

struct TunnelYawParams
{
  bool enabled = false;
  double clearance_enter = 0.45;
  double clearance_exit = 0.50;
  double max_width = 0.90;
  double axis_window = 0.30;
  double min_coherence = 0.80;
  // Vehicle +x is the heading and the short 0.571 m face is the front, so
  // driving along the tunnel axis needs no extra rotation (2026-10-23).
  double yaw_offset = 0.0;
  double pre_align_distance = 0.53;
  double post_release_distance = 0.31;
  double axis_tolerance = 0.20;
  double side_wall_gradient_min = 0.35;
  double side_wall_cos2_min = 0.85;
  double forward_width_limit = 0.0;
  double front_extent = 0.0;
  double circumscribed_radius = 0.44;
  // O3 independent switches for tunnel PCA, dynamic pre-align, and narrow terminal pre-align.
  bool sidewall_pca_enabled = true;
  bool dynamic_pre_align_enabled = true;
  bool terminal_goal_axial_override_enabled = true;
  double armed_distance = 0.20;
  double pre_align_time_margin = 0.15;
};

struct TunnelSegment
{
  std::size_t begin = 0;
  std::size_t end = 0;
  std::vector<double> axis_yaw;
  double width_estimate = 0.0;
  double coherence = 0.0;
  bool forward_alignment = false;
  bool terminal_goal_axial_override = false;
};

std::vector<TunnelSegment> detectTunnelSegments(
  const ReferenceTrajectory & trajectory, const ats_rc_esdf::EsdfProvider & esdf,
  const TunnelYawParams & params);

void resolveTerminalTunnelGoalAlignment(
  std::vector<TunnelSegment> & tunnels,
  const ReferenceTrajectory & trajectory,
  double goal_yaw,
  bool is_final_goal,
  const TunnelYawParams & params);

double computeDynamicPreAlignDistance(
  const ReferenceTrajectory & trajectory,
  const TunnelSegment & tunnel,
  double initial_yaw,
  double initial_yaw_rate,
  const TunnelYawParams & params,
  double yaw_rate_limit,
  double yaw_accel_limit);

TunnelTraversalPhase classifyTunnelTraversalPhase(
  const ReferenceTrajectory & trajectory,
  std::size_t point_index,
  const TunnelSegment & tunnel,
  double dynamic_pre_align_distance,
  const TunnelYawParams & params);

double tunnelAlignedYaw(double axis_yaw, double yaw_offset, double reference_yaw);
bool forwardTunnelEntriesAligned(
  const ReferenceTrajectory & trajectory, const std::vector<TunnelSegment> & tunnels,
  const TunnelYawParams & params);

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TUNNEL_YAW_ALIGNMENT_HPP_
