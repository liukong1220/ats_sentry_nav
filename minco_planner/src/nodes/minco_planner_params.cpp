// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

void MincoPlannerNode::declareAndLoadParams()
{
  declare_parameter<std::string>("grid_topic", grid_topic_);
  declare_parameter<std::string>("planning_snapshot_topic", planning_snapshot_topic_);
  declare_parameter<std::string>("goal_topic", goal_topic_);
  declare_parameter<std::string>("goal_request_topic", goal_request_topic_);
  declare_parameter<std::string>("planner_status_topic", planner_status_topic_);
  declare_parameter<std::string>("raw_path_topic", raw_path_topic_);
  declare_parameter<std::string>("reference_path_topic", reference_path_topic_);
  declare_parameter<std::string>("candidate_reference_path_topic", candidate_reference_path_topic_);
  declare_parameter<std::string>("preprocessed_guide_topic", preprocessed_guide_topic_);
  declare_parameter<std::string>("esdf_refined_guide_topic", esdf_refined_guide_topic_);
  declare_parameter<std::string>("rolling_anchor_topic", rolling_anchor_topic_);
  declare_parameter<std::string>("debug_marker_topic", debug_marker_topic_);
  declare_parameter<std::string>("map_ready_topic", map_ready_topic_);
  declare_parameter<std::string>("emergency_stop_topic", emergency_stop_topic_);
  declare_parameter<double>("map_ready_timeout_sec", map_ready_timeout_sec_);
  declare_parameter<double>("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  declare_parameter<double>("runtime_tf_lookup_timeout_sec", runtime_tf_lookup_timeout_sec_);
  declare_parameter<double>("local_route_anchor_offset", local_route_params_.anchor_offset);
  declare_parameter<double>("local_route_bypass_padding", local_route_params_.bypass_padding);
  declare_parameter<double>("local_route_max_local_length", local_route_params_.max_local_length);
  declare_parameter<std::int64_t>("log_throttle_ms", static_cast<std::int64_t>(log_throttle_ms_));
  map_commit_trace_enabled_ = declare_parameter<bool>("map_commit_trace_enabled", false);
  declare_parameter<double>(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  declare_parameter<double>("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  declare_parameter<double>("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  declare_parameter<bool>(
    "retain_safe_reference_on_snapshot_change", retain_safe_reference_on_snapshot_change_);
  declare_parameter<double>("retain_reference_horizon_sec", retain_reference_horizon_sec_);
  declare_parameter<bool>(
    "reference_window_from_robot_progress", reference_window_from_robot_progress_);
  declare_parameter<double>("reference_window_backward_sec", reference_window_backward_sec_);
  declare_parameter<bool>("replan_advisory_enabled", replan_advisory_enabled_);
  declare_parameter<int>("far_conflict_confirm_snapshots", far_conflict_confirm_snapshots_);
  declare_parameter<double>("plan_lead", replan_start_params_.plan_lead_sec);
  declare_parameter<double>("deviation_max", replan_start_params_.deviation_max_m);
  declare_parameter<double>("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  declare_parameter<bool>("force_body_yaw_follow", force_body_yaw_follow_);
  declare_parameter<bool>("spin_enabled", spin_policy_.enabled);
  declare_parameter<double>("spin_rate", spin_yaw_params_.rate);
  declare_parameter<double>("spin_min_clearance", spin_policy_.min_clearance);
  declare_parameter<double>("spin_align_lookahead", spin_policy_.align_lookahead_m);
  declare_parameter<double>("spin_blend_rate_limit", spin_yaw_params_.blend_rate_limit);
  declare_parameter<double>("spin_angular_accel", spin_yaw_params_.angular_accel);
  declare_parameter<double>("spin_resume_after", spin_yaw_params_.resume_after_narrow_m);
  declare_parameter<std::string>("global_frame", global_frame_);
  declare_parameter<std::string>("robot_frame", robot_frame_);
  declare_parameter<std::string>("lidar_frame", lidar_frame_);
  declare_parameter<std::string>("search_algorithm", search_algorithm_);
  declare_parameter<std::string>("prior_guide_map_topic", prior_guide_map_topic_);
  declare_parameter<bool>("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  declare_parameter<bool>(
    "escape_from_contact_enabled", escape_prefix_params_.enabled);
  declare_parameter<double>(
    "escape_from_contact_max_head_offset", escape_prefix_params_.max_head_offset_m);
  declare_parameter<double>(
    "escape_from_contact_max_prefix_length", escape_prefix_params_.max_prefix_length_m);
  declare_parameter<double>(
    "escape_from_contact_max_prefix_yaw_sweep",
    escape_prefix_params_.max_prefix_yaw_sweep_rad);
  declare_parameter<int>(
    "escape_from_contact_max_prefix_points",
    static_cast<int>(escape_prefix_params_.max_prefix_points));
  declare_parameter<double>(
    "escape_from_contact_max_contact_depth", escape_prefix_params_.max_contact_depth_m);
  declare_parameter<double>(
    "escape_from_contact_monotone_max_depth", escape_prefix_params_.monotone_max_contact_depth_m);
  declare_parameter<double>(
    "escape_from_contact_monotone_depth_step", escape_prefix_params_.monotone_depth_step_m);
  declare_parameter<double>(
    "escape_from_contact_monotone_max_prefix_length",
    escape_prefix_params_.monotone_max_prefix_length_m);
  declare_parameter<bool>(
    "goal_pose_admission_enabled", goal_pose_admission_params_.enabled);
  declare_parameter<double>(
    "goal_admission_position_tolerance", goal_pose_admission_params_.position_tolerance_m);
  declare_parameter<double>(
    "goal_admission_yaw_tolerance", goal_pose_admission_params_.yaw_tolerance_rad);
  declare_parameter<double>(
    "goal_admission_position_shrink", goal_pose_admission_params_.position_shrink);
  declare_parameter<double>(
    "goal_admission_yaw_shrink", goal_pose_admission_params_.yaw_shrink);
  declare_parameter<double>(
    "goal_admission_extra_margin", goal_pose_admission_params_.preferred_extra_margin_m);
  declare_parameter<double>(
    "goal_admission_position_step", goal_pose_admission_params_.position_step_m);
  declare_parameter<int>(
    "goal_admission_yaw_samples",
    static_cast<int>(goal_pose_admission_params_.yaw_samples));
  // <=0 disables. Gazebo red_box enables to block north mega-detours (d21).
  declare_parameter<double>(
    "commit_max_length_ratio", commit_geometry_limits_.max_length_ratio);
  declare_parameter<double>(
    "commit_max_lateral_deviation_m", commit_geometry_limits_.max_lateral_deviation_m);
  declare_parameter<bool>("planner_manages_emergency_stop", planner_manages_emergency_stop_);

  GridSearchParams astar_params;
  declare_parameter<int>("obstacle_value_threshold", astar_params.obstacle_value_threshold);
  declare_parameter<bool>("unknown_is_obstacle", astar_params.unknown_is_obstacle);
  declare_parameter<bool>("allow_diagonal", astar_params.allow_diagonal);
  declare_parameter<double>("search_clearance_cost_weight", astar_params.clearance_cost_weight);
  declare_parameter<double>(
    "search_clearance_cost_distance", astar_params.clearance_cost_distance);
  declare_parameter<int>("start_escape_max_steps", astar_params.start_escape_max_steps);
  declare_parameter<int>("jps_max_expanded_nodes", astar_params.max_expanded_nodes);
  declare_parameter<double>("jps_safe_distance", astar_params.safe_distance);
  declare_parameter<bool>("clearance_relaxation_enabled", clearance_relaxation_enabled_);
  // 首选净空路径超过直线距离该倍数时,再用下限档搜一次并优先明显更短的;<=0 关闭。
  declare_parameter<double>("clearance_detour_ratio", clearance_detour_ratio_);
  declare_parameter<bool>(
    "endpoint_clearance_relaxation_enabled", endpoint_clearance_relaxation_enabled_);
  declare_parameter<double>("search_clearance_floor", -1.0);

  MincoTrajectoryOptimizerParams optimizer_params;
  declare_parameter<double>("reference_speed", optimizer_params.reference_speed);
  declare_parameter<double>("min_segment_time", optimizer_params.min_segment_time);
  declare_parameter<double>("sample_spacing", optimizer_params.sample_spacing);
  declare_parameter<double>("max_velocity", optimizer_params.max_velocity);
  declare_parameter<double>("max_acceleration", optimizer_params.max_acceleration);
  declare_parameter<double>("max_jerk", optimizer_params.max_jerk);
  declare_parameter<double>("max_lateral_acceleration", optimizer_params.max_lateral_acceleration);
  declare_parameter<double>(
    "corner_turn_threshold_rad", optimizer_params.corner_turn_threshold_rad);
  declare_parameter<double>(
    "corner_inscribed_ratio", optimizer_params.corner_inscribed_ratio);
  declare_parameter<double>(
    "min_corner_radius", optimizer_params.min_corner_radius);
  declare_parameter<int>(
    "max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  declare_parameter<double>("time_scaling_factor", optimizer_params.time_scaling_factor);
  declare_parameter<bool>(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  declare_parameter<double>("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  declare_parameter<double>(
    "esdf_obstacle_trigger_clearance", optimizer_params.esdf_obstacle_trigger_clearance);
  declare_parameter<double>(
    "esdf_obstacle_target_clearance", optimizer_params.esdf_obstacle_target_clearance);
  declare_parameter<int>(
    "esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  declare_parameter<double>(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  declare_parameter<double>(
    "guide_control_point_spacing", optimizer_params.guide_control_point_spacing);
  declare_parameter<int>(
    "guide_smoothing_iterations", optimizer_params.guide_smoothing_iterations);
  declare_parameter<double>("guide_smoothing_alpha", optimizer_params.guide_smoothing_alpha);
  declare_parameter<double>(
    "guide_smoothing_fidelity", optimizer_params.guide_smoothing_fidelity);
  declare_parameter<double>(
    "guide_smoothing_max_deviation", optimizer_params.guide_smoothing_max_deviation);
  declare_parameter<double>(
    "guide_smoothing_min_clearance", optimizer_params.guide_smoothing_min_clearance);
  declare_parameter<double>("joint_waypoint_spacing", optimizer_params.joint_waypoint_spacing);
  declare_parameter<int>("joint_samples_per_piece", optimizer_params.joint_samples_per_piece);
  declare_parameter<double>("joint_max_piece_time", optimizer_params.joint_max_piece_time);
  declare_parameter<double>("joint_energy_weight", optimizer_params.joint_energy_weight);
  declare_parameter<double>("joint_time_weight", optimizer_params.joint_time_weight);
  declare_parameter<double>("joint_obstacle_weight", optimizer_params.joint_obstacle_weight);
  declare_parameter<double>("joint_velocity_weight", optimizer_params.joint_velocity_weight);
  declare_parameter<double>(
    "joint_acceleration_weight", optimizer_params.joint_acceleration_weight);
  declare_parameter<double>("joint_lateral_weight", optimizer_params.joint_lateral_weight);
  declare_parameter<double>("joint_guide_weight", optimizer_params.joint_guide_weight);
  declare_parameter<double>(
    "joint_guide_max_deviation", optimizer_params.joint_guide_max_deviation);
  declare_parameter<double>("joint_center_clearance", optimizer_params.joint_center_clearance);
  declare_parameter<double>(
    "joint_footprint_clearance", optimizer_params.joint_footprint_clearance);
  declare_parameter<int>(
    "joint_footprint_edge_samples", optimizer_params.joint_footprint_edge_samples);
  declare_parameter<double>("joint_g_epsilon", optimizer_params.joint_g_epsilon);
  declare_parameter<int>("joint_max_iterations", optimizer_params.joint_max_iterations);
  declare_parameter<double>("joint_time_budget_ms", optimizer_params.joint_time_budget_ms);
  declare_parameter<std::string>("joint_distance_field", optimizer_params.joint_distance_field);
  declare_parameter<double>(
    "joint_obstacle_smooth_l1_weight", optimizer_params.joint_obstacle_smooth_l1_weight);
  declare_parameter<double>(
    "joint_obstacle_smooth_l1_mu", optimizer_params.joint_obstacle_smooth_l1_mu);
  declare_parameter<double>("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  declare_parameter<double>(
    "esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  declare_parameter<double>(
    "esdf_obstacle_trust_region", optimizer_params.esdf_obstacle_trust_region);
  declare_parameter<int>(
    "esdf_obstacle_backtracking_steps", optimizer_params.esdf_obstacle_backtracking_steps);
  declare_parameter<double>(
    "esdf_obstacle_smoothing_weight", optimizer_params.esdf_obstacle_smoothing_weight);
  declare_parameter<bool>(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  declare_parameter<double>("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  declare_parameter<double>(
    "esdf_footprint_trigger_clearance", optimizer_params.esdf_footprint_trigger_clearance);
  declare_parameter<double>(
    "esdf_footprint_target_clearance", optimizer_params.esdf_footprint_target_clearance);
  declare_parameter<double>(
    "esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);
  declare_parameter<double>(
    "path_duplicate_epsilon", optimizer_params.geometry_preprocessor.duplicate_epsilon);
  declare_parameter<double>(
    "path_collinear_lateral_tolerance",
    optimizer_params.geometry_preprocessor.collinear_lateral_tolerance);
  declare_parameter<double>(
    "path_short_segment_length", optimizer_params.geometry_preprocessor.short_segment_length);
  declare_parameter<double>(
    "path_corner_angle_threshold_rad",
    optimizer_params.geometry_preprocessor.corner_angle_threshold_rad);
  declare_parameter<bool>(
    "path_footprint_aware_shortcut_enabled",
    optimizer_params.geometry_preprocessor.footprint_aware_shortcut_enabled);
  declare_parameter<double>(
    "path_fillet_radius", optimizer_params.geometry_preprocessor.fillet_radius);
  declare_parameter<int>(
    "path_fillet_arc_samples", optimizer_params.geometry_preprocessor.fillet_arc_samples);
  declare_parameter<double>(
    "path_shortcut_min_clearance", optimizer_params.geometry_preprocessor.shortcut_min_clearance);

  YawSplinePlannerParams yaw_params;
  declare_parameter<std::string>("yaw_mode", yaw_params.mode);
  declare_parameter<double>("yaw_rate_limit", yaw_params.yaw_rate_limit);
  declare_parameter<double>("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  declare_parameter<double>("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  declare_parameter<int>("yaw_tangent_symmetry_order", yaw_params.tangent_symmetry_order);
  declare_parameter<double>("yaw_narrow_gap_bridge_time", yaw_params.narrow_gap_bridge_time);
  declare_parameter<double>("yaw_acceleration_limit", yaw_params.yaw_acceleration_limit);
  declare_parameter<std::string>("yaw_open_area_mode", yaw_params.open_area_yaw_mode);
  declare_parameter<double>("yaw_narrow_start_hold_length", 0.0);
  declare_parameter<double>(
    "wheel_speed_time_scaling_limit", wheel_speed_time_scaling_params_.wheel_speed_limit);
  declare_parameter<double>(
    "wheel_speed_time_scaling_offset_x", wheel_speed_time_scaling_params_.wheel_offset_x);
  declare_parameter<double>(
    "wheel_speed_time_scaling_offset_y", wheel_speed_time_scaling_params_.wheel_offset_y);
  declare_parameter<double>(
    "wheel_speed_time_scaling_change_rate", wheel_speed_time_scaling_params_.scale_change_rate);
  declare_parameter<double>(
    "narrow_turn_speed_limit", wheel_speed_time_scaling_params_.narrow_turn_speed_limit);
  declare_parameter<double>(
    "narrow_turn_yaw_rate_threshold",
    wheel_speed_time_scaling_params_.narrow_turn_yaw_rate_threshold);
  // <= 0：与 yaw 规划共用 narrow_clearance_enter（旧行为）。
  declare_parameter<double>("narrow_turn_clearance", 0.0);
  declare_parameter<double>(
    "clearance_speed_min", wheel_speed_time_scaling_params_.clearance_speed_min);
  declare_parameter<double>(
    "clearance_speed_low", wheel_speed_time_scaling_params_.clearance_speed_low);
  declare_parameter<double>(
    "clearance_speed_high", wheel_speed_time_scaling_params_.clearance_speed_high);
  declare_parameter<double>(
    "terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);
  declare_parameter<bool>("yaw_tunnel_enabled", yaw_params.tunnel.enabled);
  declare_parameter<double>("yaw_tunnel_yaw_offset", yaw_params.tunnel.yaw_offset);
  declare_parameter<double>("yaw_tunnel_clearance_enter", yaw_params.tunnel.clearance_enter);
  declare_parameter<double>("yaw_tunnel_clearance_exit", yaw_params.tunnel.clearance_exit);
  declare_parameter<double>("yaw_tunnel_max_width", yaw_params.tunnel.max_width);
  declare_parameter<double>("yaw_tunnel_axis_window", yaw_params.tunnel.axis_window);
  declare_parameter<double>(
    "yaw_tunnel_axis_min_coherence", yaw_params.tunnel.min_coherence);
  declare_parameter<double>(
    "yaw_tunnel_pre_align_distance", yaw_params.tunnel.pre_align_distance);
  declare_parameter<double>(
    "yaw_tunnel_post_release_distance", yaw_params.tunnel.post_release_distance);
  declare_parameter<double>(
    "yaw_tunnel_axis_tolerance", yaw_params.tunnel.axis_tolerance);
  declare_parameter<double>(
    "yaw_tunnel_side_wall_gradient_min", yaw_params.tunnel.side_wall_gradient_min);
  declare_parameter<double>(
    "yaw_tunnel_side_wall_cos2_min", yaw_params.tunnel.side_wall_cos2_min);
  declare_parameter<bool>(
    "yaw_tunnel_sidewall_pca_enabled", yaw_params.tunnel.sidewall_pca_enabled);
  declare_parameter<bool>(
    "yaw_tunnel_dynamic_pre_align_enabled", yaw_params.tunnel.dynamic_pre_align_enabled);
  declare_parameter<bool>(
    "yaw_tunnel_terminal_goal_axial_override_enabled",
    yaw_params.tunnel.terminal_goal_axial_override_enabled);
  declare_parameter<double>(
    "yaw_tunnel_armed_distance", yaw_params.tunnel.armed_distance);
  declare_parameter<double>(
    "yaw_tunnel_pre_align_time_margin", yaw_params.tunnel.pre_align_time_margin);
  declare_parameter<bool>("yaw_tunnel_lock_gimbal", tunnel_lock_gimbal_);
  declare_parameter<bool>(
    "terminal_yaw_relocation_enabled", terminal_yaw_relocation_enabled_);
  declare_parameter<int>(
    "footprint_yaw_refinement_rounds", footprint_yaw_refinement_rounds_);
  declare_parameter<int>(
    "terminal_yaw_relocation_max_candidates", terminal_yaw_relocation_max_candidates_);
  declare_parameter<double>(
    "terminal_yaw_relocation_window_length",
    terminal_yaw_relocation_params_.terminal_window_length);
  declare_parameter<double>(
    "terminal_rotation_comfortable_clearance_m", terminal_rotation_comfortable_clearance_m_);
  declare_parameter<double>(
    "terminal_rotation_tight_yaw_span_rad", terminal_rotation_tight_yaw_span_rad_);
  declare_parameter<double>(
    "sidestep_min_duration", terminal_yaw_relocation_params_.sidestep_min_duration);
  declare_parameter<double>(
    "sidestep_max_speed", terminal_yaw_relocation_params_.sidestep_max_speed);
  declare_parameter<double>(
    "sidestep_max_accel", terminal_yaw_relocation_params_.sidestep_max_accel);
  declare_parameter<double>("sidestep_min_esdf_margin_m", sidestep_min_esdf_margin_m_);
  declare_parameter<double>("sidestep_desired_esdf_margin_m", sidestep_desired_esdf_margin_m_);
  declare_parameter<double>("sidestep_max_radius_m", sidestep_max_radius_m_);
  declare_parameter<int>("sidestep_max_tries", sidestep_max_tries_);
  declare_parameter<double>("sidestep_accept_clearance_m", sidestep_accept_clearance_m_);
  declare_parameter<double>("sidestep_min_improvement_m", sidestep_min_improvement_m_);
  declare_parameter<bool>("rolling_horizon_enabled", rolling_horizon_enabled_);
  declare_parameter<double>("rolling_horizon_length_m", rolling_horizon_length_m_);
  declare_parameter<double>("rolling_min_tail_length_m", rolling_min_tail_length_m_);
  declare_parameter<bool>(
    "partial_subgoal_continuation_enabled", partial_subgoal_continuation_enabled_);
  declare_parameter<bool>(
    "partial_subgoal_only_without_prior", partial_subgoal_only_without_prior_);
  declare_parameter<double>(
    "partial_subgoal_min_advance_m", partial_subgoal_min_advance_m_);
  declare_parameter<double>(
    "optimistic_global_unknown_cost", optimistic_global_unknown_cost_);
  declare_parameter<double>("frontier_taboo_radius_m", frontier_taboo_radius_m_);
  declare_parameter<double>("frontier_taboo_ttl_sec", frontier_taboo_ttl_sec_);
  declare_parameter<double>(
    "footprint_refinement_skip_clearance_m", footprint_refinement_skip_clearance_m_);
  declare_parameter<double>("minco_stage_budget_ms", minco_stage_budget_ms_);
  declare_parameter<double>("plan_wall_budget_ms", plan_wall_budget_ms_);
  declare_parameter<double>("rolling_replan_wall_budget_ms", rolling_replan_wall_budget_ms_);
  declare_parameter<double>("plan_hard_wall_budget_ms", plan_hard_wall_budget_ms_);

  FootprintSafetyParams footprint_params;
  declare_parameter<double>("footprint_length", footprint_params.length);
  declare_parameter<double>("footprint_width", footprint_params.width);
  declare_parameter<double>("footprint_safety_margin", footprint_params.safety_margin);
  declare_parameter<double>(
    "swept_max_corner_step_cells", footprint_params.swept_max_corner_step_cells);
  declare_parameter<bool>(
    "adaptive_lipschitz_swept_enabled", footprint_params.adaptive_lipschitz_swept_enabled);
  declare_parameter<double>(
    "swept_min_corner_step_m", footprint_params.swept_min_corner_step_m);

  LocalCollisionRepairParams repair_params;
  declare_parameter<bool>("local_repair_enabled", repair_params.enabled);
  declare_parameter<int>("local_repair_max_iterations", repair_params.max_iterations);
  declare_parameter<double>("local_repair_search_radius", repair_params.search_radius);

  get_parameter("grid_topic", grid_topic_);
  get_parameter("planning_snapshot_topic", planning_snapshot_topic_);
  get_parameter("goal_topic", goal_topic_);
  get_parameter("goal_request_topic", goal_request_topic_);
  get_parameter("planner_status_topic", planner_status_topic_);
  get_parameter("raw_path_topic", raw_path_topic_);
  get_parameter("reference_path_topic", reference_path_topic_);
  get_parameter("candidate_reference_path_topic", candidate_reference_path_topic_);
  get_parameter("preprocessed_guide_topic", preprocessed_guide_topic_);
  get_parameter("esdf_refined_guide_topic", esdf_refined_guide_topic_);
  get_parameter("rolling_anchor_topic", rolling_anchor_topic_);
  get_parameter("debug_marker_topic", debug_marker_topic_);
  get_parameter("map_ready_topic", map_ready_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("map_ready_timeout_sec", map_ready_timeout_sec_);
  map_ready_timeout_sec_ = std::max(0.1, map_ready_timeout_sec_);
  get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  tf_lookup_timeout_sec_ = std::max(0.01, tf_lookup_timeout_sec_);
  get_parameter("runtime_tf_lookup_timeout_sec", runtime_tf_lookup_timeout_sec_);
  runtime_tf_lookup_timeout_sec_ = std::max(0.01, runtime_tf_lookup_timeout_sec_);
  get_parameter("local_route_anchor_offset", local_route_params_.anchor_offset);
  local_route_params_.anchor_offset = std::max(0.0, local_route_params_.anchor_offset);
  get_parameter("local_route_bypass_padding", local_route_params_.bypass_padding);
  local_route_params_.bypass_padding = std::max(0.0, local_route_params_.bypass_padding);
  get_parameter("local_route_max_local_length", local_route_params_.max_local_length);
  local_route_params_.max_local_length = std::max(0.5, local_route_params_.max_local_length);
  std::int64_t configured_log_throttle_ms = static_cast<std::int64_t>(log_throttle_ms_);
  get_parameter("log_throttle_ms", configured_log_throttle_ms);
  log_throttle_ms_ = static_cast<std::uint64_t>(
    std::clamp<std::int64_t>(configured_log_throttle_ms, 250, 60000));
  get_parameter(
    "emergency_stop_heartbeat_period_sec", emergency_stop_heartbeat_period_sec_);
  emergency_stop_heartbeat_period_sec_ = std::max(0.02, emergency_stop_heartbeat_period_sec_);
  get_parameter("runtime_safety_recheck_hz", runtime_safety_recheck_hz_);
  runtime_safety_recheck_hz_ = std::max(0.1, runtime_safety_recheck_hz_);
  get_parameter("runtime_safety_horizon_sec", runtime_safety_horizon_sec_);
  runtime_safety_horizon_sec_ = std::max(0.0, runtime_safety_horizon_sec_);
  get_parameter(
    "retain_safe_reference_on_snapshot_change", retain_safe_reference_on_snapshot_change_);
  get_parameter("retain_reference_horizon_sec", retain_reference_horizon_sec_);
  get_parameter("reference_window_from_robot_progress", reference_window_from_robot_progress_);
  get_parameter("replan_advisory_enabled", replan_advisory_enabled_);
  get_parameter("far_conflict_confirm_snapshots", far_conflict_confirm_snapshots_);
  far_conflict_confirm_snapshots_ = std::max(1, far_conflict_confirm_snapshots_);
  get_parameter("plan_lead", replan_start_params_.plan_lead_sec);
  replan_start_params_.plan_lead_sec = std::isfinite(replan_start_params_.plan_lead_sec) ?
    std::max(0.0, replan_start_params_.plan_lead_sec) : 0.0;
  get_parameter("deviation_max", replan_start_params_.deviation_max_m);
  get_parameter("reference_window_backward_sec", reference_window_backward_sec_);
  reference_window_backward_sec_ = std::max(0.0, reference_window_backward_sec_);
  get_parameter("body_yaw_follow_clearance", body_yaw_follow_clearance_);
  body_yaw_follow_clearance_ = std::max(0.0, body_yaw_follow_clearance_);
  get_parameter("force_body_yaw_follow", force_body_yaw_follow_);
  get_parameter("spin_enabled", spin_policy_.enabled);
  get_parameter("spin_rate", spin_yaw_params_.rate);
  get_parameter("spin_min_clearance", spin_policy_.min_clearance);
  get_parameter("spin_align_lookahead", spin_policy_.align_lookahead_m);
  get_parameter("spin_blend_rate_limit", spin_yaw_params_.blend_rate_limit);
  get_parameter("spin_angular_accel", spin_yaw_params_.angular_accel);
  get_parameter("spin_resume_after", spin_yaw_params_.resume_after_narrow_m);
  spin_policy_.min_clearance = std::max(0.0, spin_policy_.min_clearance);
  spin_policy_.align_lookahead_m = std::max(0.0, spin_policy_.align_lookahead_m);
  spin_yaw_params_.rate = std::max(0.0, spin_yaw_params_.rate);
  spin_yaw_params_.min_clearance = spin_policy_.min_clearance;
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("lidar_frame", lidar_frame_);
  get_parameter("search_algorithm", search_algorithm_);
  get_parameter("prior_guide_map_topic", prior_guide_map_topic_);
  get_parameter("publish_unsafe_trajectory", publish_unsafe_trajectory_);
  get_parameter("escape_from_contact_enabled", escape_prefix_params_.enabled);
  get_parameter("escape_from_contact_max_head_offset", escape_prefix_params_.max_head_offset_m);
  get_parameter(
    "escape_from_contact_max_prefix_length", escape_prefix_params_.max_prefix_length_m);
  get_parameter(
    "escape_from_contact_max_prefix_yaw_sweep",
    escape_prefix_params_.max_prefix_yaw_sweep_rad);
  {
    int escape_max_prefix_points =
      static_cast<int>(escape_prefix_params_.max_prefix_points);
    get_parameter("escape_from_contact_max_prefix_points", escape_max_prefix_points);
    escape_prefix_params_.max_prefix_points =
      static_cast<std::size_t>(std::max(0, escape_max_prefix_points));
  }
  get_parameter(
    "escape_from_contact_max_contact_depth", escape_prefix_params_.max_contact_depth_m);
  get_parameter(
    "escape_from_contact_monotone_max_depth", escape_prefix_params_.monotone_max_contact_depth_m);
  get_parameter(
    "escape_from_contact_monotone_depth_step", escape_prefix_params_.monotone_depth_step_m);
  get_parameter(
    "escape_from_contact_monotone_max_prefix_length",
    escape_prefix_params_.monotone_max_prefix_length_m);
  escape_prefix_params_.monotone_max_prefix_length_m =
    std::max(0.0, escape_prefix_params_.monotone_max_prefix_length_m);
  escape_prefix_params_.monotone_max_contact_depth_m =
    std::max(0.0, escape_prefix_params_.monotone_max_contact_depth_m);
  escape_prefix_params_.monotone_depth_step_m =
    std::max(1e-3, escape_prefix_params_.monotone_depth_step_m);
  get_parameter("goal_pose_admission_enabled", goal_pose_admission_params_.enabled);
  get_parameter(
    "goal_admission_position_tolerance", goal_pose_admission_params_.position_tolerance_m);
  get_parameter("goal_admission_yaw_tolerance", goal_pose_admission_params_.yaw_tolerance_rad);
  get_parameter("goal_admission_position_shrink", goal_pose_admission_params_.position_shrink);
  get_parameter("goal_admission_yaw_shrink", goal_pose_admission_params_.yaw_shrink);
  get_parameter("goal_admission_extra_margin",
    goal_pose_admission_params_.preferred_extra_margin_m);
  get_parameter("goal_admission_position_step", goal_pose_admission_params_.position_step_m);
  {
    int goal_admission_yaw_samples =
      static_cast<int>(goal_pose_admission_params_.yaw_samples);
    get_parameter("goal_admission_yaw_samples", goal_admission_yaw_samples);
    goal_pose_admission_params_.yaw_samples =
      static_cast<std::size_t>(std::max(1, goal_admission_yaw_samples));
  }
  get_parameter("commit_max_length_ratio", commit_geometry_limits_.max_length_ratio);
  get_parameter(
    "commit_max_lateral_deviation_m", commit_geometry_limits_.max_lateral_deviation_m);
  commit_geometry_limits_.max_length_ratio =
    std::max(0.0, commit_geometry_limits_.max_length_ratio);
  commit_geometry_limits_.max_lateral_deviation_m =
    std::max(0.0, commit_geometry_limits_.max_lateral_deviation_m);
  get_parameter("planner_manages_emergency_stop", planner_manages_emergency_stop_);
  get_parameter("obstacle_value_threshold", obstacle_value_threshold_);
  get_parameter("unknown_is_obstacle", unknown_is_obstacle_);
  astar_params.obstacle_value_threshold = obstacle_value_threshold_;
  astar_params.unknown_is_obstacle = unknown_is_obstacle_;
  get_parameter("allow_diagonal", astar_params.allow_diagonal);
  get_parameter("search_clearance_cost_weight", astar_params.clearance_cost_weight);
  get_parameter("search_clearance_cost_distance", astar_params.clearance_cost_distance);
  get_parameter("start_escape_max_steps", astar_params.start_escape_max_steps);
  astar_params.start_escape_max_steps = std::max(0, astar_params.start_escape_max_steps);
  if (search_algorithm_ != "astar" && search_algorithm_ != "jps" &&
    search_algorithm_ != "hybrid")
  {
    RCLCPP_WARN(
      get_logger(), "Unknown search_algorithm '%s'; using astar.", search_algorithm_.c_str());
    search_algorithm_ = "astar";
  }
  astar_params.jump_enabled = search_algorithm_ != "astar";
  get_parameter("jps_max_expanded_nodes", astar_params.max_expanded_nodes);
  get_parameter("jps_safe_distance", astar_params.safe_distance);
  get_parameter("reference_speed", optimizer_params.reference_speed);
  get_parameter("min_segment_time", optimizer_params.min_segment_time);
  get_parameter("sample_spacing", optimizer_params.sample_spacing);
  get_parameter("max_velocity", optimizer_params.max_velocity);
  get_parameter("max_acceleration", optimizer_params.max_acceleration);
  // 起点保护跟随本profile的动力学上限，避免配置5 m/s后仍把换轨首速截成旧2 m/s。
  if (optimizer_params.max_velocity > 0.0) {
    optimizer_params.initial_state_max_speed = optimizer_params.max_velocity;
  }
  get_parameter("max_jerk", optimizer_params.max_jerk);
  get_parameter("max_lateral_acceleration", optimizer_params.max_lateral_acceleration);
  get_parameter("corner_turn_threshold_rad", optimizer_params.corner_turn_threshold_rad);
  get_parameter("corner_inscribed_ratio", optimizer_params.corner_inscribed_ratio);
  get_parameter("min_corner_radius", optimizer_params.min_corner_radius);
  get_parameter("max_time_scaling_iterations", optimizer_params.max_time_scaling_iterations);
  get_parameter("time_scaling_factor", optimizer_params.time_scaling_factor);
  get_parameter(
    "esdf_obstacle_optimization_enabled", optimizer_params.esdf_obstacle_optimization_enabled);
  get_parameter("esdf_obstacle_clearance", optimizer_params.esdf_obstacle_clearance);
  get_parameter(
    "esdf_obstacle_trigger_clearance", optimizer_params.esdf_obstacle_trigger_clearance);
  get_parameter(
    "esdf_obstacle_target_clearance", optimizer_params.esdf_obstacle_target_clearance);
  get_parameter("esdf_obstacle_max_iterations", optimizer_params.esdf_obstacle_max_iterations);
  get_parameter(
    "esdf_obstacle_control_point_spacing", optimizer_params.esdf_obstacle_control_point_spacing);
  get_parameter(
    "guide_control_point_spacing", optimizer_params.guide_control_point_spacing);
  get_parameter("guide_smoothing_iterations", optimizer_params.guide_smoothing_iterations);
  get_parameter("guide_smoothing_alpha", optimizer_params.guide_smoothing_alpha);
  get_parameter("guide_smoothing_fidelity", optimizer_params.guide_smoothing_fidelity);
  get_parameter(
    "guide_smoothing_max_deviation", optimizer_params.guide_smoothing_max_deviation);
  get_parameter(
    "guide_smoothing_min_clearance", optimizer_params.guide_smoothing_min_clearance);
  get_parameter("joint_waypoint_spacing", optimizer_params.joint_waypoint_spacing);
  get_parameter("joint_samples_per_piece", optimizer_params.joint_samples_per_piece);
  get_parameter("joint_max_piece_time", optimizer_params.joint_max_piece_time);
  get_parameter("joint_energy_weight", optimizer_params.joint_energy_weight);
  get_parameter("joint_time_weight", optimizer_params.joint_time_weight);
  get_parameter("joint_obstacle_weight", optimizer_params.joint_obstacle_weight);
  get_parameter("joint_velocity_weight", optimizer_params.joint_velocity_weight);
  get_parameter("joint_acceleration_weight", optimizer_params.joint_acceleration_weight);
  get_parameter("joint_lateral_weight", optimizer_params.joint_lateral_weight);
  get_parameter("joint_guide_weight", optimizer_params.joint_guide_weight);
  get_parameter("joint_guide_max_deviation", optimizer_params.joint_guide_max_deviation);
  get_parameter("joint_center_clearance", optimizer_params.joint_center_clearance);
  get_parameter("joint_footprint_clearance", optimizer_params.joint_footprint_clearance);
  get_parameter("joint_footprint_edge_samples", optimizer_params.joint_footprint_edge_samples);
  get_parameter("joint_g_epsilon", optimizer_params.joint_g_epsilon);
  get_parameter("joint_max_iterations", optimizer_params.joint_max_iterations);
  get_parameter("joint_time_budget_ms", optimizer_params.joint_time_budget_ms);
  get_parameter("joint_distance_field", optimizer_params.joint_distance_field);
  get_parameter(
    "joint_obstacle_smooth_l1_weight", optimizer_params.joint_obstacle_smooth_l1_weight);
  get_parameter("joint_obstacle_smooth_l1_mu", optimizer_params.joint_obstacle_smooth_l1_mu);
  get_parameter("esdf_obstacle_max_step", optimizer_params.esdf_obstacle_max_step);
  get_parameter("esdf_obstacle_max_deviation", optimizer_params.esdf_obstacle_max_deviation);
  get_parameter("esdf_obstacle_trust_region", optimizer_params.esdf_obstacle_trust_region);
  get_parameter(
    "esdf_obstacle_backtracking_steps", optimizer_params.esdf_obstacle_backtracking_steps);
  get_parameter(
    "esdf_obstacle_smoothing_weight", optimizer_params.esdf_obstacle_smoothing_weight);
  get_parameter(
    "esdf_footprint_optimization_enabled", optimizer_params.esdf_footprint_optimization_enabled);
  get_parameter("esdf_footprint_clearance", optimizer_params.esdf_footprint_clearance);
  get_parameter(
    "esdf_footprint_trigger_clearance", optimizer_params.esdf_footprint_trigger_clearance);
  get_parameter(
    "esdf_footprint_target_clearance", optimizer_params.esdf_footprint_target_clearance);
  get_parameter("esdf_footprint_sample_spacing", optimizer_params.esdf_footprint_sample_spacing);
  get_parameter("path_duplicate_epsilon", optimizer_params.geometry_preprocessor.duplicate_epsilon);
  get_parameter(
    "path_collinear_lateral_tolerance",
    optimizer_params.geometry_preprocessor.collinear_lateral_tolerance);
  get_parameter(
    "path_short_segment_length", optimizer_params.geometry_preprocessor.short_segment_length);
  get_parameter(
    "path_corner_angle_threshold_rad",
    optimizer_params.geometry_preprocessor.corner_angle_threshold_rad);
  get_parameter(
    "path_footprint_aware_shortcut_enabled",
    optimizer_params.geometry_preprocessor.footprint_aware_shortcut_enabled);
  get_parameter(
    "path_fillet_radius", optimizer_params.geometry_preprocessor.fillet_radius);
  get_parameter(
    "path_fillet_arc_samples", optimizer_params.geometry_preprocessor.fillet_arc_samples);
  get_parameter(
    "path_shortcut_min_clearance", optimizer_params.geometry_preprocessor.shortcut_min_clearance);
  get_parameter("yaw_mode", yaw_params.mode);
  get_parameter("yaw_rate_limit", yaw_params.yaw_rate_limit);
  get_parameter("narrow_clearance_enter", yaw_params.narrow_clearance_enter);
  get_parameter("narrow_clearance_exit", yaw_params.narrow_clearance_exit);
  get_parameter("yaw_tangent_symmetry_order", yaw_params.tangent_symmetry_order);
  get_parameter("yaw_narrow_gap_bridge_time", yaw_params.narrow_gap_bridge_time);
  get_parameter("yaw_acceleration_limit", yaw_params.yaw_acceleration_limit);
  get_parameter("yaw_open_area_mode", yaw_params.open_area_yaw_mode);
  if (yaw_params.open_area_yaw_mode != "goal_heading" &&
    yaw_params.open_area_yaw_mode != "minimal_rotation")
  {
    RCLCPP_WARN(
      get_logger(), "【航向参数告警】yaw_open_area_mode=%s 无效，已回退为 goal_heading。",
      yaw_params.open_area_yaw_mode.c_str());
    yaw_params.open_area_yaw_mode = "goal_heading";
  }
  get_parameter("wheel_speed_time_scaling_limit", wheel_speed_time_scaling_params_.wheel_speed_limit);
  get_parameter(
    "wheel_speed_time_scaling_offset_x", wheel_speed_time_scaling_params_.wheel_offset_x);
  get_parameter(
    "wheel_speed_time_scaling_offset_y", wheel_speed_time_scaling_params_.wheel_offset_y);
  get_parameter(
    "wheel_speed_time_scaling_change_rate", wheel_speed_time_scaling_params_.scale_change_rate);
  get_parameter(
    "narrow_turn_speed_limit", wheel_speed_time_scaling_params_.narrow_turn_speed_limit);
  get_parameter(
    "narrow_turn_yaw_rate_threshold",
    wheel_speed_time_scaling_params_.narrow_turn_yaw_rate_threshold);
  get_parameter("clearance_speed_min", wheel_speed_time_scaling_params_.clearance_speed_min);
  get_parameter("clearance_speed_low", wheel_speed_time_scaling_params_.clearance_speed_low);
  get_parameter("clearance_speed_high", wheel_speed_time_scaling_params_.clearance_speed_high);
  // 默认与 yaw 规划共用进入阈值，两者对"窄"的定义一致。单独给 narrow_turn_clearance > 0 时，
  // 限速覆盖更宽的"边平移边转向"区段而不改 yaw 规划的窄通道判据（报告 5.1.10 第 7 项：
  // 2025 红框落差转弯与走廊入口在 1.0-1.4 m/s 下边走边转，横向误差 0.2-0.5 m，中心净空
  // 大于 0.55 m 的 narrow_clearance_enter，旧限速不生效）。
  double narrow_turn_clearance = 0.0;
  get_parameter("narrow_turn_clearance", narrow_turn_clearance);
  wheel_speed_time_scaling_params_.narrow_turn_clearance =
    narrow_turn_clearance > 0.0 ? narrow_turn_clearance : yaw_params.narrow_clearance_enter;
  get_parameter("terminal_yaw_sample_period", yaw_params.terminal_yaw_sample_period);
  get_parameter("yaw_narrow_start_hold_length", narrow_start_hold_length_);
  narrow_start_hold_length_ = std::max(0.0, narrow_start_hold_length_);
  get_parameter("yaw_tunnel_enabled", yaw_params.tunnel.enabled);
  get_parameter("yaw_tunnel_yaw_offset", yaw_params.tunnel.yaw_offset);
  get_parameter("yaw_tunnel_clearance_enter", yaw_params.tunnel.clearance_enter);
  get_parameter("yaw_tunnel_clearance_exit", yaw_params.tunnel.clearance_exit);
  get_parameter("yaw_tunnel_max_width", yaw_params.tunnel.max_width);
  get_parameter("yaw_tunnel_axis_window", yaw_params.tunnel.axis_window);
  get_parameter("yaw_tunnel_axis_min_coherence", yaw_params.tunnel.min_coherence);
  get_parameter("yaw_tunnel_pre_align_distance", yaw_params.tunnel.pre_align_distance);
  get_parameter("yaw_tunnel_post_release_distance", yaw_params.tunnel.post_release_distance);
  get_parameter("yaw_tunnel_axis_tolerance", yaw_params.tunnel.axis_tolerance);
  yaw_params.tunnel.axis_tolerance = std::max(0.01, yaw_params.tunnel.axis_tolerance);
  get_parameter("yaw_tunnel_side_wall_gradient_min", yaw_params.tunnel.side_wall_gradient_min);
  yaw_params.tunnel.side_wall_gradient_min = std::clamp(
    yaw_params.tunnel.side_wall_gradient_min, 0.0, 1.0);
  get_parameter("yaw_tunnel_side_wall_cos2_min", yaw_params.tunnel.side_wall_cos2_min);
  yaw_params.tunnel.side_wall_cos2_min = std::clamp(
    yaw_params.tunnel.side_wall_cos2_min, 0.0, 1.0);
  get_parameter("yaw_tunnel_sidewall_pca_enabled", yaw_params.tunnel.sidewall_pca_enabled);
  get_parameter(
    "yaw_tunnel_dynamic_pre_align_enabled", yaw_params.tunnel.dynamic_pre_align_enabled);
  get_parameter(
    "yaw_tunnel_terminal_goal_axial_override_enabled",
    yaw_params.tunnel.terminal_goal_axial_override_enabled);
  get_parameter("yaw_tunnel_armed_distance", yaw_params.tunnel.armed_distance);
  yaw_params.tunnel.armed_distance = std::max(0.0, yaw_params.tunnel.armed_distance);
  get_parameter("yaw_tunnel_pre_align_time_margin", yaw_params.tunnel.pre_align_time_margin);
  yaw_params.tunnel.pre_align_time_margin = std::max(0.0, yaw_params.tunnel.pre_align_time_margin);
  get_parameter("yaw_tunnel_lock_gimbal", tunnel_lock_gimbal_);
  get_parameter("terminal_yaw_relocation_enabled", terminal_yaw_relocation_enabled_);
  get_parameter("footprint_yaw_refinement_rounds", footprint_yaw_refinement_rounds_);
  footprint_yaw_refinement_rounds_ = std::max(0, footprint_yaw_refinement_rounds_);
  get_parameter(
    "terminal_yaw_relocation_max_candidates", terminal_yaw_relocation_max_candidates_);
  terminal_yaw_relocation_max_candidates_ = std::max(
    1, terminal_yaw_relocation_max_candidates_);
  // 重定位必须与 yaw 规划用同一组转向参数,否则两条路径生成的原地转向时长
  // 不一致,MPC 侧会看到两种不同的终端时间预算。
  terminal_yaw_relocation_params_.yaw_rate_limit = yaw_params.yaw_rate_limit;
  terminal_yaw_relocation_params_.yaw_acceleration_limit = yaw_params.yaw_acceleration_limit;
  terminal_yaw_relocation_params_.sample_period = yaw_params.terminal_yaw_sample_period;
  spin_yaw_params_.sample_period = yaw_params.terminal_yaw_sample_period;
  get_parameter(
    "terminal_yaw_relocation_window_length",
    terminal_yaw_relocation_params_.terminal_window_length);
  get_parameter(
    "terminal_rotation_comfortable_clearance_m", terminal_rotation_comfortable_clearance_m_);
  terminal_rotation_comfortable_clearance_m_ =
    std::max(0.0, terminal_rotation_comfortable_clearance_m_);
  get_parameter(
    "terminal_rotation_tight_yaw_span_rad", terminal_rotation_tight_yaw_span_rad_);
  terminal_rotation_tight_yaw_span_rad_ = std::max(0.05, terminal_rotation_tight_yaw_span_rad_);
  get_parameter(
    "sidestep_min_duration", terminal_yaw_relocation_params_.sidestep_min_duration);
  terminal_yaw_relocation_params_.sidestep_min_duration =
    std::max(0.05, terminal_yaw_relocation_params_.sidestep_min_duration);
  get_parameter(
    "sidestep_max_speed", terminal_yaw_relocation_params_.sidestep_max_speed);
  terminal_yaw_relocation_params_.sidestep_max_speed =
    std::max(0.1, terminal_yaw_relocation_params_.sidestep_max_speed);
  get_parameter(
    "sidestep_max_accel", terminal_yaw_relocation_params_.sidestep_max_accel);
  terminal_yaw_relocation_params_.sidestep_max_accel =
    std::max(0.2, terminal_yaw_relocation_params_.sidestep_max_accel);
  get_parameter("sidestep_min_esdf_margin_m", sidestep_min_esdf_margin_m_);
  sidestep_min_esdf_margin_m_ = std::max(0.0, sidestep_min_esdf_margin_m_);
  get_parameter("sidestep_desired_esdf_margin_m", sidestep_desired_esdf_margin_m_);
  sidestep_desired_esdf_margin_m_ =
    std::max(sidestep_min_esdf_margin_m_, sidestep_desired_esdf_margin_m_);
  get_parameter("sidestep_max_radius_m", sidestep_max_radius_m_);
  sidestep_max_radius_m_ = std::max(0.15, sidestep_max_radius_m_);
  get_parameter("sidestep_max_tries", sidestep_max_tries_);
  sidestep_max_tries_ = std::max(1, sidestep_max_tries_);
  get_parameter("sidestep_accept_clearance_m", sidestep_accept_clearance_m_);
  sidestep_accept_clearance_m_ = std::max(0.0, sidestep_accept_clearance_m_);
  get_parameter("sidestep_min_improvement_m", sidestep_min_improvement_m_);
  sidestep_min_improvement_m_ = std::max(0.0, sidestep_min_improvement_m_);
  get_parameter("rolling_horizon_enabled", rolling_horizon_enabled_);
  get_parameter("rolling_horizon_length_m", rolling_horizon_length_m_);
  rolling_horizon_length_m_ = std::max(2.0, rolling_horizon_length_m_);
  get_parameter("rolling_min_tail_length_m", rolling_min_tail_length_m_);
  rolling_min_tail_length_m_ = std::max(0.5, rolling_min_tail_length_m_);
  get_parameter(
    "partial_subgoal_continuation_enabled", partial_subgoal_continuation_enabled_);
  get_parameter(
    "partial_subgoal_only_without_prior", partial_subgoal_only_without_prior_);
  get_parameter("partial_subgoal_min_advance_m", partial_subgoal_min_advance_m_);
  partial_subgoal_min_advance_m_ = std::max(0.15, partial_subgoal_min_advance_m_);
  get_parameter("optimistic_global_unknown_cost", optimistic_global_unknown_cost_);
  optimistic_global_unknown_cost_ = std::max(1.0, optimistic_global_unknown_cost_);
  get_parameter("frontier_taboo_radius_m", frontier_taboo_radius_m_);
  frontier_taboo_radius_m_ = std::max(0.1, frontier_taboo_radius_m_);
  get_parameter("frontier_taboo_ttl_sec", frontier_taboo_ttl_sec_);
  frontier_taboo_ttl_sec_ = std::max(1.0, frontier_taboo_ttl_sec_);
  get_parameter("footprint_refinement_skip_clearance_m", footprint_refinement_skip_clearance_m_);
  footprint_refinement_skip_clearance_m_ = std::max(0.0, footprint_refinement_skip_clearance_m_);
  get_parameter("minco_stage_budget_ms", minco_stage_budget_ms_);
  minco_stage_budget_ms_ = std::max(1.0, minco_stage_budget_ms_);
  get_parameter("plan_wall_budget_ms", plan_wall_budget_ms_);
  plan_wall_budget_ms_ = std::max(1.0, plan_wall_budget_ms_);
  get_parameter("rolling_replan_wall_budget_ms", rolling_replan_wall_budget_ms_);
  rolling_replan_wall_budget_ms_ = std::max(0.0, rolling_replan_wall_budget_ms_);
  get_parameter("plan_hard_wall_budget_ms", plan_hard_wall_budget_ms_);
  plan_hard_wall_budget_ms_ = std::max(0.0, plan_hard_wall_budget_ms_);
  get_parameter("footprint_length", footprint_params.length);
  get_parameter("footprint_width", footprint_params.width);
  get_parameter("footprint_safety_margin", footprint_params.safety_margin);
  get_parameter(
    "swept_max_corner_step_cells", footprint_params.swept_max_corner_step_cells);
  footprint_params.swept_max_corner_step_cells = std::max(
    1e-3, footprint_params.swept_max_corner_step_cells);
  get_parameter(
    "adaptive_lipschitz_swept_enabled", footprint_params.adaptive_lipschitz_swept_enabled);
  get_parameter("swept_min_corner_step_m", footprint_params.swept_min_corner_step_m);
  footprint_params.swept_min_corner_step_m = std::max(
    0.005, footprint_params.swept_min_corner_step_m);
  footprint_length_ = footprint_params.length;
  footprint_width_ = footprint_params.width;
  footprint_safety_margin_ = footprint_params.safety_margin;
  optimizer_params.footprint_length = footprint_length_;
  optimizer_params.footprint_width = footprint_width_;
  optimizer_params.footprint_safety_margin = footprint_safety_margin_;
  const double all_yaw_footprint_radius = std::hypot(
    0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_,
    0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_);
  if (astar_params.safe_distance + 1e-6 < all_yaw_footprint_radius) {
    RCLCPP_WARN(
      get_logger(),
      "【规划参数修正】图搜索优选净空由 %.3f m 提高到矩形全航向足迹半径 %.3f m。",
      astar_params.safe_distance, all_yaw_footprint_radius);
    astar_params.safe_distance = all_yaw_footprint_radius;
  }
  // The all-yaw radius is the circumscribed disc: it demands room to spin in
  // place at every cell. A rectangular body aligned with a corridor only needs
  // the inscribed half-width, so refusing to search below the circumscribed
  // radius makes tight-but-passable corridors permanently unreachable. The
  // ladder keeps the circumscribed radius as the preferred level and only falls
  // back to the inscribed one when nothing is reachable, leaving the yaw-aware
  // footprint gate and local collision repair as the authoritative safety check.
  const double inscribed_footprint_radius =
    0.5 * std::max(0.0, std::min(footprint_length_, footprint_width_)) +
    footprint_safety_margin_;
  preferred_search_clearance_ = astar_params.safe_distance;
  get_parameter("clearance_relaxation_enabled", clearance_relaxation_enabled_);
  get_parameter("clearance_detour_ratio", clearance_detour_ratio_);
  get_parameter(
    "endpoint_clearance_relaxation_enabled", endpoint_clearance_relaxation_enabled_);
  double configured_floor = -1.0;
  get_parameter("search_clearance_floor", configured_floor);
  inscribed_footprint_radius_ = inscribed_footprint_radius;
  search_clearance_floor_configured_ = configured_floor >= 0.0;
  // 这里还不知道规划栅格分辨率,所以自动下限先记成内切半宽;真正与 footprint gate
  // 一致的下限在 runGraphSearch 里按当次 snapshot 的分辨率补上半个格对角线。
  search_clearance_floor_ = configured_floor < 0.0
    ? std::min(inscribed_footprint_radius, preferred_search_clearance_)
    : std::min(std::max(0.0, configured_floor), preferred_search_clearance_);
  // 自动下限依赖当次 snapshot 的栅格分辨率,启动时还取不到,所以这里明确标注下限是
  // "按 snapshot 解析"的,不要让这行 INFO 被当成实际生效值。
  RCLCPP_INFO(
    get_logger(),
    "【图搜索净空梯度】优选=%.3f m 下限=%.3f m（%s）放宽=%s",
    preferred_search_clearance_, search_clearance_floor_,
    search_clearance_floor_configured_
      ? "显式配置" : "内切半径；每个快照另加栅格量化余量",
    clearance_relaxation_enabled_ ? "开启" : "关闭");
  astar_params.min_safe_distance = search_clearance_floor_;
  get_parameter("local_repair_enabled", repair_params.enabled);
  get_parameter("local_repair_max_iterations", repair_params.max_iterations);
  get_parameter("local_repair_search_radius", repair_params.search_radius);

  footprint_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  footprint_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  repair_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  repair_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  // 拒绝轨迹的是 yaw 相关矩形足迹门禁,所以修复必须按同一
  // 套几何挑格子:候选格中心周围一个外接圆半径内全空,矩形
  // 在任意 yaw 下都放得下。只按"该格空闲"会挑回原地,domain
  // 193 目标 8 就是这样每 2 s 拒同一条轨迹。找不到合格候选时
  // 返回 false,行为与今天完全一致(拒绝),方向是 fail-closed。
  repair_params.required_clearance_m = all_yaw_footprint_radius;
  // 严格档在实机栅格上常是空集(domain 169 目标 9 连续 89 次拒绝、domain 171
  // 目标 8 同样,一次都没挑出候选格)。第二档按内切半宽推 footprint 一致下限,
  // 与图搜索梯子共用几何;最终安全性仍由重解 MINCO 后的矩形足迹门禁裁定。
  repair_params.inscribed_radius_m = inscribed_footprint_radius_;
  if (repair_params.enabled &&
    repair_params.search_radius + 1e-6 < repair_params.inscribed_radius_m)
  {
    // 连第二档都够不到时修复必然是空集。这里显式告警而不是悄悄放宽搜索窗口:
    // search_radius 是用户对引导点位移的上界,由配置负责,不由代码覆盖。
    RCLCPP_WARN(
      get_logger(),
      "【局部修复参数告警】搜索半径 %.3f m 小于足迹一致净空下限（内切半径=%.3f m）；"
      "无法选出候选栅格，拒绝轨迹不会被修复。",
      repair_params.search_radius, repair_params.inscribed_radius_m);
  }

  yaw_params.tunnel.forward_width_limit = std::hypot(
    footprint_length_ + 2.0 * footprint_safety_margin_,
    footprint_width_ + 2.0 * footprint_safety_margin_);
  yaw_params.tunnel.front_extent = 0.5 * footprint_length_ + footprint_safety_margin_;
  yaw_params.tunnel.circumscribed_radius = all_yaw_footprint_radius;
  tunnel_yaw_params_ = yaw_params.tunnel;
  graph_search_.setParams(astar_params);
  optimizer_params_cached_speed_ = optimizer_params.max_velocity > 0.0 ?
    std::min(optimizer_params.max_velocity, optimizer_params.reference_speed) :
    optimizer_params.reference_speed;
  optimizer_params_cached_max_accel_ = optimizer_params.max_acceleration;
  optimizer_params_cached_lat_accel_ = optimizer_params.max_lateral_acceleration;
  optimizer_params.clearance_speed_min = wheel_speed_time_scaling_params_.clearance_speed_min;
  optimizer_params.clearance_speed_low = wheel_speed_time_scaling_params_.clearance_speed_low;
  optimizer_params.clearance_speed_high = wheel_speed_time_scaling_params_.clearance_speed_high;
  wheel_speed_time_scaling_params_.clearance_speed_max = optimizer_params_cached_speed_;
  optimizer_.setParams(optimizer_params);
  // 4 阶对称（切线 ±pi/2 等价）只对正方形足迹成立；非正方形时 ±pi/2 会把长边横进
  // 通道，这里回退到 2 阶，fail-closed。
  const bool square_footprint = std::abs(footprint_length_ - footprint_width_) <= 1e-3;
  if (yaw_params.tangent_symmetry_order != 2 &&
    !(yaw_params.tangent_symmetry_order == 4 && square_footprint))
  {
    RCLCPP_WARN(
      get_logger(),
      "【航向参数告警】yaw_tangent_symmetry_order=%d 不适用于足迹 %.3f x %.3f m，已回退为 2。",
      yaw_params.tangent_symmetry_order, footprint_length_, footprint_width_);
    yaw_params.tangent_symmetry_order = 2;
  }
  yaw_planner_.setParams(yaw_params);
  YawSplinePlannerParams narrow_start_hold_params = yaw_params;
  narrow_start_hold_params.narrow_start_hold_length = narrow_start_hold_length_;
  narrow_start_hold_yaw_planner_.setParams(narrow_start_hold_params);
  // 缓存一份给目标位姿准入用，保证两者的矩形几何完全一致。
  footprint_params_ = footprint_params;
  safety_checker_.setParams(footprint_params);
  collision_repair_.setParams(repair_params);
}


}  // namespace minco_planner
