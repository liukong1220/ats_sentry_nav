// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp_relocalization/localization_status_input_core.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  // 恢复期（LOST 多猜测、恢复中的粗配准）与启动时全局地图预处理用的线程数；<=0 时与 num_threads 相同
  this->declare_parameter("recovery_num_threads", 0);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("min_source_points", 500);
  this->declare_parameter("min_inliers", 200);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_registration_error", -1.0);
  this->declare_parameter("relax_convergence_for_sim", false);
  this->declare_parameter("log_registration_details", true);
  this->declare_parameter("log_throttle_ms", 2000);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("confirmation_count", 2);
  this->declare_parameter("confirmation_timeout_s", 10.0);
  this->declare_parameter("confirmation_translation_tolerance", 0.15);
  this->declare_parameter("confirmation_yaw_tolerance", 0.10);
  this->declare_parameter("confirmation_min_interval_s", 0.05);
  this->declare_parameter("confirmation_motion_translation_tolerance", 0.25);
  this->declare_parameter("confirmation_motion_yaw_tolerance", 0.15);
  this->declare_parameter("registration_interval_s", 0.25);
  this->declare_parameter("transform_publish_rate_hz", 50.0);
  this->declare_parameter("max_accumulation_age_s", 0.30);
  this->declare_parameter("min_registration_translation_delta", 0.10);
  this->declare_parameter("min_registration_yaw_delta", 0.12);
  this->declare_parameter("accepted_observation_refresh_interval_s", 1.0);
  this->declare_parameter("initial_pose_force_registration_window_s", 2.0);
  this->declare_parameter("transform_future_offset_s", 0.25);
  this->declare_parameter("max_scan_stamp_lag_s", 0.25);
  this->declare_parameter("max_scan_age_s", 1.0);
  this->declare_parameter("max_scan_future_s", 0.10);
  this->declare_parameter("scan_min_range_m", 0.10);
  this->declare_parameter("scan_max_range_m", 100.0);
  this->declare_parameter("scan_min_z_m", -5.0);
  this->declare_parameter("scan_max_z_m", 5.0);
  this->declare_parameter("min_scan_valid_ratio", 0.50);
  this->declare_parameter("max_accumulated_points", 40000);
  this->declare_parameter("max_accumulated_frames", 30);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  mapping_mode_ = this->declare_parameter<bool>("mapping_mode", false);
  require_odom_identity_ = this->declare_parameter<bool>("require_odom_identity", false);
  const auto growing_capacity = this->declare_parameter<int>("mapping.max_voxels", 500000);
  const auto growing_size = this->declare_parameter<std::vector<double>>(
    "mapping.size", {40.0, 30.0, 20.0});
  const auto growing_origin = this->declare_parameter<std::vector<double>>(
    "mapping.origin", {-20.0, -15.0, -10.0});
  if (growing_size.size() != 3) {throw std::invalid_argument("mapping.size requires xyz");}
  if (growing_origin.size() != 3) {throw std::invalid_argument("mapping.origin requires xyz");}
  if (growing_capacity <= 0) {throw std::invalid_argument("mapping.max_voxels must be positive");}
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  // Coarse-to-fine windowed registration (borrowed from BIT icp_relocalization).
  this->declare_parameter("registration_mode", "initial_guess");
  this->declare_parameter("accumulate_frames", 1);
  this->declare_parameter("fine_alignment.enable", true);
  this->declare_parameter("fine_alignment.coarse_first_window_only", true);
  this->declare_parameter("fine_alignment.max_correspondence_distance", 0.45);
  this->declare_parameter("coarse_max_iterations", 10);
  this->declare_parameter("fine_max_iterations", 16);
  // Quality gates (borrowed from HWSentry odom_localizer). 0 disables a gate.
  this->declare_parameter("min_overlap_ratio", 0.0);
  this->declare_parameter("min_information_eigenvalue", 0.0);
  this->declare_parameter("max_information_condition_number", 0.0);
  this->declare_parameter("multi_guess.search_half_xy", 2.0);
  this->declare_parameter("multi_guess.step_xy", 1.0);
  this->declare_parameter("multi_guess.step_yaw", 0.785398);
  this->declare_parameter("multi_guess.z_candidates", std::vector<double>{0.0});
  this->declare_parameter("multi_guess.time_budget_s", 1.5);
  this->declare_parameter("multi_guess.fine_time_reserve_s", 0.35);
  this->declare_parameter("multi_guess.max_candidates_per_scan", 48);
  this->declare_parameter("multi_guess.max_fine_per_scan", 4);
  // coarse 筛选专用的稀疏尺度。<=registered_leaf_size 时不建立独立筛选云。
  this->declare_parameter("multi_guess.screen_leaf_size", 0.0);
  this->declare_parameter("multi_guess.screen_min_inliers", 0);
  this->declare_parameter("multi_guess.log_candidates", false);
  this->declare_parameter("multi_guess.candidate_log_path", "");
  // 组合评分权重与饱和尺度。各项归一化到 [0,1]，总分范围 [0, sum(weights)]。
  this->declare_parameter("candidate_score.weight_error", 1.0);
  this->declare_parameter("candidate_score.weight_overlap", 1.0);
  this->declare_parameter("candidate_score.weight_information", 0.5);
  this->declare_parameter("candidate_score.weight_motion", 0.5);
  this->declare_parameter("candidate_score.weight_prior", 0.2);
  this->declare_parameter("candidate_score.error_scale", 1.0);
  this->declare_parameter("candidate_score.information_eigenvalue_reference", 1.0);
  this->declare_parameter("candidate_score.condition_number_reference", 1.0e4);
  this->declare_parameter("candidate_score.motion_scale", 0.5);
  this->declare_parameter("candidate_score.prior_scale", 2.0);
  this->declare_parameter("candidate_score.motion_reference_max_age_s", 2.0);
  // <=0 表示歧义门未启用；阈值必须由 correct/wrong 候选分布决定。
  this->declare_parameter("ambiguity.min_score_margin", 0.0);
  this->declare_parameter("ambiguity.min_separation_xy", 0.5);
  this->declare_parameter("ambiguity.min_separation_yaw", 0.35);
  this->declare_parameter("follow_localization_status", true);
  this->declare_parameter("auto_multi_guess_on_lost", true);
  this->declare_parameter("force_registration_when_lost", true);
  this->declare_parameter("status_stale_skip_registration_s", 1.0);
  this->declare_parameter("cold_start_prior_max_xy_m", 1.0);
  this->declare_parameter("cold_start_prior_max_yaw_rad", 0.60);
  this->declare_parameter("cold_start_seed_attempts", 3);
  this->declare_parameter("height_filter.enable", false);
  this->declare_parameter("height_filter.min_z", -0.5);
  this->declare_parameter("height_filter.max_z", 2.5);
  this->declare_parameter("local_submap.enable", local_submap_enable_);
  this->declare_parameter("local_submap.radius_m", local_submap_radius_m_);
  this->declare_parameter("local_submap.recenter_distance_m", local_submap_recenter_distance_m_);
  this->declare_parameter(
    "local_submap.min_points", static_cast<std::int64_t>(local_submap_min_points_));
  this->declare_parameter(
    "degeneracy.weak_translation_eigenvalue", weak_translation_eigenvalue_threshold_);
  this->declare_parameter("degeneracy.weak_yaw_eigenvalue", weak_yaw_eigenvalue_threshold_);
  this->declare_parameter(
    "degeneracy.max_translation_condition_ratio", max_translation_condition_ratio_);
  this->declare_parameter(
    "degeneracy.weak_translation_per_inlier", weak_translation_per_inlier_threshold_);
  this->declare_parameter("tracking_max_motion_residual", tracking_max_motion_residual_);
  this->declare_parameter("tracking_max_prior_deviation", tracking_max_prior_deviation_);
  this->declare_parameter("scan_context.enable", scan_context_enable_);
  this->declare_parameter("scan_context.grid_step_xy", 2.0);
  this->declare_parameter("scan_context.max_radius_m", 15.0);
  this->declare_parameter("scan_context.top_k", scan_context_top_k_);
  this->declare_parameter("scan_context.max_distance", scan_context_max_distance_);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("recovery_num_threads", recovery_num_threads_);
  num_threads_ = std::max(1, num_threads_);
  recovery_num_threads_ = recovery_num_threads_ > 0 ? recovery_num_threads_ : num_threads_;
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_inliers", min_inliers_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_registration_error", max_registration_error_);
  this->get_parameter("relax_convergence_for_sim", relax_convergence_for_sim_);
  this->get_parameter("log_registration_details", log_registration_details_);
  std::int64_t configured_log_throttle_ms = static_cast<std::int64_t>(log_throttle_ms_);
  this->get_parameter("log_throttle_ms", configured_log_throttle_ms);
  log_throttle_ms_ = static_cast<std::uint64_t>(
    std::clamp<std::int64_t>(configured_log_throttle_ms, 250, 60000));
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("confirmation_count", confirmation_count_);
  this->get_parameter("confirmation_timeout_s", confirmation_timeout_s_);
  if (!std::isfinite(confirmation_timeout_s_) || confirmation_timeout_s_ <= 0.0) {
    throw std::invalid_argument("confirmation_timeout_s must be finite and positive");
  }
  this->get_parameter("confirmation_translation_tolerance", confirmation_translation_tolerance_);
  this->get_parameter("confirmation_yaw_tolerance", confirmation_yaw_tolerance_);
  this->get_parameter("confirmation_min_interval_s", confirmation_min_interval_s_);
  this->get_parameter(
    "confirmation_motion_translation_tolerance", confirmation_motion_translation_tolerance_);
  this->get_parameter("confirmation_motion_yaw_tolerance", confirmation_motion_yaw_tolerance_);
  confirmation_count_ = std::max(1, confirmation_count_);
  confirmation_translation_tolerance_ = std::max(0.0, confirmation_translation_tolerance_);
  confirmation_yaw_tolerance_ = std::max(0.0, confirmation_yaw_tolerance_);
  confirmation_min_interval_s_ = std::max(0.0, confirmation_min_interval_s_);
  confirmation_motion_translation_tolerance_ =
    std::max(0.0, confirmation_motion_translation_tolerance_);
  confirmation_motion_yaw_tolerance_ = std::max(0.0, confirmation_motion_yaw_tolerance_);
  this->get_parameter("registration_interval_s", registration_interval_s_);
  this->get_parameter("transform_publish_rate_hz", transform_publish_rate_hz_);
  if (!std::isfinite(transform_publish_rate_hz_) || transform_publish_rate_hz_ <= 0.0) {
    transform_publish_rate_hz_ = 50.0;
  }
  transform_publish_rate_hz_ = std::clamp(transform_publish_rate_hz_, 1.0, 200.0);
  this->get_parameter("max_accumulation_age_s", max_accumulation_age_s_);
  this->get_parameter("min_registration_translation_delta", min_registration_translation_delta_);
  this->get_parameter("min_registration_yaw_delta", min_registration_yaw_delta_);
  this->get_parameter(
    "accepted_observation_refresh_interval_s", accepted_observation_refresh_interval_s_);
  accepted_observation_refresh_interval_s_ =
    std::max(0.0, accepted_observation_refresh_interval_s_);
  this->get_parameter(
    "initial_pose_force_registration_window_s", initial_pose_force_registration_window_s_);
  this->get_parameter("transform_future_offset_s", transform_future_offset_s_);
  this->get_parameter("max_scan_stamp_lag_s", max_scan_stamp_lag_s_);
  this->get_parameter("max_scan_age_s", max_scan_age_s_);
  this->get_parameter("max_scan_future_s", max_scan_future_s_);
  this->get_parameter("scan_min_range_m", scan_min_range_m_);
  this->get_parameter("scan_max_range_m", scan_max_range_m_);
  this->get_parameter("scan_min_z_m", scan_min_z_m_);
  this->get_parameter("scan_max_z_m", scan_max_z_m_);
  this->get_parameter("min_scan_valid_ratio", min_scan_valid_ratio_);
  std::int64_t max_accumulated_points = static_cast<std::int64_t>(max_accumulated_points_);
  std::int64_t max_accumulated_frames = static_cast<std::int64_t>(max_accumulated_frames_);
  this->get_parameter("max_accumulated_points", max_accumulated_points);
  this->get_parameter("max_accumulated_frames", max_accumulated_frames);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  this->get_parameter("registration_mode", registration_mode_);
  this->get_parameter("accumulate_frames", accumulate_frames_);
  this->get_parameter("fine_alignment.enable", fine_alignment_enabled_);
  this->get_parameter("fine_alignment.coarse_first_window_only", coarse_first_window_only_);
  double fine_max_correspondence_distance = 0.45;
  this->get_parameter(
    "fine_alignment.max_correspondence_distance", fine_max_correspondence_distance);
  this->get_parameter("coarse_max_iterations", coarse_max_iterations_);
  this->get_parameter("fine_max_iterations", fine_max_iterations_);
  this->get_parameter("min_overlap_ratio", min_overlap_ratio_);
  this->get_parameter("min_information_eigenvalue", min_information_eigenvalue_);
  this->get_parameter("max_information_condition_number", max_information_condition_number_);
  this->get_parameter("multi_guess.search_half_xy", multi_guess_search_half_xy_);
  this->get_parameter("multi_guess.step_xy", multi_guess_step_xy_);
  this->get_parameter("multi_guess.step_yaw", multi_guess_step_yaw_);
  this->get_parameter("multi_guess.z_candidates", multi_guess_z_candidates_);
  this->get_parameter("multi_guess.time_budget_s", multi_guess_time_budget_s_);
  this->get_parameter("multi_guess.fine_time_reserve_s", multi_guess_fine_time_reserve_s_);
  this->get_parameter("multi_guess.max_candidates_per_scan", multi_guess_max_candidates_per_scan_);
  this->get_parameter("multi_guess.max_fine_per_scan", multi_guess_max_fine_per_scan_);
  multi_guess_screen_leaf_size_ =
    static_cast<float>(this->get_parameter("multi_guess.screen_leaf_size").as_double());
  this->get_parameter("multi_guess.screen_min_inliers", multi_guess_screen_min_inliers_);
  this->get_parameter("multi_guess.log_candidates", multi_guess_log_candidates_);
  this->get_parameter("multi_guess.candidate_log_path", multi_guess_candidate_log_path_);
  this->get_parameter("candidate_score.weight_error", score_weights_.error);
  this->get_parameter("candidate_score.weight_overlap", score_weights_.overlap);
  this->get_parameter("candidate_score.weight_information", score_weights_.information);
  this->get_parameter("candidate_score.weight_motion", score_weights_.motion);
  this->get_parameter("candidate_score.weight_prior", score_weights_.prior);
  this->get_parameter("candidate_score.error_scale", score_weights_.error_scale);
  this->get_parameter(
    "candidate_score.information_eigenvalue_reference",
    score_weights_.information_eigenvalue_reference);
  this->get_parameter(
    "candidate_score.condition_number_reference", score_weights_.condition_number_reference);
  this->get_parameter("candidate_score.motion_scale", score_weights_.motion_scale);
  this->get_parameter("candidate_score.prior_scale", score_weights_.prior_scale);
  this->get_parameter("candidate_score.motion_reference_max_age_s", motion_reference_max_age_s_);
  this->get_parameter("ambiguity.min_score_margin", ambiguity_.min_score_margin);
  this->get_parameter("ambiguity.min_separation_xy", ambiguity_.min_separation_xy);
  this->get_parameter("ambiguity.min_separation_yaw", ambiguity_.min_separation_yaw);
  this->get_parameter("follow_localization_status", follow_localization_status_);
  this->get_parameter("auto_multi_guess_on_lost", auto_multi_guess_on_lost_);
  this->get_parameter("force_registration_when_lost", force_registration_when_lost_);
  this->get_parameter("status_stale_skip_registration_s", status_stale_skip_registration_s_);
  this->get_parameter("cold_start_prior_max_xy_m", cold_start_prior_max_xy_m_);
  this->get_parameter("cold_start_prior_max_yaw_rad", cold_start_prior_max_yaw_rad_);
  this->get_parameter("cold_start_seed_attempts", cold_start_seed_attempts_);
  cold_start_seed_attempts_ = std::max(0, cold_start_seed_attempts_);
  status_stale_skip_registration_s_ = std::max(0.0, status_stale_skip_registration_s_);
  cold_start_prior_max_xy_m_ = std::max(0.0, cold_start_prior_max_xy_m_);
  cold_start_prior_max_yaw_rad_ = std::max(0.0, cold_start_prior_max_yaw_rad_);
  max_scan_age_s_ = std::max(0.0, max_scan_age_s_);
  max_scan_future_s_ = std::max(0.0, max_scan_future_s_);
  scan_min_range_m_ = std::max(0.0, scan_min_range_m_);
  scan_max_range_m_ = std::max(scan_min_range_m_, scan_max_range_m_);
  if (!std::isfinite(scan_min_z_m_) || !std::isfinite(scan_max_z_m_)) {
    throw std::invalid_argument("scan z limits must be finite");
  }
  scan_max_z_m_ = std::max(scan_min_z_m_, scan_max_z_m_);
  min_scan_valid_ratio_ = std::clamp(min_scan_valid_ratio_, 0.0, 1.0);
  if (
    max_accumulated_points <= 0 || max_accumulated_frames <= 0 ||
    max_accumulated_frames > std::numeric_limits<int>::max() || accumulate_frames_ <= 0) {
    throw std::invalid_argument("scan accumulation limits must be positive and representable");
  }
  max_accumulated_points_ = static_cast<std::size_t>(max_accumulated_points);
  max_accumulated_frames_ = static_cast<int>(max_accumulated_frames);
  scan_window_ = decltype(scan_window_)(
    max_accumulated_points_, max_accumulated_frames_, accumulate_frames_);
  double height_min_z = -0.5;
  double height_max_z = 2.5;
  this->get_parameter("height_filter.enable", height_filter_enable_);
  this->get_parameter("height_filter.min_z", height_min_z);
  this->get_parameter("height_filter.max_z", height_max_z);
  height_filter_min_z_ = static_cast<float>(height_min_z);
  height_filter_max_z_ = static_cast<float>(std::max(height_min_z + 1e-3, height_max_z));
  this->get_parameter("local_submap.enable", local_submap_enable_);
  this->get_parameter("local_submap.radius_m", local_submap_radius_m_);
  local_submap_radius_m_ = std::max(4.0, local_submap_radius_m_);
  this->get_parameter("local_submap.recenter_distance_m", local_submap_recenter_distance_m_);
  local_submap_recenter_distance_m_ = std::max(0.5, local_submap_recenter_distance_m_);
  std::int64_t submap_min_pts = static_cast<std::int64_t>(local_submap_min_points_);
  this->get_parameter("local_submap.min_points", submap_min_pts);
  local_submap_min_points_ = static_cast<std::size_t>(std::max<std::int64_t>(100, submap_min_pts));
  this->get_parameter(
    "degeneracy.weak_translation_eigenvalue", weak_translation_eigenvalue_threshold_);
  weak_translation_eigenvalue_threshold_ = std::max(0.0, weak_translation_eigenvalue_threshold_);
  this->get_parameter("degeneracy.weak_yaw_eigenvalue", weak_yaw_eigenvalue_threshold_);
  weak_yaw_eigenvalue_threshold_ = std::max(0.0, weak_yaw_eigenvalue_threshold_);
  this->get_parameter(
    "degeneracy.max_translation_condition_ratio", max_translation_condition_ratio_);
  max_translation_condition_ratio_ = std::max(0.0, max_translation_condition_ratio_);
  this->get_parameter(
    "degeneracy.weak_translation_per_inlier", weak_translation_per_inlier_threshold_);
  weak_translation_per_inlier_threshold_ = std::max(0.0, weak_translation_per_inlier_threshold_);
  this->get_parameter("tracking_max_motion_residual", tracking_max_motion_residual_);
  tracking_max_motion_residual_ = std::max(0.0, tracking_max_motion_residual_);
  this->get_parameter("tracking_max_prior_deviation", tracking_max_prior_deviation_);
  tracking_max_prior_deviation_ = std::max(0.0, tracking_max_prior_deviation_);

  ScanContextConfig sc_cfg;
  this->get_parameter("scan_context.enable", scan_context_enable_);
  this->get_parameter("scan_context.grid_step_xy", sc_cfg.grid_step_xy_m);
  this->get_parameter("scan_context.max_radius_m", sc_cfg.max_radius_m);
  this->get_parameter("scan_context.top_k", scan_context_top_k_);
  scan_context_top_k_ = std::clamp(scan_context_top_k_, 1, 16);
  this->get_parameter("scan_context.max_distance", scan_context_max_distance_);
  scan_context_max_distance_ = std::clamp(scan_context_max_distance_, 0.05, 1.0);
  scan_context_db_.setConfig(sc_cfg);

  coarse_max_iterations_ = std::max(1, coarse_max_iterations_);
  fine_max_iterations_ = std::max(1, fine_max_iterations_);
  fine_max_correspondence_distance = std::max(1e-3, fine_max_correspondence_distance);
  fine_max_dist_sq_ =
    static_cast<float>(fine_max_correspondence_distance * fine_max_correspondence_distance);
  min_overlap_ratio_ = std::clamp(min_overlap_ratio_, 0.0, 1.0);
  min_information_eigenvalue_ = std::max(0.0, min_information_eigenvalue_);
  max_information_condition_number_ = std::max(0.0, max_information_condition_number_);
  multi_guess_search_half_xy_ = std::max(0.0, multi_guess_search_half_xy_);
  multi_guess_step_xy_ = std::max(0.05, multi_guess_step_xy_);
  multi_guess_step_yaw_ = std::max(0.05, multi_guess_step_yaw_);
  multi_guess_time_budget_s_ = std::max(0.05, multi_guess_time_budget_s_);
  multi_guess_fine_time_reserve_s_ = std::clamp(
    multi_guess_fine_time_reserve_s_, 0.05, std::max(0.05, multi_guess_time_budget_s_ - 0.01));
  multi_guess_max_candidates_per_scan_ = std::max(1, multi_guess_max_candidates_per_scan_);
  multi_guess_screen_min_inliers_ = std::max(0, multi_guess_screen_min_inliers_);
  score_weights_.error = std::max(0.0, score_weights_.error);
  score_weights_.overlap = std::max(0.0, score_weights_.overlap);
  score_weights_.information = std::max(0.0, score_weights_.information);
  score_weights_.motion = std::max(0.0, score_weights_.motion);
  score_weights_.prior = std::max(0.0, score_weights_.prior);
  score_weights_.error_scale = std::max(1e-6, score_weights_.error_scale);
  score_weights_.information_eigenvalue_reference =
    std::max(1e-9, score_weights_.information_eigenvalue_reference);
  score_weights_.condition_number_reference =
    std::max(1e-6, score_weights_.condition_number_reference);
  score_weights_.motion_scale = std::max(1e-6, score_weights_.motion_scale);
  score_weights_.prior_scale = std::max(1e-6, score_weights_.prior_scale);
  ambiguity_.min_separation_xy = std::max(0.0, ambiguity_.min_separation_xy);
  ambiguity_.min_separation_yaw = std::max(0.0, ambiguity_.min_separation_yaw);
  motion_reference_max_age_s_ = std::max(0.0, motion_reference_max_age_s_);
  if (multi_guess_z_candidates_.empty()) {
    multi_guess_z_candidates_ = {0.0};
  }
  if (registration_mode_ != "multi_guess" && registration_mode_ != "initial_guess") {
    RCLCPP_WARN(
      this->get_logger(), "【重定位参数告警】registration_mode='%s' 无效，已回退为 initial_guess",
      registration_mode_.c_str());
    registration_mode_ = "initial_guess";
  }

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;
  growing_seed_transform_ = growing_fused_transform_ = result_t_;
  last_registration_robot_base_to_odom_ = Eigen::Isometry3d::Identity();
  need_coarse_alignment_ = true;
  has_accepted_alignment_ = false;

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  if (!mapping_mode_ && !loadGlobalMap(prior_pcd_file_)) {
    if (!rclcpp::ok(this->get_node_base_interface()->get_context())) {
      // Return an inert node so the component executable can unwind normally.
      // No registration timers, subscriptions, or observation publisher exist yet.
      return;
    }
    throw std::runtime_error("Global map unavailable");
  }

  if (mapping_mode_) {
    growing_map_ = std::make_unique<GrowingMapStore>(global_leaf_size_, growing_capacity,
      std::array<double, 3>{growing_size[0], growing_size[1], growing_size[2]},
      std::array<double, 3>{growing_origin[0], growing_origin[1], growing_origin[2]});
  } else {
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, recovery_num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(recovery_num_threads_));
  if (scan_context_enable_ && target_ && !target_->empty()) {
    scan_context_db_.buildFromCloud(target_->points);
    RCLCPP_INFO(
      this->get_logger(), "【ScanContext数据库就绪】锚点数=%zu 步长=%.2fm 半径=%.1fm",
      scan_context_db_.size(), scan_context_db_.config().grid_step_xy_m,
      scan_context_db_.config().max_radius_m);
  }
  }

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  localization_status_sub_ =
    this->create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &SmallGicpRelocalizationNode::localizationStatusCallback, this, std::placeholders::_1));
  observation_pub_ =
    this->create_publisher<ats_navigation_interfaces::msg::RelocalizationObservation>(
      "relocalization_observation", rclcpp::QoS(10).reliable());

  register_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(std::max(0.01, registration_interval_s_)),
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, transform_publish_rate_hz_)),
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  RCLCPP_INFO(
    this->get_logger(),
    "【GICP重定位就绪】模式=%s 累积帧=%d 精配准=%s 首窗口粗配准=%s "
    "精配准最大对应距离=%.3f 最小重叠率=%.3f 跟随状态=%s 丢失时自动多猜测=%s "
    "高度滤波=%s 确认次数=%d（最小间隔=%.3fs 运动容差=%.2fm/%.2frad） "
    "多猜测预算=%.2fs/%d 歧义裕量=%.3f（间隔=%.2fm/%.2frad） "
    "仿真放宽收敛=%s",
    registration_mode_.c_str(), accumulate_frames_, fine_alignment_enabled_ ? "true" : "false",
    coarse_first_window_only_ ? "true" : "false", std::sqrt(static_cast<double>(fine_max_dist_sq_)),
    min_overlap_ratio_, follow_localization_status_ ? "true" : "false",
    auto_multi_guess_on_lost_ ? "true" : "false", height_filter_enable_ ? "true" : "false",
    confirmation_count_, confirmation_min_interval_s_, confirmation_motion_translation_tolerance_,
    confirmation_motion_yaw_tolerance_, multi_guess_time_budget_s_,
    multi_guess_max_candidates_per_scan_, ambiguity_.min_score_margin, ambiguity_.min_separation_xy,
    ambiguity_.min_separation_yaw, relax_convergence_for_sim_ ? "true" : "false");

  RCLCPP_INFO(
    this->get_logger(),
    "【重定位输入门】期望坐标系='%s' 当前滞后=%.2fs 未来=%.2fs 距离=[%.2f,%.2f] z=[%.2f,%.2f] "
    "有效比例=%.2f 累积上限=%zu 点/%d 帧",
    odom_frame_.c_str(), max_scan_age_s_, max_scan_future_s_, scan_min_range_m_, scan_max_range_m_,
    scan_min_z_m_, scan_max_z_m_, min_scan_valid_ratio_, max_accumulated_points_,
    max_accumulated_frames_);
}

SmallGicpRelocalizationNode::~SmallGicpRelocalizationNode() { cancelAsyncMultiGuess(); }

bool SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "【重定位地图错误】无法读取 PCD 文件：%s", file_name.c_str());
    return false;
  }
  if (global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "【重定位地图错误】全局 PCD 地图为空：%s", file_name.c_str());
    return false;
  }
  // The prior is already expressed in map_frame_; mechanical sensor extrinsics
  // must never shift it. Registration estimates T_map_odom from odom-frame scans.
  RCLCPP_INFO(
    this->get_logger(), "【重定位地图加载】坐标系='%s' points=%zu",
    map_frame_.c_str(), global_map_->size());
  return true;
}

void SmallGicpRelocalizationNode::rebuildGrowingTarget()
{
  global_map_->clear();
  for (const auto & p : growing_map_->points()) {
    global_map_->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
  }
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, PointCovarianceCloud>(*global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, recovery_num_threads_);
  target_tree_ = std::make_shared<PointKdTree>(
    target_, small_gicp::KdTreeBuilderOMP(recovery_num_threads_));
  submap_center_xy_.reset();
  if (scan_context_enable_ && target_ && target_->size() >= local_submap_min_points_) {
    scan_context_db_.buildFromCloud(target_->points);
  }
  RCLCPP_INFO(get_logger(), "S3 growing map target=%zu dropped=%zu (fusion-committed growth)",
    target_->size(), growing_map_->dropped());
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    recordDroppedScan("received null scan message");
    return;
  }
  ScanInputConfig input_config;
  input_config.expected_frame = odom_frame_;
  input_config.max_age_s = max_scan_age_s_;
  input_config.max_future_s = max_scan_future_s_;
  input_config.min_range_m = scan_min_range_m_;
  input_config.max_range_m = scan_max_range_m_;
  input_config.min_z_m = scan_min_z_m_;
  input_config.max_z_m = scan_max_z_m_;
  input_config.min_valid_ratio = min_scan_valid_ratio_;
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if ((mapping_mode_ || require_odom_identity_) && (odom_instance_id_ == 0 ||
    odom_epoch_ == 0 || odom_identity_latched_ || stamp_ns < odom_epoch_start_ns_))
  {recordDroppedScan("unknown, latched or obsolete frontend epoch"); return;}
  if (mapping_reseed_needed_) {
    const auto odom_to_base = getOdomToRobotBase(rclcpp::Time(msg->header.stamp));
    if (!odom_to_base || !mapping_last_map_base_) {return;}
    previous_result_t_ = result_t_ = *mapping_last_map_base_ * odom_to_base->inverse();
    growing_seed_transform_ = previous_result_t_;
    mapping_reseed_needed_ = false;
  } else if (mapping_mode_ && has_accepted_alignment_) {
    if (const auto odom_to_base = getOdomToRobotBase(rclcpp::Time(msg->header.stamp))) {
      mapping_last_map_base_ = growing_fused_transform_ * *odom_to_base;
    }
  }
  const std::int64_t now_ns = this->now().nanoseconds();
  const std::string metadata_error = validateScanMetadata(
    msg->header.frame_id, stamp_ns, now_ns, last_received_scan_stamp_ns_, input_config);
  if (!metadata_error.empty()) {
    if (
      metadata_error.find("stale") != std::string::npos ||
      metadata_error.find("future") != std::string::npos ||
      metadata_error.find("order") != std::string::npos ||
      metadata_error.find("duplicate") != std::string::npos) {
      ++stale_scan_count_;
    } else {
      ++invalid_scan_count_;
    }
    recordDroppedScan(metadata_error);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  try {
    pcl::fromROSMsg(*msg, *scan);
  } catch (const std::exception & exception) {
    ++invalid_scan_count_;
    recordDroppedScan(std::string("failed to decode scan: ") + exception.what());
    return;
  }
  ScanInputStats stats;
  stats.total_points = scan->size();
  for (const auto & point : scan->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    ++stats.finite_points;
    if (scanPointInRange(point.x, point.y, point.z, input_config)) {
      scan->points[stats.valid_points++] = point;
    }
  }
  const std::string point_error = validateScanPointStats(stats, input_config);
  if (!point_error.empty()) {
    ++invalid_scan_count_;
    recordDroppedScan(point_error);
    return;
  }
  scan->points.resize(stats.valid_points);
  const auto trim = scan_window_.append(std::move(scan->points), stamp_ns);
  if (trim.sampled_points != 0 || trim.evicted_frames != 0) {
    ++trimmed_accumulation_count_;
  }
  sampled_scan_points_count_ += trim.sampled_points;
  evicted_scan_frames_count_ += trim.evicted_frames;
  last_scan_time_ = msg->header.stamp;
  last_received_scan_stamp_ns_ = stamp_ns;
  current_scan_frame_id_ = msg->header.frame_id;
  has_received_scan_ = true;
  ++accepted_scan_count_;
}

void SmallGicpRelocalizationNode::recordDroppedScan(const std::string & reason)
{
  ++dropped_scan_count_;
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "【重定位点云丢弃】registered_scan（%s）；丢弃=%s 过期=%s 无效=%s 已接受=%s "
    "裁剪窗口=%s 采样点=%s 淘汰帧=%s",
    reason.c_str(), std::to_string(dropped_scan_count_).c_str(),
    std::to_string(stale_scan_count_).c_str(), std::to_string(invalid_scan_count_).c_str(),
    std::to_string(accepted_scan_count_).c_str(),
    std::to_string(trimmed_accumulation_count_).c_str(),
    std::to_string(sampled_scan_points_count_).c_str(),
    std::to_string(evicted_scan_frames_count_).c_str());
}

void SmallGicpRelocalizationNode::clearAccumulation()
{
  scan_window_.clear();
}

bool SmallGicpRelocalizationNode::isRecoveringLocalization() const
{
  if (!follow_localization_status_) {
    return !has_accepted_alignment_ || need_coarse_alignment_;
  }
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  // 已锁定先验地图后的短暂 STATE_DEGRADED（如穿隧道时连续数帧 fine_only 未收敛）
  // 仍由高精度 Point-LIO 维持 map->odom，不应升级为 1.73 m 大半径 coarse 搜墙；
  // 只有真正 LOST、冷启动或由 LOST 进入的 RELOCALIZING 才进入恢复态。
  return localization_state_ == LS::STATE_UNINITIALIZED || localization_state_ == LS::STATE_LOST ||
         localization_state_ == LS::STATE_BOOTSTRAP ||
         (localization_state_ == LS::STATE_RELOCALIZING && recovery_from_lost_) ||
         !has_accepted_alignment_;
}

bool SmallGicpRelocalizationNode::inInitialPoseForceWindow() const
{
  return initial_pose_override_time_ && (this->now() - *initial_pose_override_time_).seconds() <=
                                          initial_pose_force_registration_window_s_;
}

/// 仿真放宽只允许放过 optimizer 的 converged 标志；有限误差、overlap、信息矩阵与
/// transform finite 永不可绕过。适用范围：/initialpose force window，以及首次
/// accept 之前的冷启动（含 confirmation_recheck）。Gazebo194 在 5s force window
/// 结束后 confirmation_recheck 因 converged=false 卡住（inliers~800/overlap~0.46），
/// sim_relax 计数为 0。首次 accept 后恢复严格门。
bool SmallGicpRelocalizationNode::simRelaxAllowed() const
{
  if (!relax_convergence_for_sim_) {
    return false;
  }
  if (inInitialPoseForceWindow()) {
    return true;
  }
  return !has_accepted_alignment_;
}

bool SmallGicpRelocalizationNode::preferMultiGuess() const
{
  if (mapping_mode_) {return false;}
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  // Pending confirmation owns the registration path: refine that hypothesis only.
  // Leaving LOST for RELOCALIZING/BOOTSTRAP must not fall through to fine_only,
  // which previously cleared pending on reject (straight190 @ t≈5.7s).
  if (pending_confirmation_ && pending_confirmation_count_ > 0) {
    return true;
  }
  // Only suppress lattice search while the /initialpose force window is active.
  // A stale initial_pose_override_time_ must NOT permanently disable LOST recovery.
  const bool in_initial_pose_force_window = inInitialPoseForceWindow();

  // Explicit multi_guess mode: lattice until first accept / while coarse needed.
  if (registration_mode_ == "multi_guess") {
    if (in_initial_pose_force_window) {
      return false;
    }
    return need_coarse_alignment_ || !has_accepted_alignment_;
  }
  // Auto multi_guess for LOST recovery. Also keep multi_guess through cold
  // start before the first accept: Gazebo health often passes on DEGRADED=3
  // ("awaiting first accepted"), and LOST-only auto multi_guess previously
  // fell through to coarse+fine (gazebo195: 84x not converged, sim_relax=0
  // on that path still needs relax_convergence_for_sim=true).
  if (!auto_multi_guess_on_lost_ || !follow_localization_status_) {
    return false;
  }
  if (in_initial_pose_force_window) {
    return false;
  }
  if (!has_accepted_alignment_) {
    return true;
  }
  return localization_state_ == LS::STATE_LOST ||
         (localization_state_ == LS::STATE_RELOCALIZING && recovery_from_lost_);
}

void SmallGicpRelocalizationNode::preprocessAccumulatedSource(int num_threads)
{
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  scan_window_.assemble(filtered->points);
  filtered->width = static_cast<std::uint32_t>(filtered->size());
  filtered->height = 1;
  filtered->is_dense = true;
  if (height_filter_enable_ && filtered && !filtered->empty()) {
    auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped->points.reserve(filtered->points.size());
    for (const auto & point : filtered->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < height_filter_min_z_ || point.z > height_filter_max_z_) {
        continue;
      }
      cropped->points.push_back(point);
    }
    cropped->width = static_cast<uint32_t>(cropped->points.size());
    cropped->height = 1;
    cropped->is_dense = false;
    filtered = cropped;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *filtered, registered_leaf_size_);
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads));

  // 稀疏筛选云只服务于 multi_guess 的 coarse 级：候选成本与源点数近似线性，
  // 稀疏化直接决定单次扫描能覆盖多少候选。fine 与验收仍用 source_。
  if (multi_guess_screen_leaf_size_ > registered_leaf_size_) {
    screen_source_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *filtered, multi_guess_screen_leaf_size_);
    small_gicp::estimate_covariances_omp(*screen_source_, num_neighbors_, num_threads);
    screen_source_tree_ =
      std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        screen_source_, small_gicp::KdTreeBuilderOMP(num_threads));
  } else {
    screen_source_.reset();
    screen_source_tree_.reset();
  }
}

void SmallGicpRelocalizationNode::updateTrackingSubmap(const Eigen::Vector2d & center_xy)
{
  if (!local_submap_enable_ || !target_ || target_->empty() || !center_xy.allFinite()) {
    return;
  }
  if (submap_center_xy_ && submap_target_ && submap_target_tree_ &&
    (center_xy - *submap_center_xy_).norm() < local_submap_recenter_distance_m_)
  {
    return;
  }
  const double r2 = local_submap_radius_m_ * local_submap_radius_m_;
  auto cropped = std::make_shared<PointCovarianceCloud>();
  cropped->points.reserve(std::min<std::size_t>(target_->size(), 8192U));
  for (const auto & pt : target_->points) {
    const double dx = static_cast<double>(pt.x) - center_xy.x();
    const double dy = static_cast<double>(pt.y) - center_xy.y();
    if (dx * dx + dy * dy <= r2) {
      cropped->points.push_back(pt);
    }
  }
  cropped->width = static_cast<std::uint32_t>(cropped->points.size());
  cropped->height = 1;
  cropped->is_dense = true;
  if (cropped->points.size() < local_submap_min_points_) {
    submap_target_.reset();
    submap_target_tree_.reset();
    submap_center_xy_.reset();
    return;
  }
  submap_target_ = std::move(cropped);
  submap_target_tree_ = std::make_shared<PointKdTree>(
    submap_target_, small_gicp::KdTreeBuilderOMP(num_threads_));
  submap_center_xy_ = center_xy;
}


}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
