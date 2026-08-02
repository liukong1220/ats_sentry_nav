// Copyright 2026

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rog_map/rog_map_core/config.hpp"

namespace ats_rog_map
{

// ROS-facing core configuration. Keep this independent from rclcpp so the
// same values can be compared against the legacy file loader in unit tests.
struct RogMapCoreParameters
{
  bool esdf_enable{true};
  double esdf_resolution{0.1};
  std::array<double, 3> esdf_local_update_box{8.0, 8.0, 1.0};
  int esdf_update_interval_updates{1};
  bool load_pcd_enable{false};
  std::string pcd_name{"map.pcd"};
  bool map_sliding_enable{true};
  double map_sliding_threshold{1.0};
  std::array<double, 3> fix_map_origin{0.0, 0.0, 0.5};
  bool frontier_extraction_enable{false};
  bool ros_callback_enable{false};
  std::string ros_callback_cloud_topic{"/cloud_registered"};
  std::string ros_callback_odom_topic{"/lidar_slam/odom"};
  double ros_callback_odom_timeout{0.05};
  bool visualization_enable{false};
  bool visualization_publish_unknown{false};
  std::string visualization_frame_id{"world"};
  double visualization_time_rate{0.0};
  int visualization_frame_rate{0};
  std::array<double, 3> visualization_range{0.0, 0.0, 0.0};
  double resolution{0.1};
  double inflation_resolution{0.1};
  bool unknown_inflation_enable{false};
  int unknown_inflation_step{1};
  int inflation_step{2};
  int intensity_threshold{-1};
  std::array<double, 3> map_size{10.0, 10.0, 1.0};
  int point_filter_count{1};
  bool raycasting_enable{true};
  int raycasting_batch_update_size{1};
  double raycasting_unknown_threshold{0.70};
  double raycasting_p_hit{0.70};
  double raycasting_p_miss{0.30};
  double raycasting_p_min{0.12};
  double raycasting_p_max{0.97};
  double raycasting_p_occupied{0.80};
  double raycasting_p_free{0.30};
  std::array<double, 2> raycasting_range{0.30, 10.0};
  std::array<double, 3> raycasting_local_update_box{8.0, 8.0, 1.0};
  bool stale_decay_enable{true};
  int stale_decay_soft_ttl_updates{30};
  int stale_decay_hard_ttl_updates{90};
  double stale_decay_log_odds_step{0.25};
  int stale_decay_sweep_interval_updates{1};
  bool stale_decay_local_update_box_only{true};
  bool clear_clipped_endpoint{true};
  double virtual_ground_height{-0.5};
  double virtual_ceil_height{1.0};
};

inline rog_map::Vec3f toVec3(const std::array<double, 3> & value)
{
  return rog_map::Vec3f(value[0], value[1], value[2]);
}

inline rog_map::Config makeRogMapConfig(const RogMapCoreParameters & parameters)
{
  rog_map::Config config;
  config.esdf_en = parameters.esdf_enable;
  config.esdf_resolution = parameters.esdf_resolution;
  config.esdf_local_update_box = toVec3(parameters.esdf_local_update_box);
  config.esdf_update_interval_updates = std::max(1, parameters.esdf_update_interval_updates);
  config.load_pcd_en = parameters.load_pcd_enable;
  config.pcd_name = parameters.pcd_name;
  config.map_sliding_en = parameters.map_sliding_enable;
  config.map_sliding_thresh = parameters.map_sliding_threshold;
  config.fix_map_origin = toVec3(parameters.fix_map_origin);
  config.frontier_extraction_en = parameters.frontier_extraction_enable;
  config.ros_callback_en = parameters.ros_callback_enable;
  config.cloud_topic = parameters.ros_callback_cloud_topic;
  config.odom_topic = parameters.ros_callback_odom_topic;
  config.odom_timeout = parameters.ros_callback_odom_timeout;
  config.visualization_en = parameters.visualization_enable;
  config.pub_unknown_map_en = parameters.visualization_publish_unknown;
  config.frame_id = parameters.visualization_frame_id;
  config.viz_time_rate = parameters.visualization_time_rate;
  config.viz_frame_rate = parameters.visualization_frame_rate;
  config.visualization_range = toVec3(parameters.visualization_range);
  config.resolution = parameters.resolution;
  config.inflation_resolution = parameters.inflation_resolution;
  config.unk_inflation_en = parameters.unknown_inflation_enable;
  config.unk_inflation_step = parameters.unknown_inflation_step;
  config.inflation_step = parameters.inflation_step;
  config.intensity_thresh = parameters.intensity_threshold;
  config.map_size_d = toVec3(parameters.map_size);
  config.point_filt_num = parameters.point_filter_count;
  config.raycasting_en = parameters.raycasting_enable;
  config.batch_update_size = parameters.raycasting_batch_update_size;
  config.unk_thresh = parameters.raycasting_unknown_threshold;
  config.p_hit = static_cast<float>(parameters.raycasting_p_hit);
  config.p_miss = static_cast<float>(parameters.raycasting_p_miss);
  config.p_min = static_cast<float>(parameters.raycasting_p_min);
  config.p_max = static_cast<float>(parameters.raycasting_p_max);
  config.p_occ = static_cast<float>(parameters.raycasting_p_occupied);
  config.p_free = static_cast<float>(parameters.raycasting_p_free);
  config.raycast_range_min = parameters.raycasting_range[0];
  config.raycast_range_max = parameters.raycasting_range[1];
  config.local_update_box_d = toVec3(parameters.raycasting_local_update_box);
  config.stale_decay_en = parameters.stale_decay_enable;
  config.stale_soft_ttl_updates = parameters.stale_decay_soft_ttl_updates;
  config.stale_hard_ttl_updates = parameters.stale_decay_hard_ttl_updates;
  config.stale_decay_log_odds_step = static_cast<float>(parameters.stale_decay_log_odds_step);
  config.stale_sweep_interval_updates = parameters.stale_decay_sweep_interval_updates;
  config.stale_decay_local_update_box_only = parameters.stale_decay_local_update_box_only;
  config.clear_clipped_endpoint = parameters.clear_clipped_endpoint;
  config.virtual_ground_height = parameters.virtual_ground_height;
  config.virtual_ceil_height = parameters.virtual_ceil_height;

  if (config.resolution > config.inflation_resolution) {
    throw std::invalid_argument("The inflation resolution should be equal or larger than the resolution!");
  }
  config.point_filt_num = std::max(1, config.point_filt_num);
  config.batch_update_size = std::max(1, config.batch_update_size);
  config.stale_soft_ttl_updates = std::max(0, config.stale_soft_ttl_updates);
  config.stale_hard_ttl_updates = std::max(0, config.stale_hard_ttl_updates);
  if (config.stale_hard_ttl_updates > 0 &&
    config.stale_hard_ttl_updates < config.stale_soft_ttl_updates)
  {
    config.stale_hard_ttl_updates = config.stale_soft_ttl_updates;
  }
  config.stale_decay_log_odds_step = std::abs(config.stale_decay_log_odds_step);
  config.stale_sweep_interval_updates = std::max(1, config.stale_sweep_interval_updates);
  config.sqr_raycast_range_min = config.raycast_range_min * config.raycast_range_min;
  config.sqr_raycast_range_max = config.raycast_range_max * config.raycast_range_max;
  config.resetMapSize();

  const auto probabilityToLogOdds = [](float probability) {
      return std::log(probability / (1.0F - probability));
    };
  config.l_hit = probabilityToLogOdds(config.p_hit);
  config.l_miss = probabilityToLogOdds(config.p_miss);
  config.l_min = probabilityToLogOdds(config.p_min);
  config.l_max = probabilityToLogOdds(config.p_max);
  config.l_occ = probabilityToLogOdds(config.p_occ);
  config.l_free = probabilityToLogOdds(config.p_free);

  const auto sort_by_distance = [](const rog_map::Vec3i & left, const rog_map::Vec3i & right) {
      return left.squaredNorm() < right.squaredNorm();
    };
  config.inf_spherical_neighbor.clear();
  for (int dx = -config.inflation_step; dx <= config.inflation_step; ++dx) {
    for (int dy = -config.inflation_step; dy <= config.inflation_step; ++dy) {
      for (int dz = -config.inflation_step; dz <= config.inflation_step; ++dz) {
        if (config.inflation_step == 1 ||
          dx * dx + dy * dy + dz * dz <= config.inflation_step * config.inflation_step)
        {
          config.inf_spherical_neighbor.emplace_back(dx, dy, dz);
        }
      }
    }
  }
  std::sort(
    config.inf_spherical_neighbor.begin(), config.inf_spherical_neighbor.end(), sort_by_distance);

  config.unk_inf_spherical_neighbor.clear();
  if (config.unk_inflation_en) {
    for (int dx = -config.unk_inflation_step; dx <= config.unk_inflation_step; ++dx) {
      for (int dy = -config.unk_inflation_step; dy <= config.unk_inflation_step; ++dy) {
        for (int dz = -config.unk_inflation_step; dz <= config.unk_inflation_step; ++dz) {
          if (config.unk_inflation_step == 1 ||
            dx * dx + dy * dy + dz * dz <= config.unk_inflation_step * config.unk_inflation_step)
          {
            config.unk_inf_spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(
      config.unk_inf_spherical_neighbor.begin(), config.unk_inf_spherical_neighbor.end(),
      sort_by_distance);
  }

  config.spherical_neighbor.clear();
  const int max_search_step = static_cast<int>(std::ceil(5.0 / config.resolution));
  for (int dx = -max_search_step; dx <= max_search_step; ++dx) {
    for (int dy = -max_search_step; dy <= max_search_step; ++dy) {
      for (int dz = -max_search_step; dz <= max_search_step; ++dz) {
        if (dx * dx + dy * dy + dz * dz <= max_search_step * max_search_step) {
          config.spherical_neighbor.emplace_back(dx, dy, dz);
        }
      }
    }
  }
  std::sort(config.spherical_neighbor.begin(), config.spherical_neighbor.end(), sort_by_distance);
  return config;
}

}  // namespace ats_rog_map
