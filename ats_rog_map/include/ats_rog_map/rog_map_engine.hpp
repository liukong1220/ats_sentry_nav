// Copyright 2026

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rog_map/rog_map.h"

namespace ats_rog_map
{

class RogMapEngine final : public rog_map::ROGMap
{
public:
  RogMapEngine(const rclcpp::Clock::SharedPtr & clock, const std::string & config_file)
  : clock_(clock)
  {
    cfg_ = rog_map::Config(config_file);
    init();
  }

  RogMapEngine(const rclcpp::Clock::SharedPtr & clock, rog_map::Config config)
  : clock_(clock)
  {
    cfg_ = std::move(config);
    init();
  }

  bool update(
    const rog_map::PointCloud & cloud, const rog_map::Pose & robot_pose,
    const rog_map::Pose & sensor_pose)
  {
    const std::uint64_t previous_map_update = map_update_index_;
    const rog_map::Vec3f previous_origin = getLocalMapOrigin();

    // The robot moves the sliding window while the sensor remains the ray origin.
    updateRobotState(robot_pose);
    updateProbMap(cloud, sensor_pose, robot_pose.first);

    const bool mutated = map_update_index_ != previous_map_update ||
      (getLocalMapOrigin() - previous_origin).cwiseAbs().maxCoeff() > 1e-9;
    if (mutated) {
      ++snapshot_generation_;
      const bool core_rebuilt_esdf = map_update_index_ != previous_map_update && cfg_.esdf_en &&
        map_update_index_ % static_cast<std::uint64_t>(cfg_.esdf_update_interval_updates) == 0U;
      // updateProbMap() performs this rebuild at the configured update interval.  Keep the
      // immutable snapshot generation aligned so projection does not rebuild the same ESDF.
      esdf_generation_ = core_rebuilt_esdf ? snapshot_generation_ : 0U;
    }
    return mutated;
  }

  bool ensureCurrentEsdf()
  {
    if (!esdf_map_ || snapshot_generation_ == 0) {
      return false;
    }
    if (esdf_generation_ != snapshot_generation_) {
      esdf_map_->updateESDF3D(robot_state_.p);
      esdf_generation_ = snapshot_generation_;
    }
    return true;
  }

  bool getCurrentEsdfBounds(rog_map::Vec3f & box_min, rog_map::Vec3f & box_max) const
  {
    if (!esdf_map_ || snapshot_generation_ == 0 || esdf_generation_ != snapshot_generation_) {
      return false;
    }
    esdf_map_->getUpdatedBbox(box_min, box_max);
    return true;
  }

  std::uint64_t generation() const
  {
    return snapshot_generation_;
  }

private:
  const double getSystemWalltimeNow() override
  {
    return clock_->now().seconds();
  }

  rclcpp::Clock::SharedPtr clock_;
  std::uint64_t snapshot_generation_{0};
  std::uint64_t esdf_generation_{0};
};

}  // namespace ats_rog_map
