// Copyright 2026

#ifndef ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_HPP_
#define ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_HPP_

#include <memory>

#include "ats_rog_map/ground_projection.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ats_rog_map::planning
{

/// Creates the planning-map owner node ("ats_rog_map_adapter"). It fuses the
/// in-process ROGMap projection with the static, terrain and slope layers and
/// publishes the planning snapshot, status and ready heartbeat. `provider` runs
/// on the node's own worker thread, never on an executor thread.
std::shared_ptr<rclcpp::Node> makePlanningMapPublisher(
  GroundProjectionProvider provider, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace ats_rog_map::planning

#endif  // ATS_ROG_MAP__PLANNING__PLANNING_MAP_PUBLISHER_HPP_
