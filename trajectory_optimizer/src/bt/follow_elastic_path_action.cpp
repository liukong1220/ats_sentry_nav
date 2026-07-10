// Copyright 2026

#include "trajectory_optimizer/bt/follow_elastic_path_action.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace trajectory_optimizer
{

namespace
{

double pointDistance(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace

FollowElasticPathAction::FollowElasticPathAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BtActionNode(name, "follow_path", config)
{
  getInput("elastic_path_topic", elastic_path_topic_);
  getInput("elastic_path_timeout_s", elastic_path_timeout_s_);
  getInput("elastic_update_min_period_s", elastic_update_min_period_s_);
  getInput("goal_match_distance", goal_match_distance_);

  elastic_path_timeout_s_ = std::max(0.05, elastic_path_timeout_s_);
  elastic_update_min_period_s_ = std::max(0.0, elastic_update_min_period_s_);
  goal_match_distance_ = std::max(0.01, goal_match_distance_);

  elastic_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    elastic_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FollowElasticPathAction::elasticPathCallback, this, std::placeholders::_1));
}

void FollowElasticPathAction::on_tick()
{
  getInput("path", latest_global_reference_);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  goal_.path = latest_global_reference_;
  applyLatestElasticPath();
}

void FollowElasticPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>)
{
  nav_msgs::msg::Path global_reference;
  if (getInput("path", global_reference) && globalReferenceChanged(global_reference)) {
    latest_global_reference_ = global_reference;
    goal_.path = latest_global_reference_;
    applied_revision_ = 0;
    goal_updated_ = true;
    return;
  }

  if (applyLatestElasticPath()) {
    goal_updated_ = true;
  }
}

void FollowElasticPathAction::elasticPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.size() < 2) {
    return;
  }

  std::lock_guard<std::mutex> lock(elastic_path_mutex_);
  latest_elastic_path_ = *msg;
  ++received_revision_;
}

bool FollowElasticPathAction::applyLatestElasticPath()
{
  nav_msgs::msg::Path candidate;
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(elastic_path_mutex_);
    if (received_revision_ <= applied_revision_) {
      return false;
    }
    candidate = latest_elastic_path_;
    revision = received_revision_;
  }

  if (!pathMatchesCurrentGoal(candidate)) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_update_time_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(now - last_update_time_).count() < elastic_update_min_period_s_)
  {
    return false;
  }

  const rclcpp::Time stamp(candidate.header.stamp);
  if (stamp.nanoseconds() > 0 && (node_->now() - stamp).seconds() > elastic_path_timeout_s_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Ignoring stale local elastic path: age=%.3fs timeout=%.3fs",
      (node_->now() - stamp).seconds(), elastic_path_timeout_s_);
    return false;
  }

  goal_.path = std::move(candidate);
  applied_revision_ = revision;
  last_update_time_ = now;
  return true;
}

bool FollowElasticPathAction::pathMatchesCurrentGoal(const nav_msgs::msg::Path & candidate) const
{
  if (candidate.poses.size() < 2 || latest_global_reference_.poses.empty()) {
    return false;
  }
  if (!candidate.header.frame_id.empty() && !latest_global_reference_.header.frame_id.empty() &&
    candidate.header.frame_id != latest_global_reference_.header.frame_id)
  {
    return false;
  }

  return pointDistance(
    candidate.poses.back().pose.position,
    latest_global_reference_.poses.back().pose.position) <= goal_match_distance_;
}

bool FollowElasticPathAction::globalReferenceChanged(const nav_msgs::msg::Path & candidate) const
{
  return pathsMateriallyDifferent(candidate, latest_global_reference_, 0.01);
}

bool FollowElasticPathAction::pathsMateriallyDifferent(
  const nav_msgs::msg::Path & lhs,
  const nav_msgs::msg::Path & rhs,
  double threshold)
{
  if (lhs.header.frame_id != rhs.header.frame_id || lhs.poses.size() != rhs.poses.size()) {
    return true;
  }
  if (lhs.poses.empty()) {
    return false;
  }

  const std::size_t last = lhs.poses.size() - 1;
  constexpr std::size_t kSamples = 16;
  for (std::size_t sample = 0; sample < kSamples; ++sample) {
    const std::size_t index = (last * sample) / (kSamples - 1);
    if (pointDistance(lhs.poses[index].pose.position, rhs.poses[index].pose.position) > threshold) {
      return true;
    }
  }
  return false;
}

}  // namespace trajectory_optimizer

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trajectory_optimizer::FollowElasticPathAction>("FollowElasticPath");
}
