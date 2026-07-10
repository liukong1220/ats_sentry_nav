// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__BT__FOLLOW_ELASTIC_PATH_ACTION_HPP_
#define TRAJECTORY_OPTIMIZER__BT__FOLLOW_ELASTIC_PATH_ACTION_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trajectory_optimizer
{

// Keeps Nav2's FollowPath action on a stable global reference until a newer,
// compatible RC-ESDF local path is available. Updates are rate-limited so the
// controller is not preempted by every optimizer timer tick.
class FollowElasticPathAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowElasticPathAction(const std::string & name, const BT::NodeConfiguration & config);

  void on_tick() override;
  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Stable global reference path"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
        BT::InputPort<std::string>(
          "elastic_path_topic", "local_elastic_path", "RC-ESDF local path topic"),
        BT::InputPort<double>(
          "elastic_path_timeout_s", 0.8, "Maximum accepted local path age"),
        BT::InputPort<double>(
          "elastic_update_min_period_s", 0.2, "Minimum controller path update period"),
        BT::InputPort<double>(
          "goal_match_distance", 0.25, "Maximum local/global goal endpoint mismatch"),
      });
  }

private:
  void elasticPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  bool applyLatestElasticPath();
  bool pathMatchesCurrentGoal(const nav_msgs::msg::Path & candidate) const;
  bool globalReferenceChanged(const nav_msgs::msg::Path & candidate) const;
  static bool pathsMateriallyDifferent(
    const nav_msgs::msg::Path & lhs,
    const nav_msgs::msg::Path & rhs,
    double threshold);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr elastic_path_sub_;
  std::string elastic_path_topic_{"local_elastic_path"};
  double elastic_path_timeout_s_{0.8};
  double elastic_update_min_period_s_{0.2};
  double goal_match_distance_{0.25};
  std::mutex elastic_path_mutex_;
  nav_msgs::msg::Path latest_elastic_path_;
  nav_msgs::msg::Path latest_global_reference_;
  std::uint64_t received_revision_{0};
  std::uint64_t applied_revision_{0};
  std::chrono::steady_clock::time_point last_update_time_{};
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__BT__FOLLOW_ELASTIC_PATH_ACTION_HPP_
