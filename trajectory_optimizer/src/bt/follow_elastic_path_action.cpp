// Copyright 2026

#include "trajectory_optimizer/bt/follow_elastic_path_action.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

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

double poseYaw(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double shortestAngularDistance(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

// PipelineSequence can tick its follower while planning is still RUNNING. This
// action holds that branch until the planner has produced a usable reference.
class WaitForValidPath : public BT::ActionNodeBase
{
public:
  WaitForValidPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path", "Path to validate")};
  }

  BT::NodeStatus tick() override
  {
    return pathReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  bool pathReady()
  {
    nav_msgs::msg::Path path;
    return getInput("path", path) && path.poses.size() >= 2;
  }
};

// Keep the global guide stable during normal tracking. Costmap noise is handled
// by the local elastic layer; replacing the reference here repeatedly preempts
// FollowPath and resets the controller's progress checker.
class HasValidPath : public BT::ConditionNode
{
public:
  HasValidPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path", "Path to validate")};
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    return getInput("path", path) && path.poses.size() >= 2 ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// A recovery is an anomaly event. Clearing only the blackboard reference makes
// the next planning tick create a new global guide without ever issuing an
// empty FollowPath goal to controller_server.
class ClearPath : public BT::SyncActionNode
{
public:
  ClearPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<nav_msgs::msg::Path>("path", "Path to clear")};
  }

  BT::NodeStatus tick() override
  {
    setOutput("path", nav_msgs::msg::Path {});
    return BT::NodeStatus::SUCCESS;
  }
};

// An inflated cell alone is not a recovery event: following a safe path often
// crosses the inflation halo. It becomes an anomaly only when the footprint
// stays in that halo without measurable translation or heading progress.
class IsStuckInInflation : public BT::ConditionNode
{
public:
  IsStuckInInflation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    getInput("costmap_topic", costmap_topic_);
    getInput("odom_topic", odom_topic_);
    getInput("inflation_cost_threshold", inflation_cost_threshold_);
    getInput("footprint_radius", footprint_radius_);
    getInput("movement_radius", movement_radius_);
    getInput("rotation_progress_rad", rotation_progress_rad_);
    getInput("stuck_time_s", stuck_time_s_);

    inflation_cost_threshold_ = std::clamp(inflation_cost_threshold_, 1, 254);
    footprint_radius_ = std::max(0.05, footprint_radius_);
    movement_radius_ = std::max(0.01, movement_radius_);
    rotation_progress_rad_ = std::max(0.01, rotation_progress_rad_);
    stuck_time_s_ = std::max(0.1, stuck_time_s_);

    auto node = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&IsStuckInInflation::costmapCallback, this, std::placeholders::_1),
      subscription_options);
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&IsStuckInInflation::odomCallback, this, std::placeholders::_1),
      subscription_options);
    callback_group_executor_.add_callback_group(
      callback_group_, node->get_node_base_interface());
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "local_costmap/costmap_raw"),
      BT::InputPort<std::string>("odom_topic", "odometry"),
      BT::InputPort<int>("inflation_cost_threshold", 128, "Minimum inflation cost"),
      BT::InputPort<double>("footprint_radius", 0.42, "Cost sampling radius"),
      BT::InputPort<double>("movement_radius", 0.10, "Minimum net translation"),
      BT::InputPort<double>("rotation_progress_rad", 0.18, "Minimum net rotation"),
      BT::InputPort<double>("stuck_time_s", 3.0, "Stall detection duration"),
    };
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();

    nav2_msgs::msg::Costmap::SharedPtr costmap_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_costmap_ || !latest_odom_) {
        return BT::NodeStatus::FAILURE;
      }
      costmap_msg = latest_costmap_;
      odom_msg = latest_odom_;
    }
    const auto & costmap = *costmap_msg;
    const auto & odom = *odom_msg;

    const auto & metadata = costmap.metadata;
    if (metadata.resolution <= 0.0 || metadata.size_x == 0 || metadata.size_y == 0 ||
      costmap.data.empty() ||
      (!costmap.header.frame_id.empty() && !odom.header.frame_id.empty() &&
      costmap.header.frame_id != odom.header.frame_id))
    {
      reset();
      return BT::NodeStatus::FAILURE;
    }

    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    const int map_x = static_cast<int>(std::floor(
      (x - metadata.origin.position.x) / metadata.resolution));
    const int map_y = static_cast<int>(std::floor(
      (y - metadata.origin.position.y) / metadata.resolution));
    if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(metadata.size_x) ||
      map_y >= static_cast<int>(metadata.size_y))
    {
      reset();
      return BT::NodeStatus::FAILURE;
    }

    const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(footprint_radius_ / metadata.resolution)));
    int max_cost = 0;
    int inflation_cells = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      const int sample_y = map_y + dy;
      if (sample_y < 0 || sample_y >= static_cast<int>(metadata.size_y)) {
        continue;
      }
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int sample_x = map_x + dx;
        if (sample_x < 0 || sample_x >= static_cast<int>(metadata.size_x)) {
          continue;
        }
        const double cell_x = metadata.origin.position.x +
          (static_cast<double>(sample_x) + 0.5) * metadata.resolution;
        const double cell_y = metadata.origin.position.y +
          (static_cast<double>(sample_y) + 0.5) * metadata.resolution;
        if (std::hypot(cell_x - x, cell_y - y) > footprint_radius_) {
          continue;
        }
        const auto index = static_cast<std::size_t>(sample_y) * metadata.size_x +
          static_cast<std::size_t>(sample_x);
        if (index >= costmap.data.size()) {
          continue;
        }
        const int cost = static_cast<unsigned char>(costmap.data[index]);
        if (cost == 255) {
          continue;
        }
        max_cost = std::max(max_cost, cost);
        if (cost >= inflation_cost_threshold_) {
          ++inflation_cells;
        }
      }
    }

    if (inflation_cells == 0) {
      reset();
      return BT::NodeStatus::FAILURE;
    }

    const auto now = clock_->now();
    const double yaw = poseYaw(odom.pose.pose);
    if (!inflation_anchor_) {
      inflation_anchor_ = geometry_msgs::msg::Point {};
      inflation_anchor_->x = x;
      inflation_anchor_->y = y;
      inflation_anchor_yaw_ = yaw;
      anchor_time_ = now;
      RCLCPP_WARN(
        logger_,
        "[脱困监测] 车体进入膨胀/障碍区域，开始 %.1f 秒卡滞计时："
        "足迹最大代价=%d，高代价栅格=%d。",
        stuck_time_s_, max_cost, inflation_cells);
      return BT::NodeStatus::FAILURE;
    }

    const double moved = std::hypot(
      x - inflation_anchor_->x, y - inflation_anchor_->y);
    const double rotated = std::abs(shortestAngularDistance(*inflation_anchor_yaw_, yaw));
    const double elapsed = (now - anchor_time_).seconds();
    if (elapsed < stuck_time_s_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[脱困监测] 正在计时：%.1f/%.1f 秒，净移动=%.3f 米，净转角=%.1f 度，"
        "足迹最大代价=%d。",
        elapsed, stuck_time_s_, moved, rotated * 180.0 / M_PI, max_cost);
      return BT::NodeStatus::FAILURE;
    }

    if (moved >= movement_radius_ || rotated >= rotation_progress_rad_) {
      inflation_anchor_->x = x;
      inflation_anchor_->y = y;
      inflation_anchor_yaw_ = yaw;
      anchor_time_ = now;
      RCLCPP_INFO(
        logger_,
        "[脱困监测] 观察窗口内仍有有效运动，不触发脱困：移动=%.3f 米，转角=%.1f 度。",
        moved, rotated * 180.0 / M_PI);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[脱困触发] 车辆在膨胀/障碍区域卡滞 %.1f 秒：净移动=%.3f 米，"
      "净转角=%.1f 度，足迹最大代价=%d；立即请求自研 Backup。",
      elapsed, moved, rotated * 180.0 / M_PI, max_cost);
    return BT::NodeStatus::SUCCESS;
  }

private:
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_costmap_ = msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = msg;
  }

  void reset()
  {
    inflation_anchor_.reset();
    inflation_anchor_yaw_.reset();
  }

  std::string costmap_topic_{"local_costmap/costmap_raw"};
  std::string odom_topic_{"odometry"};
  int inflation_cost_threshold_{128};
  double footprint_radius_{0.42};
  double movement_radius_{0.10};
  double rotation_progress_rad_{0.18};
  double stuck_time_s_{3.0};
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::optional<geometry_msgs::msg::Point> inflation_anchor_;
  std::optional<double> inflation_anchor_yaw_;
  rclcpp::Time anchor_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("IsStuckInInflation")};
};

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
  getInput("heading_change_threshold_rad", heading_change_threshold_rad_);

  elastic_path_timeout_s_ = std::max(0.05, elastic_path_timeout_s_);
  elastic_update_min_period_s_ = std::max(0.0, elastic_update_min_period_s_);
  goal_match_distance_ = std::max(0.01, goal_match_distance_);
  heading_change_threshold_rad_ = std::max(0.01, heading_change_threshold_rad_);

  elastic_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    elastic_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FollowElasticPathAction::elasticPathCallback, this, std::placeholders::_1));
}

void FollowElasticPathAction::on_tick()
{
  nav_msgs::msg::Path global_reference;
  if (!getInput("path", global_reference) || global_reference.poses.size() < 2) {
    // BtActionNode would otherwise send an empty goal during the initial
    // PipelineSequence tick, causing controller_server to abort FollowPath.
    should_send_goal_ = false;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Waiting for a stable global reference path before sending FollowPath.");
    return;
  }

  latest_global_reference_ = std::move(global_reference);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  goal_.path = latest_global_reference_;
  applyLatestElasticPath();
}

void FollowElasticPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>)
{
  // A FollowPath update is only valid when its action goal still has a path.
  // Keep the last stable reference as the hard fallback through planner and
  // smoother blackboard transitions.
  if (!ensureUsableGoalPath()) {
    goal_updated_ = false;
    return;
  }

  nav_msgs::msg::Path global_reference;
  if (getInput("path", global_reference) && global_reference.poses.size() >= 2 &&
    globalReferenceChanged(global_reference))
  {
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

bool FollowElasticPathAction::ensureUsableGoalPath()
{
  if (goal_.path.poses.size() >= 2) {
    return true;
  }
  if (latest_global_reference_.poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Suppressing FollowPath update because no valid reference path is available.");
    return false;
  }

  goal_.path = latest_global_reference_;
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Restored the stable global reference before a FollowPath update.");
  return true;
}

bool FollowElasticPathAction::pathMatchesCurrentGoal(const nav_msgs::msg::Path & candidate) const
{
  if (candidate.poses.size() < 2 || latest_global_reference_.poses.size() < 2) {
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
  return pathsMateriallyDifferent(
    candidate, latest_global_reference_, 0.01, heading_change_threshold_rad_);
}

bool FollowElasticPathAction::pathsMateriallyDifferent(
  const nav_msgs::msg::Path & lhs,
  const nav_msgs::msg::Path & rhs,
  double position_threshold,
  double heading_threshold_rad)
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
    if (pointDistance(
        lhs.poses[index].pose.position,
        rhs.poses[index].pose.position) > position_threshold)
    {
      return true;
    }
    if (std::abs(shortestAngularDistance(
        poseYaw(lhs.poses[index].pose), poseYaw(rhs.poses[index].pose))) > heading_threshold_rad)
    {
      return true;
    }
  }
  return false;
}

}  // namespace trajectory_optimizer

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trajectory_optimizer::WaitForValidPath>("WaitForValidPath");
  factory.registerNodeType<trajectory_optimizer::HasValidPath>("HasValidPath");
  factory.registerNodeType<trajectory_optimizer::ClearPath>("ClearPath");
  factory.registerNodeType<trajectory_optimizer::IsStuckInInflation>("IsStuckInInflation");
  factory.registerNodeType<trajectory_optimizer::FollowElasticPathAction>("FollowElasticPath");
}
