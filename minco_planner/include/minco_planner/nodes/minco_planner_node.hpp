// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "minco_planner/debug/planner_debug_visualizer.hpp"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_jps.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace minco_planner
{

class MincoPlannerNode : public rclcpp::Node
{
public:
  explicit MincoPlannerNode(const rclcpp::NodeOptions & options);

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onMapReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onMapReadyWatchdog();
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPlannerGoal(const ats_navigation_interfaces::msg::PlannerGoal::SharedPtr msg);
  void onGlobalPlan(const nav_msgs::msg::Path::SharedPtr msg);
  void planGoal(const geometry_msgs::msg::PoseStamped &goal,
                std::uint64_t goal_id, std::uint64_t localization_epoch,
                bool report_status);
  bool lookupStartPose(
    const nav_msgs::msg::OccupancyGrid & grid, geometry_msgs::msg::PoseStamped & start) const;
  bool transformGoalToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const;
  bool transformPathToGlobal(
    const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output) const;
  nav_msgs::msg::Path toPath(const ReferenceTrajectory & trajectory) const;
  void annotateClearance(
    ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const;
  std::vector<Eigen::Vector2d> footprintSamples(
    const nav_msgs::msg::OccupancyGrid & grid) const;
  void setPlanSafe(bool safe);
  bool publishReferenceIfCurrent(
      const std::shared_ptr<const PlanningMapSnapshot> &snapshot,
      std::uint64_t map_health_epoch, const nav_msgs::msg::Path &reference_path,
      std::uint64_t goal_id, std::uint64_t localization_epoch,
      bool report_status);
  void publishEmergencyStop(bool stop);
  void
  publishPlannerStatus(std::uint64_t goal_id, std::uint64_t localization_epoch,
                       std::uint64_t map_generation, std::uint8_t state,
                       const std::string &reason,
                       const builtin_interfaces::msg::Time &reference_stamp =
                           builtin_interfaces::msg::Time());
  void declareAndLoadParams();

  std::string grid_topic_ = "traversability_grid";
  std::string goal_topic_ = "goal_pose";
  std::string global_plan_topic_ = "/plan";
  std::string goal_request_topic_;
  std::string planner_status_topic_;
  std::string raw_path_topic_ = "minco/raw_path";
  std::string reference_path_topic_ = "minco/reference_path";
  std::string candidate_reference_path_topic_;
  std::string debug_marker_topic_ = "minco/debug_markers";
  std::string map_ready_topic_;
  std::string emergency_stop_topic_ = "/planner/emergency_stop";
  std::string global_frame_ = "map";
  std::string robot_frame_ = "base_link";
  std::string search_algorithm_ = "jps";
  bool astar_fallback_ = true;
  bool publish_unsafe_trajectory_ = false;
  bool planner_manages_emergency_stop_ = true;
  int obstacle_value_threshold_ = 50;
  bool unknown_is_obstacle_ = true;
  double footprint_length_ = 0.70;
  double footprint_width_ = 0.55;
  double footprint_safety_margin_ = 0.05;
  double map_ready_timeout_sec_ = 3.0;
  double emergency_stop_heartbeat_period_sec_ = 0.1;

  GridAstar astar_;
  GridJps jps_;
  MincoTrajectoryOptimizer optimizer_;
  YawSplinePlanner yaw_planner_;
  FootprintSafetyChecker safety_checker_;
  LocalCollisionRepair collision_repair_;
  PlannerDebugVisualizer visualizer_;

  std::mutex map_mutex_;
  std::shared_ptr<const PlanningMapSnapshot> latest_map_snapshot_;
  std::uint64_t next_map_generation_{0};
  std::uint64_t map_health_epoch_{0};
  PlannerSafetyState safety_state_;
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlannerGoal>::SharedPtr goal_request_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::TimerBase::SharedPtr safety_watchdog_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_reference_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlannerStatus>::SharedPtr planner_status_pub_;

  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
