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
#include "minco_planner/trajectory/trajectory_quality_evaluator.hpp"
#include "minco_planner/trajectory/yaw_authority_policy.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
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
  void planGoal(const geometry_msgs::msg::PoseStamped &goal,
                std::uint64_t goal_id, std::uint64_t localization_epoch,
                std::uint64_t plan_request_sequence,
                std::uint64_t map_publication_sequence,
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
      const ReferenceTrajectory &safety_reference,
      std::uint64_t goal_id, std::uint64_t localization_epoch,
      std::uint64_t plan_request_sequence,
      std::uint64_t map_publication_sequence,
      std::uint8_t yaw_authority,
      bool report_status);
  void onRuntimeSafetyRecheck();
  void publishEmergencyStop(bool stop);
  void
  publishPlannerStatus(std::uint64_t goal_id, std::uint64_t localization_epoch,
                       std::uint64_t plan_request_sequence,
                       std::uint64_t map_generation,
                       std::uint64_t map_publication_sequence,
                       std::uint8_t state, std::uint8_t failure_reason,
                       const builtin_interfaces::msg::Time &reference_stamp =
                           builtin_interfaces::msg::Time(),
                       std::uint8_t yaw_authority = 0,
                       bool requires_gimbal_lock = false);
  void declareAndLoadParams();

  std::string grid_topic_ = "traversability_grid";
  std::string goal_topic_ = "goal_pose";
  std::string goal_request_topic_;
  std::string planner_status_topic_;
  std::string raw_path_topic_ = "minco/raw_path";
  std::string reference_path_topic_ = "minco/reference_path";
  std::string candidate_reference_path_topic_;
  std::string preprocessed_guide_topic_ = "/minco/preprocessed_guide";
  std::string esdf_refined_guide_topic_ = "/minco/esdf_refined_guide";
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
  double runtime_safety_recheck_hz_ = 10.0;
  double runtime_safety_horizon_sec_ = 1.0;
  double body_yaw_follow_clearance_ = 0.55;
  bool force_body_yaw_follow_ = false;

  GridAstar astar_;
  GridJps jps_;
  MincoTrajectoryOptimizer optimizer_;
  YawSplinePlanner yaw_planner_;
  FootprintSafetyChecker safety_checker_;
  LocalCollisionRepair collision_repair_;
  PlannerDebugVisualizer visualizer_;
  TrajectoryQualityEvaluator quality_evaluator_;

  std::mutex map_mutex_;
  std::shared_ptr<const PlanningMapSnapshot> latest_map_snapshot_;
  std::uint64_t next_map_generation_{0};
  std::uint64_t map_health_epoch_{0};
  PlannerSafetyState safety_state_;
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;
  struct ActiveSafetyReference
  {
    ReferenceTrajectory trajectory;
    std::uint64_t goal_id{0};
    std::uint64_t localization_epoch{0};
    std::uint64_t plan_request_sequence{0};
    std::uint64_t map_generation{0};
    std::uint64_t map_publication_sequence{0};
  };
  std::optional<ActiveSafetyReference> active_safety_reference_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlannerGoal>::SharedPtr goal_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::TimerBase::SharedPtr safety_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr runtime_safety_recheck_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preprocessed_guide_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr esdf_refined_guide_pub_;
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
