// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include "minco_planner/planning/graph_search_failure.hpp"
#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/nodes/planning_digest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "minco_planner/trajectory/reference_path_timing.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{

namespace
{

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
    std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
    std::isfinite(pose.orientation.w);
}

std::string formatCollisionIndices(const std::vector<CollisionSample> & collisions)
{
  std::ostringstream out;
  const std::size_t cap = std::min<std::size_t>(16U, collisions.size());
  for (std::size_t i = 0; i < cap; ++i) {
    if (i != 0U) {
      out << ',';
    }
    out << collisions[i].trajectory_index;
  }
  if (collisions.size() > cap) {
    out << ",...";
  }
  return out.str();
}

// 把拒绝点周围的规划栅格原样打成一行,用于区分
// "车自身格子被占"与"外侧存在成片障碍"。只在 footprint
// 拒绝分支调用,窗口为 ±radius_m,行序从北到南。
std::string describeGridWindow(
  const nav_msgs::msg::OccupancyGrid & grid, double center_x, double center_y,
  double radius_m, int obstacle_value_threshold, bool unknown_is_obstacle)
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0U || grid.info.height == 0U) {
    return std::string("<invalid grid>");
  }
  const double resolution = grid.info.resolution;
  const int span = std::max(1, static_cast<int>(std::lround(radius_m / resolution)));
  const int center_mx = static_cast<int>(
    std::floor((center_x - grid.info.origin.position.x) / resolution));
  const int center_my = static_cast<int>(
    std::floor((center_y - grid.info.origin.position.y) / resolution));
  std::ostringstream stream;
  stream << "res=" << resolution << " center_cell=(" << center_mx << ", " << center_my
         << ") span=" << span << " rows_north_to_south=[";
  for (int dy = span; dy >= -span; --dy) {
    const int my = center_my + dy;
    for (int dx = -span; dx <= span; ++dx) {
      const int mx = center_mx + dx;
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) ||
        my >= static_cast<int>(grid.info.height))
      {
        // 越界在 footprint 判据里等价于占据,这里用独立
      // 符号标出来,避免与真实障碍混淆。
        stream << 'X';
        continue;
      }
      const int value = grid.data[
        static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
        static_cast<std::size_t>(mx)];
      if (value < 0) {
        stream << (unknown_is_obstacle ? 'U' : 'u');
      } else if (value >= obstacle_value_threshold) {
        stream << '#';
      } else if (value == 0) {
        stream << '.';
      } else {
        // 连续风险 1..threshold-1 保留量级,便于判断是否贴着 terrain 风险带走。
        stream << static_cast<char>('0' + std::min(9, value / 10));
      }
    }
    if (dy != -span) {
      stream << '/';
    }
  }
  stream << ']';
  return stream.str();
}

// 两条同几何参考的 yaw 序列是否不同（逐点 1e-9 rad）；相同时不必再过两遍足迹检查。
bool yawSequencesDiffer(
  const minco_planner::ReferenceTrajectory & a, const minco_planner::ReferenceTrajectory & b)
{
  if (a.points.size() != b.points.size()) {
    return true;
  }
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    if (std::abs(std::remainder(a.points[i].yaw - b.points[i].yaw, 2.0 * M_PI)) > 1e-9) {
      return true;
    }
  }
  return false;
}

struct SplitGuideResult
{
  bool valid{false};
  double total_length{0.0};
  double split_s{0.0};
  Eigen::Vector2d junction_pos{0.0, 0.0};
  Eigen::Vector2d junction_tangent{1.0, 0.0};
  double junction_turn_rad{0.0};
  nav_msgs::msg::Path prefix;
  nav_msgs::msg::Path suffix;
};

double pathArcLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return length;
}

SplitGuideResult splitPathAtArcLength(
  const nav_msgs::msg::Path & path, double target_split_s, double min_tail_s)
{
  SplitGuideResult out;
  if (path.poses.size() < 2U || target_split_s <= 0.2) {
    return out;
  }
  std::vector<double> cumulative(path.poses.size(), 0.0);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double seg = std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    cumulative[i] = cumulative[i - 1] + seg;
  }
  out.total_length = cumulative.back();
  if (out.total_length <= target_split_s + std::max(0.5, min_tail_s)) {
    return out;
  }
  const auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target_split_s);
  if (upper == cumulative.begin() || upper == cumulative.end()) {
    return out;
  }
  const std::size_t seg_idx = static_cast<std::size_t>(std::distance(cumulative.begin(), upper)) - 1U;
  const double seg_len = cumulative[seg_idx + 1U] - cumulative[seg_idx];
  const double u = seg_len > 1e-9 ?
    std::clamp((target_split_s - cumulative[seg_idx]) / seg_len, 0.0, 1.0) : 0.0;
  const auto & a = path.poses[seg_idx];
  const auto & b = path.poses[seg_idx + 1U];
  geometry_msgs::msg::PoseStamped junction = a;
  junction.pose.position.x = a.pose.position.x + u * (b.pose.position.x - a.pose.position.x);
  junction.pose.position.y = a.pose.position.y + u * (b.pose.position.y - a.pose.position.y);
  junction.pose.position.z = a.pose.position.z + u * (b.pose.position.z - a.pose.position.z);

  Eigen::Vector2d dir(
    b.pose.position.x - a.pose.position.x,
    b.pose.position.y - a.pose.position.y);
  if (dir.norm() <= 1e-6 && seg_idx + 2U < path.poses.size()) {
    dir = Eigen::Vector2d(
      path.poses[seg_idx + 2U].pose.position.x - a.pose.position.x,
      path.poses[seg_idx + 2U].pose.position.y - a.pose.position.y);
  }
  if (dir.norm() <= 1e-6) {
    return out;
  }
  out.junction_tangent = dir.normalized();
  out.junction_pos = Eigen::Vector2d(junction.pose.position.x, junction.pose.position.y);
  out.split_s = target_split_s;

  // Estimate local corner turn within +/- 0.8 m around split_s to cap junction speed on turns.
  const double before_s = std::max(0.0, target_split_s - 0.8);
  const double after_s = std::min(out.total_length, target_split_s + 0.8);
  const auto sampleAtS = [&](double s_query) {
    const auto it = std::lower_bound(cumulative.begin(), cumulative.end(), s_query);
    if (it == cumulative.begin()) {
      return Eigen::Vector2d(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
    }
    if (it == cumulative.end()) {
      return Eigen::Vector2d(path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(cumulative.begin(), it)) - 1U;
    const double d = cumulative[idx + 1U] - cumulative[idx];
    const double r = d > 1e-9 ? std::clamp((s_query - cumulative[idx]) / d, 0.0, 1.0) : 0.0;
    return Eigen::Vector2d(
      path.poses[idx].pose.position.x + r * (path.poses[idx + 1U].pose.position.x - path.poses[idx].pose.position.x),
      path.poses[idx].pose.position.y + r * (path.poses[idx + 1U].pose.position.y - path.poses[idx].pose.position.y));
  };
  const Eigen::Vector2d v_in = out.junction_pos - sampleAtS(before_s);
  const Eigen::Vector2d v_out = sampleAtS(after_s) - out.junction_pos;
  if (v_in.norm() > 1e-4 && v_out.norm() > 1e-4) {
    out.junction_turn_rad = std::abs(std::atan2(
      v_in.x() * v_out.y() - v_in.y() * v_out.x(), v_in.dot(v_out)));
  }

  out.prefix.header = path.header;
  for (std::size_t i = 0; i <= seg_idx; ++i) {
    out.prefix.poses.push_back(path.poses[i]);
  }
  if (std::hypot(
      out.prefix.poses.back().pose.position.x - junction.pose.position.x,
      out.prefix.poses.back().pose.position.y - junction.pose.position.y) > 1e-4)
  {
    out.prefix.poses.push_back(junction);
  }

  out.suffix.header = path.header;
  out.suffix.poses.push_back(junction);
  for (std::size_t i = seg_idx + 1U; i < path.poses.size(); ++i) {
    if (std::hypot(
        out.suffix.poses.back().pose.position.x - path.poses[i].pose.position.x,
        out.suffix.poses.back().pose.position.y - path.poses[i].pose.position.y) > 1e-4)
    {
      out.suffix.poses.push_back(path.poses[i]);
    }
  }
  out.valid = out.prefix.poses.size() >= 2U && out.suffix.poses.size() >= 2U;
  return out;
}

minco_planner::ReferenceTrajectory spliceReferenceTrajectories(
  const minco_planner::ReferenceTrajectory & prefix,
  const minco_planner::ReferenceTrajectory & tail,
  double tail_base_time_offset)
{
  minco_planner::ReferenceTrajectory out = prefix;
  if (!prefix.valid() || tail.points.empty()) {
    return minco_planner::ReferenceTrajectory();
  }
  double prev_t = out.points.back().t;
  double prev_s = out.points.back().s;
  double prev_x = out.points.back().x;
  double prev_y = out.points.back().y;
  for (const auto & pt : tail.points) {
    const double shifted_t = pt.t + tail_base_time_offset;
    if (shifted_t <= prev_t + 1e-4) {
      continue;
    }
    minco_planner::ReferencePoint copy = pt;
    copy.t = shifted_t;
    prev_s += std::hypot(copy.x - prev_x, copy.y - prev_y);
    copy.s = prev_s;
    out.points.push_back(copy);
    prev_t = copy.t;
    prev_x = copy.x;
    prev_y = copy.y;
  }
  return out.valid() ? out : minco_planner::ReferenceTrajectory();
}

double empiricalPercentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double clamped = std::clamp(q, 0.0, 1.0);
  const std::size_t idx = static_cast<std::size_t>(
    std::ceil(clamped * static_cast<double>(values.size())) - 1.0);
  return values[std::min(idx, values.size() - 1U)];
}

}  // namespace

MincoPlannerNode::MincoPlannerNode(const rclcpp::NodeOptions & options)
: Node("minco_planner", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  declareAndLoadParams();

  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions map_options;
  map_options.callback_group = map_callback_group_;
  rclcpp::SubscriptionOptions health_options;
  health_options.callback_group = health_callback_group_;

  // Match adapter TRANSIENT_LOCAL publication. VOLATILE missed the first
  // planning grid under DualMap-era ready races (recovery231:
  // ready=1 + snapshot log, but MINCO kept MAP_UNREADY / zero plans).
  if (!planning_snapshot_topic_.empty()) {
    planning_snapshot_sub_ =
      create_subscription<ats_navigation_interfaces::msg::PlanningMapSnapshot>(
      planning_snapshot_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MincoPlannerNode::onPlanningSnapshot, this, std::placeholders::_1),
      map_options);
  } else {
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MincoPlannerNode::onGrid, this, std::placeholders::_1), map_options);
  }
  if (!prior_guide_map_topic_.empty()) {
    prior_guide_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prior_guide_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr prior) {
        if (prior_guide_map_ && prior_guide_map_->header.frame_id == prior->header.frame_id &&
          prior_guide_map_->info.origin == prior->info.origin &&
          prior_guide_map_->info.resolution == prior->info.resolution &&
          prior_guide_map_->info.width == prior->info.width &&
          prior_guide_map_->info.height == prior->info.height &&
          prior_guide_map_->data == prior->data)
        {
          return;
        }
        global_route_.reset();
        prior_guide_map_.reset();
        prior_blocked_squared_.clear();
        if (prior->header.frame_id.empty() || !(prior->info.resolution > 0.0) ||
          !std::isfinite(prior->info.resolution) || prior->data.empty() ||
          !finitePose(prior->info.origin) || prior->data.size() !=
          static_cast<std::size_t>(prior->info.width) * prior->info.height)
        {
          return;
        }
        prior_blocked_squared_ = computeBlockedSquaredDistanceCells(
          *prior, {obstacle_value_threshold_, unknown_is_obstacle_});
        prior_guide_map_ = std::move(prior);
      }, planning_options);
  }
  if (!goal_topic_.empty()) {
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&MincoPlannerNode::onGoal, this, std::placeholders::_1), planning_options);
  }
  if (!goal_request_topic_.empty()) {
    goal_request_sub_ = create_subscription<ats_navigation_interfaces::msg::PlannerGoal>(
      goal_request_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&MincoPlannerNode::onPlannerGoal, this, std::placeholders::_1), planning_options);
  }
  if (!raw_path_topic_.empty()) {
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, rclcpp::QoS(1));
  }
  if (!preprocessed_guide_topic_.empty()) {
    preprocessed_guide_pub_ = create_publisher<nav_msgs::msg::Path>(
      preprocessed_guide_topic_, rclcpp::QoS(1).reliable());
  }
  if (!esdf_refined_guide_topic_.empty()) {
    esdf_refined_guide_pub_ = create_publisher<nav_msgs::msg::Path>(
      esdf_refined_guide_topic_, rclcpp::QoS(1).reliable());
  }
  if (planner_manages_emergency_stop_) {
    reference_path_pub_ =
      create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(1));
  }
  if (!candidate_reference_path_topic_.empty()) {
    candidate_reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      candidate_reference_path_topic_, rclcpp::QoS(10).reliable());
  }
  if (!debug_marker_topic_.empty()) {
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic_, rclcpp::QoS(1));
  }
  if (planner_manages_emergency_stop_) {
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());
  }
  if (!planner_status_topic_.empty()) {
    planner_status_pub_ = create_publisher<ats_navigation_interfaces::msg::PlannerStatus>(
      planner_status_topic_, rclcpp::QoS(1).reliable().transient_local());
  }
  safety_state_.map_ready = map_ready_topic_.empty();
  if (!map_ready_topic_.empty()) {
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MincoPlannerNode::onMapReady, this, std::placeholders::_1), health_options);
  }
  safety_watchdog_timer_ = create_wall_timer(
    std::chrono::duration<double>(emergency_stop_heartbeat_period_sec_),
    std::bind(&MincoPlannerNode::onMapReadyWatchdog, this), health_callback_group_);
  runtime_safety_recheck_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(0.1, runtime_safety_recheck_hz_)),
    std::bind(&MincoPlannerNode::onRuntimeSafetyRecheck, this), health_callback_group_);
  publishEmergencyStop(true);

  RCLCPP_INFO(
    get_logger(), "【规划器就绪】地图输入='%s' 目标='%s' 请求='%s' 原始路径='%s' 参考路径='%s'",
    planning_snapshot_topic_.empty() ? grid_topic_.c_str() : planning_snapshot_topic_.c_str(), goal_topic_.c_str(), goal_request_topic_.c_str(), raw_path_topic_.c_str(),
    reference_path_topic_.c_str());
}

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

GridSearchResult MincoPlannerNode::runGraphSearch(
  const nav_msgs::msg::OccupancyGrid & planning_grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool goal_pose_footprint_verified,
  double & used_clearance,
  const std::vector<double> * blocked_squared,
  bool allow_partial_subgoal)
{
  // 显式配置的下限按用户意图使用;自动下限必须补上栅格量化余量,否则梯子会稳定产出
  // footprint gate 必然拒绝的路径,把"窄通道不可通行"表现成目标超时。
  const double effective_floor = search_clearance_floor_configured_
    ? search_clearance_floor_
    : footprintConsistentClearanceFloor(
      inscribed_footprint_radius_, planning_grid.info.resolution,
      preferred_search_clearance_);
  RCLCPP_INFO_ONCE(
    get_logger(),
    "【图搜索净空生效】下限=%.3f m（内切半径=%.3f m 栅格分辨率=%.3f m 量化余量=%.3f m）；"
    "低于此值的路径无法通过矩形足迹门禁。",
    effective_floor, inscribed_footprint_radius_, planning_grid.info.resolution,
    gridQuantizationAllowance(planning_grid.info.resolution));
  std::vector<double> ladder {preferred_search_clearance_};
  if (clearance_relaxation_enabled_ && effective_floor + 1e-6 < preferred_search_clearance_) {
    ladder.push_back(effective_floor);
  }

  bool enable_partial = allow_partial_subgoal && partial_subgoal_continuation_enabled_;
  if (enable_partial && planning_grid.info.resolution > 0.0 && !planning_grid.data.empty()) {
    // 若目标点已在当前窗口内且整个容差圆都被实测确认占据（>= obstacle_value_threshold，而非 -1 未知），
    // 则属于明确物理不可达目标，禁止生成 partial 子目标，立即走确定性急停拒绝。
    const double g_yaw = tf2::getYaw(planning_grid.info.origin.orientation);
    const double cg = std::cos(g_yaw);
    const double sg = std::sin(g_yaw);
    const double dx = goal.pose.position.x - planning_grid.info.origin.position.x;
    const double dy = goal.pose.position.y - planning_grid.info.origin.position.y;
    const double lx = (cg * dx + sg * dy) / planning_grid.info.resolution;
    const double ly = (-sg * dx + cg * dy) / planning_grid.info.resolution;
    if (lx >= 0.0 && ly >= 0.0 &&
      lx < static_cast<double>(planning_grid.info.width) &&
      ly < static_cast<double>(planning_grid.info.height))
    {
      const int rad_cells = std::max(
        0, static_cast<int>(std::ceil(
          std::max(0.0, goal_pose_admission_params_.position_tolerance_m) /
          planning_grid.info.resolution)));
      const int cx = static_cast<int>(std::floor(lx));
      const int cy = static_cast<int>(std::floor(ly));
      bool all_known_occupied = true;
      for (int oy = -rad_cells; oy <= rad_cells && all_known_occupied; ++oy) {
        for (int ox = -rad_cells; ox <= rad_cells && all_known_occupied; ++ox) {
          const int ix = cx + ox;
          const int iy = cy + oy;
          if (ix < 0 || iy < 0 ||
            ix >= static_cast<int>(planning_grid.info.width) ||
            iy >= static_cast<int>(planning_grid.info.height))
          {
            all_known_occupied = false;
            break;
          }
          const int8_t val = planning_grid.data[
            static_cast<std::size_t>(iy) * planning_grid.info.width +
            static_cast<std::size_t>(ix)];
          if (val < obstacle_value_threshold_) {
            all_known_occupied = false;
          }
        }
      }
      if (all_known_occupied) {
        enable_partial = false;
      }
    }
  }

  const double start_goal_dist = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  const auto search_at = [&](double clearance, double endpoint_minimum = -1.0) {
    const auto begin = std::chrono::steady_clock::now();
    GridSearchOptions options;
    options.clearance = clearance;
    options.endpoint_minimum = endpoint_minimum;
    if (enable_partial) {
      options.can_stop = [&](const geometry_msgs::msg::PoseStamped & cand) {
        const double step_dist = std::hypot(
          cand.pose.position.x - start.pose.position.x,
          cand.pose.position.y - start.pose.position.y);
        if (step_dist < partial_subgoal_min_advance_m_) {
          return false;
        }
        const double cand_goal_dist = std::hypot(
          goal.pose.position.x - cand.pose.position.x,
          goal.pose.position.y - cand.pose.position.y);
        if (cand_goal_dist + 0.15 > start_goal_dist) {
          return false;
        }
        return goalPoseFootprintFree(
          cand.pose.position.x, cand.pose.position.y,
          tf2::getYaw(cand.pose.orientation), planning_grid, footprint_params_);
      };
    }
    GridSearchResult attempt = graph_search_.plan(
      planning_grid, start, goal, blocked_squared, options);
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    RCLCPP_INFO(
      get_logger(),
      "TRACE graph_search mode=%s requested_clearance=%.3f clearance=%.3f "
      "expanded=%d wall_ms=%.3f cost=%.9f length=%.3f partial=%d result='%s'",
      search_algorithm_.c_str(), clearance, attempt.clearance, attempt.expanded_nodes,
      wall_ms, attempt.cost, attempt.length, attempt.partial ? 1 : 0, attempt.reason.c_str());
    return attempt;
  };

  GridSearchResult last_result;
  GridSearchResult best_partial;
  double best_partial_goal_dist = std::numeric_limits<double>::infinity();
  for (std::size_t level = 0; level < ladder.size(); ++level) {
    const double clearance = ladder[level];
    if (last_result.expanded_nodes > 0 && last_result.clearance <= clearance + 1e-9) {
      break;  // An endpoint relaxation already searched this effective clearance.
    }
    GridSearchResult attempt = search_at(clearance);
    if (attempt.success) {
      // 首选档比窄通道半宽大时,绕远路也能在这一档成功,梯子就永远不降档。
      // 路径相对直线明显绕行时,用下限档再搜一次,明显更短才取代;
      // 安全仍由矩形足迹门禁与局部修复裁定。
      if (level == 0 && ladder.size() > 1 && attempt.clearance > ladder[1] + 1e-9) {
        const double straight = std::hypot(
          goal.pose.position.x - start.pose.position.x,
          goal.pose.position.y - start.pose.position.y);
        if (shouldProbeRelaxedClearance(attempt.length, straight, clearance_detour_ratio_)) {
          GridSearchResult relaxed = search_at(ladder[1]);
          if (relaxed.success &&
            shouldTakeRelaxedRoute(attempt.length, relaxed.length, clearance_detour_ratio_))
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), log_throttle_ms_,
              "【绕行回退】首选净空 %.3f m 路径 %.2f m（直线 %.2f m）明显绕行，"
              "改用净空 %.3f m 路径 %.2f m；最终仍以航向感知足迹门禁为准。",
              clearance, attempt.length, straight, ladder[1], relaxed.length);
            used_clearance = relaxed.clearance;
            return relaxed;
          }
        }
      }
      used_clearance = attempt.clearance;
      if (clearance + 1e-6 < preferred_search_clearance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【规划净空放宽】图搜索从 %.3f m 放宽到 %.3f m 后成功；最终仍以航向感知足迹门禁为准。",
          preferred_search_clearance_, clearance);
      }
      return attempt;
    }
    if (attempt.partial && !attempt.path.poses.empty()) {
      const auto & p = attempt.path.poses.back().pose.position;
      const double rem_dist = std::hypot(
        goal.pose.position.x - p.x, goal.pose.position.y - p.y);
      if (!best_partial.partial || rem_dist + 0.35 < best_partial_goal_dist) {
        best_partial = attempt;
        best_partial_goal_dist = rem_dist;
      }
    }
    last_result = attempt;
  }

  // 最后一档:目标格净空低于 footprint 一致下限,但目标位姿本身已通过同一套矩形
  // 足迹门禁。目标格净空是各向同性代理量,矩形门禁是精确判定;此时继续用代理量
  // 否决精确判定,结果是一条路径都产不出来(domain 187 目标 9:目标格净空约 0.26 m
  // 对下限 0.341 m,图搜索报 goal occupied,7 个规划周期零可执行计划直到看门狗耗尽)。
  // 放宽只作用于"目标格是否可作为终点",搜索器随后按实测目标净空重规划整条路径,
  // 安全性仍由重解 MINCO 之后的矩形足迹门禁与局部修复裁定,不是靠放宽判据换成功。
  const double relaxed_endpoint_min = gridQuantizationAllowance(planning_grid.info.resolution);
  if (goal_pose_footprint_verified && endpoint_clearance_relaxation_enabled_ &&
    (last_result.expanded_nodes == 0 || last_result.clearance > effective_floor + 1e-9) &&
    !last_result.success && relaxed_endpoint_min + 1e-6 < effective_floor)
  {
    GridSearchResult attempt = search_at(effective_floor, relaxed_endpoint_min);
    if (attempt.success) {
      used_clearance = attempt.clearance;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终点净空放宽】终点下限从 %.3f m 放宽到 %.3f m 后成功；目标已通过矩形足迹门禁，"
        "轨迹安全仍由该门禁最终裁定。",
        effective_floor, relaxed_endpoint_min);
      return attempt;
    }
    last_result = attempt;
  }

  if (best_partial.partial) {
    used_clearance = best_partial.clearance;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【远目标局部前沿接续】目标超出当前观测连通域，推进至安全前沿子目标 (%.3f, %.3f) "
      "length=%.2f m clearance=%.3f m reason='%s'。",
      best_partial.path.poses.back().pose.position.x,
      best_partial.path.poses.back().pose.position.y,
      best_partial.length, best_partial.clearance, best_partial.reason.c_str());
    return best_partial;
  }

  used_clearance = ladder.back();
  return last_result;
}

GridSearchResult MincoPlannerNode::planRoute(
  const std::shared_ptr<const PlanningMapSnapshot> & snapshot,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const GoalUuid & goal_uuid,
  std::uint64_t goal_id, std::uint64_t localization_epoch, bool goal_pose_footprint_verified,
  double & used_clearance)
{
  const auto & grid = snapshot->grid;
  const auto * squared = snapshot->blocked_squared_cells.get();
  const bool managed_goal = std::any_of(goal_uuid.begin(), goal_uuid.end(),
      [](std::uint8_t byte) {return byte != 0;});
  const bool prior_mode = managed_goal && prior_guide_map_ &&
    prior_guide_map_->header.frame_id == grid.header.frame_id;
  GridSearchResult result;
  if (prior_mode) {
    const bool same_goal = global_route_ && global_route_->goal_uuid == goal_uuid &&
      global_route_->goal_id == goal_id && global_route_->localization_epoch == localization_epoch &&
      global_route_->path->header.frame_id == grid.header.frame_id &&
      std::hypot(global_route_->goal.position.x - goal.pose.position.x,
      global_route_->goal.position.y - goal.pose.position.y) <= 0.001 &&
      std::abs(std::remainder(tf2::getYaw(global_route_->goal.orientation) -
      tf2::getYaw(goal.pose.orientation), 2.0 * M_PI)) <= 0.001;
    nav_msgs::msg::Path remaining;
    double clearance = 0.0;
    if (same_goal) {
      remaining = remainingGuide(*global_route_->path, start, replan_start_params_.deviation_max_m);
      clearance = global_route_->clearance;
    } else {
      auto prior = runGraphSearch(
        *prior_guide_map_, start, goal, goal_pose_footprint_verified, clearance,
        &prior_blocked_squared_);
      RCLCPP_INFO(
        get_logger(), "TRACE global_route source=prior goal=%llu generation=%llu "
        "publication=%llu source_generation=%llu points=%zu length=%.3f result='%s'",
        static_cast<unsigned long long>(goal_id), static_cast<unsigned long long>(snapshot->generation),
        static_cast<unsigned long long>(snapshot->publication_sequence),
        static_cast<unsigned long long>(snapshot->source_generation),
        prior.path.poses.size(), prior.length, prior.reason.c_str());
      remaining = std::move(prior.path);
    }
    if (!remaining.poses.empty()) {
      remaining.header = grid.header;
      std::optional<geometry_msgs::msg::Point> reference_conflict;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (reference_conflict_ && reference_conflict_->goal_id == goal_id &&
          reference_conflict_->localization_epoch == localization_epoch &&
          reference_conflict_->snapshot_generation == snapshot->generation)
        {
          reference_conflict = reference_conflict_->position;
          reference_conflict_.reset();
        }
      }
      const auto begin = std::chrono::steady_clock::now();
      const auto * accepted_grid = same_goal ? &global_route_->snapshot->grid : prior_guide_map_.get();
      auto local = rejoinGlobalRoute(
        remaining, grid, *squared, clearance, safety_checker_, graph_search_, reference_conflict,
        accepted_grid, local_route_params_);
      if (local.obstruction) {
        const double wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();
        const auto rejoin = local.rejoin.value_or(geometry_msgs::msg::Point());
        RCLCPP_INFO(
          get_logger(), "TRACE local_rejoin goal=%llu generation=%llu publication=%llu "
          "source_generation=%llu cached_publication=%llu cached_source_generation=%llu "
          "blocked=%zu..%zu expanded=%d wall_ms=%.3f bypass_m=%.3f rejoin=(%.4f,%.4f) result='%s'",
          static_cast<unsigned long long>(goal_id),
          static_cast<unsigned long long>(snapshot->generation),
          static_cast<unsigned long long>(snapshot->publication_sequence),
          static_cast<unsigned long long>(snapshot->source_generation),
          static_cast<unsigned long long>(same_goal ? global_route_->snapshot->publication_sequence : 0),
          static_cast<unsigned long long>(same_goal ? global_route_->snapshot->source_generation : 0),
          local.obstruction->first, local.obstruction->last, local.route.expanded_nodes,
          wall_ms, local.bypass_length, rejoin.x, rejoin.y, local.route.reason.c_str());
      }
      result = std::move(local.route);
    }
  }
  if (!result.success) {
    const bool allow_partial = partial_subgoal_continuation_enabled_ &&
      (!partial_subgoal_only_without_prior_ || !prior_mode);
    result = runGraphSearch(
      grid, start, goal, goal_pose_footprint_verified, used_clearance, squared, allow_partial);
    if (prior_mode) {
      RCLCPP_INFO(
        get_logger(), "TRACE global_route source=fused goal=%llu publication=%llu result='%s'",
        static_cast<unsigned long long>(goal_id),
        static_cast<unsigned long long>(snapshot->publication_sequence), result.reason.c_str());
    }
  }
  global_route_.reset();
  if (result.success && prior_mode) {
    global_route_ = GlobalRoute{
      goal_uuid, goal_id, localization_epoch, snapshot, goal.pose,
      std::make_shared<const nav_msgs::msg::Path>(result.path), result.clearance};
  }
  if (result.success || result.partial) {
    used_clearance = result.clearance;
  }
  return result;
}

void MincoPlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const auto received_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  ingestMapSnapshot(
    [this, &msg](std::uint64_t generation) {
      return PlanningMapSnapshot::create(
        generation, *msg, obstacle_value_threshold_, unknown_is_obstacle_);
    }, msg->header.stamp, received_steady_ns);
}

void MincoPlannerNode::onPlanningSnapshot(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot::SharedPtr msg)
{
  const auto received_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (!msg->ready) {
    // 地图所有者撤回地图（输入过期、epoch 变化、融合失败），与 ready=false 心跳同一结果：
    // 已提交快照与参考不再可用。
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      invalidateMapLocked();
      publishEmergencyStop(true);
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划快照不可用】地图所有者发布 ready=false（epoch=%llu sequence=%llu），已急停。",
      static_cast<unsigned long long>(msg->localization_epoch),
      static_cast<unsigned long long>(msg->publication_sequence));
    return;
  }
  if (msg->occupied_value_threshold != obstacle_value_threshold_ ||
    msg->unknown_is_obstacle != unknown_is_obstacle_)
  {
    // 距离场按快照声明的阻塞语义计算；与规划器不同就不能复用，fromMessage 返回空并急停。
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划快照策略不一致】快照阈值=%d unknown_is_obstacle=%d，规划器阈值=%d "
      "unknown_is_obstacle=%d，拒绝使用。",
      static_cast<int>(msg->occupied_value_threshold), static_cast<int>(msg->unknown_is_obstacle),
      obstacle_value_threshold_, static_cast<int>(unknown_is_obstacle_));
  }
  ingestMapSnapshot(
    [this, &msg](std::uint64_t generation) {
      return PlanningMapSnapshot::fromMessage(
        generation, *msg, obstacle_value_threshold_, unknown_is_obstacle_);
    }, msg->source_stamp, received_steady_ns);
}

void MincoPlannerNode::ingestMapSnapshot(
  const SnapshotBuilder & build, const builtin_interfaces::msg::Time & source_stamp,
  std::int64_t received_steady_ns)
{
  std::uint64_t candidate_generation = 0;
  std::uint64_t candidate_health_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    candidate_generation = next_map_generation_ + 1;
    candidate_health_epoch = map_health_epoch_;
  }
  const auto snapshot = build(candidate_generation);
  if (!snapshot) {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      invalidateMapLocked();
      publishEmergencyStop(true);
    }
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【规划栅格错误】收到无效规划栅格，已拒绝并急停。");
    return;
  }
  // 可选：安全语义变化的新 snapshot 若仍让已提交参考的完整剩余段通过同一套矩形足迹
  // 门禁（含提交门同源的擦边逃逸规则），就把参考改绑到新 generation 继续执行，而不是
  // 每次地图刷新都作废参考、急停、等 GoalManager 重派。判定在锁外对不可变 snapshot
  // 完成，锁内再核对参考身份未变；任何不一致都回到原先的作废路径（fail-closed）。
  std::optional<ActiveSafetyReference> retain_candidate;
  if (retain_safe_reference_on_snapshot_change_) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (active_safety_reference_ && latest_map_snapshot_ && safety_state_.plan_safe &&
      snapshot->localization_epoch == latest_map_snapshot_->localization_epoch &&
      !snapshot->hasSameSafetyContent(*latest_map_snapshot_))
    {
      retain_candidate = active_safety_reference_;
    }
  }
  std::string retain_reason;
  geometry_msgs::msg::PoseStamped progress_pose;
  std::optional<geometry_msgs::msg::Point> reference_conflict;
  std::size_t far_collision_count = 0;
  // map->odom 在同一定位 epoch 内的连续修正不作废参考，检查按当前 TF 重新表达的参考。
  const ReferenceTrajectory retain_aligned = retain_candidate ?
    alignedToCurrentTf(*retain_candidate) : ReferenceTrajectory();
  const bool retain_verdict = retain_candidate &&
    retain_candidate->trajectory.header.frame_id == snapshot->grid.header.frame_id &&
    remainingReferenceSafeOn(
      retain_aligned, snapshot->grid, &retain_reason, &progress_pose, &reference_conflict,
      &far_collision_count);
  bool guide_blocked = false;
  if (replan_advisory_enabled_ && retain_verdict && retain_candidate->guide &&
    retain_candidate->guide_snapshot)
  {
    const auto remaining = remainingGuide(
      *retain_candidate->guide, progress_pose, replan_start_params_.deviation_max_m);
    guide_blocked = obstructedGuide(
      traceGuide(remaining), snapshot->grid, safety_checker_,
      &retain_candidate->guide_snapshot->grid).has_value();
    if (guide_blocked) {
      retain_reason = "global guide footprint conflict";
    }
  }
  // 远端冲突、近端安全：继续执行并建议重规划，交给 10 Hz 运行期复检兜底近端。
  const bool far_conflict_raw = replan_advisory_enabled_ && retain_candidate &&
    (guide_blocked || (!retain_verdict && runtimeWindowSafeOn(retain_aligned, snapshot->grid)));
  const bool far_conflict_severe = !retain_verdict && far_collision_count >= 4U;
  std::optional<ActiveSafetyReference> advised_reference;
  bool retained_reference = false;
  std::uint64_t retained_reference_goal = 0;
  bool retained_identical_snapshot = false;
  std::uint64_t retained_generation = 0;
  std::string retained_digest;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (
      candidate_health_epoch != map_health_epoch_ ||
      candidate_generation != next_map_generation_ + 1)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【规划栅格丢弃】地图健康状态在构建期间变化，已丢弃本次栅格。");
      return;
    }
    const bool replaces_existing_snapshot = latest_map_snapshot_ != nullptr;
    retained_identical_snapshot =
      replaces_existing_snapshot && snapshot->hasSameSafetyContent(*latest_map_snapshot_);
    if (retained_identical_snapshot) {
      retained_generation = latest_map_snapshot_->generation;
      retained_digest = latest_map_snapshot_->safety_content_digest;
    } else {
      reference_conflict_.reset();
      if (retain_candidate && reference_conflict) {
        reference_conflict_ = ReferenceConflict{
          retain_candidate->goal_id, retain_candidate->localization_epoch,
          snapshot->generation, *reference_conflict};
      }
      retained_reference = (retain_verdict || far_conflict_raw) && replaces_existing_snapshot &&
        active_safety_reference_ && safety_state_.plan_safe &&
        safety_state_.mapSnapshotUsable(snapshot->generation) &&
        active_safety_reference_->goal_id == retain_candidate->goal_id &&
        active_safety_reference_->localization_epoch == retain_candidate->localization_epoch &&
        active_safety_reference_->plan_request_sequence ==
          retain_candidate->plan_request_sequence &&
        active_safety_reference_->map_generation == retain_candidate->map_generation &&
        active_safety_reference_->trajectory.header.stamp ==
          retain_candidate->trajectory.header.stamp;
      if (retained_reference) {
        active_safety_reference_->map_generation = snapshot->generation;
        retained_reference_goal = active_safety_reference_->goal_id;
        if (far_conflict_raw) {
          if (!far_conflict_reference_stamp_ ||
            *far_conflict_reference_stamp_ != active_safety_reference_->trajectory.header.stamp)
          {
            far_conflict_reference_stamp_ = active_safety_reference_->trajectory.header.stamp;
            far_conflict_streak_ = 0;
          }
          ++far_conflict_streak_;
          const bool far_conflict_confirmed =
            far_conflict_severe || far_conflict_streak_ >= far_conflict_confirm_snapshots_;
          if (far_conflict_confirmed && (!replan_advised_reference_stamp_ ||
            *replan_advised_reference_stamp_ != active_safety_reference_->trajectory.header.stamp))
          {
            replan_advised_reference_stamp_ = active_safety_reference_->trajectory.header.stamp;
            advised_reference = active_safety_reference_;
          }
        } else {
          far_conflict_streak_ = 0;
          far_conflict_reference_stamp_.reset();
        }
      } else {
        active_safety_reference_.reset();
        far_conflict_streak_ = 0;
        far_conflict_reference_stamp_.reset();
      }
      next_map_generation_ = snapshot->generation;
      latest_map_snapshot_ = snapshot;
      if (replaces_existing_snapshot && !retained_reference) {
        // A safety-semantic change creates a new immutable map identity.  The
        // old reference cannot execute even though the source heartbeat stays
        // healthy; a fresh plan must bind this exact local generation.
        safety_state_.invalidatePlanForNewMap(snapshot->generation);
        invalidatePlannerStatusLocked(
          ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
          snapshot->generation);
      }
      if (map_ready_topic_.empty()) {
        safety_state_.map_ready = true;
      }
      publishEmergencyStop(safety_state_.emergencyStopRequired());
    }
  }
  if (map_commit_trace_enabled_) {
    const auto committed_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(get_logger(),
      "S3 map_commit stamp_ns=%lld received_steady_ns=%lld committed_steady_ns=%lld digest=%s",
      static_cast<long long>(rclcpp::Time(source_stamp).nanoseconds()),
      static_cast<long long>(received_steady_ns), static_cast<long long>(committed_steady_ns),
      snapshot->safety_content_digest.c_str());
  }
  if (retain_candidate && !retained_reference && !retained_identical_snapshot) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【参考轨迹失效】地图快照变化后无法保留目标=%llu 的已提交参考（判定=%d 前视窗=%.2f s 原因=%s），"
      "将在 generation=%llu 上重新规划。",
      static_cast<unsigned long long>(retain_candidate->goal_id),
      static_cast<int>(retain_verdict), retain_reference_horizon_sec_,
      retain_reason.empty() ? "身份或帧不一致" : retain_reason.c_str(),
      static_cast<unsigned long long>(snapshot->generation));
  }
  if (advised_reference) {
    // 直接发布、不进入 planner_status_state_：它不是该请求的终态，不能被后续作废复用。
    // STATE_ACCEPTED 不会改变 MPC/arbiter 的地图授权。
    if (planner_status_pub_) {
      ats_navigation_interfaces::msg::PlannerStatus status;
      status.header.stamp = now();
      status.header.frame_id = global_frame_;
      status.goal_id = advised_reference->goal_id;
      status.localization_epoch = advised_reference->localization_epoch;
      status.plan_request_sequence = advised_reference->plan_request_sequence;
      status.map_generation = snapshot->generation;
      status.map_publication_sequence = advised_reference->map_publication_sequence;
      status.state = ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED;
      status.failure_reason = ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REPLAN_ADVISED;
      planner_status_pub_->publish(status);
    }
    RCLCPP_WARN(
      get_logger(),
      "【建议重规划】目标=%llu 的参考在 generation=%llu 上远端冲突（%s），运行期窗口 %.2f s 内仍安全，"
      "继续执行并请求无停车重规划。",
      static_cast<unsigned long long>(advised_reference->goal_id),
      static_cast<unsigned long long>(snapshot->generation),
      retain_reason.empty() ? "未知" : retain_reason.c_str(), runtime_safety_horizon_sec_);
  }
  if (retained_reference) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "【参考轨迹保留】目标=%llu 跨地图快照变化继续执行，剩余轨迹通过 generation=%llu 的足迹门禁。",
      static_cast<unsigned long long>(retained_reference_goal),
      static_cast<unsigned long long>(snapshot->generation));
    return;
  }
  if (retained_identical_snapshot) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "【快照复用】安全内容未变，继续使用不可变规划快照 generation=%llu digest=%s；"
      "adapter/source generation 仍仅作为心跳证据。",
      static_cast<unsigned long long>(retained_generation), retained_digest.c_str());
    return;
  }
}

void MincoPlannerNode::onMapReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!msg->data) {
      invalidateMapLocked();
    } else {
      safety_state_.map_ready = true;
    }
    last_map_ready_signal_ = std::chrono::steady_clock::now();
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
}

void MincoPlannerNode::onMapReadyWatchdog()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (
      !map_ready_topic_.empty() && safety_state_.map_ready &&
      !steadyHeartbeatLeaseValid(
        last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_))
    {
      invalidateMapLocked();
      timed_out = true;
    }
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
  if (timed_out) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【地图租约超时】规划地图 ready 心跳超时，已进入急停。");
  }
}

bool MincoPlannerNode::remainingReferenceSafeOn(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
  std::string * reason, geometry_msgs::msg::PoseStamped * progress_pose,
  std::optional<geometry_msgs::msg::Point> * conflict,
  std::size_t * collision_count)
{
  if (collision_count) {
    *collision_count = 0;
  }
  if (trajectory.points.size() < 2) {
    if (reason) {
      *reason = "参考点不足";
    }
    return false;
  }
  const rclcpp::Time reference_stamp(trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  // 保留窗口必须覆盖运行期复检窗口,否则改绑后下一拍复检就可能作废。
  const double progress = referenceProgress(trajectory, grid, elapsed);
  const double horizon_end = retain_reference_horizon_sec_ > 0.0 ?
    progress + std::max(retain_reference_horizon_sec_, runtime_safety_horizon_sec_) :
    std::numeric_limits<double>::infinity();
  const double window_start = std::max(0.0, progress - reference_window_backward_sec_);
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    trajectory, window_start, horizon_end);
  if (remaining.points.size() < 2) {
    if (reason) {
      std::ostringstream text;
      text << "剩余参考不足两点 墙钟=" << elapsed << "s 起点=" << window_start <<
        "s 参考时长=" << trajectory.points.back().t << "s";
      *reason = text.str();
    }
    return false;
  }
  const auto & head = remaining.points.front();
  if (progress_pose) {
    progress_pose->header = grid.header;
    progress_pose->pose.position.x = head.x;
    progress_pose->pose.position.y = head.y;
    progress_pose->pose.orientation.z = std::sin(head.yaw * 0.5);
    progress_pose->pose.orientation.w = std::cos(head.yaw * 0.5);
  }
  const FootprintSafetyResult safety = safety_checker_.check(remaining, grid);
  if (safety.safe) {
    return true;
  }
  const EscapePrefixDecision escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  const bool shallow = escapePrefixContactAdmitted(
    remaining, escape, footprint_params_, grid, escape_prefix_params_);
  if (!shallow && !safety.collisions.empty()) {
    if (collision_count) {
      *collision_count = safety.collisions.size();
    }
    const CollisionSample & first = safety.collisions.front();
    const ReferencePoint & center =
      remaining.points[std::min(first.trajectory_index, remaining.points.size() - 1U)];
    if (conflict) {
      auto & position = conflict->emplace();
      position.x = center.x;
      position.y = center.y;
    }
    if (reason) {
      std::ostringstream text;
      text << "窗口足迹冲突=" << safety.collisions.size() << " 墙钟=" << elapsed << "s 起点=" <<
        window_start << "s 首冲突序号=" << first.trajectory_index << " 中心=(" << center.x << "," <<
        center.y << ") 起点位置=(" << head.x << "," << head.y << ")";
      *reason = text.str();
    }
  }
  return shallow;
}

ReferenceTrajectory MincoPlannerNode::alignedToCurrentTf(
  const ActiveSafetyReference & reference)
{
  if (!reference.control_from_grid) {
    return reference.trajectory;
  }
  geometry_msgs::msg::TransformStamped grid_from_control;
  try {
    grid_from_control = tf_buffer_->lookupTransform(
      reference.trajectory.header.frame_id, reference.control_from_grid->header.frame_id,
      tf2::TimePointZero, tf2::durationFromSec(runtime_tf_lookup_timeout_sec_));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【TF失败】参考复检无法获取当前 %s -> %s：%s；撤销参考。",
      reference.control_from_grid->header.frame_id.c_str(),
      reference.trajectory.header.frame_id.c_str(), exception.what());
    return ReferenceTrajectory();
  }
  tf2::Transform committed;
  tf2::Transform current;
  tf2::fromMsg(reference.control_from_grid->transform, committed);
  tf2::fromMsg(grid_from_control.transform, current);
  // 栅格坐标下的漂移：map->odom 未变时为恒等。
  const tf2::Transform drift = current * committed;
  if (drift.getOrigin().length() < 1e-6 &&
    drift.getRotation().getAngleShortestPath() < 1e-6) {
    return reference.trajectory;
  }
  ReferenceTrajectory aligned = reference.trajectory;
  const double drift_yaw = tf2::getYaw(drift.getRotation());
  const double c = std::cos(drift_yaw);
  const double s = std::sin(drift_yaw);
  if (reference.control_reference &&
    reference.control_reference->poses.size() != aligned.points.size())
  {
    return ReferenceTrajectory();
  }
  for (std::size_t i = 0; i < aligned.points.size(); ++i) {
    auto & point = aligned.points[i];
    tf2::Vector3 moved;
    if (reference.control_reference) {
      geometry_msgs::msg::PoseStamped pose;
      tf2::doTransform(reference.control_reference->poses[i], pose, grid_from_control);
      moved = tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      point.yaw = tf2::getYaw(pose.pose.orientation);
    } else {
      // Legacy in-process fixtures have no committed cross-frame path.
      moved = drift * tf2::Vector3(point.x, point.y, 0.0);
      point.yaw = std::atan2(std::sin(point.yaw + drift_yaw), std::cos(point.yaw + drift_yaw));
    }
    point.x = moved.x();
    point.y = moved.y();
    const double vx = point.vx;
    point.vx = c * vx - s * point.vy;
    point.vy = s * vx + c * point.vy;
    const double ax = point.ax;
    point.ax = c * ax - s * point.ay;
    point.ay = s * ax + c * point.ay;
  }
  return aligned;
}

bool MincoPlannerNode::runtimeWindowSafeOn(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid)
{
  if (trajectory.points.size() < 2 || trajectory.header.frame_id != grid.header.frame_id) {
    return false;
  }
  const rclcpp::Time reference_stamp(trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  const double progress = referenceProgress(trajectory, grid, elapsed);
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    trajectory, std::max(0.0, progress - reference_window_backward_sec_),
    progress + runtime_safety_horizon_sec_);
  if (remaining.points.size() < 2) {
    return false;
  }
  const FootprintSafetyResult safety = safety_checker_.check(remaining, grid);
  if (safety.safe) {
    return true;
  }
  const EscapePrefixDecision escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  return escapePrefixContactAdmitted(
    remaining, escape, footprint_params_, grid, escape_prefix_params_);
}

double MincoPlannerNode::referenceProgress(
  const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
  double elapsed)
{
  if (!reference_window_from_robot_progress_ ||
    trajectory.header.frame_id != grid.header.frame_id)
  {
    return elapsed;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!lookupStartPose(grid, pose)) {
    return elapsed;
  }
  // 车沿参考的实际推进（不超过墙钟）。MPC 按几何投影跟踪，拐角减速后车落后于墙钟；
  // 墙钟窗口会跳过车前方还没走的那段参考（漏检），又把车身后很远的冲突算进"前视窗"。
  // 保留检查与运行期复检都以它为锚点，窗口语义是"车前方 horizon 秒的参考"。
  return std::min(elapsed, projectedReferenceTime(
    trajectory, pose.pose.position.x, pose.pose.position.y, elapsed));
}

void MincoPlannerNode::onRuntimeSafetyRecheck()
{
  std::optional<ActiveSafetyReference> active_reference;
  std::shared_ptr<const PlanningMapSnapshot> snapshot;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ || !latest_map_snapshot_ ||
      !safety_state_.mapSnapshotUsable(latest_map_snapshot_->generation))
    {
      return;
    }
    active_reference = active_safety_reference_;
    snapshot = latest_map_snapshot_;
  }

  if (active_reference->trajectory.header.frame_id != snapshot->grid.header.frame_id ||
    active_reference->trajectory.points.size() < 2)
  {
    return;
  }
  const rclcpp::Time reference_stamp(active_reference->trajectory.header.stamp);
  const double elapsed = std::max(0.0, (now() - reference_stamp).seconds());
  const ReferenceTrajectory aligned = alignedToCurrentTf(*active_reference);
  if (aligned.points.size() < 2) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ ||
      active_safety_reference_->goal_id != active_reference->goal_id ||
      active_safety_reference_->localization_epoch != active_reference->localization_epoch ||
      active_safety_reference_->plan_request_sequence != active_reference->plan_request_sequence ||
      active_safety_reference_->map_generation != active_reference->map_generation ||
      active_safety_reference_->trajectory.header.stamp != active_reference->trajectory.header.stamp)
    {
      return;
    }
    active_safety_reference_.reset();
    safety_state_.plan_safe = false;
    publishEmergencyStop(true);
    publishPlannerStatusLocked(
      active_reference->goal_id, active_reference->localization_epoch,
      active_reference->plan_request_sequence, snapshot->generation,
      active_reference->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REFERENCE_TF);
    return;
  }
  const double progress = referenceProgress(aligned, snapshot->grid, elapsed);
  const double horizon_end = progress + runtime_safety_horizon_sec_;
  const ReferenceTrajectory remaining = remainingReferenceWindow(
    aligned, std::max(0.0, progress - reference_window_backward_sec_), horizon_end);
  if (remaining.points.size() < 2) {
    return;
  }

  const FootprintSafetyResult safety = safety_checker_.check(remaining, snapshot->grid);
  if (safety.safe) {
    return;
  }
  // remaining.points[0] 是车当前跟踪到的位姿，所以这道运行期门和提交门面对同一个边界：
  // 因"车现在所在的位姿被占据"而撤销轨迹，会立刻把正在执行的逃逸动作掐掉，车留在接触
  // 里不动。判据与提交门同源——只有冲突紧贴 remaining 起点、且有界前缀之后不再有冲突时
  // 才放行；车一旦沿轨迹驶离，head_offset 增大，这个放行自然停止生效。
  const EscapePrefixDecision runtime_escape =
    evaluateEscapePrefix(remaining.points, safety.collisions, escape_prefix_params_);
  if (escapePrefixContactAdmitted(
      remaining, runtime_escape, footprint_params_, snapshot->grid, escape_prefix_params_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "【接触脱困】运行期扫掠门禁下保留脱困参考：冲突=%zu 起始偏移=%.3f m 前缀末端=%zu 长度=%.3f m。",
      runtime_escape.collision_count, runtime_escape.head_offset_m,
      runtime_escape.prefix_end, runtime_escape.prefix_length_m);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!active_safety_reference_ ||
      active_safety_reference_->goal_id != active_reference->goal_id ||
      active_safety_reference_->localization_epoch != active_reference->localization_epoch ||
      active_safety_reference_->plan_request_sequence != active_reference->plan_request_sequence ||
      active_safety_reference_->map_generation != active_reference->map_generation ||
      active_safety_reference_->trajectory.header.stamp != active_reference->trajectory.header.stamp)
    {
      return;
    }
    active_safety_reference_.reset();
    safety_state_.plan_safe = false;
    publishEmergencyStop(true);
    publishPlannerStatusLocked(
      active_reference->goal_id, active_reference->localization_epoch,
      active_reference->plan_request_sequence, snapshot->generation,
      active_reference->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_RUNTIME_UNSAFE);
  }
  const bool has_collision = !safety.collisions.empty();
  const std::size_t collision_index = has_collision ?
    std::min(safety.collisions.front().trajectory_index, remaining.points.size() - 1U) : 0U;
  const ReferencePoint & collision_reference = remaining.points[collision_index];
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), log_throttle_ms_,
    "【运行期安全拒绝】扫掠足迹拒绝（goal=%llu candidate_generation=%llu "
    "current_generation=%llu collisions=%zu discrete_samples=%zu swept_samples=%zu "
    "first_index=%zu first_swept=%d first_collision=(%.3f,%.3f) "
    "remaining_start=(%.3f,%.3f,%.3f) reference_center=(%.3f,%.3f,%.3f)）。",
    static_cast<unsigned long long>(active_reference->goal_id),
    static_cast<unsigned long long>(active_reference->map_generation),
    static_cast<unsigned long long>(snapshot->generation), safety.collisions.size(),
    safety.discrete_samples_checked, safety.swept_samples_checked, collision_index,
    has_collision && safety.collisions.front().swept ? 1 : 0,
    has_collision ? safety.collisions.front().x : 0.0,
    has_collision ? safety.collisions.front().y : 0.0,
    remaining.points.front().x, remaining.points.front().y, remaining.points.front().yaw,
    collision_reference.x, collision_reference.y, collision_reference.yaw);
}

void MincoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // P2/P3 过渡兼容入口：P2 保持直接 PoseStamped，不携带任务生命周期编号。
  planGoal(*msg, 0, 0, 0, 0, false);
}

void MincoPlannerNode::onPlannerGoal(
  const ats_navigation_interfaces::msg::PlannerGoal::SharedPtr msg)
{
  if (msg->goal_id == 0 || msg->plan_request_sequence == 0 ||
    !std::isfinite(msg->position_tolerance) || msg->position_tolerance < 0.0) {
    publishPlannerStatus(
        msg->goal_id, msg->localization_epoch, msg->plan_request_sequence, 0,
        msg->map_publication_sequence,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_INVALID_GOAL);
    return;
  }
  InitialKinematicState frozen_seed;
  const geometry_msgs::msg::PoseStamped * frozen_start = nullptr;
  if (!msg->start_pose.header.frame_id.empty()) {
    if (!finitePose(msg->start_pose.pose) ||
      !std::isfinite(msg->start_twist.linear.x) ||
      !std::isfinite(msg->start_twist.linear.y) ||
      !std::isfinite(msg->start_twist.linear.z) ||
      !std::isfinite(msg->start_twist.angular.x) ||
      !std::isfinite(msg->start_twist.angular.y) ||
      !std::isfinite(msg->start_twist.angular.z))
    {
      publishPlannerStatus(
          msg->goal_id, msg->localization_epoch, msg->plan_request_sequence, 0,
          msg->map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED,
          ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER);
      return;
    }
    frozen_start = &msg->start_pose;
    frozen_seed.valid = true;
    frozen_seed.velocity = Eigen::Vector2d(
      msg->start_twist.linear.x, msg->start_twist.linear.y);
  }
  publishPlannerStatus(
      msg->goal_id, msg->localization_epoch, msg->plan_request_sequence, 0,
      msg->map_publication_sequence,
      ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED,
      ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE);
  planGoal(
    msg->goal_pose, msg->goal_id, msg->localization_epoch,
    msg->plan_request_sequence,
    msg->map_publication_sequence, true, frozen_start,
    frozen_seed.valid ? &frozen_seed : nullptr, msg->position_tolerance, msg->goal_uuid,
    msg->preserve_active_reference);
}

void MincoPlannerNode::planGoal(
    const geometry_msgs::msg::PoseStamped &input_goal, std::uint64_t goal_id,
    std::uint64_t localization_epoch, std::uint64_t plan_request_sequence,
    std::uint64_t map_publication_sequence,
    bool report_status,
    const geometry_msgs::msg::PoseStamped * frozen_start,
    const InitialKinematicState * initial_state, double position_tolerance,
    const GoalUuid & goal_uuid, bool preserve_active_reference) {
  const auto plan_wall_begin = std::chrono::steady_clock::now();
  const rclcpp::Time plan_begin_time = now();
  bool keep_active_reference = false;
  // 无停车重规划时拷一份正在执行的已提交参考，用来取新参考的起点（附录 A.3）。
  std::optional<ActiveSafetyReference> committed_for_start;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    // 由【建议重规划】或目标管理器连续终点修正（preserve_active_reference）触发的同目标
    // 非停车替换请求：旧参考仍在执行，保留它的运行期复检守护与 plan_safe，直到新参考提交
    // （publishReferenceIfCurrent 替换）或本次规划以 STATE_ACCEPTED 报告失败。
    const bool advised_match = replan_advised_reference_stamp_ &&
      active_safety_reference_ &&
      active_safety_reference_->trajectory.header.stamp == *replan_advised_reference_stamp_;
    keep_active_reference = active_safety_reference_ &&
      safety_state_.plan_safe && active_safety_reference_->goal_id == goal_id &&
      active_safety_reference_->localization_epoch == localization_epoch &&
      active_safety_reference_->plan_request_sequence < plan_request_sequence &&
      (preserve_active_reference || advised_match);
    if (keep_active_reference) {
      committed_for_start = active_safety_reference_;
    } else {
      active_safety_reference_.reset();
    }
  }
  if (!keep_active_reference) {
    setPlanSafe(false);
  }
  const auto fail = [this, goal_id, localization_epoch, plan_request_sequence,
                     map_publication_sequence, report_status, keep_active_reference](
                        std::uint8_t failure_reason, std::uint64_t generation) {
    if (keep_active_reference) {
      // 无停车重规划失败：旧参考仍在执行且受 10 Hz 运行期复检守护，不撤销它。失败以
      // STATE_ACCEPTED 发布（MPC/arbiter 忽略，不停车），GoalManager 清除替换标记；下一次
      // 地图变化若仍远端冲突会再次建议重规划。旧参考近端失效时运行期复检照常急停。
      bool still_guarded = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        still_guarded = active_safety_reference_.has_value() && safety_state_.plan_safe;
        replan_advised_reference_stamp_.reset();
        if (still_guarded && planner_status_pub_ && report_status) {
          ats_navigation_interfaces::msg::PlannerStatus status;
          status.header.stamp = now();
          status.header.frame_id = global_frame_;
          status.goal_id = goal_id;
          status.localization_epoch = localization_epoch;
          status.plan_request_sequence = plan_request_sequence;
          status.map_generation = generation != 0 ? generation : next_map_generation_;
          status.map_publication_sequence = map_publication_sequence;
          status.state = ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED;
          status.failure_reason = failure_reason;
          planner_status_pub_->publish(status);
        }
      }
      if (still_guarded) {
        RCLCPP_WARN(
          get_logger(),
          "【无停车重规划失败】目标=%llu 请求=%llu 原因=%u，继续执行受运行期复检守护的旧参考。",
          static_cast<unsigned long long>(goal_id),
          static_cast<unsigned long long>(plan_request_sequence),
          static_cast<unsigned>(failure_reason));
        return;
      }
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      active_safety_reference_.reset();
    }
    setPlanSafe(false);
    if (report_status) {
      publishPlannerStatus(
          goal_id, localization_epoch, plan_request_sequence, generation,
          map_publication_sequence,
          ats_navigation_interfaces::msg::PlannerStatus::STATE_FAILED, failure_reason);
    }
  };
  std::shared_ptr<const PlanningMapSnapshot> map_snapshot;
  std::uint64_t map_health_epoch = 0;
  bool map_ready = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_snapshot = latest_map_snapshot_;
    map_health_epoch = map_health_epoch_;
    map_ready = map_snapshot && safety_state_.mapSnapshotUsable(map_snapshot->generation);
  }
  if (!map_ready || !map_snapshot ||
    (localization_epoch != 0 && map_snapshot->localization_epoch != 0 &&
    map_snapshot->localization_epoch != localization_epoch))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【等待地图】尚未收到可通行栅格，规划保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY, 0);
    return;
  }
  const auto & planning_grid = map_snapshot->grid;
  const auto & clearance_esdf = map_snapshot->clearance_esdf;
  InitialKinematicState frozen_seed;
  const InitialKinematicState * seed_ptr = nullptr;
  if (initial_state && initial_state->valid) {
    frozen_seed = *initial_state;
    seed_ptr = &frozen_seed;
  }

  geometry_msgs::msg::PoseStamped start;
  if (!resolveStartPose(planning_grid, frozen_start, start)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】无法获取起点位姿，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_START_TF,
      map_snapshot->generation);
    return;
  }

  // 无停车重规划的起点（调研报告附录 A.3）。请求里冻结的实测位姿在规划耗时内已落到车后面
  // （domain 111 目标 69965bd0 第二条参考的起点落后车 0.26 m，车速 0.96 m/s），所以新参考
  // 改从旧参考上"车的推进位置 + plan_lead"处取 p/v/a/yaw。车离旧参考 >= deviation_max、
  // 定位 epoch 不同、旧参考已不在执行（含急停撤销）时仍用实测，行为与此前一致。
  ReplanStart replan_start;
  bool replan_frame_matches = true;
  double replan_elapsed_sec = 0.0;
  const double measured_start_x = start.pose.position.x;
  const double measured_start_y = start.pose.position.y;
  if (committed_for_start) {
    MeasuredStart measured;
    measured.x = measured_start_x;
    measured.y = measured_start_y;
    measured.yaw = tf2::getYaw(start.pose.orientation);
    if (seed_ptr) {
      measured.vx = seed_ptr->velocity.x();
      measured.vy = seed_ptr->velocity.y();
    }
    // 旧参考按当前 map->odom 修正后再和实测比较，两者才在同一栅格坐标系里。
    const ReferenceTrajectory aligned = alignedToCurrentTf(*committed_for_start);
    replan_frame_matches = aligned.header.frame_id == planning_grid.header.frame_id;
    replan_elapsed_sec = std::max(0.0, (now() - rclcpp::Time(aligned.header.stamp)).seconds());
    replan_start = selectReplanStart(
      replan_frame_matches ? &aligned : nullptr, committed_for_start->localization_epoch,
      localization_epoch, replan_elapsed_sec, measured, replan_start_params_);
    if (replan_start.source == ReplanStartSource::kCommitted) {
      start.pose.position.x = replan_start.x;
      start.pose.position.y = replan_start.y;
      start.pose.orientation.x = 0.0;
      start.pose.orientation.y = 0.0;
      start.pose.orientation.z = std::sin(replan_start.yaw * 0.5);
      start.pose.orientation.w = std::cos(replan_start.yaw * 0.5);
      frozen_seed.valid = true;
      frozen_seed.velocity = Eigen::Vector2d(replan_start.vx, replan_start.vy);
      frozen_seed.acceleration = Eigen::Vector2d(replan_start.ax, replan_start.ay);
      seed_ptr = &frozen_seed;
    }
  }

  geometry_msgs::msg::PoseStamped goal = input_goal;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id =
      planning_grid.header.frame_id.empty() ? global_frame_ : planning_grid.header.frame_id;
  }
  geometry_msgs::msg::PoseStamped goal_in_grid;
  if (!transformGoalToGrid(planning_grid, goal, goal_in_grid)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【规划失败】目标位姿转换失败，等待 TF 恢复。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_GOAL_TF,
      map_snapshot->generation);
    return;
  }
  goal = goal_in_grid;

  // 目标位姿准入。action 的成功判据是一个容差域，而不是一个点：目标点足迹与栅格墙面
  // 重叠几毫米时，容差域内往往仍有大量完全可行的位姿。此前规划器只认那个点，于是提交门
  // 正确拒绝每一条轨迹，goal manager 反复重试直到预算耗尽（domain 175 目标 8 实测：
  // 车已停在目标 0.16 m 处，只差一个可行的终点标注）。这里在容差域内挑一个可行终点，
  // 候选过的是与提交门完全相同的足迹判定，偏移严格小于容差，所以 SUCCEEDED 依然由原目标
  // 的判据给出，不是靠放宽判据换来的。找不到可行位姿时保持原目标不变，走既有失败路径。
  // 终点净空放宽档只有在目标位姿确实过了矩形足迹门禁时才允许启用;准入关闭时
  // 保持 false,行为与今天完全一致。
  bool goal_pose_footprint_verified = false;
  auto admission_params = goal_pose_admission_params_;
  if (position_tolerance > 0.0) {
    // Per-request copy: a waypoint override must not alter later/final goals.
    admission_params.position_tolerance_m = position_tolerance;
  }
  if (admission_params.enabled) {
    const double commanded_goal_yaw = tf2::getYaw(goal.pose.orientation);
    const GoalPoseAdmissionResult admission = admitGoalPose(
      goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
      planning_grid, footprint_params_, admission_params);
    if (!admission.feasible) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "【目标准入】成功容差内没有可行位姿：目标=(%.3f, %.3f, yaw=%.3f) "
        "候选数=%zu；保留原目标。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.candidates_checked);
    }
    goal_pose_footprint_verified = admission.feasible;
    if (admission.feasible && admission.relocated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【目标准入】在成功容差内找到可行位姿：原目标=(%.3f, %.3f, yaw=%.3f) "
        "采用=(%.3f, %.3f, yaw=%.3f) 位置偏差=%.3f m 航向偏差=%.3f rad "
        "额外净空=%d 候选数=%zu。",
        goal.pose.position.x, goal.pose.position.y, commanded_goal_yaw,
        admission.x, admission.y, admission.yaw, admission.position_deviation_m,
        admission.yaw_deviation_rad, admission.used_preferred_margin ? 1 : 0,
        admission.candidates_checked);
      goal.pose.position.x = admission.x;
      goal.pose.position.y = admission.y;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = std::sin(admission.yaw * 0.5);
      goal.pose.orientation.w = std::cos(admission.yaw * 0.5);
    }
  }

  // 起点来自 TF、终点来自 goal_topic，因此此分支不依赖 Smac 的路径几何。
  double used_clearance = preferred_search_clearance_;
  const auto search_wall_begin = std::chrono::steady_clock::now();
  const GridSearchResult search_result = planRoute(
    map_snapshot, start, goal, goal_uuid, goal_id, localization_epoch,
    goal_pose_footprint_verified, used_clearance);
  const double search_wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_wall_begin).count();
  if (!search_result.success && !search_result.partial) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划失败】%s 失败（failed: %s expanded=%d clearance=%.3f m） "
      "goal=%llu localization_epoch=%llu plan_request_sequence=%llu",
      search_algorithm_.c_str(), search_result.reason.c_str(),
      search_result.expanded_nodes, used_clearance,
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(localization_epoch),
      static_cast<unsigned long long>(plan_request_sequence));
    const std::uint8_t failure_reason =
      classifyGraphSearchFailure(search_result.reason, search_result.expanded_nodes);
    fail(failure_reason, map_snapshot->generation);
    return;
  }

  // The raw JPS route, guide stages and final candidate all come from this
  // immutable snapshot.  The guide shortcut delegates collision semantics to
  // the exact oriented footprint/swept checker used by the final gate.
  // These are diagnostic-only products. They deliberately publish before the
  // candidate gate so a fail-closed rejection can still be attributed to a
  // geometry, ESDF, time-allocation, or dynamic-limit stage.
  if (raw_path_pub_ && raw_path_pub_->get_subscription_count() > 0) {
    raw_path_pub_->publish(search_result.path);
  }
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = (search_result.partial && !search_result.path.poses.empty()) ?
    tf2::getYaw(search_result.path.poses.back().pose.orientation) :
    tf2::getYaw(goal.pose.orientation);
  std::vector<TunnelSegment> guide_tunnels;
  ReferenceTrajectory traced_guide;
  if (tunnel_yaw_params_.enabled && clearance_esdf) {
    traced_guide = traceGuide(search_result.path);
    annotatePositionClearance(traced_guide, *map_snapshot);
    guide_tunnels = detectTunnelSegments(traced_guide, *clearance_esdf, tunnel_yaw_params_);
    resolveTerminalTunnelGoalAlignment(
      guide_tunnels, traced_guide, goal_yaw, !search_result.partial, tunnel_yaw_params_);
    for (const auto & tunnel : guide_tunnels) {
      if (tunnel.forward_alignment) {
        const auto & entry = traced_guide.points[tunnel.begin];
        RCLCPP_INFO(
          get_logger(), "TRACE guide_tunnel publication=%llu width_m=%.3f "
          "entry=(%.3f,%.3f) yaw=%.3f exit=(%.3f,%.3f)",
          static_cast<unsigned long long>(map_snapshot->publication_sequence),
          tunnel.width_estimate, entry.x, entry.y, tunnel.axis_yaw.front(),
          traced_guide.points[tunnel.end].x, traced_guide.points[tunnel.end].y);
      }
    }
  }

  double minco_wall_ms = 0.0;
  const auto elapsedWallMs = [&plan_wall_begin]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_wall_begin).count();
    };
  const auto rollingBudgetExceeded = [&]() {
      return keep_active_reference && rolling_replan_wall_budget_ms_ > 0.0 &&
             elapsedWallMs() >= rolling_replan_wall_budget_ms_;
    };
  const auto runOptimize = [&](
    const nav_msgs::msg::Path & guide_path,
    const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf_ptr,
    const ReferenceTrajectory * yaw_ref,
    MincoOptimizationTrace * out_trace,
    const TerminalKinematicState * tail_ptr = nullptr) {
      const auto t0 = std::chrono::steady_clock::now();
      ReferenceTrajectory out = optimizer_.optimize(
        guide_path, esdf_ptr, yaw_ref, seed_ptr, &planning_grid, &safety_checker_,
        out_trace, tail_ptr, map_snapshot->blocked_squared_cells.get());
      minco_wall_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return out;
    };
  const auto minTrajectoryClearance = [](const ReferenceTrajectory & traj) {
      double min_c = std::numeric_limits<double>::infinity();
      for (const auto & pt : traj.points) {
        if (std::isfinite(pt.clearance)) {
          min_c = std::min(min_c, pt.clearance);
        }
      }
      return std::isfinite(min_c) ? min_c : -std::numeric_limits<double>::infinity();
    };

  bool has_spliced_tail = false;
  bool used_old_trajectory_splice = false;
  nav_msgs::msg::Path active_guide_path = search_result.path;
  double active_goal_yaw = goal_yaw;
  bool active_is_final_goal = !search_result.partial;
  MincoOptimizationTrace selected_trace;
  ReferenceTrajectory center_reference;

  // O2 真局部滚动地平线 + C2 复用拼接：
  // 当路径总长 > rolling_horizon_length_m + rolling_min_tail_length_m 时：
  //   (A) 若已提交安全参考直达同一终点且后段仍安全，优先 C2 拼接至旧参考后段；
  //   (B) 否则在隧道外且足迹可停的滚动地平线处截断为局部窗口（约 8~12 m，零速确定性停车），
  //       仅对局部窗口跑完整 ESDF/足迹联合优化（~8-12 段，< 15 ms），同时保留全局引导供绕障
  //       检测，由 GoalManager 在进入制动尾段前无停车滚动刷新下一窗口。
  const double total_guide_length = pathArcLength(search_result.path);
  if (rolling_horizon_enabled_ &&
    total_guide_length > rolling_horizon_length_m_ + rolling_min_tail_length_m_)
  {
    auto avoidTunnels = [&](double split_s) {
        for (const auto & tunnel : guide_tunnels) {
          if (tunnel.end < traced_guide.points.size()) {
            const double entry_s = traced_guide.points[tunnel.begin].s - 1.5;
            const double exit_s = traced_guide.points[tunnel.end].s + 1.5;
            if (split_s >= entry_s && split_s <= exit_s) {
              if (exit_s + rolling_min_tail_length_m_ < total_guide_length &&
                exit_s <= rolling_horizon_length_m_ + 4.0)
              {
                split_s = exit_s;
              } else if (entry_s >= 4.0) {
                split_s = entry_s;
              } else if (exit_s + rolling_min_tail_length_m_ < total_guide_length) {
                split_s = exit_s;
              }
            }
          }
        }
        return split_s;
      };
    const double base_split_s = avoidTunnels(rolling_horizon_length_m_);
    SplitGuideResult split;
    const double offsets[] = {0.0, -0.5, 0.5, -1.0, 1.0, -1.5, 1.5};
    for (const double delta : offsets) {
      const double try_s = avoidTunnels(base_split_s + delta);
      if (try_s < 3.5 || try_s + rolling_min_tail_length_m_ >= total_guide_length) {
        continue;
      }
      SplitGuideResult cand = splitPathAtArcLength(
        search_result.path, try_s, rolling_min_tail_length_m_);
      if (!cand.valid) {
        continue;
      }
      const double cand_yaw = std::atan2(cand.junction_tangent.y(), cand.junction_tangent.x());
      if (goalPoseFootprintFree(
          cand.junction_pos.x(), cand.junction_pos.y(), cand_yaw,
          planning_grid, footprint_params_))
      {
        split = std::move(cand);
        break;
      }
    }
    if (split.valid) {
      // (A) 优先尝试直接 C2 拼接到仍在执行且直达最终目标的安全旧参考轨迹。
      if (committed_for_start && replan_frame_matches &&
        committed_for_start->trajectory.valid() &&
        !search_result.path.poses.empty())
      {
        const auto & old_traj = committed_for_start->trajectory;
        const auto & goal_pt = search_result.path.poses.back().pose.position;
        const double goal_match_dist = std::hypot(
          old_traj.points.back().x - goal_pt.x, old_traj.points.back().y - goal_pt.y);
        const double t_rejoin = projectedReferenceTime(
          old_traj, split.junction_pos.x(), split.junction_pos.y(), old_traj.totalTime());
        const double min_rejoin_t = std::isfinite(replan_start.t_ref) ?
          replan_start.t_ref + 0.35 : 0.35;
        if (goal_match_dist <= 0.05 && t_rejoin > min_rejoin_t &&
          t_rejoin + 0.35 < old_traj.totalTime())
        {
          const ReferencePoint ahead_j = interpolateReferencePoint(old_traj, t_rejoin);
          if (std::hypot(ahead_j.x - split.junction_pos.x(), ahead_j.y - split.junction_pos.y()) <=
            0.15)
          {
            const ReferenceTrajectory old_tail = remainingReferenceWindow(
              old_traj, t_rejoin, std::numeric_limits<double>::infinity());
            if (old_tail.points.size() >= 2U &&
              safety_checker_.check(old_tail, planning_grid, clearance_esdf.get()).safe)
            {
              nav_msgs::msg::Path exact_prefix = split.prefix;
              exact_prefix.poses.back().pose.position.x = ahead_j.x;
              exact_prefix.poses.back().pose.position.y = ahead_j.y;
              TerminalKinematicState exact_tail;
              exact_tail.valid = true;
              exact_tail.velocity = Eigen::Vector2d(ahead_j.vx, ahead_j.vy);
              exact_tail.acceleration = Eigen::Vector2d(ahead_j.ax, ahead_j.ay);
              exact_tail.exact_splice = true;
              MincoOptimizationTrace splice_trace;
              ReferenceTrajectory local_prefix = runOptimize(
                exact_prefix, clearance_esdf.get(), nullptr, &splice_trace, &exact_tail);
              if (local_prefix.valid() && std::abs(splice_trace.seed_scale - 1.0) <= 1e-6) {
                ReferenceTrajectory spliced = spliceReferenceTrajectories(
                  local_prefix, old_tail, local_prefix.totalTime() - t_rejoin);
                if (spliced.valid()) {
                  spliced.header.stamp = now();
                  planYaw(spliced, *map_snapshot, start_yaw, goal_yaw, !search_result.partial);
                  if (safety_checker_.check(spliced, planning_grid, clearance_esdf.get()).safe) {
                    center_reference = std::move(spliced);
                    selected_trace = std::move(splice_trace);
                    has_spliced_tail = true;
                    used_old_trajectory_splice = true;
                  }
                }
              }
            }
          }
        }
      }
      // (B) 真局部滚动窗口：仅对近端局部窗口跑完整 ESDF/足迹优化（末端安全零速停车），
      // 保留完整全局引导用于远端绕障，由 GoalManager 在进入制动区前无停车滚动推进。
      if (!center_reference.valid()) {
        active_guide_path = std::move(split.prefix);
        active_goal_yaw = std::atan2(split.junction_tangent.y(), split.junction_tangent.x());
        active_is_final_goal = false;
        has_spliced_tail = true;
      }
    }
  }

  FootprintSafetyResult center_safety;
  if (center_reference.valid()) {
    center_safety.safe = true;
  } else {
    center_reference = runOptimize(
      active_guide_path, clearance_esdf.get(), nullptr, &selected_trace, nullptr);
    if (!center_reference.valid() && has_spliced_tail && !used_old_trajectory_splice) {
      // 局部窗口求解失败时回退至整条路径一次，保证不因局部裁剪引入额外失败。
      has_spliced_tail = false;
      active_guide_path = search_result.path;
      active_goal_yaw = goal_yaw;
      active_is_final_goal = !search_result.partial;
      center_reference = runOptimize(
        active_guide_path, clearance_esdf.get(), nullptr, &selected_trace, nullptr);
    }
  }
  if (preprocessed_guide_pub_ && preprocessed_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.preprocessed_guide.poses.empty())
  {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && esdf_refined_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.esdf_refined_guide.poses.empty())
  {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!center_reference.valid()) {
    std::ostringstream durations;
    durations.setf(std::ios::fixed);
    durations.precision(3);
    for (std::size_t index = 0; index < selected_trace.segment_durations.size(); ++index) {
      if (index > 0U) {
        durations << ',';
      }
      durations << selected_trace.segment_durations[index];
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】候选被拒绝（candidate rejected generation=%llu snapshot_publication=%llu stage=%s "
      "raw_points=%zu preprocessed_points=%zu esdf_refined_points=%zu peak_v=%.3f "
      "peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f joint_termination=%s joint_iterations=%d "
      "joint_wall_ms=%.3f seed_scale=%.2f segment_durations=[%s]）",
      static_cast<unsigned long long>(map_snapshot->generation),
      static_cast<unsigned long long>(map_publication_sequence),
      selected_trace.failure_reason.empty() ? "unknown" : selected_trace.failure_reason.c_str(),
      search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
      selected_trace.esdf_refined_guide.poses.size(), selected_trace.peak_velocity,
      selected_trace.peak_acceleration, selected_trace.peak_jerk,
      selected_trace.solver_wall_time_ms,
      selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
      selected_trace.joint_iterations, selected_trace.joint_wall_time_ms,
      selected_trace.seed_scale, durations.str().c_str());
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  if (!used_old_trajectory_splice) {
    center_reference.header.stamp = now();
    planYaw(center_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal);
    center_safety = safety_checker_.check(
      center_reference, planning_grid, clearance_esdf.get());
  }
  ReferenceTrajectory reference = center_reference;
  FootprintSafetyResult safety = center_safety;
  // 足迹罚项用的是上一条轨迹的 yaw，而 planYaw 会按新轨迹重算 yaw：两者不一致时，
  // 角点可落进薄墙尖（优化器看不到、门禁拒绝）。用刚算出的 yaw 作参考再解，
  // 直到门禁通过或轮数用尽；每轮结果都过同一矩形门禁，不放宽任何判据，
  // 碰撞数变多即停并保留上一轮。
  const auto refineFootprintYaw = [&](
    const nav_msgs::msg::Path & guide, ReferenceTrajectory & candidate,
    FootprintSafetyResult & candidate_safety, MincoOptimizationTrace & candidate_trace) {
      for (int round = 0; !candidate_safety.safe && round < footprint_yaw_refinement_rounds_ &&
        !rollingBudgetExceeded();
        ++round)
      {
        MincoOptimizationTrace refined_trace;
        ReferenceTrajectory refined = runOptimize(
          guide, clearance_esdf.get(), &candidate, &refined_trace, nullptr);
        if (!refined.valid()) {
          return;
        }
        refined.header.stamp = now();
        planYaw(refined, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal);
        FootprintSafetyResult refined_safety = safety_checker_.check(
          refined, planning_grid, clearance_esdf.get());
        if (refined_safety.collisions.size() > candidate_safety.collisions.size()) {
          return;
        }
        candidate = std::move(refined);
        candidate_safety = std::move(refined_safety);
        candidate_trace = std::move(refined_trace);
      }
    };
  const double center_min_clearance = minTrajectoryClearance(center_reference);
  const bool need_footprint_refinement = !center_safety.safe ||
    center_min_clearance < footprint_refinement_skip_clearance_m_;
  if (optimizer_.esdfFootprintOptimizationEnabled() && need_footprint_refinement &&
    !used_old_trajectory_splice && !rollingBudgetExceeded())
  {
    MincoOptimizationTrace footprint_trace;
    ReferenceTrajectory footprint_reference = runOptimize(
      active_guide_path, clearance_esdf.get(), &center_reference, &footprint_trace, nullptr);
    if (footprint_reference.valid()) {
      footprint_reference.header.stamp = now();
      planYaw(
        footprint_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal);
      FootprintSafetyResult footprint_safety = safety_checker_.check(
        footprint_reference, planning_grid, clearance_esdf.get());
      refineFootprintYaw(
        active_guide_path, footprint_reference, footprint_safety, footprint_trace);
      if (footprint_safety.safe || !center_safety.safe) {
        reference = std::move(footprint_reference);
        safety = std::move(footprint_safety);
        selected_trace = std::move(footprint_trace);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【足迹候选回退】RC-ESDF 足迹候选有 %zu 个冲突，改用安全的中心 ESDF 候选。",
          footprint_safety.collisions.size());
      }
    }
  }
  bool esdf_fallback_used = false;
  if (!safety.safe && optimizer_.esdfObstacleOptimizationEnabled() && !rollingBudgetExceeded()) {
    // 外推候选仍碰撞时回到不做 ESDF 位移的 JPS-MINCO，避免“修正越修越差”。
    MincoOptimizationTrace fallback_trace;
    ReferenceTrajectory fallback_reference = runOptimize(
      active_guide_path, nullptr, nullptr, &fallback_trace, nullptr);
    if (fallback_reference.valid()) {
      fallback_reference.header.stamp = now();
      planYaw(fallback_reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal);
      const FootprintSafetyResult fallback_safety = safety_checker_.check(
        fallback_reference, planning_grid, clearance_esdf.get());
      if (fallback_safety.safe) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【足迹候选回退】RC-ESDF 外层候选有 %zu 个足迹冲突，改用安全的 JPS-MINCO 基线。",
          safety.collisions.size());
        esdf_fallback_used = true;
        reference = std::move(fallback_reference);
        safety = fallback_safety;
        selected_trace = std::move(fallback_trace);
      }
    }
  }
  LocalCollisionRepairStats repair_stats;
  // repair() 就地改引导点,所以要先留一份修复前的轨迹:重解 MINCO 失败时
  // 必须能退回原状。以前这条路走不到(严格档恒为空集),现在会真的走到,
  // 而 FAILURE_REPAIR 在 goal manager 里不是瞬时故障,会立刻判死目标。
  // 保证"尝试修复"永远不比"不尝试"更差。
  ReferenceTrajectory pre_repair_reference;
  if (!safety.safe) {
    pre_repair_reference = reference;
  }
  const bool repair_changed = !safety.safe &&
    collision_repair_.repair(reference, safety, planning_grid, &repair_stats);
  if (!safety.safe) {
    // repair() 自身没有 logger。没有这一行时无法区分"没尝试""挑不出候选格"
    // 和"挑出来但位移可忽略",domain 169 的 89 次连续拒绝就完全没有痕迹。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【局部碰撞修复】changed=%d points=%zu strict=%zu fallback=%zu "
      "no_candidate=%zu negligible=%zu endpoint_protected=%zu strict_clearance=%.3f "
      "fallback_clearance=%.3f search_radius=%.3f m.",
      repair_changed ? 1 : 0, repair_stats.collision_points,
      repair_stats.strict_repaired, repair_stats.fallback_repaired,
      repair_stats.no_candidate, repair_stats.negligible_shift,
      repair_stats.endpoint_protected,
      repair_stats.strict_clearance_m, repair_stats.fallback_clearance_m,
      repair_stats.effective_search_radius_m);
  }
  bool local_repair_used = false;
  if (repair_changed) {
    // 局部修复只改变几何引导线，必须重新求 MINCO、yaw 和最终矩形足迹安全性。
    MincoOptimizationTrace repair_trace;
    ReferenceTrajectory repaired = runOptimize(
      toPath(reference), clearance_esdf.get(), &reference, &repair_trace, nullptr);
    if (!repaired.valid()) {
      // 退回修复前轨迹,交给下面既有的 footprint 拒绝路径。那条路径报
      // FAILURE_FOOTPRINT(瞬时,等下一次 snapshot 重规划),而 FAILURE_REPAIR
      // 会让目标直接失败——修复只是一次尝试,失败不该比没尝试更严重。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【局部碰撞修复失败】修复后 MINCO 轨迹无效，保留修复前轨迹并进入既有足迹拒绝路径。");
      reference = std::move(pre_repair_reference);
    } else {
      local_repair_used = true;
      const nav_msgs::msg::Path repair_guide = toPath(reference);
      reference = std::move(repaired);
      reference.header.stamp = now();
      planYaw(reference, *map_snapshot, start_yaw, active_goal_yaw, active_is_final_goal);
      safety = safety_checker_.check(reference, planning_grid, clearance_esdf.get());
      refineFootprintYaw(repair_guide, reference, safety, repair_trace);
      selected_trace = std::move(repair_trace);
    }
  }

  // 终端原地转向重定位与全向侧移掉头：
  // 1) 当冲突全部落在终端近域窗口（含起点即终点的纯原地转向）时，先尝试沿路径提前转向；
  // 2) 当沿路径候选全被拒，或虽安全但原地大角度转向的足迹净空极薄（< 6 cm，易被厘米级
  //    轮系滑移推入墙壁）时，利用四舵轮全向能力先保持当前航向侧移到开阔 ESDF 区域掉头，
  //    再保持目标航向平移至终点。所有候选仍过同一套零冲突矩形足迹、扫掠与洞口对齐门禁。
  const double comfortable_rotation_clearance_m = terminal_rotation_comfortable_clearance_m_;
  const std::size_t relocation_tail_start = terminalCoincidentTailStart(reference);
  const bool unsafe_terminal_rotation = !safety.safe && (
    collisionsConfinedToTerminalApproach(
      reference, safety, terminal_yaw_relocation_params_.terminal_window_length) ||
    (relocation_tail_start == 0U && reference.points.size() >= 2U));
  const bool tight_terminal_rotation = active_is_final_goal && safety.safe &&
    hasTightInPlaceTerminalRotation(
      reference, terminal_rotation_tight_yaw_span_rad_, comfortable_rotation_clearance_m);
  if (terminal_yaw_relocation_enabled_ &&
    (unsafe_terminal_rotation || tight_terminal_rotation))
  {
    const auto minClearanceOf = [](const ReferenceTrajectory & traj) {
      double min_c = std::numeric_limits<double>::infinity();
      for (const auto & pt : traj.points) {
        if (std::isfinite(pt.clearance)) {
          min_c = std::min(min_c, pt.clearance);
        }
      }
      return std::isfinite(min_c) ? min_c : -std::numeric_limits<double>::infinity();
    };

    const std::size_t tail_start = relocation_tail_start;
    const std::size_t window_start = std::min(
      tail_start,
      terminalApproachWindowStart(
        reference, terminal_yaw_relocation_params_.terminal_window_length));
    const std::size_t original_collisions = safety.collisions.size();
    const double original_min_clearance = safety.safe ?
      minClearanceOf(reference) : -std::numeric_limits<double>::infinity();
    const std::vector<std::size_t> candidates = tail_start > 0U ?
      terminalYawRelocationCandidates(
        reference, static_cast<std::size_t>(terminal_yaw_relocation_max_candidates_)) :
      std::vector<std::size_t>{};
    std::ostringstream candidate_list;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (i != 0U) {
        candidate_list << ',';
      }
      candidate_list << candidates[i];
    }
    std::size_t first_conflict = 0;
    std::size_t last_conflict = 0;
    if (!safety.collisions.empty()) {
      first_conflict = safety.collisions.front().trajectory_index;
      last_conflict = first_conflict;
      for (const CollisionSample & collision : safety.collisions) {
        first_conflict = std::min(first_conflict, collision.trajectory_index);
        last_conflict = std::max(last_conflict, collision.trajectory_index);
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【终端航向重定位】尝试：tail_start=%zu window_start=%zu candidates=[%s] "
      "conflict_span=[%zu,%zu] collisions=%zu tight=%d min_clearance=%.3f.",
      tail_start, window_start, candidate_list.str().c_str(), first_conflict, last_conflict,
      original_collisions, tight_terminal_rotation ? 1 : 0,
      std::isfinite(original_min_clearance) ? original_min_clearance : -1.0);
    std::size_t built = 0;
    std::ostringstream rejected;
    std::optional<ReferenceTrajectory> best_candidate;
    FootprintSafetyResult best_safety;
    double best_clearance = original_min_clearance;
    std::size_t best_rotation_index = 0U;
    bool best_is_sidestep = false;
    double best_turn_x = 0.0;
    double best_turn_y = 0.0;

    auto evaluateCandidate = [&](ReferenceTrajectory & candidate, const std::string & label,
      FootprintSafetyResult & out_safety, double & out_clearance) -> bool {
      candidate.header.stamp = now();
      applyWheelSpeedTimeScaling(candidate, wheel_speed_time_scaling_params_);
      // 只补净空标注,绝不调用 planYaw:planYaw 会在末点再追加一段原地转向,
      // 把刚刚移开的问题原样搬回来。
      annotateClearance(candidate, *map_snapshot);
      out_safety = safety_checker_.check(candidate, planning_grid, clearance_esdf.get());
      if (!out_safety.safe) {
        rejected << label << ":footprint;";
        return false;
      }
      if (tunnel_yaw_params_.enabled && clearance_esdf) {
        auto candidate_center = candidate;
        annotatePositionClearance(candidate_center, *map_snapshot);
        auto candidate_tunnels = detectTunnelSegments(
          candidate_center, *clearance_esdf, tunnel_yaw_params_);
        resolveTerminalTunnelGoalAlignment(
          candidate_tunnels, candidate_center, active_goal_yaw,
          active_is_final_goal, tunnel_yaw_params_);
        if (!forwardTunnelEntriesAligned(candidate, candidate_tunnels, tunnel_yaw_params_)) {
          rejected << label << ":tunnel_align;";
          return false;
        }
      }
      out_clearance = minClearanceOf(candidate);
      return true;
    };

    for (const std::size_t rotation_index : candidates) {
      ReferenceTrajectory candidate;
      if (!relocateTerminalYawRotation(
          reference, active_goal_yaw, rotation_index, terminal_yaw_relocation_params_, &candidate))
      {
        rejected << rotation_index << ":build_failed;";
        continue;
      }
      ++built;
      FootprintSafetyResult candidate_safety;
      double candidate_clearance = -std::numeric_limits<double>::infinity();
      if (!evaluateCandidate(
          candidate, std::to_string(rotation_index), candidate_safety, candidate_clearance))
      {
        continue;
      }
      if (!best_candidate || candidate_clearance > best_clearance) {
        best_candidate = std::move(candidate);
        best_safety = std::move(candidate_safety);
        best_clearance = candidate_clearance;
        best_rotation_index = rotation_index;
        best_is_sidestep = false;
      }
      if (best_clearance >= comfortable_rotation_clearance_m) {
        break;
      }
    }

    // 沿路径重定位全部失败或净空仍不足 comfortable_rotation_clearance_m 时，搜索附近开阔 ESDF 点进行全向侧移掉头。
    // 若终点位于窄通道内（如贴壁掉头或进洞反转），额外尝试在起点静止/低速附近（anchor=0）先侧移掉头再沿原安全轨迹进洞。
    if ((!best_candidate || best_clearance < comfortable_rotation_clearance_m) &&
        clearance_esdf && clearance_esdf->available() && !reference.points.empty())
    {
      std::vector<std::size_t> anchor_indices;
      if (tail_start == 0U ||
          (reference.points[tail_start].s <= 0.60 && reference.points.front().v <= 0.15))
      {
        anchor_indices.push_back(0U);
      } else {
        anchor_indices.push_back(tail_start);
        if (reference.points.front().v <= 0.25) {
          anchor_indices.push_back(0U);
        }
      }
      for (const std::size_t anchor_index : anchor_indices) {
        if (best_candidate && best_clearance >= sidestep_accept_clearance_m_) {
          break;
        }
        const ReferencePoint & pivot = reference.points[anchor_index];
        const ReferencePoint & goal_pt = reference.points.back();
        const double yaw_diff = std::abs(std::atan2(
          std::sin(active_goal_yaw - pivot.yaw), std::cos(active_goal_yaw - pivot.yaw)));
        if (yaw_diff <= 0.25 && safety.safe) {
          continue;
        }
        const double half_l = 0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_;
        const double half_w = 0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_;
        const double circumradius = std::hypot(half_l, half_w);
        const double min_esdf = circumradius + sidestep_min_esdf_margin_m_;
        const double desired_esdf = circumradius + sidestep_desired_esdf_margin_m_;

        struct SidestepSpot
        {
          double x;
          double y;
          double esdf;
          double detour;
          bool preferred;
        };
        std::vector<SidestepSpot> spots;
        const double radius_scale = sidestep_max_radius_m_ / 0.45;
        const std::array<double, 6> radii{
          0.12 * radius_scale, 0.18 * radius_scale, 0.24 * radius_scale,
          0.30 * radius_scale, 0.38 * radius_scale, sidestep_max_radius_m_};
        std::vector<Eigen::Vector2d> dirs;
        dirs.reserve(17U);
        const Eigen::Vector2d grad = clearance_esdf->getGradient(pivot.x, pivot.y);
        if (grad.allFinite() && grad.norm() > 1e-4) {
          dirs.push_back(grad.normalized());
        }
        for (int k = 0; k < 16; ++k) {
          const double angle = (2.0 * M_PI * static_cast<double>(k)) / 16.0;
          dirs.emplace_back(std::cos(angle), std::sin(angle));
        }
        for (double r : radii) {
          for (const auto & d : dirs) {
            const double tx = pivot.x + r * d.x();
            const double ty = pivot.y + r * d.y();
            const double dist = clearance_esdf->getDistance(tx, ty);
            if (!std::isfinite(dist) || dist < min_esdf) {
              continue;
            }
            bool duplicate = false;
            for (const auto & existing : spots) {
              if (std::hypot(existing.x - tx, existing.y - ty) < 0.04) {
                duplicate = true;
                break;
              }
            }
            if (duplicate) {
              continue;
            }
            const double detour =
              anchor_index == 0U
                ? (2.0 * std::hypot(tx - pivot.x, ty - pivot.y))
                : (std::hypot(tx - pivot.x, ty - pivot.y) +
                   std::hypot(goal_pt.x - tx, goal_pt.y - ty));
            spots.push_back({tx, ty, dist, detour, dist >= desired_esdf});
          }
        }
        std::sort(
          spots.begin(), spots.end(),
          [](const SidestepSpot & a, const SidestepSpot & b) {
            if (a.preferred != b.preferred) {
              return a.preferred;
            }
            if (std::abs(a.detour - b.detour) > 0.04) {
              return a.detour < b.detour;
            }
            return a.esdf > b.esdf;
          });
        const std::size_t max_sidestep_tries = std::min<std::size_t>(
          spots.size(), static_cast<std::size_t>(sidestep_max_tries_));
        for (std::size_t s_idx = 0U; s_idx < max_sidestep_tries; ++s_idx) {
          const auto & spot = spots[s_idx];
          ReferenceTrajectory candidate;
          if (!buildSidestepTerminalYawTurnaround(
              reference, active_goal_yaw, anchor_index, spot.x, spot.y,
              terminal_yaw_relocation_params_, &candidate))
          {
            rejected << "sidestep" << anchor_index << "_" << s_idx << ":build_failed;";
            continue;
          }
          ++built;
          FootprintSafetyResult candidate_safety;
          double candidate_clearance = -std::numeric_limits<double>::infinity();
          if (!evaluateCandidate(
              candidate, "sidestep" + std::to_string(anchor_index) + "_" + std::to_string(s_idx),
              candidate_safety, candidate_clearance))
          {
            continue;
          }
          if (!best_candidate || candidate_clearance > best_clearance + 0.02) {
            best_candidate = std::move(candidate);
            best_safety = std::move(candidate_safety);
            best_clearance = candidate_clearance;
            best_is_sidestep = true;
            best_turn_x = spot.x;
            best_turn_y = spot.y;
          }
          if (best_clearance >= sidestep_accept_clearance_m_) {
            break;
          }
        }
      }
    }

    const bool accept_relocated = best_candidate.has_value() &&
      (!safety.safe || best_clearance > original_min_clearance + sidestep_min_improvement_m_);
    if (accept_relocated) {
      if (best_is_sidestep) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【终端航向侧移掉头成功】turn=(%.3f,%.3f) tail_start=%zu "
          "cleared_collisions=%zu clearance=%.3f->%.3f points=%zu。",
          best_turn_x, best_turn_y, tail_start, original_collisions,
          std::isfinite(original_min_clearance) ? original_min_clearance : -1.0,
          best_clearance, best_candidate->points.size());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "【终端航向重定位成功】rotation_index=%zu tail_start=%zu "
          "window_start=%zu candidates=%zu built=%zu cleared_collisions=%zu clearance=%.3f points=%zu。",
          best_rotation_index, tail_start, window_start, candidates.size(), built,
          original_collisions, best_clearance, best_candidate->points.size());
      }
      reference = std::move(*best_candidate);
      safety = std::move(best_safety);
    } else if (!safety.safe) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【终端航向重定位失败】tail_start=%zu window_start=%zu candidates=[%s] "
        "built=%zu collisions=%zu rejected=%s；保留现有足迹拒绝路径。",
        tail_start, window_start, candidate_list.str().c_str(), built, original_collisions,
        rejected.str().c_str());
    }
  }

  if (preprocessed_guide_pub_ && preprocessed_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.preprocessed_guide.poses.empty())
  {
    preprocessed_guide_pub_->publish(selected_trace.preprocessed_guide);
  }
  if (esdf_refined_guide_pub_ && esdf_refined_guide_pub_->get_subscription_count() > 0 &&
    !selected_trace.esdf_refined_guide.poses.empty())
  {
    esdf_refined_guide_pub_->publish(selected_trace.esdf_refined_guide);
  }
  if (!reference.valid()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】参考轨迹无效，保持急停。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  EscapePrefixDecision escape_decision = safety.safe ?
    EscapePrefixDecision{} :
    evaluateEscapePrefix(reference.points, safety.collisions, escape_prefix_params_);
  if (escape_decision.allowed &&
    !escapePrefixContactAdmitted(
      reference, escape_decision, footprint_params_, planning_grid, escape_prefix_params_))
  {
    // 前缀压进占据格超过深度上界：不是擦边接触，照旧 fail-closed。
    escape_decision.prefix_end = 0U;
    escape_decision.allowed = false;
  }
  // 贴障起步时 yaw 规划在起点就转向，角点扫进障碍，逃逸前缀的深度/扫掠两道上界都拒绝它。
  // 换成"先保持起点 yaw 平移驶离再追回原 yaw"的候选，重新过同一套门禁与逃逸判据；
  // 几何路径和终端 yaw 不变。只在逃逸开关打开且冲突紧贴起点时尝试。
  if (!safety.safe && !escape_decision.allowed && escape_prefix_params_.enabled &&
    !safety.collisions.empty() &&
    std::min_element(
      safety.collisions.begin(), safety.collisions.end(),
      [](const CollisionSample & a, const CollisionSample & b) {
        return a.trajectory_index < b.trajectory_index;
      })->trajectory_index == 0U)
  {
    std::ostringstream tried;
    std::vector<double> hold_lengths {0.20, escape_prefix_params_.max_prefix_length_m};
    if (escape_prefix_params_.monotone_max_contact_depth_m > 0.0 &&
      escape_prefix_params_.monotone_max_prefix_length_m > escape_prefix_params_.max_prefix_length_m)
    {
      hold_lengths.push_back(escape_prefix_params_.monotone_max_prefix_length_m);
    }
    for (const double hold_length : hold_lengths) {
      ReferenceTrajectory candidate;
      if (!holdStartYawForEscape(
          reference, hold_length, terminal_yaw_relocation_params_.yaw_rate_limit, &candidate))
      {
        tried << hold_length << ":build_failed;";
        continue;
      }
      annotateClearance(candidate, *map_snapshot);
      const FootprintSafetyResult candidate_safety =
        safety_checker_.check(candidate, planning_grid);
      EscapePrefixDecision candidate_escape = candidate_safety.safe ?
        EscapePrefixDecision{} :
        evaluateEscapePrefix(candidate.points, candidate_safety.collisions, escape_prefix_params_);
      if (candidate_escape.allowed &&
        !escapePrefixContactAdmitted(
          candidate, candidate_escape, footprint_params_, planning_grid, escape_prefix_params_))
      {
        candidate_escape.prefix_end = 0U;
        candidate_escape.allowed = false;
      }
      if (!candidate_safety.safe && !candidate_escape.allowed) {
        tried << hold_length << ":collisions=" << candidate_safety.collisions.size() <<
          ",prefix_end=" << candidate_escape.candidate_prefix_end <<
          ",yaw_sweep=" << candidate_escape.prefix_yaw_sweep_rad << ';';
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【脱困航向保持】接受：长度=%.2f m safe=%d 冲突=%zu 前缀末端=%zu。",
        hold_length, candidate_safety.safe ? 1 : 0, candidate_safety.collisions.size(),
        candidate_escape.prefix_end);
      reference = std::move(candidate);
      safety = candidate_safety;
      escape_decision = candidate_escape;
      break;
    }
    if (!safety.safe && !escape_decision.allowed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【脱困航向保持失败】已尝试：%s", tried.str().c_str());
    }
  }
  // 局部倒退脱困：当近端（如窄通道/隧道口附近贴障起步或前向拐弯角点擦墙）存在足迹冲突且零位点尚未被完全锁死时，
  // 优先尝试保持当前航向直线倒退（v_x < 0）或沿局部 ESDF 梯度平移 0.25~0.65 m 到更高净空的局部锚点，
  // 再从局部锚点重接剩余路径，整条组合候选必须严格通过同一零冲突矩形足迹与扫掠门禁。
  if (!safety.safe && !safety.collisions.empty() && !rollingBudgetExceeded() &&
    (!seed_ptr || !seed_ptr->valid || seed_ptr->velocity.norm() <= 0.20))
  {
    const std::size_t earliest_conflict = std::min_element(
      safety.collisions.begin(), safety.collisions.end(),
      [](const CollisionSample & a, const CollisionSample & b) {
        return a.trajectory_index < b.trajectory_index;
      })->trajectory_index;
    if (earliest_conflict <= 12U && clearance_esdf && clearance_esdf->available()) {
      const double sx = start.pose.position.x;
      const double sy = start.pose.position.y;
      const double start_d = clearance_esdf->getDistance(sx, sy);
      std::vector<Eigen::Vector2d> dirs{
        Eigen::Vector2d(-std::cos(start_yaw), -std::sin(start_yaw)),
        Eigen::Vector2d(std::cos(start_yaw), std::sin(start_yaw))};
      const Eigen::Vector2d grad = clearance_esdf->getGradient(sx, sy);
      if (grad.norm() > 0.20) {
        dirs.push_back(grad.normalized());
      }
      const std::array<double, 3> backoff_dists{0.25, 0.45, 0.65};
      bool escaped = false;
      for (const auto & dir : dirs) {
        if (escaped || rollingBudgetExceeded()) {
          break;
        }
        for (const double dist : backoff_dists) {
          if (rollingBudgetExceeded()) {
            break;
          }
          const double ax = sx + dir.x() * dist;
          const double ay = sy + dir.y() * dist;
          const double ad = clearance_esdf->getDistance(ax, ay);
          if (!std::isfinite(ad) || ad <= std::max(0.32, start_d + 0.02)) {
            continue;
          }
          ReferenceTrajectory backoff;
          backoff.header = reference.header;
          const double v_cap = std::max(0.25, terminal_yaw_relocation_params_.sidestep_max_speed);
          const double a_cap = std::max(0.40, terminal_yaw_relocation_params_.sidestep_max_accel);
          const double dur = std::max({
            0.35, 1.875 * dist / v_cap, std::sqrt(5.7735026919 * dist / a_cap)});
          const double sample_dt = std::max(0.02, terminal_yaw_relocation_params_.sample_period);
          const int steps = std::max(4, static_cast<int>(std::ceil(dur / sample_dt)));
          backoff.points.reserve(static_cast<std::size_t>(steps + 1));
          for (int k = 0; k <= steps; ++k) {
            const double u = static_cast<double>(k) / static_cast<double>(steps);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double b = u3 * (10.0 - 15.0 * u + 6.0 * u2);
            const double db = (30.0 * u2 - 60.0 * u3 + 30.0 * u2 * u2) / dur;
            const double ddb = (60.0 * u - 180.0 * u2 + 120.0 * u3) / (dur * dur);
            ReferencePoint pt;
            pt.t = dur * u;
            pt.s = dist * b;
            pt.x = sx + dir.x() * dist * b;
            pt.y = sy + dir.y() * dist * b;
            pt.vx = dir.x() * dist * db;
            pt.vy = dir.y() * dist * db;
            pt.v = std::hypot(pt.vx, pt.vy);
            pt.ax = dir.x() * dist * ddb;
            pt.ay = dir.y() * dist * ddb;
            pt.yaw = start_yaw;
            pt.yaw_rate = 0.0;
            backoff.points.push_back(pt);
          }
          if (!safety_checker_.check(backoff, planning_grid, clearance_esdf.get()).safe) {
            continue;
          }
          geometry_msgs::msg::PoseStamped anchor_pose = start;
          anchor_pose.pose.position.x = ax;
          anchor_pose.pose.position.y = ay;
          double local_clearance = used_clearance;
          const GridSearchResult tail_search = runGraphSearch(
            planning_grid, anchor_pose, goal, goal_pose_footprint_verified,
            local_clearance, map_snapshot->blocked_squared_cells.get());
          if (!tail_search.success || tail_search.path.poses.size() < 2U) {
            continue;
          }
          nav_msgs::msg::Path tail_guide_path = tail_search.path;
          double tail_goal_yaw = goal_yaw;
          bool tail_is_final = !tail_search.partial;
          bool tail_spliced = false;
          if (rolling_horizon_enabled_ &&
            pathArcLength(tail_guide_path) >
              rolling_horizon_length_m_ + rolling_min_tail_length_m_)
          {
            SplitGuideResult tail_split = splitPathAtArcLength(
              tail_guide_path, rolling_horizon_length_m_, rolling_min_tail_length_m_);
            if (tail_split.valid) {
              tail_guide_path = std::move(tail_split.prefix);
              tail_goal_yaw = std::atan2(
                tail_split.junction_tangent.y(), tail_split.junction_tangent.x());
              tail_is_final = false;
              tail_spliced = true;
            }
          }
          MincoOptimizationTrace tail_trace;
          ReferenceTrajectory tail_traj = optimizer_.optimize(
            tail_guide_path, clearance_esdf.get(), nullptr, nullptr,
            &planning_grid, &safety_checker_, &tail_trace, nullptr,
            map_snapshot->blocked_squared_cells.get());
          if (!tail_traj.valid()) {
            continue;
          }
          tail_traj.header.stamp = now();
          planYaw(tail_traj, *map_snapshot, start_yaw, tail_goal_yaw, tail_is_final);
          ReferenceTrajectory combined = spliceReferenceTrajectories(
            backoff, tail_traj, backoff.totalTime());
          if (!combined.valid()) {
            continue;
          }
          annotateClearance(combined, *map_snapshot);
          const FootprintSafetyResult combined_safety =
            safety_checker_.check(combined, planning_grid, clearance_esdf.get());
          if (!combined_safety.safe) {
            continue;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "【局部倒退脱困成功】anchor=(%.3f,%.3f) dist=%.2f m clearance=%.3f->%.3f points=%zu。",
            ax, ay, dist, start_d, ad, combined.points.size());
          reference = std::move(combined);
          safety = combined_safety;
          if (tail_spliced) {
            has_spliced_tail = true;
            active_goal_yaw = tail_goal_yaw;
            active_is_final_goal = false;
          }
          selected_trace = std::move(tail_trace);
          escaped = true;
          break;
        }
      }
    }
  }
  if (!safety.safe && !publish_unsafe_trajectory_ && escape_decision.allowed) {
    // 车此刻就压在冲突区里，冲突集证明这条轨迹在有界前缀内驶出障碍并且不再驶回。
    // 继续拒绝只会让唯一能挪走车的执行器拿不到轨迹，参见 escape_prefix.hpp 的说明。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【脱困候选】MINCO 轨迹驶离接触区，仍须通过最终零冲突门：冲突=%zu 起始偏移=%.3f m "
      "prefix_end=%zu prefix_length=%.3f m prefix_yaw_sweep=%.3f rad.",
      escape_decision.collision_count, escape_decision.head_offset_m,
      escape_decision.prefix_end, escape_decision.prefix_length_m,
      escape_decision.prefix_yaw_sweep_rad);
  } else if (!safety.safe && !publish_unsafe_trajectory_) {
    if (safety.collisions.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】安全校验失败，未发布不安全轨迹。");
    } else {
      const CollisionSample & first_collision = safety.collisions.front();
      // 只报冲突数和一个 footprint 采样坐标无法区分三种完全不同的失败:车此刻就压在
      // 障碍上、轨迹先走一段再撞墙、以及轨迹整条贴着障碍蹭行。这三种的修复位置分别在
      // 逃逸放行、图搜索净空和栅格量化一致性上,所以这里把判据本身打全:冲突的离散/
      // 扫掠构成、首末冲突下标、首个冲突点的轨迹中心位姿(注意 first_collision 的坐标
      // 是矩形上的采样点,不是中心),以及逃逸判据为什么没有放行。
      std::size_t discrete_collisions = 0;
      std::size_t swept_collisions = 0;
      std::size_t last_collision_index = 0;
      for (const CollisionSample & collision : safety.collisions) {
        if (collision.swept) {
          ++swept_collisions;
        } else {
          ++discrete_collisions;
        }
        last_collision_index = std::max(last_collision_index, collision.trajectory_index);
      }
      const ReferencePoint & first_center =
        reference.points[std::min(first_collision.trajectory_index, reference.points.size() - 1U)];
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【MINCO拒绝】足迹冲突=%zu，未发布不安全轨迹 "
        "（discrete=%zu swept=%zu）first_index=%zu last_index=%zu points=%zu "
        "first_sample=(%.3f, %.3f) first_center=(%.3f, %.3f, yaw=%.3f) "
        "start=(%.3f, %.3f, yaw=%.3f) raw_points=%zu preprocessed_points=%zu "
        "esdf_refined_points=%zu escape_allowed=%d "
        "escape_head_offset=%.3f m escape_prefix_end=%zu escape_candidate_prefix_end=%zu "
        "escape_prefix_length=%.3f m escape_prefix_yaw_sweep=%.3f rad）",
        safety.collisions.size(), discrete_collisions, swept_collisions,
        first_collision.trajectory_index, last_collision_index, reference.points.size(),
        first_collision.x, first_collision.y,
        first_center.x, first_center.y, first_center.yaw,
        reference.points.front().x, reference.points.front().y, reference.points.front().yaw,
        search_result.path.poses.size(), selected_trace.preprocessed_guide.poses.size(),
        selected_trace.esdf_refined_guide.poses.size(), static_cast<int>(escape_decision.allowed),
        escape_decision.head_offset_m, escape_decision.prefix_end,
        escape_decision.candidate_prefix_end,
        escape_decision.prefix_length_m, escape_decision.prefix_yaw_sweep_rad);
      // 上面的数字只说明整条轨迹都不安全,不说明是哪
      // 一层把格子打成了障碍。静态层已离线排除:停车位
      // 姿到最近静态占据格心约 0.61 m,大于 0.4187 m 的
      // 全 yaw 半径。所以必须把实际栅格打出来,否则无法
      // 区分 ROG 投影、terrain 硬障碍与 slope 障碍。两个
      // 窗口分别覆盖轨迹起点和首个冲突点。
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】起点：%s",
        describeGridWindow(
          planning_grid, reference.points.front().x, reference.points.front().y, 0.6,
          obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      if (first_collision.trajectory_index != 0U) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_, "【拒绝栅格窗口】首个冲突点：%s",
          describeGridWindow(
            planning_grid, first_center.x, first_center.y, 0.6,
            obstacle_value_threshold_, unknown_is_obstacle_).c_str());
      }
    }
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  TrajectoryQualityMetrics quality = quality_evaluator_.evaluate(
    reference, clearance_esdf.get(), footprintSamples(planning_grid),
    selected_trace.segment_durations);
  for (const CollisionSample & collision : safety.collisions) {
    if (collision.swept) {
      ++quality.swept_collision_count;
    } else {
      ++quality.footprint_collision_count;
    }
  }
  if (!quality.finite || !quality.strictly_monotonic_time) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【MINCO拒绝】导数包含非有限值或时间不单调。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  // Nominal commits only: escape-from-contact may need a short lateral shove.
  // d21 gen173 committed length_ratio=5.496 lateral=7.424 and drove north of spawn.
  if (safety.safe && !admitsCommitGeometry(quality, commit_geometry_limits_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【MINCO拒绝】绕行轨迹提交前未通过几何约束（length_ratio=%.3f max=%.3f "
      "lateral=%.3f m max=%.3f m path_length=%.2f direct=%.2f）。",
      quality.length_ratio, commit_geometry_limits_.max_length_ratio,
      quality.max_lateral_deviation, commit_geometry_limits_.max_lateral_deviation_m,
      quality.path_length, quality.direct_length);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NO_PATH,
      map_snapshot->generation);
    return;
  }
  // 小陀螺：开启时对整条参考生成自旋 yaw（开阔段自旋，窄段前 spin_align_lookahead 起回到规划 yaw，
  // 窄段后 spin_resume_after 再自旋），重做轮速时间缩放和足迹门禁；门禁不过就沿用规划 yaw，
  // 授权按原规则判定。
  std::uint8_t yaw_authority = selectYawAuthority(
    reference, force_body_yaw_follow_, body_yaw_follow_clearance_);
  std::vector<TunnelSegment> forward_tunnels;
  if (tunnel_yaw_params_.enabled && clearance_esdf) {
    auto center = reference;
    annotatePositionClearance(center, *map_snapshot);
    forward_tunnels = detectTunnelSegments(center, *clearance_esdf, tunnel_yaw_params_);
    resolveTerminalTunnelGoalAlignment(
      forward_tunnels, center, active_goal_yaw, active_is_final_goal, tunnel_yaw_params_);
  }
  if (!forwardTunnelEntriesAligned(reference, forward_tunnels, tunnel_yaw_params_)) {
    std::size_t bad_index = 0U;
    double bad_yaw = 0.0;
    double bad_target = 0.0;
    double bad_width = 0.0;
    for (const auto & tunnel : forward_tunnels) {
      if (!tunnel.forward_alignment || tunnel.axis_yaw.empty()) {
        continue;
      }
      const double entry_s = reference.points[tunnel.begin].s - tunnel_yaw_params_.front_extent;
      std::size_t first = tunnel.begin;
      while (first > 1U && reference.points[first - 1U].s >= entry_s) {
        --first;
      }
      for (std::size_t idx = first; idx <= tunnel.end; ++idx) {
        const auto local_index = idx > tunnel.begin ? idx - tunnel.begin : 0U;
        const double target = tunnel.axis_yaw[local_index] + tunnel_yaw_params_.yaw_offset;
        if (std::abs(std::atan2(
            std::sin(reference.points[idx].yaw - target),
            std::cos(reference.points[idx].yaw - target))) > 0.20)
        {
          bad_index = idx;
          bad_yaw = reference.points[idx].yaw;
          bad_target = target;
          bad_width = tunnel.width_estimate;
          break;
        }
      }
      if (bad_width > 0.0) {
        break;
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "【洞口对齐拒绝】目标=%llu 快照=%llu，候选进入窄通道前未对向入口 "
      "(idx=%zu/%zu yaw=%.3f target=%.3f width=%.3f)。",
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(map_snapshot->publication_sequence),
      bad_index, reference.points.size(), bad_yaw, bad_target, bad_width);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  bool robot_in_or_arming_tunnel = false;
  if (tunnel_lock_gimbal_ && !forward_tunnels.empty() && !reference.points.empty()) {
    for (const auto & tunnel : forward_tunnels) {
      const double dyn_pre = computeDynamicPreAlignDistance(
        reference, tunnel, reference.points.front().yaw,
        replan_start.yaw_rate, tunnel_yaw_params_,
        spin_yaw_params_.blend_rate_limit, spin_yaw_params_.angular_accel);
      const TunnelTraversalPhase start_phase = classifyTunnelTraversalPhase(
        reference, 0U, tunnel, dyn_pre, tunnel_yaw_params_);
      if (start_phase == TunnelTraversalPhase::kArmed ||
        start_phase == TunnelTraversalPhase::kCommitted ||
        start_phase == TunnelTraversalPhase::kExit)
      {
        robot_in_or_arming_tunnel = true;
        break;
      }
    }
  }
  if (robot_in_or_arming_tunnel) {
    // 隧道内（含洞口锁定段与整车退洞段）严格禁止小陀螺并锁定云台不动，车头正方向保持对齐隧道。
    yaw_authority = ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW;
  }
  if (spin_policy_.enabled && !force_body_yaw_follow_ && !robot_in_or_arming_tunnel) {
    ReferenceTrajectory spun = reference;
    SpinYawParams spin_params = spin_yaw_params_;
    spin_params.initial_rate = replan_start.yaw_rate;
    spin_params.stop_before_narrow_m = spin_policy_.align_lookahead_m;
    bool spin_ok = applySpinYaw(
      spun, spin_params,
      forward_tunnels.empty() ? nullptr : &forward_tunnels,
      &tunnel_yaw_params_);
    const bool attempted = spin_ok;
    FootprintSafetyResult spun_safety;
    if (spin_ok) {
      applyWheelSpeedTimeScaling(spun, wheel_speed_time_scaling_params_);
      annotateClearance(spun, *map_snapshot);
      spun_safety = safety_checker_.check(spun, planning_grid, clearance_esdf.get());
      spin_ok = spun.valid() && spun_safety.safe &&
        forwardTunnelEntriesAligned(spun, forward_tunnels, tunnel_yaw_params_);
    }
    if (spin_ok) {
      reference = std::move(spun);
      safety = std::move(spun_safety);
      quality = quality_evaluator_.evaluate(
        reference, clearance_esdf.get(), footprintSamples(planning_grid),
        selected_trace.segment_durations);
      yaw_authority = ats_navigation_interfaces::msg::PlannerStatus::YAW_AUTHORITY_SPIN;
    } else if (attempted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_throttle_ms_,
        "【小陀螺回退】自旋 yaw 未通过足迹门禁或序列无效，沿用规划 yaw。");
    }
  }
  // Escape-prefix geometry/depth is diagnostic, not an execution permission.
  // All final references, including recovery and debug candidates, obey the
  // same zero-collision contract. Keep the candidate machinery for diagnosis.
  if (!safety.safe) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【最终提交拒绝】参考仍有 %zu 个足迹冲突；脱困前缀不豁免，保持急停。",
      safety.collisions.size());
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_FOOTPRINT,
      map_snapshot->generation);
    return;
  }
  nav_msgs::msg::Path control_reference;
  geometry_msgs::msg::TransformStamped control_from_grid;
  control_from_grid.header.frame_id.clear();
  if (!transformPathToGlobal(toPath(reference), control_reference, &control_from_grid)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考发布失败】控制坐标系转换失败，未发布 MINCO 参考。");
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_REFERENCE_TF,
      map_snapshot->generation);
    return;
  }
  const double total_plan_wall_ms = elapsedWallMs();
  if (plan_hard_wall_budget_ms_ > 0.0 && total_plan_wall_ms > plan_hard_wall_budget_ms_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【规划墙钟超预算】goal=%llu total_wall_ms=%.2f > hard_budget=%.2f，按契约拒绝提交。",
      static_cast<unsigned long long>(goal_id), total_plan_wall_ms, plan_hard_wall_budget_ms_);
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_OPTIMIZER,
      map_snapshot->generation);
    return;
  }
  PlannerCommitTelemetry telemetry;
  telemetry.grid_topic = planning_snapshot_topic_.empty() ? grid_topic_ : planning_snapshot_topic_;
  telemetry.validation_frame = planning_grid.header.frame_id.empty() ?
    global_frame_ : planning_grid.header.frame_id;
  telemetry.gate_collision_count =
    static_cast<std::uint32_t>(safety.collisions.size());
  telemetry.gate_collision_indices = formatCollisionIndices(safety.collisions);
  telemetry.occupancy_digest = planning_digest::occupancyDigest(planning_grid);
  telemetry.content_digest = planning_digest::pathContentDigest(control_reference);
  telemetry.reference_point_count =
    static_cast<std::uint32_t>(reference.points.size());
  telemetry.is_partial_subgoal = search_result.partial;
  telemetry.has_spliced_tail = has_spliced_tail;
  telemetry.search_wall_ms = static_cast<float>(search_wall_ms);
  telemetry.minco_wall_ms = static_cast<float>(minco_wall_ms);
  telemetry.total_plan_wall_ms = static_cast<float>(total_plan_wall_ms);
  if (control_reference.poses.size() == reference.points.size()) {
    double rot_cos = 1.0;
    double rot_sin = 0.0;
    if (!control_from_grid.header.frame_id.empty()) {
      tf2::Transform tf_ctrl_from_grid;
      tf2::fromMsg(control_from_grid.transform, tf_ctrl_from_grid);
      const auto & basis = tf_ctrl_from_grid.getBasis();
      const double delta_yaw = std::atan2(basis[1][0], basis[0][0]);
      rot_cos = std::cos(delta_yaw);
      rot_sin = std::sin(delta_yaw);
    }
    const std::size_t n_pts = reference.points.size();
    telemetry.reference_vx.reserve(n_pts);
    telemetry.reference_vy.reserve(n_pts);
    telemetry.reference_ax.reserve(n_pts);
    telemetry.reference_ay.reserve(n_pts);
    telemetry.reference_yaw_rate.reserve(n_pts);
    for (const auto & pt : reference.points) {
      telemetry.reference_vx.push_back(rot_cos * pt.vx - rot_sin * pt.vy);
      telemetry.reference_vy.push_back(rot_sin * pt.vx + rot_cos * pt.vy);
      telemetry.reference_ax.push_back(rot_cos * pt.ax - rot_sin * pt.ay);
      telemetry.reference_ay.push_back(rot_sin * pt.ax + rot_cos * pt.ay);
      telemetry.reference_yaw_rate.push_back(pt.yaw_rate);
    }
  }
  {
    std::lock_guard<std::mutex> stats_lock(plan_wall_stats_.mutex);
    constexpr std::size_t kMaxSamples = 2048U;
    ++plan_wall_stats_.calls;
    if (minco_wall_ms > minco_stage_budget_ms_) {
      ++plan_wall_stats_.minco_over_budget;
    }
    if (total_plan_wall_ms > plan_wall_budget_ms_) {
      ++plan_wall_stats_.plan_over_budget;
    }
    if (plan_wall_stats_.minco_samples_ms.size() < kMaxSamples) {
      plan_wall_stats_.minco_samples_ms.push_back(minco_wall_ms);
      plan_wall_stats_.total_samples_ms.push_back(total_plan_wall_ms);
    } else {
      const std::size_t idx = plan_wall_stats_.ring_cursor % kMaxSamples;
      plan_wall_stats_.minco_samples_ms[idx] = minco_wall_ms;
      plan_wall_stats_.total_samples_ms[idx] = total_plan_wall_ms;
      ++plan_wall_stats_.ring_cursor;
    }
    const double minco_p50 = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 0.50);
    const double minco_p95 = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 0.95);
    const double minco_max = empiricalPercentile(plan_wall_stats_.minco_samples_ms, 1.00);
    const double plan_p50 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.50);
    const double plan_p95 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.95);
    const double plan_p99 = empiricalPercentile(plan_wall_stats_.total_samples_ms, 0.99);
    const double plan_max = empiricalPercentile(plan_wall_stats_.total_samples_ms, 1.00);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TRACE plan_wall_stats calls=%llu search_ms=%.2f minco_ms=%.2f total_ms=%.2f "
      "minco_p50_ms=%.2f minco_p95_ms=%.2f minco_max_ms=%.2f "
      "plan_p50_ms=%.2f plan_p95_ms=%.2f plan_p99_ms=%.2f plan_max_ms=%.2f "
      "minco_over_budget=%llu plan_over_budget=%llu spliced=%d partial=%d",
      static_cast<unsigned long long>(plan_wall_stats_.calls),
      search_wall_ms, minco_wall_ms, total_plan_wall_ms,
      minco_p50, minco_p95, minco_max,
      plan_p50, plan_p95, plan_p99, plan_max,
      static_cast<unsigned long long>(plan_wall_stats_.minco_over_budget),
      static_cast<unsigned long long>(plan_wall_stats_.plan_over_budget),
      has_spliced_tail ? 1 : 0, search_result.partial ? 1 : 0);
  }
  if (!publishReferenceIfCurrent(map_snapshot, map_health_epoch,
                                 control_reference, reference, goal_id, localization_epoch,
                                 plan_request_sequence, map_publication_sequence, yaw_authority,
                                 report_status, telemetry,
                                 control_from_grid.header.frame_id.empty() ?
                                 nullptr : &control_from_grid,
                                 replan_start.start_delay_sec,
                                 keep_active_reference)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【参考提交丢弃】generation=%llu 因规划地图变化或过期而丢弃，等待重规划。",
      static_cast<unsigned long long>(map_snapshot->generation));
    fail(ats_navigation_interfaces::msg::PlannerStatus::FAILURE_SNAPSHOT_CHANGED,
      map_snapshot->generation);
    return;
  }
  if (committed_for_start) {
    // 只有无停车重规划才有这一行（不节流，每次建议重规划一行）。source=committed 表示新起点
    // 取自旧参考，measured 表示退回实测，reason 说明原因。start_gap 是新起点到请求时实测
    // 位姿的距离，约等于 车速*plan_lead；拿它和提交时刻车的实际位置比，就是新参考采用瞬间
    // 的起点间隙。plan_ms 为进入 planGoal 到提交的节点时钟耗时，plan_wall_ms 为同段墙钟耗时。
    const double plan_ms = (now() - plan_begin_time).seconds() * 1000.0;
    const double plan_wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - plan_wall_begin).count();
    const double start_gap = std::hypot(
      start.pose.position.x - measured_start_x, start.pose.position.y - measured_start_y);
    RCLCPP_INFO(
      get_logger(),
      "【重规划起点】目标=%llu 请求=%llu source=%s reason=%s frame_ok=%d elapsed=%.3f "
      "progress=%.3f lag=%.3f t_ref=%.3f origin_delay=%.3f deviation=%.3f plan_lead=%.3f "
      "deviation_max=%.3f measured=(%.3f, %.3f) start=(%.3f, %.3f, yaw=%.3f) start_gap=%.3f "
      "v=(%.3f, %.3f) a=(%.3f, %.3f) plan_ms=%.1f plan_wall_ms=%.1f",
      static_cast<unsigned long long>(goal_id),
      static_cast<unsigned long long>(plan_request_sequence),
      replan_start.source == ReplanStartSource::kCommitted ? "committed" : "measured",
      replanStartReasonName(replan_start.reason), replan_frame_matches ? 1 : 0,
      replan_elapsed_sec, replan_start.progress, replan_elapsed_sec - replan_start.progress,
      replan_start.t_ref, replan_start.start_delay_sec, replan_start.deviation,
      replan_start_params_.plan_lead_sec, replan_start_params_.deviation_max_m,
      measured_start_x, measured_start_y, start.pose.position.x, start.pose.position.y,
      tf2::getYaw(start.pose.orientation), start_gap, replan_start.vx, replan_start.vy,
      replan_start.ax, replan_start.ay, plan_ms, plan_wall_ms);
  }
  if (marker_pub_ && marker_pub_->get_subscription_count() > 0) {
    // Marker publication follows the immutable-snapshot commit, so the red
    // curve is exactly the safety-validated reference handed to MPC.
    marker_pub_->publish(visualizer_.buildMarkers(
      selected_trace.preprocessed_guide, reference, safety));
  }

  std::ostringstream durations;
  durations.setf(std::ios::fixed);
  durations.precision(3);
  for (std::size_t index = 0; index < quality.segment_durations.size(); ++index) {
    if (index > 0U) {
      durations << ',';
    }
    durations << quality.segment_durations[index];
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), log_throttle_ms_,
    "【规划完成】planned generation=%llu snapshot_publication=%llu "
    "raw_points=%zu preprocessed_points=%zu "
    "esdf_refined_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu "
    "footprint_collisions=%zu swept_collisions=%zu "
    "collision_indices=%s occupancy_digest=%s content_digest=%s grid_topic=%s validation_frame=%s "
    "fallback=%d local_repair=%d optimization_failure_reason=%s escape_prefix_end=%zu "
    "expanded=%d yaw_authority=%u center_clearance=%.3f footprint_clearance=%.3f "
    "length_ratio=%.3f lateral=%.3f curvature_max=%.3f curvature_p95=%.3f turn=%.3f "
    "curvature_tv=%.3f curvature_sign_changes=%zu local_scaled=%d uniform_scaled=%d "
    "peak_v=%.3f peak_a=%.3f peak_j=%.3f solver_wall_ms=%.3f seed_scale=%.2f "
    "joint_pieces=%d joint_iterations=%d joint_evaluations=%d joint_termination=%s "
    "joint_cost=%.3f->%.3f joint_wall_ms=%.3f joint_satisfied=%d joint_center_violation=%.3f "
    "joint_footprint_violation=%.3f joint_guide_excess=%.3f joint_v_excess=%.3f "
    "joint_a_excess=%.3f "
    "joint_lat_excess=%.3f segment_durations=[%s]",
    static_cast<unsigned long long>(map_snapshot->generation),
    static_cast<unsigned long long>(map_publication_sequence), search_result.path.poses.size(),
    selected_trace.preprocessed_guide.poses.size(), selected_trace.esdf_refined_guide.poses.size(),
    reference.points.size(), reference.totalLength(), reference.totalTime(), safety.collisions.size(),
    quality.footprint_collision_count, quality.swept_collision_count,
    telemetry.gate_collision_indices.c_str(),
    telemetry.occupancy_digest.c_str(), telemetry.content_digest.c_str(), telemetry.grid_topic.c_str(),
    telemetry.validation_frame.c_str(),
    esdf_fallback_used ? 1 : 0, local_repair_used ? 1 : 0,
    selected_trace.failure_reason.empty() ? "none" : selected_trace.failure_reason.c_str(),
    escape_decision.prefix_end,
    search_result.expanded_nodes, static_cast<unsigned int>(yaw_authority),
    quality.minimum_center_clearance, quality.minimum_footprint_clearance, quality.length_ratio,
    quality.max_lateral_deviation, quality.max_geometric_curvature,
    quality.p95_geometric_curvature, quality.total_turning_angle,
    quality.curvature_total_variation, quality.curvature_sign_changes,
    selected_trace.local_time_scaled ? 1 : 0, selected_trace.uniform_time_scaled ? 1 : 0,
    quality.peak_velocity, quality.peak_acceleration, quality.peak_jerk,
    selected_trace.solver_wall_time_ms, selected_trace.seed_scale,
    selected_trace.joint_piece_count,
    selected_trace.joint_iterations, selected_trace.joint_evaluations,
    selected_trace.joint_termination.empty() ? "none" : selected_trace.joint_termination.c_str(),
    selected_trace.joint_initial_cost, selected_trace.joint_final_cost,
    selected_trace.joint_wall_time_ms, selected_trace.joint_constraints_satisfied ? 1 : 0,
    selected_trace.joint_max_center_violation, selected_trace.joint_max_footprint_violation,
    selected_trace.joint_max_guide_excess, selected_trace.joint_max_velocity_excess,
    selected_trace.joint_max_acceleration_excess,
    selected_trace.joint_max_lateral_excess, durations.str().c_str());
}

void MincoPlannerNode::annotatePositionClearance(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const
{
  const bool available = snapshot.clearance_esdf && snapshot.clearance_esdf->available();
  for (auto & point : trajectory.points) {
    point.clearance = available ?
      snapshot.clearance_esdf->getDistance(point.x, point.y) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

void MincoPlannerNode::planYaw(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot,
  double start_yaw, double goal_yaw, bool is_final_goal) const
{
  // 这三步的顺序就是这段逻辑的全部内容,所以收进一个函数,避免四个调用点各自写错。
  // 窄通道判据必须用与 yaw 无关的位置净空:yaw 正是这一步要决定的量,拿 yaw 相关的
  // footprint 净空当输入构成循环依赖,而 annotateClearance 恰好是 yaw 相关的。
  annotatePositionClearance(trajectory, snapshot);
  // 净空限速只依赖与 yaw 无关的中心净空，放在 yaw 规划之前执行：这样弯窄通道内已具备真实降速后的
  // 时间步长 dt，避免高速时间网格下角加速度受限导致洞口/弯隧道内 yaw 跟踪滞后。
  applyClearanceSpeedTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  std::vector<TunnelSegment> tunnel_segments;
  if (tunnel_yaw_params_.enabled && snapshot.clearance_esdf) {
    tunnel_segments = detectTunnelSegments(
      trajectory, *snapshot.clearance_esdf, tunnel_yaw_params_);
    resolveTerminalTunnelGoalAlignment(
      tunnel_segments, trajectory, goal_yaw, is_final_goal, tunnel_yaw_params_);
  }
  const std::vector<TunnelSegment> * tunnels =
    tunnel_segments.empty() ? nullptr : &tunnel_segments;
  // 起点在窄通道里时多算一版"先保持起点 yaw 平移"的航向（报告 5.1.12 第 2 项），只有它的
  // 矩形足迹冲突不多于原航向（整条与前 5 个点两项都不多）时才采用。最终提交仍要过同一套
  // 足迹门禁，这里只在两种 yaw 时序之间选，不放宽任何判据。
  std::optional<ReferenceTrajectory> held;
  if (narrow_start_hold_length_ > 0.0 && trajectory.points.size() > 1U &&
    std::isfinite(trajectory.points[1].clearance) &&
    trajectory.points[1].clearance <= yaw_planner_.params().narrow_clearance_enter)
  {
    held = trajectory;
  }
  yaw_planner_.apply(trajectory, start_yaw, goal_yaw, tunnels, is_final_goal);
  if (held) {
    narrow_start_hold_yaw_planner_.apply(*held, start_yaw, goal_yaw, tunnels, is_final_goal);
    if (yawSequencesDiffer(*held, trajectory)) {
      const FootprintSafetyResult planned = safety_checker_.check(
        trajectory, snapshot.grid, snapshot.clearance_esdf.get());
      const FootprintSafetyResult holding = safety_checker_.check(
        *held, snapshot.grid, snapshot.clearance_esdf.get());
      const auto early = [](const FootprintSafetyResult & result) {
          return std::count_if(
            result.collisions.begin(), result.collisions.end(),
            [](const CollisionSample & sample) {return sample.trajectory_index < 5U;});
        };
      auto planned_tunnels = (tunnel_yaw_params_.enabled && snapshot.clearance_esdf &&
        !tunnel_segments.empty()) ?
        detectTunnelSegments(trajectory, *snapshot.clearance_esdf, tunnel_yaw_params_) :
        tunnel_segments;
      resolveTerminalTunnelGoalAlignment(
        planned_tunnels, trajectory, goal_yaw, is_final_goal, tunnel_yaw_params_);
      auto holding_tunnels = (tunnel_yaw_params_.enabled && snapshot.clearance_esdf &&
        !tunnel_segments.empty()) ?
        detectTunnelSegments(*held, *snapshot.clearance_esdf, tunnel_yaw_params_) :
        tunnel_segments;
      resolveTerminalTunnelGoalAlignment(
        holding_tunnels, *held, goal_yaw, is_final_goal, tunnel_yaw_params_);
      const bool planned_aligned =
        forwardTunnelEntriesAligned(trajectory, planned_tunnels, tunnel_yaw_params_);
      const bool holding_aligned =
        forwardTunnelEntriesAligned(*held, holding_tunnels, tunnel_yaw_params_);
      if ((!planned_aligned || holding_aligned) &&
        holding.collisions.size() <= planned.collisions.size() &&
        early(holding) <= early(planned))
      {
        trajectory = std::move(*held);
      }
    }
  }
  // yaw 定下后才知道平移与转向叠加的轮速；超出底盘能力的局部放慢时间，位置和 yaw 不变。
  applyWheelSpeedTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  // 窄通道边平移边转向时再放慢，压低 MPC 横向/yaw 跟踪误差；同样只改时间。
  applyNarrowTurnTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  // 下游(yaw authority、轨迹质量评估、planned 记录)读的是 yaw 相关的 footprint 净空,
  // 所以 yaw 定下来之后再覆盖回 footprint 净空,对外语义不变。
  annotateClearance(trajectory, snapshot);
}

void MincoPlannerNode::annotateClearance(
  ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const
{
  const bool available = snapshot.clearance_esdf && snapshot.clearance_esdf->available();
  const std::vector<Eigen::Vector2d> samples = footprintSamples(snapshot.grid);
  for (auto & point : trajectory.points) {
    point.clearance = available ? snapshot.clearance_esdf->getFootprintClearance(
      Eigen::Vector2d(point.x, point.y), point.yaw, samples) :
      std::numeric_limits<double>::quiet_NaN();
  }
}

std::vector<Eigen::Vector2d> MincoPlannerNode::footprintSamples(
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const double half_length = 0.5 * std::max(0.0, footprint_length_) + footprint_safety_margin_;
  const double half_width = 0.5 * std::max(0.0, footprint_width_) + footprint_safety_margin_;
  const double resolution = std::max(0.02, static_cast<double>(grid.info.resolution));
  const int samples_x = std::max(2, static_cast<int>(std::ceil((2.0 * half_length) / resolution)));
  const int samples_y = std::max(2, static_cast<int>(std::ceil((2.0 * half_width) / resolution)));
  // ESDF 对外部障碍的极小值落在矩形边界上，内部辅以中心及四象限采样点兼顾孤立内部障碍。
  std::vector<Eigen::Vector2d> samples;
  samples.reserve(static_cast<std::size_t>(2 * (samples_x + samples_y) + 5));
  for (int ix = 0; ix <= samples_x; ++ix) {
    const double x = -half_length + 2.0 * half_length * ix / static_cast<double>(samples_x);
    samples.emplace_back(x, -half_width);
    samples.emplace_back(x, half_width);
  }
  for (int iy = 1; iy < samples_y; ++iy) {
    const double y = -half_width + 2.0 * half_width * iy / static_cast<double>(samples_y);
    samples.emplace_back(-half_length, y);
    samples.emplace_back(half_length, y);
  }
  samples.emplace_back(0.0, 0.0);
  samples.emplace_back(-0.5 * half_length, -0.5 * half_width);
  samples.emplace_back(-0.5 * half_length, 0.5 * half_width);
  samples.emplace_back(0.5 * half_length, -0.5 * half_width);
  samples.emplace_back(0.5 * half_length, 0.5 * half_width);
  return samples;
}

bool MincoPlannerNode::lookupStartPose(
  const nav_msgs::msg::OccupancyGrid & grid, geometry_msgs::msg::PoseStamped & start)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      !grid.header.frame_id.empty() ? grid.header.frame_id : global_frame_,
      robot_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_lookup_timeout_sec_));
    start.header = transform.header;
    start.pose.position.x = transform.transform.translation.x;
    start.pose.position.y = transform.transform.translation.y;
    // The planning grid is a 2D ground plane: the robot_frame height (gimbal /
    // lidar mount) must not lift the JPS/MINCO start above the grid.
    start.pose.position.z = grid.info.origin.position.z;
    start.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【TF失败】查询 %s -> %s 失败：%s",
      global_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

bool MincoPlannerNode::resolveStartPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped * frozen_start,
  geometry_msgs::msg::PoseStamped & start)
{
  if (frozen_start == nullptr || frozen_start->header.frame_id.empty()) {
    return lookupStartPose(grid, start);
  }
  if (!finitePose(frozen_start->pose)) {
    return false;
  }
  return transformGoalToGrid(grid, *frozen_start, start);
}

bool MincoPlannerNode::transformGoalToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output)
{
  const std::string target_frame = grid.header.frame_id.empty() ? global_frame_ : grid.header.frame_id;
  if (input.header.frame_id.empty() || input.header.frame_id == target_frame) {
    output = input;
    output.header.frame_id = target_frame;
    output.pose.position.z = grid.info.origin.position.z;
    return true;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    tf2::doTransform(input, output, transform);
    // Start/goal poses arrive at the localization frame height; project them
    // onto the 2D planning grid plane so every path vertex stays on the ground.
    output.pose.position.z = grid.info.origin.position.z;
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【TF失败】查询 %s -> %s 失败：%s", input.header.frame_id.c_str(),
      target_frame.c_str(), exception.what());
    return false;
  }
}

bool MincoPlannerNode::transformPathToGlobal(
  const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output,
  geometry_msgs::msg::TransformStamped * used_transform)
{
  const std::string source_frame =
    input.header.frame_id.empty() ? global_frame_ : input.header.frame_id;
  if (source_frame == global_frame_) {
    output = input;
    output.header.frame_id = global_frame_;
    for (auto & pose : output.poses) {
      pose.header.frame_id = global_frame_;
    }
    return true;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      global_frame_, source_frame, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    const auto body = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    const double control_height = body.transform.translation.z;
    tf2::Transform control_from_grid;
    tf2::fromMsg(transform.transform, control_from_grid);
    const auto & rotation = control_from_grid.getBasis();
    if (!std::isfinite(control_height) || !std::isfinite(rotation[2][2]) ||
      std::abs(rotation[2][2]) < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "【参考转换拒绝】控制平面高度无效或与规划平面投影奇异，保持急停。");
      return false;
    }
    nav_msgs::msg::Path candidate;
    candidate.header = input.header;
    candidate.header.frame_id = global_frame_;
    candidate.poses.reserve(input.poses.size());
    for (const auto & pose : input.poses) {
      if (!finitePose(pose.pose)) {return false;}
      auto lifted = pose;
      // Preserve validated planning XY while landing on the body's current
      // control Z plane. Directly transforming grid Z=0 shifts a planar
      // controller's XY target when map->odom contains roll/pitch.
      lifted.pose.position.z = (control_height - control_from_grid.getOrigin().z() -
        rotation[2][0] * pose.pose.position.x - rotation[2][1] * pose.pose.position.y) /
        rotation[2][2];
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(lifted, transformed, transform);
      if (!finitePose(transformed.pose)) {return false;}
      transformed.pose.position.z = control_height;
      transformed.header = pose.header;
      transformed.header.frame_id = global_frame_;
      candidate.poses.push_back(std::move(transformed));
    }
    output = std::move(candidate);
    if (used_transform) {*used_transform = transform;}
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "【TF失败】MINCO 参考查询 %s -> %s 失败：%s",
      source_frame.c_str(), global_frame_.c_str(), exception.what());
    return false;
  }
}

void MincoPlannerNode::publishEmergencyStop(bool stop)
{
  if (!planner_manages_emergency_stop_ || !emergency_stop_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = stop;
  emergency_stop_pub_->publish(message);
}

void MincoPlannerNode::publishPlannerStatus(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_generation, std::uint64_t map_publication_sequence,
    std::uint8_t state, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time &reference_stamp,
    std::uint8_t yaw_authority, bool requires_gimbal_lock,
    const PlannerCommitTelemetry & telemetry) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  publishPlannerStatusLocked(
    goal_id, localization_epoch, plan_request_sequence, map_generation,
    map_publication_sequence, state, failure_reason, reference_stamp,
    yaw_authority, requires_gimbal_lock, telemetry);
}

void MincoPlannerNode::publishPlannerStatusLocked(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_generation, std::uint64_t map_publication_sequence,
    std::uint8_t state, std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time & reference_stamp,
    std::uint8_t yaw_authority, bool requires_gimbal_lock,
    const PlannerCommitTelemetry & telemetry) {
  if (!planner_status_pub_) {
    return;
  }
  ats_navigation_interfaces::msg::PlannerStatus status;
  status.header.stamp = now();
  status.header.frame_id = global_frame_;
  status.goal_id = goal_id;
  status.localization_epoch = localization_epoch;
  status.plan_request_sequence = plan_request_sequence;
  status.map_generation = map_generation;
  if (status.map_generation == 0) {
    status.map_generation = next_map_generation_;
  }
  status.map_publication_sequence = map_publication_sequence;
  status.state = state;
  status.reference_stamp = reference_stamp;
  status.failure_reason = failure_reason;
  status.yaw_authority = yaw_authority;
  status.requires_gimbal_lock = requires_gimbal_lock;
  status.grid_topic = telemetry.grid_topic;
  status.validation_frame = telemetry.validation_frame;
  status.gate_collision_count = telemetry.gate_collision_count;
  status.gate_collision_indices = telemetry.gate_collision_indices;
  status.occupancy_digest = telemetry.occupancy_digest;
  status.content_digest = telemetry.content_digest;
  status.reference_point_count = telemetry.reference_point_count;
  if (state == ats_navigation_interfaces::msg::PlannerStatus::STATE_REFERENCE_READY) {
    status.reference_vx = telemetry.reference_vx;
    status.reference_vy = telemetry.reference_vy;
    status.reference_ax = telemetry.reference_ax;
    status.reference_ay = telemetry.reference_ay;
    status.reference_yaw_rate = telemetry.reference_yaw_rate;
    status.is_partial_subgoal = telemetry.is_partial_subgoal;
    status.has_spliced_tail = telemetry.has_spliced_tail;
  }
  status.search_wall_ms = telemetry.search_wall_ms;
  status.minco_wall_ms = telemetry.minco_wall_ms;
  status.total_plan_wall_ms = telemetry.total_plan_wall_ms;
  if (!planner_status_state_.update(status)) {
    return;
  }
  if (state == ats_navigation_interfaces::msg::PlannerStatus::STATE_ACCEPTED) {
    // Acceptance is not a map-health/reference grant.  Retain the local
    // identity for invalidation, but preserve the unbound wire status until
    // the commit gate publishes REFERENCE_READY.
    status.map_generation = map_generation;
  }
  planner_status_pub_->publish(status);
}

void MincoPlannerNode::invalidatePlannerStatusLocked(
    std::uint8_t failure_reason, std::uint64_t generation) {
  if (planner_status_pub_ && planner_status_state_.invalidate(generation, failure_reason, now())) {
    planner_status_pub_->publish(*planner_status_state_.latest);
  }
}

void MincoPlannerNode::invalidateMapLocked() {
  reference_conflict_.reset();
  far_conflict_streak_ = 0;
  far_conflict_reference_stamp_.reset();
  ++map_health_epoch_;
  latest_map_snapshot_.reset();
  active_safety_reference_.reset();
  safety_state_.invalidateMap(next_map_generation_);
  invalidatePlannerStatusLocked(
    ats_navigation_interfaces::msg::PlannerStatus::FAILURE_MAP_UNREADY,
    next_map_generation_);
}

void MincoPlannerNode::setPlanSafe(bool safe)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    safety_state_.plan_safe = safe;
    publishEmergencyStop(safety_state_.emergencyStopRequired());
  }
}

bool MincoPlannerNode::publishReferenceIfCurrent(
    const std::shared_ptr<const PlanningMapSnapshot> &snapshot,
    std::uint64_t map_health_epoch, const nav_msgs::msg::Path &reference_path,
    const ReferenceTrajectory &safety_reference,
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence,
    std::uint64_t map_publication_sequence,
    std::uint8_t yaw_authority,
    bool report_status,
    const PlannerCommitTelemetry & telemetry,
    const geometry_msgs::msg::TransformStamped * control_from_grid,
    double reference_origin_delay_sec,
    bool preserve_active_reference) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  const bool heartbeat_fresh = map_ready_topic_.empty() || steadyHeartbeatLeaseValid(
    last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_);
  if (!heartbeat_fresh && safety_state_.map_ready) {
    invalidateMapLocked();
  }
  const bool publish = heartbeat_fresh && snapshot && latest_map_snapshot_ &&
    safety_reference.header.frame_id == snapshot->grid.header.frame_id &&
    safety_state_.mapSnapshotUsable(snapshot->generation) &&
    map_health_epoch == map_health_epoch_ &&
    snapshot->generation == latest_map_snapshot_->generation &&
    (!control_from_grid || reference_path.poses.size() == safety_reference.points.size());
  if (!publish) {
    // 无停车重规划的替换候选过期：正在执行的旧参考仍受运行期复检守护，只在地图
    // 健康且快照可用时保留它；其余情况照旧撤销。
    const bool keep_guarded_reference = heartbeat_fresh && latest_map_snapshot_ &&
      (preserve_active_reference || replan_advised_reference_stamp_.has_value()) &&
      active_safety_reference_ &&
      active_safety_reference_->goal_id == goal_id &&
      active_safety_reference_->localization_epoch == localization_epoch &&
      active_safety_reference_->plan_request_sequence < plan_request_sequence &&
      safety_state_.mapSnapshotUsable(latest_map_snapshot_->generation);
    if (!keep_guarded_reference) {
      safety_state_.plan_safe = false;
      publishEmergencyStop(true);
    }
    return false;
  }
  if (!planner_manages_emergency_stop_ && !candidate_reference_path_pub_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "【规划器配置错误】P3 模式没有候选参考发布器。");
    return false;
  }
  nav_msgs::msg::Path committed_reference = reference_path;
  // 新参考的 t=0 是旧参考上的取点时刻 t0（附录 A.3），比提交时刻晚 reference_origin_delay_sec；
  // 沿用这里已有的时间重基，不另加一套机制。GoalManager 提交时仍会按自己的 now() 重基。
  rebasePathTimestamps(
    committed_reference,
    now() + rclcpp::Duration::from_seconds(std::max(0.0, reference_origin_delay_sec)));
  safety_state_.plan_safe = true;
  ActiveSafetyReference active_reference;
  active_reference.trajectory = safety_reference;
  active_reference.trajectory.header.stamp = committed_reference.header.stamp;
  active_reference.goal_id = goal_id;
  active_reference.localization_epoch = localization_epoch;
  active_reference.plan_request_sequence = plan_request_sequence;
  active_reference.map_generation = snapshot->generation;
  active_reference.map_publication_sequence = map_publication_sequence;
  if (global_route_ && global_route_->snapshot->generation == snapshot->generation &&
    global_route_->goal_id == goal_id && global_route_->localization_epoch == localization_epoch)
  {
    active_reference.guide = global_route_->path;
    active_reference.guide_snapshot = global_route_->snapshot;
  }
  if (control_from_grid) {
    active_reference.control_from_grid = *control_from_grid;
    active_reference.control_reference = committed_reference;
  }
  active_safety_reference_ = std::move(active_reference);
  far_conflict_streak_ = 0;
  far_conflict_reference_stamp_.reset();
  if (planner_manages_emergency_stop_) {
    // Preserve the P2 stop-clear/reference ordering inside the same map lock.
    publishEmergencyStop(false);
    reference_path_pub_->publish(committed_reference);
  } else {
    // Commit candidate and READY before any map/health invalidation can publish.
    // GoalManager remains the sole P3 final-reference/emergency-stop authority.
    candidate_reference_path_pub_->publish(committed_reference);
    if (report_status) {
      publishPlannerStatusLocked(
        goal_id, localization_epoch, plan_request_sequence,
        snapshot->generation, map_publication_sequence,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_REFERENCE_READY,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE,
        committed_reference.header.stamp, yaw_authority,
        yaw_authority == ats_navigation_interfaces::msg::PlannerStatus::
          YAW_AUTHORITY_BODY_YAW_FOLLOW,
        telemetry);
    }
  }
  return true;
}

nav_msgs::msg::Path MincoPlannerNode::toPath(const ReferenceTrajectory & trajectory) const
{
  nav_msgs::msg::Path path;
  path.header = trajectory.header;
  path.poses.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory.header;
    const rclcpp::Time point_stamp =
      rclcpp::Time(trajectory.header.stamp) + rclcpp::Duration::from_seconds(point.t);
    pose.header.stamp.sec = static_cast<int32_t>(point_stamp.nanoseconds() / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<uint32_t>(point_stamp.nanoseconds() % 1000000000LL);
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * point.yaw);
    pose.pose.orientation.w = std::cos(0.5 * point.yaw);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace minco_planner

RCLCPP_COMPONENTS_REGISTER_NODE(minco_planner::MincoPlannerNode)
