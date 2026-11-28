// Copyright 2026

#include "minco_planner_node_helpers.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace minco_planner
{

using namespace node_detail;

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
  if (!rolling_anchor_topic_.empty()) {
    rolling_anchor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      rolling_anchor_topic_, rclcpp::QoS(1).reliable());
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
  double start_yaw, double goal_yaw, bool is_final_goal,
  std::optional<double> preferred_lidar_fov_yaw_offset,
  bool force_goal_yaw) const
{
  // 这三步的顺序就是这段逻辑的全部内容,所以收进一个函数,避免四个调用点各自写错。
  // 窄通道判据必须用与 yaw 无关的位置净空:yaw 正是这一步要决定的量,拿 yaw 相关的
  // footprint 净空当输入构成循环依赖,而 annotateClearance 恰好是 yaw 相关的。
  annotatePositionClearance(trajectory, snapshot);
  // 净空限速只依赖与 yaw 无关的中心净空，放在 yaw 规划之前执行：这样弯窄通道内已具备真实降速后的
  // 时间步长 dt，避免高速时间网格下角加速度受限导致洞口/弯隧道内 yaw 跟踪滞后。
  applyClearanceSpeedTimeScaling(trajectory, wheel_speed_time_scaling_params_);
  if (force_goal_yaw) {
    yaw_planner_.apply(
      trajectory, start_yaw, goal_yaw, nullptr, true, std::nullopt, true);
    annotateClearance(trajectory, snapshot);
    return;
  }
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
  std::optional<ReferenceTrajectory> pre_yaw_for_mouth_spin;
  if (!tunnel_segments.empty() && !trajectory.points.empty() &&
    trajectory.points.front().v < 0.15 &&
    !tunnel_segments.front().forward_alignment &&
    tunnel_segments.front().begin > 0U &&
    !tunnel_segments.front().axis_yaw.empty())
  {
    pre_yaw_for_mouth_spin = trajectory;
  }
  yaw_planner_.apply(
    trajectory, start_yaw, goal_yaw, tunnels, is_final_goal, preferred_lidar_fov_yaw_offset);
  if (held) {
    narrow_start_hold_yaw_planner_.apply(
      *held, start_yaw, goal_yaw, tunnels, is_final_goal, preferred_lidar_fov_yaw_offset);
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
  if (pre_yaw_for_mouth_spin) {
    const FootprintSafetyResult current_safety = safety_checker_.check(
      trajectory, snapshot.grid, snapshot.clearance_esdf.get());
    if (!current_safety.safe) {
      auto forced_tunnels = tunnel_segments;
      auto & first_tunnel = forced_tunnels.front();
      const double mouth_ref_yaw = preferred_lidar_fov_yaw_offset.has_value()
        ? std::atan2(
            std::sin(first_tunnel.axis_yaw.front() - *preferred_lidar_fov_yaw_offset),
            std::cos(first_tunnel.axis_yaw.front() - *preferred_lidar_fov_yaw_offset))
        : start_yaw;
      const double nearest_axis = tunnelAlignedYaw(
        first_tunnel.axis_yaw.front(), tunnel_yaw_params_.yaw_offset, mouth_ref_yaw);
      const double base_target = std::atan2(
        std::sin(first_tunnel.axis_yaw.front() + tunnel_yaw_params_.yaw_offset),
        std::cos(first_tunnel.axis_yaw.front() + tunnel_yaw_params_.yaw_offset));
      if (std::abs(std::atan2(
          std::sin(nearest_axis - base_target), std::cos(nearest_axis - base_target))) > 0.5 * M_PI)
      {
        for (double & yaw : first_tunnel.axis_yaw) {
          yaw = std::atan2(std::sin(yaw + M_PI), std::cos(yaw + M_PI));
        }
      }
      first_tunnel.forward_alignment = true;
      yaw_planner_.apply(
        *pre_yaw_for_mouth_spin, start_yaw, goal_yaw, &forced_tunnels, is_final_goal,
        preferred_lidar_fov_yaw_offset);
      const FootprintSafetyResult spin_safety = safety_checker_.check(
        *pre_yaw_for_mouth_spin, snapshot.grid, snapshot.clearance_esdf.get());
      if (spin_safety.safe || spin_safety.collisions.size() < current_safety.collisions.size()) {
        trajectory = std::move(*pre_yaw_for_mouth_spin);
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

std::optional<double> MincoPlannerNode::resolveLidarFovYawOffsetRad()
{
  if (lidar_frame_.empty() || robot_frame_.empty() || !tf_buffer_) {
    return std::nullopt;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
      robot_frame_, lidar_frame_, tf2::TimePointZero,
      tf2::durationFromSec(std::min(tf_lookup_timeout_sec_, 0.02)));
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    if (!std::isfinite(q.x()) || !std::isfinite(q.y()) ||
      !std::isfinite(q.z()) || !std::isfinite(q.w()) || q.length2() < 1e-8)
    {
      return std::nullopt;
    }
    q.normalize();
    // Livox Mid-360 vertical FOV (-7..+52 deg) is symmetric around its sensor +Z optical axis.
    // Projecting sensor +Z into the robot XY plane yields the actual primary observation bearing
    // for any chassis/gimbal tilt mounting without hardcoded extrinsic offsets.
    const tf2::Vector3 z_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
    const double xy_norm = std::hypot(z_axis.x(), z_axis.y());
    if (xy_norm <= 0.05) {
      RCLCPP_INFO_ONCE(
        get_logger(),
        "【雷达观测方向】TF %s -> %s 光轴接近竖直（tilt=%.1f deg），全向360度视野无定向盲区。",
        robot_frame_.c_str(), lidar_frame_.c_str(),
        std::asin(std::clamp(xy_norm, 0.0, 1.0)) * 180.0 / M_PI);
      return std::nullopt;
    }
    const double yaw_offset = std::atan2(z_axis.y(), z_axis.x());
    RCLCPP_INFO_ONCE(
      get_logger(),
      "【雷达观测方向】TF %s -> %s 倾角=%.1f deg，水平主观测方位=%.1f deg（%.3f rad）。",
      robot_frame_.c_str(), lidar_frame_.c_str(),
      std::asin(std::clamp(xy_norm, 0.0, 1.0)) * 180.0 / M_PI,
      yaw_offset * 180.0 / M_PI, yaw_offset);
    return yaw_offset;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "【雷达观测方向】查询 TF %s -> %s 失败（%s），暂按车体+X(0 rad)估算。",
      robot_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    return 0.0;
  }
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
    bool preserve_active_reference,
    bool scan_turn) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  const bool heartbeat_fresh = map_ready_topic_.empty() || steadyHeartbeatLeaseValid(
    last_map_ready_signal_, std::chrono::steady_clock::now(), map_ready_timeout_sec_);
  if (!heartbeat_fresh && safety_state_.map_ready) {
    invalidateMapLocked();
  }
  auto effective_snapshot = snapshot;
  std::uint64_t effective_pub_seq = map_publication_sequence;
  PlannerCommitTelemetry committed_telemetry = telemetry;
  bool generation_current = snapshot && latest_map_snapshot_ &&
    snapshot->generation == latest_map_snapshot_->generation;
  if (!generation_current && heartbeat_fresh && snapshot && latest_map_snapshot_ &&
    map_health_epoch == map_health_epoch_ &&
    latest_map_snapshot_->generation > snapshot->generation &&
    latest_map_snapshot_->generation <= snapshot->generation + 3U &&
    latest_map_snapshot_->localization_epoch == snapshot->localization_epoch &&
    latest_map_snapshot_->grid.header.frame_id == safety_reference.header.frame_id &&
    safety_state_.mapSnapshotUsable(latest_map_snapshot_->generation))
  {
    // High-rate mapless updates (10 Hz) can advance generation by 1-2 ticks during a
    // 50-120 ms global+MINCO plan. Re-verify the exact trajectory against the newest
    // snapshot under the same atomic lock; if 100% footprint-safe, promote cleanly.
    const FootprintSafetyResult recheck = safety_checker_.check(
      safety_reference, latest_map_snapshot_->grid,
      latest_map_snapshot_->clearance_esdf.get());
    if (recheck.safe) {
      effective_snapshot = latest_map_snapshot_;
      effective_pub_seq = latest_map_snapshot_->publication_sequence;
      committed_telemetry.occupancy_digest =
        planning_digest::occupancyDigest(latest_map_snapshot_->grid);
      committed_telemetry.content_digest = latest_map_snapshot_->safety_content_digest;
      generation_current = true;
    }
  }
  const bool publish = heartbeat_fresh && effective_snapshot && latest_map_snapshot_ &&
    safety_reference.header.frame_id == effective_snapshot->grid.header.frame_id &&
    safety_state_.mapSnapshotUsable(effective_snapshot->generation) &&
    map_health_epoch == map_health_epoch_ &&
    generation_current &&
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
  active_reference.map_generation = effective_snapshot->generation;
  active_reference.map_publication_sequence = effective_pub_seq;
  if (global_route_ && global_route_->snapshot->generation == snapshot->generation &&
    global_route_->goal_id == goal_id && global_route_->localization_epoch == localization_epoch)
  {
    active_reference.guide = global_route_->path;
    active_reference.guide_snapshot = effective_snapshot;
    active_reference.guide_optimistic = global_route_->optimistic;
  }
  active_reference.scan_turn = scan_turn;
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
        effective_snapshot->generation, effective_pub_seq,
        ats_navigation_interfaces::msg::PlannerStatus::STATE_REFERENCE_READY,
        ats_navigation_interfaces::msg::PlannerStatus::FAILURE_NONE,
        committed_reference.header.stamp, yaw_authority,
        yaw_authority == ats_navigation_interfaces::msg::PlannerStatus::
          YAW_AUTHORITY_BODY_YAW_FOLLOW,
        committed_telemetry);
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
