// Copyright 2026

#include "minco_planner_node_helpers.hpp"

namespace minco_planner
{

using namespace node_detail;

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
    retain_candidate->guide_snapshot && !retain_candidate->scan_turn)
  {
    const auto remaining = remainingGuide(
      *retain_candidate->guide, progress_pose, replan_start_params_.deviation_max_m);
    if (retain_candidate->guide_optimistic) {
      if (snapshot->global_grid.has_value() && remaining.poses.size() >= 2U) {
        const GridOccupancyPolicy opt_policy{obstacle_value_threshold_, false};
        const auto global_blocked_sq =
          computeBlockedSquaredDistanceCells(*snapshot->global_grid, opt_policy);
        guide_blocked = isOptimisticPathBlocked(
          remaining, *snapshot->global_grid, global_blocked_sq, snapshot->grid,
          obstacle_value_threshold_, inscribed_footprint_radius_);
        if (guide_blocked) {
          retain_reason = "optimistic global guide newly obstructed";
        }
      }
    } else {
      guide_blocked = obstructedGuide(
        traceGuide(remaining), snapshot->grid, safety_checker_,
        &retain_candidate->guide_snapshot->grid).has_value();
      if (guide_blocked) {
        retain_reason = "global guide footprint conflict";
      }
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


}  // namespace minco_planner
