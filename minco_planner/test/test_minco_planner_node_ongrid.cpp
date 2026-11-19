// Copyright 2026

// Regression tests for the snapshot-change branches of MincoPlannerNode::onGrid:
//   retain     remaining reference is safe on the new grid -> rebind it to the new generation
//   advise     only the far part conflicts, the runtime window is safe -> keep executing and
//              publish FAILURE_REPLAN_ADVISED
//   invalidate near window unsafe, or retention disabled -> drop the reference (fail closed)
//
// The real node is driven synchronously, like tools/minco_sandbox_cli.cpp: nothing spins the
// node, so its timers never run. Private members are opened with `#define private public`
// in this translation unit only, after every header the node header includes.

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "minco_planner/debug/planner_debug_visualizer.hpp"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_jps.hpp"
#include "minco_planner/safety/escape_prefix.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/goal_pose_admission.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "minco_planner/trajectory/terminal_yaw_relocation.hpp"
#include "minco_planner/trajectory/trajectory_quality_evaluator.hpp"
#include "minco_planner/trajectory/wheel_speed_time_scaling.hpp"
#include "minco_planner/trajectory/yaw_authority_policy.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#define private public
#include "minco_planner/nodes/minco_planner_node.hpp"
#undef private

namespace
{

using minco_planner::MincoPlannerNode;
using PlannerStatus = ats_navigation_interfaces::msg::PlannerStatus;
using Grid = nav_msgs::msg::OccupancyGrid;

constexpr double kResolution = 0.1;
constexpr double kArenaLength = 14.0;    // x in [0, 14]
constexpr double kArenaHalfWidth = 3.0;  // y in [-3, 3]
constexpr double kStartX = 1.5;
constexpr double kGoalX = 12.5;
constexpr double kBlockHalf = 0.15;  // 0.3 m square, far smaller than the 0.68 m footprint
constexpr double kRuntimeHorizonSec = 2.0;
// Reference times (s since commit) where a block is put on the committed reference. The profile
// starts slowly (x = 2.2 m at t = 1.5 s, 7.3 m at t = 5 s), so a block at 1.6 s starts to
// conflict about 1.1 s into the reference, ahead of the robot and inside the runtime window.
// A block at 5.0 s conflicts from about 4.5 s on, far outside it. The slack on both sides
// absorbs the wall-clock time that passes between commit and onGrid.
constexpr double kNearBlockTime = 1.6;
constexpr double kFarBlockTime = 5.0;
constexpr double kHeadTime = 0.3;  // poses up to here are the robot's current position
constexpr std::uint64_t kGoalId = 1;
constexpr char kTopicPrefix[] = "/minco_ongrid_test/";

Grid makeArena()
{
  Grid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = static_cast<float>(kResolution);
  grid.info.width = static_cast<std::uint32_t>(std::lround(kArenaLength / kResolution));
  grid.info.height = static_cast<std::uint32_t>(std::lround(2.0 * kArenaHalfWidth / kResolution));
  grid.info.origin.position.y = -kArenaHalfWidth;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
  for (std::uint32_t j = 0; j < grid.info.height; ++j) {
    for (std::uint32_t i = 0; i < grid.info.width; ++i) {
      const bool wall = i < 2 || j < 2 || i + 2 >= grid.info.width || j + 2 >= grid.info.height;
      if (wall) {
        grid.data[static_cast<std::size_t>(j) * grid.info.width + i] = 100;
      }
    }
  }
  return grid;
}

// Marks every cell whose centre lies in the square [cx +- half] x [cy +- half] as occupied.
void addBlock(Grid & grid, double cx, double cy, double half)
{
  for (std::uint32_t j = 0; j < grid.info.height; ++j) {
    for (std::uint32_t i = 0; i < grid.info.width; ++i) {
      const double x = grid.info.origin.position.x + (i + 0.5) * kResolution;
      const double y = grid.info.origin.position.y + (j + 0.5) * kResolution;
      if (std::abs(x - cx) <= half && std::abs(y - cy) <= half) {
        grid.data[static_cast<std::size_t>(j) * grid.info.width + i] = 100;
      }
    }
  }
}

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

rclcpp::NodeOptions makeNodeOptions(bool retain, bool advisory)
{
  std::map<std::string, std::string> params = {
    // Production profile: GoalManager owns the stop, the planner reports through PlannerStatus.
    {"planner_manages_emergency_stop", "false"},
    // Pin the window semantics so the scene does not depend on parameter defaults.
    {"runtime_safety_horizon_sec", std::to_string(kRuntimeHorizonSec)},
    {"retain_reference_horizon_sec", "0.0"},
    {"reference_window_backward_sec", "0.0"},
    {"reference_window_from_robot_progress", "false"},
    {"escape_from_contact_enabled", "false"},
    {"retain_safe_reference_on_snapshot_change", retain ? "true" : "false"},
    {"replan_advisory_enabled", advisory ? "true" : "false"},
  };
  // Private topic names: the test never touches topics of a running system.
  for (const char * name :
       {"grid_topic", "goal_topic", "planner_status_topic", "candidate_reference_path_topic",
        "raw_path_topic", "preprocessed_guide_topic", "esdf_refined_guide_topic",
        "debug_marker_topic"}) {
    params[name] = std::string(kTopicPrefix) + name;
  }
  std::vector<std::string> args = {"--ros-args", "-r", "__node:=minco_planner_ongrid_test"};
  for (const auto & item : params) {
    args.push_back("-p");
    args.push_back(item.first + ":=" + item.second);
  }
  rclcpp::NodeOptions options;
  options.arguments(args);
  return options;
}

class OnGridBranchTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Builds the node, subscribes a probe to its PlannerStatus topic, then commits a reference
  // exactly like the sandbox CLI: free-space grid -> onGrid -> planGoal.
  void startAndCommit(
    bool retain, bool advisory, const MincoPlannerNode::GoalUuid & goal_uuid = {})
  {
    node_ = std::make_shared<MincoPlannerNode>(makeNodeOptions(retain, advisory));
    probe_ = std::make_shared<rclcpp::Node>("minco_ongrid_probe");
    executor_.add_node(probe_);
    status_sub_ = probe_->create_subscription<PlannerStatus>(
      node_->planner_status_topic_, rclcpp::QoS(50).reliable().transient_local(),
      [this](PlannerStatus::SharedPtr message) { statuses_.push_back(*message); });
    ASSERT_TRUE(
      spinUntil([this] { return node_->planner_status_pub_->get_subscription_count() > 0; }))
      << "probe never matched the planner status publisher";

    node_->onGrid(std::make_shared<Grid>(makeArena()));
    if (goal_uuid.front() != 0) {
      node_->prior_guide_map_ = std::make_shared<const Grid>(makeArena());
      const auto & params = node_->safety_checker_.params();
      node_->prior_blocked_squared_ = minco_planner::computeBlockedSquaredDistanceCells(
        *node_->prior_guide_map_, {params.obstacle_value_threshold, params.unknown_is_obstacle});
    }
    const auto start = makePose(kStartX, 0.0);
    node_->planGoal(
      makePose(kGoalX, 0.0), kGoalId, /*localization_epoch=*/1, /*plan_request_sequence=*/1,
      /*map_publication_sequence=*/1, /*report_status=*/true, &start, nullptr, 0.0, goal_uuid);

    ASSERT_TRUE(node_->active_safety_reference_.has_value()) << "free-space plan was not committed";
    ASSERT_TRUE(node_->safety_state_.plan_safe);
    ASSERT_EQ(node_->latest_map_snapshot_->generation, 1U);
    ASSERT_TRUE(node_->planner_status_state_.latest.has_value());
    ASSERT_EQ(node_->planner_status_state_.latest->state, PlannerStatus::STATE_REFERENCE_READY);
    committed_ = *node_->active_safety_reference_;
    ASSERT_GT(committed_.trajectory.totalTime(), kFarBlockTime + 2.0)
      << "reference too short for the far-block scene";
  }

  // Free-space arena plus a block centred on the committed reference at `time_sec`.
  Grid arenaWithBlockOnReference(double time_sec) const
  {
    Grid grid = makeArena();
    for (const auto & point : committed_.trajectory.points) {
      if (point.t >= time_sec) {
        addBlock(grid, point.x, point.y, kBlockHalf);
        break;
      }
    }
    return grid;
  }

  // A different map that leaves the whole reference corridor free.
  static Grid arenaWithSideBlock()
  {
    Grid grid = makeArena();
    addBlock(grid, 7.0, 2.0, kBlockHalf);
    return grid;
  }

  void feed(const Grid & grid) { node_->onGrid(std::make_shared<Grid>(grid)); }

  // Scene classification with the predicates onGrid itself uses, so a mis-sized block shows up
  // as a bad scene and not as a planner regression.
  bool remainingSafe(const Grid & grid)
  {
    return node_->remainingReferenceSafeOn(committed_.trajectory, grid);
  }
  bool runtimeWindowSafe(const Grid & grid)
  {
    return node_->runtimeWindowSafeOn(committed_.trajectory, grid);
  }
  // The robot's own pose (start of the reference) is free on `grid`: the conflict is ahead of it.
  bool headSafe(const Grid & grid)
  {
    return node_->safety_checker_
      .check(minco_planner::remainingReferenceWindow(committed_.trajectory, 0.0, kHeadTime), grid)
      .safe;
  }

  void expectRetained(std::uint64_t generation)
  {
    ASSERT_TRUE(node_->active_safety_reference_.has_value()) << "reference was dropped";
    const auto & active = *node_->active_safety_reference_;
    EXPECT_EQ(active.map_generation, generation);
    EXPECT_EQ(active.goal_id, committed_.goal_id);
    EXPECT_EQ(active.trajectory.header.stamp.sec, committed_.trajectory.header.stamp.sec);
    EXPECT_EQ(active.trajectory.header.stamp.nanosec, committed_.trajectory.header.stamp.nanosec);
    EXPECT_EQ(active.trajectory.points.size(), committed_.trajectory.points.size());
    EXPECT_TRUE(node_->safety_state_.plan_safe);
    EXPECT_FALSE(node_->safety_state_.emergencyStopRequired());
    EXPECT_TRUE(node_->safety_state_.mapSnapshotUsable(generation));
    ASSERT_NE(node_->latest_map_snapshot_, nullptr);
    EXPECT_EQ(node_->latest_map_snapshot_->generation, generation);
    // Retention publishes no failure: the status identity is still the commit's READY.
    ASSERT_TRUE(node_->planner_status_state_.latest.has_value());
    EXPECT_EQ(node_->planner_status_state_.latest->state, PlannerStatus::STATE_REFERENCE_READY);
    EXPECT_EQ(node_->planner_status_state_.latest->failure_reason, PlannerStatus::FAILURE_NONE);
  }

  void expectInvalidated(std::uint64_t generation)
  {
    EXPECT_FALSE(node_->active_safety_reference_.has_value()) << "stale reference still active";
    EXPECT_FALSE(node_->safety_state_.plan_safe);
    EXPECT_TRUE(node_->safety_state_.emergencyStopRequired());
    // A new grid is still a healthy map: the lease stays, but only a plan bound to the new
    // generation may run.
    EXPECT_TRUE(node_->safety_state_.map_ready);
    EXPECT_EQ(node_->safety_state_.minimum_map_generation, generation);
    EXPECT_TRUE(node_->safety_state_.mapSnapshotUsable(generation));
    EXPECT_FALSE(node_->safety_state_.mapSnapshotUsable(generation - 1));
    ASSERT_NE(node_->latest_map_snapshot_, nullptr);
    EXPECT_EQ(node_->latest_map_snapshot_->generation, generation);
    EXPECT_FALSE(node_->replan_advised_reference_stamp_.has_value());
    ASSERT_TRUE(node_->planner_status_state_.latest.has_value());
    const PlannerStatus & latest = *node_->planner_status_state_.latest;
    EXPECT_EQ(latest.state, PlannerStatus::STATE_FAILED);
    EXPECT_EQ(latest.failure_reason, PlannerStatus::FAILURE_SNAPSHOT_CHANGED);
    EXPECT_EQ(latest.map_generation, generation);
    EXPECT_EQ(latest.goal_id, kGoalId);
  }

  const PlannerStatus * findStatus(std::uint8_t state, std::uint8_t reason) const
  {
    for (const auto & status : statuses_) {
      if (status.state == state && status.failure_reason == reason) {
        return &status;
      }
    }
    return nullptr;
  }

  std::size_t countState(std::uint8_t state) const
  {
    return static_cast<std::size_t>(std::count_if(
      statuses_.begin(), statuses_.end(),
      [state](const PlannerStatus & status) { return status.state == state; }));
  }

  template <typename Predicate>
  bool spinUntil(Predicate done, std::chrono::milliseconds timeout = std::chrono::seconds(3))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      executor_.spin_once(std::chrono::milliseconds(20));
    }
    return true;
  }

  std::shared_ptr<MincoPlannerNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr status_sub_;
  std::vector<PlannerStatus> statuses_;
  MincoPlannerNode::ActiveSafetyReference committed_;
};

// retain: the new map differs but the whole remaining reference is still safe.
TEST_F(OnGridBranchTest, RetainsReferenceWhenRemainingSegmentStaysSafe)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
  const Grid next = arenaWithSideBlock();
  ASSERT_TRUE(remainingSafe(next)) << "bad scene: side block touches the reference";

  feed(next);

  ASSERT_NO_FATAL_FAILURE(expectRetained(2));
  EXPECT_FALSE(node_->replan_advised_reference_stamp_.has_value()) << "no advisory when fully safe";
}

TEST_F(OnGridBranchTest, RetainsRelocatedYawReferenceAcrossHarmlessMapUpdates)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
  minco_planner::ReferenceTrajectory relocated;
  ASSERT_TRUE(minco_planner::relocateTerminalYawRotation(
      committed_.trajectory, 0.2, 0U, {}, &relocated));
  ASSERT_TRUE(node_->remainingReferenceSafeOn(relocated, arenaWithSideBlock()));
  node_->active_safety_reference_->trajectory = relocated;
  committed_ = *node_->active_safety_reference_;

  feed(arenaWithSideBlock());

  ASSERT_NO_FATAL_FAILURE(expectRetained(2));
}

TEST_F(OnGridBranchTest, RejectsSafetyReferenceWithMissingOrDifferentGridFrame)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
  for (const std::string & frame : {std::string(), std::string("other_frame")}) {
    auto reference = committed_.trajectory;
    reference.header.frame_id = frame;
    EXPECT_FALSE(node_->publishReferenceIfCurrent(
        node_->latest_map_snapshot_, node_->map_health_epoch_, node_->toPath(reference),
        reference, kGoalId, 1, 2, 1, PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW, false));
    EXPECT_TRUE(node_->safety_state_.emergencyStopRequired());
  }
}

// advise: obstacle only on the far part of the reference, runtime window still safe.
TEST_F(OnGridBranchTest, AdvisesReplanWhenOnlyFarSegmentConflicts)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
  const Grid next = arenaWithBlockOnReference(kFarBlockTime);
  ASSERT_FALSE(remainingSafe(next)) << "bad scene: far block does not conflict";
  ASSERT_TRUE(runtimeWindowSafe(next)) << "bad scene: far block reaches the runtime window";

  feed(next);

  ASSERT_NO_FATAL_FAILURE(expectRetained(2));
  ASSERT_TRUE(node_->replan_advised_reference_stamp_.has_value());
  EXPECT_EQ(node_->replan_advised_reference_stamp_->sec, committed_.trajectory.header.stamp.sec);
  EXPECT_EQ(
    node_->replan_advised_reference_stamp_->nanosec, committed_.trajectory.header.stamp.nanosec);
  // The advisory goes out directly as ACCEPTED + REPLAN_ADVISED for the running reference.
  ASSERT_TRUE(spinUntil([this] {
    return findStatus(PlannerStatus::STATE_ACCEPTED, PlannerStatus::FAILURE_REPLAN_ADVISED);
  }))
    << "no REPLAN_ADVISED status was published";
  const PlannerStatus * advisory =
    findStatus(PlannerStatus::STATE_ACCEPTED, PlannerStatus::FAILURE_REPLAN_ADVISED);
  EXPECT_EQ(advisory->goal_id, kGoalId);
  EXPECT_EQ(advisory->plan_request_sequence, committed_.plan_request_sequence);
  EXPECT_EQ(advisory->map_generation, 2U);
  EXPECT_EQ(countState(PlannerStatus::STATE_FAILED), 0U);
}

// invalidate: the near window is unsafe, so even with retention and advisory both enabled the
// reference must be dropped. This is the fail-closed path.
TEST_F(OnGridBranchTest, InvalidatesFailClosedWhenNearSegmentIsUnsafe)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
  const Grid next = arenaWithBlockOnReference(kNearBlockTime);
  ASSERT_FALSE(remainingSafe(next)) << "bad scene: near block does not conflict";
  ASSERT_FALSE(runtimeWindowSafe(next)) << "bad scene: near block is outside the runtime window";
  ASSERT_TRUE(headSafe(next)) << "bad scene: near block touches the robot's current pose";

  feed(next);

  ASSERT_NO_FATAL_FAILURE(expectInvalidated(2));
  ASSERT_TRUE(spinUntil([this] {
    return findStatus(PlannerStatus::STATE_FAILED, PlannerStatus::FAILURE_SNAPSHOT_CHANGED);
  }))
    << "no SNAPSHOT_CHANGED failure was published";
  const PlannerStatus * failure =
    findStatus(PlannerStatus::STATE_FAILED, PlannerStatus::FAILURE_SNAPSHOT_CHANGED);
  EXPECT_EQ(failure->goal_id, kGoalId);
  EXPECT_EQ(failure->map_generation, 2U);
  EXPECT_EQ(
    findStatus(PlannerStatus::STATE_ACCEPTED, PlannerStatus::FAILURE_REPLAN_ADVISED), nullptr)
    << "near-window conflict must never be advised";
}

TEST_F(OnGridBranchTest, BlockedPriorRouteKeepsTheStopAfterTheFullSearchFails)
{
  const MincoPlannerNode::GoalUuid goal_uuid{1};
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true, goal_uuid));
  ASSERT_TRUE(node_->global_route_);
  ASSERT_EQ(node_->global_route_->goal_uuid, goal_uuid);

  auto blocked = makeArena();
  const auto column = static_cast<std::size_t>(2.3 / kResolution);
  for (std::size_t row = 0; row < blocked.info.height; ++row) {
    blocked.data[row * blocked.info.width + column] = 100;
  }
  ASSERT_TRUE(headSafe(blocked));
  ASSERT_FALSE(runtimeWindowSafe(blocked));
  feed(blocked);
  ASSERT_NO_FATAL_FAILURE(expectInvalidated(2));

  const auto start = makePose(kStartX, 0.0);
  node_->planGoal(
    makePose(kGoalX, 0.0), kGoalId, /*localization_epoch=*/1, /*plan_request_sequence=*/2,
    /*map_publication_sequence=*/2, /*report_status=*/true, &start, nullptr, 0.0, goal_uuid);

  EXPECT_FALSE(node_->global_route_);
  EXPECT_FALSE(node_->active_safety_reference_);
  EXPECT_TRUE(node_->safety_state_.emergencyStopRequired());
  ASSERT_TRUE(spinUntil([this] {
    return findStatus(PlannerStatus::STATE_FAILED, PlannerStatus::FAILURE_NO_PATH);
  }));
  const auto * failure = findStatus(PlannerStatus::STATE_FAILED, PlannerStatus::FAILURE_NO_PATH);
  EXPECT_EQ(failure->goal_id, kGoalId);
  EXPECT_EQ(failure->localization_epoch, 1U);
  EXPECT_EQ(failure->plan_request_sequence, 2U);
  EXPECT_EQ(failure->map_generation, 2U);
}

// retention disabled: a harmless map change still invalidates the previous reference.
TEST_F(OnGridBranchTest, InvalidatesAnyChangedMapWhenRetentionIsDisabled)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/false, /*advisory=*/true));
  const Grid next = arenaWithSideBlock();
  ASSERT_TRUE(remainingSafe(next)) << "bad scene: side block touches the reference";

  feed(next);

  ASSERT_NO_FATAL_FAILURE(expectInvalidated(2));
}

// advisory disabled (the default): the same far conflict as the advise test invalidates.
TEST_F(OnGridBranchTest, InvalidatesFarConflictWhenAdvisoryIsDisabled)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/false));
  const Grid next = arenaWithBlockOnReference(kFarBlockTime);
  ASSERT_FALSE(remainingSafe(next)) << "bad scene: far block does not conflict";
  ASSERT_TRUE(runtimeWindowSafe(next)) << "bad scene: far block reaches the runtime window";

  feed(next);

  ASSERT_NO_FATAL_FAILURE(expectInvalidated(2));
}

// 无停车重规划的起点（调研报告 A.3）：advisory 之后同目标的新请求，起点取旧参考上
// "车的推进位置 + plan_lead"处，而不是请求冻结的实测位姿；车离旧参考过远时退回实测。
constexpr double kReplanElapsedSec = 2.0;

class ReplanStartTest : public OnGridBranchTest
{
protected:
  // 把旧参考的提交时刻回拨 kReplanElapsedSec（"车已执行 2 s"不依赖墙钟），实测位姿放在
  // 旧参考 t=kReplanElapsedSec 处再横向偏 lateral，然后发同目标的第二次请求。
  void replanFromCar(double lateral)
  {
    ASSERT_NO_FATAL_FAILURE(startAndCommit(/*retain=*/true, /*advisory=*/true));
    feed(arenaWithBlockOnReference(kFarBlockTime));
    ASSERT_TRUE(node_->replan_advised_reference_stamp_.has_value()) << "no advisory was raised";
    const builtin_interfaces::msg::Time backdated =
      node_->now() - rclcpp::Duration::from_seconds(kReplanElapsedSec);
    node_->active_safety_reference_->trajectory.header.stamp = backdated;
    node_->replan_advised_reference_stamp_ = backdated;

    car_ = minco_planner::interpolateReferencePoint(committed_.trajectory, kReplanElapsedSec);
    const auto start = makePose(car_.x, car_.y + lateral);
    minco_planner::InitialKinematicState seed;
    seed.valid = true;
    seed.velocity = Eigen::Vector2d(car_.vx, car_.vy);
    node_->planGoal(
      makePose(kGoalX, 0.0), kGoalId, /*localization_epoch=*/1, /*plan_request_sequence=*/2,
      /*map_publication_sequence=*/2, /*report_status=*/true, &start, &seed);
    done_ = node_->now();

    ASSERT_TRUE(node_->active_safety_reference_.has_value());
    ASSERT_EQ(node_->active_safety_reference_->plan_request_sequence, 2U)
      << "the second request did not commit a new reference";
    replanned_ = *node_->active_safety_reference_;
  }

  // 新参考时间原点比提交完成时刻晚多少秒（>0 表示 t=0 还在未来）。
  double originAheadSec() const
  {
    return (rclcpp::Time(replanned_.trajectory.header.stamp) - done_).seconds();
  }

  minco_planner::ReferencePoint car_;
  MincoPlannerNode::ActiveSafetyReference replanned_;
  rclcpp::Time done_{0, 0, RCL_ROS_TIME};
};
TEST_F(ReplanStartTest, StartsOnCommittedReferenceAheadOfMeasuredPose)
{
  ASSERT_NO_FATAL_FAILURE(replanFromCar(/*lateral=*/0.0));
  const double lead = node_->replan_start_params_.plan_lead_sec;
  const auto expected =
    minco_planner::interpolateReferencePoint(committed_.trajectory, kReplanElapsedSec + lead);
  const auto & first = replanned_.trajectory.points.front();
  ASSERT_GT(expected.x - car_.x, 0.05) << "bad scene: the reference barely moves in plan_lead";

  // 新参考的起点是旧参考 t_now + plan_lead 处，而不是请求冻结的实测位姿。
  EXPECT_NEAR(first.x, expected.x, 0.02) << "measured x=" << car_.x;
  EXPECT_NEAR(first.y, expected.y, 0.02);
  EXPECT_GT(first.x - car_.x, 0.8 * (expected.x - car_.x));
  // 新参考的 t=0 比提交时刻晚 plan_lead（沿用提交处已有的时间重基）。
  EXPECT_GT(originAheadSec(), 0.0);
  EXPECT_LE(originAheadSec(), lead + 1e-9);
}

TEST_F(ReplanStartTest, KeepsMeasuredStartWhenCarIsFarFromCommittedReference)
{
  // 横向偏 1.0 m，超过 deviation_max（0.5 m）：起点仍取实测，时间原点就是提交时刻。
  ASSERT_NO_FATAL_FAILURE(replanFromCar(/*lateral=*/1.0));
  const auto & first = replanned_.trajectory.points.front();
  EXPECT_NEAR(first.x, car_.x, 0.1);
  EXPECT_NEAR(first.y, car_.y + 1.0, 0.1);
  EXPECT_LE(originAheadSec(), 1e-9);
}

geometry_msgs::msg::TransformStamped staticTransform(
  const std::string & parent, const std::string & child, const tf2::Transform & value)
{
  geometry_msgs::msg::TransformStamped result;
  result.header.frame_id = parent;
  result.child_frame_id = child;
  result.transform = tf2::toMsg(value);
  return result;
}

TEST_F(OnGridBranchTest, TiltedGridReferenceStartsAtTheBodyControlHeight)
{
  node_ = std::make_shared<MincoPlannerNode>(makeNodeOptions(true, true));
  node_->global_frame_ = "odom";
  node_->robot_frame_ = "body";
  tf2::Quaternion rotation;
  rotation.setRPY(0.16, -0.07, 0.3);
  const tf2::Transform grid_from_control(rotation, tf2::Vector3(13.4, 2.0, 0.24));
  const tf2::Vector3 actual(0.5, 0.2, 0.18);
  ASSERT_TRUE(node_->tf_buffer_->setTransform(
    staticTransform("map", "odom", grid_from_control), "test", true));
  ASSERT_TRUE(node_->tf_buffer_->setTransform(staticTransform("odom", "body",
    tf2::Transform(tf2::Quaternion::getIdentity(), actual)), "test", true));
  const auto grid_start = grid_from_control * actual;
  nav_msgs::msg::Path input;
  input.header.frame_id = "map";
  input.poses = {makePose(grid_start.x(), grid_start.y()),
    makePose(grid_start.x() + 1.0, grid_start.y() + 2.0)};
  input.poses[1].header.stamp.sec = 9;
  input.poses[1].header.stamp.nanosec = 200000000;
  nav_msgs::msg::Path output;
  geometry_msgs::msg::TransformStamped used;
  ASSERT_TRUE(node_->transformPathToGlobal(input, output, &used));
  ASSERT_EQ(output.poses.size(), input.poses.size());
  EXPECT_NEAR(output.poses.front().pose.position.x, actual.x(), 1e-10);
  EXPECT_NEAR(output.poses.front().pose.position.y, actual.y(), 1e-10);
  for (std::size_t i = 0; i < output.poses.size(); ++i) {
    const auto & p = output.poses[i].pose.position;
    EXPECT_NEAR(p.z, actual.z(), 1e-10);
    const auto recovered = grid_from_control * tf2::Vector3(p.x, p.y, p.z);
    EXPECT_NEAR(recovered.x(), input.poses[i].pose.position.x, 1e-10);
    EXPECT_NEAR(recovered.y(), input.poses[i].pose.position.y, 1e-10);
    EXPECT_EQ(output.poses[i].header.stamp, input.poses[i].header.stamp);
  }
  EXPECT_EQ(used.header.frame_id, "odom");
  EXPECT_EQ(used.child_frame_id, "map");
}

TEST_F(OnGridBranchTest, CrossFrameProjectionRejectsMissingBodyHeight)
{
  node_ = std::make_shared<MincoPlannerNode>(makeNodeOptions(true, true));
  node_->global_frame_ = "odom";
  node_->robot_frame_ = "body";
  ASSERT_TRUE(node_->tf_buffer_->setTransform(staticTransform("map", "odom",
    tf2::Transform::getIdentity()), "test", true));
  nav_msgs::msg::Path input, output;
  input.header.frame_id = "map";
  input.poses = {makePose(1.0, 1.0), makePose(2.0, 1.0)};
  EXPECT_FALSE(node_->transformPathToGlobal(input, output));
}

TEST_F(OnGridBranchTest, CrossFrameProjectionRejectsSingularControlPlane)
{
  node_ = std::make_shared<MincoPlannerNode>(makeNodeOptions(true, true));
  node_->global_frame_ = "odom";
  node_->robot_frame_ = "body";
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, std::acos(-1.0) * 0.5, 0.0);
  ASSERT_TRUE(node_->tf_buffer_->setTransform(staticTransform("map", "odom",
    tf2::Transform(rotation, tf2::Vector3(0.0, 0.0, 0.0))), "test", true));
  ASSERT_TRUE(node_->tf_buffer_->setTransform(staticTransform("odom", "body",
    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.0, 0.18))),
    "test", true));
  nav_msgs::msg::Path input, output;
  input.header.frame_id = "map";
  input.poses = {makePose(1.0, 1.0), makePose(2.0, 1.0)};
  EXPECT_FALSE(node_->transformPathToGlobal(input, output));
}

TEST_F(OnGridBranchTest, RuntimeRecheckPreservesTheCommittedControlPlane)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(true, true));
  node_->global_frame_ = "odom";
  node_->robot_frame_ = "body";
  tf2::Quaternion initial_rotation;
  initial_rotation.setRPY(0.1, -0.06, 0.05);
  const tf2::Transform initial(initial_rotation, tf2::Vector3(0.12, -0.2, 0.15));
  ASSERT_TRUE(node_->tf_buffer_->setTransform(
    staticTransform("map", "odom", initial), "test", true));
  ASSERT_TRUE(node_->tf_buffer_->setTransform(staticTransform("odom", "body",
    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(1.5, 0.0, 0.18))),
    "test", true));
  nav_msgs::msg::Path control_path;
  geometry_msgs::msg::TransformStamped used;
  ASSERT_TRUE(node_->transformPathToGlobal(node_->toPath(committed_.trajectory),
    control_path, &used));
  ASSERT_TRUE(node_->publishReferenceIfCurrent(node_->latest_map_snapshot_,
    node_->map_health_epoch_, control_path, committed_.trajectory, kGoalId, 1, 2, 1,
    PlannerStatus::YAW_AUTHORITY_BODY_YAW_FOLLOW, false, {}, &used));
  tf2::Quaternion corrected_rotation;
  corrected_rotation.setRPY(-0.05, 0.08, -0.03);
  const tf2::Transform corrected(corrected_rotation, tf2::Vector3(0.1, -0.15, 0.14));
  const auto current = staticTransform("map", "odom", corrected);
  ASSERT_TRUE(node_->tf_buffer_->setTransform(current, "test", true));
  const auto aligned = node_->alignedToCurrentTf(*node_->active_safety_reference_);
  ASSERT_EQ(aligned.points.size(), control_path.poses.size());
  for (std::size_t i = 0; i < aligned.points.size(); ++i) {
    geometry_msgs::msg::PoseStamped expected;
    tf2::doTransform(control_path.poses[i], expected, current);
    EXPECT_NEAR(aligned.points[i].x, expected.pose.position.x, 1e-10);
    EXPECT_NEAR(aligned.points[i].y, expected.pose.position.y, 1e-10);
  }
}

TEST_F(OnGridBranchTest, RuntimeRecheckRevokesReferenceWhenItsTransformIsMissing)
{
  ASSERT_NO_FATAL_FAILURE(startAndCommit(true, true));
  auto & active = *node_->active_safety_reference_;
  active.control_from_grid = staticTransform("odom", "map", tf2::Transform::getIdentity());
  active.control_reference = node_->toPath(active.trajectory);
  active.control_reference->header.frame_id = "odom";
  node_->onRuntimeSafetyRecheck();
  EXPECT_FALSE(node_->active_safety_reference_.has_value());
  EXPECT_FALSE(node_->safety_state_.plan_safe);
  ASSERT_TRUE(node_->planner_status_state_.latest.has_value());
  EXPECT_EQ(node_->planner_status_state_.latest->failure_reason,
    PlannerStatus::FAILURE_REFERENCE_TF);
}

TEST_F(OnGridBranchTest, RequestedToleranceAdmitsSafeEndpointWithoutChangingLaterDefaults)
{
  startAndCommit(true, true);
  node_->goal_pose_admission_params_.enabled = true;
  node_->goal_pose_admission_params_.position_tolerance_m = 0.02;
  node_->goal_pose_admission_params_.position_step_m = 0.005;
  node_->goal_pose_admission_params_.yaw_tolerance_rad = 0.0;
  auto grid = makeArena();
  const auto blocked = static_cast<std::size_t>(33) * grid.info.width + 121;
  grid.data[blocked] = 100;
  ASSERT_FALSE(minco_planner::goalPoseFootprintFree(
    kGoalX, 0.0, 0.0, grid, node_->footprint_params_));
  node_->onGrid(std::make_shared<Grid>(grid));
  const auto start = makePose(kStartX, 0.0);
  node_->planGoal(makePose(kGoalX, 0.0), 2, 1, 1, 2, true, &start, nullptr, 0.08);
  ASSERT_EQ(node_->planner_status_state_.latest->state, PlannerStatus::STATE_REFERENCE_READY);
  ASSERT_TRUE(node_->active_safety_reference_);
  const auto & endpoint = node_->active_safety_reference_->trajectory.points.back();
  EXPECT_LE(std::hypot(endpoint.x - kGoalX, endpoint.y), 0.06 + 1e-6);
  EXPECT_TRUE(minco_planner::goalPoseFootprintFree(
    endpoint.x, endpoint.y, endpoint.yaw, grid, node_->footprint_params_));
  EXPECT_DOUBLE_EQ(node_->goal_pose_admission_params_.position_tolerance_m, 0.02);
  EXPECT_EQ(node_->latest_map_snapshot_->grid.data[blocked], 100);
  node_->planGoal(makePose(kGoalX, 0.0), 3, 1, 1, 2, true, &start, nullptr);
  EXPECT_EQ(node_->planner_status_state_.latest->state, PlannerStatus::STATE_FAILED);
  EXPECT_DOUBLE_EQ(node_->goal_pose_admission_params_.position_tolerance_m, 0.02);
}

TEST_F(OnGridBranchTest, InvalidRequestedToleranceReportsTheSameGoalIdentity)
{
  startAndCommit(true, true);
  for (const double tolerance : {-0.01, std::numeric_limits<double>::quiet_NaN()}) {
    auto request = std::make_shared<ats_navigation_interfaces::msg::PlannerGoal>();
    request->goal_id = 99;
    request->localization_epoch = 1;
    request->plan_request_sequence = 3;
    request->position_tolerance = tolerance;
    node_->onPlannerGoal(request);
    ASSERT_TRUE(node_->planner_status_state_.latest);
    EXPECT_EQ(node_->planner_status_state_.latest->goal_id, 99U);
    EXPECT_EQ(node_->planner_status_state_.latest->failure_reason, PlannerStatus::FAILURE_INVALID_GOAL);
  }
}

}  // namespace
