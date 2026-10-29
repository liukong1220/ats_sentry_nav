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
  void startAndCommit(bool retain, bool advisory)
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
    const auto start = makePose(kStartX, 0.0);
    node_->planGoal(
      makePose(kGoalX, 0.0), kGoalId, /*localization_epoch=*/1, /*plan_request_sequence=*/1,
      /*map_publication_sequence=*/1, /*report_status=*/true, &start, nullptr);

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

}  // namespace
