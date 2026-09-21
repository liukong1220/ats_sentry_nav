// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl/time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationFrameTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    auto pattern = (std::filesystem::temp_directory_path() / "gicp_frames_XXXXXX").string();
    const auto directory = mkdtemp(pattern.data());
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
    prior_path_ = directory_ / "map.pcd";
    for (int x = 1; x <= 4; ++x) {
      for (int y = 0; y < 4; ++y) {
        for (int z = 0; z < 4; ++z) {
          prior_.push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
    ASSERT_EQ(pcl::io::savePCDFileASCII(prior_path_.string(), prior_), 0);
  }

  void TearDown() override
  {
    node_.reset();
    if (!directory_.empty()) {
      std::filesystem::remove_all(directory_);
    }
  }

  rclcpp::NodeOptions options(const std::string & path, const std::string & odom = "odom")
  {
    rclcpp::NodeOptions result;
    result.parameter_overrides({
      rclcpp::Parameter("prior_pcd_file", path),
      rclcpp::Parameter("map_frame", "prior_map"),
      rclcpp::Parameter("odom_frame", odom),
      rclcpp::Parameter("robot_base_frame", "robot_body"),
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("publish_tf", false),
    });
    return result;
  }

  void createNode(const std::string & odom = "odom")
  {
    node_ = std::make_shared<SmallGicpRelocalizationNode>(options(prior_path_.string(), odom));
  }

  void installMechanicalTransform()
  {
    geometry_msgs::msg::TransformStamped extrinsic;
    extrinsic.header.frame_id = "robot_body";
    extrinsic.child_frame_id = "front_mid360";
    extrinsic.transform.translation.x = 7.0;
    extrinsic.transform.translation.y = -3.0;
    extrinsic.transform.translation.z = 1.25;
    extrinsic.transform.rotation.z = 0.6;
    extrinsic.transform.rotation.w = 0.8;
    ASSERT_TRUE(node_->tf_buffer_->setTransform(extrinsic, "frame_contract_test", true));
    const auto stored = node_->tf_buffer_->lookupTransform(
      "robot_body", "front_mid360", tf2::TimePointZero);
    ASSERT_DOUBLE_EQ(stored.transform.translation.x, 7.0);
    ASSERT_DOUBLE_EQ(stored.transform.rotation.z, 0.6);
  }

  void expectPriorUnchanged()
  {
    ASSERT_EQ(node_->global_map_->size(), prior_.size());
    for (std::size_t i = 0; i < prior_.size(); ++i) {
      EXPECT_FLOAT_EQ(node_->global_map_->at(i).x, prior_[i].x);
      EXPECT_FLOAT_EQ(node_->global_map_->at(i).y, prior_[i].y);
      EXPECT_FLOAT_EQ(node_->global_map_->at(i).z, prior_[i].z);
    }
  }

  bool reloadPrior() { return node_->loadGlobalMap(prior_path_.string()); }

  void receiveScan(const std::string & frame)
  {
    auto scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(prior_, *scan);
    scan->header.frame_id = frame;
    scan->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
    node_->registeredPcdCallback(scan);
  }

  void expectScanCounts(std::uint64_t accepted, std::uint64_t dropped)
  {
    EXPECT_EQ(node_->accepted_scan_count_, accepted);
    EXPECT_EQ(node_->dropped_scan_count_, dropped);
    EXPECT_EQ(node_->scan_window_.pointCount(), accepted * prior_.size());
  }

  void createWindowNode(int required = 3, int max_frames = 30, int max_points = 40000)
  {
    auto settings = options(prior_path_.string());
    settings.append_parameter_override("accumulate_frames", required);
    settings.append_parameter_override("max_accumulated_frames", max_frames);
    settings.append_parameter_override("max_accumulated_points", max_points);
    node_ = std::make_shared<SmallGicpRelocalizationNode>(settings);
  }

  void receiveStatus(std::int64_t stamp_ns, std::uint8_t state, std::uint64_t epoch = 1)
  {
    auto clock = node_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, stamp_ns), RCL_RET_OK);
    auto status = std::make_shared<ats_navigation_interfaces::msg::LocalizationStatus>();
    status->header.frame_id = "prior_map";
    status->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    status->state = state;
    status->epoch = epoch;
    node_->localizationStatusCallback(status);
    EXPECT_EQ(node_->localization_state_, state);
  }

  void receiveDenseScan(std::int64_t stamp_ns, std::size_t count = 16400)
  {
    receiveStatus(stamp_ns, ats_navigation_interfaces::msg::LocalizationStatus::STATE_TRACKING);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(count, pcl::PointXYZ(1.0F, 2.0F, 0.5F));
    cloud.width = static_cast<std::uint32_t>(count);
    cloud.height = 1;
    auto scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *scan);
    scan->header.frame_id = "odom";
    scan->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    node_->registeredPcdCallback(scan);
  }

  void expectWindow(std::size_t frames, std::size_t points, std::int64_t first_stamp)
  {
    EXPECT_EQ(node_->scan_window_.frameCount(), frames);
    EXPECT_EQ(node_->scan_window_.pointCount(), points);
    if (frames == 0) {
      EXPECT_FALSE(node_->scan_window_.firstStamp());
      EXPECT_DOUBLE_EQ(node_->accumulatedCloudAgeSeconds(), 0.0);
    } else {
      ASSERT_TRUE(node_->scan_window_.firstStamp());
      EXPECT_EQ(*node_->scan_window_.firstStamp(), first_stamp);
      EXPECT_NEAR(
        node_->accumulatedCloudAgeSeconds(),
        static_cast<double>(node_->last_scan_time_.nanoseconds() - first_stamp) * 1e-9, 1e-12);
    }
  }

  bool shouldRegister() { return node_->shouldRunRegistration(); }
  void clearWindow() { node_->clearAccumulation(); }

  void resetInitialPose(bool install_tf)
  {
    if (install_tf) {
      node_->robot_base_frame_ = "robot_body";
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "robot_body";
      transform.transform.rotation.w = 1.0;
      ASSERT_TRUE(node_->tf_buffer_->setTransform(transform, "window_test", true));
    }
    auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose->pose.pose.orientation.w = 1.0;
    node_->initialPoseCallback(pose);
  }

  void expectTrimCounts(std::uint64_t windows, std::uint64_t points, std::uint64_t frames)
  {
    EXPECT_EQ(node_->trimmed_accumulation_count_, windows);
    EXPECT_EQ(node_->sampled_scan_points_count_, points);
    EXPECT_EQ(node_->evicted_scan_frames_count_, frames);
  }

  using Observation = ats_navigation_interfaces::msg::RelocalizationObservation;
  using Status = ats_navigation_interfaces::msg::LocalizationStatus;

  void createConfirmationNode(int count = 3, bool odometry = true)
  {
    auto settings = options(prior_path_.string());
    settings.append_parameter_override("confirmation_count", count);
    node_ = std::make_shared<SmallGicpRelocalizationNode>(settings);
    node_->register_timer_->cancel();
    node_->transform_timer_->cancel();
    if (odometry) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "robot_body";
      transform.transform.rotation.w = 1.0;
      ASSERT_TRUE(node_->tf_buffer_->setTransform(transform, "confirmation_test", true));
    }
  }

  void observePublications()
  {
    observation_sub_ = node_->create_subscription<Observation>(
      "relocalization_observation", rclcpp::QoS(10).reliable(),
      [this](const Observation::SharedPtr msg) { observations_.push_back(*msg); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (node_->observation_pub_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(node_->observation_pub_->get_subscription_count(), 0U);
  }

  void expectPublication(std::int64_t stamp_ns, std::uint8_t status)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (observations_.empty() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_EQ(observations_.size(), 1U);
    EXPECT_EQ(rclcpp::Time(observations_.front().header.stamp).nanoseconds(), stamp_ns);
    EXPECT_EQ(observations_.front().status, status);
    observations_.clear();
  }

  void attempt(std::int64_t stamp_ns, double x = 0.0, bool ok = true)
  {
    SmallGicpRelocalizationNode::RegistrationAttempt result;
    result.ok = ok;
    result.converged = ok;
    result.registration_error = 0.01;
    result.num_inliers = 900;
    result.information.setIdentity();
    result.transform.translation().x() = x;
    node_->handleRegistrationAttempt(result, rclcpp::Time(stamp_ns, RCL_ROS_TIME), 1000);
  }

  void drainCapturedResult(std::int64_t stamp_ns, bool ok, std::uint64_t generation)
  {
    node_->last_scan_time_ = rclcpp::Time(stamp_ns + 500000000LL, RCL_ROS_TIME);
    node_->has_received_scan_ = true;
    node_->async_result_ = SmallGicpRelocalizationNode::MultiGuessOutcome{};
    node_->async_result_.generation = generation;
    node_->async_result_.best.ok = ok;
    node_->async_result_.best.registration_error = 0.01;
    node_->async_result_.best.num_inliers = 900;
    node_->async_result_.best.information.setIdentity();
    node_->async_result_scan_time_ = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    node_->async_result_source_points_ = 1000;
    node_->async_result_ready_ = true;
    node_->drainAsyncMultiGuessResult();
  }

  std::uint64_t generation() const { return node_->relocalization_generation_.current(); }
  int pendingCount() const { return node_->pending_confirmation_count_; }
  bool accepted() const { return node_->has_accepted_alignment_; }
  double previousResultX() const { return node_->previous_result_t_.translation().x(); }
  bool latticeAllowed() const { return node_->preferMultiGuess(); }
  std::uint64_t sequence() const { return node_->observation_sequence_; }
  std::uint64_t staleResults() const { return node_->stale_async_result_count_; }
  std::optional<std::chrono::steady_clock::time_point> episodeStart() const
  {
    return node_->confirmation_started_at_;
  }
  void ageEpisode()
  {
    ASSERT_TRUE(node_->confirmation_started_at_);
    *node_->confirmation_started_at_ -= std::chrono::seconds(11);
  }
  void runRegistrationTimer() { node_->performRegistration(); }
  void removeOdometry() { node_->robot_base_frame_ = "unavailable_robot_body"; }
  void setColdStartPrior(double max_xy_m, double max_yaw_rad)
  {
    node_->cold_start_prior_max_xy_m_ = max_xy_m;
    node_->cold_start_prior_max_yaw_rad_ = max_yaw_rad;
  }

  void expectAnchor(double first, double counted)
  {
    ASSERT_TRUE(node_->pending_confirmation_);
    EXPECT_DOUBLE_EQ(node_->pending_confirmation_->scan_time_s, first);
    EXPECT_DOUBLE_EQ(node_->pending_confirmation_->last_counted_scan_time_s, counted);
  }

  rclcpp::Subscription<Observation>::SharedPtr observation_sub_;
  std::vector<Observation> observations_;
  std::filesystem::path directory_;
  std::filesystem::path prior_path_;
  pcl::PointCloud<pcl::PointXYZ> prior_;
  std::shared_ptr<SmallGicpRelocalizationNode> node_;
};

TEST_F(SmallGicpRelocalizationFrameTest, MechanicalExtrinsicCannotShiftMapPrior)
{
  createNode();  // No body-to-lidar TF or advancing ROS clock is needed at startup.
  EXPECT_FALSE(node_->has_parameter("base_frame"));
  EXPECT_FALSE(node_->has_parameter("lidar_frame"));
  expectPriorUnchanged();
  installMechanicalTransform();
  ASSERT_TRUE(reloadPrior());
  expectPriorUnchanged();
}

TEST_F(SmallGicpRelocalizationFrameTest, RejectsPhysicalLidarFrameAndAcceptsOdomCoordinates)
{
  createNode();
  installMechanicalTransform();
  receiveScan("front_mid360");
  expectScanCounts(0, 1);
  // Same stamp: the rejected frame must not poison the accepted timestamp gate.
  receiveScan("odom");
  expectScanCounts(1, 1);
}

TEST_F(SmallGicpRelocalizationFrameTest, ConfiguredOdomFrameIsAnExactContract)
{
  createNode("registered_odom");
  receiveScan("odom");
  expectScanCounts(0, 1);
  receiveScan("registered_odom");
  expectScanCounts(1, 1);
}

TEST_F(SmallGicpRelocalizationFrameTest, EmptyAndUnreadablePriorsFailStartup)
{
  const auto empty_path = directory_ / "empty.pcd";
  {
    std::ofstream empty(empty_path);
    empty << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
          << "COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n";
  }
  EXPECT_THROW(
    std::make_shared<SmallGicpRelocalizationNode>(options(empty_path.string())),
    std::runtime_error);
  EXPECT_THROW(
    std::make_shared<SmallGicpRelocalizationNode>(options((directory_ / "missing.pcd").string())),
    std::runtime_error);
  EXPECT_THROW(std::make_shared<SmallGicpRelocalizationNode>(options("")), std::runtime_error);
}

TEST_F(SmallGicpRelocalizationFrameTest, Dense20HzCallbackReachesUnchangedMinimumSpanGate)
{
  createWindowNode();
  for (int frame = 1; frame <= 60; ++frame) {
    const auto stamp = 1000000000LL + (frame - 1) * 50000000LL;
    receiveDenseScan(stamp);
    const auto first_stamp = 1000000000LL + std::max(0, frame - 30) * 50000000LL;
    expectWindow(std::min(frame, 30), std::min(frame, 30) * 1333U, first_stamp);
    // Seven 20 Hz frames span 0.30 s. Neither minimum frames nor age is weakened.
    EXPECT_EQ(shouldRegister(), frame >= 7);
  }
  expectTrimCounts(60, 60 * 15067, 30);
  clearWindow();
  expectWindow(0, 0, 0);
  EXPECT_FALSE(shouldRegister());
  receiveDenseScan(4000000000LL);
  expectWindow(1, 1333, 4000000000LL);
  EXPECT_FALSE(shouldRegister());
}

TEST_F(SmallGicpRelocalizationFrameTest, OversizedCallbackCannotExceedPointCap)
{
  createWindowNode();
  receiveDenseScan(1000000000LL, 100001);
  expectWindow(1, 1333, 1000000000LL);
  expectTrimCounts(1, 98668, 0);
}

TEST_F(SmallGicpRelocalizationFrameTest, InitialPoseAndRecoveryClearAllRetainedScans)
{
  createWindowNode();
  receiveDenseScan(1000000000LL);
  resetInitialPose(false);  // Even a missing TF must invalidate pre-reset scans.
  expectWindow(0, 0, 0);
  receiveDenseScan(2000000000LL);
  resetInitialPose(true);
  expectWindow(0, 0, 0);
  receiveDenseScan(3000000000LL);
  receiveStatus(3050000000LL, ats_navigation_interfaces::msg::LocalizationStatus::STATE_LOST);
  expectWindow(0, 0, 0);
  receiveDenseScan(4000000000LL);
  expectWindow(1, 1333, 4000000000LL);
  EXPECT_FALSE(shouldRegister());
}

TEST_F(SmallGicpRelocalizationFrameTest, ImpossibleAccumulationParametersFailStartup)
{
  EXPECT_THROW(createWindowNode(3, 2, 40000), std::invalid_argument);
  EXPECT_THROW(createWindowNode(3, 30, 2), std::invalid_argument);
  EXPECT_THROW(createWindowNode(3, 30, 20), std::invalid_argument);
  EXPECT_THROW(createWindowNode(0, 30, 40000), std::invalid_argument);
  EXPECT_THROW(createWindowNode(3, 0, 40000), std::invalid_argument);
  EXPECT_THROW(createWindowNode(3, 30, 0), std::invalid_argument);
}

TEST_F(SmallGicpRelocalizationFrameTest, AsyncObservationsKeepCapturedScanStampOnEveryOutcome)
{
  createConfirmationNode(2);
  observePublications();
  drainCapturedResult(1000000000LL, true, generation());
  expectPublication(1000000000LL, Observation::STATUS_PENDING_CONFIRMATION);
  drainCapturedResult(1200000000LL, true, generation());
  expectPublication(1200000000LL, Observation::STATUS_ACCEPTED);
  drainCapturedResult(1400000000LL, false, generation());
  expectPublication(1400000000LL, Observation::STATUS_REJECTED);
  removeOdometry();
  drainCapturedResult(1600000000LL, true, generation());
  expectPublication(1600000000LL, Observation::STATUS_NO_ODOM);
}

TEST_F(SmallGicpRelocalizationFrameTest, ThirdConfirmationCannotCountSecondScanTwice)
{
  createConfirmationNode();
  attempt(1000000000LL);
  const auto started = episodeStart();
  attempt(1200000000LL, 0.10);
  EXPECT_EQ(pendingCount(), 2);
  expectAnchor(1.0, 1.2);
  attempt(1200000000LL, 0.10);
  attempt(1220000000LL, 0.10);
  EXPECT_EQ(pendingCount(), 2);
  EXPECT_FALSE(accepted());
  EXPECT_EQ(episodeStart(), started);
  attempt(1400000000LL, 0.10);
  EXPECT_TRUE(accepted());
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
}

TEST_F(SmallGicpRelocalizationFrameTest, ConfirmationRetainsFirstGeometryAndEpisodeDeadline)
{
  createConfirmationNode();
  attempt(1000000000LL);
  const auto started = episodeStart();
  attempt(1200000000LL, 0.10);
  attempt(1400000000LL, 0.20);  // Close to second, but not to the first anchor.
  EXPECT_FALSE(accepted());
  EXPECT_EQ(pendingCount(), 1);
  expectAnchor(1.4, 1.4);
  EXPECT_EQ(episodeStart(), started);  // Geometry restart is not a fresh episode.
}

TEST_F(SmallGicpRelocalizationFrameTest, ColdLostMismatchRetainsLatticeOrigin)
{
  createConfirmationNode(2);
  receiveStatus(1000000000LL, Status::STATE_LOST);
  attempt(1000000000LL);
  EXPECT_EQ(pendingCount(), 1);
  expectAnchor(1.0, 1.0);
  observePublications();
  // Far from the first pending anchor while LOST and never accepted: must not
  // adopt the mismatch as the new pending seed (that recenters multi_guess).
  attempt(1200000000LL, 0.50);
  expectPublication(1200000000LL, Observation::STATUS_REJECTED);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
  EXPECT_FALSE(accepted());
}

TEST_F(SmallGicpRelocalizationFrameTest, ColdBootstrapMismatchRetainsLatticeOrigin)
{
  createConfirmationNode(2);
  // BOOTSTRAP does not open preferMultiGuess(), but a never-accepted node must
  // still refuse to adopt a mismatch as the pending seed.
  receiveStatus(1000000000LL, Status::STATE_BOOTSTRAP);
  attempt(1000000000LL);
  EXPECT_EQ(pendingCount(), 1);
  expectAnchor(1.0, 1.0);
  observePublications();
  attempt(1200000000LL, 0.50);
  expectPublication(1200000000LL, Observation::STATUS_REJECTED);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
  EXPECT_FALSE(accepted());
}

TEST_F(SmallGicpRelocalizationFrameTest, PostAcceptLostMismatchRetainsLatticeOrigin)
{
  createConfirmationNode(2);
  // Earn an accepted alignment first (TRACKING-style confirmation adopts).
  attempt(1000000000LL);
  attempt(1200000000LL);
  EXPECT_TRUE(accepted());
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_DOUBLE_EQ(previousResultX(), 0.0);

  // odometry_stale LOST must not let a mismatched multi_guess candidate
  // rewrite pending and recenter the lattice seed away from last accept.
  receiveStatus(1400000000LL, Status::STATE_LOST);
  attempt(1400000000LL);
  EXPECT_EQ(pendingCount(), 1);
  expectAnchor(1.4, 1.4);
  observePublications();
  attempt(1600000000LL, 0.50);
  expectPublication(1600000000LL, Observation::STATUS_REJECTED);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
  EXPECT_TRUE(accepted());
  EXPECT_DOUBLE_EQ(previousResultX(), 0.0);
}

TEST_F(SmallGicpRelocalizationFrameTest, ColdStartPriorRejectsFarHypothesisBeforePending)
{
  // straight189 accepted a yaw~-2.3 local minimum before first STATUS_ACCEPTED,
  // poisoning the planning grid. Cold-start hypotheses must stay in the
  // init_pose basin until the first accept.
  createConfirmationNode(2);
  setColdStartPrior(1.0, 0.60);
  observePublications();
  attempt(1000000000LL, 1.50);
  expectPublication(1000000000LL, Observation::STATUS_REJECTED);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(accepted());
  EXPECT_FALSE(episodeStart());

  attempt(1200000000LL, 0.20);
  EXPECT_EQ(pendingCount(), 1);
  EXPECT_FALSE(accepted());
}

TEST_F(SmallGicpRelocalizationFrameTest, PendingOwnsRegistrationPathAfterLeavingLost)
{
  // straight190: pending opened under LOST, fusion moved to RELOCALIZING,
  // preferMultiGuess went false, fine_only reject wiped pending.
  createConfirmationNode(2);
  receiveStatus(1000000000LL, Status::STATE_LOST);
  attempt(1000000000LL);
  EXPECT_EQ(pendingCount(), 1);
  EXPECT_TRUE(latticeAllowed());

  receiveStatus(1100000000LL, Status::STATE_RELOCALIZING);
  EXPECT_TRUE(latticeAllowed());

  observePublications();
  attempt(1200000000LL, 0.0, false);
  expectPublication(1200000000LL, Observation::STATUS_REJECTED);
  EXPECT_EQ(pendingCount(), 1);
  EXPECT_TRUE(episodeStart());
  EXPECT_FALSE(accepted());
  EXPECT_TRUE(latticeAllowed());
}








TEST_F(SmallGicpRelocalizationFrameTest, SteadyTimeoutRejectsConfirmationWithPausedRosClock)
{
  createConfirmationNode(2);
  receiveStatus(1000000000LL, Status::STATE_LOST);
  attempt(1000000000LL);
  receiveStatus(1100000000LL, Status::STATE_RELOCALIZING);
  const auto before = generation();
  ageEpisode();  // Advance only the stored steady anchor, not ROS time.
  observePublications();
  attempt(1200000000LL);
  expectPublication(1200000000LL, Observation::STATUS_REJECTED);
  EXPECT_GT(generation(), before);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
  EXPECT_FALSE(accepted());
  attempt(1400000000LL);
  EXPECT_EQ(pendingCount(), 1);
  EXPECT_FALSE(accepted());
}

TEST_F(SmallGicpRelocalizationFrameTest, TimerTimeoutFencesLateWorkerBeforeObservationMutation)
{
  createConfirmationNode();
  attempt(1000000000LL);
  const auto old_generation = generation();
  const auto old_sequence = sequence();
  ageEpisode();
  runRegistrationTimer();
  EXPECT_GT(generation(), old_generation);
  EXPECT_EQ(pendingCount(), 0);
  drainCapturedResult(1200000000LL, true, old_generation);
  EXPECT_EQ(sequence(), old_sequence);
  EXPECT_EQ(staleResults(), 1U);
  EXPECT_FALSE(accepted());
}

TEST_F(SmallGicpRelocalizationFrameTest, MissingOdometryAndFailedInitialPoseClearPendingEpisode)
{
  createConfirmationNode();
  attempt(1000000000LL);
  removeOdometry();
  attempt(1200000000LL);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
  resetInitialPose(true);
  attempt(1400000000LL);
  EXPECT_EQ(pendingCount(), 1);
  removeOdometry();
  resetInitialPose(false);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_FALSE(episodeStart());
}

TEST_F(SmallGicpRelocalizationFrameTest, RecoveryAndEpochBoundariesFenceWorkersWithoutResetLoops)
{
  createConfirmationNode();
  receiveStatus(1000000000LL, Status::STATE_BOOTSTRAP, 0);
  attempt(1000000000LL);
  auto before = generation();
  receiveStatus(1100000000LL, Status::STATE_LOST, 0);
  EXPECT_GT(generation(), before);
  EXPECT_EQ(pendingCount(), 0);
  EXPECT_TRUE(latticeAllowed());
  attempt(1200000000LL);
  before = generation();
  const auto started = episodeStart();
  receiveStatus(1300000000LL, Status::STATE_RELOCALIZING, 0);
  receiveStatus(1400000000LL, Status::STATE_RELOCALIZING, 0);
  EXPECT_EQ(generation(), before);
  EXPECT_EQ(pendingCount(), 1);
  EXPECT_EQ(episodeStart(), started);
  EXPECT_TRUE(latticeAllowed());  // LOST origin survives the pending wire state.
  receiveStatus(1500000000LL, Status::STATE_LOST, 0);
  EXPECT_GT(generation(), before);
  EXPECT_EQ(pendingCount(), 0);
  const auto old_sequence = sequence();
  drainCapturedResult(1450000000LL, true, before);
  EXPECT_EQ(sequence(), old_sequence);
  EXPECT_EQ(staleResults(), 1U);
  receiveStatus(1600000000LL, Status::STATE_DEGRADED, 0);
  EXPECT_FALSE(latticeAllowed());
  attempt(1700000000LL);
  before = generation();
  receiveStatus(1800000000LL, Status::STATE_RELOCALIZING, 0);
  EXPECT_EQ(generation(), before);
  EXPECT_FALSE(latticeAllowed());
  receiveStatus(1900000000LL, Status::STATE_CONFIRMED, 1);
  EXPECT_GT(generation(), before);
  EXPECT_EQ(pendingCount(), 0);
  before = generation();
  receiveStatus(2000000000LL, Status::STATE_CONFIRMED, 1);
  receiveStatus(2100000000LL, Status::STATE_TRACKING, 1);
  EXPECT_EQ(generation(), before);
  attempt(2200000000LL);
  receiveStatus(2300000000LL, Status::STATE_TRACKING, 2);
  EXPECT_GT(generation(), before);
  EXPECT_EQ(pendingCount(), 0);
  const auto sequence_after_epoch = sequence();
  drainCapturedResult(2250000000LL, true, before);
  EXPECT_EQ(sequence(), sequence_after_epoch);
  EXPECT_EQ(staleResults(), 2U);
}

TEST_F(SmallGicpRelocalizationFrameTest, ConfirmationTimeoutMustBeFiniteAndPositive)
{
  for (const double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()}) {
    auto settings = options(prior_path_.string());
    settings.append_parameter_override("confirmation_timeout_s", timeout);
    EXPECT_THROW(std::make_shared<SmallGicpRelocalizationNode>(settings), std::invalid_argument);
  }
}

}  // namespace small_gicp_relocalization
