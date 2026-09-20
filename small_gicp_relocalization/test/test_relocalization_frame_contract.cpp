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

  void receiveStatus(std::int64_t stamp_ns, std::uint8_t state)
  {
    auto clock = node_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, stamp_ns), RCL_RET_OK);
    auto status = std::make_shared<ats_navigation_interfaces::msg::LocalizationStatus>();
    status->header.frame_id = "prior_map";
    status->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    status->state = state;
    status->epoch = 1;
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

}  // namespace small_gicp_relocalization
