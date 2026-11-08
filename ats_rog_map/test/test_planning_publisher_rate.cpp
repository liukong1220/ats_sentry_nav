// Copyright 2026
//
// Mapping's fast numerical source must not inherit the 0.5 Hz competition
// cadence: either the mapping timer or the odom-layer update signal drives it.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "ats_rog_map/planning/planning_map_publisher.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using ats_navigation_interfaces::msg::LocalizationStatus;

class PlanningPublisherRate : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_P(PlanningPublisherRate, MappingUsesItsOwnProjectionCadence)
{
  const bool source_updates = GetParam();
  auto driver = std::make_shared<rclcpp::Node>("s3_mapping_rate_driver");
  std::atomic<int> received{0};
  auto clock = driver->get_clock();

  auto provider = [&received, clock](
    const ats_rog_map::GroundProjectionRequest &, ats_rog_map::GroundProjection & response) {
      const int count = ++received;
      response.ready = true;
      response.stale = false;
      response.odom_layer_included = true;
      response.odom_instance_id = 101;
      response.odom_epoch = 1;
      response.generation = static_cast<std::uint64_t>(count);
      auto & grid = response.occupancy_grid;
      grid.header.frame_id = "odom";
      grid.header.stamp = clock->now();
      grid.info.resolution = 0.1F;
      grid.info.width = 10;
      grid.info.height = 10;
      grid.info.origin.orientation.w = 1.0;
      grid.data.assign(100, 0);
      response.signed_distance.assign(100, 1.0F);
      response.gradient_x.assign(100, 0.0F);
      response.gradient_y.assign(100, 0.0F);
    };

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"mapping_mode", true},
    {"projection_rate_hz", 0.5},
    {"mapping_projection_rate_hz", source_updates ? 0.5 : 40.0},
    {"allow_missing_static_map", true},
    {"require_terrain_inputs", false},
  });
  auto publisher = ats_rog_map::planning::makePlanningMapPublisher(provider, options);

  auto status_pub = driver->create_publisher<LocalizationStatus>(
    "/localization/status", rclcpp::QoS(1).transient_local());
  auto update_pub = driver->create_publisher<std_msgs::msg::UInt64>(
    "/rog_map/odom_layer_updated", rclcpp::QoS(1).reliable());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(driver);
  executor.add_node(publisher);
  std::thread spinner([&executor]() {executor.spin();});

  const auto start = std::chrono::steady_clock::now();
  auto last_update = start - std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(4)) {
    LocalizationStatus status;
    status.header.stamp = driver->now();
    status.state = LocalizationStatus::STATE_BOOTSTRAP;
    status.epoch = 1;
    status.odom_instance_id = 101;
    status.odom_epoch = 1;
    status.odom_epoch_start_stamp = status.header.stamp;
    status_pub->publish(status);
    const auto now = std::chrono::steady_clock::now();
    if (source_updates && now - last_update >= std::chrono::milliseconds(50)) {
      std_msgs::msg::UInt64 update;
      update.data = static_cast<std::uint64_t>(received.load() + 1);
      update_pub->publish(update);
      last_update = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  executor.cancel();
  spinner.join();
  publisher.reset();

  const int minimum = source_updates ? 40 : 100;
  EXPECT_GE(received.load(), minimum) << "mapping still uses the slow projection cadence";
}

TEST_P(PlanningPublisherRate, RectangleMissingTfDoesNotSuppressPlanningSnapshot)
{
  const bool have_tf = GetParam();
  auto driver = std::make_shared<rclcpp::Node>("rectangle_tf_driver");
  auto clock = driver->get_clock();
  std::atomic<int> calls{0};
  auto provider = [clock, &calls](
    const ats_rog_map::GroundProjectionRequest &, ats_rog_map::GroundProjection & response) {
      response.ready = true;
      response.stale = false;
      response.odom_layer_included = true;
      response.odom_instance_id = 101;
      response.odom_epoch = 1;
      response.generation = ++calls;
      auto & grid = response.occupancy_grid;
      grid.header.frame_id = "odom";
      grid.header.stamp = clock->now();
      grid.info.resolution = 1.0F;
      grid.info.width = grid.info.height = 4;
      grid.info.origin.orientation.w = 1.0;
      grid.data.assign(16, 0);
      response.signed_distance.assign(16, 1.0F);
      response.gradient_x.assign(16, 0.0F);
      response.gradient_y.assign(16, 0.0F);
    };
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"mapping_mode", true}, {"mapping_projection_rate_hz", 10.0},
    {"planning_resolution", 1.0}, {"allow_missing_static_map", true},
    {"require_terrain_inputs", true}, {"diagnostic_allow_terrain_free_override", true},
    {"terrain_free_override_frame", "rectangle_map"},
    {"terrain_free_override_rects", std::vector<double>{4.77, 10.43, 4.97, 10.63}},
  });
  auto publisher = ats_rog_map::planning::makePlanningMapPublisher(provider, options);
  auto terrain_pub = driver->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/traversability_grid", 1);
  auto slope_pub = driver->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/traversability_slope_grid", 1);
  auto status_pub = driver->create_publisher<LocalizationStatus>(
    "/localization/status", rclcpp::QoS(1).transient_local());
  std::atomic<int> ready_count{0};
  std::atomic<int> wrong_count{0};
  auto subscription = driver->create_subscription<
    ats_navigation_interfaces::msg::PlanningMapSnapshot>(
    "/rog_map_adapter/planning_snapshot", rclcpp::QoS(1).reliable().transient_local(),
    [&](const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot) {
      if (!snapshot.ready) {return;}
      ++ready_count;
      if (snapshot.occupancy.size() != 16 ||
        snapshot.occupancy[5] != (have_tf ? 0 : 100)) {++wrong_count;}
    });
  tf2_ros::StaticTransformBroadcaster broadcaster(driver);
  if (have_tf) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "rectangle_map";
    transform.child_frame_id = "odom";
    transform.transform.translation.x = 3.37;
    transform.transform.translation.y = 9.03;
    transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(transform);
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(driver);
  executor.add_node(publisher);
  std::thread spinner([&executor]() {executor.spin();});
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "odom";
    grid.header.stamp = driver->now();
    grid.info.resolution = 1.0F;
    grid.info.width = grid.info.height = 4;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(16, 0);
    slope_pub->publish(grid);
    grid.data[5] = 100;
    terrain_pub->publish(grid);
    LocalizationStatus status;
    status.header.stamp = driver->now();
    status.state = LocalizationStatus::STATE_BOOTSTRAP;
    status.epoch = 1;
    status.odom_instance_id = 101;
    status.odom_epoch = 1;
    status.odom_epoch_start_stamp = status.header.stamp;
    status_pub->publish(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  executor.cancel();
  spinner.join();
  publisher.reset();
  EXPECT_GE(ready_count.load(), 10);
  EXPECT_EQ(wrong_count.load(), 0);
}

INSTANTIATE_TEST_SUITE_P(SourceUpdates, PlanningPublisherRate, ::testing::Values(false, true));

}  // namespace
