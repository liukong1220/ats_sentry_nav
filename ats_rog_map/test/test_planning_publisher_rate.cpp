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
#include "ats_rog_map/planning/planning_map_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

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

INSTANTIATE_TEST_SUITE_P(SourceUpdates, PlanningPublisherRate, ::testing::Values(false, true));

}  // namespace
