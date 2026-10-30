// 静态外参缓存的行为测试：TF 到位之前不发布 /odometry，首次查询成功后缓存，
// 之后即使 /tf_static 里的外参被改写，输出仍沿用缓存值。
// 参数化为 intra-process 关/开，开时同时验证 message_filters 订阅能走 intra-process。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_scan_generation/sensor_scan_generation.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using namespace std::chrono_literals;  // NOLINT

// TF 未就绪时 odometry 回调要阻塞 0.5 s，同一个 callback group 里排在后面的点云订阅会被饿住，
// 同步回调（以及 /sensor_scan）不一定在窗口内触发。所以不拿 /sensor_scan 当"回调已跑完"的信号，
// 而是数 sensor_scan_generation 的 "required lidar->robot_base TF is unavailable" 这条日志。
std::atomic<int> g_skip_log_count{0};
rcutils_logging_output_handler_t g_previous_log_handler = nullptr;

void countingLogHandler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (name != nullptr && std::strcmp(name, "sensor_scan_generation") == 0) {
    char buffer[256];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    if (std::strstr(buffer, "required lidar->robot_base TF is unavailable") != nullptr) {
      ++g_skip_log_count;
    }
  }
  if (g_previous_log_handler != nullptr) {
    g_previous_log_handler(location, severity, name, timestamp, format, args);
  }
}

class SensorScanGenerationTest : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    const bool intra_process = GetParam();

    g_skip_log_count = 0;
    g_previous_log_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(countingLogHandler);

    rclcpp::NodeOptions ssg_options;
    ssg_options.use_intra_process_comms(intra_process);
    ssg_options.parameter_overrides(
      {rclcpp::Parameter("lidar_frame", "front_mid360"),
       rclcpp::Parameter("robot_base_frame", "gimbal_yaw_odom"),
       rclcpp::Parameter("base_frame", "")});
    ssg_ = std::make_shared<sensor_scan_generation::SensorScanGenerationNode>(ssg_options);

    rclcpp::NodeOptions harness_options;
    harness_options.use_intra_process_comms(intra_process);
    harness_ = std::make_shared<rclcpp::Node>("ssg_test_harness", harness_options);

    // 与 loam_interface 的发布端一致：reliable、keep_last(5)、volatile，
    // 对面是 sensor_scan_generation 的 best_effort、keep_last(1) 订阅。
    odom_pub_ = harness_->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);
    cloud_pub_ = harness_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
    odometry_sub_ = harness_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odometry_msgs_.push_back(*msg);
      });
    scan_sub_ = harness_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "sensor_scan", 10, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++scan_count_; });

    // /tf_static 是 transient_local，Humble 不允许它走 intra-process，
    // 所以 TF 收发放在单独的节点上。
    tf_node_ = std::make_shared<rclcpp::Node>("ssg_test_tf");
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(tf_node_);
    probe_buffer_ = std::make_unique<tf2_ros::Buffer>(tf_node_->get_clock());
    probe_listener_ = std::make_unique<tf2_ros::TransformListener>(*probe_buffer_, tf_node_, false);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ssg_);
    executor_->add_node(harness_);
    executor_->add_node(tf_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    spin_thread_.join();
    probe_listener_.reset();
    probe_buffer_.reset();
    static_broadcaster_.reset();
    rcutils_logging_set_output_handler(g_previous_log_handler);
  }

  // gimbal_yaw_odom -> front_mid360，与 bringup 静态 TF 同方向。
  void publishStaticExtrinsic(double x)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tf_node_->now();
    tf.header.frame_id = "gimbal_yaw_odom";
    tf.child_frame_id = "front_mid360";
    tf.transform.translation.x = x;
    tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(tf);
  }

  // 等测试自己的 buffer 看到新外参，再留一点余量给 sensor_scan_generation 的监听线程。
  bool waitForExtrinsicInProbe(double x)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        const auto tf =
          probe_buffer_->lookupTransform("gimbal_yaw_odom", "front_mid360", tf2::TimePointZero);
        if (std::abs(tf.transform.translation.x - x) < 1e-9) {
          std::this_thread::sleep_for(300ms);
          return true;
        }
      } catch (const tf2::TransformException &) {
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  // 两条消息用同一个时间戳，odom 位姿取单位阵。
  void sendFrame()
  {
    const rclcpp::Time stamp = harness_->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "front_mid360";
    odom.pose.pose.orientation.w = 1.0;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(2);
    cloud_pub_->publish(std::make_unique<sensor_msgs::msg::PointCloud2>(std::move(cloud)));
    odom_pub_->publish(std::make_unique<nav_msgs::msg::Odometry>(std::move(odom)));
  }

  // ApproximateTime 可能把最后一对压到下一帧才放出，所以边发边等。
  bool sendFramesUntil(const std::function<bool()> & done, std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return true;
      }
      sendFrame();
      std::this_thread::sleep_for(100ms);
    }
    return done();
  }

  std::size_t odometryCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return odometry_msgs_.size();
  }

  std::vector<nav_msgs::msg::Odometry> odometrySince(std::size_t first)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<nav_msgs::msg::Odometry>(
      odometry_msgs_.begin() + first, odometry_msgs_.end());
  }

  std::shared_ptr<sensor_scan_generation::SensorScanGenerationNode> ssg_;
  std::shared_ptr<rclcpp::Node> harness_;
  std::shared_ptr<rclcpp::Node> tf_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> probe_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> probe_listener_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  std::vector<nav_msgs::msg::Odometry> odometry_msgs_;
  std::atomic<std::size_t> scan_count_{0};
};

TEST_P(SensorScanGenerationTest, ExtrinsicIsCachedAfterFirstSuccess)
{
  if (GetParam()) {
    // intra-process 开启时，message_filters 的两个订阅都应在 intra-process 管理器里登记。
    EXPECT_EQ(odom_pub_->get_intra_process_subscription_count(), 1u);
    EXPECT_EQ(cloud_pub_->get_intra_process_subscription_count(), 1u);
  }

  // 1. 外参不存在：不得发布 /odometry。等到"外参不可用"那条日志出现，说明回调已走过该分支。
  ASSERT_TRUE(sendFramesUntil([]() { return g_skip_log_count >= 1; }, 10s));
  EXPECT_EQ(odometryCount(), 0u);

  // 2. 外参出现（gimbal_yaw_odom -> front_mid360 = +0.25 m，查询方向取逆，得到 -0.25）。
  publishStaticExtrinsic(0.25);
  ASSERT_TRUE(sendFramesUntil([this]() { return odometryCount() >= 1; }, 10s));
  {
    const auto first = odometrySince(0).front();
    EXPECT_EQ(first.header.frame_id, "odom");
    EXPECT_EQ(first.child_frame_id, "gimbal_yaw_odom");
    EXPECT_NEAR(first.pose.pose.position.x, -0.25, 1e-6);
  }

  // 3. 外参被改写成 +0.50 m：缓存生效时，之后的输出仍是 -0.25。
  publishStaticExtrinsic(0.50);
  ASSERT_TRUE(waitForExtrinsicInProbe(0.50));
  const std::size_t before = odometryCount();
  ASSERT_TRUE(sendFramesUntil([this, before]() { return odometryCount() >= before + 3; }, 10s));
  for (const auto & msg : odometrySince(before)) {
    EXPECT_NEAR(msg.pose.pose.position.x, -0.25, 1e-6);
  }

  // 4. 缓存之后回调不再阻塞，点云订阅不会被饿住，同步回调应当发布 /sensor_scan。
  EXPECT_TRUE(sendFramesUntil([this]() { return scan_count_ >= 1; }, 10s));
}

INSTANTIATE_TEST_SUITE_P(
  IntraProcess, SensorScanGenerationTest, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "On" : "Off"; });

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
