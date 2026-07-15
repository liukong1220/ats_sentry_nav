// Copyright 2026

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_types.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rog_map/rog_map.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ats_rog_map
{

namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

rog_map::Pose poseFromTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  return {
    rog_map::Vec3f(translation.x, translation.y, translation.z),
    super_utils::Quatf(rotation.w, rotation.x, rotation.y, rotation.z)};
}

class RogMapEngine final : public rog_map::ROGMap
{
public:
  RogMapEngine(const rclcpp::Clock::SharedPtr & clock, const std::string & config_file)
  : clock_(clock)
  {
    cfg_ = rog_map::Config(config_file);
    init();
  }

  void update(
    const rog_map::PointCloud & cloud, const rog_map::Pose & robot_pose,
    const rog_map::Pose & sensor_pose)
  {
    // The robot moves the sliding window while the sensor remains the ray origin.
    updateRobotState(robot_pose);
    updateProbMap(cloud, sensor_pose, robot_pose.first);
  }

private:
  const double getSystemWalltimeNow() override
  {
    return clock_->now().seconds();
  }

  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace

class AtsRogMapNode final : public rclcpp::Node
{
public:
  AtsRogMapNode()
  : Node("ats_rog_map"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto package_share = ament_index_cpp::get_package_share_directory("ats_rog_map");
    map_config_file_ = declare_parameter<std::string>(
      "map_config_file", package_share + "/config/rog_map.yaml");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "gimbal_yaw_odom");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "front_mid360");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/registered_scan");
    cloud_timeout_sec_ = std::max(0.0, declare_parameter<double>("cloud_timeout_sec", 0.5));
    odom_timeout_sec_ = std::max(0.0, declare_parameter<double>("odom_timeout_sec", 0.5));
    tf_timeout_sec_ = std::max(0.0, declare_parameter<double>("tf_timeout_sec", 0.1));
    debug_rate_hz_ = std::max(0.0, declare_parameter<double>("debug_rate_hz", 2.0));
    esdf_visualization_height_ = declare_parameter<double>("esdf_visualization_height", 0.15);
    debug_qos_depth_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("debug_qos_depth", 1)));
    input_qos_reliable_ = declare_parameter<bool>("input_qos_reliable", false);
    debug_qos_reliable_ = declare_parameter<bool>("debug_qos_reliable", false);

    map_ = std::make_unique<RogMapEngine>(get_clock(), map_config_file_);

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(debug_qos_depth_));
    if (input_qos_reliable_) {
      input_qos.reliable();
    } else {
      input_qos.best_effort();
    }
    auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(debug_qos_depth_));
    if (debug_qos_reliable_) {
      debug_qos.reliable();
    } else {
      debug_qos.best_effort();
    }

    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/occ", debug_qos);
    inflated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", debug_qos);
    unknown_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/unk", debug_qos);
    esdf_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf", debug_qos);
    stale_pub_ = create_publisher<std_msgs::msg::Bool>("rog_map/stale", rclcpp::QoS(1).reliable());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, input_qos,
      std::bind(&AtsRogMapNode::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, input_qos,
      std::bind(&AtsRogMapNode::cloudCallback, this, std::placeholders::_1));

    const auto health_period = std::chrono::milliseconds(100);
    health_timer_ = create_wall_timer(health_period, std::bind(&AtsRogMapNode::publishHealth, this));
    if (debug_rate_hz_ > 0.0) {
      const auto debug_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / debug_rate_hz_));
      debug_timer_ = create_wall_timer(
        std::max(debug_period, std::chrono::milliseconds(1)),
        std::bind(&AtsRogMapNode::publishDebug, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "ROGMap ready: odom='%s' cloud='%s' map='%s' base='%s' sensor='%s' config='%s'",
      odom_topic_.c_str(), cloud_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
      sensor_frame_.c_str(), map_config_file_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_odom_receive_time_ = std::chrono::steady_clock::now();
  }

  bool lookupTransform(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, source_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec_));
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ROGMap waits for TF %s -> %s at cloud time: %s",
        source_frame.c_str(), map_frame_.c_str(), exception.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ROGMap ignored a cloud without frame_id.");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    geometry_msgs::msg::TransformStamped map_from_base;
    geometry_msgs::msg::TransformStamped map_from_sensor;
    geometry_msgs::msg::TransformStamped map_from_cloud;
    if (!lookupTransform(base_frame_, stamp, map_from_base) ||
      !lookupTransform(sensor_frame_, stamp, map_from_sensor) ||
      !lookupTransform(msg->header.frame_id, stamp, map_from_cloud))
    {
      return;
    }

    rog_map::PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);
    tf2::Transform cloud_transform;
    tf2::fromMsg(map_from_cloud.transform, cloud_transform);
    for (auto & point : cloud) {
      const tf2::Vector3 transformed = cloud_transform * tf2::Vector3(point.x, point.y, point.z);
      point.x = static_cast<float>(transformed.x());
      point.y = static_cast<float>(transformed.y());
      point.z = static_cast<float>(transformed.z());
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_->update(cloud, poseFromTransform(map_from_base), poseFromTransform(map_from_sensor));
      const auto & stats = map_->getLastRaycastDebugStats();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap update=%llu input=%d endpoints=%d hits=%d occupied=%d stale=%d",
        static_cast<unsigned long long>(stats.update_index), stats.input_points,
        stats.raycast_endpoint_points, stats.hit_endpoint_points, stats.occupied_touched_cells,
        stats.stale_decayed_cells);
      last_map_stamp_ = stamp;
      has_map_data_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      last_cloud_receive_time_ = std::chrono::steady_clock::now();
    }
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const rog_map::vec_E<rog_map::Vec3f> & points, const rclcpp::Time & stamp) const
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(points.size());
    for (const auto & point : points) {
      cloud.emplace_back(point.x(), point.y(), point.z());
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    return message;
  }

  sensor_msgs::msg::PointCloud2 makeEsdfCloud(
    const rog_map::Vec3f & box_min, const rog_map::Vec3f & box_max, const rclcpp::Time & stamp) const
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    const double resolution = std::max(map_->getResolution(), 0.01);
    const float z = static_cast<float>(std::clamp(
      esdf_visualization_height_, static_cast<double>(box_min.z()), static_cast<double>(box_max.z())));
    for (double x = box_min.x(); x <= box_max.x(); x += resolution) {
      for (double y = box_min.y(); y <= box_max.y(); y += resolution) {
        const rog_map::Vec3f point(static_cast<float>(x), static_cast<float>(y), z);
        const double distance = map_->getESDFDistance(point);
        if (std::isfinite(distance)) {
          pcl::PointXYZI esdf_point;
          esdf_point.x = point.x();
          esdf_point.y = point.y();
          esdf_point.z = point.z();
          esdf_point.intensity = static_cast<float>(distance);
          cloud.push_back(esdf_point);
        }
      }
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    return message;
  }

  void publishDebug()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!has_map_data_) {
      return;
    }

    const bool publish_occupied = occupied_pub_->get_subscription_count() > 0U;
    const bool publish_inflated = inflated_pub_->get_subscription_count() > 0U;
    const bool publish_unknown = unknown_pub_->get_subscription_count() > 0U;
    const bool publish_esdf = esdf_pub_->get_subscription_count() > 0U && map_->hasESDF();
    if (!publish_occupied && !publish_inflated && !publish_unknown && !publish_esdf) {
      return;
    }

    rog_map::Vec3f box_min = map_->getLocalMapOrigin();
    rog_map::Vec3f box_max = box_min + map_->getLocalMapSize();
    map_->boundBoxByLocalMap(box_min, box_max);
    if (publish_occupied) {
      rog_map::vec_E<rog_map::Vec3f> occupied;
      map_->boxSearch(box_min, box_max, super_utils::OCCUPIED, occupied);
      occupied_pub_->publish(makeCloud(occupied, last_map_stamp_));
    }
    if (publish_inflated) {
      rog_map::vec_E<rog_map::Vec3f> inflated;
      map_->boxSearchInflate(box_min, box_max, super_utils::OCCUPIED, inflated);
      inflated_pub_->publish(makeCloud(inflated, last_map_stamp_));
    }
    if (publish_unknown) {
      rog_map::vec_E<rog_map::Vec3f> unknown;
      map_->boxSearch(box_min, box_max, super_utils::UNKNOWN, unknown);
      unknown_pub_->publish(makeCloud(unknown, last_map_stamp_));
    }
    if (publish_esdf) {
      esdf_pub_->publish(makeEsdfCloud(box_min, box_max, last_map_stamp_));
    }
  }

  void publishHealth()
  {
    const auto now = std::chrono::steady_clock::now();
    bool cloud_stale = true;
    bool odom_stale = true;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (last_cloud_receive_time_) {
        cloud_stale = std::chrono::duration<double>(now - *last_cloud_receive_time_).count() >
          cloud_timeout_sec_;
      }
      if (last_odom_receive_time_) {
        odom_stale = std::chrono::duration<double>(now - *last_odom_receive_time_).count() >
          odom_timeout_sec_;
      }
    }
    std_msgs::msg::Bool message;
    message.data = cloud_stale || odom_stale;
    stale_pub_->publish(message);
    if (message.data) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap input stale: cloud=%s odom=%s", cloud_stale ? "true" : "false",
        odom_stale ? "true" : "false");
    }
  }

  std::string map_config_file_;
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string odom_topic_;
  std::string cloud_topic_;
  double cloud_timeout_sec_{0.5};
  double odom_timeout_sec_{0.5};
  double tf_timeout_sec_{0.1};
  double debug_rate_hz_{2.0};
  double esdf_visualization_height_{0.15};
  int debug_qos_depth_{1};
  bool input_qos_reliable_{false};
  bool debug_qos_reliable_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<RogMapEngine> map_;
  std::mutex map_mutex_;
  std::mutex input_mutex_;
  std::optional<SteadyTime> last_cloud_receive_time_;
  std::optional<SteadyTime> last_odom_receive_time_;
  rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
  bool has_map_data_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stale_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace ats_rog_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_rog_map::AtsRogMapNode>());
  rclcpp::shutdown();
  return 0;
}
