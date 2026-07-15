// Copyright 2026

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ats_rog_map/rog_map_engine.hpp"
#include "ats_rog_map_interfaces/srv/get_rog_map_projection.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_types.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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
    self_filter_radius_ = std::max(0.0, declare_parameter<double>("self_filter_radius", 0.45));
    debug_qos_depth_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("debug_qos_depth", 1)));
    input_qos_reliable_ = declare_parameter<bool>("input_qos_reliable", false);
    debug_qos_reliable_ = declare_parameter<bool>("debug_qos_reliable", false);

    map_ = std::make_unique<RogMapEngine>(get_clock(), map_config_file_);

    odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    projection_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    health_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    debug_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
    projection_service_ = create_service<ats_rog_map_interfaces::srv::GetRogMapProjection>(
      "rog_map/get_ground_projection",
      std::bind(
        &AtsRogMapNode::getGroundProjection, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, projection_callback_group_);

    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, input_qos,
      std::bind(&AtsRogMapNode::odomCallback, this, std::placeholders::_1), odom_options);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, input_qos,
      std::bind(&AtsRogMapNode::cloudCallback, this, std::placeholders::_1), cloud_options);

    const auto health_period = std::chrono::milliseconds(100);
    health_timer_ = create_wall_timer(
      health_period, std::bind(&AtsRogMapNode::publishHealth, this), health_callback_group_);
    if (debug_rate_hz_ > 0.0) {
      const auto debug_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / debug_rate_hz_));
      debug_timer_ = create_wall_timer(
        std::max(debug_period, std::chrono::milliseconds(1)),
        std::bind(&AtsRogMapNode::publishDebug, this), debug_callback_group_);
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
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      last_cloud_receive_time_ = std::chrono::steady_clock::now();
    }
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

    rog_map::PointCloud input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    rog_map::PointCloud cloud;
    cloud.reserve(input_cloud.size());
    tf2::Transform cloud_transform;
    tf2::fromMsg(map_from_cloud.transform, cloud_transform);
    const auto & base_translation = map_from_base.transform.translation;
    for (auto point : input_cloud) {
      const tf2::Vector3 transformed = cloud_transform * tf2::Vector3(point.x, point.y, point.z);
      point.x = static_cast<float>(transformed.x());
      point.y = static_cast<float>(transformed.y());
      point.z = static_cast<float>(transformed.z());
      if (std::hypot(point.x - base_translation.x, point.y - base_translation.y) <
        self_filter_radius_)
      {
        continue;
      }
      cloud.push_back(point);
    }

    bool map_updated = false;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_updated = map_->update(
        cloud, poseFromTransform(map_from_base), poseFromTransform(map_from_sensor));
      if (map_updated) {
        const auto & stats = map_->getLastRaycastDebugStats();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ROGMap update=%llu snapshot=%llu input=%d endpoints=%d hits=%d occupied=%d stale=%d",
          static_cast<unsigned long long>(stats.update_index),
          static_cast<unsigned long long>(map_->generation()), stats.input_points,
          stats.raycast_endpoint_points, stats.hit_endpoint_points, stats.occupied_touched_cells,
          stats.stale_decayed_cells);
        last_map_stamp_ = stamp;
        has_map_data_ = true;
      }
    }
    if (map_updated) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      last_map_update_time_ = std::chrono::steady_clock::now();
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

    const rog_map::Vec3f map_center = map_->getLocalMapOrigin();
    const rog_map::Vec3f half_map_size = 0.5F * map_->getLocalMapSize();
    rog_map::Vec3f box_min = map_center - half_map_size;
    rog_map::Vec3f box_max = map_center + half_map_size;
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
    if (publish_esdf && map_->ensureCurrentEsdf()) {
      rog_map::Vec3f esdf_box_min;
      rog_map::Vec3f esdf_box_max;
      if (map_->getCurrentEsdfBounds(esdf_box_min, esdf_box_max)) {
        esdf_pub_->publish(makeEsdfCloud(esdf_box_min, esdf_box_max, last_map_stamp_));
      }
    }
  }

  void publishHealth()
  {
    bool map_update_stale = true;
    bool odom_stale = true;
    bool raw_cloud_stale = true;
    getHealthStaleness(map_update_stale, odom_stale, raw_cloud_stale);
    std_msgs::msg::Bool message;
    message.data = map_update_stale || odom_stale;
    stale_pub_->publish(message);
    if (message.data) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROGMap stale: map_update=%s odom=%s raw_cloud=%s",
        map_update_stale ? "true" : "false", odom_stale ? "true" : "false",
        raw_cloud_stale ? "true" : "false");
    }
  }

  void getHealthStaleness(
    bool & map_update_stale, bool & odom_stale, bool & raw_cloud_stale)
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    raw_cloud_stale = !last_cloud_receive_time_ ||
      std::chrono::duration<double>(now - *last_cloud_receive_time_).count() > cloud_timeout_sec_;
    map_update_stale = !last_map_update_time_ ||
      std::chrono::duration<double>(now - *last_map_update_time_).count() > cloud_timeout_sec_;
    odom_stale = !last_odom_receive_time_ ||
      std::chrono::duration<double>(now - *last_odom_receive_time_).count() > odom_timeout_sec_;
  }

  void getGroundProjection(
    const std::shared_ptr<ats_rog_map_interfaces::srv::GetRogMapProjection::Request> request,
    std::shared_ptr<ats_rog_map_interfaces::srv::GetRogMapProjection::Response> response)
  {
    const auto projection_started = std::chrono::steady_clock::now();
    bool map_update_stale = true;
    bool odom_stale = true;
    bool raw_cloud_stale = true;
    getHealthStaleness(map_update_stale, odom_stale, raw_cloud_stale);
    response->stale = map_update_stale || odom_stale;

    std::lock_guard<std::mutex> lock(map_mutex_);
    response->generation = map_->generation();
    response->ready = has_map_data_ && map_->ensureCurrentEsdf();
    if (!response->ready) {
      return;
    }
    rog_map::Vec3f esdf_box_min;
    rog_map::Vec3f esdf_box_max;
    if (!map_->getCurrentEsdfBounds(esdf_box_min, esdf_box_max)) {
      response->ready = false;
      return;
    }

    const rog_map::Vec3f map_center = map_->getLocalMapOrigin();
    const rog_map::Vec3f half_map_size = 0.5F * map_->getLocalMapSize();
    rog_map::Vec3f box_min = map_center - half_map_size;
    rog_map::Vec3f box_max = map_center + half_map_size;
    map_->boundBoxByLocalMap(box_min, box_max);
    const double resolution = std::max(
      map_->getResolution(), request->resolution > 0.0F ?
      static_cast<double>(request->resolution) : map_->getResolution());
    const double z_min = std::max(
      static_cast<double>(box_min.z()),
      std::min(static_cast<double>(request->min_height), static_cast<double>(request->max_height)));
    const double z_max = std::min(
      static_cast<double>(box_max.z()),
      std::max(static_cast<double>(request->min_height), static_cast<double>(request->max_height)));
    if (z_max < z_min) {
      response->ready = false;
      return;
    }

    auto & grid = response->occupancy_grid;
    grid.header.frame_id = map_frame_;
    grid.header.stamp = last_map_stamp_;
    grid.info.map_load_time = last_map_stamp_;
    grid.info.resolution = resolution;
    grid.info.width = static_cast<std::uint32_t>(std::ceil(
      (static_cast<double>(box_max.x()) - static_cast<double>(box_min.x())) / resolution));
    grid.info.height = static_cast<std::uint32_t>(std::ceil(
      (static_cast<double>(box_max.y()) - static_cast<double>(box_min.y())) / resolution));
    grid.info.origin.position.x = box_min.x();
    grid.info.origin.position.y = box_min.y();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const std::size_t cell_count =
      static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    grid.data.assign(cell_count, -1);
    response->signed_distance.assign(cell_count, nan);
    response->gradient_x.assign(cell_count, nan);
    response->gradient_y.assign(cell_count, nan);

    const double z_step = std::max(map_->getResolution(), 0.01);
    const double esdf_distance_limit = (esdf_box_max - esdf_box_min).norm() + resolution;
    const auto inside_esdf_box = [&esdf_box_min, &esdf_box_max](const rog_map::Vec3f & point) {
        constexpr double tolerance = 1e-6;
        return (point.array() >= (esdf_box_min.array() - tolerance)).all() &&
               (point.array() <= (esdf_box_max.array() + tolerance)).all();
      };
    for (std::uint32_t my = 0; my < grid.info.height; ++my) {
      for (std::uint32_t mx = 0; mx < grid.info.width; ++mx) {
        const std::size_t index =
          static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) + mx;
        const float x = static_cast<float>(
          grid.info.origin.position.x + (static_cast<double>(mx) + 0.5) * resolution);
        const float y = static_cast<float>(
          grid.info.origin.position.y + (static_cast<double>(my) + 0.5) * resolution);
        bool occupied = false;
        bool known_free = false;
        double minimum_distance = std::numeric_limits<double>::infinity();
        for (double z = z_min + 0.5 * z_step; z <= z_max + 1e-6; z += z_step) {
          const rog_map::Vec3f point(x, y, static_cast<float>(std::min(z, z_max)));
          // Downstream JPS and footprint gates apply their own metric clearance on this snapshot.
          // Project raw probability occupancy here to avoid double-inflating the robot start cell.
          const rog_map::GridType cell_type = map_->getGridType(point);
          occupied = occupied || cell_type == super_utils::OCCUPIED;
          known_free = known_free || cell_type == super_utils::KNOWN_FREE;
          if (inside_esdf_box(point) &&
            (cell_type == super_utils::OCCUPIED || cell_type == super_utils::KNOWN_FREE))
          {
            const double distance = map_->getESDFDistance(point);
            if (std::isfinite(distance) && std::abs(distance) <= esdf_distance_limit) {
              minimum_distance = std::min(minimum_distance, std::abs(distance));
            }
          }
        }
        if (occupied) {
          grid.data[index] = 100;
          if (std::isfinite(minimum_distance)) {
            response->signed_distance[index] = static_cast<float>(
              -std::max(0.5 * resolution, minimum_distance));
          }
        } else if (known_free) {
          grid.data[index] = 0;
          if (std::isfinite(minimum_distance)) {
            response->signed_distance[index] = static_cast<float>(minimum_distance);
          }
        }
      }
    }

    const auto index_of = [&grid](std::uint32_t x, std::uint32_t y) {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.info.width) + x;
      };
    for (std::uint32_t my = 0; my < grid.info.height; ++my) {
      for (std::uint32_t mx = 0; mx < grid.info.width; ++mx) {
        const std::size_t index = index_of(mx, my);
        const float center = response->signed_distance[index];
        if (!std::isfinite(center)) {
          continue;
        }
        const float left = mx > 0 ? response->signed_distance[index_of(mx - 1, my)] : nan;
        const float right = mx + 1 < grid.info.width ?
          response->signed_distance[index_of(mx + 1, my)] : nan;
        const float down = my > 0 ? response->signed_distance[index_of(mx, my - 1)] : nan;
        const float up = my + 1 < grid.info.height ?
          response->signed_distance[index_of(mx, my + 1)] : nan;
        if (std::isfinite(left) && std::isfinite(right)) {
          response->gradient_x[index] = static_cast<float>((right - left) / (2.0 * resolution));
        } else if (std::isfinite(right)) {
          response->gradient_x[index] = static_cast<float>((right - center) / resolution);
        } else if (std::isfinite(left)) {
          response->gradient_x[index] = static_cast<float>((center - left) / resolution);
        } else {
          response->gradient_x[index] = 0.0F;
        }
        if (std::isfinite(down) && std::isfinite(up)) {
          response->gradient_y[index] = static_cast<float>((up - down) / (2.0 * resolution));
        } else if (std::isfinite(up)) {
          response->gradient_y[index] = static_cast<float>((up - center) / resolution);
        } else if (std::isfinite(down)) {
          response->gradient_y[index] = static_cast<float>((center - down) / resolution);
        } else {
          response->gradient_y[index] = 0.0F;
        }
      }
    }

    getHealthStaleness(map_update_stale, odom_stale, raw_cloud_stale);
    response->stale = map_update_stale || odom_stale;
    const double projection_ms = 1000.0 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - projection_started).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ROGMap projection generation=%llu cells=%zu compute=%.1f ms stale=%s",
      static_cast<unsigned long long>(response->generation), cell_count, projection_ms,
      response->stale ? "true" : "false");
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
  double self_filter_radius_{0.45};
  int debug_qos_depth_{1};
  bool input_qos_reliable_{false};
  bool debug_qos_reliable_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<RogMapEngine> map_;
  std::mutex map_mutex_;
  std::mutex input_mutex_;
  std::optional<SteadyTime> last_cloud_receive_time_;
  std::optional<SteadyTime> last_map_update_time_;
  std::optional<SteadyTime> last_odom_receive_time_;
  rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
  bool has_map_data_{false};

  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr projection_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;
  rclcpp::CallbackGroup::SharedPtr debug_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stale_pub_;
  rclcpp::Service<ats_rog_map_interfaces::srv::GetRogMapProjection>::SharedPtr projection_service_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace ats_rog_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ats_rog_map::AtsRogMapNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
