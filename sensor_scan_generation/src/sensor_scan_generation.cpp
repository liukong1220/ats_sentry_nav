

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <memory>
#include <utility>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<std::string>("odometry_output_frame", "");
  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<bool>("publish_robot_base_tf", true);
  this->declare_parameter<bool>("publish_base_frame_tf", true);
  this->declare_parameter<bool>("cache_lidar_to_base_tf", false);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  this->declare_parameter<bool>("allow_latest_base_tf_fallback", true);
  this->declare_parameter<double>("latest_base_tf_max_age_sec", 0.2);
  this->declare_parameter<int>("sync_queue_size", 100);
  this->declare_parameter<bool>("use_identified_odometry", false);

  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odometry_output_frame", odometry_output_frame_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("publish_robot_base_tf", publish_robot_base_tf_);
  this->get_parameter("publish_base_frame_tf", publish_base_frame_tf_);
  this->get_parameter("cache_lidar_to_base_tf", cache_lidar_to_base_tf_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  this->get_parameter("allow_latest_base_tf_fallback", allow_latest_base_tf_fallback_);
  this->get_parameter("latest_base_tf_max_age_sec", latest_base_tf_max_age_sec_);
  this->get_parameter("sync_queue_size", sync_queue_size_);
  tf_lookup_timeout_sec_ = std::max(0.0, tf_lookup_timeout_sec_);
  latest_base_tf_max_age_sec_ = std::max(0.0, latest_base_tf_max_age_sec_);
  sync_queue_size_ = std::max(2, sync_queue_size_);
  if (odometry_output_frame_.empty()) {
    odometry_output_frame_ = robot_base_frame_;
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  pub_identified_odometry_ = this->create_publisher<
    ats_navigation_interfaces::msg::IdentifiedOdometry>("odometry_identified", 2);
  identified_odometry_sub_ = this->create_subscription<
    ats_navigation_interfaces::msg::IdentifiedOdometry>(
    "lidar_odometry_identified", rclcpp::SensorDataQoS(),
    [this](ats_navigation_interfaces::msg::IdentifiedOdometry::ConstSharedPtr msg) {
      if (!get_parameter("use_identified_odometry").as_bool() || msg->instance_id == 0 ||
        msg->header != msg->odometry.header)
      {
        return;
      }
      odometryHandler(std::make_shared<nav_msgs::msg::Odometry>(msg->odometry), msg->instance_id);
    });

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    20,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);
  odometry_sub_.registerCallback(
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) {
      if (!get_parameter("use_identified_odometry").as_bool()) {
        odometryHandler(msg);
      }
    });

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(sync_queue_size_), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::odometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg, std::uint64_t instance_id)
{
  if (instance_id != current_instance_id_) {
    if (instance_id != 0 &&
      (instance_id < current_instance_id_ || retired_instances_.count(instance_id) != 0U)) {
      return;
    }
    if (current_instance_id_ != 0) {
      retired_instances_.insert(current_instance_id_);
    }
    has_previous_odometry_ = false;
    twist_pose_history_.clear();
    current_instance_id_ = instance_id;
  }
  tf2::Transform tf_odom_to_lidar;
  tf2::Transform tf_lidar_to_chassis;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  // lidar->robot_base_frame 是静态外参（实车 bringup、MuJoCo、Gazebo 都只在 /tf_static 提供），
  // 首次查询成功后缓存，之后每帧不再查 TF；成功之前每帧照旧重试，失败时不发布。
  // lidar->base_frame 中间隔着 gimbal_yaw_odom_joint，会随云台 yaw 变化，
  // 不能缓存，仍逐帧按时间戳查询。
  if (!has_lidar_to_robot_base_) {
    if (!lookupTransform(
          lidar_frame_, robot_base_frame_, odometry_msg->header.stamp, tf_lidar_to_robot_base_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Skipping odometry publish because required lidar->robot_base TF is unavailable.");
      return;
    }
    has_lidar_to_robot_base_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Cached static extrinsic %s->%s, no further per-frame lookup.",
      lidar_frame_.c_str(), robot_base_frame_.c_str());
  }

  // An empty base_frame declares "this profile has no chassis-footprint
  // consumer". When odometry_output_frame_ == base_frame_ (e.g. gimbal mode
  // with base_footprint odometry decoupling), lidar->base_frame is required
  // for /odometry; otherwise its transient failure only skips odom->base_frame
  // TF without blocking /odometry on robot_base_frame_.
  const bool want_chassis_frame = !base_frame_.empty();
  const bool output_is_base_frame =
    want_chassis_frame && (odometry_output_frame_ == base_frame_);
  bool has_chassis_pose = false;
  if (want_chassis_frame) {
    if (cache_lidar_to_base_tf_ && has_lidar_to_base_) {
      tf_lidar_to_chassis = tf_lidar_to_base_;
      has_chassis_pose = true;
    } else if (lookupTransform(
        lidar_frame_, base_frame_, odometry_msg->header.stamp, tf_lidar_to_chassis,
        allow_latest_base_tf_fallback_)) {
      has_chassis_pose = true;
      if (cache_lidar_to_base_tf_) {
        tf_lidar_to_base_ = tf_lidar_to_chassis;
        has_lidar_to_base_ = true;
        RCLCPP_INFO(
          this->get_logger(), "Cached static extrinsic %s->%s, no further per-frame lookup.",
          lidar_frame_.c_str(), base_frame_.c_str());
      }
    } else if (output_is_base_frame) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Skipping odometry publish because required %s->%s TF is unavailable.",
        lidar_frame_.c_str(), base_frame_.c_str());
      return;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Optional %s->%s TF unavailable; continuing odometry on %s.",
        lidar_frame_.c_str(), base_frame_.c_str(), robot_base_frame_.c_str());
    }
  }

  tf_odom_to_robot_base_ = tf_odom_to_lidar * tf_lidar_to_robot_base_;
  if (has_chassis_pose) {
    tf_odom_to_chassis_ = tf_odom_to_lidar * tf_lidar_to_chassis;
  }
  has_robot_base_pose_ = true;

  if (publish_tf_) {
    if (publish_base_frame_tf_ && has_chassis_pose) {
      publishTransform(tf_odom_to_chassis_, odom_frame_, base_frame_, odometry_msg->header.stamp);
    }
    const bool duplicate_child =
      publish_base_frame_tf_ && has_chassis_pose && (base_frame_ == robot_base_frame_);
    if (publish_robot_base_tf_ && !duplicate_child) {
      publishTransform(
        tf_odom_to_robot_base_, odom_frame_, robot_base_frame_, odometry_msg->header.stamp);
    }
  }
  if (output_is_base_frame) {
    publishOdometry(
      tf_odom_to_chassis_, odom_frame_, base_frame_, odometry_msg->header.stamp);
  } else {
    publishOdometry(
      tf_odom_to_robot_base_, odom_frame_, robot_base_frame_, odometry_msg->header.stamp);
  }
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_odom_to_lidar;

  if (!has_robot_base_pose_ && !get_parameter("use_identified_odometry").as_bool()) {
    odometryHandler(odometry_msg);
  }
  if (!pub_laser_cloud_ || pub_laser_cloud_->get_subscription_count() == 0U) {
    return;
  }
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  // 以 unique_ptr 发布：开 intra-process 时 publish(const T &) 会整帧深拷贝一次。
  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, *out);
  pub_laser_cloud_->publish(std::move(out));
}

bool SensorScanGenerationNode::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform, bool allow_latest_fallback)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    if (allow_latest_fallback) {
      try {
        auto latest = tf_buffer_->lookupTransform(
          target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
        const rclcpp::Time latest_stamp(latest.header.stamp);
        const double skew =
          latest_stamp.nanoseconds() > 0 ? std::abs((time - latest_stamp).seconds()) : 0.0;
        if (latest_base_tf_max_age_sec_ <= 0.0 || skew <= latest_base_tf_max_age_sec_) {
          tf2::fromMsg(latest.transform, transform);
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Using latest %s->%s TF (skew=%.3fs) after lookup fallback: %s",
            target_frame.c_str(), source_frame.c_str(), skew, ex.what());
          return true;
        }
      } catch (const tf2::TransformException &) {
      }
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "TF lookup failed (%s->%s): %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (!twist_pose_history_.empty() && stamp < twist_pose_history_.back().stamp) {
    twist_pose_history_.clear();
  }
  const TwistPoseSample * ref_sample = nullptr;
  for (auto it = twist_pose_history_.rbegin(); it != twist_pose_history_.rend(); ++it) {
    ref_sample = &(*it);
    if ((stamp - it->stamp).seconds() >= 0.05) {
      break;
    }
  }
  if (ref_sample != nullptr) {
    const double dt = (stamp - ref_sample->stamp).seconds();
    if (dt > 1e-4 && dt <= 0.5) {
      const tf2::Vector3 delta_world =
        transform.getOrigin() - ref_sample->pose.getOrigin();
      const tf2::Vector3 linear_velocity_world = delta_world / dt;

      const tf2::Matrix3x3 world_to_base(transform.getRotation().inverse());
      const tf2::Vector3 linear_velocity_base = world_to_base * linear_velocity_world;

      const tf2::Quaternion q_diff =
        transform.getRotation() * ref_sample->pose.getRotation().inverse();
      tf2::Vector3 angular_axis = q_diff.getAxis();
      if (
        !std::isfinite(angular_axis.x()) || !std::isfinite(angular_axis.y()) ||
        !std::isfinite(angular_axis.z())) {
        angular_axis = tf2::Vector3(0.0, 0.0, 0.0);
      }
      double angle = q_diff.getAngle();
      if (angle > M_PI) {
        angle -= 2.0 * M_PI;
      }
      const tf2::Vector3 angular_velocity_world = angular_axis * (angle / dt);
      const tf2::Vector3 angular_velocity_base = world_to_base * angular_velocity_world;

      out.twist.twist.linear.x = linear_velocity_base.x();
      out.twist.twist.linear.y = linear_velocity_base.y();
      out.twist.twist.linear.z = linear_velocity_base.z();
      out.twist.twist.angular.x = angular_velocity_base.x();
      out.twist.twist.angular.y = angular_velocity_base.y();
      out.twist.twist.angular.z = angular_velocity_base.z();
    }
  }

  twist_pose_history_.push_back(TwistPoseSample{stamp, transform});
  while (twist_pose_history_.size() > 2U &&
    (stamp - twist_pose_history_.front().stamp).seconds() > 0.12)
  {
    twist_pose_history_.pop_front();
  }
  previous_odom_to_robot_base_ = transform;
  previous_odom_stamp_ = stamp;
  has_previous_odometry_ = true;

  if (current_instance_id_ != 0) {
    ats_navigation_interfaces::msg::IdentifiedOdometry identified;
    identified.header = out.header;
    identified.instance_id = current_instance_id_;
    identified.odometry = out;
    pub_identified_odometry_->publish(identified);
  }
  pub_chassis_odometry_->publish(std::make_unique<nav_msgs::msg::Odometry>(std::move(out)));
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
