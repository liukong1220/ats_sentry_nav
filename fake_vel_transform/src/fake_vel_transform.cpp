// Copyright 2026

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  // 官方 profile 必须关闭 cmd_spin 角速度叠加：叠加发生在坐标变换之后，
  // 会让底盘实际 wz 不再等于授权链算出的 wz，
  // 「MPC 之后不改变 [vx, vy, wz]」与统一归零判据会同时失效。
  this->declare_parameter<bool>("enable_cmd_spin", false);
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("enable_cmd_spin", enable_cmd_spin_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  if (!enable_cmd_spin_) {
    // 关闭时强制清零，避免 init_spin_speed 被误配成非零而变成常驻自旋。
    spin_speed_ = 0.0F;
  }

  current_robot_base_angle_ = 0.0;
  initial_robot_base_angle_ = 0.0;
  has_initial_robot_base_angle_ = false;

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  if (enable_cmd_spin_) {
    RCLCPP_WARN(
      get_logger(),
      "cmd_spin 角速度叠加已启用（话题 %s）：该模式会在坐标变换之后修改车体 wz，"
      "官方 profile 禁止使用，仅允许用于对照实验。",
      cmd_spin_topic_.c_str());
    cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
      cmd_spin_topic_, 1,
      std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(
      get_logger(), "cmd_spin 角速度叠加已关闭：不订阅 %s，wz 只来自输入命令。",
      cmd_spin_topic_.c_str());
  }
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  // MPC 输出是车体系 Twist。用最新 odom yaw 维护变换状态，不再通过另一条
  // 路径 topic 推断控制器活动，避免引入第二规划来源和速度时序依赖。
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  // 关闭时即使有残留发布者也不接受叠加值，保证"关闭"是行为上的关闭而非仅不订阅。
  if (!enable_cmd_spin_) {
    return;
  }
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  if (!has_initial_robot_base_angle_) {
    initial_robot_base_angle_ = yaw;
    has_initial_robot_base_angle_ = true;
  }
  current_robot_base_angle_ = yaw;
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (is_zero_vel) {
    // 零速度也经由同一发布者传递，不能等待状态更新而延迟停车。
    cmd_vel_chassis_pub_->publish(transformVelocity(msg));
    return;
  }
  cmd_vel_chassis_pub_->publish(transformVelocity(msg));
}

void FakeVelTransform::publishTransform()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  const double initial_yaw =
    has_initial_robot_base_angle_ ? initial_robot_base_angle_ : current_robot_base_angle_;
  q.setRPY(0, 0, gimbalToFakeYaw(initial_yaw, current_robot_base_angle_));
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  const double initial_yaw =
    has_initial_robot_base_angle_ ? initial_robot_base_angle_ : current_robot_base_angle_;
  const PlanarVelocity transformed = fakeToGimbalVelocity(
    {twist->linear.x, twist->linear.y}, initial_yaw, current_robot_base_angle_);
  aft_tf_vel.linear.x = transformed.x;
  aft_tf_vel.linear.y = transformed.y;
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
