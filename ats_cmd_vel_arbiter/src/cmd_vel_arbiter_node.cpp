// Copyright 2026 ATS 2026 Sentry Project

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "ats_cmd_vel_arbiter/cmd_vel_arbiter.hpp"
#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ats_cmd_vel_arbiter
{

class CmdVelArbiterNode : public rclcpp::Node
{
public:
  CmdVelArbiterNode()
  : Node("cmd_vel_arbiter")
  {
    manual_topic_ = declare_parameter<std::string>("manual_cmd_vel_topic", "/cmd_vel");
    auto_topic_ = declare_parameter<std::string>("autonomy_cmd_vel_topic", "/cmd_vel/autonomy");
    selected_topic_ = declare_parameter<std::string>("selected_cmd_vel_topic", "/cmd_vel/selected");
    execution_command_topic_ = declare_parameter<std::string>(
      "execution_command_topic", "/planner/execution_command");
    emergency_stop_topic_ = declare_parameter<std::string>(
      "emergency_stop_topic", "/planner/emergency_stop");
    link_health_topic_ = declare_parameter<std::string>("link_health_topic", "/serial/link_up");
    require_serial_link_ = declare_parameter<bool>("require_serial_link", true);
    const int manual_timeout_ms = declare_parameter<int>("manual_timeout_ms", 300);
    const int auto_timeout_ms = declare_parameter<int>("auto_timeout_ms", 300);
    const int link_timeout_ms = declare_parameter<int>("link_timeout_ms", 300);
    execution_command_timeout_s_ = declare_parameter<double>("execution_command_timeout", 0.5);
    const double rate_hz = declare_parameter<double>("control_rate_hz", 20.0);

    CmdVelArbiterConfig config;
    config.manual_timeout = std::chrono::milliseconds(manual_timeout_ms);
    config.auto_timeout = std::chrono::milliseconds(auto_timeout_ms);
    config.execution_command_timeout = std::chrono::milliseconds(
      static_cast<int>(execution_command_timeout_s_ * 1000.0));
    config.link_timeout = std::chrono::milliseconds(require_serial_link_ ? link_timeout_ms : 0);
    arbiter_.setConfig(config);
    if (!require_serial_link_) {
      arbiter_.onSerialLinkUp(std::chrono::steady_clock::now());
    }

    selected_pub_ = create_publisher<geometry_msgs::msg::Twist>(selected_topic_, 10);
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_topic_, 10,
      std::bind(&CmdVelArbiterNode::onManual, this, std::placeholders::_1));
    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_topic_, 10,
      std::bind(&CmdVelArbiterNode::onAuto, this, std::placeholders::_1));
    execution_command_sub_ =
      create_subscription<ats_navigation_interfaces::msg::ExecutionCommand>(
        execution_command_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&CmdVelArbiterNode::onExecutionCommand, this, std::placeholders::_1));
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&CmdVelArbiterNode::onEmergencyStop, this, std::placeholders::_1));
    if (require_serial_link_) {
      link_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        link_health_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&CmdVelArbiterNode::onLinkHealth, this, std::placeholders::_1));
    }

    const auto period = std::chrono::milliseconds(
      std::max(1, static_cast<int>(1000.0 / std::max(1.0, rate_hz))));
    timer_ = create_wall_timer(period, std::bind(&CmdVelArbiterNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel arbiter: manual=%s auto=%s selected=%s link=%s require_link=%s T_manual=%d ms T_auto=%d ms",
      manual_topic_.c_str(), auto_topic_.c_str(), selected_topic_.c_str(),
      link_health_topic_.c_str(), require_serial_link_ ? "true" : "false",
      manual_timeout_ms, auto_timeout_ms);
  }

private:
  void onManual(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arbiter_.onManual(
      msg->linear.x, msg->linear.y, msg->angular.z, std::chrono::steady_clock::now());
  }

  void onAuto(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arbiter_.onAuto(
      msg->linear.x, msg->linear.y, msg->angular.z, std::chrono::steady_clock::now());
  }

  void onExecutionCommand(
    const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr msg)
  {
    using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
    const bool execute = msg->mode == ExecutionCommand::MODE_EXECUTE;
    const rclcpp::Time receipt = now();
    const rclcpp::Time stamp(msg->header.stamp, receipt.get_clock_type());
    if (stamp > receipt) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting future ExecutionCommand incarnation=%llu sequence=%llu",
        static_cast<unsigned long long>(msg->manager_incarnation),
        static_cast<unsigned long long>(msg->command_sequence));
      return;
    }
    const auto age = std::chrono::milliseconds(
      static_cast<int64_t>((receipt - stamp).seconds() * 1000.0));
    std::lock_guard<std::mutex> lock(mutex_);
    arbiter_.onExecutionCommand(
      execute, msg->manager_incarnation, msg->command_sequence, age,
      std::chrono::steady_clock::now());
  }

  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arbiter_.onEmergencyStop(msg->data);
  }

  void onLinkHealth(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arbiter_.onLinkHealth(msg->data, std::chrono::steady_clock::now());
  }

  void onTimer()
  {
    CmdVelArbiterOutput output;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      output = arbiter_.tick(std::chrono::steady_clock::now());
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = output.vx;
    twist.linear.y = output.vy;
    twist.angular.z = output.wz;
    selected_pub_->publish(twist);
  }

  std::string manual_topic_;
  std::string auto_topic_;
  std::string selected_topic_;
  std::string execution_command_topic_;
  std::string emergency_stop_topic_;
  std::string link_health_topic_;
  bool require_serial_link_{true};
  double execution_command_timeout_s_{0.5};
  CmdVelArbiter arbiter_;
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::ExecutionCommand>::SharedPtr
    execution_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr link_health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ats_cmd_vel_arbiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_cmd_vel_arbiter::CmdVelArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
