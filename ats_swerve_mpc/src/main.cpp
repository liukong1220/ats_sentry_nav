// Copyright 2026

#include "rclcpp/rclcpp.hpp"

#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"

/**
 * @brief 启动 ATS Swerve MPC ROS 进程。
 * @details 节点内只有一个 `/cmd_vel_mpc` 发布者；QP shadow 若启用也只作为该节点内的
 *          诊断计算，进程入口不创建额外控制器或命令所有者。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_swerve_mpc::AtsSwerveMpcNode>());
  rclcpp::shutdown();
  return 0;
}
