// Copyright 2026

#ifndef MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_
#define MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_

#include <string>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

struct YawSplinePlannerParams
{
  std::string mode = "goal_heading";
  double yaw_rate_limit = 2.5;
  double narrow_clearance_enter = 0.55;
  double narrow_clearance_exit = 0.70;
  // 窄通道内足迹绕 z 轴的旋转对称阶数：矩形为 2（正向/反向切线等价），正方形为 4
  // （切线 ±pi/2 同样等价）。阶数越高，对齐所需的最大转角越小（pi/2 -> pi/4）。
  int tangent_symmetry_order = 2;
  // 两段窄通道之间的开阔间隙若不长于该时长（s），整体按窄通道处理，继续切线对齐，
  // 避免参考 yaw 在短间隙内先回摆到目标朝向、再转回切线造成的来回摆动。0 表示关闭。
  double narrow_gap_bridge_time = 0.0;
  // clearance_aware 逐点跟随期望 yaw 时的角加速度上限（rad/s^2）；<= 0 关闭，仅限角速度。
  // 只限角速度时窄通道切线变化会让参考 yaw_rate 阶跃，角加速度远超 MPC max_awz，
  // MPC 跟不上后 yaw 滞后再超调。开启后按"可刹停"的二阶跟踪生成 yaw。
  double yaw_acceleration_limit = 0.0;
  // clearance_aware 开阔段的期望 yaw："goal_heading"（默认，整条轨迹向目标朝向平滑过渡）或
  // "minimal_rotation"：每个开阔段从进入时的 yaw 平滑过渡到下一段窄通道入口的对齐朝向
  // （正方形取 90° 等价中最近的一个），末段过渡到目标朝向。窄通道出口后不再快速回摆到
  // 目标朝向，转向分摊到整段开阔区域，参考 yaw 角速度更小、MPC 不易超调。
  std::string open_area_yaw_mode = "goal_heading";
  // 终点独立转向采用零平移采样；每个采样仍进入离散矩形足迹安全 gate。
  double terminal_yaw_sample_period = 0.10;
};

class YawSplinePlanner
{
public:
  explicit YawSplinePlanner(YawSplinePlannerParams params = YawSplinePlannerParams());

  void setParams(const YawSplinePlannerParams & params);
  void apply(ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const;

private:
  void applyGoalHeading(
    ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const;
  void applyPathTangent(ReferenceTrajectory & trajectory, double initial_yaw) const;
  void applyClearanceAware(
    ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const;
  void appendTerminalGoalYawTransition(
    ReferenceTrajectory & trajectory, double goal_yaw) const;
  static double normalizeAngle(double angle);
  static double shortestAngularDistance(double from, double to);

  YawSplinePlannerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_
