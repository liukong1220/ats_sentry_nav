// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_

#include <Eigen/Core>

#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"

namespace trajectory_optimizer
{
class RcTraversabilityEsdfProvider;
}

namespace minco_planner
{

struct MincoTrajectoryOptimizerParams
{
  double reference_speed = 1.5;
  double min_segment_time = 0.05;
  double sample_spacing = 0.12;
  double max_velocity = 2.0;
  double max_acceleration = 2.5;
  int max_time_scaling_iterations = 5;
  double time_scaling_factor = 1.25;

  // Keep MINCO's interpolation from cutting into obstacles between JPS nodes.
  // The correction moves only inner control points and re-solves MINCO after
  // each update; endpoints and the JPS route topology remain fixed.
  bool esdf_obstacle_optimization_enabled = true;
  double esdf_obstacle_clearance = 0.45;
  int esdf_obstacle_max_iterations = 6;
  double esdf_obstacle_control_point_spacing = 0.30;
  double esdf_obstacle_max_step = 0.10;
  double esdf_obstacle_max_deviation = 0.50;

  // A second ESDF pass uses a yaw reference and the same rectangular samples as
  // the final footprint gate. It changes translation only; yaw remains independent.
  bool esdf_footprint_optimization_enabled = true;
  double esdf_footprint_clearance = 0.10;
  double esdf_footprint_sample_spacing = 0.10;
  double footprint_length = 0.70;
  double footprint_width = 0.55;
  double footprint_safety_margin = 0.05;

  // 重规划时用当前车速播种 MINCO 首端状态所允许的上限（见 InitialKinematicState）。
  // 物理含义：实车里程计/定位反馈的世界系速度、加速度会有噪声与外推误差，
  // 若原样写入首端边界条件，一次异常跳变就会让五次多项式在首段冲出可行域。
  // 因此对播种量做保守裁剪：速度上限 [m/s]，加速度上限 [m/s^2]。
  double initial_state_max_speed = 2.0;
  double initial_state_max_acceleration = 2.5;
};

/**
 * @brief 重规划时刻的车体运动状态（世界系），用于播种 MINCO 首端边界条件。
 *
 * MINCO S3 的首端边界是 2x3 矩阵 [位置 | 速度 | 加速度]。原实现恒为零，
 * 等价于"每次重规划都假设车辆静止"：车辆以 1.5 m/s 行进时，新轨迹的首段
 * 速度从 0 起算，MPC 参考前馈与实际状态出现阶跃，实车表现为重规划瞬间
 * 顿挫/超调（仿真里因为定位无噪声、重规划稀疏而不易暴露）。
 *
 * valid=false 时保持原有零初值行为，保证首次规划与单测语义不变。
 */
struct InitialKinematicState
{
  bool valid = false;
  // 世界系线速度 [m/s]，来自 /localization 的 twist 旋转到世界系后的结果。
  Eigen::Vector2d velocity{0.0, 0.0};
  // 世界系线加速度 [m/s^2]；无可靠估计时保持零，仅播种速度即可。
  Eigen::Vector2d acceleration{0.0, 0.0};
};

class MincoTrajectoryOptimizer
{
public:
  explicit MincoTrajectoryOptimizer(
    MincoTrajectoryOptimizerParams params = MincoTrajectoryOptimizerParams());

  void setParams(const MincoTrajectoryOptimizerParams & params);
  bool esdfObstacleOptimizationEnabled() const;
  bool esdfFootprintOptimizationEnabled() const;

  /**
   * @brief 由 JPS/A* 折线生成连续可跟踪的 MINCO S3 参考轨迹。
   * @param raw_path             搜索得到的折线路径（规划栅格坐标系）。
   * @param esdf                 可选 ESDF 提供者；非空时对内部控制点做净空修正。
   * @param footprint_orientation 可选 yaw 参考轨迹；非空时按旋转后的矩形采样查询 ESDF。
   * @param initial_state        可选重规划初值；valid=true 时把当前车速/加速度写入
   *                             MINCO 首端边界条件，避免重规划瞬间速度阶跃。
   * @return 参考轨迹；求解失败时返回 points 为空的对象（调用方必须按失败处理）。
   * @note 每次 planGoal 及其候选轨迹重优化时调用；函数为 const，不持有任何运行期状态。
   */
  ReferenceTrajectory optimize(
    const nav_msgs::msg::Path & raw_path,
    const trajectory_optimizer::RcTraversabilityEsdfProvider * esdf = nullptr,
    const ReferenceTrajectory * footprint_orientation = nullptr,
    const InitialKinematicState * initial_state = nullptr) const;

private:
  MincoTrajectoryOptimizerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
