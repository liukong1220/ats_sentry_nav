// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/minco_time_allocator.hpp"
#include "minco_planner/trajectory/path_geometry_preprocessor.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ats_rc_esdf
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
  // A non-positive value keeps compatibility for library users which have not
  // configured a vehicle jerk limit. The executed ATS profile sets it.
  double max_jerk = 0.0;
  double max_lateral_acceleration = 1.5;
  int max_time_scaling_iterations = 5;
  double time_scaling_factor = 1.25;

  PathGeometryPreprocessorParams geometry_preprocessor;

  // Keep MINCO's interpolation from cutting into obstacles between JPS nodes.
  // The correction moves only inner control points and re-solves MINCO after
  // each update; endpoints and the JPS route topology remain fixed.
  bool esdf_obstacle_optimization_enabled = true;
  double esdf_obstacle_clearance = 0.45;
  // Zero preserves the historical esdf_obstacle_clearance contract for
  // library callers. Deployed profiles set an explicit trigger/target pair.
  double esdf_obstacle_trigger_clearance = 0.0;
  double esdf_obstacle_target_clearance = 0.0;
  int esdf_obstacle_max_iterations = 6;
  double esdf_obstacle_control_point_spacing = 0.30;
  // 引导加密间距 [m]；0 = 不加密。加密后的引导供 ESDF 修正、弹性带平滑和
  // 联合优化重采样使用（联合优化本身按 joint_waypoint_spacing 取内点）。
  double guide_control_point_spacing = 0.0;
  // 加密后的引导点先做受净空约束的弹性带平滑，再交给 ESDF 修正与 MINCO。
  // 加密点是 MINCO 的硬插值约束，不平滑就会把 JPS/倒角折线原样保留成折角。
  // 每轮 x += alpha*(邻点中点 - x) + fidelity*(原引导 - x)：前者拉直，后者把偏离
  // 以连续方式拉回（平衡时拐角被抹成半径约 spacing*sqrt(alpha/(2*fidelity)) 的弧，
  // 不会像硬截断那样在截断处再造一个折角）。0 次迭代 = 历史行为。另有两道硬约束：
  // 偏离原引导不超过 max_deviation；ESDF 中心净空不低于 min(原净空, min_clearance)，
  // 即不会把任何点推得比原引导更靠近障碍。首尾点固定，最终安全仍由矩形足迹门禁裁定。
  int guide_smoothing_iterations = 0;
  double guide_smoothing_alpha = 0.5;
  double guide_smoothing_fidelity = 0.02;
  double guide_smoothing_max_deviation = 0.50;
  double guide_smoothing_min_clearance = 0.42;
  // 整条轨迹 MINCO 联合优化（唯一的最终轨迹生成器，见 minco_joint_optimizer.hpp）。
  // 在上面 ESDF 修正并再加密的引导上按弧长每 joint_waypoint_spacing 取一个内点，
  // 以全部内点 q 与全部段时长 T=exp(τ) 为变量，一次 L-BFGS 最小化
  // jerk 能量 + 时间 + 中心/足迹 ESDF 罚（无 ESDF 时改为引导管道罚）
  // + 速度/加速度/横向加速度罚（三次铰链）。
  // 数值失败 fail-closed（返回空轨迹）；残差不直接判失败，交给时间缩放与矩形足迹门禁。
  double joint_waypoint_spacing = 1.0;
  int joint_samples_per_piece = 16;
  double joint_max_piece_time = 3.0;
  double joint_energy_weight = 1.0;
  double joint_time_weight = 20.0;
  double joint_obstacle_weight = 1e6;
  double joint_velocity_weight = 1e3;
  double joint_acceleration_weight = 1e3;
  double joint_lateral_weight = 1e3;
  // 无 ESDF 时的引导管道：没有障碍信息（ESDF 不可用或 ESDF 修正关闭，如 JPS 无 ESDF
  // 候选）时，轨迹偏离加密引导折线超过 joint_guide_max_deviation [m] 按三次铰链罚，
  // 防止联合优化在无障碍信息下抄近路；有 ESDF 时由障碍项负责，不加管道。<= 0 关闭。
  double joint_guide_weight = 1e6;
  double joint_guide_max_deviation = 0.10;
  // 中心点 ESDF 净空目标，部署时与 jps_safe_distance 对齐。
  double joint_center_clearance = 0.42;
  // 足迹采样点（已含 safety_margin）的 ESDF 净空目标；只在有 yaw 参考时生效。
  double joint_footprint_clearance = 0.03;
  // 每条足迹边在两角点之间的采样数，1 = 边中点。
  int joint_footprint_edge_samples = 1;
  double joint_g_epsilon = 1e-5;
  int joint_max_iterations = 200;
  double joint_time_budget_ms = 15.0;
  double esdf_obstacle_max_step = 0.10;
  double esdf_obstacle_max_deviation = 0.50;
  double esdf_obstacle_trust_region = 0.10;
  int esdf_obstacle_backtracking_steps = 4;
  double esdf_obstacle_smoothing_weight = 0.25;

  // A second ESDF pass uses a yaw reference and the same rectangular samples as
  // the final footprint gate. It changes translation only; yaw remains independent.
  bool esdf_footprint_optimization_enabled = true;
  double esdf_footprint_clearance = 0.10;
  double esdf_footprint_trigger_clearance = 0.0;
  double esdf_footprint_target_clearance = 0.0;
  double esdf_footprint_sample_spacing = 0.10;
  double footprint_length = 0.58;
  double footprint_width = 0.58;
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

struct MincoOptimizationTrace
{
  nav_msgs::msg::Path preprocessed_guide;
  nav_msgs::msg::Path esdf_refined_guide;
  std::vector<double> segment_durations;
  bool esdf_geometry_refined = false;
  bool local_time_scaled = false;
  bool uniform_time_scaled = false;
  // Diagnostics only. A rejected candidate never reaches the control reference
  // publisher, but its stage boundary must remain observable in the same map snapshot.
  std::string failure_reason;
  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  double peak_jerk = 0.0;
  double solver_wall_time_ms = 0.0;
  // 整条轨迹联合优化诊断。
  int joint_piece_count = 0;
  int joint_iterations = 0;
  int joint_evaluations = 0;
  std::string joint_termination;
  double joint_initial_cost = 0.0;
  double joint_final_cost = 0.0;
  double joint_wall_time_ms = 0.0;
  bool joint_constraints_satisfied = false;
  double joint_max_center_violation = 0.0;
  double joint_max_footprint_violation = 0.0;
  double joint_max_guide_excess = 0.0;
  double joint_max_velocity_excess = 0.0;
  double joint_max_acceleration_excess = 0.0;
  double joint_max_lateral_excess = 0.0;
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
   * @param planning_grid        Optional immutable grid used only for a fail-closed,
   *                             footprint-aware guide shortcut.
   * @param safety_checker       Must refer to the same semantics as the final gate.
   * @param trace                Optional observability output; it never changes planning.
   * @return 参考轨迹；求解失败时返回 points 为空的对象（调用方必须按失败处理）。
   * @note 每次 planGoal 及其候选轨迹重优化时调用；函数为 const，不持有任何运行期状态。
   */
  ReferenceTrajectory optimize(
    const nav_msgs::msg::Path & raw_path,
    const ats_rc_esdf::RcTraversabilityEsdfProvider * esdf = nullptr,
    const ReferenceTrajectory * footprint_orientation = nullptr,
    const InitialKinematicState * initial_state = nullptr,
    const nav_msgs::msg::OccupancyGrid * planning_grid = nullptr,
    const FootprintSafetyChecker * safety_checker = nullptr,
    MincoOptimizationTrace * trace = nullptr) const;

private:
  MincoTrajectoryOptimizerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
