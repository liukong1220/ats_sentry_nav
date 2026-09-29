// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__MINCO_JOINT_OPTIMIZER_HPP_
#define MINCO_PLANNER__TRAJECTORY__MINCO_JOINT_OPTIMIZER_HPP_

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "minco_planner/trajectory/lbfgs.hpp"

namespace minco_planner
{

/**
 * 整条轨迹 MINCO S3 联合优化（GCOPTER / EGO-Planner 思路）。
 *
 * 决策变量是全部内点 q 与全部段时长 τ（T = exp(τ)），首尾位置、首端速度/加速度、
 * 尾端零速零加速度固定。代价在整条轨迹上积分：
 *   J = w_e ∫||p'''||² + ρ_T ΣT
 *     + Σ_i Σ_j (T_i/K) ω_j [ρ_obs P_obs + ρ_g P_guide + ρ_v P_v + ρ_a P_a + ρ_lat P_lat]
 * 罚项均为三次铰链 max(0, x)³；梯度先对系数/时刻求偏导，再由 MincoS3::propagateGrad
 * 一次伴随求解反传到 (q, T)。
 */
struct MincoJointOptimizerParams
{
  // 在引导线上按弧长取内点的间距 [m]。
  double waypoint_spacing = 1.0;
  // 每段采样区间数 K（梯形积分）。
  int samples_per_piece = 16;
  // 段时长软边界 [s]；超出按三次铰链罚，超出一倍后视为不可行点。
  double min_piece_time = 0.05;
  double max_piece_time = 3.0;
  double energy_weight = 1.0;
  double time_weight = 20.0;
  double time_bound_weight = 1e4;
  // 三次铰链在小违例处很软：1e4 时 L 形内角残差约 0.07 m，1e6 时约 0.01 m。
  double obstacle_weight = 1e6;
  double velocity_weight = 1e3;
  double acceleration_weight = 1e3;
  double lateral_weight = 1e3;
  // 引导管道：采样点到引导折线的距离超过 max_guide_deviation 时按三次铰链罚，
  // 只在调用方传入引导折线时生效（通常是没有 ESDF 的情形）。<= 0 关闭。
  double guide_weight = 1e6;
  double max_guide_deviation = 0.10;
  double max_velocity = 1.5;
  double max_acceleration = 2.5;
  double max_lateral_acceleration = 1.5;
  // 低于该速度不计横向加速度罚（|v×a|/|v| 在 v→0 处无定义）。
  double lateral_min_speed = 0.10;
  // 中心点 ESDF 净空目标 [m]，与 JPS safe_distance 对齐；<= 0 关闭。
  double center_clearance = 0.42;
  // 足迹采样点（已含 safety_margin 的矩形边界）ESDF 净空目标 [m]；
  // 只在提供 yaw 参考时启用。
  double footprint_clearance = 0.03;
  double footprint_length = 0.58;
  double footprint_width = 0.58;
  double footprint_safety_margin = 0.05;
  // 每条边在两角点之间的等分采样数，1 = 只取边中点。
  int footprint_edge_samples = 1;
  // 报告约束满足的残差容差：净空 [m]、动力学 [m/s, m/s^2]。
  double obstacle_tolerance = 0.02;
  double guide_tolerance = 0.05;
  double dynamics_tolerance = 0.05;
  lbfgs::LbfgsParams solver;
};

// 查询 ESDF 距离与梯度；返回 false 表示该点无数据（不计罚）。
using JointDistanceQuery =
  std::function<bool(
      const Eigen::Vector2d & position, double & distance, Eigen::Vector2d & gradient)>;
// 归一化进度 [0,1] -> yaw；优化中视为常量，不对 yaw 求导。
using JointYawQuery = std::function<double(double progress)>;

struct MincoJointPenaltyStats
{
  double energy = 0.0;
  double time = 0.0;
  double obstacle_cost = 0.0;
  double guide_cost = 0.0;
  double dynamics_cost = 0.0;
  // 最大违例量（未加权）。
  double max_center_violation = 0.0;
  double max_footprint_violation = 0.0;
  double max_guide_excess = 0.0;
  double max_velocity_excess = 0.0;
  double max_acceleration_excess = 0.0;
  double max_lateral_excess = 0.0;
};

struct MincoJointProblem
{
  // col(0) 为起点，col(1)/col(2) 为首端速度/加速度。
  Eigen::Matrix<double, 2, 3> head_state = Eigen::Matrix<double, 2, 3>::Zero();
  Eigen::Vector2d tail_position = Eigen::Vector2d::Zero();
  int piece_count = 0;
  JointDistanceQuery distance;
  JointYawQuery yaw;
  // 计算 yaw 进度所用的固定时长（通常为初值）；长度不符时退化为当前时长。
  // 固定后 yaw 与决策变量无关，解析梯度与目标函数严格一致。
  Eigen::VectorXd yaw_progress_durations;
  // 引导折线（可为空 = 不计管道罚）与每段可搜索的折线段下标区间 [first, last]；
  // 区间为空时对整条折线搜索。
  std::vector<Eigen::Vector2d> guide;
  std::vector<std::pair<int, int>> guide_segment_ranges;
};

struct MincoJointResult
{
  // true = 得到有限的整条轨迹（不代表约束全部满足）。
  bool solved = false;
  bool constraints_satisfied = false;
  std::string termination;
  int iterations = 0;
  int evaluations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double wall_time_ms = 0.0;
  MincoJointPenaltyStats stats;
  // 起点、内点、终点。
  std::vector<Eigen::Vector2d> waypoints;
  Eigen::VectorXd durations;
};

class MincoJointOptimizer
{
public:
  explicit MincoJointOptimizer(MincoJointOptimizerParams params = MincoJointOptimizerParams());

  const MincoJointOptimizerParams & params() const {return params_;}

  /**
   * @brief 按弧长在折线上等距取点，段数 = max(1, round(L / spacing))，首尾点精确保留。
   */
  static std::vector<Eigen::Vector2d> resampleByArcLength(
    const std::vector<Eigen::Vector2d> & polyline, double spacing);

  /**
   * @brief 以给定航点与时长为初值做整条轨迹联合优化。
   * @param head_state        首端 [p|v|a]；p 被 initial_waypoints.front() 覆盖。
   * @param initial_waypoints 起点、内点初值、终点（至少 2 个）。
   * @param initial_durations 段时长初值，长度 = initial_waypoints.size() - 1。
   * @param distance          可为空；空时不计障碍罚。
   * @param yaw               可为空；空时只计中心点净空，不计足迹罚。
   * @param guide             可为空；非空时计引导管道罚（通常为加密后的引导折线）。
   */
  MincoJointResult optimize(
    const Eigen::Matrix<double, 2, 3> & head_state,
    const std::vector<Eigen::Vector2d> & initial_waypoints,
    const Eigen::VectorXd & initial_durations,
    const JointDistanceQuery & distance,
    const JointYawQuery & yaw,
    const std::vector<Eigen::Vector2d> & guide = {}) const;

  /**
   * @brief 代价与梯度，x = [vec(q) (2(N-1)); τ (N)]。
   * @return 代价；不可行（时长越界、MINCO 求解失败）返回 +inf。
   */
  double evaluate(
    const MincoJointProblem & problem,
    const Eigen::VectorXd & x,
    Eigen::VectorXd & gradient,
    MincoJointPenaltyStats * stats = nullptr) const;

private:
  MincoJointOptimizerParams params_;
  std::vector<Eigen::Vector2d> footprint_offsets_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__MINCO_JOINT_OPTIMIZER_HPP_
