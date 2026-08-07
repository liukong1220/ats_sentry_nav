// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_
#define ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/zero_speed_guard.hpp"

namespace ats_swerve_mpc {

/**
 * @brief Solver-independent LTV-MPC quadratic program description.
 *
 * Decision ordering is [delta_x_0 ... delta_x_N, delta_u_0 ... delta_u_N-1].
 * This type deliberately contains no solver status or ROS state.  A later
 * OSQP/HPIPM/qpOASES adapter must validate the hard constraints before its
 * result can reach the existing fail-stop control node.
 */
struct LtvQpProblem {
  bool valid = false;
  std::string validation_error;
  int horizon = 0;
  int state_dimension = 3;
  int control_dimension = 3;
  bool zero_speed_guard_active = false;
  std::array<bool, 4> angle_rate_linearization_valid{{false, false, false, false}};

  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd equality_matrix;
  Eigen::VectorXd equality_lower;
  Eigen::VectorXd equality_upper;
  Eigen::MatrixXd inequality_matrix;
  Eigen::VectorXd inequality_lower;
  Eigen::VectorXd inequality_upper;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;

  /** @brief 返回第 step 个状态偏差块在固定决策向量中的起始列。 */
  int stateOffset(int step) const { return state_dimension * step; }
  /** @brief 返回第 step 个控制偏差块在固定决策向量中的起始列。 */
  int controlOffset(int step) const {
    return state_dimension * (horizon + 1) + control_dimension * step;
  }
  /** @brief 返回 [delta_x,delta_u] 固定布局的总决策维度。 */
  int decisionSize() const {
    return state_dimension * (horizon + 1) + control_dimension * horizon;
  }
};

class LtvQpBuilder {
public:
  /**
   * @brief 在控制 timer 外按固定 horizon 预分配稠密 LTV-QP 工作缓冲。
   * @details 决策维度与约束行数在此确定；timer 内只覆盖数值，禁止借此函数隐式改变结构。
   */
  static LtvQpProblem allocate(int horizon);

  /**
   * @brief 围绕 iLQR 名义 rollout 构造凸二次跟踪问题。
   * @details 包含 SE(2) 一阶线性动力学、车体系速度边界和车体加速度/控制增量边界。
   *          轮速范数与舵角约束不在此伪线性化；低速方向未定义由
   *          angle_rate_linearization_valid 显式暴露，未来主链接入前必须独立验证。
   */
  static LtvQpProblem build(
      const State &current_state, const std::vector<State> &nominal_states,
      const std::vector<Control> &nominal_controls,
      const std::vector<Se2Reference> &references, const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config = ZeroSpeedGuardConfig());

  /**
   * @brief 在不改变维度的前提下重填已有的 LTV-QP 数值缓冲。
   * @details 只有调用方给出错误 horizon 的缓冲时才会分配；节点在构造期已创建匹配缓冲，
   *          因而正常控制 timer 只写入已有数值存储。
   */
  static bool build(
      const State &current_state, const std::vector<State> &nominal_states,
      const std::vector<Control> &nominal_controls,
      const std::vector<Se2Reference> &references, const Control &last_control,
      const Se2MpcConfig &config, LtvQpProblem &problem,
      const ZeroSpeedGuardConfig &guard_config = ZeroSpeedGuardConfig());
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_
