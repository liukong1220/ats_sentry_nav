// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_
#define ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"
#include "ats_swerve_mpc/zero_speed_guard.hpp"

namespace ats_swerve_mpc {

/**
 * @brief LTV-QP 的唯一 checked dimension 结果。
 * @details 所有运行期 dense buffer、CSC setup 和节点 controller 构造都必须消费同一个
 *          结果。计算只使用 checked size_t 算术，并在转成 Eigen/int 前执行 horizon、整数
 *          表示范围和 dense payload 上限检查，避免 timer 或启动路径发生巨型分配/有符号溢出。
 */
struct LtvQpDimensions {
  bool valid = false;
  int horizon = 0;
  int decision_size = 0;
  int equality_rows = 0;
  int inequality_rows = 0;
  int constraint_rows = 0;
  std::size_t dense_buffer_bytes = 0;
  const char *validation_error = "invalid LTV-QP dimensions";
};

/** @brief 已审计的 dense LTV-QP 资源上限；当前正式配置 horizon=30 不受影响。 */
constexpr int kLtvQpMaximumHorizon = 64;
/** @brief 矩阵/向量 double payload 上限，不包含 Eigen/OSQP 小对象元数据。 */
constexpr std::size_t kLtvQpDenseBufferMaximumBytes = 3U * 1024U * 1024U;

/**
 * @brief 计算并校验固定 LTV-QP 的所有维度和 dense payload 大小。
 * @param horizon 预测步数。
 * @return 失败时 `valid=false` 且不进行任何分配；成功时返回可直接用于 Eigen/OSQP 的
 *         int 维度。
 */
LtvQpDimensions checkedLtvQpDimensions(int horizon);

/**
 * @brief 与后端无关的 LTV-MPC 二次规划数值描述。
 *
 * @details 决策顺序固定为 `[delta_x_0 ... delta_x_N, delta_u_0 ... delta_u_N-1]`。
 *          本类型只表达由同周期 iLQR 名义轨迹线性化得到的矩阵和值，不携带 ROS
 *          状态、求解状态或发布权。任何 OSQP/HPIPM/qpOASES 适配器都必须在其
 *          结果到达既有 fail-stop 节点之前通过独立硬约束复核。
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
  int stateOffset(int step) const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3 &&
                   step >= 0 && step <= horizon
               ? state_dimension * step
               : -1;
  }
  /** @brief 返回第 step 个控制偏差块在固定决策向量中的起始列。 */
  int controlOffset(int step) const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3 &&
                   step >= 0 && step < horizon
               ? dimensions.equality_rows + control_dimension * step
               : -1;
  }
  /** @brief 返回 [delta_x,delta_u] 固定布局的总决策维度。 */
  int decisionSize() const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3
               ? dimensions.decision_size
               : 0;
  }

  /**
   * @brief 校验 LTV-QP 稠密缓冲是否仍符合固定的状态、控制和约束行布局。
   * @details 该检查只验证各矩阵/向量的尺寸，不接受仅凭 `valid` 标志访问 Eigen 缓冲；
   *          因而可以在 backend 数值拷贝和 candidate 准入前阻止结构损坏导致的越界访问。
   */
  bool hasExpectedLayout() const;

  /**
   * @brief 校验所有双边约束下界不超过上界。
   * @details 变量上下界允许 OSQP 使用正负无穷，但 NaN 和任一 `lower > upper` 都会失败。
   *          调用前不需要假定矩阵尺寸正确：本函数会先复核固定布局。
   */
  bool hasOrderedBounds() const;

  /**
   * @brief 校验所有矩阵和向量数值后再允许 backend/reconstructor 索引访问。
   * @details 系数矩阵、gradient 和等式/不等式 bounds 必须 finite；变量上下界允许
   *          OSQP 使用 +/-infinity，但 NaN 永远拒绝。该函数不依赖 `valid` 标志。
   */
  bool hasFiniteNumerics() const;
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
   * @details 预分配 buffer 的完整 layout 不匹配时直接 fail-closed；控制 timer 不会因
   *          损坏对象隐式重新分配大型 dense 矩阵。需要新维度时必须在 timer 外显式 allocate。
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
