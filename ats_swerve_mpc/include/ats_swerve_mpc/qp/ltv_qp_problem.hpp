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
 * @brief QP 矩阵的不可变压缩列（CSC）稀疏 pattern。
 * @details 行索引和列偏移属于后端契约。具体求解器 setup 后只可更新数值；pattern 变化必须
 *          被拒绝并在控制 timer 外显式 setup，不能触发隐式分配。
 */
struct LtvQpSparseStructure {
  int rows = 0;
  int columns = 0;
  std::vector<int> column_offsets;
  std::vector<int> row_indices;

  /** @brief 校验 CSC 尺寸、列偏移和行序；失败即禁止 backend 数值更新。 */
  bool valid(std::string *error = nullptr) const;
  /** @brief 比较完整行/列 pattern，防止 timer 内隐式重建 solver workspace。 */
  bool samePattern(const LtvQpSparseStructure &other) const;
};

/**
 * @brief LTV-QP 的唯一 checked dimension 结果（固定稀疏 CSC 布局）。
 */
struct LtvQpDimensions {
  bool valid = false;
  int horizon = 0;
  int state_vars = 0;
  int control_vars = 0;
  int slack_vars = 0;
  int decision_size = 0;
  int equality_rows = 0;
  int body_accel_rows = 0;
  int wheel_octagon_rows = 0;
  int wheel_rate_rows = 0;
  int esdf_corridor_rows = 0;
  int inequality_rows = 0;
  int bound_rows = 0;
  int constraint_rows = 0;
  std::size_t sparse_buffer_bytes = 0;
  const char *validation_error = "invalid LTV-QP dimensions";
};

/** @brief 已审计的 LTV-QP 最大预测步数（正式配置 horizon=60 在内）。 */
constexpr int kLtvQpMaximumHorizon = 64;
/** @brief 稀疏数值缓冲上限（实际 horizon=60 约 120 KiB，远低于上限）。 */
constexpr std::size_t kLtvQpSparseBufferMaximumBytes = 512U * 1024U;

/** @brief 计算并校验固定稀疏 LTV-QP 的所有维度与字节上界。 */
LtvQpDimensions checkedLtvQpDimensions(int horizon);

/** @brief 单个预测步在 odom 系下的线性化 ESDF 安全走廊样本。 */
struct EsdfCorridorSample {
  bool active = false;
  double signed_distance_m = 0.0;
  double target_clearance_m = 0.0;
  Eigen::Vector2d gradient_odom = Eigen::Vector2d::Zero();
};

/** @brief QP 内部线性化 ESDF 安全走廊软约束配置。 */
struct LtvQpEsdfConfig {
  bool enabled = false;
  double corridor_clearance_m = 0.44;
  double reference_margin_m = 0.015;
  double min_corridor_clearance_m = 0.305;
  double slack_linear_weight = 400.0;
  double slack_quadratic_weight = 4000.0;
  double max_slack_m = 0.35;
};

/** @brief 预计算的固定 CSC 数值槽位索引（启动期一次构建，控制周期 O(1) 直填无查找）。 */
struct LtvQpSlotMap {
  struct StateStepSlots {
    int p_xx = -1;
    int p_xy = -1;
    int p_yy = -1;
    int p_yaw = -1;
    std::array<int, 3> a_init_diag{{-1, -1, -1}};
    std::array<int, 3> a_dyn_next_diag{{-1, -1, -1}};
    std::array<std::array<int, 3>, 3> a_dyn_state{{
        {{-1, -1, -1}}, {{-1, -1, -1}}, {{-1, -1, -1}}}};
    int a_esdf_x = -1;
    int a_esdf_y = -1;
  };

  struct ControlStepSlots {
    // Upper triangle of 3x3 diagonal block: (0,0), (0,1), (1,1), (0,2), (1,2), (2,2)
    int p_u00 = -1;
    int p_u01 = -1;
    int p_u11 = -1;
    int p_u02 = -1;
    int p_u12 = -1;
    int p_u22 = -1;
    // Full 3x3 super-diagonal block (u_{k-1} rows, u_k cols), valid when step > 0
    std::array<std::array<int, 3>, 3> p_prev_curr{{
        {{-1, -1, -1}}, {{-1, -1, -1}}, {{-1, -1, -1}}}};
    int p_slack = -1;

    // Dynamics equality step k: -B_{k,0} on u_k
    std::array<std::array<int, 3>, 3> a_dyn_curr{{
        {{-1, -1, -1}}, {{-1, -1, -1}}, {{-1, -1, -1}}}};
    // Dynamics equality step k (when k > 0): -B_{k,-1} on u_{k-1}
    std::array<std::array<int, 3>, 3> a_dyn_prev{{
        {{-1, -1, -1}}, {{-1, -1, -1}}, {{-1, -1, -1}}}};

    // Rotating-frame body accel step k: +I on u_k, -J_rot on u_{k-1} (when k > 0)
    std::array<int, 3> a_accel_curr_diag{{-1, -1, -1}};
    std::array<std::array<int, 3>, 3> a_accel_prev{{
        {{-1, -1, -1}}, {{-1, -1, -1}}, {{-1, -1, -1}}}};

    // 8 exact outer-envelope octagonal wheel-speed rows on u_k (8 rows x 3 cols)
    std::array<std::array<int, 3>, 8> a_wheel_oct{{}};

    // Control variable box bounds (3 rows) + ESDF slack coeff & bound
    std::array<int, 3> a_ctrl_bound_diag{{-1, -1, -1}};
    int a_esdf_slack = -1;
    int a_slack_bound = -1;
  };

  std::vector<StateStepSlots> states;
  std::vector<ControlStepSlots> controls;
};

/**
 * @brief 零分配固定 CSC 稀疏 LTV-QP 问题缓冲。
 *
 * @details 决策顺序固定为 `[delta_x_0..delta_x_N, delta_u_0..delta_u_{N-1}, s_1..s_N]`。
 *          彻底移除 2 MiB 稠密矩阵构造与清零开销，直接在预分配 CSC 数组中更新非零元。
 */
struct LtvQpProblem {
  bool valid = false;
  std::string validation_error;
  int horizon = 0;
  int state_dimension = 3;
  int control_dimension = 3;
  bool zero_speed_guard_active = false;
  std::array<bool, 4> angle_rate_linearization_valid{{false, false, false, false}};

  LtvQpSparseStructure hessian_structure;
  std::vector<double> hessian_values;
  LtvQpSparseStructure constraint_structure;
  std::vector<double> constraint_values;
  Eigen::VectorXd gradient;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  LtvQpSlotMap slots;

  int stateOffset(int step) const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3 &&
                   step >= 0 && step <= horizon
               ? 3 * step
               : -1;
  }

  int controlOffset(int step) const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3 &&
                   step >= 0 && step < horizon
               ? dimensions.state_vars + 3 * step
               : -1;
  }

  int slackOffset(int step) const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3 &&
                   step >= 0 && step < horizon
               ? dimensions.state_vars + dimensions.control_vars + step
               : -1;
  }

  int decisionSize() const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3
               ? dimensions.decision_size
               : 0;
  }

  int constraintRows() const {
    const auto dimensions = checkedLtvQpDimensions(horizon);
    return dimensions.valid && state_dimension == 3 && control_dimension == 3
               ? dimensions.constraint_rows
               : 0;
  }

  bool hasExpectedLayout() const;
  bool hasOrderedBounds() const;
  bool hasFiniteNumerics() const;
};

class LtvQpBuilder {
public:
  /** @brief 生成 ATS 稀疏 LTV-QP 的固定上三角 Hessian CSC pattern。 */
  static LtvQpSparseStructure buildHessianPattern(int horizon);
  /** @brief 生成 ATS 稀疏 LTV-QP 的固定约束矩阵 CSC pattern。 */
  static LtvQpSparseStructure buildConstraintPattern(int horizon);

  /** @brief 在控制 timer 外预分配固定稀疏 CSC 缓冲与 O(1) 槽位映射。 */
  static LtvQpProblem allocate(int horizon);

  /** @brief 围绕名义轨迹一次分配并直填固定稀疏 LTV-QP。 */
  static LtvQpProblem build(
      const State &current_state, const std::vector<State> &nominal_states,
      const std::vector<Control> &nominal_controls,
      const std::vector<Se2Reference> &references, const Control &last_control,
      const Se2MpcConfig &config,
      const ZeroSpeedGuardConfig &guard_config = ZeroSpeedGuardConfig(),
      double first_step_dt = -1.0,
      const LtvQpEsdfConfig &esdf_config = LtvQpEsdfConfig(),
      const std::vector<EsdfCorridorSample> *esdf_samples = nullptr);

  /** @brief 零分配直填既有固定稀疏 LTV-QP 缓冲（控制 timer 热路径）。 */
  static bool build(
      const State &current_state, const std::vector<State> &nominal_states,
      const std::vector<Control> &nominal_controls,
      const std::vector<Se2Reference> &references, const Control &last_control,
      const Se2MpcConfig &config, LtvQpProblem &problem,
      const ZeroSpeedGuardConfig &guard_config = ZeroSpeedGuardConfig(),
      double first_step_dt = -1.0,
      const LtvQpEsdfConfig &esdf_config = LtvQpEsdfConfig(),
      const std::vector<EsdfCorridorSample> *esdf_samples = nullptr);
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__LTV_QP_PROBLEM_HPP_
