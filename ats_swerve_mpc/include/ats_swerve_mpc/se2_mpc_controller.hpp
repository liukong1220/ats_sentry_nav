// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
#define ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_

#include <array>
#include <vector>

#include <Eigen/Core>

#include "ats_swerve_mpc/se2_model.hpp"

namespace ats_swerve_mpc {

/**
 * @brief 四舵轮底盘 SE(2) MPC（OSQP v1.0.0 稀疏 SQP-RTI）配置。
 *
 * 所有量纲统一为 SI：长度 m、速度 m/s、加速度 m/s^2、角度 rad、角速度 rad/s。
 * 控制量为车体系 Twist [vx, vy, wz]，状态量为世界系位姿 [x, y, yaw]。
 * 每个动力学参数都必须与实车物理部件一一对应，标注见各字段注释。
 */
struct Se2MpcConfig {
  // 预测步数（无量纲）。horizon * dt 即预测时域长度，需覆盖实车一次制动距离。
  int horizon = 26;
  // 离散步长 [s]。与 50 Hz 实时控制周期解耦，首步增量约束按实测 step_dt 计算。
  double dt = 0.10;
  // 状态跟踪权重 [x, y, yaw]：位置误差权重大于航向，保证先贴合路径再修航向。
  Eigen::Vector3d state_weight{18.0, 18.0, 10.0};
  // 控制幅值权重 [vx, vy, wz]：抑制无谓大速度，等价于能耗惩罚。
  Eigen::Vector3d control_weight{0.12, 0.12, 0.08};
  // 控制增量权重 [dvx, dvy, dwz]：抑制指令抖振，直接决定舵轮舵角的平滑度。
  Eigen::Vector3d control_delta_weight{0.70, 0.70, 0.60};
  // 终端状态权重：比 state_weight 更大，保证时域末端收敛到参考点。
  Eigen::Vector3d terminal_weight{28.0, 28.0, 8.0};
  // 沿迹（lag）与横迹（contouring）局部旋转代价权重；<=0 时回退为 state_weight(0/1)。
  double lag_weight = 15.0;
  double contour_weight = 24.0;
  // 一阶执行器响应滞后时间常数 tau_u [s]（0 表示理想瞬时响应）。
  double actuator_lag_tau_sec = 0.025;
  // 纯传输/下位机命令延迟缓冲时间 [s]（0 表示仅使用旋转相位补偿）。
  double actuator_delay_sec = 0.010;
  // 车体前向最大线速度 [m/s]。对应实车纵向可用速度上限（受电机额定转速约束）。
  double max_vx = 5.0;
  // 车体侧向最大线速度 [m/s]。四舵轮为全向底盘，横向能力与纵向同级。
  double max_vy = 5.0;
  // 车体最大偏航角速度 [rad/s]。实车受舵轮转向速率与轮速上限共同限制。
  double max_wz = 3.0;
  // 车体纵向最大加速度 [m/s^2]。对应轮胎-地胶附着极限与电机扭矩上限的较小者。
  double max_ax = 2.0;
  // 车体侧向最大加速度 [m/s^2]，物理含义同 max_ax，方向为车体 y。
  double max_ay = 2.0;
  // 车体最大偏航角加速度 [rad/s^2]。超过该值实车会出现舵轮打滑与航向过冲。
  double max_awz = 2.0;
  // 轮心相对底盘中心的 x 向半轴偏置 [m]（不是完整前后轮距）。
  // 实车对应：舵轮转向轴中心到底盘几何中心的纵向距离，MuJoCo 模型为 0.270。
  double wheel_base_x = 0.0;
  // 轮心相对底盘中心的 y 向半轴偏置 [m]（不是完整左右轮距）。
  // 实车对应：舵轮转向轴中心到底盘几何中心的横向距离，MuJoCo 模型为 0.270。
  double wheel_base_y = 0.0;
  // 单个驱动轮的最大线速度 [m/s]。实车换算关系：
  // max_wheel_speed = 2*pi*wheel_radius*motor_max_rpm/60/actuator_redundancy。
  // 该值是四舵轮真正的执行器边界，比车体 max_vx/max_vy 更严格。
  double max_wheel_speed = 0.0;
  // 单个驱动轮的最大线加速度 [m/s^2]。对应电机峰值扭矩除以等效轮端惯量。
  double max_wheel_acceleration = 0.0;
  // 单个舵轮的最大转向角速度 [rad/s]。实车对应转向电机转速经减速比换算的结果，
  // 是"原地掉头/横移切换"能否在一个控制周期内完成的关键约束。
  double max_steer_rate = 0.0;
  // 模块级增量限幅二分搜索迭代次数。
  int clamp_bisection_iterations = 20;
  // 暖启动与参考速度混合比例（0=纯参考，1=纯暖启动）。
  double warm_start_blend_ratio = 0.5;
};

struct Se2Reference {
  State state = State::Zero();
  Control control = Control::Zero();
  Eigen::Vector2d world_acceleration = Eigen::Vector2d::Zero();
};

/**
 * @brief 旋转相位补偿（报告附录 A.9）：把求解用状态的 yaw 换成 psi + omega * t_d / 2。
 */
State rotationPhaseCompensatedState(const State &measured, double yaw_rate,
                                    double delay);

/**
 * @brief MPC（OSQP SQP-RTI）求解结果与实车诊断标志。
 */
struct Se2MpcResult {
  bool success = false;
  std::vector<Control> controls;
  std::vector<State> states;
  int iterations = 0;
  int accepted_iterations = 0;
  double cost = 0.0;
  double solve_time_ms = 0.0;
  // 首个下发控制是否触及车体速度上限（vx/vy/wz 任一饱和）。
  bool body_limit_saturated = false;
  // 首个下发控制是否触及单轮速度上限（四舵轮执行器饱和）。
  bool wheel_limit_saturated = false;
  // 首个下发控制是否被舵角速率/轮加速度约束回退（增量线搜索被截断）。
  bool increment_limited = false;
  // 四舵轮模块级约束是否真正生效；为 false 说明 wheel_base_* 或轮速上限未配置，
  // 此时只有车体级限幅在起作用，属于实车高危配置错误。
  bool module_limits_active = false;
};

class Se2MpcController {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 创建四舵轮 SE(2) 运动学约束投影器与 SQP-RTI 名义轨迹构造器。 */
  explicit Se2MpcController(const Se2MpcConfig &config = Se2MpcConfig());

  /** @brief 替换动力学和代价配置，并清空与旧配置不兼容的 warm start。 */
  void setConfig(const Se2MpcConfig &config);
  /** @brief 清空控制序列和内部 warm-start 状态，使下一周期回到确定性初始路径。 */
  void reset();

  /** @brief 只读返回当前生效的 MPC 参数，供 node 和 LTV-QP 共用。 */
  const Se2MpcConfig &config() const { return config_; }

  /** @brief 计算阶段或终端的沿迹/横迹旋转状态权重矩阵 R(theta_ref)*diag(q_l,q_c)*R(theta_ref)^T。 */
  static Eigen::Matrix3d stateCostMatrix(const Se2MpcConfig &config,
                                         const Se2Reference &reference,
                                         bool terminal = false);

  /**
   * @brief 为 SQP-RTI 构造满足真实执行器与八边形轮速边界的名义控制序列（连续时间前移 warm-start + 参考混合）。
   */
  std::vector<Control> buildNominalControls(
      const std::vector<Se2Reference> &references, const Control &last_control,
      double first_step_dt, bool octagon_wheel_clamp = false,
      bool *first_step_body_saturated = nullptr,
      bool *first_step_wheel_saturated = nullptr,
      bool *first_step_increment_limited = nullptr);

  /** @brief 将通过准入的 SQP-RTI 控制序列提交回共享 warm-start 缓冲。 */
  void commitWarmControls(const std::vector<Control> &controls);

  /** @brief 使用共享模型（含可选一阶执行器滞后）执行非线性前向 rollout。 */
  std::vector<State> rolloutTrajectory(const State &initial,
                                       const std::vector<Control> &controls,
                                       const Control &last_control,
                                       double first_step_dt) const;

  /** @brief 将单步目标控制按真实旋转系加速度、轮加速度与舵角速率投影到可达集。 */
  Control clampStepIncrement(const Control &target, const Control &previous,
                             double step_dt,
                             bool *body_saturated = nullptr,
                             bool *wheel_saturated = nullptr,
                             bool *increment_limited = nullptr) const;

  /** @brief 计算给定车体控制下四轮中的最大轮速 [m/s]。 */
  double computeMaxModuleSpeed(const Control &control) const {
    return maxModuleSpeed(control);
  }

  /**
   * @brief 四舵轮模块级约束是否处于完全生效状态。
   */
  bool moduleLimitsActive() const;

private:
  // 限幅命中情况，用于向上层输出饱和诊断。
  struct SaturationReport {
    bool body = false;   // 车体 vx/vy/wz 任一触及上限
    bool wheel = false;  // 单轮速度触及上限（等比缩放已触发）
  };

  /** @brief 施加车体速度和真实四轮速度幅值限制，并记录饱和来源。 */
  Control clampControl(const Control &control,
                       SaturationReport *report = nullptr) const;
  /** @brief 计算四轮轮心速度中的最大模长，用于执行器速度门限。 */
  double maxModuleSpeed(const Control &control) const;
  /** @brief 将车体 Twist 映射为四个轮心的车体系二维速度向量。 */
  std::array<Eigen::Vector2d, 4> moduleVelocities(const Control &control) const;
  /** @brief 独立检查候选相对上一控制的轮速、轮加速度和有效舵角速率。 */
  bool moduleIncrementFeasible(const Control &candidate, const Control &previous,
                               double step_dt) const;
  /** @brief 在上一控制到目标控制之间二分寻找一个真实可达的增量。 */
  Control clampIncrement(const Control &target, const Control &previous,
                         double step_dt,
                         SaturationReport *report = nullptr,
                         bool *increment_limited = nullptr) const;
  /** @brief 生成有效参考控制 warm start，并逐步投影到真实执行器约束内。 */
  void initializeControls(const std::vector<Se2Reference> &references,
                          const Control &last_control,
                          double first_step_dt);

  Se2MpcConfig config_;
  std::vector<Control> warm_controls_;
  bool has_warm_start_ = false;
  SaturationReport first_step_saturation_;
  bool first_step_increment_limited_ = false;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
