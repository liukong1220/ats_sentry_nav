// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
#define ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_

#include <array>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "ats_swerve_mpc/se2_model.hpp"

namespace ats_swerve_mpc {

/**
 * @brief 四舵轮底盘 SE(2) MPC（iLQR）配置。
 *
 * 所有量纲统一为 SI：长度 m、速度 m/s、加速度 m/s^2、角度 rad、角速度 rad/s。
 * 控制量为车体系 Twist [vx, vy, wz]，状态量为世界系位姿 [x, y, yaw]。
 * 每个动力学参数都必须与实车物理部件一一对应，标注见各字段注释。
 */
struct Se2MpcConfig {
  // 预测步数（无量纲）。horizon * dt 即预测时域长度，需覆盖实车一次制动距离。
  int horizon = 20;
  // 离散步长 [s]。必须与控制线程周期一致，否则预测时间轴与实际执行时间轴错位。
  double dt = 0.1;
  // 状态跟踪权重 [x, y, yaw]：位置误差权重大于航向，保证先贴合路径再修航向。
  Eigen::Vector3d state_weight{18.0, 18.0, 4.0};
  // 控制幅值权重 [vx, vy, wz]：抑制无谓大速度，等价于能耗惩罚。
  Eigen::Vector3d control_weight{0.12, 0.12, 0.08};
  // 控制增量权重 [dvx, dvy, dwz]：抑制指令抖振，直接决定舵轮舵角的平滑度。
  Eigen::Vector3d control_delta_weight{0.7, 0.7, 0.25};
  // 终端状态权重：比 state_weight 更大，保证时域末端收敛到参考点。
  Eigen::Vector3d terminal_weight{28.0, 28.0, 8.0};
  // 车体前向最大线速度 [m/s]。对应实车纵向可用速度上限（受电机额定转速约束）。
  double max_vx = 2.0;
  // 车体侧向最大线速度 [m/s]。四舵轮为全向底盘，横向能力与纵向同级。
  double max_vy = 2.0;
  // 车体最大偏航角速度 [rad/s]。实车受舵轮转向速率与轮速上限共同限制。
  double max_wz = 2.5;
  // 车体纵向最大加速度 [m/s^2]。对应轮胎-地胶附着极限与电机扭矩上限的较小者。
  double max_ax = 2.5;
  // 车体侧向最大加速度 [m/s^2]，物理含义同 max_ax，方向为车体 y。
  double max_ay = 2.5;
  // 车体最大偏航角加速度 [rad/s^2]。超过该值实车会出现舵轮打滑与航向过冲。
  double max_awz = 4.0;
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
  // iLQR 单周期最大迭代次数（无量纲）。上限受控制周期实时预算约束。
  int max_iterations = 6;
  // Q_uu 正则化系数，防止低速工况下反向递推奇异。
  double regularization = 1e-5;
  // 线搜索步长衰减比例（0~1）。
  double line_search_decay = 0.5;
  // 线搜索最小步长，低于该值判定本次迭代无法下降代价。
  double min_line_search_step = 0.05;
  // 收敛判据：前馈更新量无穷范数小于该值即认为已收敛。
  double convergence_tolerance = 1e-3;
};

struct Se2Reference {
  State state = State::Zero();
  Control control = Control::Zero();
};

/**
 * @brief MPC 求解结果与实车诊断标志。
 *
 * success 语义：控制/状态序列长度合法、代价有限，并且本周期至少完成一次成功的
 * iLQR 反向递推（Q_uu 可分解、反馈/前馈增益全部有限）。反向递推失败时判定失败，
 * 因为此时返回的只是上一周期的暖启动序列，控制已失去反馈闭环。
 * 注意"线搜索未接受任何候选"不判失败：约束（车体速度/轮速/舵角速率）激活时，
 * 限幅会把候选压回当前序列，代价无法下降但当前序列仍是约束集内的可执行解。
 */
struct Se2MpcResult {
  bool success = false;
  std::vector<Control> controls;
  std::vector<State> states;
  int iterations = 0;
  // 被线搜索接受的迭代次数；为 0 表示本周期完全没有改进。
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

  /** @brief 创建 iLQR 控制器并初始化共享 SE(2) 模型和 warm-start 状态。 */
  explicit Se2MpcController(const Se2MpcConfig &config = Se2MpcConfig());

  /**
   * @brief 在当前状态和参考 horizon 上执行一次完整 iLQR 控制周期。
   * @details 该函数保留既有主控制链：先生成/复用控制序列，再做 rollout、
   *          backward pass、线搜索和真实四轮硬约束限幅，返回的首个控制才可发布。
   */
  Se2MpcResult solve(const State &current_state,
                     const std::vector<Se2Reference> &references,
                     const Control &last_control);
  /** @brief 替换动力学和代价配置，并清空与旧配置不兼容的 warm start。 */
  void setConfig(const Se2MpcConfig &config);
  /** @brief 清空控制序列和求解器内部状态，使下一周期回到确定性初始路径。 */
  void reset();

  /** @brief 只读返回当前生效的 MPC 参数，供 node 和 QP adapter 共用。 */
  const Se2MpcConfig &config() const { return config_; }

  /**
   * @brief 四舵轮模块级约束是否处于完全生效状态。
   * @return true 表示半轴偏置与单轮速度上限都已配置，模块级限幅真实生效；
   *         false 表示配置缺失，只能退化为车体平移速度限幅（实车高危）。
   * @note 供控制节点在启动与运行期打印中文告警，避免"仿真通过实车超速"。
   */
  bool moduleLimitsActive() const;

private:
  using Matrix3 = Eigen::Matrix3d;

  // 限幅命中情况，用于向上层输出饱和诊断。
  struct SaturationReport {
    bool body = false;   // 车体 vx/vy/wz 任一触及上限
    bool wheel = false;  // 单轮速度触及上限（等比缩放已触发）
  };

  struct BackwardResult {
    bool success = false;
    std::vector<Control> feedforward;
    std::vector<Matrix3> feedback;
  };

  /** @brief 对内部代价和 rollout 统一执行 yaw 归一化。 */
  static double normalizeAngle(double angle);
  /** @brief 计算 iLQR 代价使用的 SE(2) 最短状态误差。 */
  static State stateDifference(const State &lhs, const State &rhs);
  /** @brief 使用共享模型推进一个控制步，避免 iLQR 与 QP 动力学分叉。 */
  State dynamics(const State &state, const Control &control) const;
  /** @brief 计算 iLQR backward pass 所需的状态/控制线性化矩阵。 */
  void jacobians(const State &state, const Control &control,
                 Matrix3 &state_jacobian, Matrix3 &control_jacobian) const;
  /** @brief 施加车体速度和真实四轮速度幅值限制，并记录饱和来源。 */
  Control clampControl(const Control &control,
                       SaturationReport *report = nullptr) const;
  /** @brief 计算四轮轮心速度中的最大模长，用于执行器速度门限。 */
  double maxModuleSpeed(const Control &control) const;
  /** @brief 将车体 Twist 映射为四个轮心的车体系二维速度向量。 */
  std::array<Eigen::Vector2d, 4> moduleVelocities(const Control &control) const;
  /** @brief 独立检查候选相对上一控制的轮速、轮加速度和有效舵角速率。 */
  bool moduleIncrementFeasible(const Control &candidate, const Control &previous) const;
  /** @brief 在上一控制到目标控制之间二分寻找一个真实可达的增量。 */
  Control clampIncrement(const Control &target, const Control &previous,
                         SaturationReport *report = nullptr,
                         bool *increment_limited = nullptr) const;
  /** @brief 使用共享模型生成当前 iLQR 名义轨迹。 */
  std::vector<State> rollout(const State &initial,
                             const std::vector<Control> &controls) const;
  /** @brief 计算状态跟踪、控制幅值、控制增量和 terminal 总代价。 */
  double cost(const std::vector<State> &states,
              const std::vector<Control> &controls,
              const std::vector<Se2Reference> &references,
              const Control &last_control) const;
  /** @brief 对名义 rollout 执行 iLQR 反向递推，生成反馈/前馈增益。 */
  BackwardResult backwardPass(const std::vector<State> &states,
                              const std::vector<Control> &controls,
                              const std::vector<Se2Reference> &references,
                              const Control &last_control) const;
  /** @brief 生成有效参考控制 warm start，并逐步投影到真实执行器约束内。 */
  void initializeControls(const std::vector<Se2Reference> &references,
                          const Control &last_control);

  Se2MpcConfig config_;
  // iLQR 名义轨迹与 LTV-QP candidate 非线性复核共用此模型，禁止维护第二套动力学。
  Se2Model model_;
  std::vector<Control> warm_controls_;
  bool has_warm_start_ = false;
  // 本周期首步（即真正下发的那一步）限幅命中记录：在前向生成过程中累积，
  // 不能在求解结束后对已限幅的结果再限幅一次——那样永远得不到命中标志。
  SaturationReport first_step_saturation_;
  bool first_step_increment_limited_ = false;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
