// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MODEL_HPP_
#define ATS_SWERVE_MPC__SE2_MODEL_HPP_

#include <vector>

#include <Eigen/Core>

namespace ats_swerve_mpc {

using State = Eigen::Vector3d;
using Control = Eigen::Vector3d;

/**
 * @brief iLQR 与 LTV-QP 共用的全向 SE(2) 离散预测模型。
 *
 * @details 状态固定为世界系 `[x,y,yaw]`，控制固定为车体系 `[vx,vy,wz]`。
 *          该模型刻意不依赖 ROS、OSQP 或节点状态：iLQR 名义轨迹、LTV 线性化和
 *          QP candidate 非线性复核均必须调用同一套动力学、Jacobian 与 rollout，
 *          从而避免不同实现造成 frame 或离散时间轴分叉。
 */
class Se2Model {
public:
  /** @brief 创建固定离散步长的共享 SE(2) 预测模型。 */
  explicit Se2Model(double dt = 0.1) : dt_(dt) {}

  /** @brief 更新离散步长；调用者必须保证它与控制周期一致。 */
  void setTimeStep(double dt) { dt_ = dt; }
  /** @brief 返回当前模型使用的离散步长（秒）。 */
  double timeStep() const { return dt_; }

  /**
   * @brief 按车体系 Twist 将世界系位姿推进一个离散周期（SE(2) 解析指数映射积分）。
   * @param state 当前世界系 [x,y,yaw]。
   * @param control 当前车体系 [vx,vy,wz]。
   * @return 推进并将 yaw 归一化到 [-pi,pi] 后的状态。
   */
  State dynamics(const State &state, const Control &control) const;
  /** @brief 使用显式积分步长 dt 的 SE(2) 解析指数映射离散动力学（对任意 dt 恒定速度零离散误差）。 */
  static State dynamics(const State &state, const Control &control, double dt);
  /**
   * @brief 计算离散动力学对状态和控制的局部 Jacobian，供 iLQR/LTV-QP 线性化。
   */
  void jacobians(const State &state, const Control &control,
                 Eigen::Matrix3d &state_jacobian,
                 Eigen::Matrix3d &control_jacobian) const;
  /** @brief 使用显式积分步长 dt 的 SE(2) 指数映射解析 Jacobian（含 sinc 与 wz 对平移耦合）。 */
  static void jacobians(const State &state, const Control &control, double dt,
                        Eigen::Matrix3d &state_jacobian,
                        Eigen::Matrix3d &control_jacobian);
  /** @brief 从同一个初始状态按控制序列生成完整状态 rollout。 */
  std::vector<State> rollout(const State &initial,
                             const std::vector<Control> &controls) const;

  /** @brief 将车体系线速度搬运到旋转 +yaw_delta 后的新车体系 R(-yaw_delta)*[vx,vy]。 */
  static Control rotateControlLinear(const Control &control, double yaw_delta);
  /** @brief 计算 rotateControlLinear(u, u_z * step_dt) 对控制向量 u 的 3x3 解析 Jacobian。 */
  static Eigen::Matrix3d rotateControlLinearJacobian(const Control &control,
                                                     double step_dt);
  /** @brief 计算一阶执行器滞后在步长 dt 内的平均作用权重 gamma in (0, 1]（tau<=0 返回 1）。 */
  static double actuatorLagGamma(double dt, double lag_tau_sec);
  /** @brief 计算考虑一阶执行器滞后与旋转系搬运后的等效车体系控制。 */
  static Control effectiveControlWithLag(const Control &control,
                                         const Control &previous_control,
                                         double previous_carry_dt,
                                         double step_dt,
                                         double lag_tau_sec);
  /** @brief 带一阶执行器滞后的单步 SE(2) 指数映射离散动力学（tau<=0 退化为精确无滞后模型）。 */
  static State dynamicsWithLag(const State &state, const Control &control,
                               const Control &previous_control,
                               double previous_carry_dt, double dt,
                               double lag_tau_sec);
  /** @brief 带一阶执行器滞后的解析 Jacobian：返回 A = df/dx, B_curr = df/du_k, B_prev = df/du_{k-1}。 */
  static void jacobiansWithLag(const State &state, const Control &control,
                               const Control &previous_control,
                               double previous_carry_dt, double dt,
                               double lag_tau_sec,
                               Eigen::Matrix3d &state_jacobian,
                               Eigen::Matrix3d &current_control_jacobian,
                               Eigen::Matrix3d &previous_control_jacobian);
  /** @brief 带一阶执行器滞后的共享模型完整状态 rollout。 */
  static std::vector<State> rolloutWithLag(const State &initial,
                                           const std::vector<Control> &controls,
                                           const Control &last_control,
                                           double first_step_dt, double dt,
                                           double lag_tau_sec);

  /** @brief 将任意航向角映射到 [-pi,pi]，保证跨边界比较连续。 */
  static double normalizeAngle(double angle);
  /** @brief 计算 SE(2) 状态差，并对 yaw 使用最短角差。 */
  static State stateDifference(const State &lhs, const State &rhs);

private:
  double dt_ = 0.1;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__SE2_MODEL_HPP_
