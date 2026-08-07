// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MODEL_HPP_
#define ATS_SWERVE_MPC__SE2_MODEL_HPP_

#include <vector>

#include <Eigen/Core>

namespace ats_swerve_mpc {

using State = Eigen::Vector3d;
using Control = Eigen::Vector3d;

/**
 * @brief Shared holonomic SE(2) prediction model.
 *
 * State is world-frame [x, y, yaw]. Control is body-frame [vx, vy, wz].
 * The model is intentionally independent of ROS and solver implementation so
 * iLQR and LTV-QP can use the same dynamics, Jacobians and rollout.
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
   * @brief 按车体系 Twist 将世界系位姿推进一个离散周期。
   * @param state 当前世界系 [x,y,yaw]。
   * @param control 当前车体系 [vx,vy,wz]。
   * @return 推进并将 yaw 归一化到 [-pi,pi] 后的状态。
   */
  State dynamics(const State &state, const Control &control) const;
  /**
   * @brief 计算离散动力学对状态和控制的局部 Jacobian，供 iLQR/LTV-QP 线性化。
   */
  void jacobians(const State &state, const Control &control,
                 Eigen::Matrix3d &state_jacobian,
                 Eigen::Matrix3d &control_jacobian) const;
  /** @brief 从同一个初始状态按控制序列生成完整状态 rollout。 */
  std::vector<State> rollout(const State &initial,
                             const std::vector<Control> &controls) const;

  /** @brief 将任意航向角映射到 [-pi,pi]，保证跨边界比较连续。 */
  static double normalizeAngle(double angle);
  /** @brief 计算 SE(2) 状态差，并对 yaw 使用最短角差。 */
  static State stateDifference(const State &lhs, const State &rhs);

private:
  double dt_ = 0.1;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__SE2_MODEL_HPP_
