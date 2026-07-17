// Copyright 2026

#ifndef ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
#define ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_

#include <array>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace ats_swerve_mpc {

using State = Eigen::Vector3d;
using Control = Eigen::Vector3d;

struct Se2MpcConfig {
  int horizon = 20;
  double dt = 0.1;
  Eigen::Vector3d state_weight{18.0, 18.0, 4.0};
  Eigen::Vector3d control_weight{0.12, 0.12, 0.08};
  Eigen::Vector3d control_delta_weight{0.7, 0.7, 0.25};
  Eigen::Vector3d terminal_weight{28.0, 28.0, 8.0};
  double max_vx = 2.0;
  double max_vy = 2.0;
  double max_wz = 2.5;
  double max_ax = 2.5;
  double max_ay = 2.5;
  double max_awz = 4.0;
  // 轮心相对底盘中心的半轴偏置，不是完整前后/左右轮距。
  double wheel_base_x = 0.0;
  double wheel_base_y = 0.0;
  double max_wheel_speed = 0.0;
  double max_wheel_acceleration = 0.0;
  double max_steer_rate = 0.0;
  int max_iterations = 6;
  double regularization = 1e-5;
  double line_search_decay = 0.5;
  double min_line_search_step = 0.05;
  double convergence_tolerance = 1e-3;
};

struct Se2Reference {
  State state = State::Zero();
  Control control = Control::Zero();
};

struct Se2MpcResult {
  bool success = false;
  std::vector<Control> controls;
  std::vector<State> states;
  int iterations = 0;
  double cost = 0.0;
  double solve_time_ms = 0.0;
};

class Se2MpcController {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Se2MpcController(const Se2MpcConfig &config = Se2MpcConfig());

  Se2MpcResult solve(const State &current_state,
                     const std::vector<Se2Reference> &references,
                     const Control &last_control);
  void setConfig(const Se2MpcConfig &config);
  void reset();

  const Se2MpcConfig &config() const { return config_; }

private:
  using Matrix3 = Eigen::Matrix3d;

  struct BackwardResult {
    bool success = false;
    std::vector<Control> feedforward;
    std::vector<Matrix3> feedback;
  };

  static double normalizeAngle(double angle);
  static State stateDifference(const State &lhs, const State &rhs);
  State dynamics(const State &state, const Control &control) const;
  void jacobians(const State &state, const Control &control,
                 Matrix3 &state_jacobian, Matrix3 &control_jacobian) const;
  Control clampControl(const Control &control) const;
  double maxModuleSpeed(const Control &control) const;
  std::array<Eigen::Vector2d, 4> moduleVelocities(const Control &control) const;
  bool moduleIncrementFeasible(const Control &candidate, const Control &previous) const;
  Control clampIncrement(const Control &target, const Control &previous) const;
  std::vector<State> rollout(const State &initial,
                             const std::vector<Control> &controls) const;
  double cost(const std::vector<State> &states,
              const std::vector<Control> &controls,
              const std::vector<Se2Reference> &references,
              const Control &last_control) const;
  BackwardResult backwardPass(const std::vector<State> &states,
                              const std::vector<Control> &controls,
                              const std::vector<Se2Reference> &references,
                              const Control &last_control) const;
  void initializeControls(const std::vector<Se2Reference> &references,
                          const Control &last_control);

  Se2MpcConfig config_;
  std::vector<Control> warm_controls_;
  bool has_warm_start_ = false;
};

} // namespace ats_swerve_mpc

#endif // ATS_SWERVE_MPC__SE2_MPC_CONTROLLER_HPP_
