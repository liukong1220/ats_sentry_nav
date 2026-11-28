// Copyright 2026

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ats_swerve_mpc {

namespace {

constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 将上一拍车体系线速度搬运到旋转 +yaw_delta 后的新车体系（等价于惯性系恒速）。
 * @details 车体旋转 +dpsi 时，同一世界速度在新车体系下为 R(-dpsi) * v_prev；
 *          用于加速度限幅与平滑代价，避免小陀螺/边转边走时把向心/旋转坐标变换误判为车体加速度。
 */
Control rotateControlLinear(const Control &control, double yaw_delta) {
  return Se2Model::rotateControlLinear(control, yaw_delta);
}

/** @brief 计算内接正八边形（法向 pi/8 + m*pi/4）对应的等效最大轮速上限比。 */
double maxOctagonEquivalentSpeed(const std::array<Eigen::Vector2d, 4> &velocities) {
  const double cos_pi_8 = std::cos(kPi / 8.0);
  double max_equiv = 0.0;
  for (const auto &vel : velocities) {
    for (int m = 0; m < 4; ++m) {
      const double alpha = (kPi / 8.0) + static_cast<double>(m) * (kPi / 4.0);
      const double proj = std::abs(vel(0) * std::cos(alpha) + vel(1) * std::sin(alpha));
      max_equiv = std::max(max_equiv, proj / cos_pi_8);
    }
  }
  return max_equiv;
}

/**
 * @brief 计算与 LtvQpCandidateValidator 严格同源的旋转系速度+惯性加速度硬盒 [box_lo, box_hi]。
 * @details 盒内任意控制均严格满足 |u| <= max_v 且 |u - R(-wz*dt)*u_prev| <= max_a * dt；
 *          因盒为凸集，盒内任意两点的凸组合必然仍留在盒内。
 */
void computeRotatingAccelBox(const Control &previous, double step_dt,
                             const Se2MpcConfig &config,
                             Control &carried_previous, Control &box_lo,
                             Control &box_hi) {
  const Control max_delta(config.max_ax * step_dt,
                          config.max_ay * step_dt,
                          config.max_awz * step_dt);
  carried_previous = rotateControlLinear(previous, previous(2) * step_dt);
  const Control max_vel(config.max_vx, config.max_vy, config.max_wz);
  for (int axis = 0; axis < 3; ++axis) {
    box_lo(axis) = std::max(-max_vel(axis), carried_previous(axis) - max_delta(axis));
    box_hi(axis) = std::min(max_vel(axis), carried_previous(axis) + max_delta(axis));
    if (box_lo(axis) > box_hi(axis)) {
      const double clamped = std::clamp(carried_previous(axis), -max_vel(axis), max_vel(axis));
      box_lo(axis) = clamped;
      box_hi(axis) = clamped;
    }
  }
}

Control clampToBox(const Control &u, const Control &box_lo, const Control &box_hi) {
  Control out;
  for (int axis = 0; axis < 3; ++axis) {
    out(axis) = std::clamp(u(axis), box_lo(axis), box_hi(axis));
  }
  return out;
}

/**
 * @brief 检查控制量的多步可持续可行性（持久可行域/viability）：
 *        高速平移同时高速自旋时，四舵轮在惯性直行滑行下的向心滚速加速度
 *        a_cent = (v_lin * r_max * wz^2) / hypot(v_lin, r_max * |wz|) 不得超过单轮驱动加速度上限，
 *        且旋转不变轮速上界 v_lin + r_max * |wz| <= max_wheel_speed，
 *        防止预测时域某一步进入“下一步在旋转系加速度盒内无可行轮速增量”的死胡同。
 *        纯直线高速（wz=0）与纯原地自旋（v_lin=0）均零受限。
 */
bool isPersistentlyViable(const Control &u, const Se2MpcConfig &config) {
  const double r_max =
      std::hypot(std::max(0.0, config.wheel_base_x), std::max(0.0, config.wheel_base_y));
  if (r_max <= 1e-6) {
    return true;
  }
  const double v_lin = std::hypot(u(0), u(1));
  const double w = std::abs(u(2));
  if (v_lin <= 1e-6 || w <= 1e-6) {
    return true;
  }
  const double rw = r_max * w;
  if (config.max_wheel_acceleration > 1e-6) {
    const double a_cent = (v_lin * rw * w) / std::hypot(v_lin, rw);
    if (a_cent > 0.90 * config.max_wheel_acceleration) {
      return false;
    }
  }
  if (config.max_wheel_speed > 1e-6 &&
      (v_lin + rw) > config.max_wheel_speed + 1e-9) {
    return false;
  }
  return true;
}

}  // namespace

/** @brief 只把 yaw 前推 yaw_rate * delay / 2；无补偿量时原样返回以保持逐位一致。 */
State rotationPhaseCompensatedState(const State &measured, double yaw_rate,
                                    double delay) {
  const double phase = 0.5 * yaw_rate * delay;
  if (phase == 0.0 || !std::isfinite(phase)) {
    return measured;
  }
  State compensated = measured;
  compensated(2) = Se2Model::normalizeAngle(measured(2) + phase);
  return compensated;
}

/** @brief 用配置初始化四舵轮约束投影器与 SQP-RTI warm-start 状态。 */
Se2MpcController::Se2MpcController(const Se2MpcConfig &config)
    : config_(config) {}

/** @brief 切换控制配置后清除旧 horizon 的 warm start。 */
void Se2MpcController::setConfig(const Se2MpcConfig &config) {
  config_ = config;
  reset();
}

/** @brief 清空上次求解序列，防止急停、重定位或新轨迹复用过期控制。 */
void Se2MpcController::reset() {
  warm_controls_.clear();
  has_warm_start_ = false;
}

/**
 * @brief 四舵轮模块级约束是否完全生效。
 * @return true=半轴偏置与单轮速度上限均已配置；false=配置缺失，仅车体级限幅可用。
 * @note 由控制节点在参数加载后与运行期调用，用于输出实车高危配置告警。
 */
bool Se2MpcController::moduleLimitsActive() const {
  return config_.wheel_base_x > 0.0 && config_.wheel_base_y > 0.0 &&
         config_.max_wheel_speed > 1e-6;
}

Eigen::Matrix3d Se2MpcController::stateCostMatrix(
    const Se2MpcConfig &config, const Se2Reference &reference, bool terminal) {
  Eigen::Matrix3d weight =
      (terminal ? config.terminal_weight : config.state_weight).asDiagonal();
  if (terminal) {
    return weight;
  }
  const double ql =
      (std::isfinite(config.lag_weight) && config.lag_weight >= 0.0)
          ? config.lag_weight
          : config.state_weight(0);
  const double qc =
      (std::isfinite(config.contour_weight) && config.contour_weight >= 0.0)
          ? config.contour_weight
          : config.state_weight(1);
  if (std::abs(ql - qc) <= 1e-9) {
    weight(0, 0) = ql;
    weight(1, 1) = qc;
    return weight;
  }
  // 全向自旋底盘（小陀螺）的轨迹切向由世界系参考线速度决定，不能直接用自旋中的车体 yaw。
  const double ref_yaw = reference.state(2);
  const double cy = std::cos(ref_yaw);
  const double sy = std::sin(ref_yaw);
  const double vw_x =
      cy * reference.control(0) - sy * reference.control(1);
  const double vw_y =
      sy * reference.control(0) + cy * reference.control(1);
  const double speed = std::hypot(vw_x, vw_y);
  if (speed <= 0.05) {
    // 原地转向（含窄洞口 180° 掉头）无沿迹/横迹之分，XY 全向均按高横迹权重锁定转轴，
    // 防止转至 90° 时沿车体 X 轴退化为低 lag_weight 在坡道底板上侧滑。
    weight(0, 0) = qc;
    weight(1, 1) = qc;
    return weight;
  }
  const double theta_ref = std::atan2(vw_y, vw_x);
  const double ct = std::cos(theta_ref);
  const double st = std::sin(theta_ref);
  weight(0, 0) = ql * ct * ct + qc * st * st;
  weight(0, 1) = (ql - qc) * st * ct;
  weight(1, 0) = weight(0, 1);
  weight(1, 1) = ql * st * st + qc * ct * ct;
  return weight;
}

std::vector<State> Se2MpcController::rolloutTrajectory(
    const State &initial, const std::vector<Control> &controls,
    const Control &last_control, double first_step_dt) const {
  return Se2Model::rolloutWithLag(initial, controls, last_control,
                                  first_step_dt, config_.dt,
                                  config_.actuator_lag_tau_sec);
}

Control Se2MpcController::clampStepIncrement(
    const Control &target, const Control &previous, double step_dt,
    bool *body_saturated, bool *wheel_saturated,
    bool *increment_limited) const {
  SaturationReport report;
  const Control clamped =
      clampIncrement(target, previous, step_dt, &report, increment_limited);
  if (body_saturated != nullptr) {
    *body_saturated = report.body;
  }
  if (wheel_saturated != nullptr) {
    *wheel_saturated = report.wheel;
  }
  return clamped;
}

std::vector<Control> Se2MpcController::buildNominalControls(
    const std::vector<Se2Reference> &references, const Control &last_control,
    double first_step_dt, bool octagon_wheel_clamp,
    bool *first_step_body_saturated, bool *first_step_wheel_saturated,
    bool *first_step_increment_limited) {
  const double effective_first_dt =
      (std::isfinite(first_step_dt) && first_step_dt > 0.0) ? first_step_dt
                                                            : config_.dt;
  first_step_saturation_ = SaturationReport{};
  first_step_increment_limited_ = false;
  initializeControls(references, last_control, effective_first_dt);
  std::vector<Control> nominal = warm_controls_;
  if (octagon_wheel_clamp && config_.max_wheel_speed > 1e-6) {
    Control previous = last_control;
    for (int step = 0; step < config_.horizon; ++step) {
      const std::size_t idx = static_cast<std::size_t>(step);
      Control candidate = nominal[idx];
      const double oct_speed =
          maxOctagonEquivalentSpeed(moduleVelocities(candidate));
      if (oct_speed > config_.max_wheel_speed) {
        candidate *= (config_.max_wheel_speed * (1.0 - 1e-6)) / oct_speed;
        if (step == 0) {
          first_step_saturation_.wheel = true;
        }
      }
      const double step_dt = step == 0 ? effective_first_dt : config_.dt;
      SaturationReport step_sat;
      bool step_inc = false;
      candidate = clampIncrement(candidate, previous, step_dt, &step_sat, &step_inc);
      const double post_oct =
          maxOctagonEquivalentSpeed(moduleVelocities(candidate));
      if (post_oct > config_.max_wheel_speed) {
        // 在旋转系加速度盒内寻找满足八边形与模块增量的锚点，沿 anchor -> candidate 收缩，
        // 避免 previous 落在盒外时把名义控制拉出惯性加速度约束盒。
        Control carried_prev;
        Control box_lo;
        Control box_hi;
        computeRotatingAccelBox(previous, step_dt, config_, carried_prev,
                                box_lo, box_hi);
        const std::array<Control, 3> oct_anchors{{
            clampToBox(previous, box_lo, box_hi),
            clampToBox(carried_prev, box_lo, box_hi),
            clampToBox(Control::Zero(), box_lo, box_hi),
        }};
        bool found_anchor = false;
        Control anchor = oct_anchors[0];
        for (const auto &a : oct_anchors) {
          if (maxOctagonEquivalentSpeed(moduleVelocities(a)) <=
                  config_.max_wheel_speed &&
              moduleIncrementFeasible(a, previous, step_dt)) {
            anchor = a;
            found_anchor = true;
            break;
          }
        }
        if (found_anchor) {
          double lo = 0.0;
          double hi = 1.0;
          for (int it = 0; it < 16; ++it) {
            const double mid = 0.5 * (lo + hi);
            const Control trial = anchor + mid * (candidate - anchor);
            if (maxOctagonEquivalentSpeed(moduleVelocities(trial)) <=
                    config_.max_wheel_speed &&
                moduleIncrementFeasible(trial, previous, step_dt)) {
              lo = mid;
            } else {
              hi = mid;
            }
          }
          candidate = anchor + lo * (candidate - anchor);
        } else {
          candidate = clampToBox(
              candidate * ((config_.max_wheel_speed * (1.0 - 1e-6)) / post_oct),
              box_lo, box_hi);
        }
      }
      if (step == 0) {
        first_step_saturation_.body =
            first_step_saturation_.body || step_sat.body;
        first_step_saturation_.wheel =
            first_step_saturation_.wheel || step_sat.wheel;
        first_step_increment_limited_ =
            first_step_increment_limited_ || step_inc;
      }
      nominal[idx] = candidate;
      previous = candidate;
    }
  }
  if (first_step_body_saturated != nullptr) {
    *first_step_body_saturated = first_step_saturation_.body;
  }
  if (first_step_wheel_saturated != nullptr) {
    *first_step_wheel_saturated = first_step_saturation_.wheel;
  }
  if (first_step_increment_limited != nullptr) {
    *first_step_increment_limited = first_step_increment_limited_;
  }
  return nominal;
}

void Se2MpcController::commitWarmControls(
    const std::vector<Control> &controls) {
  if (static_cast<int>(controls.size()) == config_.horizon) {
    warm_controls_ = controls;
    has_warm_start_ = true;
  }
}

/**
 * @brief 将车体控制量限制在速度与单轮速度边界内。
 * @param control 待限幅的车体系控制 [vx, vy, wz]。
 * @param report  可选输出：记录本次是否触及车体级/轮级上限，供饱和日志使用。
 * @return 限幅后的控制量。
 * @note 每个控制周期在 clampIncrement 与 solve 内被反复调用。
 *       轮级限幅采用整体等比缩放，保证四轮速度向量仍来自同一车体 Twist，
 *       否则会破坏舵轮的瞬心一致性（实车表现为轮胎互相拖拽、打滑）。
 */
Control Se2MpcController::clampControl(const Control &control,
                                       SaturationReport *report) const {
  Control clamped = control;
  clamped(0) = std::clamp(clamped(0), -config_.max_vx, config_.max_vx);
  clamped(1) = std::clamp(clamped(1), -config_.max_vy, config_.max_vy);
  clamped(2) = std::clamp(clamped(2), -config_.max_wz, config_.max_wz);
  if (report != nullptr &&
      ((clamped - control).lpNorm<Eigen::Infinity>() > 1e-9)) {
    report->body = true;
  }
  const double module_speed = maxModuleSpeed(clamped);
  if (config_.max_wheel_speed > 1e-6 &&
      module_speed > config_.max_wheel_speed) {
    clamped *= config_.max_wheel_speed / module_speed;
    if (report != nullptr) {
      report->wheel = true;
    }
  }
  return clamped;
}

/**
 * @brief 计算给定车体控制下四个舵轮模块中的最大轮速。
 * @param control 车体系控制 [vx, vy, wz]。
 * @return 四轮线速度模长的最大值 [m/s]。
 * @note 数学关系：v_i = [vx - wz*y_i, vy + wz*x_i]，其中 (x_i, y_i) 为轮心相对
 *       底盘中心的位置。半轴偏置未配置（<=0）时按 0 处理，四轮退化为车体平移速度，
 *       此时函数返回 hypot(vx, vy)，仍能让单轮速度上限约束住平移分量。
 *       此前实现直接返回 0.0，会让 clampControl 的轮速判据永远不成立，
 *       等于把执行器边界整体关闭——属于"仿真通过实车超速"的典型来源。
 */
double Se2MpcController::maxModuleSpeed(const Control &control) const {
  double maximum = 0.0;
  for (const auto &velocity : moduleVelocities(control)) {
    maximum = std::max(maximum, velocity.norm());
  }
  return maximum;
}

/**
 * @brief 按底盘几何把 [vx,vy,wz] 映射为四个轮心速度向量。
 * @details 每个轮速为 [vx-wz*y_i, vy+wz*x_i]；该向量是轮速、增量和舵角速率
 *          约束的共同物理依据，禁止把四轮独立改写成不一致的 Twist。
 */
std::array<Eigen::Vector2d, 4>
Se2MpcController::moduleVelocities(const Control &control) const {
  const double x = std::max(0.0, config_.wheel_base_x);
  const double y = std::max(0.0, config_.wheel_base_y);
  const std::array<Eigen::Vector2d, 4> positions{{
      Eigen::Vector2d(x, y),
      Eigen::Vector2d(x, -y),
      Eigen::Vector2d(-x, y),
      Eigen::Vector2d(-x, -y),
  }};
  std::array<Eigen::Vector2d, 4> velocities;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    velocities[index] << control(0) - control(2) * positions[index](1),
        control(1) + control(2) * positions[index](0);
  }
  return velocities;
}

/**
 * @brief 判断候选控制相对上一控制是否满足四舵轮执行器增量约束。
 * @param candidate 候选车体控制 [vx, vy, wz]。
 * @param previous  上一周期实际下发的车体控制。
 * @return true=四个模块的轮速、轮加速度、舵角速率都在一个 dt 内可达。
 * @note 由 clampIncrement 的二分线搜索反复调用。
 *       舵角变化量取两次轮速向量的真实夹角
 *       \f$ \Delta\theta = \arccos\frac{|v_k \cdot v_{k-1}|}{\|v_k\|\|v_{k-1}\|} \f$，
 *       与舵轮底盘 `optimize_steer_angle` 一致（反向时翻转驱动滚速符号而非旋转舵向角 180°）；
 *       高速反向由滚速标量增量 `candidate_speed + previous_speed <= max_wheel_acceleration * dt`
 *       强制先减速过零，低速小幅反向与启动不再被死锁在 1e-4 角度锥内。
 */
bool Se2MpcController::moduleIncrementFeasible(
    const Control &candidate, const Control &previous, double step_dt) const {
  const auto candidate_velocities = moduleVelocities(candidate);
  const auto previous_velocities = moduleVelocities(previous);
  const double max_speed_delta = config_.max_wheel_acceleration * step_dt;
  const double max_steer_delta = config_.max_steer_rate * step_dt;
  constexpr double kSteerAngleMinSpeed = 0.01;  // 与 ZeroSpeedGuardConfig::enter_threshold 对齐
  for (std::size_t index = 0; index < candidate_velocities.size(); ++index) {
    const double candidate_speed = candidate_velocities[index].norm();
    const double previous_speed = previous_velocities[index].norm();
    if (config_.max_wheel_speed > 1e-6 &&
        candidate_speed > config_.max_wheel_speed + 1e-9) {
      return false;
    }
    double cosine = 1.0;
    const bool both_moving = candidate_speed > 1e-4 && previous_speed > 1e-4;
    if (both_moving) {
      cosine = std::clamp(
          candidate_velocities[index].dot(previous_velocities[index]) /
              (candidate_speed * previous_speed),
          -1.0, 1.0);
      if (config_.max_steer_rate > 1e-6 &&
          candidate_speed > kSteerAngleMinSpeed &&
          previous_speed > kSteerAngleMinSpeed &&
          std::acos(std::abs(cosine)) > max_steer_delta + 1e-9) {
        return false;
      }
    }
    if (config_.max_wheel_acceleration > 1e-6) {
      // 舵轮驱动电机只加速轮子滚动标量速度，方向变化由转向电机按 max_steer_rate 承担；
      // 避免将旋转参考系下的轮速向量偏转误算为驱动加速度（与 sim_node 物理执行器一致）。
      const double rolling_speed_delta = both_moving
          ? (cosine >= 0.0 ? std::abs(candidate_speed - previous_speed)
                           : (candidate_speed + previous_speed))
          : (candidate_velocities[index] - previous_velocities[index]).norm();
      if (rolling_speed_delta > max_speed_delta + 1e-9) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 将控制增量限制在车体加速度与四舵轮执行器可达范围内。
 * @param target            期望的车体控制 [vx, vy, wz]。
 * @param previous          上一周期实际下发的车体控制。
 * @param step_dt           相邻两拍控制量之间的真实时间步长 [s]。
 * @param report            可选输出：车体级/轮级速度限幅是否命中。
 * @param increment_limited 可选输出：是否因模块增量不可达而被线搜索回退。
 * @return 一个 step_dt 内实车可达的车体控制。
 * @note 每步预测与每次线搜索都会调用。先将上一拍线速度按 R(-wz*dt) 搬运至当前车体系，
 *       再按真实惯性加速度上限裁剪 Δv 并做速度限幅；若模块级增量仍不可达，
 *       则在 previous→limited 线段上做二分搜索，保证四轮速度向量始终来自同一车体 Twist。
 */
Control Se2MpcController::clampIncrement(const Control &target,
                                         const Control &previous,
                                         double step_dt,
                                         SaturationReport *report,
                                         bool *increment_limited) const {
  const Control max_delta(config_.max_ax * step_dt,
                          config_.max_ay * step_dt,
                          config_.max_awz * step_dt);
  Control carried_previous;
  Control box_lo;
  Control box_hi;
  computeRotatingAccelBox(previous, step_dt, config_, carried_previous,
                          box_lo, box_hi);

  // 记录车体速度上限与单轮速度上限触及诊断（与原语义一致）。
  Control raw_in_accel = carried_previous;
  for (int axis = 0; axis < 3; ++axis) {
    raw_in_accel(axis) +=
        std::clamp(target(axis) - carried_previous(axis), -max_delta(axis), max_delta(axis));
  }
  const Control box_target = clampToBox(target, box_lo, box_hi);
  if (report != nullptr) {
    if (std::abs(raw_in_accel(0)) > config_.max_vx + 1e-9 ||
        std::abs(raw_in_accel(1)) > config_.max_vy + 1e-9 ||
        std::abs(raw_in_accel(2)) > config_.max_wz + 1e-9) {
      report->body = true;
    }
    if (config_.max_wheel_speed > 1e-6 &&
        maxModuleSpeed(box_target) > config_.max_wheel_speed + 1e-9) {
      report->wheel = true;
    }
  }

  const auto isStrictlyAdmissible = [this, &previous, step_dt](const Control &u) {
    return moduleIncrementFeasible(u, previous, step_dt) &&
           isPersistentlyViable(u, config_);
  };

  // 快速路径：盒内目标直接满足全部模块增量与可持续可行性。
  if (isStrictlyAdmissible(box_target)) {
    return box_target;
  }

  // 若仅因轮速超限未通过，尝试单轮缩放并投影回旋转系加速度盒。
  if (config_.max_wheel_speed > 1e-6 &&
      maxModuleSpeed(box_target) > config_.max_wheel_speed + 1e-9) {
    const Control wheel_clamped =
        clampToBox(clampControl(box_target, nullptr), box_lo, box_hi);
    if (isStrictlyAdmissible(wheel_clamped)) {
      return wheel_clamped;
    }
  }

  if (increment_limited != nullptr) {
    *increment_limited = true;
  }

  // 计算单点模块级违约量（仅在极罕见回退选优时使用）。
  const auto previous_velocities = moduleVelocities(previous);
  const auto moduleViolation = [this, &previous_velocities,
                                step_dt](const Control &cand) -> double {
    const auto cand_vel = moduleVelocities(cand);
    const double max_speed_delta = config_.max_wheel_acceleration * step_dt;
    const double max_steer_delta = config_.max_steer_rate * step_dt;
    constexpr double kSteerAngleMinSpeed = 0.01;
    double viol = 0.0;
    for (std::size_t i = 0; i < cand_vel.size(); ++i) {
      const double cs = cand_vel[i].norm();
      const double ps = previous_velocities[i].norm();
      if (config_.max_wheel_speed > 1e-6) {
        viol = std::max(viol, cs - config_.max_wheel_speed);
      }
      double cosine = 1.0;
      const bool both_moving = cs > 1e-4 && ps > 1e-4;
      if (both_moving) {
        cosine = std::clamp(cand_vel[i].dot(previous_velocities[i]) / (cs * ps),
                            -1.0, 1.0);
        if (config_.max_steer_rate > 1e-6 && cs > kSteerAngleMinSpeed &&
            ps > kSteerAngleMinSpeed) {
          viol = std::max(viol, std::acos(std::abs(cosine)) - max_steer_delta);
        }
      }
      if (config_.max_wheel_acceleration > 1e-6) {
        const double rd = both_moving
                              ? (cosine >= 0.0 ? std::abs(cs - ps) : (cs + ps))
                              : (cand_vel[i] - previous_velocities[i]).norm();
        viol = std::max(viol, rd - max_speed_delta);
      }
    }
    return viol;
  };

  // 在凸盒 [box_lo, box_hi] 内寻找可行锚点，避免高速转弯时 previous 落于盒外导致
  // 二分回退破坏旋转系惯性加速度硬约束。
  const Control p_box = clampToBox(previous, box_lo, box_hi);
  const Control cp_box = clampToBox(carried_previous, box_lo, box_hi);
  const Control z_box = clampToBox(Control::Zero(), box_lo, box_hi);
  const std::array<Control, 8> anchors{{
      p_box,
      cp_box,
      clampToBox(clampControl(p_box, nullptr), box_lo, box_hi),
      clampToBox(clampControl(cp_box, nullptr), box_lo, box_hi),
      Control(p_box(0), p_box(1), z_box(2)),
      Control(cp_box(0), cp_box(1), z_box(2)),
      Control(z_box(0), z_box(1), p_box(2)),
      z_box,
  }};

  bool found_strict_anchor = false;
  Control anchor = p_box;
  Control best_mod_cand = p_box;
  double best_mod_viol = 1e9;
  for (const auto &c : anchors) {
    if (isStrictlyAdmissible(c)) {
      anchor = c;
      found_strict_anchor = true;
      break;
    }
    const double v = moduleViolation(c);
    if (v < best_mod_viol) {
      best_mod_viol = v;
      best_mod_cand = c;
    }
  }

  // 极罕见兜底：对盒内 3x3x3 网格做低开销扫描，确保只要可行交集非空必找到盒内锚点。
  if (!found_strict_anchor) {
    constexpr std::array<double, 3> kFracs{{0.0, 0.5, 1.0}};
    for (double fx : kFracs) {
      const double ux = box_lo(0) + fx * (box_hi(0) - box_lo(0));
      for (double fy : kFracs) {
        const double uy = box_lo(1) + fy * (box_hi(1) - box_lo(1));
        for (double fz : kFracs) {
          const double uz = box_lo(2) + fz * (box_hi(2) - box_lo(2));
          const Control c(ux, uy, uz);
          if (isStrictlyAdmissible(c)) {
            anchor = c;
            found_strict_anchor = true;
            break;
          }
          const double v = moduleViolation(c);
          if (v < best_mod_viol) {
            best_mod_viol = v;
            best_mod_cand = c;
          }
        }
        if (found_strict_anchor) {
          break;
        }
      }
      if (found_strict_anchor) {
        break;
      }
    }
  }

  const int bisection_iters = std::max(1, config_.clamp_bisection_iterations);
  if (found_strict_anchor) {
    // 沿盒内锚点 anchor -> box_target 二分：两端点均在凸盒内，全线段恒满足惯性加速度与速度盒约束。
    double feasible_scale = 0.0;
    double infeasible_scale = 1.0;
    for (int iteration = 0; iteration < bisection_iters; ++iteration) {
      const double scale = 0.5 * (feasible_scale + infeasible_scale);
      const Control candidate = anchor + scale * (box_target - anchor);
      if (isStrictlyAdmissible(candidate)) {
        feasible_scale = scale;
      } else {
        infeasible_scale = scale;
      }
    }
    return anchor + feasible_scale * (box_target - anchor);
  }

  // 若上一控制本身来自外部非持久可行状态，仍优先保证当拍硬模块约束与盒约束。
  if (best_mod_viol <= 1e-9) {
    double feasible_scale = 0.0;
    double infeasible_scale = 1.0;
    for (int iteration = 0; iteration < bisection_iters; ++iteration) {
      const double scale = 0.5 * (feasible_scale + infeasible_scale);
      const Control candidate =
          best_mod_cand + scale * (box_target - best_mod_cand);
      if (moduleIncrementFeasible(candidate, previous, step_dt)) {
        feasible_scale = scale;
      } else {
        infeasible_scale = scale;
      }
    }
    return best_mod_cand + feasible_scale * (box_target - best_mod_cand);
  }
  return best_mod_cand;
}

/**
 * @brief 构造或移动 SQP-RTI warm start，并逐步投影到真实四轮执行器可达集合。
 * @details 首步单独记录饱和/增量回退，因为只有它会进入 `/cmd_vel/autonomy_raw`。
 */
void Se2MpcController::initializeControls(
    const std::vector<Se2Reference> &references, const Control &last_control,
    double first_step_dt) {
  if (!has_warm_start_ ||
      static_cast<int>(warm_controls_.size()) != config_.horizon) {
    warm_controls_.assign(static_cast<std::size_t>(config_.horizon),
                          Control::Zero());
  } else if (config_.horizon > 1) {
    // 按真实经过的控制步长 first_step_dt / config_.dt 连续前移 warm start，
    // 避免高频控制周期（如 50 Hz）下盲目左移一整个预测步长 dt。
    const double shift = std::clamp(first_step_dt / config_.dt, 0.0, 1.0);
    if (shift >= 1.0 - 1e-9) {
      std::rotate(warm_controls_.begin(), warm_controls_.begin() + 1,
                  warm_controls_.end());
      warm_controls_.back() = warm_controls_[warm_controls_.size() - 2];
    } else if (shift > 1e-9) {
      for (std::size_t index = 0; index + 1 < warm_controls_.size(); ++index) {
        warm_controls_[index] =
            (1.0 - shift) * warm_controls_[index] +
            shift * warm_controls_[index + 1];
      }
    }
  }
  Control previous = last_control;
  const double blend = std::clamp(config_.warm_start_blend_ratio, 0.0, 1.0);
  for (int step = 0; step < config_.horizon; ++step) {
    const Control seed =
        has_warm_start_
            ? blend * warm_controls_[static_cast<std::size_t>(step)] +
                  (1.0 - blend) * references[static_cast<std::size_t>(step)].control
            : references[static_cast<std::size_t>(step)].control;
    // 仅第 0 步是本周期真正下发的控制，按实际经过步长 first_step_dt 限幅并记录饱和诊断。
    SaturationReport *report = step == 0 ? &first_step_saturation_ : nullptr;
    bool *increment_limited =
        step == 0 ? &first_step_increment_limited_ : nullptr;
    const double step_dt = step == 0 ? first_step_dt : config_.dt;
    warm_controls_[static_cast<std::size_t>(step)] =
        clampIncrement(seed, previous, step_dt, report, increment_limited);
    previous = warm_controls_[static_cast<std::size_t>(step)];
  }
  has_warm_start_ = true;
}

} // namespace ats_swerve_mpc
