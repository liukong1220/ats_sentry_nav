// Copyright 2026

#ifndef ATS_SWERVE_MPC__ZERO_SPEED_GUARD_HPP_
#define ATS_SWERVE_MPC__ZERO_SPEED_GUARD_HPP_

#include <array>
#include <algorithm>
#include <cmath>

namespace ats_swerve_mpc {

struct ZeroSpeedGuardConfig {
  double enter_threshold = 0.01;
  double exit_threshold = 0.02;
};

/**
 * @brief Hysteretic guard for the undefined steering direction at zero wheel speed.
 *
 * A wheel velocity vector has no meaningful angle when its norm is near zero.
 * The guard therefore exposes whether angle-rate linearization is valid and
 * keeps a hysteresis state to prevent direction-mode chatter around the deadband.
 */
class ZeroSpeedGuard {
public:
  /** @brief 创建带进入/退出滞回的轮速零点方向保护器。 */
  explicit ZeroSpeedGuard(const ZeroSpeedGuardConfig &config = {})
      : config_(config) {
    normalizeConfig();
  }

  /** @brief 更新死区参数并重置状态，避免旧速度历史跨配置沿用。 */
  void setConfig(const ZeroSpeedGuardConfig &config) {
    config_ = config;
    normalizeConfig();
    active_ = false;
  }

  /** @brief 用四轮速度更新滞回状态；低速时所有轮向角都视为未定义。 */
  bool update(const std::array<double, 4> &module_speeds) {
    double maximum = 0.0;
    for (const double speed : module_speeds) {
      maximum = std::max(maximum, std::abs(speed));
    }
    if (active_) {
      if (maximum >= config_.exit_threshold) {
        active_ = false;
      }
    } else if (maximum <= config_.enter_threshold) {
      active_ = true;
    }
    return active_;
  }

  /** @brief 返回当前是否处于零速方向未定义区间。 */
  bool active() const { return active_; }

  /**
   * @brief Angle-rate constraints are valid only when both vectors have direction.
   */
  /** @brief 仅当历史和候选轮向量均脱离死区时允许检查舵角速率。 */
  bool angleConstraintDefined(double previous_speed,
                              double candidate_speed) const {
    return !active_ && previous_speed > config_.enter_threshold &&
           candidate_speed > config_.enter_threshold;
  }

  /** @brief 返回进入零速保护的速度阈值。 */
  double enterThreshold() const { return config_.enter_threshold; }
  /** @brief 返回退出零速保护的速度阈值。 */
  double exitThreshold() const { return config_.exit_threshold; }

private:
  /** @brief 归一化配置，确保 hysteresis 退出阈值严格高于进入阈值。 */
  void normalizeConfig() {
    config_.enter_threshold = std::max(1e-6, config_.enter_threshold);
    config_.exit_threshold =
        std::max(config_.enter_threshold + 1e-6, config_.exit_threshold);
  }

  ZeroSpeedGuardConfig config_;
  bool active_ = false;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__ZERO_SPEED_GUARD_HPP_
