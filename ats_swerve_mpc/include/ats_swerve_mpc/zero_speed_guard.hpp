// Copyright 2026

#ifndef ATS_SWERVE_MPC__ZERO_SPEED_GUARD_HPP_
#define ATS_SWERVE_MPC__ZERO_SPEED_GUARD_HPP_

#include <array>
#include <algorithm>
#include <cmath>

namespace ats_swerve_mpc {

struct ZeroSpeedGuardConfig {
  // 轮速降至该值及以下时进入“方向未定义”保护区 [m/s]。
  double enter_threshold = 0.01;
  // 轮速升至该值及以上才退出保护区；必须高于 enter_threshold 以形成滞回。
  double exit_threshold = 0.02;
};

/**
 * @brief 防止零轮速附近伪造舵角方向的滞回保护器。
 *
 * @details 轮心速度向量模长接近零时没有有意义的方向，因此不能据此线性化或复核
 *          舵角速率。该类维护进入/退出死区的滞回状态，避免速度在阈值附近抖动时
 *          反复切换方向模式。它只跳过方向角约束，不会跳过轮速或轮速度向量增量硬约束。
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

  /** @brief 返回当前是否处于零速方向未定义区间，供审计记录跳过原因。 */
  bool active() const { return active_; }

  /**
   * @brief 判定一对前后轮速向量是否允许计算真实舵角变化。
   * @details 只有保护器未激活且历史、候选速度均大于进入阈值时返回 true；任一向量
   *          位于死区时必须返回 false，调用者不得以零向量或默认舵角伪造方向约束。
   */
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
  /** @brief 归一化死区配置，确保退出阈值严格高于进入阈值以保留滞回。 */
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
