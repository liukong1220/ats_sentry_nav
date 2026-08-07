// Copyright 2026

#ifndef ATS_SWERVE_MPC__EMERGENCY_STOP_WATCHDOG_HPP_
#define ATS_SWERVE_MPC__EMERGENCY_STOP_WATCHDOG_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

namespace ats_swerve_mpc
{

class EmergencyStopWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  /**
   * @brief 创建急停心跳看门狗，初始状态保守地要求停止。
   * @details 只有收到新鲜的非停止心跳后才可能解除停止；启动阶段绝不因缺少消息而
   *          假设输入健康。
   */
  explicit EmergencyStopWatchdog(double timeout_sec = 0.5)
  {
    setTimeout(timeout_sec);
  }

  /**
   * @brief 更新 emergency-stop 心跳超时，并将非法非正值夹紧为 1 ms。
   * @details 超时使用 steady-clock 纳秒数保存，避免 ROS 时间跳变把过期输入误判为新鲜。
   */
  void setTimeout(double timeout_sec)
  {
    const double bounded_timeout = timeout_sec > 0.0 ? timeout_sec : 0.001;
    timeout_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(bounded_timeout)).count(),
      std::memory_order_release);
  }

  /**
   * @brief 写入一次急停/解除急停心跳及其单调时间戳。
   * @details `stop=true` 立即置位；`stop=false` 仅在本消息已记录时间后解除显式停止，
   *          后续仍必须通过 stopRequired 的新鲜度复核。
   */
  void update(bool stop, Clock::time_point signal_time = Clock::now())
  {
    if (stop) {
      stop_requested_.store(true, std::memory_order_release);
    }
    last_signal_ns_.store(toNanoseconds(signal_time), std::memory_order_release);
    if (!stop) {
      stop_requested_.store(false, std::memory_order_release);
    }
  }

  /**
   * @brief 判断当前控制周期是否必须发送确定性零速度。
   * @details 显式急停、从未收到心跳、时钟倒退或超过 timeout 都返回 true；该函数是
   *          iLQR 和 qp_shadow 共享的上游 fail-stop 门，而非 QP 的可选诊断。
   */
  bool stopRequired(Clock::time_point current_time = Clock::now()) const
  {
    if (stop_requested_.load(std::memory_order_acquire)) {
      return true;
    }
    const std::int64_t last_signal = last_signal_ns_.load(std::memory_order_acquire);
    const std::int64_t current = toNanoseconds(current_time);
    return last_signal <= 0 || current < last_signal ||
           current - last_signal > timeout_ns_.load(std::memory_order_acquire);
  }

private:
  /** @brief 将 steady-clock 时间点无损转换为原子保存的纳秒计数。 */
  static std::int64_t toNanoseconds(Clock::time_point time)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      time.time_since_epoch()).count();
  }

  std::atomic<bool> stop_requested_{true};
  std::atomic<std::int64_t> last_signal_ns_{0};
  std::atomic<std::int64_t> timeout_ns_{500000000};
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__EMERGENCY_STOP_WATCHDOG_HPP_
