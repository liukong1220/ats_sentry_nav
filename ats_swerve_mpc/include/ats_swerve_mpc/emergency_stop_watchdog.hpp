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

  explicit EmergencyStopWatchdog(double timeout_sec = 0.5)
  {
    setTimeout(timeout_sec);
  }

  void setTimeout(double timeout_sec)
  {
    const double bounded_timeout = timeout_sec > 0.0 ? timeout_sec : 0.001;
    timeout_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(bounded_timeout)).count(),
      std::memory_order_release);
  }

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
