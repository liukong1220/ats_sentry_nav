// Copyright 2026
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace ats_goal_manager {

// All stamps use the node's ROS clock. Wall time must never release this gate:
// the reference consumer compares ROS timestamps, even when /clock is paused.
class StopReferenceBarrier {
public:
  static double boundedGuardSeconds(double seconds = 0.01) {
    return std::isfinite(seconds) ? std::clamp(seconds, 0.002, 0.1) : 0.01;
  }

  void recordStop(std::int64_t stamp_ns, double guard_seconds) {
    not_before_ns_.store(
      stamp_ns + static_cast<std::int64_t>(boundedGuardSeconds(guard_seconds) * 1e9),
      std::memory_order_release);
  }

  bool permits(std::int64_t stamp_ns) const {
    return stamp_ns > 0 && stamp_ns >= not_before_ns_.load(std::memory_order_acquire);
  }

private:
  std::atomic<std::int64_t> not_before_ns_{0};
};

}  // namespace ats_goal_manager
