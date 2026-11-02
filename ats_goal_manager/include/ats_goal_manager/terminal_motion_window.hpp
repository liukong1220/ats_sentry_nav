// Copyright 2026

#ifndef ATS_GOAL_MANAGER__TERMINAL_MOTION_WINDOW_HPP_
#define ATS_GOAL_MANAGER__TERMINAL_MOTION_WINDOW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

namespace ats_goal_manager
{

// 终点"已停稳"的速度判据所用的位姿窗口。
//
// 里程计 twist 在车静止时也有噪声：MuJoCo lio 模式下 Point-LIO 的 /odometry 速度在车已停住
// (真值速度中位数 0.003 m/s、MPC 输出 0) 时中位数 0.064 m/s、p90 0.136 m/s，按 0.05 m/s 的
// 瞬时判据只有约 37 % 的采样通过，最长连续通过 0.2 s，凑不满 0.5 s 停留 (e5 d124：位置与 yaw
// 全程在容差内 25 s，仍因这一项判失败)。同一段数据里窗口内位移折算的平均速度中位数 0.011 m/s、
// p95 0.024 m/s。
//
// 这里只看"窗口内位姿变化了多少"，不看 twist：位置噪声是零均值的几毫米，而真实的运动会在窗口
// 内累积成位移。窗口宽度由调用方给；时间用里程计消息的时间戳，与墙钟无关。
class TerminalMotionWindow
{
public:
  // window_sec <= 0 表示关闭，调用方退回瞬时 twist 判据。
  void setWindow(double window_sec)
  {
    window_sec_ = std::isfinite(window_sec) ? std::max(0.0, window_sec) : 0.0;
    if (window_sec_ <= 0.0) {
      samples_.clear();
    }
  }

  [[nodiscard]] bool enabled() const {return window_sec_ > 0.0;}

  void clear() {samples_.clear();}

  void add(double stamp_sec, double x, double y, double yaw)
  {
    if (!enabled() || !std::isfinite(stamp_sec) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(yaw))
    {
      return;
    }
    if (!samples_.empty()) {
      if (stamp_sec < samples_.back().t - 1e-6) {
        // 时间倒退(仿真重置、时钟切换)：旧窗口不可比。
        samples_.clear();
      } else if (stamp_sec <= samples_.back().t) {
        return;  // 同一条里程计消息重复进来。
      }
    }
    samples_.push_back({stamp_sec, x, y, yaw});
    // 留一个比窗口更老的锚点。
    while (samples_.size() > 2U && samples_[1].t <= stamp_sec - 2.0 * window_sec_) {
      samples_.pop_front();
    }
    while (samples_.size() > kMaxSamples) {
      samples_.pop_front();
    }
  }

  // 最近一个窗口内的平均线速度与平均角速度(位移/时间)。历史还没覆盖 80 % 窗口时返回 false，
  // 调用方按"尚未停稳"处理。
  [[nodiscard]] bool averageSpeed(double & linear_mps, double & angular_rps) const
  {
    if (!enabled() || samples_.size() < 2U) {
      return false;
    }
    const Sample & newest = samples_.back();
    const Sample * anchor = &samples_.front();
    for (const Sample & sample : samples_) {
      if (sample.t <= newest.t - window_sec_) {
        anchor = &sample;
      } else {
        break;
      }
    }
    const double span = newest.t - anchor->t;
    if (span < kMinCoverage * window_sec_) {
      return false;
    }
    const double dyaw = std::atan2(
      std::sin(newest.yaw - anchor->yaw), std::cos(newest.yaw - anchor->yaw));
    linear_mps = std::hypot(newest.x - anchor->x, newest.y - anchor->y) / span;
    angular_rps = std::abs(dyaw) / span;
    return true;
  }

private:
  struct Sample
  {
    double t;
    double x;
    double y;
    double yaw;
  };

  static constexpr double kMinCoverage = 0.8;
  static constexpr std::size_t kMaxSamples = 4096U;

  std::deque<Sample> samples_;
  double window_sec_{0.0};
};

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__TERMINAL_MOTION_WINDOW_HPP_
