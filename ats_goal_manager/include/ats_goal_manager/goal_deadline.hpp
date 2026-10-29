// Copyright 2026

#ifndef ATS_GOAL_MANAGER__GOAL_DEADLINE_HPP_
#define ATS_GOAL_MANAGER__GOAL_DEADLINE_HPP_

#include <algorithm>
#include <cstdint>

namespace ats_goal_manager
{

// 目标超时按节点时钟计：use_sim_time 下是 /clock，实车上 ROS 时间就是墙钟。
// 慢仿真（0.057 倍实时）下按墙钟计会让 600 s 的目标只剩约 34 仿真秒。
// 该类不接触 ROS，输入是节点时钟的纳秒数，便于单测锁定。
// - 时钟还没开始走（use_sim_time 且尚未收到 /clock，now_ns <= 0）时不计时，
//   第一次读到正的时钟才开始计，否则 /clock 到达的那一刻会把整个仿真时间算成已用时间。
// - 时钟倒退（仿真重启、bag 回绕）时从当前时刻重新计，不会因为负的已用时间永不超时，
//   也不会因为回绕前的起点误判超时。
class GoalDeadline
{
public:
  GoalDeadline() = default;

  GoalDeadline(std::int64_t now_ns, std::int64_t timeout_ns)
  : timeout_ns_(std::max<std::int64_t>(0, timeout_ns))
  {
    rearm(now_ns);
  }

  // 返回已用时间（纳秒，>= 0）；按上面的规则开始或重新开始计时。
  std::int64_t elapsed(std::int64_t now_ns)
  {
    if (!armed_ || now_ns < started_ns_) {
      rearm(now_ns);
    }
    return armed_ ? now_ns - started_ns_ : 0;
  }

  bool expired(std::int64_t now_ns)
  {
    const std::int64_t used = elapsed(now_ns);
    return armed_ && used >= timeout_ns_;
  }

  [[nodiscard]] bool armed() const {return armed_;}
  [[nodiscard]] std::int64_t timeoutNs() const {return timeout_ns_;}

private:
  void rearm(std::int64_t now_ns)
  {
    armed_ = now_ns > 0;
    started_ns_ = armed_ ? now_ns : 0;
  }

  std::int64_t started_ns_{0};
  std::int64_t timeout_ns_{0};
  bool armed_{false};
};

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__GOAL_DEADLINE_HPP_
