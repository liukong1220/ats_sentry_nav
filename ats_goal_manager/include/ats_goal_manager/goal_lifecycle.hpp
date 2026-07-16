// Copyright 2026

#ifndef ATS_GOAL_MANAGER__GOAL_LIFECYCLE_HPP_
#define ATS_GOAL_MANAGER__GOAL_LIFECYCLE_HPP_

#include <cstdint>

namespace ats_goal_manager
{

// 该类不接触 ROS I/O，只约束任务生命周期和每个状态的急停要求，便于单测锁定。
enum class GoalLifecycleState : std::uint8_t
{
  kIdle,
  kWaitingForMap,
  kPlanning,
  kTracking,
  kSucceeded,
  kCanceled,
  kPreempted,
  kTimedOut,
  kFailed,
};

class GoalLifecycle
{
public:
  void start(std::uint64_t goal_id, bool map_ready)
  {
    goal_id_ = goal_id;
    active_ = true;
    state_ = map_ready ? GoalLifecycleState::kPlanning : GoalLifecycleState::kWaitingForMap;
  }

  bool mapReady(std::uint64_t goal_id) const
  {
    return active_ && goal_id_ == goal_id && state_ == GoalLifecycleState::kWaitingForMap;
  }

  bool referenceReady(std::uint64_t goal_id, bool map_ready)
  {
    if (!active_ || goal_id_ != goal_id || !map_ready || state_ != GoalLifecycleState::kPlanning) {
      return false;
    }
    state_ = GoalLifecycleState::kTracking;
    return true;
  }

  bool mapBecameReady(std::uint64_t goal_id)
  {
    if (!mapReady(goal_id)) {
      return false;
    }
    state_ = GoalLifecycleState::kPlanning;
    return true;
  }

  void cancel()
  {
    finish(GoalLifecycleState::kCanceled);
  }

  void preempt()
  {
    finish(GoalLifecycleState::kPreempted);
  }

  void timeout()
  {
    finish(GoalLifecycleState::kTimedOut);
  }

  void fail()
  {
    finish(GoalLifecycleState::kFailed);
  }

  void succeed()
  {
    finish(GoalLifecycleState::kSucceeded);
  }

  [[nodiscard]] bool active() const {return active_;}
  [[nodiscard]] std::uint64_t goalId() const {return goal_id_;}
  [[nodiscard]] GoalLifecycleState state() const {return state_;}
  [[nodiscard]] bool emergencyStopRequired() const
  {
    return state_ != GoalLifecycleState::kTracking;
  }

private:
  void finish(GoalLifecycleState state)
  {
    state_ = state;
    active_ = false;
  }

  std::uint64_t goal_id_{0};
  bool active_{false};
  GoalLifecycleState state_{GoalLifecycleState::kIdle};
};

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__GOAL_LIFECYCLE_HPP_
