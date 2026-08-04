// Copyright 2026

#ifndef ATS_GOAL_MANAGER__PLAN_PROGRESS_WATCHDOG_HPP_
#define ATS_GOAL_MANAGER__PLAN_PROGRESS_WATCHDOG_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

namespace ats_goal_manager
{

// Task-level policy only.  It deliberately has no ROS I/O and cannot publish a
// reference by itself; AtsGoalManagerNode remains the sole execution authority.
struct PlanProgressWatchdogParams
{
  double progress_min_delta_m{0.10};
  double replan_stall_timeout_sec{4.0};
  double replan_min_interval_sec{2.0};
  std::uint32_t max_consecutive_replans{2};
};

struct PlanProgressIdentity
{
  std::uint64_t goal_id{0};
  std::uint64_t localization_epoch{0};
  // These are intentionally distinct.  source_generation belongs to ROGMap,
  // map_publication_sequence belongs to the adapter, and map_generation is
  // MINCO's local immutable snapshot generation.
  std::uint64_t source_generation{0};
  std::uint64_t map_publication_sequence{0};
  std::uint64_t map_generation{0};
  std::uint64_t plan_request_sequence{0};
};

struct PlanProgressGate
{
  bool goal_active{false};
  bool cancel_or_preempt{false};
  bool emergency_stop{true};
  bool map_fresh{false};
  bool localization_fresh{false};
  bool tf_healthy{false};
  bool robot_inside_map{false};
  bool robot_cell_free{false};
  bool footprint_safe{false};
  bool has_current_reference{false};
  double distance_to_goal_m{0.0};
  PlanProgressIdentity identity;
};

enum class PlanProgressDecision : std::uint8_t
{
  kNone,
  kReplan,
  kExhausted,
  kUnsafe,
};

class PlanProgressWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  explicit PlanProgressWatchdog(
    PlanProgressWatchdogParams params = PlanProgressWatchdogParams())
  {
    setParams(params);
  }

  void setParams(const PlanProgressWatchdogParams & params)
  {
    params_ = params;
    params_.progress_min_delta_m = std::max(0.0, params_.progress_min_delta_m);
    params_.replan_stall_timeout_sec = std::max(0.01, params_.replan_stall_timeout_sec);
    params_.replan_min_interval_sec = std::max(0.0, params_.replan_min_interval_sec);
  }

  void resetGoal(std::uint64_t goal_id)
  {
    goal_id_ = goal_id;
    consecutive_replans_ = 0;
    active_identity_.reset();
    last_progress_distance_m_.reset();
    last_progress_time_.reset();
    last_replan_time_.reset();
  }

  void observeReference(
    const PlanProgressIdentity & identity, double distance_to_goal_m,
    Clock::time_point now = Clock::now())
  {
    if (identity.goal_id == 0 || identity.goal_id != goal_id_ ||
      !std::isfinite(distance_to_goal_m))
    {
      return;
    }
    active_identity_ = identity;
    last_progress_distance_m_ = std::max(0.0, distance_to_goal_m);
    last_progress_time_ = now;
  }

  PlanProgressDecision evaluate(
    const PlanProgressGate & gate, Clock::time_point now = Clock::now())
  {
    if (!gate.goal_active || gate.cancel_or_preempt ||
      gate.identity.goal_id == 0 || gate.identity.goal_id != goal_id_)
    {
      return PlanProgressDecision::kNone;
    }
    if (!gate.map_fresh || !gate.localization_fresh || !gate.tf_healthy ||
      !gate.robot_inside_map || !gate.robot_cell_free || !gate.footprint_safe)
    {
      return PlanProgressDecision::kUnsafe;
    }
    if (gate.emergency_stop || !gate.has_current_reference ||
      !std::isfinite(gate.distance_to_goal_m))
    {
      return PlanProgressDecision::kNone;
    }
    if (!active_identity_ || !sameGoalAndReference(*active_identity_, gate.identity)) {
      observeReference(gate.identity, gate.distance_to_goal_m, now);
      return PlanProgressDecision::kNone;
    }
    if (!last_progress_distance_m_ || !last_progress_time_) {
      observeReference(gate.identity, gate.distance_to_goal_m, now);
      return PlanProgressDecision::kNone;
    }

    const double distance = std::max(0.0, gate.distance_to_goal_m);
    if (*last_progress_distance_m_ - distance >= params_.progress_min_delta_m) {
      last_progress_distance_m_ = distance;
      last_progress_time_ = now;
      return PlanProgressDecision::kNone;
    }
    if (now < *last_progress_time_ ||
      std::chrono::duration<double>(now - *last_progress_time_).count() <
        params_.replan_stall_timeout_sec)
    {
      return PlanProgressDecision::kNone;
    }
    if (last_replan_time_ && now >= *last_replan_time_ &&
      std::chrono::duration<double>(now - *last_replan_time_).count() <
        params_.replan_min_interval_sec)
    {
      return PlanProgressDecision::kNone;
    }
    if (consecutive_replans_ >= params_.max_consecutive_replans) {
      return PlanProgressDecision::kExhausted;
    }
    ++consecutive_replans_;
    last_replan_time_ = now;
    last_progress_time_ = now;
    last_progress_distance_m_ = distance;
    return PlanProgressDecision::kReplan;
  }

  [[nodiscard]] std::uint32_t consecutiveReplans() const
  {
    return consecutive_replans_;
  }

private:
  static bool sameGoalAndReference(
    const PlanProgressIdentity & left, const PlanProgressIdentity & right)
  {
    return left.goal_id == right.goal_id &&
      left.localization_epoch == right.localization_epoch &&
      left.map_generation == right.map_generation &&
      left.map_publication_sequence == right.map_publication_sequence &&
      left.plan_request_sequence == right.plan_request_sequence;
  }

  PlanProgressWatchdogParams params_;
  std::uint64_t goal_id_{0};
  std::uint32_t consecutive_replans_{0};
  std::optional<PlanProgressIdentity> active_identity_;
  std::optional<double> last_progress_distance_m_;
  std::optional<Clock::time_point> last_progress_time_;
  std::optional<Clock::time_point> last_replan_time_;
};

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__PLAN_PROGRESS_WATCHDOG_HPP_
