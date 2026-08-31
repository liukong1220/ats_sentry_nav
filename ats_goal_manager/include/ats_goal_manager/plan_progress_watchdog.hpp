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
  // 占据位姿逃逸仍是实验能力。没有来自规划器、且绑定 snapshot identity
  // 的结构化授权时，默认保持 fail-closed。
  bool ego_blocked_escape_enabled{false};
  // 自身位姿被占据时允许的有界等待上限，只覆盖 robot_cell_free 与
  // footprint_safe 两个条件。地图/定位/TF 类条件不受此上限影响。
  double ego_blocked_escape_timeout_sec{3.0};
  // 目标已激活、基础设施健康,但机器人拿不到任何
  // "可执行"计划时的有界等待。两种表现属于同一类:
  // 要么从未拿到参考轨迹,要么参考还在但急停压着。
  // 下面的 stall/replan 预算全部挂在 observeReference 之后,而这两种表现
  // 各自都有一条 kNone 早退,不合并成一个有界预算就会留下 livelock 入口。
  //
  // 证据一(无参考):domain 189 目标 9 连续 91 次 FAILURE_FOOTPRINT,
  // 位姿 180 s 不动,goal manager 全程零输出,最终由 runner 超时判失败。
  // 证据二(参考在但急停不放):domain 193 目标 8 停在 (8.10, -8.08),
  // 观测到的 reference 恒为 1,MINCO 每 2 s 拒同一条轨迹,急停窗口
  // 152 s 未闭合,内部没有任何一条路径能结束这个目标。
  //
  // 取值按同一次运行里"合法"的最长等待反推。domain 193 十三个已闭合急停
  // 窗口里,首个 57.17 s 属于目标派发前的启动预热(基础设施分支已单独
  // 排除),其余 12 个目标内窗口最长 11.56 s。30 s 约 2.6 倍余量,同时把
  // 病态目标从外部超时压到 30 s 内的带原因失败。
  double no_executable_plan_timeout_sec{30.0};
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

// 目标被瞬时故障挂回 kWaitingForMap 时的等待门。挂起态按定义
// 没有参考轨迹,所以这里不带 reference/distance 字段。
struct SuspendedPlanGate
{
  bool goal_active{false};
  bool cancel_or_preempt{false};
  // 地图源存活 = 心跳租约未过期且最后一次心跳自报 ready。
  // 刻意不用瞬时的 map ready 标志:挂起本身会把那个标志清掉,
  // 拿它当健康条件会让预算每个心跳周期归零。
  bool map_source_alive{false};
  bool localization_fresh{false};
  PlanProgressIdentity identity;
};

enum class PlanProgressDecision : std::uint8_t
{
  kNone,
  kReplan,
  kExhausted,
  kUnsafe,
  // 基础设施健康却始终没有可执行计划:有界等待用尽,
  // 干净失败而不是无界等待。
  kNoExecutablePlan,
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
    params_.ego_blocked_escape_timeout_sec = std::max(
      0.0, params_.ego_blocked_escape_timeout_sec);
    params_.no_executable_plan_timeout_sec = std::max(
      0.0, params_.no_executable_plan_timeout_sec);
  }

  void resetGoal(std::uint64_t goal_id)
  {
    goal_id_ = goal_id;
    consecutive_replans_ = 0;
    active_identity_.reset();
    last_progress_distance_m_.reset();
    last_progress_time_.reset();
    last_replan_time_.reset();
    ego_blocked_since_.reset();
    ego_escape_active_ = false;
    no_executable_plan_since_.reset();
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
    no_executable_plan_since_.reset();
  }

  PlanProgressDecision evaluate(
    const PlanProgressGate & gate, Clock::time_point now = Clock::now())
  {
    if (!gate.goal_active || gate.cancel_or_preempt ||
      gate.identity.goal_id == 0 || gate.identity.goal_id != goal_id_)
    {
      return PlanProgressDecision::kNone;
    }
    // 这六个条件不同质，不能合并成同一个无界 fail-closed 分支。
    // map/localization/TF/inside_map 属于基础设施：等待本身可以恢复，而且这些
    // 条件不成立时连规划都不成立，保持无界停机是正确的。
    if (!gate.map_fresh || !gate.localization_fresh || !gate.tf_healthy ||
      !gate.robot_inside_map)
    {
      ego_blocked_since_.reset();
      ego_escape_active_ = false;
      // 基础设施等待本身是刻意无界的,不能计入无参考预算,
      // 否则地图/定位预热会被记成"规划器交付不出轨迹"。
      no_executable_plan_since_.reset();
      return PlanProgressDecision::kUnsafe;
    }
    // robot_cell_free/footprint_safe 描述的是机器人此刻已经占据的位姿。
    // 要离开该位姿只能靠执行器，而这里要停掉的正是那个执行器：
    // 无界拒绝无法把车挪走，唯一结果是 livelock
    // （domain 226 目标 1 连续 88 次 unsafe、33 个位姿散布 4 mm，
    // 直到 180 s 目标超时）。因此先给一段有界等待让地图自行澄清，
    // 超时后放行，交回下方既有的 stall/replan 预算兜底：
    // 车能挪出去就恢复推进，挪不出去就在 kReplan 用尽后
    // kExhausted 干净失败，两条路都是有界的。
    if (gate.robot_cell_free && gate.footprint_safe) {
      ego_blocked_since_.reset();
      ego_escape_active_ = false;
    } else {
      if (!params_.ego_blocked_escape_enabled) {
        ego_blocked_since_.reset();
        ego_escape_active_ = false;
        return PlanProgressDecision::kUnsafe;
      }
      if (!ego_blocked_since_ || now < *ego_blocked_since_) {
        ego_blocked_since_ = now;
      }
      if (!ego_escape_active_ &&
        std::chrono::duration<double>(now - *ego_blocked_since_).count() >=
          params_.ego_blocked_escape_timeout_sec)
      {
        ego_escape_active_ = true;
      }
      if (!ego_escape_active_) {
        return PlanProgressDecision::kUnsafe;
      }
    }
    // 三种表现合并成同一个有界预算:没有参考、参考在
    // 但急停压着、距离非有限。它们都表示"此刻没有可
    // 执行计划",而且原本各自是一条无界 kNone 早退。
    if (!gate.has_current_reference || gate.emergency_stop ||
      !std::isfinite(gate.distance_to_goal_m))
    {
      if (!no_executable_plan_since_ || now < *no_executable_plan_since_) {
        no_executable_plan_since_ = now;
      }
      if (std::chrono::duration<double>(now - *no_executable_plan_since_).count() >=
        params_.no_executable_plan_timeout_sec)
      {
        return PlanProgressDecision::kNoExecutablePlan;
      }
      return PlanProgressDecision::kNone;
    }
    // 参考在且急停已解除:这一拍确实可执行,预算清零。
    no_executable_plan_since_.reset();
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

  // kWaitingForMap 下的无界等待兜底。evaluate() 只在 kTracking 被调用,
  // 而瞬时故障(FOOTPRINT/START_ENCLOSED 等)会把目标挂回 kWaitingForMap。
  // dispatch->fail->suspend 每 2 s 一轮,每轮成功派发都会清掉
  // waiting_since,所以 map_wait_timeout_sec 的 5 s 预算永远不会到期:
  // 这类目标在 goal manager 内部没有任何上界,只能等外部目标超时,
  // 而超时不携带任何原因。
  //
  // 证据:domain 168 目标 4(START_ENCLOSED)与 domain 169 目标 9
  // (FOOTPRINT)分别跑满 179.6 s 与 176.0 s,期间 goal manager 只输出
  // ready=1 心跳。同一次 domain 169 运行里成功的目标 1..8 连续失败次数
  // 为 0,即合法的连续挂起时长是 0 s,与病态段 176 s/89 次完全可分。
  // domain 189 目标 9 的连续 91 次 FAILURE_FOOTPRINT 是同一形态。
  //
  // 预算与 evaluate() 共用 no_executable_plan_since_:"没有可执行计划"
  // 是同一个概念。共用同一个时钟还保证 tracking 与 waiting 之间的
  // churn 不清零预算,只有真正发布出参考(observeReference)或换目标
  // (resetGoal)才清零。
  PlanProgressDecision evaluateSuspended(
    const SuspendedPlanGate & gate, Clock::time_point now = Clock::now())
  {
    if (!gate.goal_active || gate.cancel_or_preempt ||
      gate.identity.goal_id == 0 || gate.identity.goal_id != goal_id_)
    {
      return PlanProgressDecision::kNone;
    }
    // 与 evaluate() 的基础设施分支同质:地图源或定位不健康时等待是
    // 刻意无界的,启动预热和注入故障不能记成"规划器交付不出轨迹"。
    // 这里只看这两个不被挂起动作清掉的信号,recovery snapshot 未就绪
    // 属于要度量的 churn 本身,不是免责条件。
    if (!gate.map_source_alive || !gate.localization_fresh) {
      no_executable_plan_since_.reset();
      return PlanProgressDecision::kUnsafe;
    }
    if (!no_executable_plan_since_ || now < *no_executable_plan_since_) {
      no_executable_plan_since_ = now;
    }
    if (std::chrono::duration<double>(now - *no_executable_plan_since_).count() >=
      params_.no_executable_plan_timeout_sec)
    {
      return PlanProgressDecision::kNoExecutablePlan;
    }
    return PlanProgressDecision::kNone;
  }

  [[nodiscard]] std::uint32_t consecutiveReplans() const
  {
    return consecutive_replans_;
  }

  // 自身位姿仍被占据，但有界等待已用尽，
  // policy 已放行让执行器把车挪出去。
  [[nodiscard]] bool egoEscapeActive() const
  {
    return ego_escape_active_;
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
  std::optional<Clock::time_point> no_executable_plan_since_;
  std::optional<PlanProgressIdentity> active_identity_;
  std::optional<double> last_progress_distance_m_;
  std::optional<Clock::time_point> last_progress_time_;
  std::optional<Clock::time_point> last_replan_time_;
  std::optional<Clock::time_point> ego_blocked_since_;
  bool ego_escape_active_{false};
};

}  // namespace ats_goal_manager

#endif  // ATS_GOAL_MANAGER__PLAN_PROGRESS_WATCHDOG_HPP_
