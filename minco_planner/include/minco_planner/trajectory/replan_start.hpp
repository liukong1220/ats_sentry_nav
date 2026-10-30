// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__REPLAN_START_HPP_
#define MINCO_PLANNER__TRAJECTORY__REPLAN_START_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

// 无停车重规划的新参考起点参数（调研报告附录 A.3）。
struct ReplanStartParams
{
  // 起点取已提交轨迹上"车的推进位置"再前推这么久处 [s]。要覆盖从取起点到新参考被 MPC
  // 采用之间车继续前行的时间（规划耗时 + 提交链路）；偏小则新参考起点仍落在车后面。
  double plan_lead_sec = 0.15;
  // 车到已提交轨迹的距离 >= 该值时放弃轨迹起点、退回实测 [m]；<=0 即关闭。
  double deviation_max_m = 0.5;
};

// 规划请求冻结的实测状态（规划栅格坐标系，世界系轴向）。
struct MeasuredStart
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
};

enum class ReplanStartSource
{
  kMeasured,
  kCommitted,
};

enum class ReplanStartReason
{
  kCommitted,       // 起点取自已提交轨迹
  kNoTrajectory,    // 没有正在执行的已提交轨迹（含急停后被撤销）
  kEpochMismatch,   // 定位 epoch 与已提交轨迹不同
  kDisabled,        // deviation_max_m <= 0
  kInvalidInput,    // 轨迹或实测含非有限值、点数不足
  kDeviation,       // 车离已提交轨迹 >= deviation_max_m
};

inline const char * replanStartReasonName(ReplanStartReason reason)
{
  switch (reason) {
    case ReplanStartReason::kCommitted: return "committed";
    case ReplanStartReason::kNoTrajectory: return "no_trajectory";
    case ReplanStartReason::kEpochMismatch: return "epoch_mismatch";
    case ReplanStartReason::kDisabled: return "disabled";
    case ReplanStartReason::kInvalidInput: return "invalid_input";
    case ReplanStartReason::kDeviation: return "deviation";
  }
  return "unknown";
}

struct ReplanStart
{
  ReplanStartSource source = ReplanStartSource::kMeasured;
  ReplanStartReason reason = ReplanStartReason::kNoTrajectory;
  // 新轨迹的起始状态。source=kMeasured 时就是实测位姿/速度，加速度为 0。
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  // 已提交轨迹时间轴（ReferencePoint::t）上的取点时刻 t_ref，以及车的推进时刻 progress [s]。
  // 没有走到轨迹那一步时为 NaN。
  double t_ref = std::numeric_limits<double>::quiet_NaN();
  double progress = std::numeric_limits<double>::quiet_NaN();
  // 车到已提交轨迹（推进位置处）的距离 [m]，没算到时为 NaN。
  double deviation = std::numeric_limits<double>::quiet_NaN();
  // 新轨迹 t=0 比"现在"晚多久 [s]，等于 t_ref - progress。车按时（progress = 墙钟）时，
  // 现在 + 该值 = traj.start + t_ref，即 A.3 的 t0；车落后时只算车自己的前行时间。
  double start_delay_sec = 0.0;
};

namespace replan_start_detail
{

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double lerp(double a, double b, double u)
{
  return a + u * (b - a);
}

}  // namespace replan_start_detail

// 已提交轨迹在时刻 t 的状态：相邻采样点间线性插值（MPC 对发布的 Path 也是这么取参考的），
// yaw 沿最短弧插值。t 越界时取端点。轨迹至少 2 个点、时间严格递增（ReferenceTrajectory::valid）。
inline ReferencePoint interpolateReferencePoint(const ReferenceTrajectory & trajectory, double t)
{
  const auto & points = trajectory.points;
  if (t <= points.front().t) {
    return points.front();
  }
  if (t >= points.back().t) {
    return points.back();
  }
  const auto upper = std::upper_bound(
    points.begin(), points.end(), t,
    [](double value, const ReferencePoint & point) {return value < point.t;});
  const ReferencePoint & a = *(upper - 1);
  const ReferencePoint & b = *upper;
  const double span = b.t - a.t;
  const double u = span > 1e-12 ? (t - a.t) / span : 0.0;
  ReferencePoint point = a;
  point.t = t;
  point.s = replan_start_detail::lerp(a.s, b.s, u);
  point.x = replan_start_detail::lerp(a.x, b.x, u);
  point.y = replan_start_detail::lerp(a.y, b.y, u);
  point.yaw = replan_start_detail::wrapAngle(
    a.yaw + u * replan_start_detail::wrapAngle(b.yaw - a.yaw));
  point.v = replan_start_detail::lerp(a.v, b.v, u);
  point.vx = replan_start_detail::lerp(a.vx, b.vx, u);
  point.vy = replan_start_detail::lerp(a.vy, b.vy, u);
  point.ax = replan_start_detail::lerp(a.ax, b.ax, u);
  point.ay = replan_start_detail::lerp(a.ay, b.ay, u);
  point.yaw_rate = replan_start_detail::lerp(a.yaw_rate, b.yaw_rate, u);
  return point;
}

// 选新参考的起点状态（调研报告附录 A.3）。
//   committed       正在执行的已提交轨迹；没有（或已被急停撤销）时传 nullptr。
//   elapsed_sec     取起点时刻相对 committed->header.stamp 的时间 [s]（即 t_now - traj.start）。
//   measured        规划请求冻结的实测状态，和 committed 在同一坐标系。
// 车在轨迹附近时：progress 取车沿轨迹的推进时刻（不超过 elapsed_sec，MPC 按几何投影跟踪，
// 拐角减速后车常落后于墙钟），t_ref = min(progress + plan_lead, 轨迹末端)，起点取轨迹在
// t_ref 的 p/v/a/yaw。车按时时 progress = elapsed_sec，即 A.3 的 t_ref = t_now - traj.start + lead。
// 其余情况（无轨迹、epoch 不同、偏离 >= deviation_max_m、输入非法）返回实测，a = 0。
inline ReplanStart selectReplanStart(
  const ReferenceTrajectory * committed, std::uint64_t committed_epoch,
  std::uint64_t request_epoch, double elapsed_sec, const MeasuredStart & measured,
  const ReplanStartParams & params)
{
  ReplanStart result;
  result.x = measured.x;
  result.y = measured.y;
  result.yaw = measured.yaw;
  result.vx = measured.vx;
  result.vy = measured.vy;
  if (committed == nullptr) {
    result.reason = ReplanStartReason::kNoTrajectory;
    return result;
  }
  if (committed_epoch != request_epoch) {
    result.reason = ReplanStartReason::kEpochMismatch;
    return result;
  }
  if (!(params.deviation_max_m > 0.0)) {
    result.reason = ReplanStartReason::kDisabled;
    return result;
  }
  if (!committed->valid() || !std::isfinite(elapsed_sec) || !std::isfinite(measured.x) ||
    !std::isfinite(measured.y) || !std::isfinite(measured.yaw) || !std::isfinite(measured.vx) ||
    !std::isfinite(measured.vy))
  {
    result.reason = ReplanStartReason::kInvalidInput;
    return result;
  }
  const double t_front = committed->points.front().t;
  const double t_back = committed->points.back().t;
  const double elapsed = std::min(std::max(elapsed_sec, t_front), t_back);
  result.progress = projectedReferenceTime(*committed, measured.x, measured.y, elapsed);
  const ReferencePoint on_track = interpolateReferencePoint(*committed, result.progress);
  result.deviation = std::hypot(on_track.x - measured.x, on_track.y - measured.y);
  if (!(result.deviation < params.deviation_max_m)) {
    result.reason = ReplanStartReason::kDeviation;
    return result;
  }
  result.t_ref = std::min(result.progress + std::max(0.0, params.plan_lead_sec), t_back);
  const ReferencePoint ahead = interpolateReferencePoint(*committed, result.t_ref);
  result.source = ReplanStartSource::kCommitted;
  result.reason = ReplanStartReason::kCommitted;
  result.x = ahead.x;
  result.y = ahead.y;
  result.yaw = ahead.yaw;
  result.vx = ahead.vx;
  result.vy = ahead.vy;
  result.ax = ahead.ax;
  result.ay = ahead.ay;
  result.start_delay_sec = result.t_ref - result.progress;
  return result;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__REPLAN_START_HPP_
