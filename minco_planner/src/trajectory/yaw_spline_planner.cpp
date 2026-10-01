// Copyright 2026

#include "minco_planner/trajectory/yaw_spline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace minco_planner
{

YawSplinePlanner::YawSplinePlanner(YawSplinePlannerParams params) : params_(params) {}

void YawSplinePlanner::setParams(const YawSplinePlannerParams & params) { params_ = params; }

void YawSplinePlanner::apply(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }
  if (params_.mode == "path_tangent") {
    applyPathTangent(trajectory, initial_yaw);
    return;
  }
  if (params_.mode == "clearance_aware") {
    applyClearanceAware(trajectory, initial_yaw, goal_yaw);
    return;
  }
  if (params_.mode == "hold") {
    for (auto & point : trajectory.points) {
      point.yaw = normalizeAngle(initial_yaw);
      point.yaw_rate = 0.0;
    }
    return;
  }
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);
}

void YawSplinePlanner::applyClearanceAware(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  // 开阔区域先平滑朝向目标；只有窄通道才强制朝向路径，保留舵轮横移能力。
  applyGoalHeading(trajectory, initial_yaw, goal_yaw);
  if (trajectory.points.size() < 2) {
    return;
  }

  std::vector<double> open_area_yaws;
  open_area_yaws.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    open_area_yaws.push_back(point.yaw);
  }

  const double enter_clearance = std::max(0.0, params_.narrow_clearance_enter);
  const double exit_clearance = std::max(enter_clearance, params_.narrow_clearance_exit);
  // 只接受 2（矩形）与 4（正方形）；其它值按矩形处理，保持保守。
  const int symmetry_order = params_.tangent_symmetry_order == 4 ? 4 : 2;
  bool narrow = false;
  for (const auto & point : trajectory.points) {
    if (std::isfinite(point.clearance) && point.clearance <= enter_clearance) {
      // A path that enters a narrow section must start rotating before the
      // vehicle reaches the wall; delaying tangent alignment creates the
      // large yaw error seen in red-box goal 8.
      narrow = true;
      break;
    }
  }
  const std::size_t point_count = trajectory.points.size();
  std::vector<bool> narrow_flags(point_count, false);
  for (std::size_t i = 1; i < point_count; ++i) {
    const auto & current = trajectory.points[i];
    if (std::isfinite(current.clearance)) {
      // 进入和退出使用不同阈值，避免净空在边界附近时航向模式来回切换。
      narrow = narrow ? current.clearance < exit_clearance : current.clearance <= enter_clearance;
    }
    narrow_flags[i] = narrow;
  }
  // 两段窄通道之间的短开阔间隙按窄通道处理：否则参考 yaw 会在间隙内先回摆到目标朝向，
  // 下一段再转回切线，MPC 跟随这种来回摆动时误差叠加，足迹角点会蹭上通道口
  // （红框斜坡口即此情形）。只桥接两侧都是窄通道的内部间隙，末端开阔段仍回到目标朝向。
  const double bridge_time = std::max(0.0, params_.narrow_gap_bridge_time);
  if (bridge_time > 0.0) {
    std::size_t i = 1;
    while (i < point_count) {
      if (narrow_flags[i]) {
        ++i;
        continue;
      }
      std::size_t gap_end = i;
      while (gap_end < point_count && !narrow_flags[gap_end]) {
        ++gap_end;
      }
      const bool interior = narrow_flags[i - 1] && gap_end < point_count;
      if (interior && trajectory.points[gap_end].t - trajectory.points[i - 1].t <= bridge_time) {
        std::fill(narrow_flags.begin() + i, narrow_flags.begin() + gap_end, true);
      }
      i = gap_end;
    }
  }

  // 每点的切线朝向（前后差分）；切线退化的点记为 NaN。
  std::vector<double> tangent_yaws(point_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 1; i < point_count; ++i) {
    const std::size_t next_index = std::min(i + 1, point_count - 1);
    const double tangent_x = trajectory.points[next_index].x - trajectory.points[i - 1].x;
    const double tangent_y = trajectory.points[next_index].y - trajectory.points[i - 1].y;
    if (std::hypot(tangent_x, tangent_y) > 1e-6) {
      tangent_yaws[i] = std::atan2(tangent_y, tangent_x);
    }
  }
  const double period = 2.0 * M_PI / static_cast<double>(symmetry_order);
  // 在所有等价朝向中选与 reference_yaw 转角最小的一个；转角相同时保留靠前的候选。
  const auto nearest_aligned_yaw = [period, symmetry_order](double tangent, double reference_yaw) {
      double best = tangent;
      double best_turn = std::abs(shortestAngularDistance(reference_yaw, tangent));
      for (int k = 1; k < symmetry_order; ++k) {
        const double candidate = normalizeAngle(tangent + period * k);
        const double turn = std::abs(shortestAngularDistance(reference_yaw, candidate));
        if (turn < best_turn) {
          best_turn = turn;
          best = candidate;
        }
      }
      return best;
    };
  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  double previous_rate = 0.0;
  const double acceleration_limit = std::max(0.0, params_.yaw_acceleration_limit);
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < point_count; ++i) {
    auto & current = trajectory.points[i];
    double desired_yaw = open_area_yaws[i];
    if (narrow_flags[i] && std::isfinite(tangent_yaws[i])) {
      // 窄通道只要求足迹与切线对齐到对称等价：矩形可正向或反向，正方形再加 ±pi/2。
      // 在所有等价朝向中选与上一时刻转角最小的一个，避免贴墙时做不必要的大角度转向
      // （正方形车沿 -pi/2 切线进南侧墙尖时，yaw 0 本身已对齐，不应再转 90 度）。
      // 转角相同时保留靠前的候选（先正向切线），与原先正/反向二选一的行为一致。
      desired_yaw = nearest_aligned_yaw(tangent_yaws[i], previous_yaw);
    }

    // 最后按 yaw_rate_limit 裁剪，避免参考轨迹要求舵轮瞬时转向。
    const double dt = std::max(1e-3, current.t - previous_t);
    const double rate_limit = std::max(0.0, params_.yaw_rate_limit);
    const double desired_delta = shortestAngularDistance(previous_yaw, desired_yaw);
    double rate = std::max(-rate_limit, std::min(rate_limit, desired_delta / dt));
    if (acceleration_limit > 0.0) {
      // 目标角速度取"走完本步后剩余角差还能以 acceleration_limit 刹停"的上界
      // （离散形式 v^2/(2a) + v*dt <= |delta|，避免越过期望 yaw），再限制相邻点的
      // 角速度变化量，使参考 yaw 的角加速度不超过 acceleration_limit。
      const double braking_rate = acceleration_limit * (
        std::sqrt(dt * dt + 2.0 * std::abs(desired_delta) / acceleration_limit) - dt);
      rate = std::copysign(std::min(std::abs(rate), braking_rate), desired_delta);
      const double max_rate_change = acceleration_limit * dt;
      rate = std::max(previous_rate - max_rate_change, std::min(previous_rate + max_rate_change, rate));
      rate = std::max(-rate_limit, std::min(rate_limit, rate));
    }
    current.yaw = normalizeAngle(previous_yaw + rate * dt);
    current.yaw_rate = rate;
    previous_yaw = current.yaw;
    previous_rate = rate;
    previous_t = current.t;
  }
  // 窄通道可能延续到最终位置。此时保留通道内的切线 yaw，再追加原地、
  // 限速的目标朝向过渡，避免 Navigate action 在位置到达后永久 tracking。
  appendTerminalGoalYawTransition(trajectory, goal_yaw);
}

void YawSplinePlanner::appendTerminalGoalYawTransition(
  ReferenceTrajectory & trajectory, double goal_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }
  const ReferencePoint terminal = trajectory.points.back();
  const double delta = shortestAngularDistance(terminal.yaw, goal_yaw);
  if (std::abs(delta) <= 1e-8) {
    trajectory.points.back().yaw = normalizeAngle(goal_yaw);
    trajectory.points.back().yaw_rate = 0.0;
    return;
  }

  const double yaw_rate_limit = std::max(1e-3, params_.yaw_rate_limit);
  double duration = 1.875 * std::abs(delta) / yaw_rate_limit;
  if (params_.yaw_acceleration_limit > 0.0) {
    // 五次 smoothstep 的峰值角加速度为 5.7735 * |delta| / T^2；小角度时只按角速度
    // 定时长会让角加速度远超 MPC max_awz。
    duration = std::max(
      duration, std::sqrt(5.7735 * std::abs(delta) / params_.yaw_acceleration_limit));
  }
  const double sample_period = std::max(0.01, params_.terminal_yaw_sample_period);
  const int sample_count = std::max(
    2, static_cast<int>(std::ceil(duration / sample_period)));

  trajectory.points.reserve(trajectory.points.size() + static_cast<std::size_t>(sample_count));
  for (int index = 1; index <= sample_count; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(sample_count);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;

    ReferencePoint point = terminal;
    point.t = terminal.t + duration * u;
    point.s = terminal.s;
    point.v = 0.0;
    point.vx = 0.0;
    point.vy = 0.0;
    point.ax = 0.0;
    point.ay = 0.0;
    point.yaw = normalizeAngle(terminal.yaw + delta * blend);
    point.yaw_rate = delta * blend_rate;
    trajectory.points.push_back(point);
  }
}

void YawSplinePlanner::applyGoalHeading(
  ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const
{
  const double start = normalizeAngle(initial_yaw);
  const double delta = shortestAngularDistance(start, goal_yaw);
  double available_duration = std::max(1e-3, trajectory.totalTime());
  const double rate_limited_duration = params_.yaw_rate_limit > 1e-6
                                         ? 1.875 * std::abs(delta) / params_.yaw_rate_limit
                                         : available_duration;
  if (rate_limited_duration > available_duration && trajectory.totalTime() > 1e-6) {
    const double time_scale = rate_limited_duration / available_duration;
    for (auto & point : trajectory.points) {
      point.t *= time_scale;
      point.vx /= time_scale;
      point.vy /= time_scale;
      point.v /= time_scale;
      point.ax /= time_scale * time_scale;
      point.ay /= time_scale * time_scale;
    }
    available_duration = rate_limited_duration;
  }
  double blend_duration = rate_limited_duration;
  if (params_.yaw_acceleration_limit > 0.0) {
    // 小角度转向只按角速度定时长会过短，五次曲线峰值角加速度 5.7735 * |delta| / T^2
    // 远超 MPC max_awz；在轨迹时长内按角加速度上限放长。不改变平移时间参数化。
    blend_duration = std::max(
      blend_duration, std::sqrt(5.7735 * std::abs(delta) / params_.yaw_acceleration_limit));
  }
  const double duration = std::max(1e-3, std::min(available_duration, blend_duration));

  for (auto & point : trajectory.points) {
    const double u = std::max(0.0, std::min(1.0, point.t / duration));
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
    point.yaw = normalizeAngle(start + delta * blend);
    point.yaw_rate = delta * blend_rate;
  }
}

void YawSplinePlanner::applyPathTangent(ReferenceTrajectory & trajectory, double initial_yaw) const
{
  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & prev = trajectory.points[i - 1];
    auto & current = trajectory.points[i];
    const double tangent_yaw = std::atan2(current.y - prev.y, current.x - prev.x);
    const double dt = std::max(1e-3, current.t - previous_t);
    const double max_delta = std::max(0.0, params_.yaw_rate_limit) * dt;
    const double desired_delta = shortestAngularDistance(previous_yaw, tangent_yaw);
    const double clamped_delta = std::max(-max_delta, std::min(max_delta, desired_delta));

    current.yaw = normalizeAngle(previous_yaw + clamped_delta);
    current.yaw_rate = clamped_delta / dt;
    previous_yaw = current.yaw;
    previous_t = current.t;
  }
}

double YawSplinePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double YawSplinePlanner::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

}  // namespace minco_planner
