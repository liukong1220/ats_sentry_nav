// Copyright 2026

#include "ats_swerve_mpc/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

/*
轨迹输入
    │
    ▼
寻找机器人当前对应轨迹位置(Project)
    │
    ▼
限制轨迹进度(Progress)
    │
    ▼
生成MPC预测参考(buildHorizon)
*/

namespace ats_swerve_mpc
{

//轨迹跟踪器类的构造函数,参数声明与加载
TrajectoryTracker::TrajectoryTracker(const TrajectoryTrackerConfig & config) : config_(config)
{
  setConfig(config);
}

//设置轨迹跟踪器参数，包括前向/后向搜索窗口、横向误差减速区间、最小进度缩放和控制延迟补偿
void TrajectoryTracker::setConfig(const TrajectoryTrackerConfig & config)
{
  config_ = config;
  config_.backward_search_window = std::max(0.0, config_.backward_search_window);
  config_.forward_search_window = std::max(0.0, config_.forward_search_window);
  config_.cross_track_slowdown_start = std::max(0.0, config_.cross_track_slowdown_start);
  config_.cross_track_slowdown_end =
    std::max(config_.cross_track_slowdown_start + 1e-6, config_.cross_track_slowdown_end);
  config_.min_progress_scale = std::clamp(config_.min_progress_scale, 0.0, 1.0);
  config_.command_latency_compensation = std::max(0.0, config_.command_latency_compensation);
  config_.terminal_position_tolerance = std::max(0.0, config_.terminal_position_tolerance);
  config_.stationary_yaw_tolerance = std::max(1e-3, config_.stationary_yaw_tolerance);
}

//通过移动语义高效接管路径点，然后重置进，丢弃旧投影信息。
void TrajectoryTracker::setTrajectory(std::vector<TimedState> trajectory)
{
  trajectory_ = std::move(trajectory);
  resetProgress();
}

//清空轨迹并重置进度
void TrajectoryTracker::clear()
{
  trajectory_.clear();
  resetProgress();
}

//将投影状态标记为未初始化，进度时间归零
void TrajectoryTracker::resetProgress()
{
  has_progress_ = false;
  last_progress_time_ = 0.0;
}

//根据给定位置，沿轨迹寻找最近投影点，并返回投影结果，包括投影时间、横向误差和轨迹段索引
TrajectoryProjection TrajectoryTracker::project(const Eigen::Vector2d & position)
{
  return projectImpl(position, 0.0, false);
}

//根据给定状态，沿轨迹寻找最近投影点，并返回投影结果，包括投影时间、横向误差和轨迹段索引
TrajectoryProjection TrajectoryTracker::project(const State & state, double current_wz)
{
  if (!state.allFinite()) {
    return {};
  }
  return projectImpl(state.head<2>(), state(2), true, current_wz);
}

//根据给定位置和航向，沿轨迹寻找最近投影点，并返回投影结果，包括投影时间、横向误差和轨迹段索引，使得投影点沿轨迹平滑推进，具有抗噪能力
TrajectoryProjection TrajectoryTracker::projectImpl(
  const Eigen::Vector2d & position, double yaw, bool use_yaw, double current_wz)
{
  // 查找当前进度之后第一个尚未完成航向对齐的原地转向段 [rot_begin, rot_end]。
  // 在未转到该段末端航向（stationary_yaw_tolerance 内）之前，投影不得越过 rot_end 跳到
  // 后续平移段——否则毫米级 XY 抖动会让 MPC 跳过洞口/窄区预对齐旋转，边转边横扫撞墙。
  std::optional<std::size_t> active_rot_begin;
  std::optional<std::size_t> active_rot_end;
  std::optional<double> rotation_time_cap;
  if (use_yaw && trajectory_.size() >= 2) {
    const double current_time = has_progress_ ? last_progress_time_ : trajectory_.front().time;
    const double search_limit =
      has_progress_ ? current_time + config_.forward_search_window : trajectory_.back().time;
    constexpr double kStationaryXyEpsSq = 1e-12;
    constexpr double kMinRotationSpanRad = 0.05;
    std::size_t idx = 0;
    while (idx + 1 < trajectory_.size()) {
      const bool stationary =
        (trajectory_[idx + 1].state.head<2>() - trajectory_[idx].state.head<2>()).squaredNorm() <=
        kStationaryXyEpsSq;
      if (!stationary) {
        ++idx;
        continue;
      }
      const std::size_t block_begin = idx;
      double angular_span = 0.0;
      while (idx + 1 < trajectory_.size() &&
             (trajectory_[idx + 1].state.head<2>() -
              trajectory_[block_begin].state.head<2>()).squaredNorm() <= kStationaryXyEpsSq) {
        angular_span += std::abs(
          normalizeAngle(trajectory_[idx + 1].state(2) - trajectory_[idx].state(2)));
        ++idx;
      }
      const std::size_t block_end = idx;
      if (angular_span <= kMinRotationSpanRad) {
        continue;
      }
      if (trajectory_[block_end].time < current_time - 1e-6) {
        continue;
      }
      if (trajectory_[block_begin].time > search_limit + 1e-6) {
        break;
      }
      const double exit_yaw_error =
        std::abs(normalizeAngle(yaw - trajectory_[block_end].state(2)));
      const bool intermediate_block = (block_end + 1 < trajectory_.size());
      const bool still_spinning = intermediate_block && std::isfinite(current_wz) &&
        std::abs(current_wz) > 0.35;
      if (exit_yaw_error > config_.stationary_yaw_tolerance || still_spinning) {
        active_rot_begin = block_begin;
        active_rot_end = block_end;
        rotation_time_cap = trajectory_[block_end].time;
        break;
      }
    }
  }

  TrajectoryProjection projection = nearestProjection(
    position, yaw, use_yaw, has_progress_, std::nullopt, rotation_time_cap);
  if (!projection.valid && has_progress_) {
    projection = nearestProjection(
      position, yaw, use_yaw, false, std::nullopt, rotation_time_cap);
  }
  if (!projection.valid) {
    return projection;
  }

  // 当已接近中途原地转向段时，直接投影到该原地转向段内按 yaw 推进。
  // 非末端原地转向（如贴墙侧移掉头或洞口预对齐）要求到达掉头点 3 cm 内才进入旋转，
  // 防止提前在离掉头点 8 cm 处起转导致角落擦墙；末端转向沿用 terminal_position_tolerance。
  const bool intermediate_rotation =
    active_rot_end && (*active_rot_end + 1 < trajectory_.size());
  const double handoff_tolerance = intermediate_rotation
    ? std::min(config_.terminal_position_tolerance, 0.03)
    : config_.terminal_position_tolerance;
  if (use_yaw && active_rot_begin && active_rot_end && *active_rot_begin > 0 &&
      handoff_tolerance > 0.0 &&
      (position - trajectory_[*active_rot_begin].state.head<2>()).norm() <= handoff_tolerance &&
      projection.segment_index < *active_rot_begin) {
    const auto & first = trajectory_[projection.segment_index];
    const auto & second = trajectory_[projection.segment_index + 1];
    const double ratio = std::clamp(
      (projection.time - first.time) / std::max(1e-9, second.time - first.time),
      0.0, 1.0);
    double remaining_translation =
      (1.0 - ratio) * (second.state.head<2>() - first.state.head<2>()).norm();
    for (std::size_t i = projection.segment_index + 1;
         i < *active_rot_begin && remaining_translation <= handoff_tolerance;
         ++i) {
      remaining_translation +=
        (trajectory_[i + 1].state.head<2>() - trajectory_[i].state.head<2>()).norm();
    }
    if (remaining_translation <= handoff_tolerance) {
      const auto rot_projection = nearestProjection(
        position, yaw, true, has_progress_, *active_rot_begin, rotation_time_cap);
      if (rot_projection.valid) {
        projection = rot_projection;
      }
    }
  }

  // Once the approach reaches the existing goal-position disk, centimetre-scale
  // XY tracking error must not pin yaw to a slightly closer approach segment.
  // Only the contiguous stationary suffix is eligible; a loop passing the goal
  // earlier in the path must not skip its remaining translation.
  if (use_yaw && config_.terminal_position_tolerance > 0.0 &&
      (position - trajectory_.back().state.head<2>()).norm() <=
        config_.terminal_position_tolerance) {
    std::size_t terminal_begin = trajectory_.size() - 1;
    while (terminal_begin > 0 &&
           (trajectory_[terminal_begin - 1].state.head<2>() -
            trajectory_.back().state.head<2>()).squaredNorm() <= 1e-12) {
      --terminal_begin;
    }
    double remaining_translation = 0.0;
    if (projection.segment_index < terminal_begin) {
      const auto & first = trajectory_[projection.segment_index];
      const auto & second = trajectory_[projection.segment_index + 1];
      const double ratio = std::clamp(
        (projection.time - first.time) / std::max(1e-9, second.time - first.time),
        0.0, 1.0);
      remaining_translation =
        (1.0 - ratio) * (second.state.head<2>() - first.state.head<2>()).norm();
      for (std::size_t i = projection.segment_index + 1;
           i < terminal_begin && remaining_translation <= config_.terminal_position_tolerance;
           ++i) {
        remaining_translation +=
          (trajectory_[i + 1].state.head<2>() - trajectory_[i].state.head<2>()).norm();
      }
    }
    if (terminal_begin + 1 < trajectory_.size() &&
        remaining_translation <= config_.terminal_position_tolerance &&
        (!rotation_time_cap || trajectory_[terminal_begin].time <= *rotation_time_cap + 1e-6)) {
      const auto terminal = nearestProjection(
        position, yaw, true, has_progress_, terminal_begin, rotation_time_cap);
      if (terminal.valid) {
        projection = terminal;
      }
    }
  }

  if (has_progress_) {
    // 进度只能单调前进且受前向窗口限制，避免定位噪声跳回旧路径或跳到远端路径段。
    projection.time = std::max(last_progress_time_, projection.time);
    projection.time =
      std::min(last_progress_time_ + config_.forward_search_window, projection.time);
  }
  projection.time = std::clamp(projection.time, trajectory_.front().time, trajectory_.back().time);
  last_progress_time_ = projection.time;
  has_progress_ = true;
  return projection;
}

//在给定位置和航向的情况下，沿轨迹寻找最近投影点的实现函数
TrajectoryProjection TrajectoryTracker::nearestProjection(
  const Eigen::Vector2d & position, double yaw, bool use_yaw, bool restrict_progress,
  std::optional<std::size_t> segment_begin,
  std::optional<double> max_time_cap) const
{
  TrajectoryProjection best;
  if (trajectory_.size() < 2 || !position.allFinite()) {
    return best;
  }

  const double minimum_time = restrict_progress
                                ? last_progress_time_ -
                                  (segment_begin ? 0.0 : config_.backward_search_window)
                                : -std::numeric_limits<double>::infinity();
  double maximum_time = restrict_progress
                          ? last_progress_time_ + config_.forward_search_window
                          : std::numeric_limits<double>::infinity();
  if (max_time_cap) {
    maximum_time = std::min(maximum_time, *max_time_cap);
  }
  if (maximum_time < minimum_time) {
    maximum_time = minimum_time;
  }
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_yaw_error = std::numeric_limits<double>::infinity();

  for (std::size_t i = segment_begin.value_or(0); i + 1 < trajectory_.size(); ++i) {
    const TimedState & first = trajectory_[i];
    const TimedState & second = trajectory_[i + 1];
    if (second.time < minimum_time || first.time > maximum_time ||
        (max_time_cap && first.time >= *max_time_cap - 1e-12 &&
         second.time > *max_time_cap + 1e-12)) {
      continue;
    }
    const Eigen::Vector2d start = first.state.head<2>();
    const Eigen::Vector2d delta = second.state.head<2>() - start;
    const double length_squared = delta.squaredNorm();
    double ratio = length_squared > 1e-12
                     ? std::clamp((position - start).dot(delta) / length_squared, 0.0, 1.0)
                     : 0.0;
    if (length_squared <= 1e-12 && use_yaw) {
      // 原地转向段没有可用的二维位置进度，必须用当前 yaw 才能推进末端 reference。
      const double yaw_delta = normalizeAngle(second.state(2) - first.state(2));
      if (std::abs(yaw_delta) > 1e-9) {
        ratio = std::clamp(
          normalizeAngle(yaw - first.state(2)) / yaw_delta, 0.0, 1.0);
      }
    }
    const double segment_duration = std::max(1e-9, second.time - first.time);
    const double minimum_ratio =
      std::clamp((minimum_time - first.time) / segment_duration, 0.0, 1.0);
    const double maximum_ratio =
      std::clamp((maximum_time - first.time) / segment_duration, 0.0, 1.0);
    ratio = std::clamp(ratio, minimum_ratio, maximum_ratio);
    const Eigen::Vector2d projected = start + ratio * delta;
    const double distance_squared = (position - projected).squaredNorm();
    const double projected_time = first.time + ratio * (second.time - first.time);
    double yaw_error = 0.0;
    if (use_yaw) {
      yaw_error = std::abs(normalizeAngle(
        yaw - interpolateAngle(first.state(2), second.state(2), ratio)));
    }
    const bool farther = distance_squared > best_distance_squared + 1e-12;
    const bool equal_distance = std::abs(distance_squared - best_distance_squared) <= 1e-12;
    if (farther || (equal_distance && yaw_error >= best_yaw_error)) {
      continue;
    }
    best.valid = true;
    best.time = projected_time;
    best.cross_track_error = std::sqrt(distance_squared);
    best.segment_index = i;
    best_distance_squared = distance_squared;
    best_yaw_error = yaw_error;
  }
  return best;
}

//根据横向误差计算进度缩放因子，横向误差越大，进度缩放越小，让底盘先回到路径附近再继续追赶
double TrajectoryTracker::progressScale(double cross_track_error) const
{
  if (
    !std::isfinite(cross_track_error) || cross_track_error <= config_.cross_track_slowdown_start) {
    return 1.0;
  }
  if (cross_track_error >= config_.cross_track_slowdown_end) {
    return config_.min_progress_scale;
  }
  // 横向偏差越大，越降低参考前馈和预测进度，让底盘先回到路径附近再继续追赶。
  const double ratio = (cross_track_error - config_.cross_track_slowdown_start) /
                       (config_.cross_track_slowdown_end - config_.cross_track_slowdown_start);
  return 1.0 - ratio * (1.0 - config_.min_progress_scale);
}

//构建预测参考序列：根据给定时间沿轨迹采样参考状态和控制量，返回采样结果，包括状态和控制量
std::vector<Se2Reference> TrajectoryTracker::buildHorizon(
  const TrajectoryProjection & projection, int horizon, double dt) const
{
  if (!projection.valid || horizon <= 0 || dt <= 0.0) {
    return {};
  }
  // 预测域以最近投影点为起点，而不是盲从 ROS 墙钟时间；延迟补偿也随降速同步缩小。
  const double scale = progressScale(projection.cross_track_error);
  const double start_time = projection.time + config_.command_latency_compensation * scale;
  std::vector<Se2Reference> references;
  references.reserve(static_cast<std::size_t>(horizon + 1));
  for (int step = 0; step <= horizon; ++step) {
    Se2Reference reference;
    if (!sampleReference(start_time + step * dt * scale, reference)) {
      return {};
    }
    reference.control *= scale;
    references.push_back(reference);
  }
  return references;
}

//单点采样：根据给定时间沿轨迹采样参考状态和控制量，返回采样结果，包括状态和控制量
bool TrajectoryTracker::sampleReference(double time, Se2Reference & reference) const
{
  if (trajectory_.empty()) {
    return false;
  }
  if (time >= trajectory_.back().time) {
    reference.state = trajectory_.back().state;
    reference.control.setZero();
    return true;
  }
  std::size_t upper = 1;
  while (upper < trajectory_.size() && trajectory_[upper].time < time) {
    ++upper;
  }
  upper = std::min(upper, trajectory_.size() - 1);
  const TimedState & first = trajectory_[upper - 1];
  const TimedState & second = trajectory_[upper];
  const double duration = std::max(1e-6, second.time - first.time);
  const double ratio = std::clamp((time - first.time) / duration, 0.0, 1.0);
  reference.state.head<2>() =
    first.state.head<2>() + ratio * (second.state.head<2>() - first.state.head<2>());
  reference.state(2) = interpolateAngle(first.state(2), second.state(2), ratio);
  const Eigen::Vector2d world_velocity =
    (second.state.head<2>() - first.state.head<2>()) / duration;
  // MINCO 速度在世界系，MPC 控制量必须转换为当前车体系 [vx, vy, wz]。
  const double cosine = std::cos(reference.state(2));
  const double sine = std::sin(reference.state(2));
  reference.control(0) = cosine * world_velocity.x() + sine * world_velocity.y();
  reference.control(1) = -sine * world_velocity.x() + cosine * world_velocity.y();
  reference.control(2) = normalizeAngle(second.state(2) - first.state(2)) / duration;
  return true;
}

//将角度归一化到 [-π, π] 范围内，避免角度跳变问题
double TrajectoryTracker::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

//在给定起始角度和目标角度的情况下，沿最短路径插值计算中间角度，避免角度跳变
double TrajectoryTracker::interpolateAngle(double from, double to, double ratio)
{
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

}  // namespace ats_swerve_mpc
