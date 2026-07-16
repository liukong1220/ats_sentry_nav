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
TrajectoryProjection TrajectoryTracker::project(const State & state)
{
  if (!state.allFinite()) {
    return {};
  }
  return projectImpl(state.head<2>(), state(2), true);
}

//根据给定位置和航向，沿轨迹寻找最近投影点，并返回投影结果，包括投影时间、横向误差和轨迹段索引，使得投影点沿轨迹平滑推进，具有抗噪能力
TrajectoryProjection TrajectoryTracker::projectImpl(
  const Eigen::Vector2d & position, double yaw, bool use_yaw)
{
  TrajectoryProjection projection = nearestProjection(position, yaw, use_yaw, has_progress_);
  if (!projection.valid && has_progress_) {
    projection = nearestProjection(position, yaw, use_yaw, false);
  }
  if (!projection.valid) {
    return projection;
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
  const Eigen::Vector2d & position, double yaw, bool use_yaw, bool restrict_progress) const
{
  TrajectoryProjection best;
  if (trajectory_.size() < 2 || !position.allFinite()) {
    return best;
  }

  const double minimum_time = restrict_progress
                                ? last_progress_time_ - config_.backward_search_window
                                : -std::numeric_limits<double>::infinity();
  const double maximum_time = restrict_progress
                                ? last_progress_time_ + config_.forward_search_window
                                : std::numeric_limits<double>::infinity();
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_yaw_error = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i + 1 < trajectory_.size(); ++i) {
    const TimedState & first = trajectory_[i];
    const TimedState & second = trajectory_[i + 1];
    if (second.time < minimum_time || first.time > maximum_time) {
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
