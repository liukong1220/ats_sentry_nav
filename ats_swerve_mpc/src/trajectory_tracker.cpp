// Copyright 2026

#include "ats_swerve_mpc/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ats_swerve_mpc
{

TrajectoryTracker::TrajectoryTracker(const TrajectoryTrackerConfig & config) : config_(config)
{
  setConfig(config);
}

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

void TrajectoryTracker::setTrajectory(std::vector<TimedState> trajectory)
{
  trajectory_ = std::move(trajectory);
  resetProgress();
}

void TrajectoryTracker::resetProgress()
{
  has_progress_ = false;
  last_progress_time_ = 0.0;
}

TrajectoryProjection TrajectoryTracker::project(const Eigen::Vector2d & position)
{
  TrajectoryProjection projection = nearestProjection(position, has_progress_);
  if (!projection.valid && has_progress_) {
    projection = nearestProjection(position, false);
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

TrajectoryProjection TrajectoryTracker::nearestProjection(
  const Eigen::Vector2d & position, bool restrict_progress) const
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
    const double segment_duration = std::max(1e-9, second.time - first.time);
    const double minimum_ratio =
      std::clamp((minimum_time - first.time) / segment_duration, 0.0, 1.0);
    const double maximum_ratio =
      std::clamp((maximum_time - first.time) / segment_duration, 0.0, 1.0);
    ratio = std::clamp(ratio, minimum_ratio, maximum_ratio);
    const Eigen::Vector2d projected = start + ratio * delta;
    const double distance_squared = (position - projected).squaredNorm();
    const double projected_time = first.time + ratio * (second.time - first.time);
    if (distance_squared >= best_distance_squared) {
      continue;
    }
    best.valid = true;
    best.time = projected_time;
    best.cross_track_error = std::sqrt(distance_squared);
    best.segment_index = i;
    best_distance_squared = distance_squared;
  }
  return best;
}

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

double TrajectoryTracker::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double TrajectoryTracker::interpolateAngle(double from, double to, double ratio)
{
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

}  // namespace ats_swerve_mpc
