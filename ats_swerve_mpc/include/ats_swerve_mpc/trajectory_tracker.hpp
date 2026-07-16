// Copyright 2026

#ifndef ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_
#define ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_

#include <cstddef>
#include <vector>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

namespace ats_swerve_mpc
{

struct TimedState
{
  double time = 0.0;
  State state = State::Zero();
};

struct TrajectoryProjection
{
  bool valid = false;
  double time = 0.0;
  double cross_track_error = 0.0;
  std::size_t segment_index = 0;
};

struct TrajectoryTrackerConfig
{
  double backward_search_window = 0.20;
  double forward_search_window = 2.00;
  double cross_track_slowdown_start = 0.15;
  double cross_track_slowdown_end = 0.60;
  double min_progress_scale = 0.25;
  double command_latency_compensation = 0.01;
};

class TrajectoryTracker
{
public:
  explicit TrajectoryTracker(const TrajectoryTrackerConfig & config = TrajectoryTrackerConfig());

  void setConfig(const TrajectoryTrackerConfig & config);
  void setTrajectory(std::vector<TimedState> trajectory);
  void clear();
  void resetProgress();

  bool empty() const { return trajectory_.empty(); }
  const State & goal() const { return trajectory_.back().state; }
  double duration() const { return trajectory_.back().time - trajectory_.front().time; }
  double finalTime() const { return trajectory_.back().time; }
  double minimumProgressScale() const { return config_.min_progress_scale; }
  TrajectoryProjection project(const Eigen::Vector2d & position);
  TrajectoryProjection project(const State & state);
  double progressScale(double cross_track_error) const;
  std::vector<Se2Reference> buildHorizon(
    const TrajectoryProjection & projection, int horizon, double dt) const;

private:
  TrajectoryProjection projectImpl(
    const Eigen::Vector2d & position, double yaw, bool use_yaw);
  TrajectoryProjection nearestProjection(
    const Eigen::Vector2d & position, double yaw, bool use_yaw, bool restrict_progress) const;
  bool sampleReference(double time, Se2Reference & reference) const;
  static double normalizeAngle(double angle);
  static double interpolateAngle(double from, double to, double ratio);

  TrajectoryTrackerConfig config_;
  std::vector<TimedState> trajectory_;
  bool has_progress_ = false;
  double last_progress_time_ = 0.0;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_
