// Copyright 2026

#include <gtest/gtest.h>

#include "ats_swerve_mpc/trajectory_tracker.hpp"

namespace
{

std::vector<ats_swerve_mpc::TimedState> straightTrajectory()
{
  std::vector<ats_swerve_mpc::TimedState> trajectory(4);
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].time = 10.0 + static_cast<double>(i);
    trajectory[i].state << static_cast<double>(i), 0.0, 0.0;
  }
  return trajectory;
}

TEST(TrajectoryTracker, ProjectsOntoNearestTimedSegment)
{
  ats_swerve_mpc::TrajectoryTracker tracker;
  tracker.setTrajectory(straightTrajectory());

  const auto projection = tracker.project(Eigen::Vector2d(0.25, 0.40));

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0U);
  EXPECT_NEAR(projection.time, 10.25, 1e-9);
  EXPECT_NEAR(projection.cross_track_error, 0.40, 1e-9);
}

TEST(TrajectoryTracker, DoesNotJumpBackwardAfterProgress)
{
  ats_swerve_mpc::TrajectoryTracker tracker;
  tracker.setTrajectory(straightTrajectory());
  ASSERT_TRUE(tracker.project(Eigen::Vector2d(1.4, 0.0)).valid);

  const auto projection = tracker.project(Eigen::Vector2d(0.4, 0.0));

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.time, 11.4, 1e-9);
}

TEST(TrajectoryTracker, BoundsForwardJumpAfterInitialProjection)
{
  ats_swerve_mpc::TrajectoryTrackerConfig config;
  config.forward_search_window = 0.5;
  ats_swerve_mpc::TrajectoryTracker tracker(config);
  tracker.setTrajectory(straightTrajectory());
  ASSERT_TRUE(tracker.project(Eigen::Vector2d(0.2, 0.0)).valid);

  const auto projection = tracker.project(Eigen::Vector2d(2.8, 0.0));

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.time, 10.7, 1e-9);
}

TEST(TrajectoryTracker, SlowsHorizonProgressForCrossTrackError)
{
  ats_swerve_mpc::TrajectoryTrackerConfig config;
  config.cross_track_slowdown_start = 0.10;
  config.cross_track_slowdown_end = 0.50;
  config.min_progress_scale = 0.20;
  config.command_latency_compensation = 0.0;
  ats_swerve_mpc::TrajectoryTracker tracker(config);
  tracker.setTrajectory(straightTrajectory());
  const auto projection = tracker.project(Eigen::Vector2d(0.0, 0.30));

  const auto horizon = tracker.buildHorizon(projection, 2, 0.5);

  ASSERT_EQ(horizon.size(), 3U);
  EXPECT_NEAR(tracker.progressScale(0.30), 0.60, 1e-9);
  EXPECT_NEAR(horizon[1].state(0), 0.30, 1e-9);
  EXPECT_NEAR(horizon[1].control(0), 0.60, 1e-9);
}

TEST(TrajectoryTracker, ClearDropsStoppedTrajectoryAndProgress)
{
  ats_swerve_mpc::TrajectoryTracker tracker;
  tracker.setTrajectory(straightTrajectory());
  ASSERT_TRUE(tracker.project(Eigen::Vector2d(0.5, 0.0)).valid);
  tracker.clear();
  EXPECT_TRUE(tracker.empty());
  EXPECT_FALSE(tracker.project(Eigen::Vector2d(0.5, 0.0)).valid);
}

TEST(TrajectoryTracker, UsesYawToAdvanceAnInPlaceTerminalRotation)
{
  std::vector<ats_swerve_mpc::TimedState> trajectory(4);
  trajectory[0].time = 10.0;
  trajectory[0].state << 0.0, 0.0, 0.0;
  trajectory[1].time = 11.0;
  trajectory[1].state << 1.0, 0.0, 0.0;
  trajectory[2].time = 12.0;
  trajectory[2].state << 1.0, 0.0, M_PI_2;
  trajectory[3].time = 13.0;
  trajectory[3].state << 1.0, 0.0, M_PI;

  ats_swerve_mpc::TrajectoryTracker tracker;
  tracker.setTrajectory(trajectory);
  ats_swerve_mpc::State state;
  state << 1.0, 0.0, 0.75 * M_PI;

  const auto projection = tracker.project(state);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 2U);
  EXPECT_NEAR(projection.time, 12.5, 1e-9);
}

TEST(TrajectoryTracker, ConvertsWorldReferenceVelocityToBodyFrameAtNinetyDegrees)
{
  std::vector<ats_swerve_mpc::TimedState> trajectory(2);
  trajectory[0].time = 0.0;
  trajectory[0].state << 0.0, 0.0, M_PI_2;
  trajectory[1].time = 1.0;
  trajectory[1].state << 0.0, 1.0, M_PI_2;
  ats_swerve_mpc::TrajectoryTrackerConfig config;
  config.command_latency_compensation = 0.0;
  ats_swerve_mpc::TrajectoryTracker tracker(config);
  tracker.setTrajectory(trajectory);
  const auto projection = tracker.project(trajectory.front().state);
  const auto horizon = tracker.buildHorizon(projection, 1, 0.1);

  ASSERT_EQ(horizon.size(), 2U);
  EXPECT_NEAR(horizon.front().control(0), 1.0, 1e-12);
  EXPECT_NEAR(horizon.front().control(1), 0.0, 1e-12);
}

TEST(TrajectoryTracker, UsesShortestYawRateAcrossPiWrap)
{
  std::vector<ats_swerve_mpc::TimedState> trajectory(2);
  trajectory[0].time = 0.0;
  trajectory[0].state << 0.0, 0.0, M_PI - 0.1;
  trajectory[1].time = 1.0;
  trajectory[1].state << 1.0, 0.0, -M_PI + 0.1;
  ats_swerve_mpc::TrajectoryTrackerConfig config;
  config.command_latency_compensation = 0.0;
  ats_swerve_mpc::TrajectoryTracker tracker(config);
  tracker.setTrajectory(trajectory);
  const auto projection = tracker.project(trajectory.front().state);
  const auto horizon = tracker.buildHorizon(projection, 1, 0.1);

  ASSERT_EQ(horizon.size(), 2U);
  EXPECT_NEAR(horizon.front().control(2), 0.2, 1e-12);
}

}  // namespace
