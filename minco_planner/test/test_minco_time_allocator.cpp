// Copyright 2026

#include <cmath>

#include "gtest/gtest.h"
#include "minco_planner/trajectory/minco_time_allocator.hpp"

namespace
{

using Point = Eigen::Vector2d;

TEST(MincoTimeAllocator, CurvatureAwareAllocationSlowsCornerWithoutSlowingUnrelatedStraightSegments)
{
  minco_planner::MincoTimeAllocatorParams params;
  params.reference_speed = 2.0;
  params.max_velocity = 2.0;
  params.max_acceleration = 3.0;
  params.max_lateral_acceleration = 0.5;
  minco_planner::MincoTimeAllocator allocator(params);
  const auto allocation = allocator.allocate(
    {Point(0.0, 0.0), Point(2.0, 0.0), Point(2.0, 0.5), Point(4.0, 0.5)}, 2.0);

  ASSERT_TRUE(allocation.valid);
  ASSERT_EQ(allocation.waypoint_speeds.size(), 4U);
  EXPECT_LT(allocation.waypoint_speeds[1], allocation.waypoint_speeds[0] + 1e-9);
  EXPECT_LT(allocation.waypoint_speeds[2], allocation.waypoint_speeds[0] + 1e-9);

  Eigen::VectorXd durations(5);
  durations << 1.0, 1.0, 1.0, 1.0, 1.0;
  ASSERT_TRUE(allocator.applyLocalDynamicScaling(
    durations, {1.0, 3.0, 1.0, 1.0, 1.0}, {1.0, 3.0, 1.0, 1.0, 1.0},
    {1.0, 1.0, 1.0, 1.0, 1.0},
    2.0, 2.0, 0.0));
  EXPECT_GT(durations(0), 1.0);
  EXPECT_GT(durations(1), 1.0);
  EXPECT_GT(durations(2), 1.0);
  EXPECT_DOUBLE_EQ(durations(3), 1.0);
  EXPECT_DOUBLE_EQ(durations(4), 1.0);
}

TEST(MincoTimeAllocator, ShortEndpointSegmentsRemainFiniteAndMonotonic)
{
  minco_planner::MincoTimeAllocatorParams params;
  params.min_segment_time = 0.02;
  minco_planner::MincoTimeAllocator allocator(params);
  const auto allocation = allocator.allocate(
    {Point(0.0, 0.0), Point(0.001, 0.0), Point(1.0, 0.0), Point(1.001, 0.0)});

  ASSERT_TRUE(allocation.valid);
  ASSERT_EQ(allocation.durations.size(), 3);
  for (int index = 0; index < allocation.durations.size(); ++index) {
    EXPECT_TRUE(std::isfinite(allocation.durations(index)));
    EXPECT_GT(allocation.durations(index), 0.0);
  }
  EXPECT_DOUBLE_EQ(allocation.waypoint_speeds.back(), 0.0);
}

}  // namespace
