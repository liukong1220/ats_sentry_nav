// Copyright 2026 ATS 2026 Sentry Project

#include <gtest/gtest.h>

#include "minco_planner/debug/planner_debug_visualizer.hpp"

namespace minco_planner
{
namespace
{

TEST(PlannerDebugVisualizer, UsesFrontendGreenAndCommittedReferenceRed)
{
  nav_msgs::msg::Path frontend;
  frontend.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped frontend_pose;
  frontend_pose.pose.position.x = 1.0;
  frontend.poses.push_back(frontend_pose);
  frontend_pose.pose.position.x = 2.0;
  frontend_pose.pose.position.y = 0.5;
  frontend.poses.push_back(frontend_pose);

  ReferenceTrajectory reference;
  reference.header.frame_id = "map";
  ReferencePoint first;
  first.t = 0.0;
  first.s = 0.0;
  first.x = 1.0;
  first.y = 0.0;
  ReferencePoint second;
  second.t = 1.0;
  second.s = 1.1;
  second.x = 2.0;
  second.y = 0.5;
  reference.points = {first, second};

  const PlannerDebugVisualizer visualizer;
  const FootprintSafetyResult safety;
  const auto markers = visualizer.buildMarkers(frontend, reference, safety);

  ASSERT_GE(markers.markers.size(), 3U);
  const auto & frontend_marker = markers.markers[1];
  EXPECT_EQ(frontend_marker.ns, "minco_planner_frontend");
  EXPECT_EQ(frontend_marker.points.size(), frontend.poses.size());
  EXPECT_GT(frontend_marker.color.g, frontend_marker.color.r);
  EXPECT_GT(frontend_marker.color.g, frontend_marker.color.b);

  const auto & final_marker = markers.markers[2];
  EXPECT_EQ(final_marker.ns, "minco_planner_final_reference");
  EXPECT_EQ(final_marker.points.size(), reference.points.size());
  EXPECT_GT(final_marker.color.r, final_marker.color.g);
  EXPECT_GT(final_marker.color.r, final_marker.color.b);
}

}  // namespace
}  // namespace minco_planner
