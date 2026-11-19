#include <cmath>

#include <gtest/gtest.h>

#include "minco_planner/trajectory/tunnel_yaw_alignment.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"

namespace
{
class CorridorEsdf final : public ats_rc_esdf::EsdfProvider
{
public:
  bool available() const override { return true; }

  double getDistance(double, double y) const override
  {
    return 0.4 - std::abs(y);
  }

  Eigen::Vector2d getGradient(double, double y) const override
  {
    if (std::abs(y) < 1e-9) {
      return Eigen::Vector2d::Zero();
    }
    return Eigen::Vector2d(0.0, y > 0.0 ? -1.0 : 1.0);
  }
};

minco_planner::ReferenceTrajectory makeTrajectory()
{
  minco_planner::ReferenceTrajectory trajectory;
  for (int i = 0; i < 9; ++i) {
    minco_planner::ReferencePoint point;
    point.t = 0.1 * i;
    point.s = 0.1 * i;
    point.x = 0.1 * i;
    point.y = 0.0;
    point.clearance = 0.4;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(TunnelYawAlignment, DetectsTwoWallCorridorAndDrivesAlongTheAxis)
{
  const CorridorEsdf esdf;
  auto trajectory = makeTrajectory();
  minco_planner::TunnelYawParams params;
  params.enabled = true;
  const auto segments = minco_planner::detectTunnelSegments(trajectory, esdf, params);
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_NEAR(segments.front().width_estimate, 0.75, 0.08);
  const double aligned = minco_planner::tunnelAlignedYaw(
    segments.front().axis_yaw.front(), params.yaw_offset, 0.0);
  // The corridor runs along +x. Vehicle +x is the heading (front face 0.571 m),
  // so the aligned yaw is the corridor axis itself, not a quarter turn from it.
  EXPECT_NEAR(params.yaw_offset, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(std::cos(aligned)), 1.0, 1e-6);
  EXPECT_NEAR(std::sin(aligned), 0.0, 1e-6);
}

TEST(TunnelYawAlignment, DisabledIsNoOp)
{
  const CorridorEsdf esdf;
  auto trajectory = makeTrajectory();
  minco_planner::TunnelYawParams params;
  EXPECT_TRUE(minco_planner::detectTunnelSegments(trajectory, esdf, params).empty());
}

TEST(TunnelYawAlignment, TurnsForwardBeforeTheFrontReachesANarrowEntrance)
{
  const CorridorEsdf esdf;
  minco_planner::ReferenceTrajectory approach;
  for (int i = 0; i < 90; ++i) {
    auto & point = approach.points.emplace_back();
    point.t = 0.1 * i;
    point.s = 0.05 * i;
    point.x = point.s;
    point.yaw = M_PI;
    point.clearance = i >= 48 && i < 75 ? 0.4 : 1.0;
  }
  minco_planner::YawSplinePlannerParams params;
  params.mode = "clearance_aware";
  params.open_area_yaw_mode = "minimal_rotation";
  params.yaw_rate_limit = 3.0;
  params.yaw_acceleration_limit = 2.0;
  params.tunnel.enabled = true;
  params.tunnel.forward_width_limit = 0.871;
  params.tunnel.front_extent = 0.3105;
  const auto tunnels = minco_planner::detectTunnelSegments(approach, esdf, params.tunnel);
  ASSERT_EQ(tunnels.size(), 1U);
  ASSERT_TRUE(tunnels.front().forward_alignment);
  EXPECT_FALSE(minco_planner::forwardTunnelEntriesAligned(approach, tunnels, params.tunnel));
  minco_planner::YawSplinePlanner(params).apply(approach, M_PI, 0.0, &tunnels);
  EXPECT_TRUE(minco_planner::forwardTunnelEntriesAligned(approach, tunnels, params.tunnel));
  EXPECT_NEAR(tunnels.front().axis_yaw.front(), 0.0, 1e-9);
}
