#include <cmath>

#include <gtest/gtest.h>

#include "minco_planner/trajectory/tunnel_yaw_alignment.hpp"

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
