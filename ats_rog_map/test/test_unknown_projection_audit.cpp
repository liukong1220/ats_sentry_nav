// Copyright 2026

#include <cmath>

#include "ats_rog_map/unknown_projection_audit.hpp"
#include "gtest/gtest.h"

namespace
{

nav_msgs::msg::OccupancyGrid makeAllUnknownGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.5F;
  grid.info.width = 2U;
  grid.info.height = 2U;
  grid.info.origin.position.x = 3.0;
  grid.info.origin.position.y = -2.0;
  grid.info.origin.position.z = 0.25;
  grid.info.origin.orientation.z = std::sqrt(0.5);
  grid.info.origin.orientation.w = std::sqrt(0.5);
  grid.data.assign(4U, -1);
  return grid;
}

TEST(UnknownProjectionAudit, EmitsEveryCellCenterForStrictAllUnknownGrid)
{
  const auto points = ats_rog_map::makeAllUnknownProjectionAudit(makeAllUnknownGrid());

  ASSERT_EQ(points.size(), 4U);
  EXPECT_NEAR(points.front().x(), 2.75F, 1e-5F);
  EXPECT_NEAR(points.front().y(), -1.75F, 1e-5F);
  EXPECT_NEAR(points.front().z(), 0.25F, 1e-5F);
  EXPECT_NEAR(points.back().x(), 2.25F, 1e-5F);
  EXPECT_NEAR(points.back().y(), -1.25F, 1e-5F);
}

TEST(UnknownProjectionAudit, RefusesMixedOrMalformedPayloads)
{
  auto mixed = makeAllUnknownGrid();
  mixed.data[2] = 0;
  EXPECT_TRUE(ats_rog_map::makeAllUnknownProjectionAudit(mixed).empty());

  auto malformed = makeAllUnknownGrid();
  malformed.data.pop_back();
  EXPECT_TRUE(ats_rog_map::makeAllUnknownProjectionAudit(malformed).empty());
}

}  // namespace
