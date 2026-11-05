// Copyright 2026

#include <gtest/gtest.h>

#include "terrain_analysis_ext/self_filter.hpp"

using terrain_analysis_ext::insideSelfFilter;

TEST(SelfFilter, DisabledByDefaultRadius)
{
  EXPECT_FALSE(insideSelfFilter(0.1f, -0.37f, 0.0f, 0.0f, 0.0));
  EXPECT_FALSE(insideSelfFilter(0.0f, 0.0f, 0.0f, 0.0f, -1.0));
}

TEST(SelfFilter, DropsReturnsBesideTheChassis)
{
  // Right-side wheel return seen in domain 197: 0.38 m from the base centre.
  EXPECT_TRUE(insideSelfFilter(0.1f, -0.37f, 0.0f, 0.0f, 0.45));
  EXPECT_TRUE(insideSelfFilter(15.1f, 4.0f, 15.0f, 4.3f, 0.45));
}

TEST(SelfFilter, KeepsReturnsOutsideTheRadius)
{
  EXPECT_FALSE(insideSelfFilter(0.5f, 0.0f, 0.0f, 0.0f, 0.45));
  EXPECT_FALSE(insideSelfFilter(0.0f, -0.46f, 0.0f, 0.0f, 0.45));
}
