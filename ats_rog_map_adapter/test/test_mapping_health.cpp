// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <limits>

#include "ats_rog_map_adapter/mapping_health.hpp"

TEST(MappingHealth, RejectsUnknownOrStaleFrontend)
{
  using ats_rog_map_adapter::mappingInputHealthy;
  EXPECT_FALSE(mappingInputHealthy(0, 1, 0.1, 0.1, 1.0));
  EXPECT_FALSE(mappingInputHealthy(1, 0, 0.1, 0.1, 1.0));
  EXPECT_FALSE(mappingInputHealthy(1, 1, 0.8, 0.3, 1.0));
  EXPECT_FALSE(mappingInputHealthy(1, 1, -0.1, 0.1, 1.0));
  EXPECT_FALSE(mappingInputHealthy(1, 1, std::numeric_limits<double>::infinity(), 0.1, 1.0));
  EXPECT_TRUE(mappingInputHealthy(1, 1, 0.2, 0.3, 1.0));
}
