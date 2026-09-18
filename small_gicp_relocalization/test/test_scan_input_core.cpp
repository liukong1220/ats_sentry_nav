// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "small_gicp_relocalization/scan_input_core.hpp"

namespace small_gicp_relocalization
{

TEST(ScanInputCore, RejectsFrameAndTimestampViolations)
{
  ScanInputConfig config;
  config.expected_frame = "front_mid360";
  config.max_age_s = 0.5;
  config.max_future_s = 0.1;

  EXPECT_NE(validateScanMetadata("", 2'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("rear_mid360", 2'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("front_mid360", 0, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("front_mid360", 1'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(
    validateScanMetadata("front_mid360", 1'000'000'000, 1'000'000'000, 1'000'000'000, config), "");
  EXPECT_NE(validateScanMetadata("front_mid360", 2'000'000'000, 1'000'000'000, 0, config), "");
  EXPECT_EQ(validateScanMetadata("front_mid360", 1'800'000'000, 2'000'000'000, 0, config), "");
}

TEST(ScanInputCore, KeepsOnlyFiniteInRangePoints)
{
  ScanInputConfig config;
  config.min_range_m = 0.1;
  config.max_range_m = 10.0;
  config.min_z_m = -1.0;
  config.max_z_m = 2.0;

  EXPECT_TRUE(scanPointInRange(1.0F, 2.0F, 0.5F, config));
  EXPECT_FALSE(scanPointInRange(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, config));
  EXPECT_FALSE(scanPointInRange(0.0F, 0.0F, 0.0F, config));
  EXPECT_FALSE(scanPointInRange(20.0F, 0.0F, 0.0F, config));
  EXPECT_FALSE(scanPointInRange(1.0F, 0.0F, 3.0F, config));
}

TEST(ScanInputCore, RejectsLowValidPointRatio)
{
  ScanInputConfig config;
  config.min_valid_ratio = 0.75;
  ScanInputStats stats;
  stats.total_points = 100;
  stats.finite_points = 80;
  stats.valid_points = 70;
  EXPECT_NE(validateScanPointStats(stats, config), "");
  stats.valid_points = 75;
  EXPECT_EQ(validateScanPointStats(stats, config), "");
}

}  // namespace small_gicp_relocalization
