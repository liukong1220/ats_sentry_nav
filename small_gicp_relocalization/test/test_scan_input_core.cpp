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

#include <numeric>
#include <vector>

#include "small_gicp_relocalization/scan_input_core.hpp"

namespace small_gicp_relocalization
{

TEST(ScanInputCore, RejectsFrameAndTimestampViolations)
{
  ScanInputConfig config;
  config.expected_frame = "odom";
  config.max_age_s = 0.5;
  config.max_future_s = 0.1;

  EXPECT_NE(validateScanMetadata("", 2'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("front_mid360", 2'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("odom", 0, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("odom", 1'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(
    validateScanMetadata("odom", 1'000'000'000, 1'000'000'000, 1'000'000'000, config), "");
  EXPECT_NE(validateScanMetadata("odom", 2'000'000'000, 1'000'000'000, 0, config), "");
  EXPECT_EQ(validateScanMetadata("odom", 1'800'000'000, 2'000'000'000, 0, config), "");
}

TEST(ScanInputCore, EmptyExpectedFrameDoesNotAcceptArbitraryCoordinates)
{
  ScanInputConfig config;
  EXPECT_NE(validateScanMetadata("front_mid360", 2'000'000'000, 2'000'000'000, 0, config), "");
  config.expected_frame = "custom_odom";
  EXPECT_EQ(validateScanMetadata("custom_odom", 2'000'000'000, 2'000'000'000, 0, config), "");
  EXPECT_NE(validateScanMetadata("odom", 2'000'000'000, 2'000'000'000, 0, config), "");
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

using ScanWindow = ScanAccumulationWindow<std::vector<int>>;

TEST(ScanInputCore, DenseScansReachRequiredFramesAndRetainFullTemporalWindow)
{
  ScanWindow window(40000, 30, 3);
  for (int frame = 1; frame <= 60; ++frame) {
    const auto trim = window.append(std::vector<int>(16400, frame), frame * 50000000LL);
    EXPECT_EQ(trim.sampled_points, 15067U);
    EXPECT_EQ(trim.evicted_frames, frame > 30 ? 1U : 0U);
    EXPECT_EQ(window.frameCount(), static_cast<std::size_t>(std::min(frame, 30)));
    EXPECT_EQ(window.pointCount(), window.frameCount() * 1333U);
    EXPECT_LE(window.pointCount(), 40000U);
    ASSERT_TRUE(window.firstStamp());
    EXPECT_EQ(*window.firstStamp(), std::max(1, frame - 29) * 50000000LL);
    std::vector<int> assembled;
    window.assemble(assembled);
    ASSERT_EQ(assembled.size(), window.pointCount());
    EXPECT_EQ(assembled.front(), std::max(1, frame - 29));
    EXPECT_EQ(assembled.back(), frame);
  }
}

TEST(ScanInputCore, OversizedScanSamplesAcrossWholeScanDeterministically)
{
  std::vector<int> points(100001);
  std::iota(points.begin(), points.end(), 0);
  ScanWindow window(40000, 30, 3);
  const auto trim = window.append(points, 1);
  EXPECT_EQ(trim.sampled_points, 98668U);
  EXPECT_EQ(trim.evicted_frames, 0U);
  EXPECT_EQ(window.pointCount(), 1333U);
  std::vector<int> assembled;
  window.assemble(assembled);
  ASSERT_EQ(assembled.size(), 1333U);
  EXPECT_EQ(assembled.front(), 0);
  EXPECT_EQ(assembled.back(), 100000);
  for (std::size_t i = 1; i < assembled.size(); ++i) {
    EXPECT_GE(assembled[i] - assembled[i - 1], 75);
    EXPECT_LE(assembled[i] - assembled[i - 1], 76);
  }
  window.clear();
  window.append(std::move(points), 2);
  std::vector<int> repeated;
  window.assemble(repeated);
  EXPECT_EQ(repeated, assembled);
}

TEST(ScanInputCore, SparseScansPreservePointsAndEvictOnlyOldestFrames)
{
  ScanWindow window(12, 4, 3);
  window.append({1}, 10);
  window.append({2, 3}, 20);
  window.append({4}, 30);
  window.append({5, 6}, 40);
  const auto trim = window.append({7, 8}, 50);
  EXPECT_EQ(trim.sampled_points, 0U);
  EXPECT_EQ(trim.evicted_frames, 1U);
  EXPECT_EQ(window.frameCount(), 4U);
  EXPECT_EQ(window.firstStamp(), 20);
  std::vector<int> assembled;
  window.assemble(assembled);
  EXPECT_EQ(assembled, (std::vector<int>{2, 3, 4, 5, 6, 7, 8}));
}

TEST(ScanInputCore, FullPointBudgetAndClearDropAllOldFrames)
{
  ScanWindow window(12, 4, 3);
  for (int i = 1; i <= 4; ++i) {
    window.append({i, i, i}, i);
  }
  const auto trim = window.append({5, 5, 5}, 5);
  EXPECT_EQ(trim.evicted_frames, 1U);
  EXPECT_EQ(window.pointCount(), 12U);
  EXPECT_EQ(window.firstStamp(), 2);
  window.clear();
  EXPECT_EQ(window.frameCount(), 0U);
  EXPECT_EQ(window.pointCount(), 0U);
  EXPECT_FALSE(window.firstStamp());
  std::vector<int> assembled{99};
  window.assemble(assembled);
  EXPECT_TRUE(assembled.empty());
  window.append({21, 22}, 100);
  window.assemble(assembled);
  EXPECT_EQ(assembled, (std::vector<int>{21, 22}));
  EXPECT_EQ(window.firstStamp(), 100);
}

TEST(ScanInputCore, MinimumBudgetRetainsOnePointFromEachRequiredScan)
{
  ScanWindow window(3, 3, 3);
  for (int i = 1; i <= 3; ++i) {
    window.append({i - 1, i, i + 1}, i);
  }
  EXPECT_EQ(window.frameCount(), 3U);
  EXPECT_EQ(window.pointCount(), 3U);
  std::vector<int> assembled;
  window.assemble(assembled);
  EXPECT_EQ(assembled, (std::vector<int>{1, 2, 3}));
}

TEST(ScanInputCore, RejectsImpossibleAccumulationParametersAndEmptyScans)
{
  EXPECT_THROW(ScanWindow(40000, 2, 3), std::invalid_argument);
  EXPECT_THROW(ScanWindow(2, 30, 3), std::invalid_argument);
  EXPECT_THROW(ScanWindow(20, 30, 3), std::invalid_argument);
  EXPECT_THROW(ScanWindow(0, 30, 1), std::invalid_argument);
  EXPECT_THROW(ScanWindow(40000, 0, 1), std::invalid_argument);
  EXPECT_THROW(ScanWindow(40000, 30, 0), std::invalid_argument);
  ScanWindow window(40000, 30, 3);
  EXPECT_THROW(window.append({}, 1), std::invalid_argument);
  EXPECT_EQ(window.frameCount(), 0U);
}

}  // namespace small_gicp_relocalization
