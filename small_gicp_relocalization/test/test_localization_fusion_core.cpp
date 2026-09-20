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

#include <cmath>
#include <deque>
#include <optional>

#include "small_gicp_relocalization/localization_fusion_core.hpp"
#include "small_gicp_relocalization/localization_status_input_core.hpp"

namespace small_gicp_relocalization
{

namespace
{

tf2::Transform makeTransform(double x, double yaw)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(x, 0.0, 0.0));
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  transform.setRotation(rotation);
  return transform;
}

rclcpp::Time seconds(double value)
{
  return rclcpp::Time(static_cast<std::int64_t>(value * 1e9), RCL_ROS_TIME);
}

}  // namespace

TEST(LocalizationFusionCore, InterpolatesPoseAtObservationTime)
{
  std::deque<OdomPoseSample> history{
    OdomPoseSample{seconds(1.0), makeTransform(0.0, 0.0)},
    OdomPoseSample{seconds(3.0), makeTransform(2.0, M_PI_2)}};

  const auto interpolated = interpolateOdomPose(history, seconds(2.0), 0.1, 3.0);

  ASSERT_TRUE(interpolated.has_value());
  EXPECT_NEAR(interpolated->getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(interpolated->getRotation()), M_PI_4, 1e-9);
}

TEST(LocalizationFusionCore, RejectsUncoveredOrExcessivelySparseHistory)
{
  std::deque<OdomPoseSample> history{
    OdomPoseSample{seconds(1.0), makeTransform(0.0, 0.0)},
    OdomPoseSample{seconds(3.0), makeTransform(2.0, 0.0)}};

  EXPECT_FALSE(interpolateOdomPose(history, seconds(0.5), 0.1, 3.0).has_value());
  EXPECT_FALSE(interpolateOdomPose(history, seconds(2.0), 0.1, 0.5).has_value());
}

TEST(LocalizationFusionCore, UsesStampBeforeSequenceAcrossPublisherRestart)
{
  const std::optional<rclcpp::Time> last_stamp = seconds(5.0);
  const std::optional<std::uint64_t> last_sequence = 100;

  EXPECT_FALSE(observationIsNewer(seconds(4.0), 101, last_stamp, last_sequence));
  EXPECT_FALSE(observationIsNewer(seconds(5.0), 100, last_stamp, last_sequence));
  EXPECT_TRUE(observationIsNewer(seconds(5.0), 101, last_stamp, last_sequence));
  EXPECT_TRUE(observationIsNewer(seconds(6.0), 1, last_stamp, last_sequence));
}

TEST(LocalizationFusionCore, RejectsDelayedOrFarFutureObservationStamp)
{
  EXPECT_NE(validateObservationStamp(seconds(1.0), seconds(3.0), 1.0, 0.25), "");
  EXPECT_NE(validateObservationStamp(seconds(3.5), seconds(3.0), 1.0, 0.25), "");
  EXPECT_EQ(validateObservationStamp(seconds(2.2), seconds(3.0), 1.0, 0.25), "");
}

TEST(LocalizationFusionCore, MeasuresWrappedCorrectionDelta)
{
  const auto older = makeTransform(0.0, M_PI - 0.05);
  const auto newer = makeTransform(0.3, -M_PI + 0.05);

  const CorrectionDelta delta = correctionDelta(newer, older);

  EXPECT_NEAR(delta.translation, 0.3, 1e-9);
  EXPECT_NEAR(delta.yaw, 0.1, 1e-9);
}

TEST(LocalizationFusionCore, AppliesOnlyCorrectionsThatAdvanceEpoch)
{
  const tf2::Transform current = makeTransform(1.0, 0.10);
  const tf2::Transform small_candidate = makeTransform(1.02, 0.12);
  const CorrectionUpdate retained = selectCorrectionUpdate(small_candidate, current, 0.05, 0.05);
  EXPECT_FALSE(retained.apply);
  EXPECT_FALSE(retained.advance_epoch);
  EXPECT_NEAR(retained.map_to_odom.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(retained.map_to_odom.getRotation()), 0.10, 1e-9);

  const tf2::Transform large_candidate = makeTransform(1.08, 0.12);
  const CorrectionUpdate applied = selectCorrectionUpdate(large_candidate, current, 0.05, 0.05);
  EXPECT_TRUE(applied.apply);
  EXPECT_TRUE(applied.advance_epoch);
  EXPECT_NEAR(applied.map_to_odom.getOrigin().x(), 1.08, 1e-9);

  const CorrectionUpdate initialized =
    selectCorrectionUpdate(small_candidate, std::nullopt, 0.05, 0.05);
  EXPECT_TRUE(initialized.apply);
  EXPECT_TRUE(initialized.advance_epoch);
}

TEST(LocalizationFusionCore, CountCapPreservesNewestInterpolationAndRejectsEvictedSamples)
{
  std::deque<OdomPoseSample> history;
  for (int index = 0; index < 5; ++index) {
    history.push_back({seconds(1.0 + index * 0.1), makeTransform(index, 0.0)});
  }
  // Time eviction removes the first sample, then the count cap removes two more.
  EXPECT_EQ(pruneOdomHistory(history, seconds(1.1), 2), 2U);
  ASSERT_EQ(history.size(), 2U);
  EXPECT_EQ(history.front().stamp, seconds(1.3));
  EXPECT_EQ(pruneOdomHistory(history, seconds(1.1), 2), 0U);
  EXPECT_FALSE(interpolateOdomPose(history, seconds(1.2), 0.02, 0.2));
  const auto pose = interpolateOdomPose(history, seconds(1.35), 0.02, 0.2);
  ASSERT_TRUE(pose);
  EXPECT_NEAR(pose->getOrigin().x(), 3.5, 1e-7);
  EXPECT_EQ(pruneOdomHistory(history, seconds(2.0), 2), 0U);
  EXPECT_TRUE(history.empty());
}

TEST(LocalizationStatusInput, ChecksExactFrameAndCanonicalNonzeroStamp)
{
  const auto validate = [](const std::string & frame, int sec, std::uint32_t ns) {
    return validateLocalizationStatusMetadata(
      frame, "map", sec, ns, 2000000000LL, 0, 1, 0, true, 1.0, 0.1);
  };
  EXPECT_EQ(validate("map", 2, 0), "");
  EXPECT_NE(validate("", 2, 0), "");
  EXPECT_NE(validate("/map", 2, 0), "");
  EXPECT_NE(validate("odom", 2, 0), "");
  EXPECT_NE(validate("map", 0, 0), "");
  EXPECT_NE(validate("map", -1, 0), "");
  EXPECT_NE(validate("map", 1, 1000000000U), "");
}

TEST(LocalizationStatusInput, FreshnessBoundariesAndReplayAreFailClosed)
{
  const auto validate = [](int sec, std::uint32_t ns, std::int64_t last = 0) {
    return validateLocalizationStatusMetadata(
      "map", "map", sec, ns, 2000000000LL, last, 2, 2, true, 1.0, 0.1);
  };
  EXPECT_EQ(validate(1, 0), "");
  EXPECT_NE(validate(0, 999999999), "");
  EXPECT_EQ(validate(2, 100000000), "");
  EXPECT_NE(validate(2, 100000001), "");
  EXPECT_NE(validate(2, 0, 2000000000LL), "");
  EXPECT_NE(validate(2, 0, 2000000001LL), "");
  EXPECT_NE(validateLocalizationStatusMetadata(
    "map", "map", 2, 0, 0, 0, 1, 0, true, 1.0, 0.1), "");
}

TEST(LocalizationStatusInput, AllowsZeroEpochBootstrapButNeverTrackingOrRollback)
{
  const auto validate = [](std::uint64_t epoch, std::uint64_t last, bool tracking) {
    return validateLocalizationStatusMetadata(
      "map", "map", 2, 0, 2000000000LL, 0, epoch, last, tracking, 1.0, 0.1);
  };
  EXPECT_EQ(validate(0, 0, false), "");
  EXPECT_NE(validate(0, 0, true), "");
  EXPECT_EQ(validate(1, 0, true), "");
  EXPECT_EQ(validate(2, 2, true), "");
  EXPECT_EQ(validate(3, 2, true), "");
  EXPECT_NE(validate(1, 2, true), "");
  EXPECT_NE(validate(0, 2, false), "");
}

}  // namespace small_gicp_relocalization
