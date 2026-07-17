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

}  // namespace small_gicp_relocalization
