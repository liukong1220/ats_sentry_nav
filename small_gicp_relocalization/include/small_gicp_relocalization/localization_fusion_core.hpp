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

#ifndef SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_
#define SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"

namespace small_gicp_relocalization
{

struct OdomPoseSample
{
  rclcpp::Time stamp;
  tf2::Transform odom_to_base;
};

struct CorrectionDelta
{
  double translation{0.0};
  double yaw{0.0};
};

struct CorrectionUpdate
{
  tf2::Transform map_to_odom{tf2::Transform::getIdentity()};
  CorrectionDelta delta;
  bool apply{false};
  bool advance_epoch{false};
};

inline std::optional<tf2::Transform> interpolateOdomPose(
  const std::deque<OdomPoseSample> & history, const rclcpp::Time & stamp,
  double boundary_tolerance_s, double maximum_interpolation_gap_s)
{
  if (history.empty()) {
    return std::nullopt;
  }
  const auto tolerance = rclcpp::Duration::from_seconds(std::max(0.0, boundary_tolerance_s));
  if (stamp + tolerance < history.front().stamp || stamp > history.back().stamp + tolerance) {
    return std::nullopt;
  }

  const auto newer = std::lower_bound(
    history.begin(), history.end(), stamp,
    [](const OdomPoseSample & sample, const rclcpp::Time & target) {
      return sample.stamp < target;
    });
  if (newer == history.begin()) {
    return newer->odom_to_base;
  }
  if (newer == history.end()) {
    return history.back().odom_to_base;
  }
  if (newer->stamp == stamp) {
    return newer->odom_to_base;
  }

  const auto older = std::prev(newer);
  const double gap = (newer->stamp - older->stamp).seconds();
  if (gap <= 0.0 || (maximum_interpolation_gap_s > 0.0 && gap > maximum_interpolation_gap_s)) {
    return std::nullopt;
  }
  const double alpha = std::clamp((stamp - older->stamp).seconds() / gap, 0.0, 1.0);
  tf2::Transform interpolated;
  interpolated.setOrigin(
    older->odom_to_base.getOrigin().lerp(newer->odom_to_base.getOrigin(), alpha));
  tf2::Quaternion rotation =
    older->odom_to_base.getRotation().slerp(newer->odom_to_base.getRotation(), alpha);
  rotation.normalize();
  interpolated.setRotation(rotation);
  return interpolated;
}

inline bool observationIsNewer(
  const rclcpp::Time & stamp, std::uint64_t sequence,
  const std::optional<rclcpp::Time> & last_stamp,
  const std::optional<std::uint64_t> & last_sequence)
{
  if (!last_stamp) {
    return true;
  }
  if (stamp > *last_stamp) {
    return true;
  }
  if (stamp < *last_stamp) {
    return false;
  }
  return !last_sequence || sequence > *last_sequence;
}

inline CorrectionDelta correctionDelta(const tf2::Transform & newer, const tf2::Transform & older)
{
  CorrectionDelta delta;
  delta.translation = (newer.getOrigin() - older.getOrigin()).length();
  const double yaw_delta = tf2::getYaw(newer.getRotation()) - tf2::getYaw(older.getRotation());
  delta.yaw = std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
  return delta;
}

inline CorrectionUpdate selectCorrectionUpdate(
  const tf2::Transform & candidate, const std::optional<tf2::Transform> & current,
  double translation_threshold, double yaw_threshold)
{
  CorrectionUpdate update;
  if (!current) {
    update.map_to_odom = candidate;
    update.apply = true;
    update.advance_epoch = true;
    return update;
  }

  update.map_to_odom = *current;
  update.delta = correctionDelta(candidate, *current);
  update.advance_epoch = update.delta.translation > std::max(0.0, translation_threshold) ||
                         update.delta.yaw > std::max(0.0, yaw_threshold);
  update.apply = update.advance_epoch;
  if (update.apply) {
    update.map_to_odom = candidate;
  }
  return update;
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__LOCALIZATION_FUSION_CORE_HPP_
