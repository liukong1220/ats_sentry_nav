// Copyright 2026 Lihan Chen
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

#ifndef SMALL_GICP_RELOCALIZATION__LOCALIZATION_STATUS_INPUT_CORE_HPP_
#define SMALL_GICP_RELOCALIZATION__LOCALIZATION_STATUS_INPUT_CORE_HPP_

#include <cmath>
#include <cstdint>
#include <string>

namespace small_gicp_relocalization
{

template<typename LS>
inline bool isKnownLocalizationState(std::uint8_t state)
{
  switch (state) {
    case LS::STATE_UNINITIALIZED:
    case LS::STATE_BOOTSTRAP:
    case LS::STATE_TRACKING:
    case LS::STATE_DEGRADED:
    case LS::STATE_LOST:
    case LS::STATE_RELOCALIZING:
    case LS::STATE_CONFIRMED:
      return true;
    default:
      return false;
  }
}

/// Admission only: callers must commit state and refresh their lease only on success.
/// Epoch ordering is local to this consumer lifetime, not a producer incarnation ID.
inline std::string validateLocalizationStatusMetadata(
  const std::string & frame, const std::string & expected_frame,
  std::int32_t stamp_sec, std::uint32_t stamp_nanosec, std::int64_t now_ns,
  std::int64_t last_stamp_ns, std::uint64_t epoch, std::uint64_t last_epoch,
  bool tracking, double max_age_s, double max_future_s)
{
  if (frame.empty() || frame != expected_frame) {
    return "localization status frame mismatch";
  }
  if (stamp_sec < 0 || stamp_nanosec >= 1000000000U) {
    return "invalid localization status stamp";
  }
  const std::int64_t stamp_ns = static_cast<std::int64_t>(stamp_sec) * 1000000000LL + stamp_nanosec;
  if (stamp_ns <= 0 || now_ns <= 0) {
    return "localization status clock is uninitialized";
  }
  if (stamp_ns <= last_stamp_ns) {
    return "localization status stamp is not increasing";
  }
  if (!std::isfinite(max_age_s) || !std::isfinite(max_future_s)) {
    return "invalid localization status freshness limits";
  }
  const double age_s = static_cast<double>(now_ns - stamp_ns) * 1e-9;
  if (max_age_s > 0.0 && age_s > max_age_s) {
    return "localization status stamp is stale";
  }
  if (max_future_s >= 0.0 && age_s < -max_future_s) {
    return "localization status stamp is in the future";
  }
  if (epoch < last_epoch || (epoch == 0 && tracking)) {
    return "localization status epoch is invalid or regressed";
  }
  return {};
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__LOCALIZATION_STATUS_INPUT_CORE_HPP_
