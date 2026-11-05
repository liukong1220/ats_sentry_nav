// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef ATS_ROG_MAP__PLANNING__MAPPING_HEALTH_HPP_
#define ATS_ROG_MAP__PLANNING__MAPPING_HEALTH_HPP_

#include <cmath>
#include <cstdint>

namespace ats_rog_map::planning
{

inline bool projectionIdentityMatches(
  std::uint64_t instance, std::uint64_t epoch,
  std::uint64_t source_instance, std::uint64_t source_epoch)
{
  return instance != 0 && epoch != 0 && instance == source_instance && epoch == source_epoch;
}

inline bool mappingInputHealthy(
  std::uint64_t instance, std::uint64_t epoch, double odom_silence,
  double status_age, double timeout)
{
  return instance != 0 && epoch != 0 && std::isfinite(odom_silence) &&
         std::isfinite(status_age) && std::isfinite(timeout) && timeout > 0.0 &&
         odom_silence >= 0.0 && status_age >= 0.0 &&
         odom_silence + status_age <= timeout;
}

}  // namespace ats_rog_map::planning

#endif  // ATS_ROG_MAP__PLANNING__MAPPING_HEALTH_HPP_
