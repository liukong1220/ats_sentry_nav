// Copyright 2026

#ifndef FAKE_VEL_TRANSFORM__FAKE_YAW_MATH_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_YAW_MATH_HPP_

#include <cmath>

namespace fake_vel_transform
{

struct PlanarVelocity
{
  double x{0.0};
  double y{0.0};
};

inline double normalizeYaw(double yaw) { return std::atan2(std::sin(yaw), std::cos(yaw)); }

// The fake frame keeps the yaw observed at startup.  This returns the yaw from
// the physical gimbal frame to that stable frame.
inline double gimbalToFakeYaw(double initial_gimbal_yaw, double current_gimbal_yaw)
{
  return normalizeYaw(initial_gimbal_yaw - current_gimbal_yaw);
}

// Velocity commands expressed by fake_yaw must be rotated into the physical
// gimbal frame with the inverse of gimbal->fake.
inline PlanarVelocity fakeToGimbalVelocity(
  const PlanarVelocity & velocity, double initial_gimbal_yaw, double current_gimbal_yaw)
{
  const double yaw = normalizeYaw(current_gimbal_yaw - initial_gimbal_yaw);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {cosine * velocity.x - sine * velocity.y, sine * velocity.x + cosine * velocity.y};
}

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_YAW_MATH_HPP_
