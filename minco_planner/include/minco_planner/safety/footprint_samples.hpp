// Copyright 2026

#ifndef MINCO_PLANNER__FOOTPRINT_SAMPLES_HPP_
#define MINCO_PLANNER__FOOTPRINT_SAMPLES_HPP_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace minco_planner
{

inline std::vector<Eigen::Vector2d> makeRectangularFootprintSamples(
  double length,
  double width,
  double safety_margin,
  double spacing)
{
  const double half_length = 0.5 * std::max(0.0, length) + std::max(0.0, safety_margin);
  const double half_width = 0.5 * std::max(0.0, width) + std::max(0.0, safety_margin);
  const double sample_spacing = std::max(0.02, spacing);
  const int samples_x = std::max(
    2, static_cast<int>(std::ceil((2.0 * half_length) / sample_spacing)));
  const int samples_y = std::max(
    2, static_cast<int>(std::ceil((2.0 * half_width) / sample_spacing)));

  std::vector<Eigen::Vector2d> samples;
  samples.reserve(static_cast<std::size_t>((samples_x + 1) * (samples_y + 1) + 1));
  for (int ix = 0; ix <= samples_x; ++ix) {
    const double x = -half_length + 2.0 * half_length * ix / static_cast<double>(samples_x);
    for (int iy = 0; iy <= samples_y; ++iy) {
      const double y = -half_width + 2.0 * half_width * iy / static_cast<double>(samples_y);
      samples.emplace_back(x, y);
    }
  }
  // Odd sample counts need not include the centre, but it is part of every footprint.
  samples.emplace_back(Eigen::Vector2d::Zero());
  return samples;
}

}  // namespace minco_planner

#endif  // MINCO_PLANNER__FOOTPRINT_SAMPLES_HPP_
