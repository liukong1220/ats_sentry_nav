// Copyright 2026
//
// RViz-only rendering of the planning snapshot's signed distance field. Nothing
// here feeds the planning or safety path.

#ifndef ATS_ROG_MAP__PLANNING__DISTANCE_VIZ_HPP_
#define ATS_ROG_MAP__PLANNING__DISTANCE_VIZ_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ats_rog_map::planning
{

struct DistanceColor
{
  std::uint8_t r{0};
  std::uint8_t g{0};
  std::uint8_t b{0};
};

// Single colour band (report 2.2.1 H): occupied black, inside the footprint
// circumscribed radius red, yellow -> green up to green_distance, green beyond,
// unknown grey.
inline DistanceColor distanceColor(
  int occupancy, double distance, int occupied_threshold, double circumscribed_radius,
  double green_distance)
{
  if (occupancy < 0) {
    return {128, 128, 128};
  }
  if (occupancy >= occupied_threshold || !std::isfinite(distance) || distance <= 0.0) {
    return {0, 0, 0};
  }
  if (distance < circumscribed_radius) {
    return {220, 30, 30};
  }
  const double span = green_distance - circumscribed_radius;
  const double t = span > 1e-6 ?
    std::clamp((distance - circumscribed_radius) / span, 0.0, 1.0) : 1.0;
  // yellow (240, 220, 40) -> green (40, 190, 70)
  return {
    static_cast<std::uint8_t>(std::lround(240.0 + (40.0 - 240.0) * t)),
    static_cast<std::uint8_t>(std::lround(220.0 + (190.0 - 220.0) * t)),
    static_cast<std::uint8_t>(std::lround(40.0 + (70.0 - 40.0) * t))};
}

// Iso-distance segments by marching squares over cell-centre samples, in cell
// units (x = column + 0.5 at a cell centre). Squares touching an unknown cell
// or a non-finite sample are skipped, so no contour is drawn through unknown
// space. Each segment is {x0, y0, x1, y1}.
inline std::vector<std::array<double, 4>> distanceContourSegments(
  const std::vector<double> & field, const std::vector<std::int8_t> & occupancy,
  std::size_t width, std::size_t height, double level)
{
  std::vector<std::array<double, 4>> segments;
  if (width < 2 || height < 2 || field.size() != width * height ||
    occupancy.size() != width * height)
  {
    return segments;
  }
  const auto sample = [&](std::size_t x, std::size_t y, double & value) {
      const std::size_t index = y * width + x;
      value = field[index];
      return occupancy[index] >= 0 && std::isfinite(value);
    };
  for (std::size_t y = 0; y + 1 < height; ++y) {
    for (std::size_t x = 0; x + 1 < width; ++x) {
      // Corners counter-clockwise from the lower-left cell centre.
      std::array<double, 4> v{};
      if (!sample(x, y, v[0]) || !sample(x + 1, y, v[1]) ||
        !sample(x + 1, y + 1, v[2]) || !sample(x, y + 1, v[3]))
      {
        continue;
      }
      const std::array<std::array<double, 2>, 4> corner{{
        {static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5},
        {static_cast<double>(x) + 1.5, static_cast<double>(y) + 0.5},
        {static_cast<double>(x) + 1.5, static_cast<double>(y) + 1.5},
        {static_cast<double>(x) + 0.5, static_cast<double>(y) + 1.5}}};
      std::vector<std::array<double, 2>> crossings;
      for (int edge = 0; edge < 4; ++edge) {
        const int a = edge;
        const int b = (edge + 1) % 4;
        const bool above_a = v[a] >= level;
        const bool above_b = v[b] >= level;
        if (above_a == above_b) {
          continue;
        }
        const double t = (level - v[a]) / (v[b] - v[a]);
        crossings.push_back({
          corner[a][0] + t * (corner[b][0] - corner[a][0]),
          corner[a][1] + t * (corner[b][1] - corner[a][1])});
      }
      if (crossings.size() == 2) {
        segments.push_back({crossings[0][0], crossings[0][1], crossings[1][0], crossings[1][1]});
      } else if (crossings.size() == 4) {
        // Saddle: pair by the centre value so the two branches do not cross.
        const double centre = 0.25 * (v[0] + v[1] + v[2] + v[3]);
        if ((centre >= level) == (v[0] >= level)) {
          segments.push_back({crossings[0][0], crossings[0][1], crossings[1][0], crossings[1][1]});
          segments.push_back({crossings[2][0], crossings[2][1], crossings[3][0], crossings[3][1]});
        } else {
          segments.push_back({crossings[0][0], crossings[0][1], crossings[3][0], crossings[3][1]});
          segments.push_back({crossings[1][0], crossings[1][1], crossings[2][0], crossings[2][1]});
        }
      }
    }
  }
  return segments;
}

}  // namespace ats_rog_map::planning

#endif  // ATS_ROG_MAP__PLANNING__DISTANCE_VIZ_HPP_
