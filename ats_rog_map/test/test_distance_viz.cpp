#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "ats_rog_map/planning/distance_viz.hpp"

namespace
{

using ats_rog_map::planning::distanceColor;
using ats_rog_map::planning::distanceContourSegments;

constexpr int kOccupied = 65;
constexpr double kCircumscribed = 0.48;
constexpr double kGreen = 1.0;

TEST(DistanceViz, ColourBandFollowsOccupancyAndDistance)
{
  const auto unknown = distanceColor(-1, 2.0, kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(unknown.r, 128);
  EXPECT_EQ(unknown.g, 128);
  EXPECT_EQ(unknown.b, 128);

  const auto occupied = distanceColor(100, 2.0, kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(occupied.r + occupied.g + occupied.b, 0);
  const auto non_positive = distanceColor(0, 0.0, kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(non_positive.r + non_positive.g + non_positive.b, 0);
  const auto nan = distanceColor(
    0, std::numeric_limits<double>::quiet_NaN(), kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(nan.r + nan.g + nan.b, 0);

  const auto red = distanceColor(0, 0.3, kOccupied, kCircumscribed, kGreen);
  EXPECT_GT(red.r, 200);
  EXPECT_LT(red.g, 50);

  const auto yellow = distanceColor(0, kCircumscribed, kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(yellow.r, 240);
  EXPECT_EQ(yellow.g, 220);

  const auto green = distanceColor(0, 1.0, kOccupied, kCircumscribed, kGreen);
  const auto far = distanceColor(0, 5.0, kOccupied, kCircumscribed, kGreen);
  EXPECT_EQ(green.r, 40);
  EXPECT_EQ(green.g, 190);
  EXPECT_EQ(far.r, green.r);
  EXPECT_EQ(far.g, green.g);

  const auto mid = distanceColor(0, 0.74, kOccupied, kCircumscribed, kGreen);
  EXPECT_LT(mid.r, yellow.r);
  EXPECT_GT(mid.r, green.r);
}

TEST(DistanceViz, ContourOfLinearFieldIsAStraightLine)
{
  // field = column index (0..4) on a 5x3 grid; level 2.25 crosses between
  // columns 2 and 3, i.e. at x = 2.75 in cell units for every row square.
  const std::size_t width = 5;
  const std::size_t height = 3;
  std::vector<double> field;
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      field.push_back(static_cast<double>(x));
    }
  }
  const std::vector<std::int8_t> occupancy(width * height, 0);
  const auto segments = distanceContourSegments(field, occupancy, width, height, 2.25);
  ASSERT_EQ(segments.size(), height - 1);
  for (const auto & segment : segments) {
    EXPECT_NEAR(segment[0], 2.75, 1e-9);
    EXPECT_NEAR(segment[2], 2.75, 1e-9);
    EXPECT_NEAR(std::abs(segment[3] - segment[1]), 1.0, 1e-9);
  }
}

TEST(DistanceViz, ContourSkipsSquaresTouchingUnknownCells)
{
  const std::size_t width = 4;
  const std::size_t height = 2;
  const std::vector<double> field{0.0, 1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 3.0};
  std::vector<std::int8_t> occupancy(width * height, 0);
  ASSERT_EQ(distanceContourSegments(field, occupancy, width, height, 1.5).size(), 1U);
  occupancy[2] = -1;
  EXPECT_TRUE(distanceContourSegments(field, occupancy, width, height, 1.5).empty());
}

TEST(DistanceViz, ContourRejectsMismatchedInput)
{
  const std::vector<double> field(6, 1.0);
  const std::vector<std::int8_t> occupancy(5, 0);
  EXPECT_TRUE(distanceContourSegments(field, occupancy, 3, 2, 0.5).empty());
  EXPECT_TRUE(distanceContourSegments(field, std::vector<std::int8_t>(6, 0), 6, 1, 0.5).empty());
}

}  // namespace
