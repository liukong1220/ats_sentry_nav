#ifndef TERRAIN_ANALYSIS_EXT__TERRAIN_SLOPE_GRID_HPP_
#define TERRAIN_ANALYSIS_EXT__TERRAIN_SLOPE_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "terrain_analysis_ext/overhead_filter.hpp"

extern bool checkTerrainConn;
extern double traversabilityObstacleHeightThre;
extern double traversabilitySafeHeightThre;
extern double traversabilityOccupancyRatioThre;
extern double traversabilityGroundConfidenceThre;
extern double traversabilityHeightWeight;
extern double traversabilityOccupancyWeight;
extern double traversabilityGroundWeight;
extern double traversabilityDebugHeightDiffCap;
extern double traversabilityDebugOccupancyRatioCap;
extern int traversabilityMinPointCount;
extern bool traversabilityUnknownAsOccupied;
extern bool publishTraversabilityDebugGrids;
extern double slopeGridMaxDeg;
extern double slopeGentleDegThre;
extern double slopeModerateDegThre;
extern double slopeSteepDegThre;
extern bool useSlopeAsObstacle;
extern double slopeObstacleDegThre;
extern float planarVoxelSize;
extern int planarVoxelWidth;
extern int planarVoxelHalfWidth;
extern int planarVoxelNum;
extern float planarAnchorX;
extern float planarAnchorY;
extern std::vector<float> planarVoxelElev;
extern std::vector<int> planarVoxelConn;
extern std::vector<float> planarVoxelMaxRelHeight;
extern std::vector<float> planarVoxelMinZ;
extern std::vector<float> planarVoxelMaxZ;
extern std::vector<float> planarVoxelHeightDiff;
extern std::vector<float> planarVoxelOccupancyRatio;
extern std::vector<float> planarVoxelGroundConfidence;
extern std::vector<float> planarVoxelSlopeDeg;
extern std::vector<float> planarVoxelSlopeBand;
extern std::vector<int> planarVoxelPointCount;

double clampUnit(double value);
int8_t normalizedToOccupancy(double value);
double slopeDegToBand(double slope_deg);
void computeSlopeGrid();
nav_msgs::msg::OccupancyGrid makePlanarGridMessage(const rclcpp::Time & stamp);
std::size_t planarIndexToOccupancyGridIndex(int planar_ind);
void publishScalarGrid(
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub,
  const rclcpp::Time & stamp,
  const std::vector<float> & values,
  double scale_cap);
void publishTraversabilityGrid(
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_traversability,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_slope,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_slope_band,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_height_diff,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_occupancy_ratio,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub_ground_confidence,
  const rclcpp::Time & stamp);

#endif  // TERRAIN_ANALYSIS_EXT__TERRAIN_SLOPE_GRID_HPP_
