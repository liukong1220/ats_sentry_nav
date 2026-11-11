// Copyright 2026

#ifndef ATS_ROG_MAP__GROUND_COLUMN_HPP_
#define ATS_ROG_MAP__GROUND_COLUMN_HPP_

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <vector>

namespace ats_rog_map
{
enum class ColumnState {kUnknown, kFree, kOccupied};

struct GroundColumnConfig
{
  int min_observed{1};
  double voxel_resolution{0.05};
  double surface_max{0.10};
  double wall_span_min{0.20};
  double wall_ratio_min{0.80};
  double tunnel_span_min{0.24};
  double tunnel_span_max{0.40};
  double tunnel_ratio_max{0.55};
  double robot_height{0.255};
  double top_margin{0.05};
  double max_ground_slope{0.6};
  double ground_seed_radius{0.65};
  // Plane-fit residual limit and tolerated support inconsistency (m).
  double support_noise{0.005};
};

// All comparisons use the local connected ground, never the robot's plane.
ColumnState classifyGroundColumn(
  const std::vector<double> & occupied_z, int observed_count,
  std::optional<double> ground_z, const GroundColumnConfig & config);

// Fixed odom array of column evidence. Ground is the lowest surface connected
// to the robot's known supporting surface, with bounded slope. A floating
// plate cannot bootstrap its own ground; unsupported columns stay unknown.
class GroundColumnGrid
{
public:
  GroundColumnGrid(int width, int height, double resolution, double origin_x,
    double origin_y, GroundColumnConfig config);
  void observe(const std::vector<std::array<double, 3>> & points,
    const std::array<double, 3> & robot_ground);
  ColumnState state(int x, int y) const;
  std::optional<double> ground(int x, int y) const;
  void clear();
  // Return true only when a connected support was actually removed.
  bool clearVoxel(const std::array<double, 3> & point);
  bool clearVoxel(const std::array<double, 3> & point, double ray_z);
  // ray is the sample where the ray crosses the voxel; its XY bounds the slope.
  bool clearVoxel(const std::array<double, 3> & point, const std::array<double, 3> & ray);
  void observeRayCell(int x, int y);
  std::size_t outOfRange() const {return out_of_range_;}
  const GroundColumnConfig & config() const {return config_;}

private:
  using Point = std::array<double, 3>;
  struct VoxelReturns
  {
    Point low;
    Point high;
    std::array<double, 4> xy_bounds;  // min x/y, max x/y of every retained return
  };
  // Bounds at the query XY, never a neighbouring column's unshifted height.
  std::optional<std::array<double, 2>> supportBounds(int x, int y,
    double query_x, double query_y) const;
  bool valid(int x, int y) const;
  std::size_t index(int x, int y) const;
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  GroundColumnConfig config_;
  std::vector<std::map<int, VoxelReturns>> columns_;
  std::vector<std::optional<VoxelReturns>> ground_;
  std::vector<bool> ray_observed_;
  std::size_t out_of_range_{0};
};
}  // namespace ats_rog_map
#endif  // ATS_ROG_MAP__GROUND_COLUMN_HPP_
