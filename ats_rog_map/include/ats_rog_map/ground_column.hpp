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
  // A plane enclosing every support measurement within this residual is accepted
  // as the local floor (interval = plane +- residual). Steps and walls exceed it.
  double plane_residual_max{0.02};
  // Overhead plane semantics: non-floor returns of one flat board are fitted as a
  // plane; returns inside +-ceiling_plane_band are the board surface and are judged
  // at the plane height. Anything lower than the band stays an obstacle as measured.
  bool ceiling_plane_enabled{true};
  double ceiling_plane_band{0.015};
  int ceiling_plane_radius{2};            // columns, so a 5 x 5 window at 0.1 m
  int ceiling_plane_min_points{12};
  double ceiling_plane_min_inlier_ratio{0.6};
  double ceiling_plane_min_extent{0.30};  // m, inlier XY extent along both axes
  double ceiling_plane_max_slope{0.2};
  double ceiling_plane_max_sigma{0.008};  // m, inlier residual standard deviation
  // The connected floor under a board is one flat surface too. A trimmed plane fit
  // over the connected support returns (outliers beyond floor_plane_band dropped)
  // replaces the per-voxel extrema as the support estimate when it is well posed.
  bool floor_plane_enabled{true};
  double floor_plane_band{0.015};
};

// Measured clearance of the lowest return above the connected support interval.
struct ColumnHeadroom
{
  double ceiling_low;
  double support_lower;
  double support_upper;
  // Support height used for the gate: the fitted plane when one encloses every
  // connected support within plane_residual_max, otherwise support_upper.
  double support_estimate;
  double headroom;  // ceiling_low - support_estimate, the quantity gated by the robot height
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
  // Diagnostic view of what state() compares against the gate; no new decision.
  std::optional<ColumnHeadroom> ceilingHeadroom(int x, int y) const;
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
  // plane_z (optional) receives the fitted plane height at the query, or NaN
  // when no valid plane produced the bounds.
  std::optional<std::array<double, 2>> supportBounds(int x, int y,
    double query_x, double query_y, double * plane_z = nullptr) const;
  struct SupportInterval
  {
    double lower;
    double upper;     // conservative: never below any admissible floor
    double estimate;  // plane height (max over the voxel XY extent), else upper
  };
  // Overhead plane z = c + a (x - x0) + b (y - y0) fitted to the non-floor returns
  // around a column. Valid only for a single flat surface (e.g. a tunnel board).
  struct CeilingPlane
  {
    double x0, y0, c, a, b;
    double tol{0.0};  // inlier limit of the fit: only returns this close are the board
    double at(double x, double y) const {return c + a * (x - x0) + b * (y - y0);}
  };
  std::optional<CeilingPlane> ceilingPlane(int x, int y) const;
  std::optional<CeilingPlane> floorPlane(int x, int y) const;
  std::optional<CeilingPlane> robustPlane(
    const std::vector<std::array<double, 3>> & points, double band, double max_slope) const;
  // Height used for the gate: the plane when the return is within the fit's own
  // inlier tolerance (the board surface), else the raw return.
  static double snapToPlane(
    const std::optional<CeilingPlane> & plane, const std::array<double, 3> & p, double band);
  // Clearance of a voxel whose stored extremes both lie on the ceiling plane:
  // min over the voxel XY corners of (ceiling plane - floor plane), same XY for both.
  std::optional<double> boardClearance(
    int x, int y, const VoxelReturns & evidence, const std::optional<CeilingPlane> & ceiling) const;
  std::optional<SupportInterval> voxelSupportInterval(int x, int y,
    const VoxelReturns & evidence) const;
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
