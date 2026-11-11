// Copyright 2026

#ifndef ATS_ROG_MAP__ODOM_LAYER_HPP_
#define ATS_ROG_MAP__ODOM_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <array>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>

namespace ats_rog_map
{

enum class OdomCellState : std::uint8_t
{
  kUnknown,
  kFree,
  kOccupied
};

enum class OdomCellLayer : std::uint8_t
{
  kDynamic,
  kStatic
};

struct OdomLayerConfig
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.10};
  // Global mapping layer: 40 m x 30 m at 0.1 m resolution.  The origin is
  // the lower-left corner of the fixed global array; mapping-start coordinates
  // are kept at its centre by the default launch configuration.
  std::uint32_t width{400};
  std::uint32_t height{300};
  double static_promotion_seconds{3.0};
  double dynamic_hold_seconds{3.0};
  double max_frame_dt_seconds{0.25};
  double save_persistence_ratio{0.8};
  double cloud_resolution{0.10};
  double cloud_clear_ray_radius{0.01};
  double cloud_clear_endpoint_margin{0.20};
};

struct OdomLayerCell
{
  OdomCellState state{OdomCellState::kUnknown};
  OdomCellLayer layer{OdomCellLayer::kDynamic};
  double first_hit_time{0.0};
  double last_hit_time{0.0};
  double hit_time_accum{0.0};
  double observed_time{0.0};
  double last_visible_time{0.0};
  bool cleared_once{false};
  std::uint32_t clear_count{0};
};

struct OdomLayerTransform
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
};

struct OdomLayerSaveStats
{
  std::size_t occupied_cells{0};
  std::size_t persistent_cells{0};
  std::size_t persistent_voxels{0};
  std::size_t raster_occupied_cells{0};
  std::size_t occupied_dynamic{0};
  std::size_t occupied_cleared_once{0};
  std::size_t occupied_other{0};
};

struct OdomVoxelKeyHash
{
  std::size_t operator()(const std::array<int, 3> & key) const
  {
    return static_cast<std::size_t>(key[0]) * 73856093U ^
           static_cast<std::size_t>(key[1]) * 19349663U ^
           static_cast<std::size_t>(key[2]) * 83492791U;
  }
};

class OdomLayerStore
{
public:
  explicit OdomLayerStore(const OdomLayerConfig & config);

  const OdomLayerConfig & config() const {return config_;}
  const std::vector<OdomLayerCell> & cells() const {return cells_;}
  std::vector<OdomLayerCell> & cells() {return cells_;}

  bool validCell(int x, int y) const;
  std::size_t index(int x, int y) const;
  OdomLayerCell * cell(int x, int y);
  const OdomLayerCell * cell(int x, int y) const;

  // A hit is an occupied observation.  Promotion uses accumulated hit time,
  // not continuous visibility time.
  bool observeHit(int x, int y, double stamp, double observed_dt);
  bool observePoint(const std::array<double, 3> & point, double stamp, double observed_dt);
  bool observeFree(int x, int y, double stamp, double observed_dt);
  void clearCloudVoxel(const std::array<double, 3> & point, double stamp);
  bool hasRayClearCandidate(const std::array<double, 3> & voxel_location, double stamp) const;
  std::vector<std::array<int, 3>> rayClearCandidates(double stamp) const;
  // Coarse voxel traversal alone is not a contradiction: a grazing beam may
  // pass above a floor or beside a wall represented inside that voxel.
  void clearCloudVoxelAlongRay(const std::array<double, 3> & voxel_location,
    const std::array<double, 3> & sensor, const std::array<double, 3> & endpoint,
    double stamp);
  // Conservative area-overlap sampling in odom; outside and unobserved are unknown.
  std::int8_t sampleFootprint(double min_x, double min_y, double max_x, double max_y) const;

  // Mark a cell as visible without asserting either a hit or a ray clear.
  bool observeVisible(int x, int y, double stamp);
  // Losing support revokes free, never occupied evidence. No ray-clear claim.
  bool invalidateFree(int x, int y, double stamp);

  // A ray clear is authoritative only when the cell was visible to the ray.
  // It never turns an out-of-view cell into free space.
  bool observeRayClear(int x, int y, double stamp, bool visible);

  // Applies the out-of-view hold rule to dynamic cells.  Expired cells become
  // unknown rather than free.
  void tick(double stamp);

  // Move the odom layer into a new epoch.  The transform maps new coordinates
  // into old coordinates, matching the old_from_new contract.
  bool rebase(const OdomLayerTransform & old_from_new);
  void clear();

  // Only persistent static cells which have never been ray-cleared are saved.
  bool saveEligible(int x, int y) const;

  // Transform a single immutable save snapshot. YAML retains the rotated grid
  // origin exactly (no centre sampling or raster rounding); .pcd is persistent
  // 3D scan evidence and .cells.pcd is the separate 2D debugging artifact.
  bool save(const std::string & output_prefix, const std::string & frame_id,
    const OdomLayerTransform & map_from_odom = {}, OdomLayerSaveStats * stats = nullptr) const;

private:
  OdomLayerConfig config_;
  std::vector<OdomLayerCell> cells_;
  struct CloudVoxel
  {
    std::array<double, 3> point;
    OdomLayerCell evidence;
    std::size_t candidate_index{std::numeric_limits<std::size_t>::max()};
  };
  struct RayClearCandidate
  {
    std::array<int, 3> key;
    double last_hit_time;
  };
  void removeRayClearCandidate(CloudVoxel & voxel);
  std::unordered_map<std::array<int, 3>, CloudVoxel, OdomVoxelKeyHash> cloud_voxels_;
  std::vector<RayClearCandidate> ray_candidates_;
};

}  // namespace ats_rog_map

#endif  // ATS_ROG_MAP__ODOM_LAYER_HPP_
