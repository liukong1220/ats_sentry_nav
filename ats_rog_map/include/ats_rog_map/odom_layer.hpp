// Copyright 2026

#ifndef ATS_ROG_MAP__ODOM_LAYER_HPP_
#define ATS_ROG_MAP__ODOM_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <array>
#include <string>
#include <vector>

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

  // Mark a cell as visible without asserting either a hit or a ray clear.
  bool observeVisible(int x, int y, double stamp);

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

  // Writes the persistent odom layer as map_server-compatible PGM/YAML and
  // an ASCII PCD containing the same persistent occupied cell centers.
  bool save(const std::string & output_prefix, const std::string & frame_id) const;

private:
  OdomLayerConfig config_;
  std::vector<OdomLayerCell> cells_;
};

}  // namespace ats_rog_map

#endif  // ATS_ROG_MAP__ODOM_LAYER_HPP_
