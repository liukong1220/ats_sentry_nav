// Copyright 2026

#ifndef ATS_ROG_MAP_ADAPTER__GROUND_PROJECTION_FUSION_HPP_
#define ATS_ROG_MAP_ADAPTER__GROUND_PROJECTION_FUSION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ats_rog_map_interfaces/srv/get_rog_map_projection.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"

namespace ats_rog_map_adapter
{

enum class RogMapEsdfQueryStatus
{
  kKnown,
  kUnknown,
  kOutside
};

struct RogMapEsdfQuery
{
  RogMapEsdfQueryStatus status{RogMapEsdfQueryStatus::kOutside};
  float signed_distance{0.0F};
  float gradient_x{0.0F};
  float gradient_y{0.0F};
};

struct RogMapEsdfSnapshot
{
  std_msgs::msg::Header header;
  nav_msgs::msg::MapMetaData info;
  std::uint64_t generation{0};
  bool ready{false};
  bool stale{true};
  std::vector<int8_t> occupancy;
  std::vector<float> signed_distance;
  std::vector<float> gradient_x;
  std::vector<float> gradient_y;

  bool valid() const;
  bool available() const;
  bool unknown(std::size_t index) const;
  RogMapEsdfQuery query(double world_x, double world_y) const;

  static RogMapEsdfSnapshot fromResponse(
    const ats_rog_map_interfaces::srv::GetRogMapProjection::Response & response);
};

/// @brief Structured audit of whether a ROGMap numeric projection is genuinely
///        all-unknown.  Every counter is filled even when the verdict is false
///        so the refusal reason can name the disqualifying evidence.
struct SourceUnknownEvidence
{
  bool structurally_valid{false};
  bool all_unknown{false};
  std::size_t cell_count{0};
  std::size_t unknown_cells{0};
  std::size_t free_cells{0};
  std::size_t occupied_cells{0};
  std::size_t out_of_range_cells{0};
  std::size_t finite_numeric_cells{0};
  std::uint64_t generation{0};
  std::string reason;
};

/// @brief Strict all-unknown predicate for the isolated source-unknown fixture.
/// @details `all_unknown` is true only when every one of these holds:
///          - `ready == true` and `stale == false`;
///          - non-empty `frame_id`, positive header stamp, finite resolution > 0,
///            `width > 0`, `height > 0`;
///          - `occupancy.size() == width * height`, and `signed_distance`,
///            `gradient_x`, `gradient_y` all have the same length;
///          - every occupancy cell is strictly `== -1` (a single free, occupied
///            or out-of-range cell disqualifies the fixture);
///          - every signed-distance and gradient sample is NaN;
///          - `generation > baseline_generation`.
///          Mixed unknown/free and mixed unknown/occupied responses are refused
///          with the offending counters recorded in `reason`.
SourceUnknownEvidence evaluateSourceUnknownEvidence(
  const RogMapEsdfSnapshot & snapshot, std::uint64_t baseline_generation);

struct GroundProjectionFusionParams
{
  int static_obstacle_value_threshold{50};
  int terrain_obstacle_value_threshold{50};
  double slope_grid_max_degrees{45.0};
  double slope_obstacle_degrees{28.0};
  double planning_resolution{0.10};
  bool unknown_is_obstacle{true};
};

struct GroundProjectionFusionResult
{
  nav_msgs::msg::OccupancyGrid planning_grid;
  std::size_t known_free_cells{0};
  std::size_t occupied_cells{0};
  std::size_t unknown_cells{0};
};

class GroundProjectionFusion
{
public:
  static bool fuse(
    const nav_msgs::msg::OccupancyGrid & rog_projection,
    const nav_msgs::msg::OccupancyGrid & traversability_grid,
    const nav_msgs::msg::OccupancyGrid & slope_grid,
    const nav_msgs::msg::OccupancyGrid & static_map,
    const geometry_msgs::msg::TransformStamped & static_from_projection,
    const GroundProjectionFusionParams & params,
    GroundProjectionFusionResult & result);
  static std::size_t clearUnknownCircle(
    GroundProjectionFusionResult & result, double world_x, double world_y, double radius);
};

}  // namespace ats_rog_map_adapter

#endif  // ATS_ROG_MAP_ADAPTER__GROUND_PROJECTION_FUSION_HPP_
