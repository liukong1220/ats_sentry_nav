// Copyright 2026

#ifndef MINCO_PLANNER__GRID_ASTAR_HPP_
#define MINCO_PLANNER__GRID_ASTAR_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "minco_planner/planning/grid_clearance.hpp"

namespace minco_planner
{

struct GridAstarParams
{
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  bool allow_diagonal = true;
  double diagonal_cost = 1.41421356237;
  /// Omnidirectional clearance every expanded cell must have, in metres.
  ///
  /// This lives in the base params on purpose: when it only existed on
  /// GridJpsParams, the JPS -> A* fallback planned with zero clearance and
  /// produced wall-hugging seeds that the footprint gate had to reject, which
  /// the goal manager then retried as a transient failure until goal timeout.
  double safe_distance = 0.0;
  /// Admit the start cell even when it fails the clearance ring.
  ///
  /// The robot already occupies that pose; refusing to plan cannot move it, so a
  /// fail-closed rejection here is a livelock rather than a safety measure. The
  /// yaw-aware footprint gate and local collision repair stay authoritative.
  bool assume_start_traversable = true;
  /// Fall back to the clearance a start/goal cell actually has, capped by
  /// `safe_distance`, instead of rejecting the endpoint outright.
  bool relax_endpoint_clearance = true;
  /// Hard lower bound the endpoint relaxation may not go below, in metres.
  ///
  /// Without it the relaxation would re-create the zero-clearance fallback it
  /// exists to remove: a goal sitting in a 0.05 m nook would drag the whole
  /// search down to 0.05 m. Set this to the inscribed footprint half-width, the
  /// clearance below which no body yaw fits at all.
  double min_safe_distance = 0.0;
  /// 离墙代价权重；<= 0 关闭（纯最短路）。开启后每步代价乘以
  /// 1 + weight * ((clearance_cost_distance - d) / clearance_cost_distance)^2（d 小于
  /// clearance_cost_distance 时），d 为该格到最近阻塞格的格心距离。最短路会沿内角贴着
  /// 安全净空下限走，这一项让路径在通道里居中、优先走更宽的路线。代价不小于步长，
  /// 欧氏启发仍可采纳。
  double clearance_cost_weight = 0.0;
  /// 离墙代价生效的距离上界（米）；超过该距离视为开阔，不再加代价。
  double clearance_cost_distance = 0.0;
  /// 起点格净空低于 `safe_distance` 时，允许从起点沿"净空不减"的空闲格向外爬的最大格数。
  ///
  /// `assume_start_traversable` 只豁免起点格本身，它的 8 个邻格仍要满足 `safe_distance`。
  /// 车被推到墙边（净空 0.1–0.2 m，低于搜索下限 0.376 m）时所有邻格都不达标，A* 展开
  /// 1 个节点就结束（`expanded=1`），逃逸前缀与局部修复都在 A* 之后运行，一次都不会被调用。
  /// 这里只放开"离开"这一个方向：下一格必须是空闲格，且到最近阻塞格的距离不小于当前格，
  /// 因此路径不会为了绕路而重新靠近障碍；累计超过本上限仍没爬到 `safe_distance` 就放弃。
  /// 足迹门禁与图搜索的净空判据（`safe_distance`、阈值、unknown 语义）都不变。
  /// 0 = 关闭，与旧行为逐位一致。
  int start_escape_max_steps = 0;
};

struct GridAstarResult
{
  bool success = false;
  std::string reason;
  nav_msgs::msg::Path path;
  double length = 0.0;
  int expanded_nodes = 0;
};

class GridAstar
{
public:
  explicit GridAstar(GridAstarParams params = GridAstarParams());

  void setParams(const GridAstarParams & params);
  GridAstarResult plan(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Plan with a one-off clearance override; negative keeps the configured value.
  GridAstarResult planWithClearance(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double clearance_override) const;

private:
  struct GridIndex
  {
    int x = 0;
    int y = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx,
    double wy,
    GridIndex & out) const;
  geometry_msgs::msg::PoseStamped gridToPose(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridIndex & index) const;
  bool isTraversable(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  bool isTraversable(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & blocked_squared,
    const GridIndex & index) const;
  double clearanceCostFactor(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<double> & blocked_squared,
    const GridIndex & index) const;
  GridOccupancyPolicy occupancyPolicy() const;
  std::size_t linearIndex(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  double heuristic(const GridIndex & from, const GridIndex & to) const;

  GridAstarParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GRID_ASTAR_HPP_
