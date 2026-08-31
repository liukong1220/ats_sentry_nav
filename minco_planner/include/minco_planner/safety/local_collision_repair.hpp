// Copyright 2026

#ifndef MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_
#define MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_

#include <cstddef>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

struct LocalCollisionRepairParams
{
  bool enabled = true;
  int max_iterations = 2;
  double search_radius = 0.35;
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  // 修复点必须落在"净空足够"的格子上,而不只是"该格自身空闲"。
  // 拒绝轨迹的是 yaw 相关矩形足迹门禁,只按点占据挑格子会挑回原地:
  // 观测证据 domain 193 目标 8,冲突点自身格子是空闲的,墙在西侧 4 格,
  // 于是 findNearestClearCell 选中原格、报告 changed 却毫无改善。
  // 判据按中心到中心距离,与 jps_safe_distance 同一套约定。
  // <=0 保留历史行为,仅按点占据修复。
  double required_clearance_m = 0.0;
  // 严格档(外接圆)在实机栅格上经常是空集:domain 169 目标 9 连续 89 次
  // 拒绝里一次都没挑出候选格,domain 171 目标 8 同样。原因是严格档要求
  // 候选格四周各向 0.419 m 全空,而搜索半径只有 0.35 m,连能排除该障碍
  // 的格子都够不到。这里给第二档:按内切半宽 + 栅格量化余量算出的
  // footprint 一致下限,与图搜索梯子用同一套几何。<=0 表示不启用第二档。
  // 放宽只影响"挑哪个格做引导点",最终仍由矩形足迹门禁在重解 MINCO 后
  // 判定,所以不会放过不安全轨迹,只是把必然失败换成有机会成功。
  double inscribed_radius_m = 0.0;
};

/// 修复过程的可观测结果。repair() 本身没有 logger,由调用方打印;
/// 没有这个结构时无法区分"没尝试""尝试了挑不出候选格""挑出来但位移可忽略"。
struct LocalCollisionRepairStats
{
  std::size_t collision_points = 0;
  std::size_t strict_repaired = 0;
  std::size_t fallback_repaired = 0;
  std::size_t no_candidate = 0;
  std::size_t negligible_shift = 0;
  // 被端点保护跳过的冲突点数。起点是机器人当前位姿,终点是 action 下发的
  // 目标位姿,两者都不是"引导点",挪动它们等于偷偷改写任务。
  std::size_t endpoint_protected = 0;
  double strict_clearance_m = 0.0;
  double fallback_clearance_m = 0.0;
  double effective_search_radius_m = 0.0;
};

class LocalCollisionRepair
{
public:
  explicit LocalCollisionRepair(LocalCollisionRepairParams params = LocalCollisionRepairParams());

  void setParams(const LocalCollisionRepairParams & params);
  bool repair(
    ReferenceTrajectory & trajectory,
    const FootprintSafetyResult & collisions,
    const nav_msgs::msg::OccupancyGrid & grid,
    LocalCollisionRepairStats * stats = nullptr) const;

private:
  bool findNearestClearCell(
    const nav_msgs::msg::OccupancyGrid & grid,
    double x,
    double y,
    double required_clearance_m,
    double search_radius_m,
    double & repaired_x,
    double & repaired_y) const;
  bool isFree(const nav_msgs::msg::OccupancyGrid & grid, int mx, int my) const;
  bool hasClearance(
    const nav_msgs::msg::OccupancyGrid & grid, int mx, int my,
    double required_clearance_m) const;

  LocalCollisionRepairParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_
