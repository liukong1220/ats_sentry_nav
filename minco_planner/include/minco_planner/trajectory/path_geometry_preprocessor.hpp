// Copyright 2026

#ifndef MINCO_PLANNER__TRAJECTORY__PATH_GEOMETRY_PREPROCESSOR_HPP_
#define MINCO_PLANNER__TRAJECTORY__PATH_GEOMETRY_PREPROCESSOR_HPP_

#include <vector>

#include <Eigen/Core>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace minco_planner
{

struct PathGeometryPreprocessorParams
{
  double duplicate_epsilon = 1e-6;
  double collinear_lateral_tolerance = 0.025;
  double short_segment_length = 0.05;
  double corner_angle_threshold_rad = 0.20;
  bool footprint_aware_shortcut_enabled = true;
  // Zero keeps historical sharp JPS corners. Deployed profiles set a positive
  // radius so MINCO interpolates an inner arc instead of the vertex.
  double fillet_radius = 0.0;
  int fillet_arc_samples = 1;
  // 捷径/圆角的中心净空保持（米）；<= 0 关闭。footprint 捷径会选最远的可行点，把居中的
  // 图搜索路径重新拉成贴着内角的折线（只剩 safety_margin 的余量）。开启后新线段沿途的
  // 格心净空不得低于 min(该值, 被替换折线沿途的最小净空) - 0.25 格。
  double shortcut_min_clearance = 0.0;
};

struct PathGeometryResult
{
  nav_msgs::msg::Path guide_path;
  std::vector<Eigen::Vector2d> waypoints;
  std::vector<bool> corner_waypoints;
  std::size_t duplicate_points_removed = 0;
  std::size_t collinear_points_removed = 0;
  std::size_t short_segments_merged = 0;
  std::size_t shortcut_waypoints_removed = 0;
};

// Converts a discrete JPS route into a compact guide.  A shortcut is accepted
// only after the same fail-closed rectangular footprint/swept check used by
// the final trajectory gate passes on the immutable planning grid.
class PathGeometryPreprocessor
{
public:
  explicit PathGeometryPreprocessor(
    PathGeometryPreprocessorParams params = PathGeometryPreprocessorParams());

  void setParams(const PathGeometryPreprocessorParams & params);
  PathGeometryResult preprocess(
    const nav_msgs::msg::Path & raw_path,
    const nav_msgs::msg::OccupancyGrid * planning_grid = nullptr,
    const FootprintSafetyChecker * safety_checker = nullptr) const;

private:
  bool shortcutSafe(
    const Eigen::Vector2d & start, const Eigen::Vector2d & end,
    const nav_msgs::msg::OccupancyGrid * planning_grid,
    const FootprintSafetyChecker * safety_checker) const;
  std::vector<Eigen::Vector2d> insertInnerCornerFillets(
    const std::vector<Eigen::Vector2d> & waypoints,
    const nav_msgs::msg::OccupancyGrid * planning_grid,
    const FootprintSafetyChecker * safety_checker,
    const std::vector<double> & blocked_squared) const;
  // 折线沿途（按 1/4 格采样）的最小格心净空；无栅格或无距离场时返回 +inf。
  double polylineClearance(
    const std::vector<Eigen::Vector2d> & points,
    const nav_msgs::msg::OccupancyGrid * planning_grid,
    const std::vector<double> & blocked_squared) const;
  // 用 replacement 替换 original 是否保持了中心净空（见 shortcut_min_clearance）。
  bool clearancePreserved(
    const std::vector<Eigen::Vector2d> & replacement,
    const std::vector<Eigen::Vector2d> & original,
    const nav_msgs::msg::OccupancyGrid * planning_grid,
    const std::vector<double> & blocked_squared) const;
  static nav_msgs::msg::Path makePath(
    const std_msgs::msg::Header & header, const std::vector<Eigen::Vector2d> & points);

  PathGeometryPreprocessorParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__PATH_GEOMETRY_PREPROCESSOR_HPP_
