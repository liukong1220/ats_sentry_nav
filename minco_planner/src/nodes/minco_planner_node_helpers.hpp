#ifndef MINCO_PLANNER__NODES__MINCO_PLANNER_NODE_HELPERS_HPP_
#define MINCO_PLANNER__NODES__MINCO_PLANNER_NODE_HELPERS_HPP_

// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include "minco_planner/planning/graph_search_failure.hpp"
#include "minco_planner/planning/clearance_ladder.hpp"
#include "minco_planner/nodes/planning_digest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "minco_planner/trajectory/reference_path_timing.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace minco_planner
{

namespace node_detail
{


inline bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
    std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
    std::isfinite(pose.orientation.w);
}

inline std::string formatCollisionIndices(const std::vector<CollisionSample> & collisions)
{
  std::ostringstream out;
  const std::size_t cap = std::min<std::size_t>(16U, collisions.size());
  for (std::size_t i = 0; i < cap; ++i) {
    if (i != 0U) {
      out << ',';
    }
    out << collisions[i].trajectory_index;
  }
  if (collisions.size() > cap) {
    out << ",...";
  }
  return out.str();
}

// 把拒绝点周围的规划栅格原样打成一行,用于区分
// "车自身格子被占"与"外侧存在成片障碍"。只在 footprint
// 拒绝分支调用,窗口为 ±radius_m,行序从北到南。
inline std::string describeGridWindow(
  const nav_msgs::msg::OccupancyGrid & grid, double center_x, double center_y,
  double radius_m, int obstacle_value_threshold, bool unknown_is_obstacle)
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0U || grid.info.height == 0U) {
    return std::string("<invalid grid>");
  }
  const double resolution = grid.info.resolution;
  const int span = std::max(1, static_cast<int>(std::lround(radius_m / resolution)));
  const int center_mx = static_cast<int>(
    std::floor((center_x - grid.info.origin.position.x) / resolution));
  const int center_my = static_cast<int>(
    std::floor((center_y - grid.info.origin.position.y) / resolution));
  std::ostringstream stream;
  stream << "res=" << resolution << " center_cell=(" << center_mx << ", " << center_my
         << ") span=" << span << " rows_north_to_south=[";
  for (int dy = span; dy >= -span; --dy) {
    const int my = center_my + dy;
    for (int dx = -span; dx <= span; ++dx) {
      const int mx = center_mx + dx;
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) ||
        my >= static_cast<int>(grid.info.height))
      {
        // 越界在 footprint 判据里等价于占据,这里用独立
      // 符号标出来,避免与真实障碍混淆。
        stream << 'X';
        continue;
      }
      const int value = grid.data[
        static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
        static_cast<std::size_t>(mx)];
      if (value < 0) {
        stream << (unknown_is_obstacle ? 'U' : 'u');
      } else if (value >= obstacle_value_threshold) {
        stream << '#';
      } else if (value == 0) {
        stream << '.';
      } else {
        // 连续风险 1..threshold-1 保留量级,便于判断是否贴着 terrain 风险带走。
        stream << static_cast<char>('0' + std::min(9, value / 10));
      }
    }
    if (dy != -span) {
      stream << '/';
    }
  }
  stream << ']';
  return stream.str();
}

// 两条同几何参考的 yaw 序列是否不同（逐点 1e-9 rad）；相同时不必再过两遍足迹检查。
inline bool yawSequencesDiffer(
  const minco_planner::ReferenceTrajectory & a, const minco_planner::ReferenceTrajectory & b)
{
  if (a.points.size() != b.points.size()) {
    return true;
  }
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    if (std::abs(std::remainder(a.points[i].yaw - b.points[i].yaw, 2.0 * M_PI)) > 1e-9) {
      return true;
    }
  }
  return false;
}

struct SplitGuideResult
{
  bool valid{false};
  double total_length{0.0};
  double split_s{0.0};
  Eigen::Vector2d junction_pos{0.0, 0.0};
  Eigen::Vector2d junction_tangent{1.0, 0.0};
  double junction_turn_rad{0.0};
  nav_msgs::msg::Path prefix;
  nav_msgs::msg::Path suffix;
};

inline double pathArcLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return length;
}

inline SplitGuideResult splitPathAtArcLength(
  const nav_msgs::msg::Path & path, double target_split_s, double min_tail_s)
{
  SplitGuideResult out;
  if (path.poses.size() < 2U || target_split_s <= 0.2) {
    return out;
  }
  std::vector<double> cumulative(path.poses.size(), 0.0);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double seg = std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    cumulative[i] = cumulative[i - 1] + seg;
  }
  out.total_length = cumulative.back();
  if (out.total_length <= target_split_s + std::max(0.5, min_tail_s)) {
    return out;
  }
  const auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target_split_s);
  if (upper == cumulative.begin() || upper == cumulative.end()) {
    return out;
  }
  const std::size_t seg_idx = static_cast<std::size_t>(std::distance(cumulative.begin(), upper)) - 1U;
  const double seg_len = cumulative[seg_idx + 1U] - cumulative[seg_idx];
  const double u = seg_len > 1e-9 ?
    std::clamp((target_split_s - cumulative[seg_idx]) / seg_len, 0.0, 1.0) : 0.0;
  const auto & a = path.poses[seg_idx];
  const auto & b = path.poses[seg_idx + 1U];
  geometry_msgs::msg::PoseStamped junction = a;
  junction.pose.position.x = a.pose.position.x + u * (b.pose.position.x - a.pose.position.x);
  junction.pose.position.y = a.pose.position.y + u * (b.pose.position.y - a.pose.position.y);
  junction.pose.position.z = a.pose.position.z + u * (b.pose.position.z - a.pose.position.z);

  Eigen::Vector2d dir(
    b.pose.position.x - a.pose.position.x,
    b.pose.position.y - a.pose.position.y);
  if (dir.norm() <= 1e-6 && seg_idx + 2U < path.poses.size()) {
    dir = Eigen::Vector2d(
      path.poses[seg_idx + 2U].pose.position.x - a.pose.position.x,
      path.poses[seg_idx + 2U].pose.position.y - a.pose.position.y);
  }
  if (dir.norm() <= 1e-6) {
    return out;
  }
  out.junction_tangent = dir.normalized();
  out.junction_pos = Eigen::Vector2d(junction.pose.position.x, junction.pose.position.y);
  out.split_s = target_split_s;

  // Estimate local corner turn within +/- 0.8 m around split_s to cap junction speed on turns.
  const double before_s = std::max(0.0, target_split_s - 0.8);
  const double after_s = std::min(out.total_length, target_split_s + 0.8);
  const auto sampleAtS = [&](double s_query) {
    const auto it = std::lower_bound(cumulative.begin(), cumulative.end(), s_query);
    if (it == cumulative.begin()) {
      return Eigen::Vector2d(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
    }
    if (it == cumulative.end()) {
      return Eigen::Vector2d(path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(cumulative.begin(), it)) - 1U;
    const double d = cumulative[idx + 1U] - cumulative[idx];
    const double r = d > 1e-9 ? std::clamp((s_query - cumulative[idx]) / d, 0.0, 1.0) : 0.0;
    return Eigen::Vector2d(
      path.poses[idx].pose.position.x + r * (path.poses[idx + 1U].pose.position.x - path.poses[idx].pose.position.x),
      path.poses[idx].pose.position.y + r * (path.poses[idx + 1U].pose.position.y - path.poses[idx].pose.position.y));
  };
  const Eigen::Vector2d v_in = out.junction_pos - sampleAtS(before_s);
  const Eigen::Vector2d v_out = sampleAtS(after_s) - out.junction_pos;
  if (v_in.norm() > 1e-4 && v_out.norm() > 1e-4) {
    out.junction_turn_rad = std::abs(std::atan2(
      v_in.x() * v_out.y() - v_in.y() * v_out.x(), v_in.dot(v_out)));
  }

  out.prefix.header = path.header;
  for (std::size_t i = 0; i <= seg_idx; ++i) {
    out.prefix.poses.push_back(path.poses[i]);
  }
  if (std::hypot(
      out.prefix.poses.back().pose.position.x - junction.pose.position.x,
      out.prefix.poses.back().pose.position.y - junction.pose.position.y) > 1e-4)
  {
    out.prefix.poses.push_back(junction);
  }

  out.suffix.header = path.header;
  out.suffix.poses.push_back(junction);
  for (std::size_t i = seg_idx + 1U; i < path.poses.size(); ++i) {
    if (std::hypot(
        out.suffix.poses.back().pose.position.x - path.poses[i].pose.position.x,
        out.suffix.poses.back().pose.position.y - path.poses[i].pose.position.y) > 1e-4)
    {
      out.suffix.poses.push_back(path.poses[i]);
    }
  }
  out.valid = out.prefix.poses.size() >= 2U && out.suffix.poses.size() >= 2U;
  return out;
}

inline minco_planner::ReferenceTrajectory spliceReferenceTrajectories(
  const minco_planner::ReferenceTrajectory & prefix,
  const minco_planner::ReferenceTrajectory & tail,
  double tail_base_time_offset)
{
  minco_planner::ReferenceTrajectory out = prefix;
  if (!prefix.valid() || tail.points.empty()) {
    return minco_planner::ReferenceTrajectory();
  }
  double prev_t = out.points.back().t;
  double prev_s = out.points.back().s;
  double prev_x = out.points.back().x;
  double prev_y = out.points.back().y;
  for (const auto & pt : tail.points) {
    const double shifted_t = pt.t + tail_base_time_offset;
    if (shifted_t <= prev_t + 1e-4) {
      continue;
    }
    minco_planner::ReferencePoint copy = pt;
    copy.t = shifted_t;
    prev_s += std::hypot(copy.x - prev_x, copy.y - prev_y);
    copy.s = prev_s;
    out.points.push_back(copy);
    prev_t = copy.t;
    prev_x = copy.x;
    prev_y = copy.y;
  }
  return out.valid() ? out : minco_planner::ReferenceTrajectory();
}

inline double empiricalPercentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double clamped = std::clamp(q, 0.0, 1.0);
  const std::size_t idx = static_cast<std::size_t>(
    std::ceil(clamped * static_cast<double>(values.size())) - 1.0);
  return values[std::min(idx, values.size() - 1U)];
}



inline bool sampleOccupancyGridCell(
  const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, std::int8_t & out_value,
  int * out_ix = nullptr, int * out_iy = nullptr)
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0U || grid.info.height == 0U ||
    grid.data.size() != static_cast<std::size_t>(grid.info.width) * grid.info.height)
  {
    return false;
  }
  const double yaw = tf2::getYaw(grid.info.origin.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double dx = wx - grid.info.origin.position.x;
  const double dy = wy - grid.info.origin.position.y;
  const double lx = (c * dx + s * dy) / grid.info.resolution;
  const double ly = (-s * dx + c * dy) / grid.info.resolution;
  if (!(lx >= 0.0 && ly >= 0.0 &&
    lx < static_cast<double>(grid.info.width) &&
    ly < static_cast<double>(grid.info.height)))
  {
    return false;
  }
  const int ix = static_cast<int>(std::floor(lx));
  const int iy = static_cast<int>(std::floor(ly));
  if (out_ix) {*out_ix = ix;}
  if (out_iy) {*out_iy = iy;}
  out_value = grid.data[static_cast<std::size_t>(iy) * grid.info.width +
    static_cast<std::size_t>(ix)];
  return true;
}

inline bool isOptimisticPathBlocked(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & global_grid,
  const std::vector<double> & global_blocked_sq,
  const nav_msgs::msg::OccupancyGrid & local_grid,
  int occupied_threshold, double min_clearance_m)
{
  if (path.poses.size() < 2U) {
    return true;
  }
  const double g_res = global_grid.info.resolution;
  const double min_cells_sq = g_res > 0.0 ?
    std::pow(std::max(0.0, min_clearance_m) / g_res, 2.0) : 0.0;
  for (std::size_t i = 1U; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    int gx = 0;
    int gy = 0;
    std::int8_t gval = -1;
    if (sampleOccupancyGridCell(global_grid, p.x, p.y, gval, &gx, &gy)) {
      if (gval >= occupied_threshold) {
        return true;
      }
      const std::size_t idx =
        static_cast<std::size_t>(gy) * global_grid.info.width + static_cast<std::size_t>(gx);
      if (idx < global_blocked_sq.size() && min_cells_sq > 0.0 &&
        global_blocked_sq[idx] <= min_cells_sq)
      {
        return true;
      }
    }
    std::int8_t lval = -1;
    if (sampleOccupancyGridCell(local_grid, p.x, p.y, lval) && lval >= occupied_threshold) {
      return true;
    }
  }
  return false;
}


}  // namespace node_detail


struct MincoPlannerNode::PlanGoalCommitContext
{
  std::uint64_t goal_id{0};
  std::uint64_t localization_epoch{0};
  std::uint64_t plan_request_sequence{0};
  std::uint64_t map_publication_sequence{0};
  bool report_status{false};
  bool keep_active_reference{false};
  std::chrono::steady_clock::time_point plan_wall_begin;
  rclcpp::Time plan_begin_time;
  const std::optional<ActiveSafetyReference> & committed_for_start;
  const std::shared_ptr<const PlanningMapSnapshot> & map_snapshot;
  std::uint64_t map_health_epoch{0};
  const nav_msgs::msg::OccupancyGrid & planning_grid;
  const std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> & clearance_esdf;
  const geometry_msgs::msg::PoseStamped & start;
  const geometry_msgs::msg::PoseStamped & goal;
  double active_goal_yaw{0.0};
  bool active_is_final_goal{true};
  const ReplanStart & replan_start;
  bool replan_frame_matches{false};
  double replan_elapsed_sec{0.0};
  double measured_start_x{0.0};
  double measured_start_y{0.0};
  const GridSearchResult & search_result;
  double search_wall_ms{0.0};
  double minco_wall_ms{0.0};
  const std::optional<double> & exploration_lidar_fov;
  ReferenceTrajectory & reference;
  FootprintSafetyResult & safety;
  const MincoOptimizationTrace & selected_trace;
  bool has_spliced_tail{false};
  bool esdf_fallback_used{false};
  bool local_repair_used{false};
  const std::function<double()> & elapsed_wall_ms;
  const std::function<void(std::uint8_t, std::uint64_t)> & fail;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__NODES__MINCO_PLANNER_NODE_HELPERS_HPP_
