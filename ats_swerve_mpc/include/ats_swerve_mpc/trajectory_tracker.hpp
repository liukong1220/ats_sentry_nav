// Copyright 2026

#ifndef ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_
#define ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_

#include <cstddef>
#include <vector>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

namespace ats_swerve_mpc
{

struct TimedState
{
  double time = 0.0;
  State state = State::Zero();
};

struct TrajectoryProjection
{
  bool valid = false;
  double time = 0.0;
  double cross_track_error = 0.0;
  std::size_t segment_index = 0;
};

struct TrajectoryTrackerConfig
{
  double backward_search_window = 0.20;
  double forward_search_window = 2.00;
  double cross_track_slowdown_start = 0.15;
  double cross_track_slowdown_end = 0.60;
  double min_progress_scale = 0.25;
  double command_latency_compensation = 0.01;
};

class TrajectoryTracker
{
public:
  /** @brief 创建路径投影器；配置决定有限前后搜索窗和偏差降速曲线。 */
  explicit TrajectoryTracker(const TrajectoryTrackerConfig & config = TrajectoryTrackerConfig());

  /** @brief 替换投影/降速参数，并清除旧路径的进度记忆。 */
  void setConfig(const TrajectoryTrackerConfig & config);
  /** @brief 原子接管经过时间校验的轨迹点，并从路径起点重新投影。 */
  void setTrajectory(std::vector<TimedState> trajectory);
  /** @brief 删除当前 reference，令上层下一控制周期走确定性零速度。 */
  void clear();
  /** @brief 保留路径内容但清空单调进度，供重定位或新授权后重新捕获路径。 */
  void resetProgress();

  /** @brief 返回当前是否没有可供控制的 reference。 */
  bool empty() const { return trajectory_.empty(); }
  /** @brief 返回路径终点；调用者应先确认轨迹非空。 */
  const State & goal() const { return trajectory_.back().state; }
  /** @brief 返回路径的相对持续时间（秒），用于 reference 新鲜度 deadline。 */
  double duration() const { return trajectory_.back().time - trajectory_.front().time; }
  /** @brief 返回最后一个轨迹点的相对时间。 */
  double finalTime() const { return trajectory_.back().time; }
  /** @brief 返回横向偏差降速允许的下限，供 deadline 保守放大。 */
  double minimumProgressScale() const { return config_.min_progress_scale; }
  /** @brief 仅按二维位置投影，供无可靠 yaw 的调用方使用。 */
  TrajectoryProjection project(const Eigen::Vector2d & position);
  /** @brief 按完整 SE(2) 状态投影，结合 yaw 抑制路径交叉处跳段。 */
  TrajectoryProjection project(const State & state);
  /** @brief 将横向误差转换为 [min_progress_scale,1] 的连续时间缩放。 */
  double progressScale(double cross_track_error) const;
  /** @brief 从投影时刻采样 horizon+1 个状态和车体系速度参考。 */
  std::vector<Se2Reference> buildHorizon(
    const TrajectoryProjection & projection, int horizon, double dt) const;

private:
  /** @brief 投影入口的共用实现；按参数决定是否将 yaw 计入距离度量。 */
  TrajectoryProjection projectImpl(
    const Eigen::Vector2d & position, double yaw, bool use_yaw);
  /** @brief 在受限时间窗内寻找最近线段，必要时禁止倒退以保证控制进度单调。 */
  TrajectoryProjection nearestProjection(
    const Eigen::Vector2d & position, double yaw, bool use_yaw, bool restrict_progress) const;
  /** @brief 在相邻时间点间插值位置/yaw，并由离散差分生成 body-frame 控制参考。 */
  bool sampleReference(double time, Se2Reference & reference) const;
  /** @brief 统一 trajectory yaw 插值的最短角归一化。 */
  static double normalizeAngle(double angle);
  /** @brief 沿最短旋转方向插值 yaw，避免穿越 +/-pi 时反向大转。 */
  static double interpolateAngle(double from, double to, double ratio);

  TrajectoryTrackerConfig config_;
  std::vector<TimedState> trajectory_;
  bool has_progress_ = false;
  double last_progress_time_ = 0.0;
};

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__TRAJECTORY_TRACKER_HPP_
