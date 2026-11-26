#ifndef ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HELPERS_HPP_
#define ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HELPERS_HPP_

// Copyright 2026

#include "ats_swerve_mpc/ats_swerve_mpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_swerve_mpc {


/** @brief 读取三元素参数并在维度错误时保留物理安全默认值。 */
inline Eigen::Vector3d vectorParameter(rclcpp::Node &node, const std::string &name,
                                const Eigen::Vector3d &defaults) {
  const std::vector<double> values =
      node.declare_parameter<std::vector<double>>(
          name, {defaults(0), defaults(1), defaults(2)});
  if (values.size() != 3) {
    RCLCPP_WARN(node.get_logger(),
                "【参数错误】参数 '%s' 必须包含 3 个值。", name.c_str());
    return defaults;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

/** @brief 把 steady-clock 相邻时刻转换为毫秒，统一 timer 内所有阶段的时间单位。 */
inline double elapsedMilliseconds(std::chrono::steady_clock::time_point start,
                           std::chrono::steady_clock::time_point end) {
  return 1000.0 * std::chrono::duration<double>(end - start).count();
}

/** @brief 填写一个有限阶段计时，禁止让无效 duration 混入后续 p50/p95/p99。 */
inline void recordTiming(ControlCycleTelemetrySample &telemetry,
                  ControlCycleTimingStage stage, double milliseconds) {
  const std::size_t index = controlCycleTimingStageIndex(stage);
  telemetry.stage_ms[index] = milliseconds;
  telemetry.stage_recorded[index] = std::isfinite(milliseconds) && milliseconds >= 0.0;
}

/** @brief 读取当前 DDS domain；缺失或格式错误时返回 -1 而非伪造 domain。 */
inline int currentRosDomainId() {
  const char *value = std::getenv("ROS_DOMAIN_ID");
  if (value == nullptr || *value == '\0') {
    return -1;
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 0 || parsed > 232) {
    return -1;
  }
  return static_cast<int>(parsed);
}

/**
 * @brief 从预分配稀疏 CSC LTV-QP 中以 O(nnz) 提取有限尺度诊断指标。
 */
inline void recordQpProblemMetrics(const LtvQpProblem &problem,
                            ControlCycleTelemetrySample &telemetry) {
  const auto dims = checkedLtvQpDimensions(problem.horizon);
  if (!problem.valid || !dims.valid || !problem.hasExpectedLayout() ||
      !problem.hasFiniteNumerics()) {
    return;
  }
  telemetry.hessian_diagonal_minimum = std::numeric_limits<double>::infinity();
  telemetry.hessian_diagonal_maximum =
      -std::numeric_limits<double>::infinity();
  for (int col = 0; col < problem.decisionSize(); ++col) {
    const int end = problem.hessian_structure
                        .column_offsets[static_cast<std::size_t>(col + 1)];
    if (end > 0) {
      const double diag_val =
          problem.hessian_values[static_cast<std::size_t>(end - 1)];
      telemetry.hessian_diagonal_minimum =
          std::min(telemetry.hessian_diagonal_minimum, diag_val);
      telemetry.hessian_diagonal_maximum =
          std::max(telemetry.hessian_diagonal_maximum, diag_val);
    }
  }
  const int m = problem.constraintRows();
  std::vector<double> row_norm_sq(static_cast<std::size_t>(m), 0.0);
  for (std::size_t idx = 0; idx < problem.constraint_values.size(); ++idx) {
    const int row = problem.constraint_structure.row_indices[idx];
    const double val = problem.constraint_values[idx];
    row_norm_sq[static_cast<std::size_t>(row)] += val * val;
  }
  telemetry.constraint_row_l2_minimum = std::numeric_limits<double>::infinity();
  telemetry.constraint_row_l2_maximum = 0.0;
  const int check_rows = dims.equality_rows + dims.inequality_rows;
  for (int row = 0; row < check_rows; ++row) {
    const double norm = std::sqrt(row_norm_sq[static_cast<std::size_t>(row)]);
    telemetry.constraint_row_l2_minimum =
        std::min(telemetry.constraint_row_l2_minimum, norm);
    telemetry.constraint_row_l2_maximum =
        std::max(telemetry.constraint_row_l2_maximum, norm);
  }
  telemetry.nonzero_bound_abs_minimum = std::numeric_limits<double>::infinity();
  telemetry.bound_abs_maximum = 0.0;
  const auto update_bound_range = [&telemetry](const Eigen::VectorXd &bounds) {
    for (int index = 0; index < bounds.size(); ++index) {
      const double magnitude = std::abs(bounds(index));
      if (!std::isfinite(magnitude)) {
        continue;
      }
      telemetry.bound_abs_maximum =
          std::max(telemetry.bound_abs_maximum, magnitude);
      if (magnitude > 1e-12) {
        telemetry.nonzero_bound_abs_minimum =
            std::min(telemetry.nonzero_bound_abs_minimum, magnitude);
      }
    }
  };
  update_bound_range(problem.lower);
  update_bound_range(problem.upper);
  if (!std::isfinite(telemetry.nonzero_bound_abs_minimum)) {
    telemetry.nonzero_bound_abs_minimum = 0.0;
  }
  telemetry.zero_delta_dynamic_equality_residual =
      dims.equality_rows > 3
          ? problem.lower.segment(3, dims.equality_rows - 3)
                .cwiseAbs()
                .maxCoeff()
          : 0.0;
  telemetry.qp_problem_metrics_recorded =
      std::isfinite(telemetry.hessian_diagonal_minimum) &&
      std::isfinite(telemetry.hessian_diagonal_maximum) &&
      std::isfinite(telemetry.constraint_row_l2_minimum) &&
      std::isfinite(telemetry.constraint_row_l2_maximum) &&
      std::isfinite(telemetry.nonzero_bound_abs_minimum) &&
      std::isfinite(telemetry.bound_abs_maximum) &&
      std::isfinite(telemetry.zero_delta_dynamic_equality_residual);
}

inline bool validPlanningSnapshotPayload(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot) {
  const auto count =
      static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height;
  if (!snapshot.ready || snapshot.header.frame_id.empty() ||
      snapshot.source_generation == 0U ||
      snapshot.publication_sequence == 0U ||
      snapshot.occupied_value_threshold <= 0 ||
      snapshot.occupied_value_threshold > 100 ||
      !std::isfinite(snapshot.info.resolution) ||
      snapshot.info.resolution <= 0.0F || count == 0U ||
      snapshot.occupancy.size() != count ||
      snapshot.signed_distance_m.size() != count ||
      snapshot.gradient_x.size() != count ||
      snapshot.gradient_y.size() != count) {
    return false;
  }
  const auto &q = snapshot.info.origin.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto occ = snapshot.occupancy[i];
    const auto dist = snapshot.signed_distance_m[i];
    const auto gx = snapshot.gradient_x[i];
    const auto gy = snapshot.gradient_y[i];
    if (occ < -1 || occ > 100 || !std::isfinite(dist) || !std::isfinite(gx) ||
        !std::isfinite(gy)) {
      return false;
    }
    const bool blocked =
        occ >= snapshot.occupied_value_threshold ||
        (occ == -1 && snapshot.unknown_is_obstacle);
    if (blocked ? dist > 0.0F : dist < 0.0F) {
      return false;
    }
  }
  return true;
}

inline bool sampleSnapshotEsdfBilinear(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    double snap_x, double snap_y, double &out_distance_m,
    Eigen::Vector2d *out_gradient_snap) {
  const double res = static_cast<double>(snapshot.info.resolution);
  const auto &origin = snapshot.info.origin;
  const double oyaw = tf2::getYaw(origin.orientation);
  const double co = std::cos(oyaw);
  const double so = std::sin(oyaw);
  const double dx = snap_x - origin.position.x;
  const double dy = snap_y - origin.position.y;
  const double gx = (co * dx + so * dy) / res - 0.5;
  const double gy = (-so * dx + co * dy) / res - 0.5;
  const int w = static_cast<int>(snapshot.info.width);
  const int h = static_cast<int>(snapshot.info.height);
  if (gx < -0.5 || gy < -0.5 || gx > static_cast<double>(w) - 0.5 ||
      gy > static_cast<double>(h) - 0.5) {
    out_distance_m = -res;
    if (out_gradient_snap != nullptr) {
      out_gradient_snap->setZero();
    }
    return false;
  }
  const double clamped_x = std::clamp(gx, 0.0, static_cast<double>(w - 1));
  const double clamped_y = std::clamp(gy, 0.0, static_cast<double>(h - 1));
  const int x0 = std::min(w - 1, static_cast<int>(std::floor(clamped_x)));
  const int y0 = std::min(h - 1, static_cast<int>(std::floor(clamped_y)));
  const int x1 = std::min(w - 1, x0 + 1);
  const int y1 = std::min(h - 1, y0 + 1);
  const double tx = clamped_x - static_cast<double>(x0);
  const double ty = clamped_y - static_cast<double>(y0);

  const auto idx = [w](int cx, int cy) {
    return static_cast<std::size_t>(cy) * static_cast<std::size_t>(w) +
           static_cast<std::size_t>(cx);
  };
  const std::size_t i00 = idx(x0, y0);
  const std::size_t i10 = idx(x1, y0);
  const std::size_t i01 = idx(x0, y1);
  const std::size_t i11 = idx(x1, y1);
  const double d00 = snapshot.signed_distance_m[i00];
  const double d10 = snapshot.signed_distance_m[i10];
  const double d01 = snapshot.signed_distance_m[i01];
  const double d11 = snapshot.signed_distance_m[i11];
  out_distance_m = (1.0 - ty) * ((1.0 - tx) * d00 + tx * d10) +
                   ty * ((1.0 - tx) * d01 + tx * d11);
  if (out_gradient_snap != nullptr) {
    const double gx_val =
        (1.0 - ty) * ((1.0 - tx) * snapshot.gradient_x[i00] +
                      tx * snapshot.gradient_x[i10]) +
        ty * ((1.0 - tx) * snapshot.gradient_x[i01] +
              tx * snapshot.gradient_x[i11]);
    const double gy_val =
        (1.0 - ty) * ((1.0 - tx) * snapshot.gradient_y[i00] +
                      tx * snapshot.gradient_y[i10]) +
        ty * ((1.0 - tx) * snapshot.gradient_y[i01] +
              tx * snapshot.gradient_y[i11]);
    *out_gradient_snap = Eigen::Vector2d(gx_val, gy_val);
  }
  return true;
}

inline bool snapshotCenterCellFree(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    double snap_x, double snap_y) {
  const double res = static_cast<double>(snapshot.info.resolution);
  const auto &origin = snapshot.info.origin;
  const double oyaw = tf2::getYaw(origin.orientation);
  const double co = std::cos(oyaw);
  const double so = std::sin(oyaw);
  const double dx = snap_x - origin.position.x;
  const double dy = snap_y - origin.position.y;
  const double cx = (co * dx + so * dy) / res;
  const double cy = (-so * dx + co * dy) / res;
  if (cx < 0.0 || cy < 0.0 || cx >= snapshot.info.width ||
      cy >= snapshot.info.height) {
    return false;
  }
  const int ix = static_cast<int>(std::floor(cx));
  const int iy = static_cast<int>(std::floor(cy));
  const std::size_t index =
      static_cast<std::size_t>(iy) * snapshot.info.width +
      static_cast<std::size_t>(ix);
  const int occ = snapshot.occupancy[index];
  if (occ >= snapshot.occupied_value_threshold) {
    return false;
  }
  if (occ < 0 && snapshot.unknown_is_obstacle) {
    return false;
  }
  return true;
}

constexpr std::uint64_t kTelemetrySummaryInterval = 16;


} // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__ATS_SWERVE_MPC_NODE_HELPERS_HPP_
