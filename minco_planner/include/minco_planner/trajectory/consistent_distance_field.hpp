// Copyright 2026
//
// 联合优化专用的距离场：值与梯度来自同一个函数。
//
// RcTraversabilityEsdfProvider::query() 的距离取原始有符号场的双线性插值，梯度却取 3x3
// 平滑后场的双线性导数（rc_traversability_esdf_provider.cpp 的 query）。对 L-BFGS 来说代价
// 和梯度因此属于两个不同的函数：线搜索失败点上沿 -g 的数值方向导数与解析 g·d 可以差一个
// 数量级甚至反号。这里把原始场拷出来自己插值，梯度是同一插值函数的精确导数。
//   kBilinear: 分片双线性，C0；梯度是该插值的精确导数，格线上不连续。
//   kBicubic : Catmull-Rom 双三次，C1；值经过全部格心采样，梯度处处连续。
// 栅格几何必须与 provider 构造时的栅格一致（调用方传入同一张 planning_grid），不一致时
// build() 返回 false，调用方回退到 provider 自带查询。

#ifndef MINCO_PLANNER__TRAJECTORY__CONSISTENT_DISTANCE_FIELD_HPP_
#define MINCO_PLANNER__TRAJECTORY__CONSISTENT_DISTANCE_FIELD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

enum class JointDistanceFieldMode
{
  kLegacy,
  kBilinear,
  kBicubic,
};

inline bool parseJointDistanceFieldMode(const std::string & name, JointDistanceFieldMode & mode)
{
  if (name == "legacy") {
    mode = JointDistanceFieldMode::kLegacy;
  } else if (name == "bilinear") {
    mode = JointDistanceFieldMode::kBilinear;
  } else if (name == "bicubic") {
    mode = JointDistanceFieldMode::kBicubic;
  } else {
    return false;
  }
  return true;
}

class ConsistentDistanceField
{
public:
  /**
   * @brief 从 provider 拷出原始有符号场并校验与栅格几何一致。
   * @return false = 几何不符或场不可用（调用方回退到 provider 查询）。
   */
  bool build(
    const ats_rc_esdf::RcTraversabilityEsdfProvider & esdf,
    const nav_msgs::msg::OccupancyGrid & grid, JointDistanceFieldMode mode)
  {
    field_.clear();
    if (mode == JointDistanceFieldMode::kLegacy) {
      return false;
    }
    width_ = static_cast<int>(grid.info.width);
    height_ = static_cast<int>(grid.info.height);
    resolution_ = grid.info.resolution;
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;
    if (width_ < 2 || height_ < 2 || !(resolution_ > 0.0) || !esdf.available()) {
      return false;
    }
    const ats_rc_esdf::RollingWindowBounds bounds = esdf.getRollingWindowBounds();
    const double tolerance = 1e-9 + 1e-6 * resolution_;
    if (!bounds.valid || std::abs(bounds.min_x - origin_x_) > tolerance ||
      std::abs(bounds.min_y - origin_y_) > tolerance ||
      std::abs(bounds.max_x - (origin_x_ + width_ * resolution_)) > tolerance ||
      std::abs(bounds.max_y - (origin_y_ + height_ * resolution_)) > tolerance)
    {
      return false;
    }
    if (!esdf.copySignedDistanceField(field_) ||
      field_.size() != static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_))
    {
      field_.clear();
      return false;
    }
    mode_ = mode;
    return true;
  }

  bool valid() const {return !field_.empty();}

  /**
   * @brief 距离与梯度（世界系）；在栅格可查询范围外或邻域含非有限值时返回 false。
   *        可查询范围与 provider 一致：格心坐标 [0, w-1] x [0, h-1]。
   */
  bool query(const Eigen::Vector2d & position, double & distance, Eigen::Vector2d & gradient) const
  {
    if (field_.empty()) {
      return false;
    }
    const double gx = (position.x() - origin_x_) / resolution_ - 0.5;
    const double gy = (position.y() - origin_y_) / resolution_ - 0.5;
    if (!(gx >= 0.0) || !(gy >= 0.0) || gx > width_ - 1 || gy > height_ - 1) {
      return false;
    }
    const int x0 = std::min(static_cast<int>(std::floor(gx)), width_ - 1);
    const int y0 = std::min(static_cast<int>(std::floor(gy)), height_ - 1);
    const double tx = gx - x0;
    const double ty = gy - y0;
    double value = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    if (mode_ == JointDistanceFieldMode::kBilinear) {
      const double d00 = at(x0, y0);
      const double d10 = at(x0 + 1, y0);
      const double d01 = at(x0, y0 + 1);
      const double d11 = at(x0 + 1, y0 + 1);
      value = (1.0 - tx) * (1.0 - ty) * d00 + tx * (1.0 - ty) * d10 +
        (1.0 - tx) * ty * d01 + tx * ty * d11;
      dx = (1.0 - ty) * (d10 - d00) + ty * (d11 - d01);
      dy = (1.0 - tx) * (d01 - d00) + tx * (d11 - d10);
    } else {
      double row_value[4];
      double row_slope[4];
      for (int j = 0; j < 4; ++j) {
        catmullRom(
          at(x0 - 1, y0 + j - 1), at(x0, y0 + j - 1), at(x0 + 1, y0 + j - 1),
          at(x0 + 2, y0 + j - 1), tx, row_value[j], row_slope[j]);
      }
      double unused = 0.0;
      catmullRom(row_value[0], row_value[1], row_value[2], row_value[3], ty, value, dy);
      catmullRom(row_slope[0], row_slope[1], row_slope[2], row_slope[3], ty, dx, unused);
    }
    if (!std::isfinite(value) || !std::isfinite(dx) || !std::isfinite(dy)) {
      return false;
    }
    distance = value;
    gradient = Eigen::Vector2d(dx, dy) / resolution_;
    return true;
  }

private:
  double at(int x, int y) const
  {
    x = std::max(0, std::min(x, width_ - 1));
    y = std::max(0, std::min(y, height_ - 1));
    return field_[static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(x)];
  }

  // 一维 Catmull-Rom：t in [0,1] 在 p1 与 p2 之间，返回值及对 t 的导数。
  static void catmullRom(
    double p0, double p1, double p2, double p3, double t, double & value, double & slope)
  {
    const double a = -p0 + 3.0 * p1 - 3.0 * p2 + p3;
    const double b = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
    const double c = -p0 + p2;
    value = 0.5 * (2.0 * p1 + t * (c + t * (b + t * a)));
    slope = 0.5 * (c + t * (2.0 * b + 3.0 * t * a));
  }

  std::vector<double> field_;
  JointDistanceFieldMode mode_ = JointDistanceFieldMode::kLegacy;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__CONSISTENT_DISTANCE_FIELD_HPP_
