// Copyright 2026

#ifndef SMALL_GICP_RELOCALIZATION__GROWING_MAP_HPP_
#define SMALL_GICP_RELOCALIZATION__GROWING_MAP_HPP_

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <vector>

namespace small_gicp_relocalization
{
// Voxel evidence only: no keyframes, graph or loop closure. The first window
// supplies a target; subsequent windows enter it only after fusion commits.
class GrowingMapStore
{
public:
  using Points = std::vector<std::array<double, 3>>;
  GrowingMapStore(double resolution, std::size_t capacity,
    std::array<double, 3> bounds_size = {40.0, 30.0, 20.0},
    std::array<double, 3> bounds_origin = {-20.0, -15.0, -10.0})
  : resolution_(resolution), capacity_(capacity), bounds_size_(bounds_size),
    bounds_origin_(bounds_origin)
  {
    if (!std::isfinite(resolution) || resolution <= 0 || capacity == 0) {
      throw std::invalid_argument("invalid growing map geometry");
    }
    for (const double size : bounds_size_) {
      if (!std::isfinite(size) || size <= 0) {
        throw std::invalid_argument("invalid growing map bounds");
      }
    }
    for (const double origin : bounds_origin_) {
      if (!std::isfinite(origin)) {throw std::invalid_argument("invalid growing map origin");}
    }
  }
  bool seed(const Points & points, const Eigen::Isometry3d & transform)
  {
    if (!voxels_.empty() || !transform.matrix().allFinite()) {return false;}
    bounds_from_map_ = transform.inverse();
    return appendAccepted(points, transform, true);
  }
  bool appendAccepted(const Points & points, const Eigen::Isometry3d & transform, bool accepted)
  {
    if (!accepted || !transform.matrix().allFinite()) {return false;}
    const auto before = voxels_.size();
    for (const auto & p : points) {
      const Eigen::Vector3d v = transform * Eigen::Vector3d(p[0], p[1], p[2]);
      const Eigen::Vector3d local = bounds_from_map_ * v;
      bool inside = v.allFinite();
      for (int axis = 0; axis < 3; ++axis) {
        inside = inside && local[axis] >= bounds_origin_[axis] &&
          local[axis] < bounds_origin_[axis] + bounds_size_[axis];
      }
      if (!inside) {++dropped_; continue;}
      const std::array<int, 3> key{static_cast<int>(std::floor(v.x() / resolution_)),
        static_cast<int>(std::floor(v.y() / resolution_)),
        static_cast<int>(std::floor(v.z() / resolution_))};
      if (voxels_.count(key)) {continue;}
      if (voxels_.size() >= capacity_) {++dropped_; continue;}
      voxels_.emplace(key, std::array<double, 3>{v.x(), v.y(), v.z()});
    }
    return voxels_.size() > before;
  }
  Points points() const
  {
    Points result; result.reserve(voxels_.size());
    for (const auto & pair : voxels_) {result.push_back(pair.second);}
    return result;
  }
  void clear() {voxels_.clear();}
  std::size_t dropped() const {return dropped_;}

private:
  double resolution_;
  std::size_t capacity_;
  std::array<double, 3> bounds_size_;
  std::array<double, 3> bounds_origin_;
  Eigen::Isometry3d bounds_from_map_{Eigen::Isometry3d::Identity()};
  std::size_t dropped_{0};
  std::map<std::array<int, 3>, std::array<double, 3>> voxels_;
};
}  // namespace small_gicp_relocalization
#endif  // SMALL_GICP_RELOCALIZATION__GROWING_MAP_HPP_
