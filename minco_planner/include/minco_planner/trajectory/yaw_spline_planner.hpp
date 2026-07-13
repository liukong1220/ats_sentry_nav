// Copyright 2026

#ifndef MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_
#define MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_

#include <string>

#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

struct YawSplinePlannerParams
{
  std::string mode = "goal_heading";
  double yaw_rate_limit = 2.5;
};

class YawSplinePlanner
{
public:
  explicit YawSplinePlanner(YawSplinePlannerParams params = YawSplinePlannerParams());

  void setParams(const YawSplinePlannerParams & params);
  void apply(ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const;

private:
  void applyGoalHeading(
    ReferenceTrajectory & trajectory, double initial_yaw, double goal_yaw) const;
  void applyPathTangent(ReferenceTrajectory & trajectory, double initial_yaw) const;
  static double normalizeAngle(double angle);
  static double shortestAngularDistance(double from, double to);

  YawSplinePlannerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__YAW_SPLINE_PLANNER_HPP_
