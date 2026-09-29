// Copyright 2026

#include "minco_planner/trajectory/minco_joint_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "minco_planner/trajectory/minco_s3.hpp"

namespace minco_planner
{

namespace
{

using Point = Eigen::Vector2d;

double cube(double value)
{
  return value * value * value;
}

// 从 x = [vec(q); τ] 取出内点矩阵（2 x (N-1)）与时长向量。
void unpack(
  const Eigen::VectorXd & x, int piece_count, Eigen::MatrixXd & inner, Eigen::VectorXd & tau)
{
  const int inner_count = piece_count - 1;
  inner = Eigen::Map<const Eigen::MatrixXd>(x.data(), 2, inner_count);
  tau = x.segment(2 * inner_count, piece_count);
}

// 点到折线段 [first, last] 的最近点；返回距离。
double nearestOnGuide(
  const std::vector<Point> & guide, int first, int last, const Point & position, Point & nearest)
{
  double best = std::numeric_limits<double>::infinity();
  for (int index = first; index <= last; ++index) {
    const Point & start = guide[static_cast<std::size_t>(index)];
    const Point delta = guide[static_cast<std::size_t>(index + 1)] - start;
    const double length_squared = delta.squaredNorm();
    const double blend = length_squared > 1e-12 ?
      std::clamp((position - start).dot(delta) / length_squared, 0.0, 1.0) : 0.0;
    const Point candidate = start + blend * delta;
    const double distance = (position - candidate).norm();
    if (distance < best) {
      best = distance;
      nearest = candidate;
    }
  }
  return best;
}

// 点在折线上投影的弧长。
double arcOnGuide(
  const std::vector<Point> & guide, const std::vector<double> & arc, const Point & p)
{
  double best = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  for (std::size_t index = 0; index + 1U < guide.size(); ++index) {
    const Point delta = guide[index + 1U] - guide[index];
    const double length_squared = delta.squaredNorm();
    const double blend = length_squared > 1e-12 ?
      std::clamp((p - guide[index]).dot(delta) / length_squared, 0.0, 1.0) : 0.0;
    const double distance = (p - guide[index] - blend * delta).norm();
    if (distance < best) {
      best = distance;
      best_arc = arc[index] + blend * (arc[index + 1U] - arc[index]);
    }
  }
  return best_arc;
}

}  // namespace

MincoJointOptimizer::MincoJointOptimizer(MincoJointOptimizerParams params)
: params_(std::move(params))
{
  params_.samples_per_piece = std::max(2, params_.samples_per_piece);
  params_.min_piece_time = std::max(1e-3, params_.min_piece_time);
  params_.max_piece_time = std::max(params_.min_piece_time, params_.max_piece_time);
  params_.footprint_edge_samples = std::max(0, params_.footprint_edge_samples);

  // 矩形足迹：4 个角点 + 每条边 edge_samples 个等分点（1 = 边中点）。
  const double half_length = 0.5 * std::max(0.0, params_.footprint_length) +
    std::max(0.0, params_.footprint_safety_margin);
  const double half_width = 0.5 * std::max(0.0, params_.footprint_width) +
    std::max(0.0, params_.footprint_safety_margin);
  const std::vector<Point> corners = {
    Point(half_length, half_width), Point(-half_length, half_width),
    Point(-half_length, -half_width), Point(half_length, -half_width)};
  for (std::size_t index = 0; index < corners.size(); ++index) {
    const Point & start = corners[index];
    const Point & end = corners[(index + 1U) % corners.size()];
    footprint_offsets_.push_back(start);
    for (int sample = 1; sample <= params_.footprint_edge_samples; ++sample) {
      const double fraction = static_cast<double>(sample) / (params_.footprint_edge_samples + 1);
      footprint_offsets_.push_back(start + fraction * (end - start));
    }
  }
}

std::vector<Eigen::Vector2d> MincoJointOptimizer::resampleByArcLength(
  const std::vector<Eigen::Vector2d> & polyline, double spacing)
{
  if (polyline.size() < 2U) {
    return polyline;
  }
  std::vector<double> arc(polyline.size(), 0.0);
  for (std::size_t index = 1; index < polyline.size(); ++index) {
    arc[index] = arc[index - 1U] + (polyline[index] - polyline[index - 1U]).norm();
  }
  const double length = arc.back();
  const int pieces = (spacing > 1e-6 && length > 1e-9) ?
    std::max(1, static_cast<int>(std::lround(length / spacing))) : 1;

  std::vector<Point> result;
  result.reserve(static_cast<std::size_t>(pieces + 1));
  result.push_back(polyline.front());
  std::size_t segment = 0;
  for (int piece = 1; piece < pieces; ++piece) {
    const double target = length * static_cast<double>(piece) / pieces;
    while (segment + 2U < polyline.size() && arc[segment + 1U] < target) {
      ++segment;
    }
    const double span = arc[segment + 1U] - arc[segment];
    const double fraction = span > 1e-12 ?
      std::clamp((target - arc[segment]) / span, 0.0, 1.0) : 0.0;
    result.push_back(polyline[segment] + fraction * (polyline[segment + 1U] - polyline[segment]));
  }
  result.push_back(polyline.back());
  return result;
}

double MincoJointOptimizer::evaluate(
  const MincoJointProblem & problem,
  const Eigen::VectorXd & x,
  Eigen::VectorXd & gradient,
  MincoJointPenaltyStats * stats) const
{
  constexpr double kInfeasible = std::numeric_limits<double>::infinity();
  const int piece_count = problem.piece_count;
  gradient.setZero(x.size());
  if (piece_count <= 0 || x.size() != 3 * piece_count - 2 || !x.allFinite()) {
    return kInfeasible;
  }

  Eigen::MatrixXd inner;
  Eigen::VectorXd tau;
  unpack(x, piece_count, inner, tau);
  // 硬边界只防止 exp 溢出/退化；[Tmin, Tmax] 之内外的正常调节交给软罚。
  const double tau_lower = std::log(0.5 * params_.min_piece_time);
  const double tau_upper = std::log(2.0 * params_.max_piece_time);
  if ((tau.array() < tau_lower).any() || (tau.array() > tau_upper).any()) {
    return kInfeasible;
  }
  const Eigen::VectorXd durations = tau.array().exp().matrix();

  Eigen::Matrix<double, 2, 3> tail = Eigen::Matrix<double, 2, 3>::Zero();
  tail.col(0) = problem.tail_position;
  MincoS3 minco;
  if (!minco.solve(problem.head_state, tail, inner, durations)) {
    return kInfeasible;
  }

  MincoJointPenaltyStats local;
  Eigen::MatrixX2d grad_coeffs;
  Eigen::VectorXd grad_times;
  minco.getEnergyPartialGradByCoeffs(grad_coeffs);
  minco.getEnergyPartialGradByTimes(grad_times);
  local.energy = params_.energy_weight * minco.getEnergy();
  grad_coeffs *= params_.energy_weight;
  grad_times *= params_.energy_weight;

  local.time = params_.time_weight * durations.sum();
  grad_times.array() += params_.time_weight;
  double time_bound_cost = 0.0;
  for (int i = 0; i < piece_count; ++i) {
    const double below = params_.min_piece_time - durations(i);
    const double above = durations(i) - params_.max_piece_time;
    if (below > 0.0) {
      time_bound_cost += params_.time_bound_weight * cube(below);
      grad_times(i) -= 3.0 * params_.time_bound_weight * below * below;
    } else if (above > 0.0) {
      time_bound_cost += params_.time_bound_weight * cube(above);
      grad_times(i) += 3.0 * params_.time_bound_weight * above * above;
    }
  }

  const bool obstacle_enabled = static_cast<bool>(problem.distance);
  const bool center_enabled = obstacle_enabled && params_.center_clearance > 0.0;
  const bool footprint_enabled = obstacle_enabled && static_cast<bool>(problem.yaw);
  const double max_speed_squared = params_.max_velocity * params_.max_velocity;
  const double max_acceleration_squared = params_.max_acceleration * params_.max_acceleration;
  const int samples = params_.samples_per_piece;
  const int guide_segments = static_cast<int>(problem.guide.size()) - 1;
  const bool guide_enabled = params_.max_guide_deviation > 0.0 && guide_segments >= 1;
  const bool guide_ranged =
    static_cast<int>(problem.guide_segment_ranges.size()) == piece_count;
  const bool fixed_progress = problem.yaw_progress_durations.size() == piece_count;
  const Eigen::VectorXd & progress_durations =
    fixed_progress ? problem.yaw_progress_durations : durations;
  const double total_duration = std::max(1e-9, progress_durations.sum());

  double elapsed = 0.0;
  for (int i = 0; i < piece_count; ++i) {
    const double piece_duration = durations(i);
    const auto c = minco.coefficients().block<6, 2>(6 * i, 0);
    const double step = piece_duration / samples;
    for (int j = 0; j <= samples; ++j) {
      const double fraction = static_cast<double>(j) / samples;
      const double t1 = fraction * piece_duration;
      const double t2 = t1 * t1;
      const double t3 = t2 * t1;
      const double t4 = t2 * t2;
      const double t5 = t4 * t1;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
      beta0 << 1.0, t1, t2, t3, t4, t5;
      beta1 << 0.0, 1.0, 2.0 * t1, 3.0 * t2, 4.0 * t3, 5.0 * t4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * t1, 12.0 * t2, 20.0 * t3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * t1, 60.0 * t2;
      const Point position = c.transpose() * beta0;
      const Point velocity = c.transpose() * beta1;
      const Point acceleration = c.transpose() * beta2;
      const Point jerk = c.transpose() * beta3;

      double obstacle_penalty = 0.0;
      double dynamics_penalty = 0.0;
      Point grad_position = Point::Zero();
      Point grad_velocity = Point::Zero();
      Point grad_acceleration = Point::Zero();

      if (center_enabled) {
        double distance = 0.0;
        Point distance_gradient = Point::Zero();
        if (problem.distance(position, distance, distance_gradient) && std::isfinite(distance)) {
          const double violation = params_.center_clearance - distance;
          if (violation > 0.0) {
            obstacle_penalty += params_.obstacle_weight * cube(violation);
            grad_position -=
              3.0 * params_.obstacle_weight * violation * violation * distance_gradient;
            local.max_center_violation = std::max(local.max_center_violation, violation);
          }
        }
      }
      if (footprint_enabled) {
        // yaw 按固定时长下的归一化进度取参考值，与 (q, τ) 无关：
        // 足迹点对位置的雅可比为单位阵。
        const double yaw = problem.yaw(
          (elapsed + fraction * progress_durations(i)) / total_duration);
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        for (const Point & offset : footprint_offsets_) {
          const Point world = position + Point(
            cos_yaw * offset.x() - sin_yaw * offset.y(),
            sin_yaw * offset.x() + cos_yaw * offset.y());
          double distance = 0.0;
          Point distance_gradient = Point::Zero();
          if (!problem.distance(world, distance, distance_gradient) || !std::isfinite(distance)) {
            continue;
          }
          const double violation = params_.footprint_clearance - distance;
          if (violation > 0.0) {
            obstacle_penalty += params_.obstacle_weight * cube(violation);
            grad_position -=
              3.0 * params_.obstacle_weight * violation * violation * distance_gradient;
            local.max_footprint_violation = std::max(local.max_footprint_violation, violation);
          }
        }
      }

      double guide_penalty = 0.0;
      if (guide_enabled) {
        const int first = guide_ranged ? problem.guide_segment_ranges[i].first : 0;
        const int last = guide_ranged ? problem.guide_segment_ranges[i].second : guide_segments - 1;
        Point nearest = position;
        const double deviation = nearestOnGuide(problem.guide, first, last, position, nearest);
        const double excess = deviation - params_.max_guide_deviation;
        if (excess > 0.0 && deviation > 1e-9) {
          guide_penalty = params_.guide_weight * cube(excess);
          grad_position += 3.0 * params_.guide_weight * excess * excess *
            (position - nearest) / deviation;
          local.max_guide_excess = std::max(local.max_guide_excess, excess);
        }
      }

      if (params_.max_velocity > 0.0) {
        const double violation = velocity.squaredNorm() - max_speed_squared;
        if (violation > 0.0) {
          dynamics_penalty += params_.velocity_weight * cube(violation);
          grad_velocity += 6.0 * params_.velocity_weight * violation * violation * velocity;
          local.max_velocity_excess = std::max(
            local.max_velocity_excess, velocity.norm() - params_.max_velocity);
        }
      }
      if (params_.max_acceleration > 0.0) {
        const double violation = acceleration.squaredNorm() - max_acceleration_squared;
        if (violation > 0.0) {
          dynamics_penalty += params_.acceleration_weight * cube(violation);
          grad_acceleration +=
            6.0 * params_.acceleration_weight * violation * violation * acceleration;
          local.max_acceleration_excess = std::max(
            local.max_acceleration_excess, acceleration.norm() - params_.max_acceleration);
        }
      }
      const double speed = velocity.norm();
      if (params_.max_lateral_acceleration > 0.0 &&
        speed > std::max(1e-6, params_.lateral_min_speed))
      {
        const double cross = velocity.x() * acceleration.y() - velocity.y() * acceleration.x();
        const double lateral = std::abs(cross) / speed;
        const double violation = lateral - params_.max_lateral_acceleration;
        if (violation > 0.0) {
          // lat = |v×a|/|v|：d/dv = σ(a_y,-a_x)/|v| - |v×a| v/|v|³，
          // d/da = σ(-v_y,v_x)/|v|。
          const double sign = cross >= 0.0 ? 1.0 : -1.0;
          const double scale = 3.0 * params_.lateral_weight * violation * violation;
          dynamics_penalty += params_.lateral_weight * cube(violation);
          grad_velocity += scale * (sign * Point(acceleration.y(), -acceleration.x()) / speed -
            std::abs(cross) * velocity / (speed * speed * speed));
          grad_acceleration += scale * sign * Point(-velocity.y(), velocity.x()) / speed;
          local.max_lateral_excess = std::max(local.max_lateral_excess, violation);
        }
      }

      const double penalty = obstacle_penalty + guide_penalty + dynamics_penalty;
      if (penalty <= 0.0) {
        continue;
      }
      // 梯形权重 ω_j，积分步长 T_i/K；样本时刻 t = (j/K) T_i 也依赖 T_i。
      const double node_weight = (j == 0 || j == samples) ? 0.5 : 1.0;
      const double weight = node_weight * step;
      grad_coeffs.block<6, 2>(6 * i, 0) += weight * (
        beta0 * grad_position.transpose() + beta1 * grad_velocity.transpose() +
        beta2 * grad_acceleration.transpose());
      grad_times(i) += weight * fraction * (
        grad_position.dot(velocity) + grad_velocity.dot(acceleration) +
        grad_acceleration.dot(jerk)) + node_weight / samples * penalty;
      local.obstacle_cost += weight * obstacle_penalty;
      local.guide_cost += weight * guide_penalty;
      local.dynamics_cost += weight * dynamics_penalty;
    }
    elapsed += progress_durations(i);
  }

  Eigen::MatrixXd grad_points;
  Eigen::VectorXd grad_durations;
  if (!minco.propagateGrad(grad_coeffs, grad_times, grad_points, grad_durations)) {
    return kInfeasible;
  }
  const int inner_count = piece_count - 1;
  gradient.head(2 * inner_count) =
    Eigen::Map<const Eigen::VectorXd>(grad_points.data(), 2 * inner_count);
  // dJ/dτ = dJ/dT ⊙ T。
  gradient.tail(piece_count) = grad_durations.cwiseProduct(durations);

  const double cost = local.energy + local.time + time_bound_cost +
    local.obstacle_cost + local.guide_cost + local.dynamics_cost;
  if (!std::isfinite(cost) || !gradient.allFinite()) {
    return kInfeasible;
  }
  if (stats != nullptr) {
    *stats = local;
  }
  return cost;
}

MincoJointResult MincoJointOptimizer::optimize(
  const Eigen::Matrix<double, 2, 3> & head_state,
  const std::vector<Eigen::Vector2d> & initial_waypoints,
  const Eigen::VectorXd & initial_durations,
  const JointDistanceQuery & distance,
  const JointYawQuery & yaw,
  const std::vector<Eigen::Vector2d> & guide) const
{
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  MincoJointResult result;
  const auto finish = [&result, &started]() {
      result.wall_time_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      return result;
    };

  const int piece_count = static_cast<int>(initial_durations.size());
  if (initial_waypoints.size() < 2U ||
    piece_count + 1 != static_cast<int>(initial_waypoints.size()) ||
    !head_state.allFinite() || !initial_durations.allFinite() ||
    (initial_durations.array() <= 0.0).any())
  {
    result.termination = "invalid_input";
    return finish();
  }
  for (const Point & point : initial_waypoints) {
    if (!point.allFinite()) {
      result.termination = "invalid_input";
      return finish();
    }
  }

  MincoJointProblem problem;
  problem.head_state = head_state;
  problem.head_state.col(0) = initial_waypoints.front();
  problem.tail_position = initial_waypoints.back();
  problem.piece_count = piece_count;
  problem.distance = distance;
  problem.yaw = yaw;
  problem.yaw_progress_durations = initial_durations.cwiseMax(params_.min_piece_time)
    .cwiseMin(params_.max_piece_time);

  for (const Point & point : guide) {
    if (!point.allFinite()) {
      result.termination = "invalid_input";
      return finish();
    }
  }
  if (guide.size() >= 2U) {
    // 每段只搜索初值两端点投影弧长 ± slack 内的折线段，
    // 保证 U 形回折时不会误吸到另一支。
    problem.guide = guide;
    std::vector<double> arc(guide.size(), 0.0);
    for (std::size_t index = 1; index < guide.size(); ++index) {
      arc[index] = arc[index - 1U] + (guide[index] - guide[index - 1U]).norm();
    }
    const double slack = 2.0 * std::max(0.0, params_.max_guide_deviation) + 0.5;
    const int last_segment = static_cast<int>(guide.size()) - 2;
    double previous_arc = arcOnGuide(guide, arc, initial_waypoints.front());
    for (int i = 0; i < piece_count; ++i) {
      const double next_arc = arcOnGuide(
        guide, arc, initial_waypoints[static_cast<std::size_t>(i + 1)]);
      const double lower = std::min(previous_arc, next_arc) - slack;
      const double upper = std::max(previous_arc, next_arc) + slack;
      int first = 0;
      while (first < last_segment && arc[static_cast<std::size_t>(first + 1)] < lower) {
        ++first;
      }
      int last = first;
      while (last < last_segment && arc[static_cast<std::size_t>(last + 1)] < upper) {
        ++last;
      }
      problem.guide_segment_ranges.emplace_back(first, last);
      previous_arc = next_arc;
    }
  }

  const int inner_count = piece_count - 1;
  Eigen::VectorXd x(2 * inner_count + piece_count);
  for (int i = 0; i < inner_count; ++i) {
    x.segment<2>(2 * i) = initial_waypoints[static_cast<std::size_t>(i + 1)];
  }
  for (int i = 0; i < piece_count; ++i) {
    x(2 * inner_count + i) = std::log(std::clamp(
      initial_durations(i), params_.min_piece_time, params_.max_piece_time));
  }

  const lbfgs::Objective objective =
    [this, &problem](const Eigen::VectorXd & point, Eigen::VectorXd & grad) {
      return evaluate(problem, point, grad);
    };
  const lbfgs::LbfgsResult solver = lbfgs::minimize(x, objective, params_.solver);
  result.termination = lbfgs::statusName(solver.status);
  result.iterations = solver.iterations;
  result.evaluations = solver.evaluations;
  result.initial_cost = solver.initial_cost;
  if (solver.status == lbfgs::LbfgsStatus::kInvalidInitial) {
    return finish();
  }

  // 输出时长硬夹到 [Tmin, Tmax]，并按夹后的点重新统计代价与残差。
  x.tail(piece_count) = x.tail(piece_count).cwiseMax(std::log(params_.min_piece_time))
    .cwiseMin(std::log(params_.max_piece_time));
  Eigen::VectorXd gradient;
  result.final_cost = evaluate(problem, x, gradient, &result.stats);
  ++result.evaluations;
  if (!std::isfinite(result.final_cost)) {
    result.termination = "invalid_final";
    return finish();
  }

  result.waypoints.reserve(initial_waypoints.size());
  result.waypoints.push_back(initial_waypoints.front());
  for (int i = 0; i < inner_count; ++i) {
    result.waypoints.push_back(x.segment<2>(2 * i));
  }
  result.waypoints.push_back(initial_waypoints.back());
  result.durations = x.tail(piece_count).array().exp().matrix();
  result.solved = true;
  const MincoJointPenaltyStats & stats = result.stats;
  result.constraints_satisfied =
    stats.max_center_violation <= params_.obstacle_tolerance &&
    stats.max_footprint_violation <= params_.obstacle_tolerance &&
    stats.max_guide_excess <= params_.guide_tolerance &&
    stats.max_velocity_excess <= params_.dynamics_tolerance &&
    stats.max_acceleration_excess <= params_.dynamics_tolerance &&
    stats.max_lateral_excess <= params_.dynamics_tolerance;
  return finish();
}

}  // namespace minco_planner
