// Copyright 2026
//
// 自实现 L-BFGS：two-loop recursion + Lewis-Overton 弱 Wolfe 线搜索。
// 算法结构参考 GCOPTER lbfgs.hpp（MIT License, Copyright (c) 2021 Zhepei Wang），
// 此处按本仓库风格重写，只保留 MINCO 联合优化需要的部分，并增加墙钟预算终止。

#ifndef MINCO_PLANNER__TRAJECTORY__LBFGS_HPP_
#define MINCO_PLANNER__TRAJECTORY__LBFGS_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace minco_planner
{
namespace lbfgs
{

struct LbfgsParams
{
  // 保存的 (s, y) 对数。
  int memory_size = 8;
  // 收敛判据：||g||_inf / max(1, ||x||_inf) < g_epsilon。
  double g_epsilon = 1e-5;
  // 相对下降判据：(f_{k-past} - f_k) / max(1, |f_k|) < delta；past <= 0 关闭。
  int past = 3;
  double delta = 1e-6;
  int max_iterations = 200;
  // 墙钟预算 [ms]；<= 0 关闭。
  double time_budget_ms = 15.0;
  int max_linesearch = 48;
  double min_step = 1e-20;
  double max_step = 1e20;
  // Armijo 充分下降系数与弱 Wolfe 曲率系数。
  double f_dec_coeff = 1e-4;
  double s_curv_coeff = 0.9;
};

enum class LbfgsStatus
{
  kConverged,
  kStopByDelta,
  kMaxIterations,
  kTimeBudget,
  kLineSearchFailed,
  kInvalidInitial,
};

inline const char * statusName(LbfgsStatus status)
{
  switch (status) {
    case LbfgsStatus::kConverged: return "converged";
    case LbfgsStatus::kStopByDelta: return "stop_by_delta";
    case LbfgsStatus::kMaxIterations: return "max_iterations";
    case LbfgsStatus::kTimeBudget: return "time_budget";
    case LbfgsStatus::kLineSearchFailed: return "line_search_failed";
    case LbfgsStatus::kInvalidInitial: return "invalid_initial";
  }
  return "unknown";
}

struct LbfgsResult
{
  LbfgsStatus status = LbfgsStatus::kInvalidInitial;
  int iterations = 0;
  int evaluations = 0;
  double initial_cost = std::numeric_limits<double>::quiet_NaN();
  double final_cost = std::numeric_limits<double>::quiet_NaN();
};

// 返回代价并写梯度；非有限返回值表示该点不可行（线搜索会缩步）。
using Objective = std::function<double(const Eigen::VectorXd & x, Eigen::VectorXd & gradient)>;

/**
 * @brief 最小化 objective，x 既是初值也是输出。
 * @note 线搜索失败或预算耗尽时 x 保持为已接受的最优迭代点（代价单调不增），
 *       调用方永远拿到一个不比初值差的可行点。
 */
inline LbfgsResult minimize(
  Eigen::VectorXd & x, const Objective & objective, const LbfgsParams & params)
{
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  const auto elapsedMs = [&started]() {
      return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    };

  LbfgsResult result;
  const int n = static_cast<int>(x.size());
  const int m = std::max(1, params.memory_size);
  Eigen::VectorXd g(n);
  double fx = objective(x, g);
  ++result.evaluations;
  result.initial_cost = fx;
  result.final_cost = fx;
  if (!std::isfinite(fx) || !g.allFinite()) {
    result.status = LbfgsStatus::kInvalidInitial;
    return result;
  }

  const auto converged = [&params](const Eigen::VectorXd & point, const Eigen::VectorXd & grad) {
      const double scale = std::max(1.0, point.cwiseAbs().maxCoeff());
      return grad.cwiseAbs().maxCoeff() / scale < params.g_epsilon;
    };
  if (n == 0 || converged(x, g)) {
    result.status = LbfgsStatus::kConverged;
    return result;
  }

  Eigen::MatrixXd s_history(n, m);
  Eigen::MatrixXd y_history(n, m);
  Eigen::VectorXd rho(m);
  Eigen::VectorXd alpha(m);
  std::vector<double> past_costs;
  if (params.past > 0) {
    past_costs.assign(static_cast<std::size_t>(params.past), fx);
  }
  int stored = 0;
  int newest = -1;

  Eigen::VectorXd d = -g;
  double step = 1.0 / std::max(1e-12, d.norm());
  Eigen::VectorXd x_previous(n);
  Eigen::VectorXd g_previous(n);

  for (int iteration = 1; ; ++iteration) {
    x_previous = x;
    g_previous = g;
    const double f_previous = fx;

    // 拟牛顿方向因舍入失去下降性时重置为最速下降并清空历史。
    if (g.dot(d) >= 0.0) {
      d = -g;
      step = 1.0 / std::max(1e-12, d.norm());
      stored = 0;
      newest = -1;
    }
    // Lewis-Overton：Armijo 不满足则二分收缩上界，曲率不满足则扩大下界。
    const double dg_init = g.dot(d);
    bool line_search_ok = false;
    if (dg_init < 0.0) {
      const double dg_test = params.f_dec_coeff * dg_init;
      const double ds_test = params.s_curv_coeff * dg_init;
      double lower = 0.0;
      double upper = std::numeric_limits<double>::infinity();
      bool bracketed = false;
      for (int trial = 0; trial < params.max_linesearch; ++trial) {
        x = x_previous + step * d;
        fx = objective(x, g);
        ++result.evaluations;
        if (!std::isfinite(fx) || !g.allFinite() || fx > f_previous + step * dg_test) {
          upper = step;
          bracketed = true;
        } else if (g.dot(d) < ds_test) {
          lower = step;
        } else {
          line_search_ok = true;
          break;
        }
        step = bracketed ? 0.5 * (lower + upper) : 2.0 * step;
        if (step < params.min_step || step > params.max_step) {
          break;
        }
      }
    }
    if (!line_search_ok) {
      x = x_previous;
      g = g_previous;
      fx = f_previous;
      result.status = LbfgsStatus::kLineSearchFailed;
      result.iterations = iteration - 1;
      break;
    }
    result.final_cost = fx;
    result.iterations = iteration;

    if (converged(x, g)) {
      result.status = LbfgsStatus::kConverged;
      break;
    }
    if (params.past > 0) {
      const std::size_t slot = static_cast<std::size_t>(iteration % params.past);
      if (iteration >= params.past &&
        (past_costs[slot] - fx) / std::max(1.0, std::abs(fx)) < params.delta)
      {
        result.status = LbfgsStatus::kStopByDelta;
        break;
      }
      past_costs[slot] = fx;
    }
    if (iteration >= params.max_iterations) {
      result.status = LbfgsStatus::kMaxIterations;
      break;
    }
    if (params.time_budget_ms > 0.0 && elapsedMs() >= params.time_budget_ms) {
      result.status = LbfgsStatus::kTimeBudget;
      break;
    }

    // 曲率条件 y^T s > 0 由弱 Wolfe 保证；数值退化时跳过本次更新。
    const Eigen::VectorXd s_new = x - x_previous;
    const Eigen::VectorXd y_new = g - g_previous;
    const double ys = y_new.dot(s_new);
    const double yy = y_new.squaredNorm();
    if (ys > 1e-16 * std::max(1.0, yy)) {
      newest = (newest + 1) % m;
      s_history.col(newest) = s_new;
      y_history.col(newest) = y_new;
      rho(newest) = 1.0 / ys;
      stored = std::min(stored + 1, m);
    }

    // Two-loop recursion：d = -H g。
    d = -g;
    int index = newest;
    for (int k = 0; k < stored; ++k) {
      alpha(index) = rho(index) * s_history.col(index).dot(d);
      d -= alpha(index) * y_history.col(index);
      index = (index - 1 + m) % m;
    }
    if (stored > 0) {
      d *= 1.0 / (rho(newest) * y_history.col(newest).squaredNorm());
    }
    for (int k = 0; k < stored; ++k) {
      index = (index + 1) % m;
      const double beta = rho(index) * y_history.col(index).dot(d);
      d += (alpha(index) - beta) * s_history.col(index);
    }
    step = 1.0;
  }
  return result;
}

}  // namespace lbfgs
}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__LBFGS_HPP_
