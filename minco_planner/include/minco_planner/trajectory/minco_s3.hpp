// Adapted in 2026 from GCOPTER MINCO (MIT License).
// Copyright (c) 2021 Zhepei Wang

#ifndef MINCO_PLANNER__TRAJECTORY__MINCO_S3_HPP_
#define MINCO_PLANNER__TRAJECTORY__MINCO_S3_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace minco_planner
{

class BandedSystem
{
public:
  void create(int size, int lower_bandwidth, int upper_bandwidth)
  {
    size_ = size;
    lower_bandwidth_ = lower_bandwidth;
    upper_bandwidth_ = upper_bandwidth;
    data_.assign(
      static_cast<std::size_t>(size_) *
      static_cast<std::size_t>(lower_bandwidth_ + upper_bandwidth_ + 1), 0.0);
  }

  void reset()
  {
    std::fill(data_.begin(), data_.end(), 0.0);
  }

  double & operator()(int row, int column)
  {
    return data_[static_cast<std::size_t>((row - column + upper_bandwidth_) * size_ + column)];
  }

  const double & operator()(int row, int column) const
  {
    return data_[static_cast<std::size_t>((row - column + upper_bandwidth_) * size_ + column)];
  }

  bool factorizeLu()
  {
    for (int k = 0; k <= size_ - 2; ++k) {
      const int max_row = std::min(k + lower_bandwidth_, size_ - 1);
      const double pivot = operator()(k, k);
      if (std::abs(pivot) <= 1e-12) {
        return false;
      }
      for (int row = k + 1; row <= max_row; ++row) {
        if (operator()(row, k) != 0.0) {
          operator()(row, k) /= pivot;
        }
      }
      const int max_column = std::min(k + upper_bandwidth_, size_ - 1);
      for (int column = k + 1; column <= max_column; ++column) {
        const double value = operator()(k, column);
        if (value == 0.0) {
          continue;
        }
        for (int row = k + 1; row <= max_row; ++row) {
          if (operator()(row, k) != 0.0) {
            operator()(row, column) -= operator()(row, k) * value;
          }
        }
      }
    }
    return std::abs(operator()(size_ - 1, size_ - 1)) > 1e-12;
  }

  template<typename Derived>
  bool solve(Eigen::MatrixBase<Derived> & right_hand_side) const
  {
    for (int column = 0; column < size_; ++column) {
      const int max_row = std::min(column + lower_bandwidth_, size_ - 1);
      for (int row = column + 1; row <= max_row; ++row) {
        if (operator()(row, column) != 0.0) {
          right_hand_side.row(row) -=
            operator()(row, column) * right_hand_side.row(column);
        }
      }
    }
    for (int column = size_ - 1; column >= 0; --column) {
      const double pivot = operator()(column, column);
      if (std::abs(pivot) <= 1e-12) {
        return false;
      }
      right_hand_side.row(column) /= pivot;
      const int min_row = std::max(0, column - upper_bandwidth_);
      for (int row = min_row; row < column; ++row) {
        if (operator()(row, column) != 0.0) {
          right_hand_side.row(row) -=
            operator()(row, column) * right_hand_side.row(column);
        }
      }
    }
    return true;
  }

  // 用已有 LU 解 A^T x = b：A^T = U^T L^T，先对 U^T 前代，再对单位下三角 L^T 回代。
  // 联合优化的伴随梯度每次迭代只需这一次转置求解。必须先 factorizeLu()。
  template<typename Derived>
  bool solveAdj(Eigen::MatrixBase<Derived> & right_hand_side) const
  {
    for (int column = 0; column < size_; ++column) {
      const int min_row = std::max(0, column - upper_bandwidth_);
      for (int row = min_row; row < column; ++row) {
        if (operator()(row, column) != 0.0) {
          right_hand_side.row(column) -=
            operator()(row, column) * right_hand_side.row(row);
        }
      }
      const double pivot = operator()(column, column);
      if (std::abs(pivot) <= 1e-12) {
        return false;
      }
      right_hand_side.row(column) /= pivot;
    }
    for (int column = size_ - 1; column >= 0; --column) {
      const int max_row = std::min(column + lower_bandwidth_, size_ - 1);
      for (int row = column + 1; row <= max_row; ++row) {
        if (operator()(row, column) != 0.0) {
          right_hand_side.row(column) -=
            operator()(row, column) * right_hand_side.row(row);
        }
      }
    }
    return true;
  }

private:
  int size_ = 0;
  int lower_bandwidth_ = 0;
  int upper_bandwidth_ = 0;
  std::vector<double> data_;
};

struct MincoSample
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
  Eigen::Vector2d jerk = Eigen::Vector2d::Zero();
};

class MincoS3
{
public:
  bool solve(
    const Eigen::Matrix<double, 2, 3> & head_state,
    const Eigen::Matrix<double, 2, 3> & tail_state,
    const Eigen::MatrixXd & inner_points,
    const Eigen::VectorXd & durations)
  {
    const int piece_count = static_cast<int>(durations.size());
    if (piece_count <= 0 || inner_points.rows() != 2 ||
      inner_points.cols() != piece_count - 1 || (durations.array() <= 0.0).any())
    {
      valid_ = false;
      return false;
    }

    piece_durations_ = durations;
    const Eigen::VectorXd t2 = durations.cwiseProduct(durations);
    const Eigen::VectorXd t3 = t2.cwiseProduct(durations);
    const Eigen::VectorXd t4 = t2.cwiseProduct(t2);
    const Eigen::VectorXd t5 = t4.cwiseProduct(durations);
    system_.create(6 * piece_count, 6, 6);
    coefficients_.resize(6 * piece_count, 2);
    system_.reset();
    coefficients_.setZero();

    system_(0, 0) = 1.0;
    system_(1, 1) = 1.0;
    system_(2, 2) = 2.0;
    coefficients_.row(0) = head_state.col(0).transpose();
    coefficients_.row(1) = head_state.col(1).transpose();
    coefficients_.row(2) = head_state.col(2).transpose();

    for (int i = 0; i < piece_count - 1; ++i) {
      system_(6 * i + 3, 6 * i + 3) = 6.0;
      system_(6 * i + 3, 6 * i + 4) = 24.0 * durations(i);
      system_(6 * i + 3, 6 * i + 5) = 60.0 * t2(i);
      system_(6 * i + 3, 6 * i + 9) = -6.0;
      system_(6 * i + 4, 6 * i + 4) = 24.0;
      system_(6 * i + 4, 6 * i + 5) = 120.0 * durations(i);
      system_(6 * i + 4, 6 * i + 10) = -24.0;

      system_(6 * i + 5, 6 * i) = 1.0;
      system_(6 * i + 5, 6 * i + 1) = durations(i);
      system_(6 * i + 5, 6 * i + 2) = t2(i);
      system_(6 * i + 5, 6 * i + 3) = t3(i);
      system_(6 * i + 5, 6 * i + 4) = t4(i);
      system_(6 * i + 5, 6 * i + 5) = t5(i);

      system_(6 * i + 6, 6 * i) = 1.0;
      system_(6 * i + 6, 6 * i + 1) = durations(i);
      system_(6 * i + 6, 6 * i + 2) = t2(i);
      system_(6 * i + 6, 6 * i + 3) = t3(i);
      system_(6 * i + 6, 6 * i + 4) = t4(i);
      system_(6 * i + 6, 6 * i + 5) = t5(i);
      system_(6 * i + 6, 6 * i + 6) = -1.0;

      system_(6 * i + 7, 6 * i + 1) = 1.0;
      system_(6 * i + 7, 6 * i + 2) = 2.0 * durations(i);
      system_(6 * i + 7, 6 * i + 3) = 3.0 * t2(i);
      system_(6 * i + 7, 6 * i + 4) = 4.0 * t3(i);
      system_(6 * i + 7, 6 * i + 5) = 5.0 * t4(i);
      system_(6 * i + 7, 6 * i + 7) = -1.0;

      system_(6 * i + 8, 6 * i + 2) = 2.0;
      system_(6 * i + 8, 6 * i + 3) = 6.0 * durations(i);
      system_(6 * i + 8, 6 * i + 4) = 12.0 * t2(i);
      system_(6 * i + 8, 6 * i + 5) = 20.0 * t3(i);
      system_(6 * i + 8, 6 * i + 8) = -2.0;
      coefficients_.row(6 * i + 5) = inner_points.col(i).transpose();
    }

    const int last = piece_count - 1;
    system_(6 * piece_count - 3, 6 * last) = 1.0;
    system_(6 * piece_count - 3, 6 * last + 1) = durations(last);
    system_(6 * piece_count - 3, 6 * last + 2) = t2(last);
    system_(6 * piece_count - 3, 6 * last + 3) = t3(last);
    system_(6 * piece_count - 3, 6 * last + 4) = t4(last);
    system_(6 * piece_count - 3, 6 * last + 5) = t5(last);
    system_(6 * piece_count - 2, 6 * last + 1) = 1.0;
    system_(6 * piece_count - 2, 6 * last + 2) = 2.0 * durations(last);
    system_(6 * piece_count - 2, 6 * last + 3) = 3.0 * t2(last);
    system_(6 * piece_count - 2, 6 * last + 4) = 4.0 * t3(last);
    system_(6 * piece_count - 2, 6 * last + 5) = 5.0 * t4(last);
    system_(6 * piece_count - 1, 6 * last + 2) = 2.0;
    system_(6 * piece_count - 1, 6 * last + 3) = 6.0 * durations(last);
    system_(6 * piece_count - 1, 6 * last + 4) = 12.0 * t2(last);
    system_(6 * piece_count - 1, 6 * last + 5) = 20.0 * t3(last);
    coefficients_.row(6 * piece_count - 3) = tail_state.col(0).transpose();
    coefficients_.row(6 * piece_count - 2) = tail_state.col(1).transpose();
    coefficients_.row(6 * piece_count - 1) = tail_state.col(2).transpose();

    valid_ = system_.factorizeLu() && system_.solve(coefficients_) && coefficients_.allFinite();
    return valid_;
  }

  MincoSample sample(int piece, double time) const
  {
    MincoSample sample;
    if (!valid_ || piece < 0 || piece >= piece_durations_.size()) {
      return sample;
    }
    const double t = std::max(0.0, std::min(time, piece_durations_(piece)));
    const auto block = coefficients_.block<6, 2>(6 * piece, 0);
    double power = 1.0;
    for (int order = 0; order <= 5; ++order) {
      sample.position += power * block.row(order).transpose();
      power *= t;
    }
    power = 1.0;
    for (int order = 1; order <= 5; ++order) {
      sample.velocity += static_cast<double>(order) * power * block.row(order).transpose();
      power *= t;
    }
    power = 1.0;
    for (int order = 2; order <= 5; ++order) {
      sample.acceleration +=
        static_cast<double>(order * (order - 1)) * power * block.row(order).transpose();
      power *= t;
    }
    power = 1.0;
    for (int order = 3; order <= 5; ++order) {
      sample.jerk += static_cast<double>(order * (order - 1) * (order - 2)) *
        power * block.row(order).transpose();
      power *= t;
    }
    return sample;
  }

  bool valid() const {return valid_;}
  int pieceCount() const {return static_cast<int>(piece_durations_.size());}
  double pieceDuration(int piece) const {return piece_durations_(piece);}
  // 6N x 2，第 6i+k 行是第 i 段 t^k 的系数。
  const Eigen::MatrixX2d & coefficients() const {return coefficients_;}

  /**
   * @brief jerk 能量 \f$ \sum_i \int_0^{T_i} \|p'''(t)\|^2 dt \f$（逐段解析积分）。
   * @note p''' = 6c3 + 24c4 t + 60c5 t^2，展开后各项系数与 GCOPTER MINCO_S3NU 一致。
   */
  double getEnergy() const
  {
    double energy = 0.0;
    for (int i = 0; i < pieceCount(); ++i) {
      const double t1 = piece_durations_(i);
      const double t2 = t1 * t1;
      const double t3 = t2 * t1;
      const double t4 = t2 * t2;
      const double t5 = t4 * t1;
      const Eigen::Vector2d c3 = coefficients_.row(6 * i + 3).transpose();
      const Eigen::Vector2d c4 = coefficients_.row(6 * i + 4).transpose();
      const Eigen::Vector2d c5 = coefficients_.row(6 * i + 5).transpose();
      energy += 36.0 * c3.squaredNorm() * t1 + 144.0 * c4.dot(c3) * t2 +
        192.0 * c4.squaredNorm() * t3 + 240.0 * c5.dot(c3) * t3 +
        720.0 * c5.dot(c4) * t4 + 720.0 * c5.squaredNorm() * t5;
    }
    return energy;
  }

  // 能量对系数的偏导（T 固定），只有 c3..c5 非零。
  void getEnergyPartialGradByCoeffs(Eigen::MatrixX2d & gradient) const
  {
    gradient.setZero(6 * pieceCount(), 2);
    for (int i = 0; i < pieceCount(); ++i) {
      const double t1 = piece_durations_(i);
      const double t2 = t1 * t1;
      const double t3 = t2 * t1;
      const double t4 = t2 * t2;
      const double t5 = t4 * t1;
      const auto c3 = coefficients_.row(6 * i + 3);
      const auto c4 = coefficients_.row(6 * i + 4);
      const auto c5 = coefficients_.row(6 * i + 5);
      gradient.row(6 * i + 3) = 72.0 * c3 * t1 + 144.0 * c4 * t2 + 240.0 * c5 * t3;
      gradient.row(6 * i + 4) = 144.0 * c3 * t2 + 384.0 * c4 * t3 + 720.0 * c5 * t4;
      gradient.row(6 * i + 5) = 240.0 * c3 * t3 + 720.0 * c4 * t4 + 1440.0 * c5 * t5;
    }
  }

  // 能量对段时长的偏导（系数固定）。
  void getEnergyPartialGradByTimes(Eigen::VectorXd & gradient) const
  {
    gradient.setZero(pieceCount());
    for (int i = 0; i < pieceCount(); ++i) {
      const double t1 = piece_durations_(i);
      const double t2 = t1 * t1;
      const double t3 = t2 * t1;
      const double t4 = t2 * t2;
      const Eigen::Vector2d c3 = coefficients_.row(6 * i + 3).transpose();
      const Eigen::Vector2d c4 = coefficients_.row(6 * i + 4).transpose();
      const Eigen::Vector2d c5 = coefficients_.row(6 * i + 5).transpose();
      gradient(i) = 36.0 * c3.squaredNorm() + 288.0 * c4.dot(c3) * t1 +
        576.0 * c4.squaredNorm() * t2 + 720.0 * c5.dot(c3) * t2 +
        2880.0 * c5.dot(c4) * t3 + 3600.0 * c5.squaredNorm() * t4;
    }
  }

  /**
   * @brief 伴随法把 \f$ \partial J/\partial c \f$（T 固定）与 \f$ \partial J/\partial T \f$
   *        （c 固定）反传成对内点 q 与时长 T 的全导数。
   * @param partial_grad_by_coeffs 6N x 2；函数内部会被 A^{-T} 就地覆盖。
   * @param partial_grad_by_times  N。
   * @param grad_by_points         输出 2 x (N-1)。
   * @param grad_by_times          输出 N。
   * @note A(T)c = b(q)：设 G = A^{-T} ∂J/∂c，则 ∂J/∂b = G，q_i 只出现在 b 的第 6i+5 行；
   *       ∂J/∂T_i = ∂J/∂T_i|_c - G^T (∂A/∂T_i) c。每次调用只做一次 A^T 求解。
   */
  bool propagateGrad(
    Eigen::MatrixX2d & partial_grad_by_coeffs,
    const Eigen::VectorXd & partial_grad_by_times,
    Eigen::MatrixXd & grad_by_points,
    Eigen::VectorXd & grad_by_times) const
  {
    const int piece_count = pieceCount();
    if (!valid_ || partial_grad_by_coeffs.rows() != 6 * piece_count ||
      partial_grad_by_times.size() != piece_count)
    {
      return false;
    }
    if (!system_.solveAdj(partial_grad_by_coeffs)) {
      return false;
    }
    const Eigen::MatrixX2d & adjoint = partial_grad_by_coeffs;
    grad_by_points.resize(2, piece_count - 1);
    grad_by_times = partial_grad_by_times;
    for (int i = 0; i < piece_count; ++i) {
      const double t1 = piece_durations_(i);
      const double t2 = t1 * t1;
      const double t3 = t2 * t1;
      const double t4 = t2 * t2;
      const auto c = coefficients_.block<6, 2>(6 * i, 0);
      // 段末 p'、p''、p''' 恰是位置/速度/加速度连续行对 T 的导数。
      const Eigen::RowVector2d velocity = c.row(1) + 2.0 * t1 * c.row(2) +
        3.0 * t2 * c.row(3) + 4.0 * t3 * c.row(4) + 5.0 * t4 * c.row(5);
      const Eigen::RowVector2d acceleration = 2.0 * c.row(2) + 6.0 * t1 * c.row(3) +
        12.0 * t2 * c.row(4) + 20.0 * t3 * c.row(5);
      const Eigen::RowVector2d jerk = 6.0 * c.row(3) + 24.0 * t1 * c.row(4) +
        60.0 * t2 * c.row(5);
      if (i < piece_count - 1) {
        const Eigen::RowVector2d snap = 24.0 * c.row(4) + 120.0 * t1 * c.row(5);
        const Eigen::RowVector2d crackle = 120.0 * c.row(5);
        grad_by_times(i) -= adjoint.row(6 * i + 3).dot(snap) +
          adjoint.row(6 * i + 4).dot(crackle) +
          adjoint.row(6 * i + 5).dot(velocity) + adjoint.row(6 * i + 6).dot(velocity) +
          adjoint.row(6 * i + 7).dot(acceleration) + adjoint.row(6 * i + 8).dot(jerk);
        grad_by_points.col(i) = adjoint.row(6 * i + 5).transpose();
      } else {
        grad_by_times(i) -= adjoint.row(6 * i + 3).dot(velocity) +
          adjoint.row(6 * i + 4).dot(acceleration) + adjoint.row(6 * i + 5).dot(jerk);
      }
    }
    return grad_by_times.allFinite() && grad_by_points.allFinite();
  }

private:
  BandedSystem system_;
  Eigen::MatrixX2d coefficients_;
  Eigen::VectorXd piece_durations_;
  bool valid_ = false;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY__MINCO_S3_HPP_
