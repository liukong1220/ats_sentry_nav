// Copyright 2026

#include "ats_swerve_mpc/qp/ltv_qp_problem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace ats_swerve_mpc {
namespace {

using Matrix3 = Eigen::Matrix3d;
constexpr double kPi = 3.14159265358979323846;
constexpr double kEsdfUpperBound = 1.0e4;

void setError(std::string *error, const char *message) {
  if (error != nullptr) {
    *error = message;
  }
}

bool validWeight(const Eigen::Vector3d &weight) {
  return weight.allFinite() && (weight.array() >= 0.0).all();
}

bool validNonnegativeLimit(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool finiteStates(const std::vector<State> &states) {
  for (const auto &state : states) {
    if (!state.allFinite()) {
      return false;
    }
  }
  return true;
}

bool finiteControls(const std::vector<Control> &controls) {
  for (const auto &control : controls) {
    if (!control.allFinite()) {
      return false;
    }
  }
  return true;
}

int findCscSlot(const LtvQpSparseStructure &structure, int row, int column) {
  if (column < 0 || column >= structure.columns) {
    return -1;
  }
  const int begin = structure.column_offsets[static_cast<std::size_t>(column)];
  const int end =
      structure.column_offsets[static_cast<std::size_t>(column + 1)];
  const auto first = structure.row_indices.begin() + begin;
  const auto last = structure.row_indices.begin() + end;
  const auto it = std::lower_bound(first, last, row);
  if (it == last || *it != row) {
    return -1;
  }
  return static_cast<int>(it - structure.row_indices.begin());
}

std::array<Eigen::Matrix<double, 2, 3>, 4> wheelJacobians(
    const Se2MpcConfig &config) {
  const double x = std::max(0.0, config.wheel_base_x);
  const double y = std::max(0.0, config.wheel_base_y);
  const std::array<Eigen::Vector2d, 4> positions{{
      Eigen::Vector2d(x, y), Eigen::Vector2d(x, -y),
      Eigen::Vector2d(-x, y), Eigen::Vector2d(-x, -y)}};
  std::array<Eigen::Matrix<double, 2, 3>, 4> matrices{};
  for (std::size_t i = 0; i < 4; ++i) {
    matrices[i] << 1.0, 0.0, -positions[i](1),
                   0.0, 1.0,  positions[i](0);
  }
  return matrices;
}

}  // namespace

bool LtvQpSparseStructure::valid(std::string *error) const {
  if (rows <= 0 || columns <= 0) {
    setError(error, "sparse matrix dimensions must be positive");
    return false;
  }
  if (column_offsets.size() != static_cast<std::size_t>(columns + 1)) {
    setError(error, "CSC column_offsets length mismatch");
    return false;
  }
  if (column_offsets.front() != 0 ||
      column_offsets.back() != static_cast<int>(row_indices.size())) {
    setError(error, "CSC column_offsets bounds mismatch");
    return false;
  }
  for (int column = 0; column < columns; ++column) {
    const int begin = column_offsets[static_cast<std::size_t>(column)];
    const int end = column_offsets[static_cast<std::size_t>(column + 1)];
    if (begin < 0 || end < begin ||
        end > static_cast<int>(row_indices.size())) {
      setError(error, "CSC column range invalid");
      return false;
    }
    int previous_row = -1;
    for (int offset = begin; offset < end; ++offset) {
      const int row = row_indices[static_cast<std::size_t>(offset)];
      if (row < 0 || row >= rows || row <= previous_row) {
        setError(error, "CSC row indices must be strictly sorted and in range");
        return false;
      }
      previous_row = row;
    }
  }
  return true;
}

bool LtvQpSparseStructure::samePattern(
    const LtvQpSparseStructure &other) const {
  return rows == other.rows && columns == other.columns &&
         column_offsets == other.column_offsets &&
         row_indices == other.row_indices;
}

LtvQpDimensions checkedLtvQpDimensions(int horizon) {
  LtvQpDimensions dimensions;
  if (horizon <= 0) {
    dimensions.validation_error = "horizon must be positive";
    return dimensions;
  }
  if (horizon > kLtvQpMaximumHorizon) {
    dimensions.validation_error = "horizon exceeds audited LTV-QP maximum";
    return dimensions;
  }
  dimensions.horizon = horizon;
  dimensions.state_vars = 3 * (horizon + 1);
  dimensions.control_vars = 3 * horizon;
  dimensions.slack_vars = horizon;
  dimensions.decision_size =
      dimensions.state_vars + dimensions.control_vars + dimensions.slack_vars;

  dimensions.equality_rows = 3 * (horizon + 1);
  dimensions.body_accel_rows = 3 * horizon;
  dimensions.wheel_octagon_rows = 8 * horizon;
  dimensions.wheel_rate_rows = 0;
  dimensions.esdf_corridor_rows = horizon;
  dimensions.inequality_rows =
      dimensions.body_accel_rows + dimensions.wheel_octagon_rows +
      dimensions.wheel_rate_rows + dimensions.esdf_corridor_rows;
  dimensions.bound_rows = 4 * horizon;
  dimensions.constraint_rows =
      dimensions.equality_rows + dimensions.inequality_rows +
      dimensions.bound_rows;

  const std::size_t nnz_p = static_cast<std::size_t>(16 * horizon - 5);
  const std::size_t nnz_a = static_cast<std::size_t>(73 * horizon - 15);
  const std::size_t total_doubles =
      nnz_p + nnz_a + static_cast<std::size_t>(dimensions.decision_size) +
      2U * static_cast<std::size_t>(dimensions.constraint_rows);
  const std::size_t bytes = total_doubles * sizeof(double);
  if (bytes > kLtvQpSparseBufferMaximumBytes) {
    dimensions.validation_error = "sparse LTV-QP payload exceeds byte limit";
    return dimensions;
  }
  dimensions.sparse_buffer_bytes = bytes;
  dimensions.valid = true;
  dimensions.validation_error = "";
  return dimensions;
}

LtvQpSparseStructure LtvQpBuilder::buildHessianPattern(int horizon) {
  LtvQpSparseStructure pattern;
  const auto dims = checkedLtvQpDimensions(horizon);
  if (!dims.valid) {
    return pattern;
  }
  pattern.rows = dims.decision_size;
  pattern.columns = dims.decision_size;
  pattern.column_offsets.reserve(static_cast<std::size_t>(dims.decision_size + 1));
  pattern.row_indices.reserve(static_cast<std::size_t>(16 * horizon - 5));
  pattern.column_offsets.push_back(0);

  // State columns: 0 .. 3*(N+1)-1 (upper triangle of [xx xy; xy yy] + yaw)
  for (int step = 0; step <= horizon; ++step) {
    const int base = 3 * step;
    // dx_k
    pattern.row_indices.push_back(base + 0);
    pattern.column_offsets.push_back(static_cast<int>(pattern.row_indices.size()));
    // dy_k: (dx_k, dy_k) and (dy_k, dy_k)
    pattern.row_indices.push_back(base + 0);
    pattern.row_indices.push_back(base + 1);
    pattern.column_offsets.push_back(static_cast<int>(pattern.row_indices.size()));
    // dyaw_k
    pattern.row_indices.push_back(base + 2);
    pattern.column_offsets.push_back(static_cast<int>(pattern.row_indices.size()));
  }

  // Control columns: 3*(N+1) .. 6*N+2
  const int ctrl_start = dims.state_vars;
  for (int step = 0; step < horizon; ++step) {
    const int base = ctrl_start + 3 * step;
    for (int axis = 0; axis < 3; ++axis) {
      if (step > 0) {
        const int prev = base - 3;
        pattern.row_indices.push_back(prev + 0);
        pattern.row_indices.push_back(prev + 1);
        pattern.row_indices.push_back(prev + 2);
      }
      for (int r = 0; r <= axis; ++r) {
        pattern.row_indices.push_back(base + r);
      }
      pattern.column_offsets.push_back(
          static_cast<int>(pattern.row_indices.size()));
    }
  }

  // Slack columns: 6*N+3 .. 7*N+2
  const int slack_start = dims.state_vars + dims.control_vars;
  for (int step = 0; step < horizon; ++step) {
    pattern.row_indices.push_back(slack_start + step);
    pattern.column_offsets.push_back(
        static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

LtvQpSparseStructure LtvQpBuilder::buildConstraintPattern(int horizon) {
  LtvQpSparseStructure pattern;
  const auto dims = checkedLtvQpDimensions(horizon);
  if (!dims.valid) {
    return pattern;
  }
  const int eq_base = 0;
  const int accel_base = dims.equality_rows;
  const int wheel_oct_base = accel_base + dims.body_accel_rows;
  const int wheel_rate_base = wheel_oct_base + dims.wheel_octagon_rows;
  const int esdf_base = wheel_rate_base + dims.wheel_rate_rows;
  const int ctrl_bound_base = esdf_base + dims.esdf_corridor_rows;
  const int slack_bound_base = ctrl_bound_base + 3 * horizon;

  std::vector<std::vector<int>> rows_by_col(
      static_cast<std::size_t>(dims.decision_size));
  const auto add = [&rows_by_col](int row, int col) {
    rows_by_col[static_cast<std::size_t>(col)].push_back(row);
  };

  // Initial state equality: I_3 * dx_0
  for (int a = 0; a < 3; ++a) {
    add(eq_base + a, a);
  }

  const int ctrl_start = dims.state_vars;
  const int slack_start = dims.state_vars + dims.control_vars;

  for (int step = 0; step < horizon; ++step) {
    const int dyn_row = eq_base + 3 * (step + 1);
    const int x_curr = 3 * step;
    const int x_next = 3 * (step + 1);
    const int u_curr = ctrl_start + 3 * step;
    const int u_prev = ctrl_start + 3 * (step - 1);

    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        add(dyn_row + r, x_curr + c);
        add(dyn_row + r, u_curr + c);
        if (step > 0) {
          add(dyn_row + r, u_prev + c);
        }
      }
      add(dyn_row + c, x_next + c);
    }

    // Rotating-frame body acceleration inequality
    const int accel_row = accel_base + 3 * step;
    for (int a = 0; a < 3; ++a) {
      add(accel_row + a, u_curr + a);
      if (step > 0) {
        for (int c = 0; c < 3; ++c) {
          add(accel_row + a, u_prev + c);
        }
      }
    }

    // 8 exact outer-envelope octagonal wheel-speed rows on u_curr
    const int oct_row = wheel_oct_base + 8 * step;
    for (int r = 0; r < 8; ++r) {
      for (int c = 0; c < 3; ++c) {
        add(oct_row + r, u_curr + c);
      }
    }

    // ESDF corridor row on (dx_{k+1}, dy_{k+1}, s_{k+1})
    const int esdf_row = esdf_base + step;
    add(esdf_row, x_next + 0);
    add(esdf_row, x_next + 1);
    add(esdf_row, slack_start + step);

    // Control & slack bounds
    const int cb_row = ctrl_bound_base + 3 * step;
    for (int a = 0; a < 3; ++a) {
      add(cb_row + a, u_curr + a);
    }
    add(slack_bound_base + step, slack_start + step);
  }

  pattern.rows = dims.constraint_rows;
  pattern.columns = dims.decision_size;
  pattern.column_offsets.reserve(static_cast<std::size_t>(dims.decision_size + 1));
  pattern.row_indices.reserve(static_cast<std::size_t>(73 * horizon - 15));
  pattern.column_offsets.push_back(0);
  for (auto &rows : rows_by_col) {
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    pattern.row_indices.insert(pattern.row_indices.end(), rows.begin(),
                               rows.end());
    pattern.column_offsets.push_back(
        static_cast<int>(pattern.row_indices.size()));
  }
  return pattern;
}

bool LtvQpProblem::hasExpectedLayout() const {
  const auto dims = checkedLtvQpDimensions(horizon);
  if (!dims.valid || state_dimension != 3 || control_dimension != 3) {
    return false;
  }
  return hessian_structure.rows == dims.decision_size &&
         hessian_structure.columns == dims.decision_size &&
         hessian_values.size() == hessian_structure.row_indices.size() &&
         constraint_structure.rows == dims.constraint_rows &&
         constraint_structure.columns == dims.decision_size &&
         constraint_values.size() == constraint_structure.row_indices.size() &&
         gradient.size() == dims.decision_size &&
         lower.size() == dims.constraint_rows &&
         upper.size() == dims.constraint_rows &&
         slots.states.size() == static_cast<std::size_t>(horizon + 1) &&
         slots.controls.size() == static_cast<std::size_t>(horizon);
}

bool LtvQpProblem::hasOrderedBounds() const {
  if (!hasExpectedLayout()) {
    return false;
  }
  return (lower.array() <= upper.array()).all();
}

bool LtvQpProblem::hasFiniteNumerics() const {
  if (!hasExpectedLayout()) {
    return false;
  }
  const auto vecFinite = [](const std::vector<double> &v) {
    for (double x : v) {
      if (!std::isfinite(x)) {
        return false;
      }
    }
    return true;
  };
  if (!vecFinite(hessian_values) || !vecFinite(constraint_values) ||
      !gradient.allFinite()) {
    return false;
  }
  for (int i = 0; i < lower.size(); ++i) {
    if (std::isnan(lower(i)) || std::isnan(upper(i))) {
      return false;
    }
  }
  return true;
}

LtvQpProblem LtvQpBuilder::allocate(int horizon) {
  LtvQpProblem problem;
  const auto dims = checkedLtvQpDimensions(horizon);
  if (!dims.valid) {
    problem.validation_error = dims.validation_error;
    return problem;
  }
  problem.horizon = horizon;
  problem.hessian_structure = buildHessianPattern(horizon);
  problem.constraint_structure = buildConstraintPattern(horizon);
  if (!problem.hessian_structure.valid() ||
      !problem.constraint_structure.valid()) {
    problem.validation_error = "failed to build valid CSC pattern";
    return problem;
  }
  problem.hessian_values.assign(problem.hessian_structure.row_indices.size(),
                                0.0);
  problem.constraint_values.assign(
      problem.constraint_structure.row_indices.size(), 0.0);
  problem.gradient = Eigen::VectorXd::Zero(dims.decision_size);
  problem.lower = Eigen::VectorXd::Zero(dims.constraint_rows);
  problem.upper = Eigen::VectorXd::Zero(dims.constraint_rows);

  const int eq_base = 0;
  const int accel_base = dims.equality_rows;
  const int wheel_oct_base = accel_base + dims.body_accel_rows;
  const int wheel_rate_base = wheel_oct_base + dims.wheel_octagon_rows;
  const int esdf_base = wheel_rate_base + dims.wheel_rate_rows;
  const int ctrl_bound_base = esdf_base + dims.esdf_corridor_rows;
  const int slack_bound_base = ctrl_bound_base + 3 * horizon;
  const int ctrl_start = dims.state_vars;
  const int slack_start = dims.state_vars + dims.control_vars;

  problem.slots.states.resize(static_cast<std::size_t>(horizon + 1));
  for (int step = 0; step <= horizon; ++step) {
    auto &ss = problem.slots.states[static_cast<std::size_t>(step)];
    const int x_col = 3 * step;
    ss.p_xx = findCscSlot(problem.hessian_structure, x_col + 0, x_col + 0);
    ss.p_xy = findCscSlot(problem.hessian_structure, x_col + 0, x_col + 1);
    ss.p_yy = findCscSlot(problem.hessian_structure, x_col + 1, x_col + 1);
    ss.p_yaw = findCscSlot(problem.hessian_structure, x_col + 2, x_col + 2);
    if (step == 0) {
      for (int a = 0; a < 3; ++a) {
        ss.a_init_diag[static_cast<std::size_t>(a)] = findCscSlot(
            problem.constraint_structure, eq_base + a, x_col + a);
      }
    } else {
      const int prev_dyn = eq_base + 3 * step;
      for (int a = 0; a < 3; ++a) {
        ss.a_dyn_next_diag[static_cast<std::size_t>(a)] = findCscSlot(
            problem.constraint_structure, prev_dyn + a, x_col + a);
      }
      const int esdf_row = esdf_base + (step - 1);
      ss.a_esdf_x =
          findCscSlot(problem.constraint_structure, esdf_row, x_col + 0);
      ss.a_esdf_y =
          findCscSlot(problem.constraint_structure, esdf_row, x_col + 1);
    }
    if (step < horizon) {
      const int dyn_row = eq_base + 3 * (step + 1);
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          ss.a_dyn_state[static_cast<std::size_t>(r)]
                        [static_cast<std::size_t>(c)] = findCscSlot(
              problem.constraint_structure, dyn_row + r, x_col + c);
        }
      }
    }
  }

  problem.slots.controls.resize(static_cast<std::size_t>(horizon));
  for (int step = 0; step < horizon; ++step) {
    auto &cs = problem.slots.controls[static_cast<std::size_t>(step)];
    const int u_col = ctrl_start + 3 * step;
    const int s_col = slack_start + step;
    cs.p_u00 = findCscSlot(problem.hessian_structure, u_col + 0, u_col + 0);
    cs.p_u01 = findCscSlot(problem.hessian_structure, u_col + 0, u_col + 1);
    cs.p_u11 = findCscSlot(problem.hessian_structure, u_col + 1, u_col + 1);
    cs.p_u02 = findCscSlot(problem.hessian_structure, u_col + 0, u_col + 2);
    cs.p_u12 = findCscSlot(problem.hessian_structure, u_col + 1, u_col + 2);
    cs.p_u22 = findCscSlot(problem.hessian_structure, u_col + 2, u_col + 2);
    cs.p_slack = findCscSlot(problem.hessian_structure, s_col, s_col);
    if (step > 0) {
      const int u_prev = u_col - 3;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          cs.p_prev_curr[static_cast<std::size_t>(r)]
                        [static_cast<std::size_t>(c)] = findCscSlot(
              problem.hessian_structure, u_prev + r, u_col + c);
        }
      }
    }

    const int dyn_row = eq_base + 3 * (step + 1);
    const int accel_row = accel_base + 3 * step;
    const int oct_row = wheel_oct_base + 8 * step;
    const int esdf_row = esdf_base + step;
    const int cb_row = ctrl_bound_base + 3 * step;
    const int sb_row = slack_bound_base + step;

    for (int r = 0; r < 3; ++r) {
      cs.a_accel_curr_diag[static_cast<std::size_t>(r)] =
          findCscSlot(problem.constraint_structure, accel_row + r, u_col + r);
      cs.a_ctrl_bound_diag[static_cast<std::size_t>(r)] =
          findCscSlot(problem.constraint_structure, cb_row + r, u_col + r);
      for (int c = 0; c < 3; ++c) {
        cs.a_dyn_curr[static_cast<std::size_t>(r)]
                     [static_cast<std::size_t>(c)] =
            findCscSlot(problem.constraint_structure, dyn_row + r, u_col + c);
        if (step > 0) {
          const int u_prev = u_col - 3;
          cs.a_dyn_prev[static_cast<std::size_t>(r)]
                       [static_cast<std::size_t>(c)] = findCscSlot(
              problem.constraint_structure, dyn_row + r, u_prev + c);
          cs.a_accel_prev[static_cast<std::size_t>(r)]
                         [static_cast<std::size_t>(c)] = findCscSlot(
              problem.constraint_structure, accel_row + r, u_prev + c);
        }
      }
    }
    for (int r = 0; r < 8; ++r) {
      for (int c = 0; c < 3; ++c) {
        cs.a_wheel_oct[static_cast<std::size_t>(r)]
                      [static_cast<std::size_t>(c)] =
            findCscSlot(problem.constraint_structure, oct_row + r, u_col + c);
      }
    }
    cs.a_esdf_slack =
        findCscSlot(problem.constraint_structure, esdf_row, s_col);
    cs.a_slack_bound =
        findCscSlot(problem.constraint_structure, sb_row, s_col);

    // Pre-write invariant constant 1.0 coefficients once!
    for (int a = 0; a < 3; ++a) {
      problem.constraint_values[static_cast<std::size_t>(
          cs.a_accel_curr_diag[static_cast<std::size_t>(a)])] = 1.0;
      problem.constraint_values[static_cast<std::size_t>(
          cs.a_ctrl_bound_diag[static_cast<std::size_t>(a)])] = 1.0;
    }
    problem.constraint_values[static_cast<std::size_t>(cs.a_esdf_slack)] = 1.0;
    problem.constraint_values[static_cast<std::size_t>(cs.a_slack_bound)] = 1.0;
  }

  for (int step = 0; step <= horizon; ++step) {
    const auto &ss = problem.slots.states[static_cast<std::size_t>(step)];
    if (step == 0) {
      for (int a = 0; a < 3; ++a) {
        problem.constraint_values[static_cast<std::size_t>(
            ss.a_init_diag[static_cast<std::size_t>(a)])] = 1.0;
      }
    } else {
      for (int a = 0; a < 3; ++a) {
        problem.constraint_values[static_cast<std::size_t>(
            ss.a_dyn_next_diag[static_cast<std::size_t>(a)])] = 1.0;
      }
    }
  }
  return problem;
}

LtvQpProblem LtvQpBuilder::build(
    const State &current_state, const std::vector<State> &nominal_states,
    const std::vector<Control> &nominal_controls,
    const std::vector<Se2Reference> &references, const Control &last_control,
    const Se2MpcConfig &config, const ZeroSpeedGuardConfig &guard_config,
    double first_step_dt, const LtvQpEsdfConfig &esdf_config,
    const std::vector<EsdfCorridorSample> *esdf_samples) {
  LtvQpProblem problem = allocate(config.horizon);
  if (!problem.hasExpectedLayout()) {
    return problem;
  }
  build(current_state, nominal_states, nominal_controls, references,
        last_control, config, problem, guard_config, first_step_dt,
        esdf_config, esdf_samples);
  return problem;
}

bool LtvQpBuilder::build(
    const State &current_state, const std::vector<State> &nominal_states,
    const std::vector<Control> &nominal_controls,
    const std::vector<Se2Reference> &references, const Control &last_control,
    const Se2MpcConfig &config, LtvQpProblem &problem,
    const ZeroSpeedGuardConfig &guard_config, double first_step_dt,
    const LtvQpEsdfConfig &esdf_config,
    const std::vector<EsdfCorridorSample> *esdf_samples) {
  problem.valid = false;
  problem.validation_error.clear();
  problem.zero_speed_guard_active = false;
  problem.angle_rate_linearization_valid = {{false, false, false, false}};

  const auto dims = checkedLtvQpDimensions(config.horizon);
  if (!dims.valid || !std::isfinite(config.dt) || config.dt <= 0.0) {
    problem.validation_error =
        dims.valid ? "MPC dt must be finite and positive" : dims.validation_error;
    return false;
  }
  if (problem.horizon != config.horizon || !problem.hasExpectedLayout()) {
    problem.validation_error = "preallocated LTV-QP sparse layout mismatch";
    return false;
  }
  if (!validWeight(config.state_weight) ||
      !validWeight(config.terminal_weight) ||
      !validWeight(config.control_weight) ||
      !validWeight(config.control_delta_weight) ||
      !validNonnegativeLimit(config.max_vx) ||
      !validNonnegativeLimit(config.max_vy) ||
      !validNonnegativeLimit(config.max_wz) ||
      !validNonnegativeLimit(config.max_ax) ||
      !validNonnegativeLimit(config.max_ay) ||
      !validNonnegativeLimit(config.max_awz)) {
    problem.validation_error = "invalid weight or kinematic limit";
    return false;
  }
  const std::size_t horizon = static_cast<std::size_t>(config.horizon);
  if (nominal_controls.size() != horizon ||
      nominal_states.size() != horizon + 1 ||
      references.size() < horizon + 1) {
    problem.validation_error = "nominal or reference horizon size mismatch";
    return false;
  }
  if (!current_state.allFinite() || !last_control.allFinite() ||
      !finiteStates(nominal_states) || !finiteControls(nominal_controls)) {
    problem.validation_error = "non-finite nominal trajectory or state";
    return false;
  }
  for (std::size_t i = 0; i <= horizon; ++i) {
    if (!references[i].state.allFinite() || !references[i].control.allFinite()) {
      problem.validation_error = "non-finite reference";
      return false;
    }
  }

  const double effective_first_dt =
      (std::isfinite(first_step_dt) && first_step_dt > 0.0) ? first_step_dt
                                                            : config.dt;
  const Matrix3 r_mat = config.control_weight.asDiagonal();
  const Matrix3 rd_mat = config.control_delta_weight.asDiagonal();
  const auto wheel_m = wheelJacobians(config);
  const bool wheel_speed_active =
      config.wheel_base_x > 0.0 && config.wheel_base_y > 0.0 &&
      std::isfinite(config.max_wheel_speed) && config.max_wheel_speed > 1e-6;

  std::array<double, 4> oct_cos{};
  std::array<double, 4> oct_sin{};
  for (int m = 0; m < 4; ++m) {
    const double alpha = (kPi / 8.0) + static_cast<double>(m) * (kPi / 4.0);
    oct_cos[static_cast<std::size_t>(m)] = std::cos(alpha);
    oct_sin[static_cast<std::size_t>(m)] = std::sin(alpha);
  }
  const double oct_radius =
      wheel_speed_active ? config.max_wheel_speed * std::cos(kPi / 8.0) : 1.0;

  const int eq_base = 0;
  const int accel_base = dims.equality_rows;
  const int wheel_oct_base = accel_base + dims.body_accel_rows;
  const int wheel_rate_base = wheel_oct_base + dims.wheel_octagon_rows;
  const int esdf_base = wheel_rate_base + dims.wheel_rate_rows;
  const int ctrl_bound_base = esdf_base + dims.esdf_corridor_rows;
  const int slack_bound_base = ctrl_bound_base + 3 * config.horizon;
  const int ctrl_start = dims.state_vars;
  const int slack_start = dims.state_vars + dims.control_vars;

  // 1. State cost (with lag/contouring rotated weights) & initial equality
  for (int step = 0; step <= config.horizon; ++step) {
    const std::size_t idx = static_cast<std::size_t>(step);
    const auto &ss = problem.slots.states[idx];
    const Matrix3 q_mat = Se2MpcController::stateCostMatrix(
        config, references[idx], step == config.horizon);
    const State err =
        Se2Model::stateDifference(nominal_states[idx], references[idx].state);
    problem.hessian_values[static_cast<std::size_t>(ss.p_xx)] =
        2.0 * q_mat(0, 0);
    problem.hessian_values[static_cast<std::size_t>(ss.p_xy)] =
        2.0 * q_mat(0, 1);
    problem.hessian_values[static_cast<std::size_t>(ss.p_yy)] =
        2.0 * q_mat(1, 1);
    problem.hessian_values[static_cast<std::size_t>(ss.p_yaw)] =
        2.0 * q_mat(2, 2);
    problem.gradient.segment<3>(3 * step).noalias() = 2.0 * (q_mat * err);
  }

  const State init_res =
      Se2Model::stateDifference(current_state, nominal_states.front());
  problem.lower.segment<3>(eq_base) = init_res;
  problem.upper.segment<3>(eq_base) = init_res;

  // Pre-initialize control diagonal Hessian & gradient with control tracking cost
  std::vector<Matrix3> u_diag_hessian(
      horizon, 2.0 * (r_mat + rd_mat));
  for (int step = 0; step < config.horizon; ++step) {
    const std::size_t idx = static_cast<std::size_t>(step);
    problem.gradient.segment<3>(ctrl_start + 3 * step).noalias() =
        2.0 * (r_mat * (nominal_controls[idx] - references[idx].control));
  }

  const double slack_lin_w =
      std::isfinite(esdf_config.slack_linear_weight) &&
              esdf_config.slack_linear_weight >= 0.0
          ? esdf_config.slack_linear_weight
          : 400.0;
  const double slack_quad_w =
      std::isfinite(esdf_config.slack_quadratic_weight) &&
              esdf_config.slack_quadratic_weight > 0.0
          ? esdf_config.slack_quadratic_weight
          : 4000.0;
  const double max_slack =
      std::isfinite(esdf_config.max_slack_m) && esdf_config.max_slack_m >= 0.0
          ? esdf_config.max_slack_m
          : 0.0;

  ZeroSpeedGuard guard(guard_config);
  {
    std::array<double, 4> init_speeds{};
    for (std::size_t w = 0; w < 4; ++w) {
      init_speeds[w] = (wheel_m[w] * last_control).norm();
    }
    guard.update(init_speeds);
  }

  // 2. Per-step dynamics, rotating-frame delta cost & accel, octagon wheel disk, wheel rate, ESDF
  for (int step = 0; step < config.horizon; ++step) {
    const std::size_t idx = static_cast<std::size_t>(step);
    const auto &ss = problem.slots.states[idx];
    const auto &ss_next = problem.slots.states[idx + 1];
    const auto &cs = problem.slots.controls[idx];
    const double step_dt = step == 0 ? effective_first_dt : config.dt;
    const Control &u_nom = nominal_controls[idx];
    const Control &u_prev =
        step == 0 ? last_control : nominal_controls[idx - 1];

    // 2a. Linearized SE(2) dynamics with optional first-order actuator lag
    Matrix3 a_mat;
    Matrix3 b_curr;
    Matrix3 b_prev;
    Se2Model::jacobiansWithLag(nominal_states[idx], u_nom, u_prev, step_dt,
                               config.dt, config.actuator_lag_tau_sec, a_mat,
                               b_curr, b_prev);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        problem.constraint_values[static_cast<std::size_t>(
            ss.a_dyn_state[static_cast<std::size_t>(r)]
                          [static_cast<std::size_t>(c)])] = -a_mat(r, c);
        problem.constraint_values[static_cast<std::size_t>(
            cs.a_dyn_curr[static_cast<std::size_t>(r)]
                         [static_cast<std::size_t>(c)])] = -b_curr(r, c);
        if (step > 0) {
          problem.constraint_values[static_cast<std::size_t>(
              cs.a_dyn_prev[static_cast<std::size_t>(r)]
                           [static_cast<std::size_t>(c)])] = -b_prev(r, c);
        }
      }
    }
    const State f_nom = Se2Model::dynamicsWithLag(
        nominal_states[idx], u_nom, u_prev, step_dt, config.dt,
        config.actuator_lag_tau_sec);
    const State dyn_res =
        Se2Model::stateDifference(f_nom, nominal_states[idx + 1]);
    problem.lower.segment<3>(eq_base + 3 * (step + 1)) = dyn_res;
    problem.upper.segment<3>(eq_base + 3 * (step + 1)) = dyn_res;

    // 2b. Rotating-frame velocity transport R(-wz*dt) for delta cost & acceleration inequality
    const Control carried_prev =
        Se2Model::rotateControlLinear(u_prev, u_prev(2) * step_dt);
    const Control nom_delta = u_nom - carried_prev;
    problem.gradient.segment<3>(ctrl_start + 3 * step).noalias() +=
        2.0 * (rd_mat * nom_delta);

    if (step > 0) {
      const Matrix3 j_rot =
          Se2Model::rotateControlLinearJacobian(u_prev, step_dt);
      const Matrix3 jt_rd = j_rot.transpose() * rd_mat;
      u_diag_hessian[idx - 1].noalias() += 2.0 * (jt_rd * j_rot);
      problem.gradient.segment<3>(ctrl_start + 3 * (step - 1)).noalias() -=
          2.0 * (jt_rd * nom_delta);
      const Matrix3 super_block = -2.0 * jt_rd;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          problem.hessian_values[static_cast<std::size_t>(
              cs.p_prev_curr[static_cast<std::size_t>(r)]
                            [static_cast<std::size_t>(c)])] = super_block(r, c);
        }
      }
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          problem.constraint_values[static_cast<std::size_t>(
              cs.a_accel_prev[static_cast<std::size_t>(r)]
                             [static_cast<std::size_t>(c)])] = -j_rot(r, c);
        }
      }
    }

    const double accel_tighten = step == 0 ? 1.0 : 0.996;
    const Control max_delta(config.max_ax * step_dt * accel_tighten,
                            config.max_ay * step_dt * accel_tighten,
                            config.max_awz * step_dt * accel_tighten);
    const int accel_row = accel_base + 3 * step;
    for (int a = 0; a < 3; ++a) {
      problem.lower(accel_row + a) =
          std::min(0.0, -max_delta(a) - nom_delta(a));
      problem.upper(accel_row + a) =
          std::max(0.0, max_delta(a) - nom_delta(a));
    }

    // 2c. Exact 8-row symmetric four-wheel octagon outer envelope on u_k
    const int oct_row = wheel_oct_base + 8 * step;
    const double half_x = std::max(0.0, config.wheel_base_x);
    const double half_y = std::max(0.0, config.wheel_base_y);
    for (int m = 0; m < 4; ++m) {
      const double c_n = oct_cos[static_cast<std::size_t>(m)];
      const double s_n = oct_sin[static_cast<std::size_t>(m)];
      const double gamma_max = std::abs(c_n) * half_y + std::abs(s_n) * half_x;
      for (int sgn_idx = 0; sgn_idx < 2; ++sgn_idx) {
        const int r_idx = 2 * m + sgn_idx;
        if (wheel_speed_active) {
          const double gamma = sgn_idx == 0 ? gamma_max : -gamma_max;
          const Eigen::RowVector3d h(c_n, s_n, gamma);
          for (int c = 0; c < 3; ++c) {
            problem.constraint_values[static_cast<std::size_t>(
                cs.a_wheel_oct[static_cast<std::size_t>(r_idx)]
                              [static_cast<std::size_t>(c)])] = h(c);
          }
          const double w_nom = h.dot(u_nom);
          problem.lower(oct_row + r_idx) = std::min(0.0, -oct_radius - w_nom);
          problem.upper(oct_row + r_idx) = std::max(0.0, oct_radius - w_nom);
        } else {
          for (int c = 0; c < 3; ++c) {
            problem.constraint_values[static_cast<std::size_t>(
                cs.a_wheel_oct[static_cast<std::size_t>(r_idx)]
                              [static_cast<std::size_t>(c)])] = 0.0;
          }
          problem.lower(oct_row + r_idx) = -1.0;
          problem.upper(oct_row + r_idx) = 1.0;
        }
      }
    }

    // 2d. ZeroSpeedGuard state tracking (exact nonlinear four-wheel rolling-accel
    //     & steer-rate constraints are enforced during primal projection & audit,
    //     avoiding LICQ rank deficiency against rotating-frame body_accel rows).
    std::array<double, 4> curr_speeds{};
    for (int w = 0; w < 4; ++w) {
      const auto &mw = wheel_m[static_cast<std::size_t>(w)];
      const double s_curr = (mw * u_nom).norm();
      const double s_prev = (mw * u_prev).norm();
      curr_speeds[static_cast<std::size_t>(w)] = s_curr;
      problem.angle_rate_linearization_valid[static_cast<std::size_t>(w)] =
          guard.angleConstraintDefined(s_prev, s_curr);
    }
    problem.zero_speed_guard_active =
        problem.zero_speed_guard_active || guard.update(curr_speeds);

    // 2e. Linearized ESDF safety corridor on state x_{step+1} with non-negative slack
    const int esdf_row = esdf_base + step;
    const bool sample_active =
        esdf_config.enabled && esdf_samples != nullptr &&
        idx < esdf_samples->size() && (*esdf_samples)[idx].active &&
        (*esdf_samples)[idx].gradient_odom.allFinite() &&
        (*esdf_samples)[idx].gradient_odom.norm() > 1e-3 &&
        max_slack > 1e-6;
    if (sample_active) {
      problem.hessian_values[static_cast<std::size_t>(cs.p_slack)] =
          2.0 * slack_quad_w;
      problem.gradient(slack_start + step) = slack_lin_w;
      const auto &sample = (*esdf_samples)[idx];
      const Eigen::Vector2d grad = sample.gradient_odom.normalized();
      problem.constraint_values[static_cast<std::size_t>(ss_next.a_esdf_x)] =
          grad(0);
      problem.constraint_values[static_cast<std::size_t>(ss_next.a_esdf_y)] =
          grad(1);
      // Clamp required deficit to max_slack so z=(0,0,s) is strictly primal-feasible!
      const double required_gain = std::clamp(
          sample.target_clearance_m - sample.signed_distance_m,
          -kEsdfUpperBound, max_slack);
      problem.lower(esdf_row) = required_gain;
      problem.upper(esdf_row) = kEsdfUpperBound;
    } else {
      // Inactive slack gets O(1) curvature and zero gradient so it never skews
      // Ruiz cost scaling (inf_norm_q / col_norm_inf(P)) or dual multipliers.
      problem.hessian_values[static_cast<std::size_t>(cs.p_slack)] = 2.0;
      problem.gradient(slack_start + step) = 0.0;
      problem.constraint_values[static_cast<std::size_t>(ss_next.a_esdf_x)] =
          0.0;
      problem.constraint_values[static_cast<std::size_t>(ss_next.a_esdf_y)] =
          0.0;
      problem.lower(esdf_row) = -1.0;
      problem.upper(esdf_row) = kEsdfUpperBound;
    }

    // 2f. Control & slack variable box bounds
    const int cb_row = ctrl_bound_base + 3 * step;
    problem.lower.segment<3>(cb_row) =
        Control(-config.max_vx, -config.max_vy, -config.max_wz) - u_nom;
    problem.upper.segment<3>(cb_row) =
        Control(config.max_vx, config.max_vy, config.max_wz) - u_nom;
    for (int a = 0; a < 3; ++a) {
      problem.lower(cb_row + a) = std::min(0.0, problem.lower(cb_row + a));
      problem.upper(cb_row + a) = std::max(0.0, problem.upper(cb_row + a));
    }
    problem.lower(slack_bound_base + step) = 0.0;
    problem.upper(slack_bound_base + step) = max_slack;
  }

  // Write accumulated 3x3 upper-triangular control Hessian blocks
  for (int step = 0; step < config.horizon; ++step) {
    const std::size_t idx = static_cast<std::size_t>(step);
    const auto &cs = problem.slots.controls[idx];
    const Matrix3 &hu = u_diag_hessian[idx];
    problem.hessian_values[static_cast<std::size_t>(cs.p_u00)] = hu(0, 0);
    problem.hessian_values[static_cast<std::size_t>(cs.p_u01)] = hu(0, 1);
    problem.hessian_values[static_cast<std::size_t>(cs.p_u11)] = hu(1, 1);
    problem.hessian_values[static_cast<std::size_t>(cs.p_u02)] = hu(0, 2);
    problem.hessian_values[static_cast<std::size_t>(cs.p_u12)] = hu(1, 2);
    problem.hessian_values[static_cast<std::size_t>(cs.p_u22)] = hu(2, 2);
  }

  problem.valid = problem.hasOrderedBounds() && problem.hasFiniteNumerics();
  if (!problem.valid) {
    problem.validation_error = "constructed sparse QP contains invalid numerics";
  }
  return problem.valid;
}

}  // namespace ats_swerve_mpc
