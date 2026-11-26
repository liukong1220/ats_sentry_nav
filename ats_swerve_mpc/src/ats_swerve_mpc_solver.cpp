#include "ats_swerve_mpc_node_helpers.hpp"

namespace ats_swerve_mpc {

bool AtsSwerveMpcNode::planningSnapshotUsableLocked(
    std::chrono::steady_clock::time_point now_steady,
    std::shared_ptr<const ats_navigation_interfaces::msg::PlanningMapSnapshot>
        *out_snapshot,
    Eigen::Vector3d *out_snapshot_from_odom) const {
  if (out_snapshot != nullptr) {
    out_snapshot->reset();
  }
  if (out_snapshot_from_odom != nullptr) {
    out_snapshot_from_odom->setZero();
  }
  if (!planning_snapshot_ || !planning_snapshot_ready_ ||
      !last_planning_snapshot_signal_ ||
      now_steady < *last_planning_snapshot_signal_) {
    return !require_planning_snapshot_;
  }
  const double age = std::chrono::duration<double>(
                         now_steady - *last_planning_snapshot_signal_)
                         .count();
  if (age > planning_snapshot_timeout_sec_) {
    return !require_planning_snapshot_;
  }
  if (active_execution_command_) {
    if (planning_snapshot_->localization_epoch !=
        active_execution_command_->localization_epoch) {
      return !require_planning_snapshot_;
    }
  } else if (require_localization_status_ && localization_epoch_) {
    if (planning_snapshot_->localization_epoch != *localization_epoch_) {
      return !require_planning_snapshot_;
    }
  }

  Eigen::Vector3d snap_from_odom = Eigen::Vector3d::Zero();
  const std::string &snap_frame = planning_snapshot_->header.frame_id;
  const std::string &ref_frame =
      trajectory_frame_.empty() ? frame_id_ : trajectory_frame_;
  if (snap_frame != ref_frame) {
    if (!active_execution_command_) {
      return !require_planning_snapshot_;
    }
    const auto &tf_msg = active_execution_command_->validation_from_reference;
    if (tf_msg.header.frame_id != snap_frame ||
        (!tf_msg.child_frame_id.empty() && tf_msg.child_frame_id != ref_frame)) {
      return !require_planning_snapshot_;
    }
    const auto &q = tf_msg.transform.rotation;
    const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(q_norm) || std::abs(q_norm - 1.0) > 1e-2 ||
        !std::isfinite(tf_msg.transform.translation.x) ||
        !std::isfinite(tf_msg.transform.translation.y)) {
      return !require_planning_snapshot_;
    }
    snap_from_odom(0) = tf_msg.transform.translation.x;
    snap_from_odom(1) = tf_msg.transform.translation.y;
    snap_from_odom(2) = tf2::getYaw(q);
  }

  if (out_snapshot != nullptr) {
    *out_snapshot = planning_snapshot_;
  }
  if (out_snapshot_from_odom != nullptr) {
    *out_snapshot_from_odom = snap_from_odom;
  }
  return true;
}

std::vector<EsdfCorridorSample> AtsSwerveMpcNode::extractEsdfCorridorSamples(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    const std::vector<State> &nominal_states,
    const std::vector<Se2Reference> &references) const {
  const int horizon = controller_->config().horizon;
  std::vector<EsdfCorridorSample> samples(static_cast<std::size_t>(horizon));
  if (!qp_esdf_config_.enabled ||
      nominal_states.size() < static_cast<std::size_t>(horizon + 1) ||
      references.size() < static_cast<std::size_t>(horizon + 1)) {
    return samples;
  }
  const double ct = std::cos(snapshot_from_odom(2));
  const double st = std::sin(snapshot_from_odom(2));
  for (int step = 0; step < horizon; ++step) {
    const std::size_t k = static_cast<std::size_t>(step + 1);
    const State &nom = nominal_states[k];
    const State &ref = references[k].state;
    const double nom_sx = ct * nom(0) - st * nom(1) + snapshot_from_odom(0);
    const double nom_sy = st * nom(0) + ct * nom(1) + snapshot_from_odom(1);
    double d_nom = 0.0;
    Eigen::Vector2d grad_snap = Eigen::Vector2d::Zero();
    if (!sampleSnapshotEsdfBilinear(snapshot, nom_sx, nom_sy, d_nom,
                                    &grad_snap)) {
      continue;
    }
    const double ref_sx = ct * ref(0) - st * ref(1) + snapshot_from_odom(0);
    const double ref_sy = st * ref(0) + ct * ref(1) + snapshot_from_odom(1);
    double d_ref = qp_esdf_config_.corridor_clearance_m;
    sampleSnapshotEsdfBilinear(snapshot, ref_sx, ref_sy, d_ref, nullptr);
    const double target = std::clamp(
        std::min(qp_esdf_config_.corridor_clearance_m,
                 d_ref - qp_esdf_config_.reference_margin_m),
        qp_esdf_config_.min_corridor_clearance_m,
        qp_esdf_config_.corridor_clearance_m);
    if (d_nom > target + 0.25 || grad_snap.norm() <= 1e-3) {
      continue;
    }
    auto &sample = samples[static_cast<std::size_t>(step)];
    sample.active = true;
    sample.signed_distance_m = d_nom;
    sample.target_clearance_m = target;
    sample.gradient_odom(0) = ct * grad_snap(0) + st * grad_snap(1);
    sample.gradient_odom(1) = -st * grad_snap(0) + ct * grad_snap(1);
  }
  return samples;
}

AtsSwerveMpcNode::RolloutSafetyEvaluation
AtsSwerveMpcNode::evaluateRolloutFootprintSafety(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot &snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    const std::vector<State> &rollout_states) const {
  RolloutSafetyEvaluation eval;
  if (!rollout_safety_enabled_ || rollout_states.size() < 2U) {
    return eval;
  }
  const double res = static_cast<double>(snapshot.info.resolution);
  const double hx = 0.5 * footprint_length_ + footprint_safety_margin_;
  const double hy = 0.5 * footprint_width_ + footprint_safety_margin_;
  const double r_circum = std::hypot(hx, hy);
  const double ct = std::cos(snapshot_from_odom(2));
  const double st = std::sin(snapshot_from_odom(2));
  const double dt = controller_->config().dt;

  const auto evalPoseClearance = [&](const State &state_odom,
                                     bool &center_free) -> double {
    const double sx =
        ct * state_odom(0) - st * state_odom(1) + snapshot_from_odom(0);
    const double sy =
        st * state_odom(0) + ct * state_odom(1) + snapshot_from_odom(1);
    const double syaw = state_odom(2) + snapshot_from_odom(2);
    center_free = snapshotCenterCellFree(snapshot, sx, sy);
    double d_center = 0.0;
    sampleSnapshotEsdfBilinear(snapshot, sx, sy, d_center, nullptr);
    const double lb = d_center - r_circum - 0.5 * res;
    if (center_free && lb >= rollout_slowdown_clearance_m_ + 0.01) {
      return lb;
    }
    const double cy = std::cos(syaw);
    const double sy_s = std::sin(syaw);
    double min_d = std::numeric_limits<double>::infinity();
    constexpr int kEdgeSamples = 3;
    for (int side = -1; side <= 1; side += 2) {
      for (int i = 0; i <= kEdgeSamples; ++i) {
        const double u = -1.0 + 2.0 * static_cast<double>(i) /
                                    static_cast<double>(kEdgeSamples);
        const std::array<Eigen::Vector2d, 2> pts{{
            Eigen::Vector2d(side * hx, u * hy),
            Eigen::Vector2d(u * hx, side * hy)}};
        for (const auto &local_p : pts) {
          const double px = sx + cy * local_p(0) - sy_s * local_p(1);
          const double py = sy + sy_s * local_p(0) + cy * local_p(1);
          double dp = 0.0;
          sampleSnapshotEsdfBilinear(snapshot, px, py, dp, nullptr);
          min_d = std::min(min_d, dp - 0.5 * res);
        }
      }
    }
    return min_d;
  };

  bool init_center_free = true;
  const double init_clearance =
      evalPoseClearance(rollout_states.front(), init_center_free);
  const int max_steps = std::min(
      static_cast<int>(rollout_states.size()) - 1,
      std::max(
          1, static_cast<int>(std::ceil(rollout_slowdown_horizon_sec_ /
                                        std::max(1e-3, dt)))));

  for (int step = 1; step <= max_steps; ++step) {
    const double t_sec = static_cast<double>(step) * dt;
    bool center_free = true;
    const double clearance = evalPoseClearance(
        rollout_states[static_cast<std::size_t>(step)], center_free);
    eval.min_footprint_clearance_m =
        std::min(eval.min_footprint_clearance_m, clearance);
    const bool worsening = clearance < init_clearance - 0.005;

    if (t_sec <= rollout_immediate_stop_horizon_sec_ + 1e-6) {
      if (!center_free ||
          (clearance < -rollout_hard_stop_penetration_m_ && worsening)) {
        eval.level = RolloutSafetyLevel::kUnsafeStop;
        eval.speed_scale = 0.0;
        return eval;
      }
    }
    if (t_sec <= rollout_brake_horizon_sec_ + 1e-6 && clearance < -0.005 &&
        (init_clearance >= -0.005 || worsening)) {
      eval.level = RolloutSafetyLevel::kBrake;
      const double ratio = std::clamp(
          t_sec / std::max(0.1, rollout_brake_horizon_sec_), 0.20, 0.60);
      eval.speed_scale = std::min(eval.speed_scale, ratio);
    } else if (clearance < rollout_slowdown_clearance_m_ &&
               eval.level != RolloutSafetyLevel::kBrake) {
      eval.level = RolloutSafetyLevel::kSlowdown;
      const double ratio =
          rollout_slowdown_clearance_m_ > 1e-6
              ? std::clamp(clearance / rollout_slowdown_clearance_m_, 0.0, 1.0)
              : 1.0;
      const double scale =
          rollout_min_slowdown_scale_ +
          (1.0 - rollout_min_slowdown_scale_) * ratio;
      eval.speed_scale = std::min(eval.speed_scale, scale);
    }
  }
  return eval;
}

State AtsSwerveMpcNode::advanceStateThroughCommandDelay(
    const State &measured_state, double now_sec, double rotation_delay) const {
  const double delay_sec = controller_->config().actuator_delay_sec;
  if (!std::isfinite(delay_sec) || delay_sec <= 1e-4) {
    return rotationPhaseCompensatedState(measured_state, last_control_(2),
                                         rotation_delay);
  }
  std::deque<CommandHistorySample> history_copy;
  {
    std::lock_guard<std::mutex> lock(command_history_mutex_);
    history_copy = command_delay_history_;
  }
  if (history_copy.empty()) {
    return Se2Model::dynamics(measured_state, last_control_, delay_sec);
  }
  const double window_start = now_sec - delay_sec;
  State predicted = measured_state;
  double cursor = window_start;
  Control active_u = history_copy.front().control;
  for (const auto &entry : history_copy) {
    if (entry.stamp_sec <= window_start) {
      active_u = entry.control;
      continue;
    }
    if (entry.stamp_sec > now_sec) {
      break;
    }
    const double seg_dt = entry.stamp_sec - cursor;
    if (seg_dt > 1e-6) {
      predicted = Se2Model::dynamics(predicted, active_u, seg_dt);
      cursor = entry.stamp_sec;
    }
    active_u = entry.control;
  }
  if (now_sec > cursor + 1e-6) {
    predicted = Se2Model::dynamics(predicted, active_u, now_sec - cursor);
  }
  return predicted;
}

bool AtsSwerveMpcNode::solveSqpRti(
    const ControlCycleSnapshot &snapshot,
    const ats_navigation_interfaces::msg::PlanningMapSnapshot *map_snapshot,
    const Eigen::Vector3d &snapshot_from_odom,
    ControlCycleTelemetrySample &telemetry, Se2MpcResult &out_result,
    RolloutSafetyEvaluation &out_rollout_safety) {
  const auto sqp_start = std::chrono::steady_clock::now();
  if (!qp_solver_ || !qp_solver_->initialized()) {
    telemetry.status = LtvQpSolverStatus::kBackendUnavailable;
    std::snprintf(telemetry.rejection_reason.data(),
                  telemetry.rejection_reason.size(), "%s",
                  "backend_unavailable");
    return false;
  }

  bool first_body_sat = false;
  bool first_wheel_sat = false;
  bool first_inc_lim = false;
  std::vector<Control> nominal_controls = controller_->buildNominalControls(
      snapshot.references, snapshot.last_control_before_solve,
      snapshot.first_step_dt, true, &first_body_sat, &first_wheel_sat,
      &first_inc_lim);
  std::vector<State> nominal_states = controller_->rolloutTrajectory(
      snapshot.current_state, nominal_controls,
      snapshot.last_control_before_solve, snapshot.first_step_dt);

  LtvQpSolveResult qp_res;
  LtvQpPrimalCandidate candidate;
  LtvQpCandidateAudit audit;
  RolloutSafetyEvaluation rollout_safety;
  double total_build_ms = 0.0;
  double total_update_ms = 0.0;
  double total_backend_ms = 0.0;
  double total_complete_ms = 0.0;
  double total_solve_ms = 0.0;
  double total_recon_ms = 0.0;
  double total_audit_ms = 0.0;
  int executed_rti = 0;

  for (int rti_pass = 0; rti_pass < qp_rti_iterations_; ++rti_pass) {
    const auto build_start = std::chrono::steady_clock::now();
    std::vector<EsdfCorridorSample> esdf_samples;
    if (map_snapshot != nullptr && qp_esdf_config_.enabled) {
      esdf_samples = extractEsdfCorridorSamples(
          *map_snapshot, snapshot_from_odom, nominal_states,
          snapshot.references);
    }
    const bool built = LtvQpBuilder::build(
        snapshot.current_state, nominal_states, nominal_controls,
        snapshot.references, snapshot.last_control_before_solve,
        controller_->config(), qp_problem_buffer_, ZeroSpeedGuardConfig(),
        snapshot.first_step_dt, qp_esdf_config_,
        esdf_samples.empty() ? nullptr : &esdf_samples);
    total_build_ms += elapsedMilliseconds(build_start,
                                          std::chrono::steady_clock::now());
    if (!built) {
      qp_res.status = LtvQpSolverStatus::kInvalidProblem;
      break;
    }
    if (rti_pass == 0) {
      recordQpProblemMetrics(qp_problem_buffer_, telemetry);
    }

    LtvQpWarmStart rti_warm;
    const LtvQpWarmStart *warm_ptr = nullptr;
    if (qp_warm_start_valid_ &&
        qp_warm_start_.dual.size() == qp_problem_buffer_.constraintRows()) {
      const int horizon = qp_problem_buffer_.horizon;
      const auto dims = checkedLtvQpDimensions(horizon);
      rti_warm.primal = Eigen::VectorXd::Zero(qp_problem_buffer_.decisionSize());
      const int esdf_base = dims.equality_rows + dims.body_accel_rows +
                            dims.wheel_octagon_rows + dims.wheel_rate_rows;
      for (int s = 0; s < horizon; ++s) {
        rti_warm.primal(qp_problem_buffer_.slackOffset(s)) =
            std::max(0.0, qp_problem_buffer_.lower(esdf_base + s));
      }
      if (rti_pass == 0 && horizon > 1) {
        // Fractionally shift dual blocks forward by first_step_dt / dt to match
        // the receding horizon window at 50 Hz (consistent with initializeControls).
        const double shift = std::clamp(
            snapshot.first_step_dt / std::max(1e-6, controller_->config().dt),
            0.0, 1.0);
        rti_warm.dual = Eigen::VectorXd::Zero(dims.constraint_rows);
        const auto shiftBlock = [&](int base, int stride, int count) {
          for (int k = 0; k + 1 < count; ++k) {
            rti_warm.dual.segment(base + k * stride, stride) =
                (1.0 - shift) * qp_warm_start_.dual.segment(base + k * stride, stride) +
                shift * qp_warm_start_.dual.segment(base + (k + 1) * stride, stride);
          }
          rti_warm.dual.segment(base + (count - 1) * stride, stride) =
              qp_warm_start_.dual.segment(base + (count - 1) * stride, stride);
        };
        rti_warm.dual.segment<3>(0) = qp_warm_start_.dual.segment<3>(0);
        shiftBlock(3, 3, horizon);
        int cursor = dims.equality_rows;
        shiftBlock(cursor, 3, horizon);
        cursor += dims.body_accel_rows;
        shiftBlock(cursor, 8, horizon);
        cursor += dims.wheel_octagon_rows + dims.wheel_rate_rows;
        shiftBlock(cursor, 1, horizon);
        cursor += dims.esdf_corridor_rows;
        shiftBlock(cursor, 3, horizon);
        cursor += 3 * horizon;
        shiftBlock(cursor, 1, horizon);
      } else {
        rti_warm.dual = qp_warm_start_.dual;
      }
      for (int s = 0; s < horizon; ++s) {
        if (qp_problem_buffer_.lower(esdf_base + s) < -0.5) {
          rti_warm.dual(esdf_base + s) = 0.0;
        }
      }
      warm_ptr = &rti_warm;
    }

    qp_res = qp_solver_->solveLtvProblem(qp_problem_buffer_,
                                         qp_solver_settings_, warm_ptr);
    ++executed_rti;
    total_update_ms += qp_res.wall_update_time_ms;
    total_backend_ms += qp_res.wall_qp_phase_time_ms;
    total_complete_ms += qp_res.wall_complete_qp_phase_time_ms;
    total_solve_ms += qp_res.wall_solve_time_ms;

    if ((qp_res.status != LtvQpSolverStatus::kSolved &&
         qp_res.status != LtvQpSolverStatus::kSolvedInaccurate) ||
        qp_res.primal_solution.size() != qp_problem_buffer_.decisionSize() ||
        qp_res.dual_solution.size() != qp_problem_buffer_.constraintRows()) {
      break;
    }
    qp_warm_start_.primal = qp_res.primal_solution;
    qp_warm_start_.dual = qp_res.dual_solution;
    qp_warm_start_valid_ = true;

    const auto recon_start = std::chrono::steady_clock::now();
    candidate = LtvQpCandidateReconstructor::reconstruct(
        snapshot.current_state, qp_problem_buffer_, nominal_controls,
        controller_->config(), qp_res, snapshot.last_control_before_solve,
        snapshot.first_step_dt);
    total_recon_ms += elapsedMilliseconds(recon_start,
                                          std::chrono::steady_clock::now());
    if (!candidate.valid) {
      break;
    }

    double max_delta_u = 0.0;
    for (int step = 0; step < qp_problem_buffer_.horizon; ++step) {
      max_delta_u = std::max(
          max_delta_u,
          qp_res.primal_solution
              .segment<3>(qp_problem_buffer_.controlOffset(step))
              .lpNorm<Eigen::Infinity>());
    }
    const double elapsed_sqp_ms =
        elapsedMilliseconds(sqp_start, std::chrono::steady_clock::now());
    if (rti_pass + 1 < qp_rti_iterations_ &&
        max_delta_u > qp_rti_second_pass_delta_threshold_ &&
        elapsed_sqp_ms < 0.45 * qp_solver_settings_.time_limit_ms) {
      Control prev_u = snapshot.last_control_before_solve;
      for (int step = 0; step < qp_problem_buffer_.horizon; ++step) {
        const std::size_t idx = static_cast<std::size_t>(step);
        const double sdt =
            step == 0 ? snapshot.first_step_dt : controller_->config().dt;
        nominal_controls[idx] = controller_->clampStepIncrement(
            candidate.controls[idx], prev_u, sdt);
        prev_u = nominal_controls[idx];
      }
      nominal_states = controller_->rolloutTrajectory(
          snapshot.current_state, nominal_controls,
          snapshot.last_control_before_solve, snapshot.first_step_dt);
      continue;
    }
    break;
  }

  recordTiming(telemetry, ControlCycleTimingStage::kQpProblemBuild,
               total_build_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kOsqpNumericUpdate,
               total_update_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kQpBackendPhase,
               total_backend_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kQpCompletePhase,
               total_complete_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kOsqpSolve, total_solve_ms);
  recordTiming(telemetry, ControlCycleTimingStage::kPrimalReconstructionRollout,
               total_recon_ms);

  if (candidate.valid && map_snapshot != nullptr) {
    rollout_safety = evaluateRolloutFootprintSafety(
        *map_snapshot, snapshot_from_odom, candidate.states);
  }

  LtvQpCandidateSafety safety;
  safety.inputs_healthy = snapshot.localization_fresh &&
                          snapshot.reference_fresh &&
                          snapshot.execution_lease_valid &&
                          snapshot.gimbal_valid;
  safety.emergency_stop_active = snapshot.emergency_stop_active;
  safety.collision_free =
      rollout_safety.level != RolloutSafetyLevel::kUnsafeStop;
  safety.localization_fresh = snapshot.localization_fresh;
  safety.reference_fresh = snapshot.reference_fresh;
  safety.execution_lease_valid = snapshot.execution_lease_valid;
  safety.gimbal_valid = snapshot.gimbal_valid;
  safety.map_fresh = snapshot.map_fresh;

  LtvQpSolverSettings audit_settings = qp_solver_settings_;
  audit_settings.time_limit_ms =
      qp_solver_settings_.time_limit_ms * std::max(1, executed_rti);
  const auto audit_start = std::chrono::steady_clock::now();
  if (candidate.valid) {
    audit = LtvQpCandidateValidator::validate(
        qp_problem_buffer_, nominal_controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), audit_settings, safety, qp_res, candidate,
        snapshot.first_step_dt);
  } else {
    audit = LtvQpCandidateValidator::validate(
        qp_problem_buffer_, nominal_controls,
        snapshot.last_control_before_solve, controller_->config(),
        ZeroSpeedGuardConfig(), audit_settings, safety, qp_res,
        snapshot.first_step_dt);
  }
  total_audit_ms =
      elapsedMilliseconds(audit_start, std::chrono::steady_clock::now());
  recordTiming(telemetry, ControlCycleTimingStage::kCandidateHardCheck,
               total_audit_ms);

  telemetry.qp_attempted = true;
  telemetry.rti_iterations = executed_rti;
  telemetry.same_snapshot_identity =
      snapshot.identity_digest_valid &&
      snapshot.identity_digest == controlCycleSnapshotIdentityDigest(snapshot);
  telemetry.status = qp_res.status;
  telemetry.iterations = qp_res.iterations;
  telemetry.warm_start_used = qp_res.warm_start_used;
  telemetry.osqp_reported_update_ms = qp_res.update_time_ms;
  telemetry.osqp_reported_solve_ms = qp_res.solve_time_ms;
  telemetry.osqp_wall_update_ms = total_update_ms;
  telemetry.osqp_wall_solve_ms = total_solve_ms;
  telemetry.qp_complete_phase_ms = total_complete_ms;
  telemetry.osqp_wall_qp_phase_ms = total_backend_ms;
  telemetry.primal_residual = qp_res.primal_residual;
  telemetry.dual_residual = qp_res.dual_residual;
  telemetry.hard_constraint_margin =
      qp_solver_settings_.max_hard_constraint_violation -
      std::max(qp_res.hard_constraint_maximum_violation,
               audit.actual_hard_constraint_maximum_violation);
  telemetry.slack_maximum =
      std::max(qp_res.slack_maximum, audit.actual_slack_maximum);
  telemetry.candidate_feasible = audit.feasible;
  const char *reason =
      audit.rejection_reason.empty() ? "none" : audit.rejection_reason.c_str();
  std::snprintf(telemetry.rejection_reason.data(),
                telemetry.rejection_reason.size(), "%s", reason);
  telemetry.collision_gate = safety.collision_free;
  telemetry.map_gate = safety.map_fresh;

  if (!audit.feasible || !candidate.valid || candidate.controls.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "【QP诊断】status=%s iter=%d prim=%.2e dual=%.2e rho=%.3e rho_upd=%d "
        "slack=%.3f hard_viol=%.2e wall_solve=%.2fms reason=%s",
        ltvQpSolverStatusName(qp_res.status), qp_res.iterations,
        qp_res.primal_residual, qp_res.dual_residual, qp_res.rho,
        qp_res.rho_updates, telemetry.slack_maximum,
        audit.actual_hard_constraint_maximum_violation, total_solve_ms, reason);
    const bool keep_warm_start =
        (qp_res.status == LtvQpSolverStatus::kSolved ||
         qp_res.status == LtvQpSolverStatus::kSolvedInaccurate ||
         qp_res.status == LtvQpSolverStatus::kMaxIterations ||
         qp_res.status == LtvQpSolverStatus::kTimeLimit) &&
        qp_res.primal_solution.size() == qp_problem_buffer_.decisionSize() &&
        qp_res.dual_solution.size() == qp_problem_buffer_.constraintRows() &&
        qp_res.primal_solution.allFinite() && qp_res.dual_solution.allFinite();
    if (keep_warm_start) {
      qp_warm_start_.primal = qp_res.primal_solution;
      qp_warm_start_.dual = qp_res.dual_solution;
      qp_warm_start_valid_ = true;
    } else {
      qp_warm_start_valid_ = false;
      qp_solver_->resetWarmStart();
    }
    out_rollout_safety = rollout_safety;
    return false;
  }

  bool post_body_sat = false;
  bool post_wheel_sat = false;
  bool post_inc_lim = false;
  candidate.controls.front() = controller_->clampStepIncrement(
      candidate.controls.front(), snapshot.last_control_before_solve,
      snapshot.first_step_dt, &post_body_sat, &post_wheel_sat, &post_inc_lim);
  controller_->commitWarmControls(candidate.controls);

  out_result.success = true;
  out_result.controls = std::move(candidate.controls);
  out_result.states = std::move(candidate.states);
  out_result.iterations = qp_res.iterations;
  out_result.accepted_iterations = executed_rti;
  out_result.cost = 0.0;
  out_result.solve_time_ms =
      elapsedMilliseconds(sqp_start, std::chrono::steady_clock::now());
  out_result.body_limit_saturated = first_body_sat || post_body_sat;
  out_result.wheel_limit_saturated = first_wheel_sat || post_wheel_sat;
  out_result.increment_limited = first_inc_lim || post_inc_lim;
  out_result.module_limits_active = controller_->moduleLimitsActive();
  out_rollout_safety = rollout_safety;
  for (int axis = 0; axis < 3; ++axis) {
    telemetry.qp_first_control[static_cast<std::size_t>(axis)] =
        out_result.controls.front()(axis);
  }
  return true;
}

/**
 * @brief 处理 legacy emergency-stop 心跳。
 * @details structured ExecutionCommand 模式下 legacy false 永远不能恢复运动；其它模式中
 *          watchdog 的 stop 状态拥有优先权，并由 engageFailStop 实际发布零速度。
 */
void AtsSwerveMpcNode::onEmergencyStop(
    const std_msgs::msg::Bool::SharedPtr message) {
  if (execution_command_enabled_) {
    // legacy false 不能解除结构化 ExecutionCommand 建立的停止状态。
    if (message->data) {
      engageFailStop();
    }
    return;
  }
  const bool first_signal = !emergency_stop_signal_received_.exchange(true);
  emergency_stop_watchdog_.update(message->data);
  if (first_signal) {
    fail_stop_engaged_.store(false);
    engageFailStop();
  }
  if (message->data) {
    engageFailStop();
  } else if (!require_localization_status_ || localization_tracking_.load()) {
    fail_stop_engaged_.store(false);
  } else {
    engageFailStop();
  }
}

/** @brief Align with adapter/goal_manager planning-healthy states.
 *  CONFIRMED is the post-accept hold; rejecting it made GM commit while MPC
 *  fail-stopped (recovery182: mode=1 DualMap-ok on arbiter, MPC accepted=0).
 *  DEGRADED remains usable for control; LOST/BOOTSTRAP/RELOCALIZING/UNINITIALIZED
 *  stay fail-closed. */
void AtsSwerveMpcNode::onLocalizationStatus(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
        message) {
  using LS = ats_navigation_interfaces::msg::LocalizationStatus;
  const bool allows_control = message->state == LS::STATE_TRACKING ||
    message->state == LS::STATE_CONFIRMED ||
    message->state == LS::STATE_DEGRADED;
  if (!allows_control) {
    localization_tracking_.store(false);
    engageFailStop();
    return;
  }
  if (localization_epoch_ && *localization_epoch_ != message->epoch) {
    engageFailStop();
  }
  localization_epoch_ = message->epoch;
  localization_tracking_.store(true);
}

/** @brief 更新云台回执并在 active command 的 yaw 权限失效时触发急停。 */
void AtsSwerveMpcNode::onGimbalYawStatus(
    const ats_navigation_interfaces::msg::GimbalYawStatus::SharedPtr message) {
  bool stop = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    gimbal_status_ = *message;
    last_gimbal_status_signal_ = std::chrono::steady_clock::now();
    stop = active_execution_command_ && !gimbalExecutionValidLocked(*active_execution_command_);
  }
  if (stop) {
    engageFailStop();
  }
}

/**
 * @brief 在 trajectory mutex 已持有时验证云台回执的新鲜性、authority 和序列。
 * @details 未要求云台状态时返回 true；要求时任何缺失、超时或 lock 不满足都不可授权控制。
 */
bool AtsSwerveMpcNode::gimbalExecutionValidLocked(
  const ats_navigation_interfaces::msg::ExecutionCommand & command) const {
  if (!require_gimbal_status_) {
    return true;
  }
  if (!gimbal_status_ || !last_gimbal_status_signal_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *last_gimbal_status_signal_).count();
  if (age > gimbal_status_timeout_ || !gimbal_status_->tf_healthy ||
    gimbal_status_->yaw_authority != command.yaw_authority ||
    gimbal_status_->request_sequence != command.gimbal_request_sequence ||
    gimbal_status_->sequence < command.gimbal_feedback_sequence) {
    return false;
  }
  return !enforce_gimbal_lock_on_execution_ || !command.requires_gimbal_lock ||
         gimbal_status_->locked;
}

/**
 * @brief iLQR 唯一控制周期。
 * @details 依次执行输入健康/急停、reference 新鲜度、目标到达检查、iLQR 求解和唯一 Twist
 *          发布；随后 qp_shadow 仅读取冻结的 ControlCycleSnapshot，不能重读 ROS 状态或
 *          改动 tracker、last_control、emergency stop 与 publisher 所有权。
 */

} // namespace ats_swerve_mpc
