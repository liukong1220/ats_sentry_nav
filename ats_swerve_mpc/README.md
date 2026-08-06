# ats_swerve_mpc

Holonomic SE(2) MPC tracker for the ATS four-wheel independent-steering chassis.

State is world-frame `[x, y, yaw]`. Control is body-frame `[vx, vy, wz]` and is
published as `geometry_msgs/Twist`. The model intentionally supports true lateral
motion; it does not import differential-drive ICR or curvature constraints.

The input is the timed `/minco/reference_path`. The measured position is projected
onto the nearest admissible trajectory segment, with backward-progress prevention
and a bounded forward search window. Large cross-track error compresses horizon
progress so lateral recovery takes priority over along-track speed. Pose timestamps
still provide interpolation and body-frame feed-forward velocity, and a short
preview compensates command latency. Velocity and acceleration limits are enforced
at every MPC step. Emergency stop, goal completion, and the scaled trajectory
deadline publish zero velocity.

MuJoCo opt-in launch:

```bash
ros2 launch ats_mujoco_sim rmuc_2026_mujoco.launch.py \
  launch_swerve_mpc:=true
```

When enabled, the launch disables `fake_vel_transform`, routes MPC output through
`/cmd_vel_mpc`, and makes `twist_to_motion_ctrl` the single subscriber that feeds
MuJoCo `/motion_control`. The default launch remains the Nav2 MPPI baseline.

## LTV-QP migration status

The production controller still uses the existing constrained iLQR path. The
first QP migration slice is intentionally solver-independent:

- `Se2Model` is the shared SE(2) dynamics, Jacobian and rollout implementation
  used by the current iLQR controller and the future LTV-QP backend.
- `ZeroSpeedGuard` prevents steering-angle linearization when a wheel velocity
  vector has no defined direction near zero speed. It uses hysteresis and does
  not invent an in-place steering command; the current Twist-only interface
  still requires the lower-level steering state machine for that behavior.
- `LtvQpBuilder` constructs a convex, linearized tracking problem with SE(2)
  equality dynamics, body velocity bounds and body control-increment bounds.
  It exposes low-speed angle validity but does not yet approximate wheel-norm
  or steering-rate constraints and is not connected to the ROS control timer.

No QP package is currently available in the workspace or system dependency set.
Until a bounded, warm-started backend is added and its hard-constraint residuals
are verified, the QP description is shadow-only and must not be described as
the real-robot solver. Emergency-stop, localization, lease, reference freshness
and zero-velocity behavior remain owned by the existing node.

Autonomous regression:

```bash
scripts/test_mujoco_minco_mpc_chain.sh
```
