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

Autonomous regression:

```bash
scripts/test_mujoco_minco_mpc_chain.sh
```
