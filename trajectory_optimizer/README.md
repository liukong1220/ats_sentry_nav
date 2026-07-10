# trajectory_optimizer

This package is the current Nav2-compatible B-spline smoothing and RC-ESDF
transition layer.

## Current V1 Test Chain

The current transition test chain keeps a stable Nav2 global reference and
publishes `local_elastic_path` only when the RC-ESDF optimization changes it
materially. `FollowElasticPath` updates the controller action from that topic
with age, goal-compatibility, and update-rate checks. If RC-ESDF is unavailable,
the controller remains on the stable global reference path.

Global replanning is handled by the Nav2 BT only when the goal changes or
`IsPathValid` fails. Recovery still uses the existing
`ats_nav2_behaviors/BackUpFreeSpace` behavior server plugin; this package does
not replace it with the upstream backup implementation.

Directory layout:

1. `include/trajectory_optimizer/bspline`, `src/bspline`
   B-spline path smoothing, continuous optimization, curvature / slope /
   obstacle speed-profile generation.
2. `include/trajectory_optimizer/esdf`, `src/esdf`
   ESDF providers and the RC-ESDF-lite traversability backend.
   `rc_traversability_esdf_provider` uses positive distance in free space and
   negative distance inside obstacles.
   `fake_costmap_esdf_provider` is only a fallback/debug adapter that converts a
   Nav2 `Costmap2D` into a non-negative obstacle-distance field. It does not
   model signed obstacle penetration, so the competition planning chain should
   prefer the traversability RC-ESDF provider when that data is available.
3. `include/trajectory_optimizer/nav2`, `src/nav2`
   Nav2 smoother plugin wrapper.
4. `include/trajectory_optimizer/control`, `src/control`
   Speed-governor node consuming trajectory profiles and controller commands.
5. `include/trajectory_optimizer/nodes`, `src/nodes`
   Standalone trajectory optimizer / debug node.

Keep new implementation work in the matching layer directory. Avoid adding new
planning, controller, or ESDF logic directly into a node wrapper.
