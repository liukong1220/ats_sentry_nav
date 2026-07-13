# minco_planner

Optional V1 planning pipeline:

`traversability_grid -> JPS -> MINCO S3 -> independent yaw -> footprint safety`

The default front end is 2D JPS with configurable clearance. A* remains available
through `search_algorithm: astar` and as an optional JPS failure fallback. The
translation backend is the non-uniform-time, fifth-order MINCO S3 solver adapted
from GCOPTER. It publishes timed `nav_msgs/Path` poses and internally retains
world-frame `vx/vy/ax/ay` for the holonomic controller boundary.

Swerve-specific rules:

1. Translation and chassis yaw are independent. `goal_heading` is the default;
   `hold` and legacy `path_tangent` are available for comparison.
2. JPS uses `jps_safe_distance`, then the exact oriented rectangular footprint is
   checked on the sampled MINCO trajectory.
3. Unsafe trajectories are not published by default.
4. Local point repair is disabled by default because nearest-cell repair is crude.
   If enabled, repaired geometry is passed through MINCO again before publication.

The node can consume a direct `goal_pose` or the final pose of Nav2 `/plan`. The
latter keeps RViz Nav2 GoalTool usable during the transition.

See `THIRD_PARTY_NOTICES.md` for the MINCO license and attribution.
