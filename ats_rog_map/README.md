# ats_rog_map

`ats_rog_map_node` keeps the ROG-Map sliding-window center at `base_frame`
and uses `sensor_frame` as the raycasting origin. It consumes an externally
registered cloud and external odometry/TF; it never publishes robot-pose TF.

The node publishes occupied, inflated, unknown, and ESDF debug clouds under
`rog_map/*`, plus `rog_map/stale`. The same process also runs the planning-map
owner (node `ats_rog_map_adapter`, `src/planning/`), which fuses the in-process
ground projection with terrain and static layers and publishes the only
planning-map contract, `/rog_map_adapter/planning_snapshot`, plus
subscriber-gated RViz views under `/rog_map_adapter/viz/` (`planning_grid`,
`esdf_cloud` coloured by distance, `clearance_contours` for the inscribed and
circumscribed radii).

`config/rog_map.yaml` is the default 10 cm startup profile, while
`config/rog_map_mujoco.yaml` lowers `p_occ` for sparse MuJoCo scans.
`config/rog_map_050.yaml` and the report-oriented 2.5 cm occupancy/inflation,
10 m x 10 m x 1 m profile `config/rog_map_report_025.yaml` can be selected
with `map_config_file`. The 2.5 cm profile keeps ESDF at 10 cm and has not
qualified target-host memory, update time, or report-level performance.
