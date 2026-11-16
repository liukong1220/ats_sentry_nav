# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

In the ATS integration, registration consumes an odom-frame registered cloud and a map-frame prior, then publishes relocalization observations. `localization_fusion` is the sole publisher of `map -> odom` and `/localization/status`; GICP does not publish a competing pose TF. Mapping mode instead matches against a growing map without loading a prior.

The sensor is fixed to the chassis; Point-LIO owns deskewing and odometry. Current MuJoCo MID360 input is 10 Hz in simulation time with a 200 Hz IMU. See the [current report](../../../docs/调研/无先验地图钻洞高速导航优化调研报告.md) and [2027 acceptance plan](../../../docs/nav_acceptance/20261116_2027_final_test_plan.md) for profile values, startup failures and unpassed navigation gates. Upstream build examples below are standalone examples, not the workspace deployment procedure.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

## Usage

1. Pass the prior pointcloud with the required `prior_pcd_file` launch argument of
   [launch file](launch/small_gicp_relocalization_launch.py); a missing or unreadable
   file aborts the launch

2. Set the coordinate-frame contract

    The prior PCD must already be expressed in `map_frame`. It is loaded unchanged;
    no mechanical body-to-lidar transform is applied or required at startup.
    `registered_scan` must be expressed in `odom_frame`, so registration estimates
    `T_map_odom`. Set `robot_base_frame` to the body pose frame used for scan-time
    `T_odom_base` when composing relocalization observations. GICP has no
    `base_frame` or `lidar_frame` parameters; sensor extrinsics belong to the
    upstream odometry pipeline. An empty or unreadable prior fails startup.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py \
      prior_pcd_file:=/absolute/path/to/the_matching_world.pcd
    ```

## Runtime input contract

`registered_scan` is consumed with `SensorDataQoS.keep_last(1)` so a delayed
scan cannot build an unbounded backlog while GICP is busy. Before a scan enters
the accumulation window, the node checks:

- non-empty `frame_id` exactly matching `odom_frame` (physical lidar frames are rejected);
- positive, strictly increasing timestamps, a bounded age (`max_scan_age_s`) and
  future tolerance (`max_scan_future_s`);
- finite XYZ values, range and height limits, and `min_scan_valid_ratio`.

Invalid points are removed from an otherwise usable frame. A frame whose valid
ratio is below the configured threshold is rejected as a whole. The accumulation
window is bounded by `max_accumulated_points` and `max_accumulated_frames`; when
the limit is reached the old window is cleared before accepting the newest frame.
Drop, stale, invalid, accepted and trimmed-window counters are emitted in a
throttled diagnostic log. This node does not deskew scans: the upstream
`registered_scan` producer must publish a cloud already expressed at its header
timestamp, or the frame is not suitable for global relocalization.

The default limits are conservative for real and simulated runs. Offline replay
may set `max_scan_age_s:=0` when message timestamps intentionally do not share the
replay node's clock; this disables only the age gate, not frame, monotonicity,
finite-point or range checks.

The fusion node has no separate stamp-age gate. An accepted observation is expressed
against `odom->base` interpolated at the observation stamp, so a delayed observation
still yields a correct `map->odom`; its age is bounded by the odometry history window
(`history_duration_s`, default 5 s) and its stamp may not be newer than the latest odometry
sample (`history_boundary_tolerance_s`). Outside that window the observation is rejected as
"outside fresh odometry history". Quality, plausibility, confirmation and epoch checks and the
`observation_timeout_s` lease remain.

For asynchronous multi-guess recovery, the coarse screen uses a sparse cloud and
therefore has its own `multi_guess.screen_min_inliers` floor (zero selects an
automatic fraction of the full-cloud `min_inliers`). The scheduler reserves
`multi_guess.fine_time_reserve_s` before the wall-time budget expires so at least
one screened candidate can reach full-cloud fine alignment. A screen pass with
`refined=0` is a scheduling/configuration failure, not evidence that the map has
no match; the fine-stage quality and confirmation gates remain unchanged.
