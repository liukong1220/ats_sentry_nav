# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/LihanChen2004/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

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
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```

## Runtime input contract

`registered_scan` is consumed with `SensorDataQoS.keep_last(1)` so a delayed
scan cannot build an unbounded backlog while GICP is busy. Before a scan enters
the accumulation window, the node checks:

- non-empty `frame_id` and (when `lidar_frame` is configured) an exact frame match;
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

The fusion node applies a separate receive-time gate to accepted observations:
`observation_stamp_max_age_s` and `observation_stamp_max_future_s` prevent a
delayed message from changing `map->odom` merely because the matching odometry
sample is still present in the history buffer. The existing odometry-history,
quality, plausibility, confirmation and epoch checks remain authoritative.
