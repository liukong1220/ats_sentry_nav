# ats_sentry_nav

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/ats_sentry_nav/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/ats_sentry_nav/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Shenzhen MSU-BIT University PolarBear Robotics Team's Sentry Navigation Simulation/Reality Robot Package For RoboMaster 2025.

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

[BiliBili: RM navigation simulation for beginners](https://www.bilibili.com/video/BV12qcXeHETR)

https://github.com/user-attachments/assets/d9e778e0-fa43-40c2-96c2-e71eaf7737d4

https://github.com/user-attachments/assets/ae4c19a0-4c73-46a0-95bd-909734da2a42

## 1. Overview

> The Chinese [README.md](./README.md) is the maintained reference. This page only summarizes the current stack.

This repository started from the PolarBear RoboMaster 2025 sentry navigation project. The current ATS runtime is Nav2-free: no Nav2 servers, costmaps or BT navigator are launched, and goals enter the ATS `NavigateToPose` action (`/ats_navigate_to_pose`).

- Localization: [point_lio](./point_lio/) provides odometry and `/registered_scan`; [small_gicp_relocalization](./small_gicp_relocalization/) maintains `map -> odom` against a prior PCD. [loam_interface](./loam_interface/) and [sensor_scan_generation](./sensor_scan_generation/) adapt the point cloud and TF chain.

- Mapping: `ats_rog_map` (probabilistic occupancy, inflation, 3D ESDF, numeric ground projection) and `ats_rog_map_adapter`, which fuses ROGMap, terrain and the static map into `/rc_esdf/planning_grid`.

- Planning and control pipeline:

    ```text
    ats_goal_manager -> minco_planner -> ats_swerve_mpc -> cmd_vel_arbiter -> /cmd_vel/selected -> chassis
    ```

    `minco_planner` runs JPS/A* graph search, path preprocessing, whole-trajectory joint MINCO optimization (waypoints and durations in one time-budgeted L-BFGS problem), clearance-aware yaw planning with local time scaling, and a rectangular footprint gate (discrete + swept) with local repair stages. `ats_goal_manager` reviews and commits the candidate reference. `ats_swerve_mpc` is a constrained iLQR swerve MPC that keeps lateral motion. On the real robot `fake_vel_transform` and `sentry_chassis_vel_transform` sit between the MPC and the arbiter. The vehicle footprint is 0.58 x 0.58 m.

- Simulation: full simulation uses MuJoCo (`ats_mujoco_sim`, outside this repository).

- File Structure

    ```txt
    .
    ├── ats_navigation_interfaces       # Action, status, authorization and atomic snapshot schemas
    ├── ats_rog_map_interfaces          # ROGMap numeric projection service
    ├── ats_rog_map                     # Occupancy, inflation, 3D ESDF, ground projection
    ├── ats_rog_map_adapter             # 2.5D conservative fusion, owner of /rc_esdf/planning_grid
    ├── ats_rc_esdf                     # 2D signed distance / static map fusion
    ├── minco_planner                   # JPS/A*, MINCO joint optimization, yaw, footprint gate
    ├── ats_goal_manager                # NavigateToPose action, commit review, execution authorization
    ├── ats_swerve_mpc                  # Omnidirectional SE(2) MPC (iLQR; OSQP shadow for diagnostics)
    ├── ats_cmd_vel_arbiter             # Manual/autonomy velocity arbitration -> /cmd_vel/selected
    ├── ats_nav_bringup                 # Localization/navigation launch files, maps, RViz
    ├── ats_teleop_twist_joy            # Gamepad control
    ├── ats_sentry_nav                  # Aggregate package
    ├── fake_vel_transform              # Gimbal-yaw velocity frame compatibility layer
    ├── sentry_chassis_vel_transform    # Chassis velocity frame adapter (imported via dependencies.repos)
    ├── livox_ros_driver2               # Livox driver
    ├── loam_interface                  # Odometry point cloud interface
    ├── point_lio                       # Odometry
    ├── pointcloud_to_laserscan         # PointCloud to LaserScan conversion
    ├── sensor_scan_generation          # Point cloud related coordinate transformation
    ├── small_gicp_relocalization       # Relocalization and localization fusion
    ├── terrain_analysis                # Near-field terrain analysis
    ├── terrain_analysis_ext            # Far-field terrain analysis
    └── third_party/osqp                # OSQP source used by ats_swerve_mpc
    ```

## 2. Quick Start

### 2.1 Option 1: Docker

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)

- Allow Docker Container to access the host's X11 display

    ```bash
    xhost +local:docker
    ```

#### 2.1.2 Create Container

```bash
docker run -it --rm --name ats_sentry_nav \
  --network host \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ghcr.io/smbu-polarbear-robotics-team/ats_sentry_nav:1.3.1
```

### 2.2 Option 2: Build From Source

#### 2.2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Simulation packages: `ats_mujoco_sim` for full dynamics/sensor validation and `nav2_loopback_sim` for fast navigation or behavior-tree tests.
- Install [small_icp](https://github.com/koide3/small_gicp):

    ```bash
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

#### 2.2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/ats_sentry_nav.git src/ats_sentry_nav
```

Download prior point cloud:

Prior point clouds are used for point_lio and small_gicp. Due to large file size, they are not stored in Git. Please download them from [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS).

> Note: The performance of point_lio with prior_pcd in large scenes is not optimal, and it is more prone to drift than without prior point clouds. Debugging and optimization are ongoing.

#### 2.2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Always restrict the build to `--base-paths src`. A single package builds with `colcon build --base-paths src --packages-select <pkg>`.

> [!NOTE]
> We highly recommend building your workspace using the symlink-install option since ats_sentry_nav extensively utilizes launch_file and YAML resources. This option installs symbolic links to those non-compiled source files meaning that you don't need to rebuild again and again when you're for example tweaking a parameter file. Instead, your changes take effect immediately and you just need to restart your application.

### 2.3 Running

#### 2.3.1 Simulation

Full simulation uses MuJoCo:

```bash
ros2 launch ats_mujoco_sim mujoco_navigation.launch.py
```

#### 2.3.2 Physical Robot

The real-robot entry is owned by the workspace-level `ats_sentry_bringup` package and uses `src/ats_sentry_bringup/params/node_params.yaml` as the single parameter source:

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py world:=rmuc_2026 use_rviz:=true
```

Send goals to the `/ats_navigate_to_pose` action (`ats_navigation_interfaces/action/NavigateToPose`). This drives the robot; only run it in a restricted low-speed area with a working physical emergency stop.

### 2.4 Launch Arguments

Use `--show-args` for the authoritative list. `ats_nav_bringup/launch/rm_navigation_reality_launch.py` currently declares: `namespace`, `assets_dir`, `world`, `use_sim_time`, `use_respawn`, `use_robot_state_pub`, `use_rviz`, `rviz_force_software`, `launch_joy_teleop`, `launch_small_gicp_relocalization`, `launch_localization_fusion`, `launch_fake_vel_transform`, `launch_chassis_vel_transform`, `chassis_vel_output_topic`, `launch_cmd_vel_arbiter`, `require_gimbal_status`, `log_level`.

### 2.5 Joy teleop

Gamepad support is off by default (`launch_joy_teleop:=False`). The key mapping is in the `ats_teleop_twist_joy_node` section of `node_params.yaml`.
