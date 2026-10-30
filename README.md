# ats_sentry_nav

ATS 四驱四转舵轮哨兵的 ROS 2 Humble 导航仓库，覆盖定位、地图、规划和全向控制。
本仓位于工作区 `src/ats_sentry_nav`；实机入口与唯一正式参数文件在根仓 `src/ats_sentry_bringup`，
MuJoCo 仿真在 `src/sim/ats_mujoco_sim`。

导航目标统一进入 ATS action `/ats_navigate_to_pose`。运行链路不启动 Nav2 的 server、costmap 和 BT navigator。
ROGMap 只做环境表示，不做定位；位姿来自 `/localization`。

## 包职责

| 包 | 职责 |
| :--- | :--- |
| `livox_ros_driver2` | Livox 雷达驱动，SDK 随包放在 `Livox-SDK2/`。 |
| `point_lio` | 雷达惯性里程计，输出 `cloud_registered` 与 `aft_mapped_to_init`。 |
| `loam_interface` | 把 Point-LIO 输出整理成 `/registered_scan` 与 `/lidar_odometry`。 |
| `sensor_scan_generation` | 由 `/lidar_odometry` 和 `/registered_scan` 生成 `/odometry`、`/sensor_scan` 与相关 TF。 |
| `small_gicp_relocalization` | `small_gicp_relocalization_node` 用先验 PCD 做 GICP 重定位；`localization_fusion_node` 融合出 `/localization`、`/localization/status` 和 `map -> odom`。small_gicp 头文件放在包内 `include/small_gicp`，不需要另装。 |
| `terrain_analysis` / `terrain_analysis_ext` | 地形分析。`terrain_analysis_ext` 输出 `/traversability_grid` 等栅格，供 adapter 融合；`terrain_analysis` 默认不启动。 |
| `ats_rog_map` | ROG-Map 封装：概率占据、膨胀、三维 ESDF，通过 `/rog_map/get_ground_projection` 服务提供数值地面投影。 |
| `ats_rog_map_interfaces` | 上述 projection 服务的 schema（`GetRogMapProjection.srv`）。 |
| `ats_rog_map_adapter` | 融合 ROGMap 地面投影、terrain 栅格和静态图，发布 `/rc_esdf/planning_grid` 与 `/rog_map_adapter/*` 状态。 |
| `ats_rc_esdf` | 二维 signed distance、gradient、unknown 与静态图融合的算法库，被 `ats_rog_map_adapter` 和 `minco_planner` 依赖，没有自己的节点。 |
| `ats_goal_manager` | `/ats_navigate_to_pose` action server、目标状态机、候选参考的提交复核、急停与 `ExecutionCommand` 执行授权。 |
| `minco_planner` | JPS/A* 图搜索、MINCO 轨迹优化、独立 yaw、矩形足迹门禁与局部修复，详见 [minco_planner/README.md](./minco_planner/README.md)。 |
| `ats_swerve_mpc` | 全向 SE2 MPC（约束 iLQR），输出 `/cmd_vel/autonomy_raw`；OSQP LTV-QP 只有诊断用的 `qp_shadow` 模式，详见 [ats_swerve_mpc/README.md](./ats_swerve_mpc/README.md)。 |
| `fake_vel_transform` | 云台 yaw 与底盘速度 frame 的兼容层。 |
| `sentry_chassis_vel_transform` | 底盘速度 frame 适配，独立仓库，由 `dependencies.repos` 导入，本仓目录默认为空。 |
| `ats_cmd_vel_arbiter` | 手动 `/cmd_vel` 与自主 `/cmd_vel/autonomy` 仲裁，唯一输出 `/cmd_vel/selected`。 |
| `ats_teleop_twist_joy` | 手柄遥控。 |
| `pointcloud_to_laserscan` | 点云转 LaserScan，只被建图入口 `slam_launch.py` 使用。 |
| `ats_navigation_interfaces` | 导航 action 与 msg（`NavigateToPose`、`PlannerGoal`、`PlannerStatus`、`ExecutionCommand`、`LocalizationStatus`、`PlanningMapStatus`、`PlanningMapSnapshot` 等）。 |
| `ats_nav_bringup` | 定位、导航、建图和 RViz 的 launch，`static_map_publisher.py`（发布 `/map`）和 Mid360 用户配置。 |
| `ats_sentry_nav` | 聚合包，只有依赖声明。 |
| `third_party/osqp`、`third_party/qdldl` | OSQP 与 QDLDL 源码，由 `ats_swerve_mpc/CMakeLists.txt` 用 `add_subdirectory` 编译。`third_party/COLCON_IGNORE` 让 colcon 不把它们当独立包。 |

## 数据流

下面的 topic 名来自根仓 `src/ats_sentry_bringup/params/node_params.yaml` 与各节点源码。

```text
Livox 雷达 + IMU            /livox/lidar, /livox/imu
  -> point_lio              cloud_registered, aft_mapped_to_init
  -> loam_interface         /registered_scan, /lidar_odometry
  -> sensor_scan_generation /odometry, /sensor_scan, TF

定位   small_gicp_relocalization (/registered_scan + 先验 PCD) -> relocalization_observation
       localization_fusion (/odometry + observation) -> /localization, /localization/status, TF map -> odom

地图   ats_rog_map (/localization + /registered_scan) -> 服务 /rog_map/get_ground_projection
       terrain_analysis_ext (/lidar_odometry + /registered_scan) -> /traversability_grid 等
       ats_rog_map_adapter (ROGMap 投影 + terrain 栅格 + /map 静态图，用 ats_rc_esdf)
         -> /rc_esdf/planning_grid, /rc_esdf/signed_distance_grid, /rog_map_adapter/{ready,status,planning_snapshot,generation}

规划   /ats_navigate_to_pose -> ats_goal_manager -> /ats_goal_manager/planner_goal
       -> minco_planner (读 /rc_esdf/planning_grid) -> /minco/planning_status, /minco/reference_path_candidate
       -> ats_goal_manager 复核并提交 -> /minco/reference_path, /planner/execution_command, /planner/emergency_stop

控制   ats_swerve_mpc (/localization + /minco/reference_path + /planner/execution_command) -> /cmd_vel/autonomy_raw
       -> fake_vel_transform -> sentry_chassis_vel_transform -> /cmd_vel/autonomy   （两层按 launch 参数启用）
       -> ats_cmd_vel_arbiter (另有手动 /cmd_vel) -> /cmd_vel/selected
       -> standard_robot_pp_ros2 串口驱动 (cmd_vel_topic: /cmd_vel/selected) -> 底盘
```

补充：

- 目标在 `map` 系，规划与 ROGMap 在 `odom` 系（`ats_goal_manager` 的 `goal_frame`、`planning_frame`，`ats_rog_map` 的 `map_frame`）。
- 速度变换两层各由一个 launch 开关控制，MPC 输出 topic 随之变化，见“启动入口”一节的速度链表。
- 唯一权威：`/rc_esdf/planning_grid` 只由 adapter 发布，`/cmd_vel/selected` 只由 arbiter 发布。
- 失去地图、定位、有效 reference、执行授权或 heartbeat 时，安全结果是确定性零速度；`/rog_map_adapter/ready` 是持续续租的 heartbeat，不是永久状态。
- `ats_rog_map` 与 `ats_rog_map_adapter` 由根仓 `bringup.launch.py` 启动，不在本仓 `navigation_launch.py` 里。

## 构建

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y     # 首次
MAKEFLAGS=-j4 colcon build --base-paths src --parallel-workers 1 \
  --packages-up-to ats_sentry_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- 必须带 `--base-paths src`：工作区里的 `参考/` 有同名参考包，不能进入构建图。
- 显式带 `-DCMAKE_BUILD_TYPE=Release`，不要依赖各包的默认值。
- 整仓构建用根仓的 `./build.sh`（按机器内存与核数自动算并发，已带 Release）。单包用 `--packages-select <pkg>`。
- 与别人正在使用的 `build/`、`install/` 并行验证时，加 `--build-base`、`--install-base` 指向独立目录，不要在共享目录里构建。
- `sentry_chassis_vel_transform` 在独立仓库：`vcs import --skip-existing src < src/ats_sentry_nav/dependencies.repos`。
- 依赖：Ubuntu 22.04、ROS 2 Humble、Eigen3、PCL、yaml-cpp、glog、OpenMP、libunwind；Livox SDK 与 small_gicp 已在源码树内。

## 启动入口与常用参数

下面的参数名与默认值取自 launch 源码；以 `ros2 launch <pkg> <file> --show-args` 的输出为准。

### 实机：根仓 `ats_sentry_bringup`

```bash
ros2 launch ats_sentry_bringup real_robot_navigation.launch.py \
  world:=rmuc_2026 prior_pcd_file:=/绝对路径/rmuc_2026.pcd use_rviz:=true
```

`real_robot_navigation.launch.py` 的参数：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `world` | `rmuc_2026` | 场地名，决定默认的先验 PCD 文件名。 |
| `prior_pcd_file` | `<ats_sentry_bringup>/pcd/<world>.pcd` | 先验 PCD，必须是同一场地、已在 map 系下。文件缺失时 `prior_pcd_preflight.launch.py` 拒绝启动，不会换用别的场地。 |
| `namespace` | 空 | 节点命名空间。 |
| `params_file` | `<ats_sentry_bringup>/params/node_params.yaml` | 唯一正式参数文件。 |
| `require_gimbal_status` | `True` | 是否要求云台状态（云台状态桥、Goal Manager、MPC 使用）。 |
| `launch_behavior` | `True` | `False` 时不启动行为节点，只调导航 action。 |
| `use_rviz` | `False` | 是否启动 RViz。 |
| `launch_fake_vel_transform` | `True` | 云台速度 frame 兼容层。 |
| `launch_chassis_vel_transform` | `True` | 底盘速度 frame 兼容层。 |
| `log_level` | `info` | 节点日志级别。 |

它把 `use_sim_time:=False`、`require_serial_link:=True`、`launch_small_gicp_relocalization:=True`、
`launch_localization_fusion:=True` 固定后转给 `bringup.launch.py`。`bringup.launch.py` 自己的
`launch_fake_vel_transform`、`launch_chassis_vel_transform` 默认值是 `False`，两个入口的默认不同，改动前先看 `--show-args`。

`bringup.launch.py` 的其他常用参数：`world`（默认 `rmuc_2025`）、`map`（默认 `<ats_sentry_bringup>/map/<world>.yaml`）、
`prior_pcd_file`、`use_sim_time`、`use_robot_state_pub`、`launch_joy_teleop`、`launch_rosbag_recorder`、
`launch_small_gicp_relocalization`、`launch_localization_fusion`、`launch_cmd_vel_arbiter`、`require_serial_link`、
`mpc_cmd_vel_topic`、`fake_vel_output_topic`、`chassis_vel_input_topic`、`chassis_vel_output_topic`、
`launch_lidar_static_tf` 及其外参 `lidar_static_tf_x/y/z/roll/pitch/yaw`、`rviz_config_file`、`use_respawn`、`log_level`。
它启动的内容：串口驱动（`use_robot_state_pub:=False` 时）、雷达静态 TF（`use_sim_time:=False` 且 `launch_lidar_static_tf` 为真时）、云台状态桥（`require_gimbal_status` 为真时）、`ats_rog_map`、
`ats_rog_map_adapter`、本仓的 `rm_navigation_reality_launch.py`、行为节点、RViz、rosbag 录制。

根仓另有 `decision_vision_test.launch.py`、`record_rosbag_launch.py`、`rviz_launch.py`。

发送单个目标（会驱动车辆，只在受限低速区域、物理急停可用且操作员授权后执行）：

```bash
ros2 action send_goal --feedback /ats_navigate_to_pose \
  ats_navigation_interfaces/action/NavigateToPose \
  "{goal_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, timeout: {sec: 60, nanosec: 0}}"
```

`timeout` 小于等于 0 时使用 Goal Manager 的服务端默认值。

### 仿真：MuJoCo

仿真入口在根仓 `scripts/test_mujoco_minco_mpc_chain.sh`：启动 MuJoCo 与导航栈，发目标并评估，常用环境变量有
`ROS_DOMAIN_ID`（默认 `88`）、`TEST_PROFILE`（`default`、`single`、`rectangle`、`south_corridor`、`red_box`、`red_box_2027`、`tunnel_2027`，默认 `single`）、
`SCENE`（默认 `rmuc_2025`）、`USE_RVIZ`（默认 `false`）、`USE_MUJOCO_VIEWER`。

```bash
cd /home/ats/ATS_2026_snetry_test
ROS_DOMAIN_ID=88 TEST_PROFILE=single USE_RVIZ=false scripts/test_mujoco_minco_mpc_chain.sh
```

手动启动与场景细节见 [src/sim/ats_mujoco_sim/README.md](../sim/ats_mujoco_sim/README.md)，
RViz 与窗口的使用规则见根仓 `AGENTS.md` 7.1。Gazebo 入口见 [src/sim/gazebo_simulator/README.md](../sim/gazebo_simulator/README.md)。

### 本仓 `ats_nav_bringup/launch`

| 文件 | 内容 |
| :--- | :--- |
| `rm_navigation_reality_launch.py` | 实机导航编排，被根 `bringup.launch.py` 包含。必填 `assets_dir`（根仓 `ats_sentry_bringup` 目录），`map`、`prior_pcd_file`、`params_file`、`rviz_config_file` 默认取自 `assets_dir` 下的 `map/<world>.yaml`、`pcd/<world>.pcd`、`params/node_params.yaml`、`rviz/sentry_default_view.rviz`；`world` 默认 `rmul`。另有 `launch_joy_teleop`、`launch_small_gicp_relocalization`、`launch_localization_fusion`、`launch_fake_vel_transform`、`launch_chassis_vel_transform`、`launch_cmd_vel_arbiter`、`require_serial_link`、`require_gimbal_status`、`use_respawn`、`use_robot_state_pub`、`use_rviz`、`log_level` 等。 |
| `bringup_launch.py` | 依次包含 `localization_launch.py` 与 `navigation_launch.py`。 |
| `localization_launch.py` | `static_map_publisher.py`（`/map`）、`point_lio`、`small_gicp_relocalization`、`localization_fusion`；关闭 GICP 时改发恒等的静态 `map -> odom`。 |
| `navigation_launch.py` | `loam_interface`、`sensor_scan_generation`、`terrain_analysis_ext`、`fake_vel_transform`、`chassis_vel_transform`、`cmd_vel_arbiter`、`minco_planner`、`ats_goal_manager`、`ats_swerve_mpc`；`launch_terrain_analysis`（默认 `False`）控制 `terrain_analysis`。 |
| `slam_launch.py` | 建图：`point_lio`（`pcd_save.pcd_save_en: True`）、`pointcloud_to_laserscan`、`slam_toolbox`、静态 `map -> odom`。 |
| `prior_pcd_preflight.launch.py` | 先验 PCD 预检；参数 `world`、`prior_pcd_file`、`launch_small_gicp_relocalization`。 |
| `joy_teleop_launch.py`、`rviz_launch.py`、`robot_state_publisher_launch.py` | 手柄、RViz、机器人模型。 |

速度链随两个开关变化。根 `bringup.launch.py`、`rm_navigation_reality_launch.py` 和 `navigation_launch.py`
按下表推导 `mpc_cmd_vel_topic`、`fake_vel_output_topic`、`chassis_vel_input_topic`，`chassis_vel_output_topic` 固定为 `/cmd_vel/autonomy`：

| `launch_fake_vel_transform` | `launch_chassis_vel_transform` | 速度链 |
| :--- | :--- | :--- |
| 真 | 真 | MPC -> `/cmd_vel/autonomy_raw` -> fake -> `/cmd_vel/autonomy_gimbal` -> chassis -> `/cmd_vel/autonomy` |
| 真 | 假 | MPC -> `/cmd_vel/autonomy_raw` -> fake -> `/cmd_vel/autonomy` |
| 假 | 真 | MPC -> `/cmd_vel/autonomy_gimbal` -> chassis -> `/cmd_vel/autonomy` |
| 假 | 假 | MPC -> `/cmd_vel/autonomy` |

arbiter 订阅的自主输入 `autonomy_cmd_vel_topic` 默认是 `/cmd_vel/autonomy`。

### 各包独立 launch

`ros2 launch ats_rog_map ats_rog_map.launch.py`、`ats_rog_map_adapter ats_rog_map_adapter.launch.py`、
`ats_swerve_mpc ats_swerve_mpc.launch.py` 可单独启动对应节点。包内的 `config/*.yaml`
（如 `ats_goal_manager/config`、`ats_swerve_mpc/config/*_reality.yaml`）不是正式入口的第二配置源，只在独立调试或测试时显式传入。

### Point-LIO 手动入口

```bash
ros2 launch point_lio point_lio.launch.py                       # 默认 rviz:=True，配置为 config/mid360.yaml
ros2 launch point_lio point_lio.launch.py rviz:=False \
  point_lio_cfg_dir:=/绝对路径/config/avia.yaml
```

参数：`namespace`、`rviz`、`point_lio_cfg_dir`。`config/` 下有 `avia`、`horizon`、`mid360`、`ouster64`、`velody16` 五份参数，
RViz 配置是 `rviz_cfg/point_lio_replay.rviz`。这个 launch 不被任何实机或仿真入口包含；
正式运行的 Point-LIO 参数来自 `node_params.yaml` 的 `point_lio:` 段。Point-LIO 本身的说明见 [point_lio/README.md](./point_lio/README.md)。

## 参数

正式参数的唯一权威是根仓 `src/ats_sentry_bringup/params/node_params.yaml`。按节点段定位：

| 节点段 | 主要内容 |
| :--- | :--- |
| `livox_ros_driver2`、`point_lio`、`loam_interface`、`sensor_scan_generation` | 雷达与 IMU topic、frame、里程计输出。 |
| `small_gicp_relocalization`、`localization_fusion` | 重定位、融合、状态与 timeout。 |
| `terrain_analysis`、`terrain_analysis_ext` | 地形与可通行栅格。 |
| `ats_rog_map`、`ats_rog_map_adapter` | 地图尺寸、分辨率、概率、膨胀、projection 服务、栅格 topic、ready/lease。 |
| `minco_planner` | JPS、MINCO、yaw、足迹、修复、输入输出 topic。 |
| `ats_goal_manager` | action、goal/planning frame、heartbeat、提交复核、执行授权。 |
| `ats_swerve_mpc` | SE2 模型、horizon、权重、约束、watchdog、输出 topic。 |
| `cmd_vel_arbiter` | 手动/自主/选中 topic、超时、串口链路检查。 |
| `fake_vel_transform`、`chassis_vel_transform`、`ats_teleop_twist_joy_node` | 速度 frame 兼容层与手柄映射。 |

接口 schema 以 `.msg`、`.srv`、`.action` 为准；行为树 XML、航点 CSV、地图图像和 PCD 是结构化资产，不放进参数 YAML。

## 测试与验证

```bash
cd /home/ats/ATS_2026_snetry_test
MAKEFLAGS=-j4 colcon build --base-paths src --packages-select <pkg> --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src --packages-select <pkg>
colcon test-result --test-result-base build/<pkg> --verbose
python3 scripts/validate_navigation_config.py          # 正式参数、launch 与 RViz 契约
ros2 launch <pkg> <file> --show-args
```

- 部分包的 ament lint 用例（copyright、cpplint、uncrustify、black、pep257 等）在当前 HEAD 上本来就是红的，
  功能结论只看对应的 GTest/pytest 用例，不要写成“全包测试通过”。
- 单元测试通过、topic 存在或 RViz 可见都不代替闭环验收；闭环用 MuJoCo 入口，结果要有终点测量和话题证据。

## 已知边界

- `PlannerCandidate.msg` 与 `ExecutionCommand` 的候选关联字段已在 schema 中定义；目前只有
  `minco_planner/include/minco_planner/nodes/atomic_planning_contract.hpp` 及其测试使用候选结构，正式运行链还没有发布它。
- 足迹门禁是离散采样（逐点与扫掠），连续 swept footprint 属于后续阶段，不是连续体证明。
- 实车动力学、制动与 HIL 尚未验证；MuJoCo 的参数和结果不能当作 ATS 实测。
- 分阶段状态与验收记录见根仓 `AGENTS.md` 与 `docs/`。

## 目录结构

```text
ats_sentry_nav/
├── ats_navigation_interfaces/    ats_rog_map_interfaces/
├── ats_rog_map/    ats_rog_map_adapter/    ats_rc_esdf/
├── minco_planner/    ats_goal_manager/    ats_swerve_mpc/    ats_cmd_vel_arbiter/
├── ats_nav_bringup/    ats_sentry_nav/    ats_teleop_twist_joy/
├── point_lio/    loam_interface/    sensor_scan_generation/    small_gicp_relocalization/
├── terrain_analysis/    terrain_analysis_ext/    pointcloud_to_laserscan/    livox_ros_driver2/
├── fake_vel_transform/    sentry_chassis_vel_transform/   （后者来自 dependencies.repos）
├── third_party/    osqp/  qdldl/  COLCON_IGNORE
├── dependencies.repos    Dockerfile
└── README.md    README_en.md
```

## 致谢与许可证

| 项目 | 用途 | 上游 |
| :--- | :--- | :--- |
| Point-LIO | 雷达惯性里程计 | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| ROG-Map | 概率占据、滑窗地图与 ESDF 核心 | [hku-mars/ROG-Map](https://github.com/hku-mars/ROG-Map) |
| GCOPTER / MINCO | MINCO S3 轨迹表示与优化基础 | [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) |
| small_gicp | 点云配准与重定位 | [koide3/small_gicp](https://github.com/koide3/small_gicp) |
| Livox ROS Driver 2 | Livox 设备接入 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |
| OSQP / QDLDL | `ats_swerve_mpc` 的 QP 诊断求解器 | [osqp/osqp](https://github.com/osqp/osqp)、[osqp/qdldl](https://github.com/osqp/qdldl) |
| pointcloud_to_laserscan | 点云转 LaserScan | [ros-perception/pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) |

本仓聚合了不同来源与许可证的包：ATS 自研的大多数包为 Apache-2.0；`ats_rog_map` 为 `LGPL-3.0-or-later`
（归属见 `ats_rog_map/NOTICE`）；`point_lio`、`terrain_analysis`、`terrain_analysis_ext`、`pointcloud_to_laserscan` 为 BSD；
`livox_ros_driver2` 为 MIT；MINCO 归属见 `minco_planner/THIRD_PARTY_NOTICES.md`。最终以各包的 `package.xml`、`LICENSE`、`NOTICE` 和上游声明为准。
