# ats_sentry_nav

ATS 2026 哨兵的定位、地图、规划、轨迹与四舵轮控制仓库。

> 状态：仓库同时包含 Nav2 对照资源和 Nav2-free 自研链。专用实机入口已能
> 关闭 Nav2，但 MuJoCo 默认与部分构建依赖仍保留 Nav2，不能声明全仓移除完成。

## 目录

- [功能模块](#功能模块)
- [依赖](#依赖)
- [Quick Start](#quick-start)
- [启动入口](#启动入口)
- [话题服务与 Action](#话题服务与-action)
- [配置文件](#配置文件)
- [数据流](#数据流)
- [地图与安全契约](#地图与安全契约)
- [目录结构](#目录结构)
- [测试](#测试)
- [验证边界](#验证边界)
- [参考与致谢](#参考与致谢)

## 功能模块

| 包 | 主要职责 | 状态 |
| :--- | :--- | :--- |
| `point_lio` | 输出 `/localization`、`/registered_scan` | 保留，ROGMap 不替代它 |
| `small_gicp_relocalization` | prior PCD 重定位与 `map -> odom` 约束 | 实机链 |
| `terrain_analysis*` | 地面、高差、坡度和可通行语义 | ROGMap adapter 的输入之一 |
| `ats_rog_map` | 概率占据、膨胀、3D ESDF、数值地面投影服务 | P2 活动实现 |
| `ats_rog_map_adapter` | 融合 ROG 投影、static map、terrain/slope，发布规划 grid 与数值 ESDF | P2 地图 owner |
| `minco_planner` | JPS/A* fallback、MINCO S3、独立 yaw、footprint gate/repair | 自研规划链 |
| `ats_goal_manager` | ATS action、目标生命周期、安全 reference 提交 | P3 自研目标 owner |
| `ats_swerve_mpc` | 全向 SE2 MPC，输出车体系 `[vx, vy, wz]` | 四舵轮控制 |
| `ats_navigation_interfaces` | action/message schema | 接口权威 |
| `trajectory_optimizer` | RC-ESDF 与 Nav2 B-spline 对照组件 | 过渡期保留 |
| `ats_nav_bringup` | 定位、Nav2 对照和自研节点编排 | 过渡期入口 |
| `ats_nav2_plugins` | Nav2 recovery/costmap 插件 | 对照资源，尚未删除 |
| `fake_vel_transform` | Nav2 fake-yaw 兼容速度变换 | 只属于兼容 profile |

## 依赖

- ROS 2 Humble
- Eigen3、PCL、OpenCV、yaml-cpp
- `geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_ros`
- `ats_navigation_interfaces`、`ats_rog_map_interfaces`
- Nav2 依赖仅用于当前对照 profile、插件和过渡 bringup

## Quick Start

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash

MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-up-to \
    ats_rog_map ats_rog_map_adapter minco_planner \
    ats_goal_manager ats_swerve_mpc \
  --parallel-workers 1 \
  --symlink-install

source install/setup.bash
```

不要让 `colcon` 扫描 `参考/`；活动 ROGMap 唯一位置是本仓的 `ats_rog_map`。

## 启动入口

### 从根仓启动 Nav2-free 实机链

```bash
ros2 launch ats_sentry_bringup real_robot_nav2_free.launch.py \
  world:=rmuc_2026 \
  planning_grid_owner:=rog_map
```

该入口固定：

| 参数 | 值 | 含义 |
| :--- | :--- | :--- |
| `launch_nav2` | `false` | 无 `bt_navigator/planner_server/controller_server/behavior_server` |
| `launch_swerve_mpc` | `true` | 启动 MINCO、Goal Manager、MPC |
| `launch_fake_vel_transform` | `false` | MPC 后不再二次旋转速度 |
| `launch_chassis_vel_transform` | `false` | MPC 直接到 `/cmd_vel` |
| `planning_grid_owner` | `rog_map` | adapter 独占 `/rc_esdf/planning_grid` |

普通 `bringup.launch.py` 的默认值仍是 `launch_nav2:=true`，不能与上表混用。

### 导航子系统入口

```bash
ros2 launch ats_nav_bringup rm_navigation_reality_launch.py \
  launch_nav2:=false \
  launch_swerve_mpc:=true \
  planning_grid_owner:=rog_map
```

直接启动子系统时，串口桥、行为树和根仓静态 TF 需由外层负责，不能把它视为
完整实机入口。

## 话题服务与 Action

### 目标和执行

| 名称 | 类型 | Producer | Consumer |
| :--- | :--- | :--- | :--- |
| `/ats_navigate_to_pose` | `ats_navigation_interfaces/action/NavigateToPose` | Goal Manager server | 行为树/测试 client |
| `/ats_goal_manager/planner_goal` | `PlannerGoal` | Goal Manager | MINCO |
| `/minco/planning_status` | `PlannerStatus` | MINCO | Goal Manager |
| `/minco/reference_path_candidate` | `nav_msgs/Path` | MINCO | Goal Manager |
| `/minco/reference_path` | `nav_msgs/Path` | Goal Manager 提交 | 调试/legacy 观察 |
| `/planner/execution_command` | `ExecutionCommand` | Goal Manager 唯一 owner | MPC |
| `/planner/emergency_stop` | `std_msgs/Bool` | Goal Manager 权威心跳 | MPC/串口安全链 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2-free 下 MPC | 串口底盘 |

`ExecutionCommand` 在一条 DDS sample 中携带授权模式、command sequence、goal、
localization epoch、map generation/publication sequence、yaw authority 和 reference。
独立的 `emergency_stop=false` 或旧 `Path` 都不能恢复运动。

### 地图

| 名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/rog_map/get_ground_projection` | `ats_rog_map_interfaces/srv/GetRogMapProjection` | 原子数值快照，禁止反解析 `/rog_map/esdf` 点云 |
| `/rc_esdf/planning_grid` | `nav_msgs/OccupancyGrid` | `rc_esdf_map|ats_rog_map_adapter` 二选一 owner |
| `/rc_esdf/signed_distance_grid` | 数值栅格 | signed distance，free 正、occupied 负、unknown NaN |
| `/rog_map_adapter/status` | `PlanningMapStatus` | publication sequence 与 source/MINCO generation 不同 |
| `/rog_map_adapter/ready` | `std_msgs/Bool` | heartbeat/lease，过期即失效 |

### ROGMap 调试可视化

ROGMap 可发布 `/rog_map/occ`、`/rog_map/inf_occ`、`/rog_map/unk`、
`/rog_map/esdf` 等 `PointCloud2` 调试层。规划端只消费数值服务，不消费这些
点云。P2 配置当前 `visualization.enable: false`；开启前需同步 RViz profile 和
带宽评估，详见根仓优化文档。

## 配置文件

| 文件 | 用途 |
| :--- | :--- |
| `ats_rog_map/config/rog_map_ground_planning_mujoco.yaml` | P2 ROGMap resolution、概率、ESDF、height band |
| `ats_rog_map_adapter/config/rog_map_ground_planning.yaml` | 投影 deadline、融合、footprint、heartbeat |
| `minco_planner/config/minco_planner_reality.yaml` | 实机 JPS/MINCO/yaw/footprint |
| `ats_goal_manager/config/ats_goal_manager_reality.yaml` | 实机 action、lease、terminal success |
| `ats_swerve_mpc/config/ats_swerve_mpc_reality.yaml` | 实机 MPC/舵轮限值 |
| `ats_swerve_mpc/config/ats_swerve_mpc.yaml` | MuJoCo 回归参数，不能与 reality 混用 |
| `ats_nav_bringup/config/reality/nav2_params.yaml` | Nav2 对照 profile |

当前三条自研节点仍从独立 `*_reality.yaml` 加载。将其迁入根仓
`node_params.yaml` 是下一阶段任务，不是已经完成的事实。迁移后 `.msg/.srv/.action`
仍是 schema 权威，YAML 只统一 ROS 参数和接线契约。

## 数据流

```text
/localization + /registered_scan
  -> ats_rog_map
  -> GetRogMapProjection(grid + signed distance + gradient + generation)
  -> ats_rog_map_adapter
       + /map
       + traversability/slope
  -> immutable planning products
  -> minco_planner(JPS -> MINCO S3 -> yaw -> footprint/repair)
  -> candidate reference + PlannerStatus
  -> ats_goal_manager(snapshot/heartbeat/gimbal recheck + retime)
  -> atomic ExecutionCommand
  -> ats_swerve_mpc
  -> body-frame /cmd_vel
```

## 地图与安全契约

- raw occupancy、概率证据、ROG inflation、JPS clearance 和 footprint margin 分层处理，禁止重复膨胀。
- static map 更细或分辨率不整除时，对输出 cell 覆盖的源 footprint 保守聚合，保留 origin 与 yaw。
- 任一来源 occupied 保持 occupied；新鲜明确 free 可消解另一来源 unknown；全来源无证据才输出 unknown。
- unknown 默认按障碍；`robot_unknown_clear_radius` 当前必须保持 `0.0`。
- projection request 使用 steady-clock deadline；timeout 清 pending 并允许新 epoch 重试。
- 单次规划只承诺 MINCO 本地 immutable snapshot 一致，不承诺三个 generation 编号相同。
- map stale/unready、目标不可达、unsafe trajectory 或 MPC 失败必须确定性发布零速度。
- footprint 当前还需要 P4 连续 swept-volume 与实车边界验收；离散采样通过不等于物理零碰撞。

## 目录结构

```text
ats_sentry_nav/
├── ats_nav_bringup/
├── ats_navigation_interfaces/
├── ats_goal_manager/
├── ats_rog_map/
├── ats_rog_map_adapter/
├── ats_rog_map_interfaces/
├── ats_swerve_mpc/
├── minco_planner/
├── point_lio/
├── small_gicp_relocalization/
├── terrain_analysis/
├── terrain_analysis_ext/
├── trajectory_optimizer/
├── ats_nav2_plugins/          # 对照资源
└── fake_vel_transform/        # Nav2 兼容层
```

`sentry_chassis_vel_transform/` 当前是嵌套独立仓库，不能由本仓使用
`git add .` 纳入提交。

## 测试

```bash
colcon test --base-paths src --packages-select \
  ats_rog_map ats_rog_map_adapter minco_planner \
  ats_goal_manager ats_swerve_mpc

colcon test-result --test-result-base build/ats_rog_map_adapter --verbose
```

P2 运行回归从根仓执行：

```bash
PLANNING_GRID_OWNER=rog_map P2_FAULT_CASE=none \
  TEST_PROFILE=red_box GOAL_TIMEOUT=180 \
  scripts/test_mujoco_minco_mpc_chain.sh
```

每种 fault case 使用新的 `ROS_DOMAIN_ID` 和新的 MuJoCo 进程，不能在一次仿真中
串行污染状态。

## 验证边界

- **已验证**：README 所列 launch 默认值、接口名、配置归属已静态核对。
- **已实现未运行**：ATS action/Goal Manager/atomic command 的源码链。
- **未验证**：本次文档更新未执行构建、MuJoCo、红框、故障注入或实车。
- **未完成**：全仓 Nav2-free、总 YAML、ROGMap RViz 新 profile、连续 swept footprint 和落地实车验收。

相关总计划见
[Nav2-free 优化文档](../../docs/项目优化文档/nav2free/README.md)。

## 参考与致谢

本仓包含或适配 Point-LIO、ROGMap、MINCO、Nav2 等开源组件。算法来源、修改边界
和许可证以对应子包文件为准；`参考/` 中的项目只用于溯源，不参与活动构建。
