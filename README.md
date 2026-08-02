# ats_sentry_nav

## 简介

ATS 哨兵的活动定位、地图、规划、目标管理与全向控制仓。Nav2 server、costmap、plugin、
`/plan` 和 Nav2 command selection 已从正式链移除；行为入口使用 ATS
`NavigateToPose` action。

## 模块

- `point_lio`：保留 `/localization` 与 `/registered_scan`。
- `ats_rog_map`：概率占据、膨胀、3D ESDF 和数值地面投影服务。
- `ats_rog_map_adapter`：融合 ROGMap、terrain 与静态图，唯一发布 planning grid。
- `ats_rc_esdf`：中立 signed-distance、unknown、gradient 与静态图融合接口。
- `minco_planner`：JPS、MINCO S3、独立 yaw、footprint gate 和 Local Collision Repair。
- `ats_goal_manager`：ATS action 生命周期、snapshot/heartbeat 复核和急停。
- `ats_swerve_mpc`：全向 SE2 MPC，输出车体系 `[vx, vy, wz]`。

## 依赖

ROS 2 Humble、Eigen3、PCL、OpenCV、yaml-cpp 与根仓的
`ats_navigation_interfaces`/`ats_rog_map_interfaces`。活动包不依赖 Nav2。

## 构建

从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --symlink-install \
  --packages-select ats_rc_esdf ats_nav_bringup ats_rog_map ats_rog_map_adapter \
  minco_planner ats_sentry_nav --parallel-workers 1
source install/setup.bash
```

必须使用 `colcon` 和 `--base-paths src`；不要让同名参考包进入构建图。

## 启动

子系统由根仓 `ats_sentry_bringup/launch/bringup.launch.py` 或
`real_robot_navigation.launch.py` 编排。导航 launch 接收根拥有的 `node_params.yaml`，
固定 `planning_grid_owner:=rog_map`，并启动 static map、ROGMap、adapter、MINCO、Goal
Manager 和 MPC；没有 Nav2 lifecycle 或 composition server。

## 接口

- `/ats_navigate_to_pose`：Goal Manager server 的唯一导航 action。
- `/rog_map/get_ground_projection`：ROGMap 数值 projection；禁止从 `/rog_map/esdf` 解析距离。
- `/rc_esdf/planning_grid`：`ats_rog_map_adapter` 唯一 owner。
- `/planner/emergency_stop` 与 `/planner/execution_command`：Goal Manager 的安全和执行契约。
- `/cmd_vel_mpc`：MPC 输出，传至唯一底盘 bridge。

## 配置

正式参数由根仓 `src/ats_sentry_bringup/params/node_params.yaml` 提供，按 adapter、ROGMap、
MINCO、Goal Manager、MPC 顺序加载。ROGMap core 用显式 ROS parameter struct 构造；正式
profile 禁止 `map_config_file` 作为第二参数来源。地图细栅格融合保留 origin/yaw 和保守 occupied
语义。

## 架构

```text
Point-LIO -> ROGMap -> numerical projection -> adapter -> RC-ESDF
         -> JPS -> MINCO S3/yaw/footprint/repair -> Goal Manager -> SE2 MPC
```

状态为世界系 `[x, y, yaw]`，控制为车体系 `[vx, vy, wz]`；不得迁入差速、ICR 或 `vy=0`。
source、publication 与 MINCO snapshot generation 分属不同编号域。

## 验证与限制

本轮已通过 RC-ESDF 语义场景、ROGMap/adapter、MINCO、Goal Manager action 与 behavior 相关
单测；集成 MuJoCo 使用数值 projection，验证无 Nav2 server、无 `/plan`、唯一 planning grid/
速度 owner 和故障零速度。P4 连续 swept footprint 与实车验证尚未完成。
