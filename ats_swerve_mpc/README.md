# ats_swerve_mpc

ATS 四舵轮独立转向底盘的全向 SE(2) MPC 跟踪器。正式控制器为约束 iLQR（`solver_mode: ilqr`）。

状态为世界系 `[x, y, yaw]`，控制为车体系 `[vx, vy, wz]`，以 `geometry_msgs/Twist` 发布到
`command_topic`（默认 `/cmd_vel/autonomy_raw`）。模型保留真实横移能力，不引入差速 ICR 或曲率约束。

## 跟踪与约束

输入为 Goal Manager 提交、带时间戳的 `/minco/reference_path`。实测位置投影到最近的可行轨迹段，
禁止进度回退，前向搜索窗口有界；横向误差大时压缩 horizon 进度，优先回到轨迹。位姿时间戳用于
插值和车体系前馈速度，短时 preview 补偿指令延迟。

每个 MPC 步同时约束：

- 车体速度与加速度：`max_vx`、`max_vy`、`max_wz`、`max_ax`、`max_ay`、`max_awz`；
- 逐轮约束：按 `wheel_base_x/y` 计算四个轮心，限制轮速 `max_wheel_speed`、轮加速度
  `max_wheel_acceleration` 和舵角速率 `max_steer_rate`。`max_wheel_speed` / `max_steer_rate`
  未显式设置时，由 `wheel_radius`、`motor_max_rpm`、`steer_max_rpm`、减速比和
  `actuator_redundancy` 换算。

急停、目标完成、缩放后的轨迹 deadline 均输出零速度；执行授权（`/planner/execution_command`）、
定位、地图 ready、无参考轨迹等情况同样输出零速度。

## 速度链

实机（`ats_nav_bringup/launch/navigation_launch.py` 默认值）：

```text
ats_swerve_mpc -> /cmd_vel/autonomy_raw -> fake_vel_transform -> /cmd_vel/autonomy_gimbal
  -> chassis_vel_transform -> /cmd_vel/autonomy -> cmd_vel_arbiter -> /cmd_vel/selected
```

MuJoCo（`ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py`）始终启动 MPC，不启动
`fake_vel_transform`；`cmd_vel_arbiter` 直接订阅 `/cmd_vel/autonomy_raw` 并唯一发布
`/cmd_vel/selected`，`twist_to_motion_ctrl` 订阅后输出 `/motion_control`：

```bash
ros2 launch ats_mujoco_sim rmuc_2025_mujoco.launch.py
# 可选：solver_mode:=qp_shadow
```

## LTV-QP 迁移状态

OSQP v1.0.0 源码位于 `../third_party/osqp`，以静态库链接；其 QDLDL 依赖通过 FetchContent
获取（v0.1.8），已缓存在 `build/ats_swerve_mpc/_deps`。

`solver_mode` 取值：

| 值 | 行为 |
| :--- | :--- |
| `ilqr`（默认） | iLQR 唯一产生控制输出 |
| `qp_shadow` | 预分配 OSQP 后端，只读取同周期冻结的输入快照做诊断与遥测，不发布 QP Twist；setup 失败时保持 iLQR |
| `qp` | 保留值，节点启动时拒绝 |

- `Se2Model`：iLQR 与 LTV-QP 共用的 SE(2) 动力学、Jacobian 与 rollout；
- `ZeroSpeedGuard`：轮速接近零、方向未定义时禁止舵角线性化（带滞回），不会凭空生成原地舵角指令；
- `LtvQpBuilder`：围绕 iLQR 名义 rollout 构造凸跟踪 QP，包含 SE(2) 线性动力学、车体速度边界和
  控制增量边界；轮速范数与舵角约束尚未线性化进 QP。

`qp_max_iterations`、`qp_time_limit_ms`、`qp_max_primal_residual`、`qp_max_dual_residual`、
`qp_max_tracking_slack`、`qp_max_hard_constraint_violation` 是 shadow 后端的准入上限，不是性能实测结论。
QP 路径目前仅用于诊断，不能写成实车求解器。急停、定位、授权租约、reference 新鲜度和零速度
行为仍由现有节点负责。

## 构建与测试

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select ats_swerve_mpc
cd build/ats_swerve_mpc
ctest --output-on-failure
```

重新配置本包时不要使用 `-UFETCHCONTENT_SOURCE_DIR_QDLDL`。

MuJoCo 闭环回归脚本位于工作区根目录：

```bash
cd /home/ats/ATS_2026_snetry_test
scripts/test_mujoco_minco_mpc_chain.sh
```
