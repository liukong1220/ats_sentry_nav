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

MuJoCo（2027入口复用`rmuc_2025_mujoco.launch.py`的公共编排）启动MPC，不启动
`fake_vel_transform`；`cmd_vel_arbiter` 直接订阅 `/cmd_vel/autonomy_raw` 并唯一发布
`/cmd_vel/selected`，`twist_to_motion_ctrl` 订阅后输出 `/motion_control`：

```bash
ros2 launch ats_mujoco_sim rmuc_2027_mujoco.launch.py localization_source:=lio
# 可选：solver_mode:=qp_shadow
```

## 当前速度口径与待验收项

2026-11-16正式根profile及独立MPC配置的`max_vx/max_vy/max_wheel_speed`均为`5.0 m/s`、`max_wz=3.0 rad/s`、`max_awz=2.0 rad/s²`，平移加速度`max_ax/max_ay=2.0 m/s²`；`horizon=60`、`dt=0.05 s`。平移叠加自旋时仍受逐轮合成速度约束，轴向上限不意味着对角线或旋转时可以超出轮速预算。

轮半径`0.0425 m`；`motor_max_rpm=1348.1359885`为结合冗余系数1.2反算的暂定等效参数，不是实测电机规格。5 m/s是暂定能力上限，实车8–10 m/s等待测量后再定；MINCO单轮时间缩放仍未启用。

当前终端接近区1 m来自旧低速配置，不能据构建和单测声称5 m/s制动/终点精度通过。直线静止到静止、加减速度2 m/s²时，达到5 m/s至少需12.5 m；当前对称点直线约12.060 m，理想峰值约4.911 m/s。最终记录reference、自主/selected/底盘命令和真值曲线，未触发上限时单列未验证，见[2027方案](../../../docs/nav_acceptance/20261116_2027_final_test_plan.md)。

## LTV-QP 迁移状态

OSQP v1.0.0 源码位于 `../third_party/osqp`，以静态库链接。QDLDL v0.1.8
（commit `138fdac58b9cd1c4137ff1b99152c8108a6cff5b`）内置于 `../third_party/qdldl`，
由 `FETCHCONTENT_SOURCE_DIR_QDLDL` 指向该目录，配置时不再从 GitHub 拉取。

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
# 在当前工作区根目录执行
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select ats_swerve_mpc \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
cd build/ats_swerve_mpc
ctest --output-on-failure
```

`FETCHCONTENT_SOURCE_DIR_QDLDL` 在本包 `CMakeLists.txt` 中被强制指向内置 QDLDL。
不要把它改回空值或 `build/` 下的 git 缓存，否则配置会再次访问 GitHub。

MuJoCo最终回归按上述2027方案显式选择场景和起终点；通用runner的旧默认2025与红框不再作为当前最终入口。单测、空场5 m/s执行器诊断与导航闭环分别报告，当前2027套件尚未通过。
