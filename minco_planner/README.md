# minco_planner

ATS 四舵轮哨兵的规划节点。它在 `/rog_map_adapter/planning_snapshot`（占据与带符号距离同一快照）上生成
带时间戳、带独立 yaw 的参考轨迹，经矩形足迹门禁后作为候选交给 `ats_goal_manager` 复核
提交，再由`ats_swerve_mpc`跟踪。当前三套部署profile车体长×宽`0.581 x 0.571 m`、每边余量`0.02 m`；源码构造器回退值不作为部署配置。

```text
ats_goal_manager --PlannerGoal--> minco_planner
  目标位姿准入
  -> JPS / A* 图搜索（净空梯子）
  -> 路径预处理（去重、共线合并、足迹感知 shortcut、内角 fillet）
  -> 引导加密、弹性带平滑、ESDF 内点修正
  -> 整条轨迹 MINCO 联合优化（q + T，一次 L-BFGS，时间预算）+ 分段时间缩放
  -> planYaw（clearance_aware yaw -> 轮速局部放慢 -> 窄通道转向局部放慢）
  -> 矩形足迹门禁（逐点 + 扫掠）
     不通过时：足迹 yaw 不动点补解、无 ESDF 基线兜底、局部碰撞修复、终端 yaw 重定位、脱困前缀
  -> 名义 commit 几何门禁
  -> /minco/reference_path_candidate + /minco/planning_status --> ats_goal_manager
ats_goal_manager --/minco/reference_path--> ats_swerve_mpc -> ... -> cmd_vel_arbiter -> /cmd_vel/selected
```

正式运行链中 `planner_manages_emergency_stop: false`：本节点只发布候选与状态，已提交
reference 和急停由 Goal Manager 发布。速度链细节见仓库根 README。

## 处理阶段

### 目标位姿准入

`goal_pose_admission_enabled` 默认开启。目标位姿的矩形足迹不可行时，在 Goal Manager
成功容差（按 shrink 系数收缩后）内搜索一个通过同一足迹判定的终点；找不到则保留原目标，
走既有失败路径。

### 图搜索

- 默认统一混合图搜索（`search_algorithm: hybrid`：均匀区跳点剪枝，近墙/窄通道自动退化为加权 A*），也可设为 `jps` 或 `astar`。
- 首选净空取 `jps_safe_distance` 与全 yaw 外接半径 `hypot(L/2 + m, W/2 + m)` 的较大者，
  当前profile足迹下约为`0.435592 m`；YAML中的`jps_safe_distance=0.36`是配置值，不等于首选搜索实际净空。
- `clearance_relaxation_enabled` 开启时，首选档无路再退到下限档。未配置
  `search_clearance_floor`（`< 0`）时，下限为内切半宽 `min(L, W)/2 + m` 加当次栅格的半格对角线。
- `endpoint_clearance_relaxation_enabled` 只在目标位姿已通过足迹准入时生效，允许目标格净空
  低于下限；整条轨迹仍由足迹门禁裁定。

### 路径预处理与引导

`PathGeometryPreprocessor` 去除重复点、合并近共线点、处理短段，并做足迹感知 shortcut 与
内角 fillet（`path_fillet_radius > 0` 时）。随后按 `guide_control_point_spacing` 加密引导，
`guide_smoothing_iterations > 0` 时做受偏离与净空约束的弹性带平滑，再用 RC-ESDF 修正内点
（有 yaw 参考时使用旋转后的矩形采样点）。

### 整条轨迹 MINCO 联合优化

在加密引导上按 `joint_waypoint_spacing` 取内点，以全部内点 `q` 与全部段时长 `T = exp(tau)`
为变量，一次 L-BFGS 最小化：jerk 能量 + 时间 + 中心/足迹 ESDF 罚（无 ESDF 时改为引导管道罚）
+ 速度/加速度/横向加速度罚。迭代数受 `joint_max_iterations` 与 `joint_time_budget_ms` 限制；
数值失败返回空轨迹（fail-closed），约束残差交给后续时间缩放和足迹门禁处理。之后按
`max_velocity`、`max_acceleration`、`max_jerk`、`max_lateral_acceleration` 做分段时间缩放并重解。

### yaw 与局部时间缩放（planYaw）

每次生成或重解轨迹后统一执行：

1. 标注与 yaw 无关的位置净空；
2. 按 `yaw_mode` 生成 yaw；
3. 轮速局部放慢（`wheel_speed_time_scaling_limit > 0` 时）；
4. 窄通道转向局部放慢（`narrow_turn_speed_limit > 0` 时）；
5. 改写为与 yaw 相关的足迹净空，供 yaw authority 与质量评估使用。

`yaw_mode` 代码默认 `goal_heading`，实机与 MuJoCo profile 均设为 `clearance_aware`。可选值：
`clearance_aware`、`goal_heading`、`path_tangent`、`hold`。

`clearance_aware` 的行为：

- 开阔区域平滑转向目标朝向，保留舵轮横移能力；
- 位置净空 `<= narrow_clearance_enter` 进入窄段、`>= narrow_clearance_exit` 才退出（滞回），
  窄段内对齐路径切线；
- `yaw_tangent_symmetry_order` 为 2 时正向/反向切线等价，为 4 时再加 `±pi/2`，选转角最小者。
  4 阶只对正方形足迹（长宽差 `<= 1e-3 m`）生效，否则告警并回退 2 阶；
- `yaw_narrow_gap_bridge_time > 0` 时，两段窄通道之间不长于该时长的开阔间隙按窄段处理，
  避免参考 yaw 来回摆；
- 角速度受 `yaw_rate_limit` 限制；`yaw_acceleration_limit > 0` 时再按可刹停的二阶跟踪限制角加速度。

两种局部时间缩放只改时间参数化，位置和 yaw 序列不变；速度按系数 `f` 缩放、加速度按 `f^2`
缩放，系数变化率受 `wheel_speed_time_scaling_change_rate` 限制。轮速按四个轮心
`(±offset_x, ±offset_y)` 计算平移与转向叠加后的线速度。窄通道转向判据为：位置净空
`<= narrow_clearance_enter`、`|yaw_rate| > narrow_turn_yaw_rate_threshold` 且平移速度超过
`narrow_turn_speed_limit`。

### 候选与足迹安全链

1. 中心 ESDF 候选：MINCO 解出后 planYaw，再过矩形足迹门禁；
2. 足迹感知候选（`esdf_footprint_optimization_enabled`）：以上一候选的 yaw 为参考重解；门禁不通过时
   按 `footprint_yaw_refinement_rounds` 用新 yaw 再解（碰撞数变多即停）；
3. 仍不安全时尝试不带 ESDF 的 JPS-MINCO 基线；
4. 局部碰撞修复（`local_repair_enabled`）：移动冲突引导点后重解 MINCO 并重新过门禁；
5. 终端 yaw 重定位（`terminal_yaw_relocation_enabled`）：冲突只落在末端 `terminal_yaw_relocation_window_length`
   弧长内时，把终点原地转向挪到更早的位置并重新过门禁；
6. 脱困前缀（`escape_from_contact_enabled`，默认关闭）：起点已处于擦边接触时，允许有界的冲突前缀；
   `escape_from_contact_max_contact_depth > 0` 时前缀须在内缩足迹下无冲突；
7. 名义 commit 几何门禁（`commit_max_length_ratio` / `commit_max_lateral_deviation_m`，默认关闭）
   拒绝相对起终点弦过大的绕行，脱困提交跳过该门。

足迹门禁对每个参考点做矩形栅格判定，并在相邻点之间按最远角点位移不超过
`swept_max_corner_step_cells` 个栅格自适应细分（扫掠检查）。这仍是采样判定，不是连续体证明。
`publish_unsafe_trajectory` 默认 `false`，未通过门禁的轨迹不发布。

提交后，节点按 `runtime_safety_recheck_hz` 对已提交 reference 前 `runtime_safety_horizon_sec`
窗口重复做足迹检查。`retain_safe_reference_on_snapshot_change` 开启时，地图快照变化后若剩余参考
仍安全则保留，而不是作废重规划。

## 当前部署参数

2026-11-16有效值按实际launch合并：实机为根`src/ats_sentry_bringup/params/node_params.yaml`；MuJoCo先叠加`rmuc_2025_navigation.yaml`，2027再叠加`rmuc_2027_navigation.yaml`与launch内联覆盖。旧文档的大表混用代码回退值和部署值，现仅列当前行为所需参数；其余声明和默认值直接核对`src/nodes/minco_planner_node.cpp`及对应参数结构体。

| 参数/契约 | 当前有效值与边界 |
| --- | --- |
| `footprint_length/width` / `footprint_safety_margin` | `0.581/0.571 m` / `0.02 m`，三套一致 |
| `jps_safe_distance` | `0.36 m`；首选按全yaw半径约0.435592 m抬高，净空梯子和最终定向足迹门继续生效 |
| `reference_speed` / `max_velocity` | 均`5.0 m/s`，暂定上限，受曲率、制动、净空与执行器限制 |
| 起点速度保护 | 正的`max_velocity`同步到内部`initial_state_max_speed`，不再固定截断到2 m/s |
| `max_acceleration` / `max_lateral_acceleration` | `2.5 / 1.5 m/s²`；MPC平移加速度为2.0 m/s²，尚需完整高速度可执行性验证 |
| `yaw_mode` / `yaw_rate_limit` / `yaw_acceleration_limit` | `clearance_aware` / `3.0 rad/s` / `2.0 rad/s²` |
| `yaw_tunnel_enabled` / `spin_enabled` | 均`true`；安全开阔段请求自旋，隧道/执行约束优先 |
| `yaw_tangent_symmetry_order` | 矩形足迹用2阶，不沿用旧正方形4阶说明 |
| `wheel_speed_time_scaling_limit` | 三套未启用，不恢复旧1.45 m/s单轮时间缩放 |
| `planner_manages_emergency_stop` | 正式链为`false`，Goal Manager拥有已提交reference与急停 |
| `publish_unsafe_trajectory` | `false`，失败不发布unsafe reference |
| `local_repair_enabled` | 根实车profile为`false`，MuJoCo覆盖`true`；属于尚未统一的能力项 |
| ROG / terrain车高门 | 均0.249 m，H2净空、M路线、H3实车仍未验收 |

前述planYaw算法支持局部时间缩放，不表示当前已打开该功能。单轮5 m/s限制由MPC和底盘执行链继续拥有，平移叠加自旋不能越过轮速预算。ROG高度门属于地图侧，不由本规划器重复膨胀。

## 正式话题与坐标系

| 接口 | 当前作用 |
| --- | --- |
| `/rog_map_adapter/planning_snapshot` | 数值规划快照，包含occupancy、signed distance、梯度、来源generation、publication sequence与localization epoch |
| `/ats_goal_manager/planner_goal` | Goal Manager的`PlannerGoal`请求；正式链不直接订阅Nav2的`/plan` |
| `/minco/planning_status`、`/minco/reference_path_candidate` | 规划状态与候选交给Goal Manager |
| `/minco/reference_path`、`/planner/execution_command`、`/planner/emergency_stop` | Goal Manager复核、重定时后统一提交；ready恢复不能复活旧reference |
| `/minco/raw_path`及调试guide | 诊断输出，不作为另一条目标或安全权威 |

规划和控制使用`odom`，Goal Manager保存`map`目标并按当前`map->odom`转换。JPS、MINCO clearance、足迹与repair在单次规划中使用同一不可变本地snapshot；不同generation编号不要求相等，端到端绑定仍须专门验证。旧`grid_topic=traversability_grid`或`/rc_esdf/planning_grid`不是正式规划入口。

## 构建与验证

在当前工作区根目录执行，保留Release和活动源码范围：

```bash
source /opt/ros/humble/setup.bash
./build.sh --packages-select minco_planner
python3 scripts/validate_navigation_config.py
```

独立`minco_planner.launch.py`的默认参数路径仍指向不存在的`config/minco_planner.yaml`，必须显式传入完整参数（例如包内`config/minco_planner_reality.yaml`）。正式bringup会传根参数文件，不依赖该缺省路径。

最终导航按[2027方案](../../../docs/nav_acceptance/20261116_2027_final_test_plan.md)通过 `rmuc_2027_mujoco.launch.py` 与 `scripts/s3_mujoco_acceptance.py` 执行真实 B 往返、基地到基地、南坡与六隧道验证。当前已新增贴墙大角度终端转向的 ESDF 侧移掉头、无停车终点修正保持运行参考与 `RELIABLE` 参考轨迹发布契约；物理接触与实车仍待独立验收。

MINCO许可证与归属见[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
