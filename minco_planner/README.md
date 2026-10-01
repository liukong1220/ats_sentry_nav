# minco_planner

ATS 四舵轮哨兵的规划节点。它在 `/rc_esdf/planning_grid` 与同一快照的 RC-ESDF 上生成
带时间戳、带独立 yaw 的参考轨迹，经矩形足迹门禁后作为候选交给 `ats_goal_manager` 复核
提交，再由 `ats_swerve_mpc` 跟踪。车辆足迹默认 `0.58 x 0.58 m`，安全余量 `0.05 m`。

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

- 默认 JPS（`search_algorithm: jps`），`astar` 切换为 A*；`astar_fallback` 让 JPS 失败时回退 A*。
- 首选净空取 `jps_safe_distance` 与全 yaw 外接半径 `hypot(L/2 + m, W/2 + m)` 的较大者，
  默认足迹下为 `0.481 m`。
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

## 参数

默认值取自代码（`declare_parameter` 及对应参数结构体）。部署值以 profile 为准：实机为根仓
`src/ats_sentry_bringup/params/node_params.yaml` 的 `minco_planner` 段，MuJoCo 为
`src/sim/ats_mujoco_sim/config/rmuc_2025_navigation.yaml`。

### 话题与坐标系

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `grid_topic` | `traversability_grid` | 规划栅格输入（RELIABLE + TRANSIENT_LOCAL） |
| `goal_topic` | `goal_pose` | 直接 `PoseStamped` 目标；空字符串不订阅 |
| `goal_request_topic` | `""` | Goal Manager 的 `PlannerGoal` 请求；空字符串不订阅 |
| `planner_status_topic` | `""` | `PlannerStatus` 输出；空字符串不发布 |
| `raw_path_topic` | `minco/raw_path` | 图搜索原始路径 |
| `reference_path_topic` | `minco/reference_path` | 仅 `planner_manages_emergency_stop: true` 时由本节点发布 |
| `candidate_reference_path_topic` | `""` | 交给 Goal Manager 复核的候选 reference |
| `preprocessed_guide_topic` | `/minco/preprocessed_guide` | 预处理后引导线（诊断） |
| `esdf_refined_guide_topic` | `/minco/esdf_refined_guide` | ESDF 修正后引导线（诊断） |
| `debug_marker_topic` | `minco/debug_markers` | RViz MarkerArray；空字符串不发布 |
| `map_ready_topic` | `""` | 地图 ready 心跳；空字符串视为地图始终 ready |
| `emergency_stop_topic` | `/planner/emergency_stop` | 仅 `planner_manages_emergency_stop: true` 时发布 |
| `global_frame` | `map` | 目标 `frame_id` 为空且栅格无 frame 时的缺省全局 frame |
| `robot_frame` | `base_link` | 起点 TF 查询 frame |

### 运行期安全与所有权

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `planner_manages_emergency_stop` | `true` | `true` 时本节点直接发布 reference 与急停；正式运行链设为 `false` |
| `publish_unsafe_trajectory` | `false` | 是否发布未通过足迹门禁的轨迹 |
| `map_ready_timeout_sec` | `3.0` | ready 心跳超时（下限 0.1） |
| `emergency_stop_heartbeat_period_sec` | `0.1` | 急停心跳与 ready watchdog 周期（下限 0.02） |
| `runtime_safety_recheck_hz` | `10.0` | 已提交 reference 的运行期复检频率（下限 0.1） |
| `runtime_safety_horizon_sec` | `1.0` | 运行期复检的前向时间窗 |
| `retain_safe_reference_on_snapshot_change` | `false` | 快照变化时保留仍安全的已提交参考 |
| `retain_reference_horizon_sec` | `0.0` | 保留判定的时间窗；`<= 0` 检查完整剩余段 |
| `body_yaw_follow_clearance` | `0.55` | 轨迹任一点足迹净空不大于该值时 yaw authority 取 BODY_YAW_FOLLOW |
| `force_body_yaw_follow` | `false` | 强制 BODY_YAW_FOLLOW |
| `commit_max_length_ratio` | `0.0` | 名义提交的轨迹长度/起终点弦长上限；`<= 0` 关闭 |
| `commit_max_lateral_deviation_m` | `0.0` | 名义提交相对起终点弦的最大横偏；`<= 0` 关闭 |

### 目标位姿准入

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `goal_pose_admission_enabled` | `true` | 目标足迹不可行时在容差域内另选终点 |
| `goal_admission_position_tolerance` | `0.08` | 位置容差（m），与 Goal Manager 一致 |
| `goal_admission_yaw_tolerance` | `0.15` | yaw 容差（rad），与 Goal Manager 一致 |
| `goal_admission_position_shrink` | `0.75` | 实际使用的位置容差比例 |
| `goal_admission_yaw_shrink` | `0.60` | 实际使用的 yaw 容差比例 |
| `goal_admission_extra_margin` | `0.03` | 第一档额外膨胀余量（m） |
| `goal_admission_position_step` | `0.02` | 位置候选步长（m） |
| `goal_admission_yaw_samples` | `5` | yaw 候选数（含原始 yaw，下限 1） |

### 图搜索

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `search_algorithm` | `jps` | `jps` 或 `astar` |
| `astar_fallback` | `true` | JPS 失败时回退 A* |
| `obstacle_value_threshold` | `50` | 栅格值不小于该值视为占据（搜索、门禁、修复共用） |
| `unknown_is_obstacle` | `false` | unknown 是否视为占据；实机 profile 设为 `true` |
| `allow_diagonal` | `true` | 允许对角扩展 |
| `jps_max_expanded_nodes` | `-1` | 扩展节点上限；`<= 0` 不限制 |
| `jps_safe_distance` | `0.0` | 首选搜索净空；低于全 yaw 外接半径时自动抬高并告警 |
| `clearance_relaxation_enabled` | `true` | 首选档无路时退到下限档 |
| `endpoint_clearance_relaxation_enabled` | `true` | 目标位姿已过准入时放宽目标格净空 |
| `search_clearance_floor` | `-1.0` | 下限档净空；`< 0` 自动取内切半宽 + 半格对角线 |

### 路径预处理

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `path_duplicate_epsilon` | `1e-6` | 重复点判定距离（m） |
| `path_collinear_lateral_tolerance` | `0.025` | 共线合并的横向容差（m） |
| `path_short_segment_length` | `0.05` | 短段长度阈值（m） |
| `path_corner_angle_threshold_rad` | `0.20` | 拐角判定角度（rad） |
| `path_footprint_aware_shortcut_enabled` | `true` | 足迹感知 shortcut |
| `path_fillet_radius` | `0.0` | 内角 fillet 半径（m）；`0` 保留尖角 |
| `path_fillet_arc_samples` | `1` | 每个 fillet 弧的采样点数 |

### 引导平滑与 ESDF 修正

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `guide_control_point_spacing` | `0.0` | 引导加密间距（m）；`0` 不加密 |
| `guide_smoothing_iterations` | `0` | 弹性带平滑迭代数；`0` 关闭 |
| `guide_smoothing_alpha` | `0.5` | 拉直系数 |
| `guide_smoothing_fidelity` | `0.02` | 拉回原引导的系数 |
| `guide_smoothing_max_deviation` | `0.50` | 平滑偏离原引导上限（m） |
| `guide_smoothing_min_clearance` | `0.42` | 平滑后中心净空下限（不低于原净空与该值的较小者） |
| `esdf_obstacle_optimization_enabled` | `true` | 中心 ESDF 内点修正 |
| `esdf_obstacle_clearance` | `0.45` | 中心净空目标（trigger/target 为 0 时使用） |
| `esdf_obstacle_trigger_clearance` | `0.0` | 触发修正的净空 |
| `esdf_obstacle_target_clearance` | `0.0` | 修正目标净空 |
| `esdf_obstacle_max_iterations` | `6` | 修正轮数 |
| `esdf_obstacle_control_point_spacing` | `0.30` | 插入内部控制点的间距（m） |
| `esdf_obstacle_max_step` | `0.10` | 单轮最大位移（m） |
| `esdf_obstacle_max_deviation` | `0.50` | 相对引导的最大偏离（m） |
| `esdf_obstacle_trust_region` | `0.10` | 信赖域（m） |
| `esdf_obstacle_backtracking_steps` | `4` | 回溯步数 |
| `esdf_obstacle_smoothing_weight` | `0.25` | 修正量邻点平滑权重 |
| `esdf_footprint_optimization_enabled` | `true` | 足迹感知候选（带 yaw 参考的第二遍） |
| `esdf_footprint_clearance` | `0.10` | 足迹采样点净空目标 |
| `esdf_footprint_trigger_clearance` | `0.0` | 足迹修正触发净空 |
| `esdf_footprint_target_clearance` | `0.0` | 足迹修正目标净空 |
| `esdf_footprint_sample_spacing` | `0.10` | 足迹边采样间距（m） |

### MINCO 联合优化与时间缩放

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `reference_speed` | `1.5` | 巡航速度（m/s），用于时间初值；联合优化软速度上限不高于它 |
| `min_segment_time` | `0.05` | 最小段时长（s） |
| `sample_spacing` | `0.12` | 输出轨迹采样间距（m，按 `reference_speed` 折算为时间步） |
| `max_velocity` | `2.0` | 时间缩放速度上限（m/s） |
| `max_acceleration` | `2.5` | 时间缩放加速度上限（m/s^2） |
| `max_jerk` | `0.0` | jerk 上限；`<= 0` 不限制 |
| `max_lateral_acceleration` | `1.5` | 横向加速度上限（m/s^2） |
| `max_time_scaling_iterations` | `5` | 分段时间缩放轮数 |
| `time_scaling_factor` | `1.25` | 每轮最小缩放倍数 |
| `joint_waypoint_spacing` | `1.0` | 联合优化内点间距（m） |
| `joint_samples_per_piece` | `16` | 每段罚项采样数 |
| `joint_max_piece_time` | `3.0` | 段时长上限（s） |
| `joint_energy_weight` | `1.0` | jerk 能量权重 |
| `joint_time_weight` | `20.0` | 总时间权重 |
| `joint_obstacle_weight` | `1e6` | 中心/足迹 ESDF 罚权重 |
| `joint_velocity_weight` | `1e3` | 速度罚权重 |
| `joint_acceleration_weight` | `1e3` | 加速度罚权重 |
| `joint_lateral_weight` | `1e3` | 横向加速度罚权重 |
| `joint_guide_weight` | `1e6` | 无 ESDF 时引导管道罚权重；`<= 0` 关闭 |
| `joint_guide_max_deviation` | `0.10` | 引导管道半宽（m） |
| `joint_center_clearance` | `0.42` | 中心 ESDF 净空目标（m），部署时与 `jps_safe_distance` 对齐 |
| `joint_footprint_clearance` | `0.03` | 足迹采样点净空目标（m），有 yaw 参考时生效 |
| `joint_footprint_edge_samples` | `1` | 每条足迹边的采样数（1 为边中点） |
| `joint_g_epsilon` | `1e-5` | L-BFGS 梯度收敛阈值 |
| `joint_max_iterations` | `200` | L-BFGS 迭代上限 |
| `joint_time_budget_ms` | `15.0` | 联合优化墙钟预算（ms） |

### yaw 与局部时间缩放

c41d922 新增的 `yaw_tangent_symmetry_order`、`yaw_narrow_gap_bridge_time`、`yaw_acceleration_limit`、
`wheel_speed_time_scaling_*`、`narrow_turn_*` 默认均保持原行为（关闭）。目前只有 MuJoCo profile
启用：symmetry order `4`、gap bridge `1.0 s`、yaw 角加速度 `3.0 rad/s^2`、轮速上限 `1.45 m/s`
（偏置 `0.27 / 0.27 m`、变化率 `2.0 /s`）、窄通道转向限速 `0.8 m/s`（阈值 `0.2 rad/s`）。
实机 profile 未设置这些参数。

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `yaw_mode` | `goal_heading` | `clearance_aware` / `goal_heading` / `path_tangent` / `hold` |
| `yaw_rate_limit` | `2.5` | yaw 角速度上限（rad/s），终端原地转向共用 |
| `yaw_acceleration_limit` | `0.0` | yaw 角加速度上限（rad/s^2）；`<= 0` 关闭 |
| `narrow_clearance_enter` | `0.55` | 进入窄段的位置净空（m），窄通道转向判据共用 |
| `narrow_clearance_exit` | `0.70` | 退出窄段的位置净空（m） |
| `yaw_tangent_symmetry_order` | `2` | 窄段切线对称阶数；`4` 仅正方形足迹有效 |
| `yaw_narrow_gap_bridge_time` | `0.0` | 窄段间开阔间隙桥接时长（s）；`0` 关闭 |
| `wheel_speed_time_scaling_limit` | `0.0` | 单轮线速度上限（m/s）；`<= 0` 关闭 |
| `wheel_speed_time_scaling_offset_x` | `0.0` | 轮心 x 半轴偏置（m），与 MPC `wheel_base_x` 同义 |
| `wheel_speed_time_scaling_offset_y` | `0.0` | 轮心 y 半轴偏置（m），与 MPC `wheel_base_y` 同义 |
| `wheel_speed_time_scaling_change_rate` | `1.0` | 缩放系数变化率上限（1/s），两种局部缩放共用 |
| `narrow_turn_speed_limit` | `0.0` | 窄通道边平移边转向时的平移限速（m/s）；`<= 0` 关闭 |
| `narrow_turn_yaw_rate_threshold` | `0.2` | 窄通道转向判据的 `|yaw_rate|` 阈值（rad/s） |
| `terminal_yaw_sample_period` | `0.10` | 终端原地转向采样周期（s） |
| `terminal_yaw_relocation_enabled` | `true` | 终端 yaw 重定位 |
| `terminal_yaw_relocation_max_candidates` | `6` | 重定位候选位置数（下限 1） |
| `terminal_yaw_relocation_window_length` | `1.0` | 允许重定位的末端弧长窗口（m） |
| `footprint_yaw_refinement_rounds` | `2` | 足迹 yaw 不动点补解轮数；`0` 关闭 |

### 足迹、修复与脱困

| 参数 | 默认值 | 含义 |
| :--- | :--- | :--- |
| `footprint_length` | `0.58` | 车体长（m） |
| `footprint_width` | `0.58` | 车体宽（m） |
| `footprint_safety_margin` | `0.05` | 每边安全余量（m） |
| `swept_max_corner_step_cells` | `0.5` | 扫掠检查中最远角点单步位移上限（栅格数） |
| `local_repair_enabled` | `true` | 局部碰撞修复；实机 profile 设为 `false` |
| `local_repair_max_iterations` | `2` | 修复轮数 |
| `local_repair_search_radius` | `0.35` | 引导点位移搜索半径（m）；低于内切半宽时告警且修复无候选 |
| `escape_from_contact_enabled` | `false` | 脱困前缀（实验，默认 fail-closed） |
| `escape_from_contact_max_head_offset` | `0.10` | 冲突允许出现的最靠前弧长（m） |
| `escape_from_contact_max_prefix_length` | `0.40` | 前缀平移上限（m） |
| `escape_from_contact_max_prefix_yaw_sweep` | `pi/2` | 前缀累计转角上限（rad） |
| `escape_from_contact_max_prefix_points` | `64` | 前缀点数上限 |
| `escape_from_contact_max_contact_depth` | `0.0` | 接触深度上限（m）；`0` 不做内缩足迹检查 |

## 构建与测试

在工作区根目录构建，必须限制 `--base-paths src`：

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select minco_planner
source install/setup.bash
```

GTest 通过 `ctest` 在构建目录运行：

```bash
cd /home/ats/ATS_2026_snetry_test/build/minco_planner
ctest -R '^test_' --output-on-failure
```

`ctest -N` 列出 21 个 `test_*` GTest 和 ament lint 测试。其中 copyright、cpplint、clang_format
为已知历史失败，完整 `ctest` 不是全绿；功能验证以 `test_*` 结果为准。

独立启动：

```bash
ros2 launch minco_planner minco_planner.launch.py params_file:=<参数 YAML>
```

launch 的默认 `params_file` 指向 `config/minco_planner.yaml`，该文件已不在源码中，独立启动时
需显式传入（例如 `config/minco_planner_reality.yaml`）。正式入口由根仓 bringup 传入
`node_params.yaml`。

MINCO 的许可证与归属见 `THIRD_PARTY_NOTICES.md`。
