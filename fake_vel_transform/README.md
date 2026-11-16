# fake_vel_transform

保留fake-yaw与速度frame兼容层：节点根据最新odom yaw维护`robot_base_frame -> fake_robot_base_frame`的TF，并将输入Twist转换后发布。当前源码只订阅速度与里程计；不再订阅`local_plan`，也不靠Nav2路径推断命令时间戳或控制器活动。

雷达已固定底盘，`gimbal_yaw_odom`沿用名字表示底盘系。通用`bringup.launch.py`默认关闭fake/chassis两层；`real_robot_navigation.launch.py`为小陀螺保留两层默认True。MuJoCo不启动这两个变换，arbiter直接读取MPC自主源。必须按实际launch核对frame/topic，不能仅根据雷达安装位置关闭其中一层。

实机两层开启时：

```text
MPC /cmd_vel/autonomy_raw -> fake -> /cmd_vel/autonomy_gimbal
 -> chassis transform /cmd_vel/autonomy -> arbiter /cmd_vel/selected
```

## 参数与输入

| 参数 | 代码回退值 | 用途 |
| --- | --- | --- |
| `robot_base_frame` / `fake_robot_base_frame` | `gimbal_link` / `gimbal_link_fake` | 正式profile覆盖为项目frame |
| `odom_topic` | `odom` | 当前yaw输入 |
| `input_cmd_vel_topic` / `output_cmd_vel_topic` | 空 | 由正式profile与launch设置 |
| `enable_cmd_spin` | `false` | 默认不订阅、不叠加`cmd_spin` |
| `cmd_spin_topic` / `init_spin_speed` | `cmd_spin` / `0.0` | 仅显式启用旧实验叠加时使用 |

正常自旋由规划yaw/reference产生，并受MPC与底盘约束。正式profile必须保持`enable_cmd_spin=false`，避免在控制器之后另加角速度绕过yaw上限。当前暂定平移上限5 m/s、yaw 3 rad/s、角加速度2 rad/s²，出口矢量限幅由chassis transform拥有；本包不增加独立低速限幅。

TF定时器代码周期20 ms是发布调度，不是本轮实测性能。构建、frame契约检查和实际导航验收分别报告；完整实车固定雷达迁移和当前2027最终套件仍未验证，见[主报告](../../../docs/调研/无先验地图钻洞高速导航优化调研报告.md)。
