# ats_rog_map

`ats_rog_map_node`使用外部`/localization`、`/registered_scan`与TF，不提供定位。滑窗中心取`base_frame`，射线原点取`sensor_frame`；底盘固定MID360的当前frame为`gimbal_yaw_odom`与`front_mid360`，底盘转动不意味着配准后的静态环境应随车旋转。

同一进程还运行规划地图所有者`ats_rog_map_adapter`（`src/planning/`），直接读取ROGMap数值投影，融合terrain和静态层，唯一发布`/rog_map_adapter/planning_snapshot`。消息包含occupancy、signed distance、梯度、来源generation、publication sequence与localization epoch。ready为持续续租的heartbeat；旧独立adapter包、projection service及`/rc_esdf/*`均不是现行接口。

`/rog_map/{occ,inf_occ,unk,esdf}`与`/rog_map_adapter/viz/{planning_grid,esdf_cloud,clearance_contours}`仅作调试，可视化点云不作为规划数值源。MuJoCo的RViz默认关闭体素、滑窗、距离场及路径调试组，保留当前帧注册点云、车体、定位和传感器轴。

## 参数加载

正式实机参数来自根仓`src/ats_sentry_bringup/params/node_params.yaml`；MuJoCo按launch叠加`rmuc_2025_navigation.yaml`和2027场景profile。独立`ats_rog_map.launch.py`默认按顺序加载：

- `params_file=config/rog_map_ground_planning_mujoco.yaml`；
- `planning_params_file=config/rog_map_ground_planning.yaml`；
- 显式frame、topic、sim time等launch覆盖。

旧`rog_map.yaml`、`rog_map_050.yaml`、`rog_map_report_025.yaml`不是当前正式入口的第二份参数权威。不要把旧`map_config_file`选择说明套到上述ROS参数加载链；报告级2.5 cm配置的内存、更新耗时及50 Hz/6 ms性能未在本机复现。

2026-11-16三套profile统一footprint长×宽0.581×0.571 m、每边margin 0.02 m，ROG列车高门`0.242+0.007=0.249 m`，terrain头顶门同值。参数同步不等于H2净空、M路线或H3实车通过；`TUNNEL_FREE`尚未完整实现。

当前进度见[主报告第5节](../../../docs/调研/无先验地图钻洞高速导航优化调研报告.md)，最终验证见[2027方案](../../../docs/nav_acceptance/20261116_2027_final_test_plan.md)。旧局部隧道记录不证明真实B往返，当前全场与对称点导航速度仍未通过。
