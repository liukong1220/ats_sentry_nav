# O1 最终参考验证的来源契约

修改基线：`a51ce3a273dfa5ea7916f11405774bae28215a44`，`develop`，tracked tree 干净。

Goal Manager 的候选准入会在最新完整 PlanningMapSnapshot 上再次检查整条参考。原 `ExecutionCommand.map_generation` 是 MINCO 本地不可变编号，`map_publication_sequence` 是该规划的 adapter 来源；二者保持原义。重新验证后的快照序号不应覆盖规划来源，也不能在分析中被默认为原规划快照。

本轮已增加独立的最终验证 publication/source 字段与 `validation_from_reference` 三维 TF。验证方法返回当次使用的 TF，在同一互斥区中通过安全检查后写入 EXECUTE；同系时使用显式单位变换。心跳保留该证据，不重新查询 TF 或改标身份。STOP 的验证字段为零。MPC 的授权、旧参考拒绝和零速语义保持原有约束。

验证应同时证明：更新地图可复检通过且保留原规划来源；阻塞新地图仍拒绝；带平移/旋转的跨 frame 候选绑定实际验证 TF；离线分析优先使用绑定的验证身份和完整三维姿态，缺失或矛盾证据不记通过。新增接口后整仓 Release 重建；DDS 集成测试须先满足本轮隔离安排。

2026-11-17 本机已完成两次 40 包 Release 构建，历史编号复用获用户明确授权。d20 Goal Manager 13 组通过，d21 的相关 MINCO/ROG/RC-ESDF/MPC 功能窄回归通过。V1 d22 的 16 条 EXECUTE 参考按绑定证据复核，q1 全部通过且无载荷截断；q2 持续冲突和定位恢复超时仍失败，不能据此声称 O1 或最终套件通过。原始证据在根工作区 `log/nav_acceptance/20261117_o1_o0_completion/`。

参考平面修复以此为基线：`toPath` 将规划点 z 置零，直接跨 frame 三维变换会使控制参考高度与机体不同，带 roll/pitch 时产生 XY 偏移。现先保持规划 XY 并投影到机体当前控制高度，提交后复检使用实际三维控制路径；TF 丢失或投影奇异则拒绝或撤销。d23 旧实现 3 个直接反例失败；第三次 40 包 Release 构建及 d24 MINCO 8 组通过，含 5 个相关节点用例。

d25 独立 V1 的 18 条执行参考 q1 通过；1388 个实际位姿仅 2 个诊断冲突 tick，无持续冲突、载荷或 frame 缺口。重复 steady 里程计接收超时后按风险条件取消本次 UUID，action code 1，V1 未通过。该运行未带 RViz，独立物理接触未验证；不把局部安全或组件结果推导为完整 O1/O0 完成。
