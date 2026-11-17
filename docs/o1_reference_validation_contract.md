# O1 最终参考验证的来源契约

修改基线：`a51ce3a273dfa5ea7916f11405774bae28215a44`，`develop`，tracked tree 干净。

Goal Manager 的候选准入会在最新完整 PlanningMapSnapshot 上再次检查整条参考。原 `ExecutionCommand.map_generation` 是 MINCO 本地不可变编号，`map_publication_sequence` 是该规划的 adapter 来源；二者保持原义。重新验证后的快照序号不应覆盖规划来源，也不能在分析中被默认为原规划快照。

本轮拟增加独立的最终验证 publication/source 字段与 `validation_from_reference` 三维 TF。验证方法返回当次使用的 TF，在同一互斥区中通过安全检查后写入 EXECUTE；同系时使用显式单位变换。心跳保留该证据，不重新查询 TF 或改标身份。STOP 的验证字段为零。MPC 的授权、旧参考拒绝和零速语义保持原有约束。

验证应同时证明：更新地图可复检通过且保留原规划来源；阻塞新地图仍拒绝；带平移/旋转的跨 frame 候选绑定实际验证 TF；离线分析优先使用绑定的验证身份和完整三维姿态，缺失或矛盾证据不记通过。新增接口后整仓 Release 重建；DDS 集成测试须先满足本轮隔离安排。
