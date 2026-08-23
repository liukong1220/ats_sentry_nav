// Copyright 2026

#ifndef ATS_SWERVE_MPC__QP__CONTROL_CYCLE_SNAPSHOT_HPP_
#define ATS_SWERVE_MPC__QP__CONTROL_CYCLE_SNAPSHOT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "ats_swerve_mpc/se2_mpc_controller.hpp"

namespace ats_swerve_mpc {

/**
 * @brief 单个 iLQR/QP shadow 控制周期共享的不可变输入快照。
 *
 * @details 控制定时器在调用 iLQR 前复制当前状态、reference、上一控制和执行授权身份。
 *          iLQR 只能读取此副本求解，随后 qp_shadow 也只能读取同一副本，禁止在二者之间
 *          重新读取 ROS 状态、tracker 或 ExecutionCommand。该类型没有可发布 Twist，因而
 *          不能改变 `/cmd_vel/autonomy_raw` 的 iLQR 唯一所有权。
 */
struct ControlCycleSnapshot {
  std::uint64_t cycle_sequence = 0;
  std::chrono::steady_clock::time_point steady_start{};
  State current_state = State::Zero();
  std::vector<Se2Reference> references;
  Control last_control_before_solve = Control::Zero();
  Se2MpcResult ilqr_result;

  // ExecutionCommand 的单调身份字段；它们是审计输入，不是 QP 可以修改的控制权限。
  std::uint64_t manager_incarnation = 0;
  std::uint64_t command_sequence = 0;
  std::uint64_t goal_id = 0;
  std::uint64_t localization_epoch = 0;
  std::uint64_t map_generation = 0;
  std::uint64_t map_publication_sequence = 0;
  std::string reference_frame;
  std::int64_t reference_stamp_ns = 0;
  std::int64_t reference_deadline_ns = 0;
  bool reference_fresh = false;
  bool emergency_stop_active = true;
  bool localization_fresh = false;
  bool gimbal_valid = false;
  bool execution_lease_valid = false;
  // Twist-only 节点尚无独立 map/footprint 健康 producer；接入真实 snapshot 前必须
  // 保持 false，令 qp_shadow candidate 以 fail-closed 方式拒绝。
  bool map_fresh = false;

  // iLQR 求解前计算。shadow 记录前以同一 canonical 规则重算并逐值比对；有效位与摘要
  // 数值分离，避免把任意合法的 64 位哈希值误当成“输入含非有限数”。
  std::uint64_t identity_digest = 0;
  bool identity_digest_valid = false;
};

/**
 * @brief 检查摘要所覆盖的数值输入是否全部有限。
 * @details 仅检查 current、reference state/control 与 solve 前 last_control；执行授权字段
 *          均为固定宽度整数。该检查不替代 MPC/QP 的输入健康、map、碰撞或 lease 安全门。
 */
bool controlCycleSnapshotIdentityInputsFinite(const ControlCycleSnapshot &snapshot);

/**
 * @brief 计算跨平台可复现的同周期输入摘要。
 * @return FNV-1a-64 摘要；若输入含 NaN/Inf 则返回 0，但有效性必须由
 *         controlCycleSnapshotIdentityInputsFinite() 或 snapshot 的显式 valid 位判断。
 * @details 编码顺序固定为 schema、current state、reference stamp/deadline/frame、全部
 *          reference state/control、solve 前 last_control、ExecutionCommand identity。
 *          所有整数和 IEEE-754 double 位模式按小端逐字节写入，字符串使用长度前缀；不使用
 *          DDS CDR 或进程内地址。它是审计 identity，不是安全授权哈希或密码学签名。
 */
std::uint64_t controlCycleSnapshotIdentityDigest(
    const ControlCycleSnapshot &snapshot);

}  // namespace ats_swerve_mpc

#endif  // ATS_SWERVE_MPC__QP__CONTROL_CYCLE_SNAPSHOT_HPP_
