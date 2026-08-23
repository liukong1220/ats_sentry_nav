// Copyright 2026 ATS 2026 Sentry Project
//
// 速度源仲裁纯逻辑：不碰 ROS、不碰串口。
// 只决定哪一个 Twist 可以成为 `/cmd_vel/selected` 的内容。
//
// 硬性契约：
//  1. 新鲜手动源优先。
//  2. 手动超时后不得复活旧手动命令；只允许输出零，或回收到当前仍合法的自动源。
//  3. 自动源必须有新鲜 ExecutionCommand 授权，且授权按 execution_command_timeout 自动过期。
//  4. ExecutionCommand 以 manager_incarnation 和 command_sequence 共同防重放；
//     新 incarnation 必须先用新鲜 MODE_STOP 接管旧租约。
//  5. 手动源不要求 ExecutionCommand。
//  6. 急停对两源立即归零。
//  7. 下位机链路失效对两源立即归零。
//  8. 各源超过自身 timeout 视为无效，不得使用超时样本。

#ifndef ATS_CMD_VEL_ARBITER__CMD_VEL_ARBITER_HPP_
#define ATS_CMD_VEL_ARBITER__CMD_VEL_ARBITER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace ats_cmd_vel_arbiter
{

enum class CmdVelSource : uint8_t {
  NONE = 0,
  MANUAL = 1,
  AUTO = 2,
};

enum class CmdVelArbiterReason : uint8_t {
  ZERO_NO_SOURCE = 0,
  MANUAL_FRESH = 1,
  AUTO_AUTHORIZED = 2,
  MANUAL_TIMEOUT = 3,
  AUTO_UNAUTHORIZED = 4,
  AUTO_TIMEOUT = 5,
  EMERGENCY_STOP = 6,
  LINK_DOWN = 7,
};

struct CmdVelArbiterConfig
{
  std::chrono::milliseconds manual_timeout{300};
  std::chrono::milliseconds auto_timeout{300};
  std::chrono::milliseconds execution_command_timeout{500};
  /// `/serial/link_up` heartbeat 超时。无样本或超龄都必须按链路失效归零。
  std::chrono::milliseconds link_timeout{300};
};

struct CmdVelSample
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  std::chrono::steady_clock::time_point stamp{};
  bool valid = false;
};

struct CmdVelArbiterOutput
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  CmdVelSource source = CmdVelSource::NONE;
  CmdVelArbiterReason reason = CmdVelArbiterReason::ZERO_NO_SOURCE;

  bool isZero() const { return vx == 0.0 && vy == 0.0 && wz == 0.0; }
};

class CmdVelArbiter
{
public:
  explicit CmdVelArbiter(const CmdVelArbiterConfig & config = CmdVelArbiterConfig())
  : config_(config)
  {
  }

  void setConfig(const CmdVelArbiterConfig & config) { config_ = config; }
  const CmdVelArbiterConfig & config() const { return config_; }

  void onManual(double vx, double vy, double wz, std::chrono::steady_clock::time_point now)
  {
    manual_.vx = vx;
    manual_.vy = vy;
    manual_.wz = wz;
    manual_.stamp = now;
    manual_.valid = true;
  }

  void onAuto(double vx, double vy, double wz, std::chrono::steady_clock::time_point now)
  {
    auto_.vx = vx;
    auto_.vy = vy;
    auto_.wz = wz;
    auto_.stamp = now;
    auto_.valid = true;
  }

  /// age 是命令戳在到达时的年龄。租约从（now - age）起算，到期后即使 autonomy 持续刷新也必须归零。
  /// stale / replay 不得延长已有授权。MODE_STOP、急停、断链立即撤销租约。
  /// 进程首次连接可接受新鲜 EXECUTE，以便 arbiter 重启后重新建立 DDS 状态；
  /// 已有 owner 时，新 manager_incarnation 只能由新鲜 MODE_STOP 接管。
  bool onExecutionCommand(
    bool execute,
    uint64_t manager_incarnation,
    uint64_t sequence,
    std::chrono::milliseconds age,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    if (
      manager_incarnation == 0 || sequence == 0 || age.count() < 0 ||
      age > config_.execution_command_timeout)
    {
      return false;
    }
    if (!has_manager_incarnation_) {
      has_manager_incarnation_ = true;
      active_manager_incarnation_ = manager_incarnation;
      has_command_sequence_ = false;
    } else if (manager_incarnation < active_manager_incarnation_) {
      return false;
    } else if (manager_incarnation > active_manager_incarnation_) {
      // A restarted manager's sequence starts from one. Its EXECUTE must not
      // reactivate an old lease before the restart STOP has invalidated it.
      if (execute) {
        return false;
      }
      clearExecutionLease();
      invalidateAuto();
      active_manager_incarnation_ = manager_incarnation;
      has_command_sequence_ = false;
    }
    if (has_command_sequence_ && sequence <= last_command_sequence_) {
      return false;
    }
    has_command_sequence_ = true;
    last_command_sequence_ = sequence;
    if (!execute) {
      clearExecutionLease();
      invalidateAuto();
    } else if (!emergency_stop_ && link_up_) {
      auto_authorized_ = true;
      has_execution_lease_ = true;
      execution_lease_stamp_ = now - age;
    } else {
      auto_authorized_ = false;
      has_execution_lease_ = false;
    }
    return true;
  }

  void onEmergencyStop(bool active)
  {
    emergency_stop_ = active;
    if (active) {
      clearExecutionLease();
      invalidateSources();
    }
  }

  void onSerialLinkDown()
  {
    link_up_ = false;
    clearExecutionLease();
    invalidateSources();
  }

  /// DOWN->UP 的第一个心跳必须丢弃链路不可用期间缓存的两源和授权。
  /// 这也覆盖进程启动后第一个 UP：无心跳时收到的命令不能在链路后来可用时复活。
  void onSerialLinkUp(std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    if (!link_up_) {
      clearExecutionLease();
      invalidateSources();
    }
    has_link_heartbeat_ = true;
    last_link_stamp_ = now;
    link_up_ = true;
  }

  /// 串口节点发布的 `/serial/link_up` 心跳。false 立即断链；true 只放开链路。
  void onLinkHealth(bool up, std::chrono::steady_clock::time_point now)
  {
    if (up) {
      onSerialLinkUp(now);
    } else {
      has_link_heartbeat_ = true;
      last_link_stamp_ = now;
      onSerialLinkDown();
    }
  }

  bool autoAuthorized() const { return auto_authorized_; }
  bool linkUp() const { return link_up_; }
  bool emergencyStop() const { return emergency_stop_; }

  CmdVelArbiterOutput tick(std::chrono::steady_clock::time_point now)
  {
    if (!has_link_heartbeat_) {
      link_up_ = false;
      return zero(CmdVelArbiterReason::LINK_DOWN);
    }
    if (config_.link_timeout.count() > 0) {
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_link_stamp_);
      if (elapsed.count() < 0 || elapsed > config_.link_timeout) {
        onSerialLinkDown();
        has_link_heartbeat_ = false;
        return zero(CmdVelArbiterReason::LINK_DOWN);
      }
    }
    if (!link_up_) {
      return zero(CmdVelArbiterReason::LINK_DOWN);
    }
    if (emergency_stop_) {
      return zero(CmdVelArbiterReason::EMERGENCY_STOP);
    }
    expireExecutionLease(now);

    const bool manual_was_valid = manual_.valid;
    const bool auto_was_valid = auto_.valid;
    const bool manual_fresh = isFresh(manual_, config_.manual_timeout, now);
    const bool auto_fresh = isFresh(auto_, config_.auto_timeout, now);
    if (!manual_fresh) {
      manual_.valid = false;
    }
    if (!auto_fresh) {
      auto_.valid = false;
    }

    if (manual_fresh) {
      return fromSample(manual_, CmdVelSource::MANUAL, CmdVelArbiterReason::MANUAL_FRESH);
    }
    if (auto_fresh) {
      if (!auto_authorized_) {
        return zero(CmdVelArbiterReason::AUTO_UNAUTHORIZED);
      }
      return fromSample(auto_, CmdVelSource::AUTO, CmdVelArbiterReason::AUTO_AUTHORIZED);
    }
    if (manual_was_valid) {
      return zero(CmdVelArbiterReason::MANUAL_TIMEOUT);
    }
    if (auto_was_valid) {
      return zero(CmdVelArbiterReason::AUTO_TIMEOUT);
    }
    return zero(CmdVelArbiterReason::ZERO_NO_SOURCE);
  }

private:
  static CmdVelArbiterOutput zero(CmdVelArbiterReason reason)
  {
    CmdVelArbiterOutput output;
    output.reason = reason;
    return output;
  }

  static CmdVelArbiterOutput fromSample(
    const CmdVelSample & sample, CmdVelSource source, CmdVelArbiterReason reason)
  {
    CmdVelArbiterOutput output;
    output.vx = sample.vx;
    output.vy = sample.vy;
    output.wz = sample.wz;
    output.source = source;
    output.reason = reason;
    return output;
  }

  static bool isFresh(
    const CmdVelSample & sample,
    std::chrono::milliseconds timeout,
    std::chrono::steady_clock::time_point now)
  {
    if (!sample.valid || timeout.count() <= 0) {
      return false;
    }
    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - sample.stamp);
    return elapsed.count() >= 0 && elapsed <= timeout;
  }

  void invalidateSources()
  {
    manual_.valid = false;
    invalidateAuto();
  }

  void invalidateAuto()
  {
    auto_.valid = false;
  }

  void clearExecutionLease()
  {
    auto_authorized_ = false;
    has_execution_lease_ = false;
  }

  void expireExecutionLease(std::chrono::steady_clock::time_point now)
  {
    if (!auto_authorized_) {
      return;
    }
    if (!has_execution_lease_) {
      auto_authorized_ = false;
      return;
    }
    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - execution_lease_stamp_);
    if (elapsed.count() < 0 || elapsed > config_.execution_command_timeout) {
      clearExecutionLease();
    }
  }

  CmdVelArbiterConfig config_;
  CmdVelSample manual_;
  CmdVelSample auto_;
  bool auto_authorized_ = false;
  bool emergency_stop_ = false;
  bool link_up_ = false;
  bool has_link_heartbeat_ = false;
  std::chrono::steady_clock::time_point last_link_stamp_{};
  bool has_command_sequence_ = false;
  uint64_t last_command_sequence_ = 0;
  bool has_manager_incarnation_ = false;
  uint64_t active_manager_incarnation_ = 0;
  bool has_execution_lease_ = false;
  std::chrono::steady_clock::time_point execution_lease_stamp_{};
};

}  // namespace ats_cmd_vel_arbiter

#endif  // ATS_CMD_VEL_ARBITER__CMD_VEL_ARBITER_HPP_
