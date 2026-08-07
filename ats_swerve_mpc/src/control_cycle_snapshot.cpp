// Copyright 2026

#include "ats_swerve_mpc/qp/control_cycle_snapshot.hpp"

#include <cmath>
#include <cstring>

namespace ats_swerve_mpc {

namespace {

constexpr std::uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnv1aPrime = 1099511628211ULL;
constexpr std::uint64_t kSnapshotIdentitySchemaVersion = 1ULL;

/** @brief 以固定字节序累积 FNV-1a-64，避免 host endian 或对象布局参与摘要。 */
class CanonicalDigestWriter {
public:
  /** @brief 创建 schema 写入器；任何字段都只能追加，不能回退或跳过顺序。 */
  CanonicalDigestWriter() = default;

  /** @brief 写入一个原始字节并更新 FNV-1a 状态。 */
  void appendByte(std::uint8_t value) {
    value_ ^= value;
    value_ *= kFnv1aPrime;
  }

  /** @brief 将无符号整数固定编码为 8 个小端字节。 */
  void appendU64(std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      appendByte(static_cast<std::uint8_t>((value >> (8 * byte)) & 0xffU));
    }
  }

  /** @brief 将有符号时间纳秒按二进制补码的固定宽度编码。 */
  void appendI64(std::int64_t value) {
    appendU64(static_cast<std::uint64_t>(value));
  }

  /** @brief 写入 IEEE-754 double 位模式；调用者已先拒绝非有限输入。 */
  void appendFiniteDouble(double value) {
    // -0.0 与 +0.0 对轨迹语义等价，先归一化以免无意义的符号位导致审计摘要分叉。
    if (value == 0.0) {
      value = 0.0;
    }
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
    std::memcpy(&bits, &value, sizeof(bits));
    appendU64(bits);
  }

  /** @brief 写入长度前缀和 UTF-8 原始字节，不依赖 std::string 的内存布局。 */
  void appendString(const std::string &value) {
    appendU64(static_cast<std::uint64_t>(value.size()));
    for (const char character : value) {
      appendByte(static_cast<std::uint8_t>(character));
    }
  }

  /** @brief 返回当前摘要；有效性由调用方单独保存，摘要值本身不承担哨兵语义。 */
  std::uint64_t value() const { return value_; }

private:
  std::uint64_t value_ = kFnv1aOffsetBasis;
};

/** @brief 顺序写入 Eigen 三元素状态或控制，不依赖 Eigen 内部存储布局。 */
void appendVector3(CanonicalDigestWriter &writer, const Eigen::Vector3d &value) {
  for (int axis = 0; axis < 3; ++axis) {
    writer.appendFiniteDouble(value(axis));
  }
}

}  // namespace

/**
 * @brief 判断摘要覆盖的浮点输入是否有限。
 * @details 这一步先于 bit 编码执行，保证 NaN payload、Inf 和不同平台的未定义表示不会进入
 *          审计摘要；调用方据此将相应 QP shadow 周期标记为不可审计，而非假定同一输入。
 */
bool controlCycleSnapshotIdentityInputsFinite(const ControlCycleSnapshot &snapshot) {
  if (!snapshot.current_state.allFinite() ||
      !snapshot.last_control_before_solve.allFinite()) {
    return false;
  }
  for (const Se2Reference &reference : snapshot.references) {
    if (!reference.state.allFinite() || !reference.control.allFinite()) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 按固定 schema 生成控制周期输入摘要。
 * @details 故意不编码 cycle_sequence、steady 时钟、iLQR result、地图健康和 candidate；这些
 *          字段不是 iLQR/QP 共用的求解输入。由此 digest 只回答“二者是否读取相同控制输入”，
 *          不把一次 QP 结果或 wall-clock 抖动误报为输入变化。
 */
std::uint64_t controlCycleSnapshotIdentityDigest(
    const ControlCycleSnapshot &snapshot) {
  if (!controlCycleSnapshotIdentityInputsFinite(snapshot)) {
    return 0;
  }

  CanonicalDigestWriter writer;
  writer.appendU64(kSnapshotIdentitySchemaVersion);
  writer.appendString("ats_swerve_mpc.control_cycle_snapshot");
  appendVector3(writer, snapshot.current_state);
  writer.appendI64(snapshot.reference_stamp_ns);
  writer.appendI64(snapshot.reference_deadline_ns);
  writer.appendString(snapshot.reference_frame);
  writer.appendU64(static_cast<std::uint64_t>(snapshot.references.size()));
  for (const Se2Reference &reference : snapshot.references) {
    appendVector3(writer, reference.state);
    appendVector3(writer, reference.control);
  }
  appendVector3(writer, snapshot.last_control_before_solve);
  writer.appendU64(snapshot.manager_incarnation);
  writer.appendU64(snapshot.command_sequence);
  writer.appendU64(snapshot.goal_id);
  writer.appendU64(snapshot.localization_epoch);
  writer.appendU64(snapshot.map_generation);
  writer.appendU64(snapshot.map_publication_sequence);
  return writer.value();
}

}  // namespace ats_swerve_mpc
