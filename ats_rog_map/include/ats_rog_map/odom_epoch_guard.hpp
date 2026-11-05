// Copyright 2026

#ifndef ATS_ROG_MAP__ODOM_EPOCH_GUARD_HPP_
#define ATS_ROG_MAP__ODOM_EPOCH_GUARD_HPP_

#include <cstdint>

namespace ats_rog_map
{
// Caller owns synchronization. The revision fences work decoded outside the
// map lock; the observation-time floor also excludes queued pre-reset scans.
class OdomEpochGuard
{
public:
  bool update(std::uint64_t instance, std::uint64_t epoch, std::int64_t start_ns,
    std::int64_t signal_ns, bool latched)
  {
    if (signal_ns <= signal_ns_ || (instance != 0 && instance < instance_)) {return false;}
    signal_ns_ = signal_ns;
    const bool valid = instance != 0 && epoch != 0 && start_ns > 0 && !latched;
    const bool changed = valid && (instance != instance_ || epoch != epoch_);
    if (changed || valid != valid_) {++revision_;}
    valid_ = valid;
    if (changed) {
      instance_ = instance;
      epoch_ = epoch;
      start_ns_ = start_ns;
    }
    return changed;
  }
  bool accepts(std::int64_t stamp_ns) const {return valid_ && stamp_ns >= start_ns_;}
  bool valid() const {return valid_;}
  std::uint64_t instance() const {return instance_;}
  std::uint64_t epoch() const {return epoch_;}
  std::uint64_t revision() const {return revision_;}

private:
  std::uint64_t instance_{0}, epoch_{0}, revision_{0};
  std::int64_t start_ns_{0}, signal_ns_{0};
  bool valid_{false};
};
}  // namespace ats_rog_map
#endif  // ATS_ROG_MAP__ODOM_EPOCH_GUARD_HPP_
