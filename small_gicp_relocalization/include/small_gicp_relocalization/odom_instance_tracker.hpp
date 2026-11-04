// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef SMALL_GICP_RELOCALIZATION__ODOM_INSTANCE_TRACKER_HPP_
#define SMALL_GICP_RELOCALIZATION__ODOM_INSTANCE_TRACKER_HPP_

#include <cstdint>
#include <cstddef>
#include <deque>
#include <limits>

namespace small_gicp_relocalization
{

// This only tracks process identities; it does not establish that an odometry
// sample belongs to an identity. Separate DDS topics provide no such ordering.
class OdomInstanceTracker
{
public:
  enum class Result {kInvalid, kPending, kFirst, kUnchanged, kChanged, kRetired, kLatched};

  explicit OdomInstanceTracker(std::size_t initial_samples = 1,
    std::size_t retired_limit = 100, double window_seconds = 1.0)
  : initial_samples_(initial_samples ? initial_samples : 1),
    retired_limit_(retired_limit), window_seconds_(window_seconds) {}

  static constexpr std::uint64_t kNoUpperBound = std::numeric_limits<std::uint64_t>::max();

  // `now` (seconds, same clock for every call) and `upper_bound` are required on
  // purpose: a forgotten `now` froze the retired-count window, and without an
  // upper bound one absurdly large id would retire the real instance.  Ids are
  // steady_clock start times, so a valid id can never exceed the receiver's own
  // current steady_clock reading plus a small tolerance.
  Result observe(std::uint64_t instance, bool fresh, double now, std::uint64_t upper_bound)
  {
    if (latched_) {
      return Result::kLatched;
    }
    if (instance == 0) {
      pending_count_ = 0;
      return Result::kInvalid;
    }
    if (instance > upper_bound) {
      pending_count_ = 0;
      ++future_rejected_count_;
      return Result::kInvalid;
    }
    const auto floor = instance_ != 0 ? instance_ : pending_instance_;
    if (instance < floor) {
      if (instance_ == 0) {
        pending_count_ = 0;
      }
      ++retired_count_;
      while (!retired_times_.empty() && now - retired_times_.front() > window_seconds_) {
        retired_times_.pop_front();
      }
      retired_times_.push_back(now);
      if (retired_times_.size() > retired_limit_) {
        latched_ = true;
        return Result::kLatched;
      }
      return Result::kRetired;
    }
    if (!fresh) {
      pending_count_ = 0;
      return Result::kInvalid;
    }
    if (instance_ == 0) {
      if (instance != pending_instance_) {
        pending_instance_ = instance;
        pending_count_ = 0;
      }
      if (++pending_count_ < initial_samples_) {
        return Result::kPending;
      }
      instance_ = instance;
      ++epoch_;
      return Result::kFirst;
    }
    if (instance == instance_) {
      return Result::kUnchanged;
    }
    instance_ = instance;
    ++epoch_;
    return Result::kChanged;
  }

  std::uint64_t instance() const {return instance_;}
  std::uint64_t epoch() const {return epoch_;}
  std::uint64_t retiredCount() const {return retired_count_;}
  std::uint64_t futureRejectedCount() const {return future_rejected_count_;}
  bool latched() const {return latched_;}

private:
  std::uint64_t instance_{0};
  std::uint64_t epoch_{0};
  std::uint64_t pending_instance_{0};
  std::uint64_t retired_count_{0};
  std::uint64_t future_rejected_count_{0};
  std::size_t pending_count_{0};
  std::size_t initial_samples_;
  std::size_t retired_limit_;
  double window_seconds_;
  std::deque<double> retired_times_;
  bool latched_{false};
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__ODOM_INSTANCE_TRACKER_HPP_
