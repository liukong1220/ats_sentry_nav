// Copyright 2026

#ifndef MINCO_PLANNER__NODES__PLANNING_DIGEST_HPP_
#define MINCO_PLANNER__NODES__PLANNING_DIGEST_HPP_

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

namespace minco_planner
{
namespace planning_digest
{

inline std::array<std::uint8_t, 32> sha256(const std::uint8_t * data, std::size_t length)
{
  static const std::uint32_t k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  auto rotr = [](std::uint32_t value, std::uint32_t bits) {
    return (value >> bits) | (value << (32U - bits));
  };

  std::uint32_t state[8] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<std::uint8_t> buffer(data, data + length);
  const std::uint64_t bit_length = static_cast<std::uint64_t>(length) * 8ULL;
  buffer.push_back(0x80U);
  while ((buffer.size() % 64U) != 56U) {
    buffer.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    buffer.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xffU));
  }

  for (std::size_t offset = 0; offset < buffer.size(); offset += 64U) {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(buffer[offset + static_cast<std::size_t>(i) * 4U]) << 24U) |
        (static_cast<std::uint32_t>(buffer[offset + static_cast<std::size_t>(i) * 4U + 1U]) << 16U) |
        (static_cast<std::uint32_t>(buffer[offset + static_cast<std::size_t>(i) * 4U + 2U]) << 8U) |
        static_cast<std::uint32_t>(buffer[offset + static_cast<std::size_t>(i) * 4U + 3U]);
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotr(w[i - 15], 7U) ^ rotr(w[i - 15], 18U) ^ (w[i - 15] >> 3U);
      const std::uint32_t s1 = rotr(w[i - 2], 17U) ^ rotr(w[i - 2], 19U) ^ (w[i - 2] >> 10U);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t S1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
      const std::uint32_t ch = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 = h + S1 + ch + k[i] + w[i];
      const std::uint32_t S0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = S0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  std::array<std::uint8_t, 32> digest{};
  for (int i = 0; i < 8; ++i) {
    digest[static_cast<std::size_t>(i) * 4U] = static_cast<std::uint8_t>((state[i] >> 24U) & 0xffU);
    digest[static_cast<std::size_t>(i) * 4U + 1U] =
      static_cast<std::uint8_t>((state[i] >> 16U) & 0xffU);
    digest[static_cast<std::size_t>(i) * 4U + 2U] =
      static_cast<std::uint8_t>((state[i] >> 8U) & 0xffU);
    digest[static_cast<std::size_t>(i) * 4U + 3U] = static_cast<std::uint8_t>(state[i] & 0xffU);
  }
  return digest;
}

inline std::string toHex(const std::array<std::uint8_t, 32> & digest, std::size_t hex_chars = 32)
{
  static const char * kDigits = "0123456789abcdef";
  const std::size_t n = std::min<std::size_t>(hex_chars, 64U);
  std::string hex;
  hex.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t byte = digest[i / 2U];
    hex[i] = (i % 2U == 0U) ? kDigits[byte >> 4U] : kDigits[byte & 0x0fU];
  }
  return hex;
}

inline std::string occupancyDigest(const std::vector<int8_t> & occupancy)
{
  std::vector<std::uint8_t> raw(occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    raw[i] = static_cast<std::uint8_t>(static_cast<int>(occupancy[i]) & 0xff);
  }
  return toHex(sha256(raw.empty() ? nullptr : raw.data(), raw.size()));
}

inline std::string occupancyDigest(const nav_msgs::msg::OccupancyGrid & grid)
{
  return occupancyDigest(grid.data);
}

// This identity deliberately excludes publication time and map_load_time: they
// renew transport/heartbeat evidence but do not change the grid consumed by
// JPS, RC-ESDF, or the swept-footprint gate.  Include every field that changes
// grid coordinates or cell semantics so a local immutable snapshot is never
// reused across a safety-relevant map change.
inline void appendDigestBytes(
  std::vector<std::uint8_t> & output, const void * data, std::size_t length)
{
  if (length == 0U) {
    return;
  }
  const auto * bytes = static_cast<const std::uint8_t *>(data);
  output.insert(output.end(), bytes, bytes + length);
}

template<typename ValueT>
inline void appendDigestValue(std::vector<std::uint8_t> & output, const ValueT & value)
{
  appendDigestBytes(output, &value, sizeof(ValueT));
}

inline void appendDigestString(std::vector<std::uint8_t> & output, const std::string & value)
{
  const auto length = static_cast<std::uint64_t>(value.size());
  appendDigestValue(output, length);
  appendDigestBytes(output, value.data(), value.size());
}

inline std::string gridSafetyDigest(
  const nav_msgs::msg::OccupancyGrid & grid, int obstacle_value_threshold,
  bool unknown_is_obstacle)
{
  std::vector<std::uint8_t> raw;
  raw.reserve(
    sizeof(std::uint64_t) + grid.header.frame_id.size() + sizeof(grid.info.resolution) +
    sizeof(grid.info.width) + sizeof(grid.info.height) + 7U * sizeof(double) +
    sizeof(std::int32_t) + sizeof(std::uint8_t) + grid.data.size());
  appendDigestString(raw, grid.header.frame_id);
  appendDigestValue(raw, grid.info.resolution);
  appendDigestValue(raw, grid.info.width);
  appendDigestValue(raw, grid.info.height);
  appendDigestValue(raw, grid.info.origin.position.x);
  appendDigestValue(raw, grid.info.origin.position.y);
  appendDigestValue(raw, grid.info.origin.position.z);
  appendDigestValue(raw, grid.info.origin.orientation.x);
  appendDigestValue(raw, grid.info.origin.orientation.y);
  appendDigestValue(raw, grid.info.origin.orientation.z);
  appendDigestValue(raw, grid.info.origin.orientation.w);
  const auto threshold = static_cast<std::int32_t>(obstacle_value_threshold);
  const auto unknown = static_cast<std::uint8_t>(unknown_is_obstacle ? 1U : 0U);
  appendDigestValue(raw, threshold);
  appendDigestValue(raw, unknown);
  appendDigestBytes(raw, grid.data.data(), grid.data.size());
  return toHex(sha256(raw.data(), raw.size()));
}

inline std::string pathContentDigest(const nav_msgs::msg::Path & path)
{
  std::vector<std::uint8_t> raw;
  raw.resize(path.poses.size() * 24U);
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    const double values[3] = {
      pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
    std::memcpy(raw.data() + i * 24U, values, sizeof(values));
  }
  return toHex(sha256(raw.empty() ? nullptr : raw.data(), raw.size()));
}

inline void fillByteDigest(
  std::array<std::uint8_t, 32> & out, const std::string & hex)
{
  out.fill(0U);
  auto from_hex = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
      return c - 'A' + 10;
    }
    return -1;
  };
  for (std::size_t i = 0; i + 1U < hex.size() && i / 2U < out.size(); i += 2U) {
    const int high = from_hex(hex[i]);
    const int low = from_hex(hex[i + 1U]);
    if (high < 0 || low < 0) {
      break;
    }
    out[i / 2U] = static_cast<std::uint8_t>((high << 4) | low);
  }
}

}  // namespace planning_digest
}  // namespace minco_planner

#endif  // MINCO_PLANNER__NODES__PLANNING_DIGEST_HPP_
