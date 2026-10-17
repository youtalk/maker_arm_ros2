// The daemon's Unix-socket wire, the C++ side of crates/maker-arm-cli/src/wire.rs.
// Little-endian, fixed-size frames, each starting with an 8-byte header:
// magic u32, version u16, kind u16.
#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace maker_arm_hardware::wire
{

static_assert(std::endian::native == std::endian::little, "the wire is little-endian");

constexpr uint32_t kMagic = 0x4D41524D;
constexpr uint16_t kVersion = 1;
constexpr size_t kStateLen = 272;

enum class Role : uint8_t { Command = 1, Monitor = 2, Release = 3 };
enum class Op : uint8_t { Activate = 1, Deactivate = 2, Poll = 3, Release = 4 };
enum class Mode : uint8_t { Holding = 0, Following = 1, Refused = 2, Fault = 3, Released = 4 };

struct StateFrame
{
  uint32_t seq;  // the last client seq
  Mode mode;
  uint64_t digest;
  double t;  // loop clock
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> effort;
  std::array<double, 7> temp;
  uint32_t fault;
  uint32_t clamped;
  uint32_t refused;
};

namespace detail
{
constexpr uint16_t kHello = 1;
constexpr uint16_t kControl = 2;
constexpr uint16_t kCommand = 3;
constexpr uint16_t kState = 4;

template<typename T>
void put(std::vector<uint8_t> & b, size_t at, const T & v)
{
  std::memcpy(b.data() + at, &v, sizeof v);
}

template<typename T>
T get(const uint8_t * b, size_t at)
{
  T v;
  std::memcpy(&v, b + at, sizeof v);
  return v;
}

inline std::vector<uint8_t> frame(uint16_t kind, size_t len)
{
  std::vector<uint8_t> b(len, 0);
  put(b, 0, kMagic);
  put(b, 4, kVersion);
  put(b, 6, kind);
  return b;
}

inline std::array<double, 7> arr(const uint8_t * b, size_t at)
{
  std::array<double, 7> a;
  std::memcpy(a.data(), b + at, sizeof a);
  return a;
}
}  // namespace detail

inline std::vector<uint8_t> encode_hello(Role role)
{
  auto b = detail::frame(detail::kHello, 16);
  b[8] = static_cast<uint8_t>(role);
  return b;
}

inline std::vector<uint8_t> encode_control(uint32_t seq, Op op)
{
  auto b = detail::frame(detail::kControl, 16);
  detail::put(b, 8, seq);
  b[12] = static_cast<uint8_t>(op);
  return b;
}

// Six joints in rad, then gripper_left_joint in m.
inline std::vector<uint8_t> encode_command(uint32_t seq, const std::array<double, 7> & q)
{
  auto b = detail::frame(detail::kCommand, 72);
  detail::put(b, 8, seq);
  detail::put(b, 16, q);
  return b;
}

// A wrong length, magic, version, kind or mode gives nullopt and a text in *error.
inline std::optional<StateFrame> decode_state(const uint8_t * data, size_t len, std::string * error)
{
  auto fail = [&](std::string msg) -> std::optional<StateFrame> {
      if (error) {*error = std::move(msg);}
      return std::nullopt;
    };
  if (len != kStateLen) {
    return fail("state frame is " + std::to_string(len) + " bytes, want " +
             std::to_string(kStateLen));
  }
  if (detail::get<uint32_t>(data, 0) != kMagic) {return fail("bad magic");}
  const auto version = detail::get<uint16_t>(data, 4);
  if (version != kVersion) {
    return fail("wire version " + std::to_string(version) + ", this build speaks " +
             std::to_string(kVersion));
  }
  const auto kind = detail::get<uint16_t>(data, 6);
  if (kind != detail::kState) {return fail("unknown frame kind " + std::to_string(kind));}
  if (data[12] > static_cast<uint8_t>(Mode::Released)) {
    return fail("unknown mode " + std::to_string(data[12]));
  }
  StateFrame s;
  s.seq = detail::get<uint32_t>(data, 8);
  s.mode = static_cast<Mode>(data[12]);
  s.digest = detail::get<uint64_t>(data, 16);
  s.t = detail::get<double>(data, 24);
  s.q = detail::arr(data, 32);
  s.dq = detail::arr(data, 88);
  s.effort = detail::arr(data, 144);
  s.temp = detail::arr(data, 200);
  s.fault = detail::get<uint32_t>(data, 256);
  s.clamped = detail::get<uint32_t>(data, 260);
  s.refused = detail::get<uint32_t>(data, 264);
  return s;
}

// FNV-1a 64 over each part as name, a 0 byte, the value, a 0 byte.
inline uint64_t digest(const std::vector<std::pair<std::string, std::string>> & parts)
{
  uint64_t h = 0xcbf29ce484222325ull;
  auto eat = [&h](const std::string & s) {
      for (unsigned char c : s) {h = (h ^ c) * 0x100000001b3ull;}
      h = (h ^ 0) * 0x100000001b3ull;
    };
  for (const auto & [name, value] : parts) {
    eat(name);
    eat(value);
  }
  return h;
}

}  // namespace maker_arm_hardware::wire
