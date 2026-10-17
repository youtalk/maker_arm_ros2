#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

#include "maker_arm_hardware/wire.hpp"

namespace w = maker_arm_hardware::wire;

namespace
{
const char * const kHelloHex = "4d52414d010001000100000000000000";
const char * const kControlHex = "4d52414d010002000700000001000000";
const char * const kCommandHex =
  "4d52414d010003002a000000000000009a9999999999b93f9a9999999999c9bf333333333333d33f9a9999999999d9bf"
  "000000000000e03f333333333333e3bf9a9999999999993f";
const char * const kStateHex =
  "4d52414d010004002a00000001000000efcdab896745230100000000000029409a9999999999b93f9a9999999999c9bf"
  "333333333333d33f9a9999999999d9bf000000000000e03f333333333333e3bf9a9999999999993f0000000000000000"
  "000000000000000000000000000000000000000000000000000000000000000000000000000000007b14ae47e17a84bf"
  "000000000000f83f00000000000000000000000000000000000000000000000000000000000000000000000000000000"
  "000000000000d03f00000000008041400000000000804140000000000080414000000000008041400000000000804140"
  "0000000000804140000000000080414000000000030000000100000000000000";
const std::array<double, 7> kQ = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.025};

std::string hex(const std::vector<uint8_t> & b)
{
  std::string s;
  char buf[3];
  for (auto x : b) {
    snprintf(buf, sizeof buf, "%02x", x);
    s += buf;
  }
  return s;
}

std::vector<uint8_t> unhex(const std::string & s)
{
  std::vector<uint8_t> b;
  for (size_t i = 0; i < s.size(); i += 2) {
    b.push_back(static_cast<uint8_t>(std::stoi(s.substr(i, 2), nullptr, 16)));
  }
  return b;
}
}  // namespace

TEST(Wire, ClientFramesMatchGolden)
{
  EXPECT_EQ(hex(w::encode_hello(w::Role::Command)), kHelloHex);
  EXPECT_EQ(hex(w::encode_control(7, w::Op::Activate)), kControlHex);
  EXPECT_EQ(hex(w::encode_command(42, kQ)), kCommandHex);
}

TEST(Wire, StateDecodesGolden)
{
  auto b = unhex(kStateHex);
  ASSERT_EQ(b.size(), w::kStateLen);
  std::string err;
  auto s = w::decode_state(b.data(), b.size(), &err);
  ASSERT_TRUE(s.has_value()) << err;
  EXPECT_EQ(s->seq, 42u);
  EXPECT_EQ(s->mode, w::Mode::Following);
  EXPECT_EQ(s->digest, 0x0123456789ABCDEFull);
  EXPECT_EQ(s->t, 12.5);
  EXPECT_EQ(s->q, kQ);
  EXPECT_EQ(s->dq, (std::array<double, 7>{0, 0, 0, 0, 0, 0, -0.01}));
  EXPECT_EQ(s->effort, (std::array<double, 7>{1.5, 0, 0, 0, 0, 0, 0.25}));
  EXPECT_EQ(s->temp, (std::array<double, 7>{35, 35, 35, 35, 35, 35, 35}));
  EXPECT_EQ(s->fault, 0u);
  EXPECT_EQ(s->clamped, 3u);
  EXPECT_EQ(s->refused, 1u);
}

TEST(Wire, DigestMatchesGolden)
{
  EXPECT_EQ(
    w::digest(
      {{"interface", "sim"},
        {"profile", R"({"max_velocity": 2.5})"},
        {"dynamics", "{}"},
        {"params", ""},
        {"residual", ""},
        {"gripper_map", ""}}),
    0xd89741bc0a7a1ac8ull);
}

TEST(Wire, OtherVersionIsRefusedWithBothNumbers)
{
  auto b = unhex(kStateHex);
  b[4] = 2;
  std::string err;
  EXPECT_FALSE(w::decode_state(b.data(), b.size(), &err).has_value());
  EXPECT_NE(err.find('2'), std::string::npos) << err;
  EXPECT_NE(err.find('1'), std::string::npos) << err;
}

TEST(Wire, BadMagicKindAndLengthAreRefused)
{
  std::string err;
  auto b = unhex(kStateHex);
  b[0] ^= 1;
  EXPECT_FALSE(w::decode_state(b.data(), b.size(), &err).has_value());
  b = unhex(kStateHex);
  b[6] = 3;
  EXPECT_FALSE(w::decode_state(b.data(), b.size(), &err).has_value());
  b = unhex(kStateHex);
  EXPECT_FALSE(w::decode_state(b.data(), b.size() - 1, &err).has_value());
}
