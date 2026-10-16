// The plugin's link to `maker-arm-cli serve`. It starts the daemon when nothing answers on the
// socket, checks the daemon's files by their digest, and then sends one frame for each State
// frame. It never stops, restarts or releases a daemon: the daemon holds the arm when this
// link closes for any reason.
#pragma once

#include <sys/types.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "maker_arm_hardware/wire.hpp"

namespace maker_arm_hardware
{

struct DaemonOptions
{
  std::string interface, profile, dynamics, params, residual, gripper_map,
    executable = "maker-arm-cli", socket, log;
  bool rt = false, persist = false;
};

// `$XDG_RUNTIME_DIR/maker-arm-<interface>.sock`, the daemon's `socket_path`. Throws when
// XDG_RUNTIME_DIR is not set.
std::string default_socket(const std::string & interface);

// The daemon's digest of its options (files.rs `load`): the interface, the bytes of each file
// (empty for an empty path) and the raw gripper_map text. Throws when a file cannot be read.
uint64_t options_digest(const DaemonOptions & o);

// `<executable> serve --flag=value ...` for these options, with the socket always named.
std::vector<std::string> serve_argv(const DaemonOptions & o);

class DaemonLink
{
public:
  // SPAWN_WAIT. INITIAL: spec section 6.2
  static constexpr std::chrono::seconds kSpawnWait{10};

  // Connects, or starts the daemon and waits up to spawn_wait for it, then says Hello as the
  // command client and checks the digest. Throws std::runtime_error with what to do.
  static std::unique_ptr<DaemonLink> open(
    const DaemonOptions & o, std::chrono::seconds spawn_wait = kSpawnWait);

  ~DaemonLink();
  DaemonLink(const DaemonLink &) = delete;
  DaemonLink & operator=(const DaemonLink &) = delete;

  // Each sends one frame and returns the daemon's State frame. Each throws std::runtime_error
  // on an I/O error, a closed socket or no reply within 50 ms, and the link is then closed.
  wire::StateFrame poll();
  wire::StateFrame control(wire::Op op);
  wire::StateFrame command(const std::array<double, 7> & q);

  // The daemon's pid, from SO_PEERCRED. Throws on a closed link.
  pid_t peer_pid() const;

private:
  explicit DaemonLink(int fd);
  void send(const std::vector<uint8_t> & frame);
  wire::StateFrame exchange(const std::vector<uint8_t> & frame, std::chrono::milliseconds wait);
  [[noreturn]] void fail(const std::string & why);

  int fd_;
  uint32_t seq_ = 0;
};

}  // namespace maker_arm_hardware
