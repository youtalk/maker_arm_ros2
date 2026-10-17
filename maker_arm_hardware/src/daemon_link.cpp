#include "maker_arm_hardware/daemon_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace maker_arm_hardware
{
namespace
{
// The maker-arm-rs revision whose daemon this plugin speaks to. It installs from a clone:
// Ubuntu's apt cargo-1.85 cannot fetch a git URL (its system libgit2 is too old).
const char * const kInstall =
  "git clone https://github.com/youtalk/maker-arm-rs && "
  "git -C maker-arm-rs checkout fdda16e5bf05ee3a658bbc9de13dce84640dff33 && "
  "cargo install --locked --path maker-arm-rs/crates/maker-arm-cli";
// INITIAL: V1 (task 11) streams 100 Hz on SimArm through it, V2 measures the reply latency
constexpr std::chrono::milliseconds kReplyWait{50};
// The first reply also waits for the daemon's 10 ms accept tick and a new connection thread.
constexpr std::chrono::milliseconds kHelloWait{1000};
constexpr std::chrono::milliseconds kRetry{100};

std::string errno_text(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

std::string socket_of(const DaemonOptions & o)
{
  return o.socket.empty() ? default_socket(o.interface) : o.socket;
}

std::string log_of(const DaemonOptions & o)
{
  if (!o.log.empty()) {return o.log;}
  const char * home = std::getenv("HOME");
  return std::string(home ? home : "") + "/.ros/log/maker-arm-" + o.interface + ".log";
}

std::string read_file(const std::string & path)
{
  if (path.empty()) {return "";}
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("cannot read " + path);}
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

// `exe` when it names a path, else its first executable match on PATH; "" when there is none.
std::string which(const std::string & exe)
{
  if (exe.find('/') != std::string::npos) {return access(exe.c_str(), X_OK) == 0 ? exe : "";}
  const char * path = std::getenv("PATH");
  std::istringstream dirs(path ? path : "");
  for (std::string dir; std::getline(dirs, dir, ':'); ) {
    const auto p = (dir.empty() ? std::string(".") : dir) + "/" + exe;
    if (access(p.c_str(), X_OK) == 0) {return p;}
  }
  return "";
}

// A connected socket, or -1 when nothing listens on `path` (no file, or a stale one).
int connect_to(const std::string & path)
{
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  if (path.size() >= sizeof a.sun_path) {
    throw std::runtime_error(
            "socket path " + path + " is longer than " + std::to_string(sizeof a.sun_path - 1) +
            " bytes");
  }
  std::memcpy(a.sun_path, path.c_str(), path.size() + 1);
  const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {throw std::runtime_error(errno_text("socket"));}
  if (::connect(fd, reinterpret_cast<const sockaddr *>(&a), sizeof a) == 0) {return fd;}
  const int err = errno;
  ::close(fd);
  if (err == ENOENT || err == ECONNREFUSED) {return -1;}
  errno = err;
  throw std::runtime_error(errno_text("connect " + path));
}

// fork, setsid, fork, exec: the daemon has its own session, so a Ctrl-C on the launch never
// reaches it, and init adopts it, so it never stays a zombie of this process.
void spawn(const std::string & exe, const std::vector<std::string> & args, const std::string & log)
{
  if (const auto dir = std::filesystem::path(log).parent_path(); !dir.empty()) {
    std::filesystem::create_directories(dir);
  }
  std::vector<char *> argv;
  for (const auto & a : args) {argv.push_back(const_cast<char *>(a.c_str()));}
  argv.push_back(nullptr);
  const int in = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
  const int out = ::open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
  if (in < 0 || out < 0) {
    const auto why = errno_text("open " + log);
    ::close(in);
    ::close(out);
    throw std::runtime_error(why);
  }
  const pid_t child = ::fork();
  if (child == 0) {
    // Only async-signal-safe calls from here: the parent may run other threads.
    if (::setsid() < 0) {::_exit(1);}
    const pid_t daemon = ::fork();
    if (daemon != 0) {::_exit(daemon < 0 ? 1 : 0);}
    sigset_t none;
    sigemptyset(&none);
    sigprocmask(SIG_SETMASK, &none, nullptr);
    ::dup2(in, 0);
    ::dup2(out, 1);
    ::dup2(out, 2);
    ::close_range(3, UINT_MAX, 0);
    ::execv(exe.c_str(), argv.data());
    const char msg[] = "maker_arm_hardware: exec of the daemon failed\n";
    if (::write(2, msg, sizeof msg - 1) < 0) {}
    ::_exit(127);
  }
  const auto why = errno_text("fork");
  ::close(in);
  ::close(out);
  if (child < 0) {throw std::runtime_error(why);}
  while (::waitpid(child, nullptr, 0) < 0 && errno == EINTR) {}
}
}  // namespace

std::string default_socket(const std::string & interface)
{
  const char * dir = std::getenv("XDG_RUNTIME_DIR");
  if (!dir || !*dir) {
    throw std::runtime_error("XDG_RUNTIME_DIR is not set; set the hardware parameter socket");
  }
  return std::string(dir) + "/maker-arm-" + interface + ".sock";
}

uint64_t options_digest(const DaemonOptions & o)
{
  return wire::digest(
    {{"interface", o.interface},
      {"profile", read_file(o.profile)},
      {"dynamics", read_file(o.dynamics)},
      {"params", read_file(o.params)},
      {"residual", read_file(o.residual)},
      {"gripper_map", o.gripper_map}});
}

std::vector<std::string> serve_argv(const DaemonOptions & o)
{
  // `--flag=value`: a value that starts with '-' (a negative gripper slope, a path) stays a value.
  std::vector<std::string> a = {o.executable, "serve", "--interface=" + o.interface};
  for (const auto & [flag, value] : {std::pair{"--profile=", o.profile},
      {"--dynamics=", o.dynamics}, {"--params=", o.params}, {"--residual=", o.residual},
      {"--gripper_map=", o.gripper_map}})
  {
    if (!value.empty()) {a.push_back(flag + value);}
  }
  a.push_back("--socket=" + socket_of(o));
  if (o.rt) {a.push_back("--rt");}
  if (o.persist) {a.push_back("--persist");}
  return a;
}

std::unique_ptr<DaemonLink> DaemonLink::open(
  const DaemonOptions & o, std::chrono::seconds spawn_wait)
{
  // Before any spawn, a missing file fails here, with its name. A missing socket directory
  // does not: connect reads it as no daemon and spawns one. The daemon refuses the directory
  // before it opens the bus and exits, and the spawn timeout then points to its log.
  const uint64_t want = options_digest(o);
  const auto socket = socket_of(o);
  const auto log = log_of(o);
  int fd = connect_to(socket);
  if (fd < 0) {
    const auto exe = which(o.executable);
    if (exe.empty()) {
      throw std::runtime_error(
              o.executable + " is not on PATH. Install the daemon with `" + kInstall +
              "` and put ~/.cargo/bin on PATH.");
    }
    spawn(exe, serve_argv(o), log);
    const auto end = std::chrono::steady_clock::now() + spawn_wait;
    while ((fd = connect_to(socket)) < 0) {
      if (std::chrono::steady_clock::now() >= end) {
        throw std::runtime_error(
                "no daemon answered on " + socket + " within " +
                std::to_string(spawn_wait.count()) + " s: see " + log +
                ". Likely causes: another daemon holds the bus lock, a fault on the first "
                "tick, an enable refused because a joint is outside its limits, or an older "
                "maker-arm-cli that refuses these flags. Find a running daemon with "
                "`maker-arm-cli status --interface " + o.interface + "` and the bus holder "
                "with `ss -xlp | grep maker-arm-bus-" + o.interface + "`. For an older "
                "maker-arm-cli, install the pinned one with `" + kInstall + "`.");
      }
      std::this_thread::sleep_for(kRetry);
    }
  }
  std::unique_ptr<DaemonLink> link(new DaemonLink(fd));
  wire::StateFrame s;
  try {
    // The daemon answers no Hello. The Poll's State frame carries the digest.
    link->send(wire::encode_hello(wire::Role::Command));
    s = link->exchange(wire::encode_control(++link->seq_, wire::Op::Poll), kHelloWait);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
            std::string(e.what()) + " on " + socket +
            ": another client is connected, or the daemon speaks another protocol: see " + log +
            ". For another protocol, install the daemon this plugin speaks to with `" + kInstall +
            "`.");
  }
  if (s.digest != want) {
    throw std::runtime_error(
            "the running daemon on " + socket + " uses other files. Support the arm, run "
            "`ros2 run maker_arm_bringup release`, then launch again.");
  }
  return link;
}

DaemonLink::DaemonLink(int fd)
: fd_(fd) {}

DaemonLink::~DaemonLink()
{
  if (fd_ >= 0) {::close(fd_);}
}

wire::StateFrame DaemonLink::poll() {return control(wire::Op::Poll);}

wire::StateFrame DaemonLink::control(wire::Op op)
{
  return exchange(wire::encode_control(++seq_, op), kReplyWait);
}

wire::StateFrame DaemonLink::command(const std::array<double, 7> & q)
{
  return exchange(wire::encode_command(++seq_, q), kReplyWait);
}

pid_t DaemonLink::peer_pid() const
{
  ucred c{};
  socklen_t len = sizeof c;
  if (fd_ < 0 || ::getsockopt(fd_, SOL_SOCKET, SO_PEERCRED, &c, &len) < 0) {
    throw std::runtime_error("the link to the daemon is closed");
  }
  return c.pid;
}

void DaemonLink::fail(const std::string & why)
{
  ::close(fd_);
  fd_ = -1;
  throw std::runtime_error(why);
}

void DaemonLink::send(const std::vector<uint8_t> & frame)
{
  if (fd_ < 0) {throw std::runtime_error("the link to the daemon is closed");}
  // MSG_NOSIGNAL: a dead daemon gives EPIPE here, not a SIGPIPE that ends the controller manager.
  if (::send(fd_, frame.data(), frame.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(frame.size())) {
    fail(errno_text("send to the daemon"));
  }
}

// After any failure the stream is out of step, so fail() closes it: a late reply is never read
// as the answer to a later frame.
wire::StateFrame DaemonLink::exchange(
  const std::vector<uint8_t> & frame, std::chrono::milliseconds wait)
{
  send(frame);
  std::array<uint8_t, wire::kStateLen> b;
  size_t got = 0;
  const auto end = std::chrono::steady_clock::now() + wait;
  while (got < b.size()) {
    const auto left = std::chrono::ceil<std::chrono::milliseconds>(
      end - std::chrono::steady_clock::now()).count();
    pollfd p{fd_, POLLIN, 0};
    const int r = left > 0 ? ::poll(&p, 1, static_cast<int>(left)) : 0;
    if (r == 0) {fail("no reply from the daemon within " + std::to_string(wait.count()) + " ms");}
    if (r < 0) {
      if (errno == EINTR) {continue;}
      fail(errno_text("poll"));
    }
    const ssize_t n = ::recv(fd_, b.data() + got, b.size() - got, 0);
    if (n == 0) {fail("the daemon closed the connection");}
    if (n < 0) {
      if (errno == EINTR) {continue;}
      fail(errno_text("receive from the daemon"));
    }
    got += static_cast<size_t>(n);
  }
  std::string err;
  const auto s = wire::decode_state(b.data(), b.size(), &err);
  if (!s) {fail(err);}
  if (s->seq != seq_) {
    fail("the daemon answered frame " + std::to_string(s->seq) + ", not " + std::to_string(seq_));
  }
  return *s;
}

}  // namespace maker_arm_hardware
