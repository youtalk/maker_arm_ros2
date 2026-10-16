// DaemonLink against a real `maker-arm-cli serve` on the sim interface. maker-arm-cli must be
// on PATH; without it, each test fails with the install line.
#include <gtest/gtest.h>

#include <signal.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "maker_arm_hardware/daemon_link.hpp"
#include "maker_arm_hardware/maker_arm_system.hpp"

namespace fs = std::filesystem;
namespace w = maker_arm_hardware::wire;
using maker_arm_hardware::DaemonLink;
using maker_arm_hardware::DaemonOptions;

namespace
{
// One test's socket, log and files. Under /tmp: a socket path holds at most 107 bytes.
struct Scratch
{
  fs::path dir;

  Scratch()
  {
    std::string t = "/tmp/maker-arm-hw-XXXXXX";
    if (!mkdtemp(t.data())) {throw std::runtime_error("mkdtemp failed");}
    dir = t;
  }
  ~Scratch() {fs::remove_all(dir);}

  std::string file(const std::string & name, const std::string & text) const
  {
    std::ofstream(dir / name) << text;
    return (dir / name).string();
  }
};

DaemonOptions sim_options(const Scratch & tmp)
{
  DaemonOptions o;
  o.interface = "sim";
  o.dynamics = FIXTURES "/dynamics_test.json";
  o.socket = (tmp.dir / "s.sock").string();
  o.log = (tmp.dir / "serve.log").string();
  return o;
}

bool gone_within(pid_t pid, std::chrono::milliseconds wait)
{
  const auto end = std::chrono::steady_clock::now() + wait;
  while (kill(pid, 0) == 0) {
    if (std::chrono::steady_clock::now() >= end) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return true;
}

std::string what_of_open(const DaemonOptions & o)
{
  try {
    DaemonLink::open(o);
  } catch (const std::runtime_error & e) {
    return e.what();
  }
  return "";
}
}  // namespace

TEST(DaemonLink, SpawnsADaemonInANewSession)
{
  Scratch tmp;
  auto link = DaemonLink::open(sim_options(tmp));
  EXPECT_NE(getsid(link->peer_pid()), getsid(0));
  EXPECT_EQ(link->poll().mode, w::Mode::Holding);
}

TEST(DaemonLink, ADigestMismatchRefusesAndNamesRelease)
{
  Scratch tmp;
  auto o = sim_options(tmp);
  o.persist = true;
  o.profile = tmp.file("a.json", R"({"max_velocity": 2.5})");
  pid_t pid = DaemonLink::open(o)->peer_pid();
  ASSERT_GT(pid, 1);
  o.profile = tmp.file("b.json", R"({"max_velocity": 2.0})");
  const auto e = what_of_open(o);
  EXPECT_NE(e.find("uses other files"), std::string::npos) << e;
  EXPECT_NE(e.find("release"), std::string::npos) << e;
  EXPECT_EQ(kill(pid, 0), 0) << "the refused daemon must keep running";
  kill(pid, SIGKILL);  // sim only: no motors to drop
}

TEST(DaemonLink, OnSimTheDaemonExitsWithItsClient)
{
  Scratch tmp;
  auto link = DaemonLink::open(sim_options(tmp));
  const pid_t pid = link->peer_pid();
  ASSERT_GT(pid, 1);
  link.reset();
  EXPECT_TRUE(gone_within(pid, std::chrono::seconds(2)));
}

TEST(DaemonLink, ASecondClientIsRefused)
{
  Scratch tmp;
  const auto o = sim_options(tmp);
  auto first = DaemonLink::open(o);
  const auto e = what_of_open(o);
  EXPECT_NE(e.find("another client is connected"), std::string::npos) << e;
  EXPECT_NE(e.find(o.log), std::string::npos) << e;
  EXPECT_EQ(first->poll().mode, w::Mode::Holding);
}

TEST(DaemonLink, AStaleSocketFileIsReplaced)
{
  Scratch tmp;
  const auto o = sim_options(tmp);
  tmp.file("s.sock", "stale");
  auto link = DaemonLink::open(o);
  EXPECT_EQ(link->poll().mode, w::Mode::Holding);
}

TEST(DaemonLink, AMissingExecutableNamesTheInstallCommand)
{
  Scratch tmp;
  auto o = sim_options(tmp);
  o.executable = "maker-arm-cli-missing";
  const auto e = what_of_open(o);
  EXPECT_NE(
    e.find(
      "cargo install --locked --git https://github.com/youtalk/maker-arm-rs --rev "
      "be17836308af8602fee5d319eb1565f30445636d maker-arm-cli"),
    std::string::npos) << e;
}

TEST(DaemonLink, ANanCommandGivesTheRefusedMode)
{
  Scratch tmp;
  auto link = DaemonLink::open(sim_options(tmp));
  EXPECT_EQ(link->control(w::Op::Activate).mode, w::Mode::Holding);
  auto q = link->poll().q;
  q[3] = std::nan("");
  const auto s = link->command(q);
  EXPECT_EQ(s.mode, w::Mode::Refused);
  EXPECT_EQ(s.refused, 1u);
}

TEST(DaemonLink, ADeadDaemonThrowsAndRaisesNoSigpipe)
{
  Scratch tmp;
  auto link = DaemonLink::open(sim_options(tmp));
  const pid_t pid = link->peer_pid();
  ASSERT_GT(pid, 1);
  kill(pid, SIGKILL);  // sim only: no motors to drop
  ASSERT_TRUE(gone_within(pid, std::chrono::seconds(2)));
  EXPECT_THROW(link->command(std::array<double, 7>{}), std::runtime_error);
  EXPECT_THROW(link->poll(), std::runtime_error);
}

namespace
{
using maker_arm_hardware::MakerArmSystem;

hardware_interface::HardwareComponentParams system_params(const DaemonOptions & o)
{
  hardware_interface::HardwareComponentParams p{};  // {} zero-fills is_async and rw_rate
  p.hardware_info.name = "MakerArm";
  p.hardware_info.type = "system";
  p.hardware_info.hardware_parameters = {{"interface", o.interface}, {"dynamics", o.dynamics},
    {"socket", o.socket}, {"log", o.log}, {"rt", "false"}, {"persist", "false"}};
  for (const char * j : {"link_002_joint", "link_003_joint", "link_004_joint", "link_005_joint",
      "link_006_joint", "link_007_joint", "gripper_left_joint"})
  {
    hardware_interface::ComponentInfo c;
    c.name = j;
    c.type = "joint";
    for (const char * i : {"position", "velocity", "effort"}) {
      c.state_interfaces.emplace_back().name = i;
    }
    c.command_interfaces.emplace_back().name = "position";
    p.hardware_info.joints.push_back(c);
  }
  return p;
}
}  // namespace

TEST(MakerArmSystem, RtAndPersistTakeOnlyTrueOrFalse)
{
  Scratch tmp;
  auto p = system_params(sim_options(tmp));
  EXPECT_EQ(MakerArmSystem().init(p), MakerArmSystem::CallbackReturn::SUCCESS);
  p.hardware_info.hardware_parameters["rt"] = "yes";
  EXPECT_EQ(MakerArmSystem().init(p), MakerArmSystem::CallbackReturn::ERROR);
  p.hardware_info.hardware_parameters["rt"] = "true";
  p.hardware_info.joints.pop_back();
  EXPECT_EQ(MakerArmSystem().init(p), MakerArmSystem::CallbackReturn::ERROR);
}

TEST(MakerArmSystem, ActivatesAtTheMeasuredPoseAndANanSendsNothing)
{
  using hardware_interface::return_type;
  Scratch tmp;
  MakerArmSystem sys;
  ASSERT_EQ(sys.init(system_params(sim_options(tmp))), MakerArmSystem::CallbackReturn::SUCCESS);
  sys.on_export_state_interfaces();
  sys.on_export_command_interfaces();
  const rclcpp_lifecycle::State none;
  ASSERT_EQ(sys.on_configure(none), MakerArmSystem::CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(none), MakerArmSystem::CallbackReturn::SUCCESS);
  const rclcpp::Time t;
  const rclcpp::Duration dt(0, 10'000'000);
  ASSERT_EQ(sys.read(t, dt), return_type::OK);
  const double q = sys.get_state("link_004_joint/position");
  EXPECT_NEAR(sys.get_command("link_004_joint/position"), q, 1e-3);
  EXPECT_EQ(sys.write(t, dt), return_type::OK);
  sys.set_command("link_004_joint/position", std::nan(""));
  EXPECT_EQ(sys.write(t, dt), return_type::OK);
  EXPECT_EQ(sys.read(t, dt), return_type::OK) << "a NaN must not reach the daemon";
  sys.set_command("link_004_joint/position", q + 0.2);
  EXPECT_EQ(sys.write(t, dt), return_type::OK);
  EXPECT_EQ(sys.read(t, dt), return_type::ERROR) << "a jump gives mode refused";
  EXPECT_EQ(sys.on_error(none), MakerArmSystem::CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.read(t, dt), return_type::ERROR);
}
