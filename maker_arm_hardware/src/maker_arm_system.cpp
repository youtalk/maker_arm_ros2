#include "maker_arm_hardware/maker_arm_system.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace maker_arm_hardware
{
namespace
{
using hardware_interface::return_type;

// The daemon's frame order.
const std::array<std::string, 7> kJoints = {"link_002_joint", "link_003_joint",
  "link_004_joint", "link_005_joint", "link_006_joint", "link_007_joint", "gripper_left_joint"};
const char * const kModes[] = {"holding", "following", "refused", "fault", "released"};

std::vector<std::string> names(const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  std::vector<std::string> n;
  for (const auto & i : interfaces) {n.push_back(i.name);}
  return n;
}

// The seven joints of maker_arm.ros2_control.xacro and the hardware parameters, or a throw.
DaemonOptions options_from(const hardware_interface::HardwareInfo & info)
{
  if (info.joints.size() != kJoints.size()) {
    throw std::runtime_error(
            "the arm has 7 joints, the description gives " + std::to_string(info.joints.size()));
  }
  const std::vector<std::string> command = {"position"};
  const std::vector<std::string> state = {"position", "velocity", "effort"};
  for (size_t j = 0; j < kJoints.size(); ++j) {
    if (info.joints[j].name != kJoints[j] ||
      names(info.joints[j].command_interfaces) != command ||
      names(info.joints[j].state_interfaces) != state)
    {
      throw std::runtime_error(
              "joint " + std::to_string(j) + " must be " + kJoints[j] +
              " with command interface position and state interfaces position, velocity, "
              "effort");
    }
  }
  auto param = [&](const std::string & name) {
      const auto it = info.hardware_parameters.find(name);
      return it == info.hardware_parameters.end() ? std::string() : it->second;
    };
  auto flag = [&](const std::string & name) {
      const auto v = param(name);
      if (v != "true" && v != "false") {
        throw std::runtime_error(
                "hardware parameter " + name + " is \"" + v + "\", not true or false");
      }
      return v == "true";
    };
  DaemonOptions o;
  o.interface = param("interface");
  o.profile = param("profile");
  o.dynamics = param("dynamics");
  o.params = param("params");
  o.residual = param("residual");
  o.gripper_map = param("gripper_map");
  if (!param("executable").empty()) {o.executable = param("executable");}
  o.socket = param("socket");
  o.log = param("log");
  o.rt = flag("rt");
  o.persist = flag("persist");
  if (o.interface.empty() || o.dynamics.empty()) {
    throw std::runtime_error("the hardware parameters interface and dynamics are required");
  }
  return o;
}
}  // namespace

MakerArmSystem::CallbackReturn MakerArmSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    options_ = options_from(info_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

MakerArmSystem::CallbackReturn MakerArmSystem::on_configure(const rclcpp_lifecycle::State &)
{
  // The daemon takes one command client: free our slot before we ask again.
  link_.reset();
  try {
    states_.clear();
    commands_.clear();
    for (const auto & j : kJoints) {
      for (const char * i : {"/position", "/velocity", "/effort"}) {
        states_.push_back(get_state_interface_handle(j + i));
      }
      commands_.push_back(get_command_interface_handle(j + "/position"));
    }
    link_ = DaemonLink::open(options_);
    RCLCPP_INFO(get_logger(), "connected to the daemon, pid %d", link_->peer_pid());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

MakerArmSystem::CallbackReturn MakerArmSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  link_.reset();
  return CallbackReturn::SUCCESS;
}

// Every command starts at the measured pose, so the first target is no jump.
MakerArmSystem::CallbackReturn MakerArmSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!link_) {return CallbackReturn::ERROR;}
  try {
    const auto s = link_->poll();
    publish(s);
    for (size_t j = 0; j < commands_.size(); ++j) {
      std::ignore = set_command(commands_[j], s.q[j], true);
    }
    const auto a = link_->control(wire::Op::Activate);
    if (a.mode != wire::Mode::Holding) {
      RCLCPP_ERROR(
        get_logger(), "the daemon did not activate: mode %s, refused %u, fault %u",
        kModes[static_cast<int>(a.mode)], a.refused, a.fault);
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "activate: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

MakerArmSystem::CallbackReturn MakerArmSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!link_) {return CallbackReturn::ERROR;}
  try {
    link_->control(wire::Op::Deactivate);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "deactivate: %s; the daemon holds the arm", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// Closing the link leaves the arm to the daemon, which holds it.
MakerArmSystem::CallbackReturn MakerArmSystem::on_error(const rclcpp_lifecycle::State &)
{
  link_.reset();
  return CallbackReturn::SUCCESS;
}

return_type MakerArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_) {return return_type::ERROR;}
  try {
    const auto s = link_->poll();
    publish(s);
    if (s.mode == wire::Mode::Holding || s.mode == wire::Mode::Following) {return return_type::OK;}
    RCLCPP_ERROR(
      get_logger(), "the daemon reports mode %s, refused %u, fault %u; it holds the arm",
      kModes[static_cast<int>(s.mode)], s.refused, s.fault);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "read: %s; the daemon holds the arm", e.what());
  }
  return return_type::ERROR;
}

return_type MakerArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_) {return return_type::ERROR;}
  std::array<double, 7> q;
  for (size_t j = 0; j < q.size(); ++j) {
    std::ignore = get_command(commands_[j], q[j], true);
    // NaN is a controller with no target yet: send nothing, and the daemon holds.
    if (std::isnan(q[j])) {return return_type::OK;}
  }
  try {
    link_->command(q);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "write: %s; the daemon holds the arm", e.what());
    return return_type::ERROR;
  }
  return return_type::OK;
}

void MakerArmSystem::publish(const wire::StateFrame & s)
{
  for (size_t j = 0; j < kJoints.size(); ++j) {
    std::ignore = set_state(states_[3 * j], s.q[j], true);
    std::ignore = set_state(states_[3 * j + 1], s.dq[j], true);
    std::ignore = set_state(states_[3 * j + 2], s.effort[j], true);
  }
}

}  // namespace maker_arm_hardware

PLUGINLIB_EXPORT_CLASS(maker_arm_hardware::MakerArmSystem, hardware_interface::SystemInterface)
