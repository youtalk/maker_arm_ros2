// The Maker Arm as a ros2_control system: a client of `maker-arm-cli serve`, which owns the
// bus and the tracking loop and holds the arm whenever this plugin stops.
#pragma once

#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "maker_arm_hardware/daemon_link.hpp"

namespace maker_arm_hardware
{

class MakerArmSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish(const wire::StateFrame & s);

  DaemonOptions options_;
  std::unique_ptr<DaemonLink> link_;
  // Frame order: joint j's position, velocity and effort at 3j, 3j + 1 and 3j + 2.
  std::vector<hardware_interface::StateInterface::SharedPtr> states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
};

}  // namespace maker_arm_hardware
