// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BDRIVE_HARDWARE_INTERFACE_HPP_
#define BDRIVE_HARDWARE_INTERFACE_HPP_

#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "bdrive_hardware_interface/visibility_control.hpp"
#include "bdrive_hardware_interface/configuration_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bdrive_hardware_interface/motor.hpp"
#include "bdrive_hardware_interface/motor_device.hpp"

namespace bdrive_hardware_interface
{
class BDriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  BDriveHardwareInterface() : hardware_interface::SystemInterface() {}
  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  ~BDriveHardwareInterface() = default;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
  
  BDRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

private:
  void clean();

private:

  std::string bus_config_;
  std::string can_interface_;

  rclcpp::Logger bdrive_hardware_interface_logger = rclcpp::get_logger("bdrive_hardware_interface");

  std::vector<int> node_id_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  motor::MotorOption motor_option_;

// diagnotistic related
  std::shared_ptr<rclcpp::Node> motor_device_node_;
  std::vector<std::unique_ptr<motor_device::MotorDevice>> motor_devices_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spin_thread_;

  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };
  
  std::vector<integration_level_t> control_level_;

};
}  // namespace bdrive_hardware_interface

#endif  // BDRIVE_HARDWARE_INTERFACE_HPP_
