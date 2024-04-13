
#include "bdrive_hardware_interface/bdrive_hardware_interface.hpp"

namespace bdrive_hardware_interface
{
hardware_interface::CallbackReturn BDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) 
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  bdrive_hardware_interface_logger = rclcpp::get_logger(info_.name + "_interface");
  RCLCPP_INFO(bdrive_hardware_interface_logger, "Registering hardware interface '%s'", info_.name.c_str());

  // Check bus config is specified.
  if (info_.hardware_parameters.find("bus_config") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      bdrive_hardware_interface_logger, "No bus_config parameter provided for '%s' hardware interface.",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }
  bus_config_ = info_.hardware_parameters["bus_config"];
  RCLCPP_INFO(bdrive_hardware_interface_logger, "'%s' has bus config: '%s'", info_.name.c_str(), bus_config_.c_str());

  // Check can_interface_name is specified.
  if (info_.hardware_parameters.find("can_interface_name") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      bdrive_hardware_interface_logger, "No can_interface_name parameter provided for '%s' hardware interface.",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }
  can_interface_ = info_.hardware_parameters["can_interface_name"];
  RCLCPP_INFO(
    bdrive_hardware_interface_logger, "'%s' has can interface: '%s'", info_.name.c_str(),
    can_interface_.c_str());

  ConfigurationManager config(bus_config_);
  config.init_config();

  // Load joint data
  for (auto joint : info.joints)
  {
    auto motor_type = config.get_entry<std::string>(joint.parameters["device_name"], "motor_type").value();
    RCLCPP_INFO(bdrive_hardware_interface_logger, "Motor type '%s', joint '%s'", motor_type.c_str(), joint.name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State&) 
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&)
{
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_shutdown(const rclcpp_lifecycle::State&)
{
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BDriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BDriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  return command_interfaces;
}

hardware_interface::return_type BDriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BDriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BDriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) 
{


  return hardware_interface::return_type::OK;
}

}  // namespace bdrive_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bdrive_hardware_interface::BDriveHardwareInterface, hardware_interface::SystemInterface)
