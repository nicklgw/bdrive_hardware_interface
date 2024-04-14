
#include "bdrive_hardware_interface/bdrive_hardware_interface.hpp"

namespace bdrive_hardware_interface
{
hardware_interface::CallbackReturn BDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) 
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);

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

  motor_option_.can_interface = can_interface_;

  ConfigurationManager config(bus_config_);
  config.init_config();

  // Load joint data
  for (auto joint : info.joints)
  {
    motor::MotorOption::Motor motor;
    motor.motor_type = config.get_entry<int>(joint.parameters["device_name"], "motor_type").value();
    motor.node_id = config.get_entry<int>(joint.parameters["device_name"], "node_id").value();
    motor.operation_mode = config.get_entry<int>(joint.parameters["device_name"], "operation_mode").value();
    motor.profile_acc = config.get_entry<int>(joint.parameters["device_name"], "profile_acc").value();
    motor.profile_dec = config.get_entry<int>(joint.parameters["device_name"], "profile_dec").value();
    motor.max_speed = config.get_entry<int>(joint.parameters["device_name"], "max_speed").value();
    motor.profile_speed = config.get_entry<int>(joint.parameters["device_name"], "profile_speed").value();
    motor.encoder_dpi = config.get_entry<int>(joint.parameters["device_name"], "encoder_dpi").value();
    motor.install_direction = config.get_entry<int>(joint.parameters["device_name"], "install_direction").value();
    motor.home_trigger = config.get_entry<int>(joint.parameters["device_name"], "home_trigger").value();
    motor.en_profile_speed = config.get_entry<int>(joint.parameters["device_name"], "en_profile_speed").value();
    motor.reduction_ratio = config.get_entry<double>(joint.parameters["device_name"], "reduction_ratio").value();

    node_id_.emplace_back(motor.node_id);

    motor_option_.motors.emplace_back(motor);

    RCLCPP_INFO(bdrive_hardware_interface_logger, "joint '%s', node_id: %d", joint.name.c_str(), motor.node_id);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State&) 
{
  int ret = motor::Init(motor_option_);
  RCLCPP_INFO(bdrive_hardware_interface_logger, "motor::Init(): %d", ret);
  if (ret != 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  executor_ =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  motor_device_node_ = std::make_shared<rclcpp::Node>("motor_device");
  executor_->add_node(motor_device_node_);

  spin_thread_ = std::make_unique<std::thread>([this](){
    executor_->spin();
    executor_->remove_node(motor_device_node_);
    RCLCPP_INFO(motor_device_node_->get_logger(), "Stopped spinning BDriveHardwareInterface ROS2 executor");
  });

  std::string topic_device_discovery;
  motor_device_node_->get_parameter_or<std::string>("device_discovery_topic"
    , topic_device_discovery, "/rics_device_discovery");

  RCLCPP_INFO(bdrive_hardware_interface_logger, "device discovery topic: %s", topic_device_discovery.c_str());

  auto device_service_options = motor_device::DeviceServiceOptions{topic_device_discovery, rclcpp::SystemDefaultsQoS().keep_last(10)};

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto node_id = node_id_[i];
    auto& joint = info_.joints[i];

    motor_devices_.emplace_back(std::make_unique<motor_device::MotorDevice>(
      motor_device_node_, joint.name, "motor", ("rise-hw-id-" + std::to_string(i)), node_id))
    ->run_server(device_service_options);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
    hw_commands_velocities_[i] = 0.0;
    hw_commands_efforts_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // halt all motors
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    switch (control_level_[i]) 
    {
      case integration_level_t::POSITION:
        {
          motor::SetPosition(node_id_[i], 0.0);
        }
        break;
      case integration_level_t::VELOCITY:
        {
          motor::SetVelocity(node_id_[i], 0.0);  // 设置指定轮子角速度，单位rad/s
        }
        break;
      case integration_level_t::EFFORT:
        break;
      case integration_level_t::UNDEFINED:
        break;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&)
{
  clean();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDriveHardwareInterface::on_shutdown(const rclcpp_lifecycle::State&)
{
  clean();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BDriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BDriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type BDriveHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    hw_velocities_[i] = motor::GetVelocity(node_id_[i]);
    hw_positions_[i] = motor::GetPosition(node_id_[i]);

    motor_devices_[i]->read_motor_status();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BDriveHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    float input_torque = 0.0, input_vel = 0.0, input_pos = 0.0;

    switch (control_level_[i]) 
    {
      case integration_level_t::POSITION:
        {
          input_pos = hw_commands_positions_[i];
          motor::SetPosition(node_id_[i], input_pos);
        }
        break;
      case integration_level_t::VELOCITY:
        {
          input_vel = hw_commands_velocities_[i];
          motor::SetVelocity(node_id_[i], input_vel);  // 设置指定轮子角速度，单位rad/s
        }
        break;
      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        break;
      case integration_level_t::UNDEFINED:
        break;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("BDriveHardwareInterface"), 
    "i: %ld, node_id: %d, control_level: %d, input_pos: %.3f, input_vel: %.3f", 
    i, node_id_[i], static_cast<int32_t>(control_level_[i]), input_pos, input_vel);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BDriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
  for (std::string key : stop_interfaces) 
  {
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
      if (key.find(info_.joints[i].name) != std::string::npos) 
      {
        control_level_[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces) 
  {
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
      switch (control_level_[i]) 
      {
        case integration_level_t::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) 
          {
            control_level_[i] = integration_level_t::EFFORT;
          }

        case integration_level_t::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) 
          {
            control_level_[i] = integration_level_t::VELOCITY;
          }

        case integration_level_t::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) 
          {
            control_level_[i] = integration_level_t::POSITION;
          }

        case integration_level_t::POSITION:
          break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BDriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) 
{
  // register interfaces to start per device
  for (auto interface : start_interfaces)
  {
    RCLCPP_INFO(bdrive_hardware_interface_logger, "start.interface '%s'", interface.c_str());
  }

  // register interfaces to stop per device
  for (auto interface : stop_interfaces)
  {
    RCLCPP_INFO(bdrive_hardware_interface_logger, "stop.interface '%s'", interface.c_str());
  }

  for (size_t i = 0; i < info_.joints.size(); i++) 
  {
    // switch (control_level_[i]) 
    // {
    //   case integration_level_t::UNDEFINED:
    //     break;

    //   case integration_level_t::EFFORT:        
    //     motor::SetOperationMode(node_id_[i], 1);
    //     break;

    //   case integration_level_t::VELOCITY:
    //     motor::SetOperationMode(node_id_[i], 2);
    //     break;

    //   case integration_level_t::POSITION:
    //     motor::SetOperationMode(node_id_[i], 3);
    //     break;
    // }

    int node_id = node_id_[i];
    int operation_mode = static_cast<int>(control_level_[i]);
    int ret = motor::SetOperationMode(node_id, operation_mode);

    RCLCPP_INFO(bdrive_hardware_interface_logger, "perform_command_mode_switch node_id: %d, operation_mode: %d, ret: %d", node_id, operation_mode, ret);
  }

  RCLCPP_INFO(bdrive_hardware_interface_logger, "perform_command_mode_switch");
  return hardware_interface::return_type::OK;
}

void BDriveHardwareInterface::clean()
{
  //printf("Cancel exectutor...");
  if (executor_) {
    executor_->cancel();
  }
  //printf("Join spin thread...");
  if (spin_thread_) {
    spin_thread_->join();
  }

  //printf("Reset variables...");
  motor_device_node_.reset();
  executor_.reset();
  spin_thread_.reset();
  motor_devices_.clear();

  motor::Exit();
  RCLCPP_INFO(rclcpp::get_logger("BDriveHardwareInterface"), "motor::Exit()");
}

}  // namespace bdrive_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bdrive_hardware_interface::BDriveHardwareInterface, hardware_interface::SystemInterface)
