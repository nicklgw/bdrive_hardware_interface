
#include "rclcpp_lifecycle/state.hpp"
#include "bdrive_hardware_interface/motor.hpp"
#include "bdrive_hardware_interface/motor_device.hpp"

namespace motor_device
{

void MotorDevice::reset() 
{
  // todo
}
  
void MotorDevice::enable() 
{
  // todo
}
  
void MotorDevice::disable() 
{
  // todo
}
  
void MotorDevice::self_test() 
{

}

void MotorDevice::read_motor_status() 
{
  auto motor_status = std::make_shared<MotorStatus>();

  motor_status->speed = motor::GetVelocity(node_id_);    // 获取指定轮子角速度, 单位rad/s
  motor_status->position = motor::GetPosition(node_id_);    // 获取指定轮子位置, 单位rad
  motor_status->error_code = motor::GetErrorCode(node_id_);   // 获取错误码
  motor_status->torque = motor::GetTorque(node_id_);      // 获取电机控制量，力矩
  motor_status->status = motor::GetStatus(node_id_);      // 获取状态字
  motor_status->voltage = motor::GetVoltage(node_id_);     // 获取电压
  motor_status->current = motor::GetCurrent(node_id_);     // 获取电流
  motor_status->online_status = motor::GetOnlineStatus(node_id_);// 获取在线状态
  motor_status->control_mode = motor::GetControlMode(node_id_); // 获取电机反馈控制字
  motor_status->work_mode = motor::GetWorkMode(node_id_);    // 获取电机反馈工作模式
  motor_status->target_speed = motor::GetTargetSpeed(node_id_); // 获取电机反馈目标速度
  motor_status->target_torque = motor::GetTargetTorque(node_id_);// 获取电机反馈力矩

  motor_status_.set(motor_status);
}

void MotorDevice::diagnostic_update(diagnostic_updater::DiagnosticStatusWrapper &stat) 
{
  std::shared_ptr<MotorStatus> motor_status;
  motor_status_.get(motor_status);

  if (!motor_status) return;

  if (motor_status->error_code != 0) {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR
      , "%s:%d", device_info_.device_name, motor_status->error_code);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }

  stat.addf("speed", "%lf", motor_status->speed);
  stat.addf("position", "%lf", motor_status->position); 
  stat.addf("torque", "%lf", motor_status->torque); 
  stat.addf("error_code", "%d", motor_status->error_code); 
  stat.addf("status", "%d", motor_status->status);
  stat.addf("voltage", "%lf", motor_status->voltage);
  stat.addf("current", "%lf", motor_status->current);
  stat.addf("online_status", "%d", motor_status->online_status);
  stat.addf("control_mode", "%d", motor_status->control_mode);
  stat.addf("work_mode", "%d", motor_status->work_mode);
  stat.addf("target_speed", "%d", motor_status->target_speed);
  stat.addf("target_torque", "%lf", motor_status->target_torque);
}

}  // namespace mecanum_device_controller
