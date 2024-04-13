
#pragma once

#include "realtime_tools/realtime_box.h"

// #include "rics_device_interface/device_interface.hpp"

namespace motor_device
{
// using rics::device_interface::DeviceInterface;
class MotorDevice /*: public DeviceInterface*/
{
public:
  template <typename NodeT>
  MotorDevice(std::shared_ptr<NodeT> node, const std::string &device_name, const std::string& type, const std::string &hardware_id, int node_id)
  : /*DeviceInterface(node, device_name, type, hardware_id),*/ node_id_(node_id) 
  {

  }

  void read_motor_status();

  void reset() override;
  
  void enable() override;
  
  void disable() override;
  
  void self_test() override;

  void diagnostic_update(diagnostic_updater::DiagnosticStatusWrapper &stat) override;
 
protected:
  struct MotorStatus {
    double  speed         = 0; //电机控制量，单位：（速度）rpm
    //double  angle         = 0; //电机控制量，单位：（角度）deg
    double  position      = 0.0; //电机控制量，单位：（编码器）cnt
    double  torque        = 0.0; //电机控制量，力矩
    int     error_code      = 0; //错误字
    int     status        = 0; //状态字
    double  voltage       = 0.0; //电压
    double  current       = 0.0; //电流
    int     online_status = 0; // 在线状态
    int     control_mode  = 0; // 电机反馈控制字
    int     work_mode     = 0; // 电机反馈工作模式
    int     target_speed  = 0; // 电机反馈目标速度
    double  target_torque = 0.0; // 电机反馈力矩
  };

  realtime_tools::RealtimeBox<std::shared_ptr<MotorStatus>> motor_status_{nullptr};

  const int node_id_;
};

}  // namespace motor_device
