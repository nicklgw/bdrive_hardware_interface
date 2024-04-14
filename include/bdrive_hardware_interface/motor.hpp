#pragma once

#include <vector>
#include <string>

namespace motor
{

struct MotorOption {
    struct Motor {
        int motor_type = 0;
        int node_id = 1;
        int operation_mode = -3;
        int profile_acc = 100;
        int profile_dec = 100;
        int max_speed = 3000;
        int profile_speed = 1000;
        int encoder_dpi = 10000;
        int install_direction = 1;
        int home_trigger = 0;
        int en_profile_speed = 1;
        double reduction_ratio = 40.0;
    };

    std::string can_interface = "can0";
    std::vector<Motor> motors;
};

int    Init(const MotorOption& option); // 电机初始化
int    Exit(); // 电机退出

double GetVelocity(int node_id);              // 获取指定轮子角速度, 单位rad/s
int    SetVelocity(int node_id, double vel);  // 设置指定轮子角速度，单位rad/s
double GetPosition(int node_id);              // 获取指定轮子位置, 单位rad
int    SetPosition(int node_id, double pos);  // 设置指定轮子位置, 单位rad

int    GetErrorCode(int node_id);   // 获取错误码
double GetTorque(int node_id);      // 获取电机控制量，力矩
int GetStatus(int node_id);         // 获取状态字
double GetVoltage(int node_id);     // 获取电压
double GetCurrent(int node_id);     // 获取电流
int GetOnlineStatus(int node_id);   // 获取在线状态
int GetControlMode(int node_id);    // 获取电机反馈控制字
int GetWorkMode(int node_id);       // 获取电机反馈工作模式
int GetTargetSpeed(int node_id);    // 获取电机反馈目标速度
int GetTargetTorque(int node_id);   // 获取电机反馈力矩

int SetOperationMode(int node_id, int operation_mode);

}
