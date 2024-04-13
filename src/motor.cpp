
#include <stdarg.h>
#include "rclcpp/logging.hpp"
#include "motor_driver_adapter.h"
#include "motor_device/motor.hpp"

namespace motor
{

#define M_PI       3.14159265358979323846

//#define USE_PRINT_LOG 1
#define USE_ROS2_LOG 1

#ifndef MOTION_LOG_LEVEL

enum MOTION_LOG_LEVEL
{
	LOG_LEVEL_FATAL = (0),
	LOG_LEVEL_ERROR = (1),
	LOG_LEVEL_WARN	= (2),
	LOG_LEVEL_INFO	= (3),
	LOG_LEVEL_DEBUG = (4),
};
#endif

void SysLog(int level, const char* fmt, ...)
{
	char	log_data[8192] = { 0 };
	va_list arg;
	va_start(arg, fmt);
	vsnprintf(log_data, sizeof(log_data), fmt, arg);
	va_end(arg);

#if USE_SPD_LOG
	switch (level)
	{
	case LOG_LEVEL_FATAL:
		spdlog::critical(log_data);
		break;
	case LOG_LEVEL_ERROR:
		spdlog::error(log_data);
		break;
	case LOG_LEVEL_WARN:
		spdlog::warn(log_data);
		break;
	case LOG_LEVEL_INFO:
		spdlog::info(log_data);
		break;
	case LOG_LEVEL_DEBUG:
		spdlog::debug(log_data);
		break;
	default:
		spdlog::debug(log_data);
		break;
	}
#endif

#if USE_PRINT_LOG
	switch (level)
	{
	case LOG_LEVEL_FATAL:
		printf("\033[31mFATAL\033[0m: %s\n", log_data);
		break;
	case LOG_LEVEL_ERROR:
		printf("\033[31mERROR\033[0m: %s\n", log_data);
		break;
	case LOG_LEVEL_WARN:
		printf("\033[33mWARN\033[0m: %s\n", log_data);
		break;
	case LOG_LEVEL_INFO:
		printf("\033[32mINFO\033[0m: %s\n", log_data);
		break;
	case LOG_LEVEL_DEBUG:
		printf("\033[34mDEBUG\033[0m: %s\n", log_data);
		break;
	default:
		printf("DEBUG: %s\n", log_data);
		break;
	}
#endif

#ifdef USE_ROS2_LOG
	static rclcpp::Logger sLogger = rclcpp::get_logger("motor");
	switch (level)
	{
	case LOG_LEVEL_FATAL:
		RCLCPP_FATAL(sLogger, "%s", log_data);
		break;
	case LOG_LEVEL_ERROR:
		RCLCPP_ERROR(sLogger, "%s", log_data);
		break;
	case LOG_LEVEL_WARN:
		RCLCPP_WARN(sLogger, "%s", log_data);
		break;
	case LOG_LEVEL_INFO:
		RCLCPP_INFO(sLogger, "%s", log_data);
		break;
	case LOG_LEVEL_DEBUG:
	default:
		RCLCPP_DEBUG(sLogger, "%s", log_data);
		break;
	}
#endif
}

typedef void (*LogFunc)(int level, const char* fmt, ...);

#define MOTOR_NUM_MAX 16

/************************* 结构体 *************************/
//单个电机控制量信息
struct MCS_SingleMotor
{
    int     node_id       = 0; //电机node_id
    uint8_t mask          = 0; //掩码，从低到高按位表示速度、舵角、编码器、力矩的使能
    double  speed         = 0; //电机控制量，单位：（速度）rpm
    double  angle         = 0; //电机控制量，单位：（角度）deg
    double  position      = 0; //电机控制量，单位：（编码器）cnt
    double  torque        = 0; //电机控制量，力矩
    int     err_code      = 0; //错误字
    int     status        = 0; //状态字
    double  voltage       = 0; //电压
    double  current       = 0; //电流
    int     online_status = 0; // 在线状态
    int     control_mode  = 0; // 电机反馈控制字
    int     work_mode     = 0; // 电机反馈工作模式
    int     target_speed  = 0; // 电机反馈目标速度
    double  target_torque = 0; // 电机反馈力矩
};

//所有电机控制量
struct MCS_Motors
{
    MCS_SingleMotor motor[MOTOR_NUM_MAX];
};

static int32_t GetSpeedCallback(uint16_t node_id, double speed);
static int32_t GetPositionAngleCb(uint16_t node_id, int32_t position_angle);
static int32_t GetPositionCallback(uint16_t node_id, int32_t position);
static int32_t GetAngleCallback(uint16_t node_id, double angle);
static int32_t GetStaCodeCb(uint16_t node_id, int32_t sta_code);
static int32_t GetCurrentCb(uint16_t node_id, double current);
static int16_t GetVoltageCb(uint16_t node_id, int32_t result);
static int	   GetGlobalErrorCallBack(int32_t result);
static int32_t GetMotorErrorCallback(uint16_t node_id, int32_t err_code);
static int32_t GetControlCodeCallback(uint16_t node_id, int32_t control_mode);
static int32_t GetWorkStateCallback(uint16_t node_id, int8_t work_state);
static int32_t GetTargetSpeedCbCallback(uint16_t node_id, double target_speed);
static int32_t GetOnLineStateCallback(uint16_t node_id, int32_t online_status);

static LogFunc			        log_func  = SysLog;
static MCS_Motors 				motors_info;
//static IniFileData_t	   		3 = {{0}};
static int 					    motor_err;
static MCS_SingleMotor         *motor_info = motors_info.motor;
static MotorCallBackList_t		MotorListCallBack =  
{
	.GetSpeedCb				= GetSpeedCallback,	
	.GetPositionAngleCb		= GetPositionAngleCb,
	.GetPositionCb			= GetPositionCallback,
	.GetAngleCb				= GetAngleCallback,
	.GetStaCodeCb			= GetStaCodeCb,	
	.GetCurrentCb			= GetCurrentCb,
	.GetDcBusVoltageCb		= GetVoltageCb,
	.GetGlobalErrorCb		= GetGlobalErrorCallBack,
	.GetErrCodeCb			= GetMotorErrorCallback,
	.GetControlCode			= GetControlCodeCallback,
	.GetWorkState			= GetWorkStateCallback,
	.GetTargetSpeedCb		= GetTargetSpeedCbCallback,
	.GetOnLineState			= GetOnLineStateCallback
};

static int getIndexFromNodeId(int node_id, MCS_SingleMotor* info)
{
	int index = -1;
	if (info != NULL && node_id > 0 && node_id < 127)
	{
		for (int i = 0; i < MOTOR_NUM_MAX; i++)
		{
			if (node_id == info[i].node_id)
			{
				index = i;
				break;
			}
		}
	}
	return index;
}

static int32_t GetSpeedCallback(uint16_t node_id, double speed)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].speed = speed;

	return 0;
}

static int32_t GetPositionAngleCb(uint16_t node_id, int32_t position_angle)
{
	(void) node_id;
	(void) position_angle;

	return 0;
}

static int32_t GetPositionCallback(uint16_t node_id, int32_t position)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].position = position;
	
	return 0;
}

static int32_t GetAngleCallback(uint16_t node_id, double angle)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].angle = angle;
	return 0;
}

static int32_t GetStaCodeCb(uint16_t node_id, int32_t sta_code)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].status = sta_code;
	return 0;
}

static int32_t GetCurrentCb(uint16_t node_id, double current)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].current = current;
	return 0;
}

static int16_t GetVoltageCb(uint16_t node_id, int32_t result)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].voltage = result;
	return 0;
}

static int	   GetGlobalErrorCallBack(int32_t result)
{
	static int last_result_motor_err = 0;
	if (last_result_motor_err != result)
	{
		last_result_motor_err = result;
		log_func(LOG_LEVEL_INFO, "<%s,%d> motor err[%d]", __func__, __LINE__, result);
	}
	motor_err = result;
	return 0;
}

static int32_t GetMotorErrorCallback(uint16_t node_id, int32_t err_code)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].err_code = err_code;
	return 0;
}

static int32_t GetControlCodeCallback(uint16_t node_id, int32_t control_mode)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].control_mode = control_mode;

	return 0;
}

static int32_t GetWorkStateCallback(uint16_t node_id, int8_t work_state)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].work_mode = work_state;

	return 0;
}

static int32_t GetTargetSpeedCbCallback(uint16_t node_id, double target_speed)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].target_speed = target_speed;

	return 0;
}

static int32_t GetOnLineStateCallback(uint16_t node_id, int32_t online_status)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index < 0)
		return -1;
	motor_info[index].online_status = online_status;

	return 0;
}

// 左轮、右轮 序号index
int RIGHT = 1;
int LEFT = 0;

// 减速比 15:1  电机转15圈，轮子转1圈
//#define REDUCTION_RATIO 40.0
static double s_reduction_ratio[MOTOR_NUM_MAX];

int Init(const MotorOption& option) // 电机初始化
{
	int		 motor_init_flag = 0;
	IniFileData_t sg_ini;
	
	if (option.motors.size() > MAX_CAN_NODE_NUM) {
		log_func(LOG_LEVEL_ERROR
			, "option.motors.size()=%ld > MAX_CAN_NODE_NUM=%d", option.motors.size(), MAX_CAN_NODE_NUM);
		return -1;
	}

	sg_ini.can_node_num = option.motors.size();
    sprintf(sg_ini.can_interface, "%s", option.can_interface.c_str());
	
	for (size_t i = 0; i < option.motors.size(); ++i) {
		auto& motor = option.motors[i];
		sg_ini.node_info[i].motor_type = motor.motor_type;
		sg_ini.node_info[i].node_id = motor.node_id;
		sg_ini.node_info[i].operation_mode = motor.operation_mode;
		sg_ini.node_info[i].profile_acc = motor.profile_acc;
		sg_ini.node_info[i].profile_dec = motor.profile_dec;
		sg_ini.node_info[i].max_speed = motor.max_speed;
		sg_ini.node_info[i].profile_speed = motor.profile_speed;
		sg_ini.node_info[i].encoder_dpi = motor.encoder_dpi;
		sg_ini.node_info[i].install_direction = motor.install_direction;
		sg_ini.node_info[i].home_trigger = motor.home_trigger;
		sg_ini.node_info[i].en_profile_speed = motor.en_profile_speed;
		sg_ini.node_info[i].reduction_ratio = motor.reduction_ratio;

		motors_info.motor[i].node_id = motor.node_id;
		s_reduction_ratio[i] = motor.reduction_ratio;
	}

    motor_init_flag = MotorInit(&sg_ini, &MotorListCallBack);
    log_func(LOG_LEVEL_INFO, "<%s,%d> Init:%d ", __func__, __LINE__, motor_init_flag);
    
    return motor_init_flag;
}

int    Exit() // 电机退出
{
	MotorExit();
	return 0;
}

double GetVelocity(int node_id) // 获取指定轮子角速度, 单位rad/s
{
	int index = getIndexFromNodeId(node_id, motor_info);

	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0.0;
	}

	double motor_rpm = motor_info[index].speed;
	
	double motor_vel = motor_rpm * M_PI / 30;
	double wheel_vel = motor_vel / s_reduction_ratio[index];

    //log_func(LOG_LEVEL_INFO, "<%s,%d> GetVelocity index:%d, motor_rpm: %f, motor_vel: %f, wheel_vel: %f", __func__, __LINE__, index, motor_rpm, motor_vel, wheel_vel);
	
	return wheel_vel;
}

int SetVelocity(int node_id, double wheel_vel) // 设置指定轮子角速度，单位rad/s
{
	int index = getIndexFromNodeId(node_id, motor_info);

	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return -1;
	}

	double motor_vel = wheel_vel * s_reduction_ratio[index];
	double motor_rpm = motor_vel * 30 / M_PI;	
	
    //log_func(LOG_LEVEL_INFO, "<%s,%d> SetVelocity node_id: %d, motor_rpm: %f, motor_vel: %f, wheel_vel: %f", __func__, __LINE__, node_id, motor_rpm, motor_vel, wheel_vel);
	
	BzlSetMotorSpeed(node_id, motor_rpm);
	
	return 0;
}

double GetPosition(int node_id) // 获取指定轮子位置, 单位rad
{
	int index = getIndexFromNodeId(node_id, motor_info);

	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0.0;
	}

	double motor_position = motor_info[index].position;
	double wheel_cycles = motor_position  / 10000.0 / s_reduction_ratio[index]; // 转了多少圈
	double wheel_position = wheel_cycles * 2 * M_PI; // 单位rad
	
	return wheel_position;
}

int SetPosition(int node_id, double pos)  // 设置指定轮子位置, 单位rad
{
	double degree = pos / M_PI * 180; // 角度转为弧度
	
	BzlSetMotorAngle(node_id, degree);
	
	return 0;
}

int GetErrorCode(int node_id) 
{
	int index = getIndexFromNodeId(node_id, motor_info);

	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].err_code;
}

double GetTorque(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0.0;
	}

	return motor_info[index].torque;
}

int GetStatus(int node_id) 
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].status;
}

double GetVoltage(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0.0;
	}

	return motor_info[index].voltage;
}

double GetCurrent(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0.0;
	}

	return motor_info[index].current;
}

int GetOnlineStatus(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].online_status;
}

int GetControlMode(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].control_mode;
}

int GetWorkMode(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].work_mode;
}

int GetTargetSpeed(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].target_speed;
}

int GetTargetTorque(int node_id)
{
	int index = getIndexFromNodeId(node_id, motor_info);
	if (index == -1) [[unlikely]] {
		log_func(LOG_LEVEL_ERROR, "<%s,%d> invalid node_id: %d", __func__, __LINE__, node_id);

		return 0;
	}

	return motor_info[index].torque;
}

}
