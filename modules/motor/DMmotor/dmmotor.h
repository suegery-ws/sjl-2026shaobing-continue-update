#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "robot_def.h"
#include "struct_typedef.h"
#include "wholecardata.h"
#include "robot_cmd.h"
#include "dmimu.h"
#include "lowpass_filter.h"
#include "dji_motor.h"

#define DM_MOTOR_CNT 2

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
//单位转换 

#ifndef MOTOR4310_ECD_TO_RAD
#define MOTOR4310_ECD_TO_RAD 0.000762939453125 //      2*  PI  /8192
#endif

#define HALF4310_ECD_RANGE  4096
#define PITCH_MID_POS 0
#define ECD4310_RANGE 8192

#define HALF4310_RAD_RANGE  3.14f
#define RAD4310_RANGE       6.28f

#define PITCH_4310_EVERY_RAD_ADD_UP 0.002
#define PITCH_4310_EVERY_RAD_ADD_DOWN -0.002
#define BIG_YAW_EVERY_L -0.014
#define BIG_YAW_EVERY_R 0.014

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;//圈数计算
    float angle_single_round;//反馈实时位置
    LowPassFilter_t position_filter; // 位置低通滤波器
    LowPassFilter_t velocity_filter;
}DM_Motor_Measure_s;//完美得MIT

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance absoulte_angle_PID;
    PIDInstance relative_angle_PID;
    PIDInstance auto_angle_PID;
    gimbal_motor_mode_e motor_mode;//电机控制模式的继承
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    int8_t flag; //预留标志位，表示是哪个电机

    fp32 motor_limit_left;
    fp32 motor_limit_right;

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;//看门狗写结构体里面
    uint32_t lost_cnt;
    int32_t ecd_sum;
    int32_t last_ecd;
    int32_t ecd;
    int32_t current_angle;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

fp32 motor4310_ecd_to_rad_change(int32_t ecd, int32_t offset_ecd);

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);

void DMMotorCaliEncoder(DMMotorInstance *motor);

void DMMotorControlInit();

void DMRelativeAngleCaculate();

void DMMotorControl();

void DMAngleCaculate(DMMotorInstance *motor);

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void DMMotorTask(void const *argument);

void DMMotorinhert(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DMMotorInstance* Instance);

void DMModeChangeControlTransmit(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DMMotorInstance* Instance,Gimbal_Data_s* gimbal_posture_data);

void DMMotorRefVerify(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv, DMMotorInstance* gimbal_motor, Gimbal_Data_s* gimbal_data, dm_imu_data_t* dm_imu_data, DJIMotorInstance* yaw_motor);

fp32 motor4310_gyro_control_change(float rad, float offset_rad);

void DMGet4310MotorData(Gimbal_Data_s* gimbal_posture_data,DMMotorInstance *motor,attitude_t* gimbal_IMU_data ,dm_imu_data_t* dmimudata);

void DMGimbalnNoLimitRef(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor,Gimbal_Data_s* gimbal_data);

void DMGimbalAutoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor, dm_imu_data_t* dm_imu_data);  //有限位

void DMGimbalAutoXunLuoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor, dm_imu_data_t* dm_imu_data);  //有限位

void DMGimbalnXunLuoNoLimitRef(DMMotorInstance* gimbal_motor,Gimbal_Data_s* gimbal_data); //无限位，小陀螺模式

void DMGimbalNUCAutoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor, dm_imu_data_t* dm_imu_data);

void DMGimbalnAutoNoLimitRef(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor,Gimbal_Data_s* gimbal_data,DJIMotorInstance* yaw_motor);  //自瞄模式下大yaw跟随

void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor);
#endif // !DMMOTOR