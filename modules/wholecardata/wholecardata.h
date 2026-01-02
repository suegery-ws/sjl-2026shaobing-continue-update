#ifndef WHOLECARDATA_H
#define WHOLECARDATA_H

#include "struct_typedef.h"
#include "ins_task.h"
#include "message_center.h"
#include "bmi088.h"
#include "arm_math.h"
#include "controller.h"
#include "ins_task.h"



#define CHASSIS_CONTROL_FREQUENCE 500.0f //底盘任务控制频率


typedef  struct
{
  fp32 yaw_relative_angle;
  fp32 yaw_absoulte_angle;
  // fp32 yaw_motor_gyro;

}Yaw_Data_s;

typedef  struct
{
  fp32 pitch_relative_angle;
  fp32 pitch_absoulte_angle;
  // fp32 pitch_motor_gyro;

}Pitch_Data_s;

typedef  struct
{
  fp32 big_yaw_relative_angle;
  fp32 big_yaw_absoulte_angle;
  // fp32 big_yaw_motor_gyro;
  
}Big_Yaw_Data_s;

// typedef __packed struct
// {
//   fp32 relative_angle;
//   fp32 absoulte_angle;
//   fp32 gyro;
// }gimbal_posture_data_s;//每个电机实例都有这么个数据来反馈整车的数据,调参时只需到各个电机实例结构体中查看即可获得

typedef  struct
{
    Yaw_Data_s Yaw_Data;
    Pitch_Data_s Pitch_Data;
    Big_Yaw_Data_s Big_Yaw_Data;

}Gimbal_Data_s;

typedef  struct
{
  fp32 vx;
  fp32 vy;
  fp32 wz;
}Chassis_speed_data_s;

typedef  struct
{
  fp32 car_yaw_posture;
  fp32 car_roll_posture;
  fp32 car_pitch_posture;

}Chassis_posture_data_s;

// typedef  struct
// {
//   fp32 motor_lf_speed;
//   fp32 motor_rf_speed;
//   fp32 motor_lb_speed;
//   fp32 motor_rb_speed;

// }Chassis_motor_speed_data_s;

typedef  struct
{
  fp32 motor_lf_accel;
  fp32 motor_rf_accel;
  fp32 motor_lb_aceel;
  fp32 motor_rb_accel;
}Chassis_motor_accel_data_s;

typedef  struct
{
  Chassis_speed_data_s chassis_speed;
  Chassis_posture_data_s chassis_posture_data; 
  // Chassis_motor_speed_data_s chassis_motor_speed_data;
  Chassis_motor_accel_data_s chassis_motor_accel_data;
    
} Chassis_Data_s;

////////////////////////////////////////////////////云台部分姿态角获取//////////////////////////////////////////////////////////////
// void GetYawMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* yaw_motor,attitude_t* gimbal_IMU_data);
// void GetPitchMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* pitch_motor,attitude_t* gimbal_IMU_data);
// void GetBigYawMotorData(Gimbal_Data_s* gimbal_posture_data,DMMotorInstance *big_yaw_motor,attitude_t* gimbal_IMU_data);
// void GetChassisMotorData(Chassis_Data_s* chassis_data, Chassis_Ctrl_Cmd_s* chassis_cmd, attitude_t* Chassis_IMU_data, DJIMotorInstance* motor_lf, DJIMotorInstance* motor_rf ,DJIMotorInstance* motor_lb, DJIMotorInstance* motor_rb);
// ////////////////////////////////////////////////////底盘部分姿态角获取//////////////////////////////////////////////////////////////

//相对角度的获取直接写CMD层了，可以结合DMMotor的实例一起看
//底盘稍微缓一缓
#endif