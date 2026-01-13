/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "robot_cmd.h"
#include "stdint.h"
#include "daemon.h"
#include "robot_def.h"
#include "struct_typedef.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "user_lib.h"
#include "wholecardata.h"
#include <stdint.h>


#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#define ECD_RAD_COEF_DJI 0.0007666015625 //(2PI/8192),将编码器值转化为弧度制
#define HALF_ECD_RANGE_DJI 4096   // 8192/2
#define HALF_RAD_RANGE_DJI 3.14 //PI
#define RAD_RANGE_DJI 6.28
#define ECD_RANGE_DJI 8192      // 8192
#define MOTOR_RPM_TO_GYRO 0.104719755119659774 //大疆6020转子到实际速度的一个转换比例
#define M2006_round_to_rad 0.174 //2006圈数转换成拨弹盘角度的比例 2PI/36

//m3508 rmp change to chassis speed,
//底盘m3508转化成电机速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//摩擦轮3508转换成电机速度(m/s)的比例
#define FRIC_RPM_TO_SPEED           0.00314159265358793f
//自动模式下小yaw每次运动的大小
#define YAW_EVERY_TIMR_ADD_L  0.002f 
#define YAW_EVERY_TIMR_ADD_R  -0.002f


/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    float speed_vector;       // 线速度,单位为:m/s，属于底盘电机
    float fric_speed_vector;  // 线速度，属于摩擦轮电机
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius
    int8_t total_round_flag;

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;              //看门狗计数器
    float dt;                       //两次喂狗的时间间隔
} DJIMotorInstance;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

void DJIMotorinhert(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DJIMotorInstance* Instance); //对电机控制模式的传承

fp32 DJI_ecd_to_angle_change(int32_t ecd, int32_t offset_ecd);

void DJIModeChangeControlTransmit(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DJIMotorInstance* Instance,Gimbal_Data_s* gimbal_posture_data);

void DJIMotorRefVerify(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv, DJIMotorInstance* gimbal_motor, Gimbal_Data_s* gimbal_posture_data); //对remote函数中的ref数值根据不同的电机控制模式进行精校

void DJIGimbalAutoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,Motor_Controller_s* gimbal_motor_control,Gimbal_Data_s* gimbal_posture_data,DJI_Motor_Measure_s* gimbal_motor_measure);

void DJIGimbalRefLimit(DJI_Motor_Measure_s* gimbal_motor_measure,Gimbal_Ctrl_Cmd_s* gimbal_cmd,Motor_Controller_s* gimbal_motor_control,Gimbal_Data_s* gimbal_posture_data);

void DJIGetYawMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* yaw_motor,attitude_t* gimbal_IMU_data);

void DJIGetPitchMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* pitch_motor,attitude_t* gimbal_IMU_data);

void DJIGetChassisMotorData(Chassis_Data_s* chassis_data, Chassis_Ctrl_Cmd_s* chassis_cmd, attitude_t* Chassis_IMU_data, DJIMotorInstance* motor_lf, DJIMotorInstance* motor_rf ,DJIMotorInstance* motor_lb, DJIMotorInstance* motor_rb);

void DJI2006MotorInhert(Shoot_Ctrl_Cmd_s* shoot_cmd_recv,DJIMotorInstance* Instance);

void trigger_motor_turn_back(DJIMotorInstance* motor);

void shoot_mode_message_change(DJIMotorInstance* loader, Shoot_Ctrl_Cmd_s* shoot_cmd_recv, Shoot_Upload_Data_s* shoot_feedback_data);
#endif // !DJI_MOTOR_H
