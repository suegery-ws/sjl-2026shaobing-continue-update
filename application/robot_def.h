/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#include "wholecardata.h"
#include <stdint.h>
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

// #define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
#define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 0.0f // 6020yaw电机位于中间位置时的C板陀螺仪反馈数据
#define YAW_6020_OFF_SET_RAD 4.68f // 6020yaw电机位于中间位置时电机弧度制的角度值
#define PITCH_6020_OFF_SET_RAD 3.08f // 6020pitch电机位于水平位置时电机弧度制的角度值

// #define BIG_YAW_CHASSIS_ANGLE_POS 0 //云台和底盘对齐时的4310位置值
// #define BIG_YAW_CHASSIS_ANGLE_ECD (BIG_YAW_CHASSIS_ANGLE_POS*6.25/8192) //把位置值转换成编码值
#define BIG_YAW_ZERO_OFFSET_ECD 8166 // 大云台中值编码值//如果要开0点保存，就要代码给0了，我们尽量避免
#define BIG_YAW_GYRO_OFFSET_RAD 0.0f // 大云台陀螺仪中值//与4310有类似机制
#define BIG_YAW_CHASSIS_ANGLE_ECD 7000 //这个存疑
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 4065      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 0.785    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出//弧度制
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
#define READY_TRIGGER_SPEED 3.0f    // 拨盘准备触发的速度,单位为rps(转每秒)
#define TurnBackSpeed -1.0f
#define BLOCK_TIME 40.0f          // 拨盘卡弹检测时间,单位为ms(毫秒)
#define BLOCK_TRIGGER_SPEED 0.1f   //拨弹轮反转速度，单位为rps
#define REVERSE_TIME 60      // 拨盘回复时间,单位为ms(毫秒) //20
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 400             // 纵向轴距(前进后退方向)    
#define TRACK_WIDTH 400             // 横向轮距(左右平移方向)
#define MOTOR_DISTANCE_TO_CENTER 200
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0//之后做非同轴双yaw可能会用到
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define BIG_YAW_MOTOR_TO_OUT 1.895f // 现在哨兵特有的非整数减速比问题，带来的误差靠前馈等消除不了

#define GYRO2GIMBAL_DIR_YAW 1  // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL -1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

//陀螺仪地址宏定义
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2 //看情况暂时是用不到了

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define angle_to_rad_45 0.785f

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define angle_to_radian  0.01745f//角度值和弧度值之间的转换

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_FOLLOW_ROS_FOLLOW_GIMBAL_YAW,//自动模式下的底盘跟随云台--3 这个模式下底盘跟随云台，数据来自上位机
    CHASSIS_AUTO_GUIDGENCE,         //叉乘小陀螺导航,这个是边转边走--4 这个模式下是边转边走，旋转速度恒定
    CHASSIS_AUTO_NO_FOLLOW_YAW,      //自瞄模式下的小陀螺，即底盘不跟随云台，但是云台可以自己转动并且会按照云台的方向进行运动 --5
    CHASSIS_NO_FOLLOW_YAW,      //和云台间没有任何联系，转过的角度自己给
    CHASSIS_OPEN,               //此模式下预设值成比例直接写进速度环
    CHASSIS_NO_MOVE,            //nuc控制下强行使电机无力
} chassis_mode_e; //顺序带来的影响未知（底盘行为模式），一个行为模式就够了


// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_INIT,           //初始化模式,上电后进入此模式,云台转到对齐位置,但由于遥控器信号不好，导致莫名其妙的回中，所以一般不用
    GIMBAL_CALI,           //校准模式
    GIMBAL_ABSOLUTE_ANGLE, //绝对角度模式，绝对角度模式只动大云台
    GIMBAL_RELATIVE_ANGLE, //相对角度模式，相对角度模式只动小云台
    GIMBAL_MOTIONLESS,     //跟随底盘
	GIMBAL_AUTO,           //自动模式,视觉控制
	GIMBAL_AIM_DEBUG       //调试模式,用于调试瞄准，暂时用不到

} gimbal_mode_e; //云台行为模式

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
    GIMBAL_MOTOR_AUTO,     //自瞄控制
    GIMBAL_MOTOR_ROTATE,   //小陀螺控制
} gimbal_motor_mode_e; //云台电机控制模式

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射    
    LOAD_1_BULLET, 
    LOAD_BURSTFIRE,
    LOAD_REVERSE,  // 反转
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角//就是相对角度
    float no_follow_yaw_angle; //自己给要转的角度
    chassis_mode_e chassis_mode;//底盘行为模式
    chassis_mode_e last_chassis_mode;//上一次底盘的行为模式
    //本来加了电机控制模式，想了想没有用，就不加了
    int chassis_speed_buff; // 底盘速度缓冲百分比,0~100%
    attitude_t* IMU_data;

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float big_yaw; //大云台的角度预设
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;//云台行为模式
    gimbal_motor_mode_e yaw_motor_mode;     //云台yaw电机控制模式
    gimbal_motor_mode_e big_yaw_motor_mode; //大云台电机控制模式
    gimbal_motor_mode_e pitch_motor_mode;   //云台俯仰电机控制模式

    gimbal_motor_mode_e last_yaw_motor_mode;     //上一次云台yaw电机控制模式
    gimbal_motor_mode_e last_big_yaw_motor_mode; //上一次大云台电机控制模式
    gimbal_motor_mode_e last_pitch_motor_mode;   //上一次云台俯仰电机控制模式
  //这里提醒一下，我对PUB结构体的修改很有可能导致数据的收发出现问题，这个东西我会再看一眼的
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;//发射模式
    loader_mode_e last_lode_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    int16_t shoot_flag; //单发限位标志位
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
// #if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
//     // attitude_t chassis_imu_data;
// #endif
//     // 后续增加底盘的真实速度
     float real_vx;
     float real_vy;
     float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_motor_single_round_angle; //相对角度
    Gimbal_Data_s* gimbal_data;
} Gimbal_Upload_Data_s;

typedef struct
{
    int16_t feedback_shoot_flag ;
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H