#include "gimbal.h"
#include "can.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include <stdint.h>
#include "dmmotor.h"
#include "wholecardata.h"
#include "dmimu.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static dm_imu_data_t *dm_gimbal_imu_data;
static DJIMotorInstance *yaw_motor;// 云台电机实例指针
static DMMotorInstance *big_yaw_motor, *pitch_motor; // 大yaw4310电机实例指针
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Gimbal_Data_s* Gimbal_motor_posture_data;   //云台各个电机所对应整车部分的姿态解算，反馈数据,要用的话直接从这里调
 
// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    dm_gimbal_imu_data = DmimuInit(&hcan2);
    // YAW//下面可能会出现can总线拥堵问题，要注意
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .absoulte_angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0,//另外宏定义死区了，暂时应该没太大影响,不写也行
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            }, //这个不用
            .relative_angle_PID = {
                .Kp = 300.0f, // 300
                .Ki = 0.0f, //0
                .Kd = 2.0f, //2
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 50.0f, //数据待更改//用弧度制就注定pid给的比较大
            },
             .auto_angle_PID = {
                .Kp = 920.0f,  // 50
                .Ki = 300.0f, // 200
                .Kd = 2.0f,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 50.0f,
            },
            .speed_PID = {
                .Kp = 450.0f,  // 50
                .Ki = 0.8f, // 200
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000.0f,
                .MaxOut = 16384,
            },
           
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            .flag = 1,
            .motor_limit_left = 3.976f, //6.00 //4.96
            .motor_limit_right = 1.364f, //3.35 //4.49
        },
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020

        };//6020不需要电流环

    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x04,
            .rx_id = 2,
        },
        .controller_param_init_config = {
            .absoulte_angle_PID = {
                .Kp = 32.0f, // 32
                .Ki = 1.00f, //1
                .Kd = 0.0f, 
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100, //100
                .MaxOut = 6, //6
                .DeadBand = 0,
            },
            .relative_angle_PID = {
                .Kp = 23, // 10
                .Ki = 0.001,
                .Kd = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.05,
                .MaxOut = 5, 
            }, //用不到
            .auto_angle_PID = {
                .Kp = 40.0f,  
                .Ki = 30.0f, 
                .Kd = 0.0f,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 6.0f,
            },
            .speed_PID = {
                .Kp = 1.6f,  // 1.5
                .Ki = 0.0f, // 0
                .Kd = 0.0f,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000.0f,
                .MaxOut = 10.0f, //10
            },
            .other_angle_feedback_ptr = (&dm_gimbal_imu_data->oula_data.roll),
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&dm_gimbal_imu_data->gyro_data.x_gyro), //这个以后改达妙陀螺仪了
            .flag = 2,
            .motor_limit_left = 0.54f,//54
            .motor_limit_right = -0.57f,//-0.57
            
        },
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            
        },
        .motor_type = DM4310,};
    // BIG YAW
    Motor_Init_Config_s big_yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0X01,
            .rx_id = 3,
        },
        .controller_param_init_config = {
            .absoulte_angle_PID = {
                .Kp = 15, // 50
                .Ki = 0.0, //25
                .Kd = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 3,//底盘跟随云台专用
                .DeadBand = 0
            },
            .relative_angle_PID = {
                .Kp = 15, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 3, //
                .DeadBand = 0
            },
            .auto_angle_PID = {
                .Kp = 50.0f,  
                .Ki = 25.0f, 
                .Kd = 0.1f,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 3.0f,
            },
            .speed_PID = {
                .Kp = 7,  // 7
                .Ki = 0.0, // 0.1
                .Kd = 0.0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,//2500
                .MaxOut = 15, //10
                .DeadBand = 0 //0.02
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Yaw,
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            // .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[0]),//
            .flag = 3,  
        },
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = DM4310,//大yaw的大妙4310的相关函数要重写
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    big_yaw_motor = DMMotorInit(&big_yaw_config);
    pitch_motor = DMMotorInit(&pitch_config); 
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    Gimbal_motor_posture_data = (Gimbal_Data_s *)malloc(sizeof(Gimbal_Data_s));//分配反馈数据内存
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref//auto在后续的版本中将会优化
    switch (gimbal_cmd_recv.yaw_motor_mode)
    {
    case GIMBAL_MOTOR_RAW:
        DJIMotorStop(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);
        break;
    case GIMBAL_MOTOR_GYRO: 
        DJIMotorEnable(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);    //反馈数据处理函数                                                                    
        DJIModeChangeControlTransmit(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        DJIMotorRefVerify(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        break;
    case GIMBAL_MOTOR_ENCONDE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);
        DJIModeChangeControlTransmit(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        DJIMotorRefVerify(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        break;
    case GIMBAL_MOTOR_ROTATE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);
        DJIModeChangeControlTransmit(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        DJIMotorRefVerify(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        break;
    case GIMBAL_MOTOR_AUTO: //自动瞄准模式
        DJIMotorEnable(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);
        DJIModeChangeControlTransmit(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        DJIMotorRefVerify(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        break;
    case GIMBAL_MOTOR_AUTO_XUNLUO: //自动巡逻模式
        DJIMotorEnable(yaw_motor);
        DJIMotorinhert(&gimbal_cmd_recv, yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIGetYawMotorData(Gimbal_motor_posture_data,yaw_motor,gimbal_IMU_data);
        DJIModeChangeControlTransmit(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        DJIMotorRefVerify(&gimbal_cmd_recv,yaw_motor,Gimbal_motor_posture_data);
        break;
    default:
        break;
    }
   
    switch (gimbal_cmd_recv.pitch_motor_mode)    
    {
    case GIMBAL_MOTOR_RAW:
        DMMotorStop(pitch_motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE,pitch_motor);//解决莫名的失能问题
        DMMotorinhert(&gimbal_cmd_recv, pitch_motor);
        DMGet4310MotorData(Gimbal_motor_posture_data,pitch_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        break;
    case GIMBAL_MOTOR_GYRO: //基本上只用这个
        DMMotorEnable(pitch_motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE,pitch_motor);
        DMMotorinhert(&gimbal_cmd_recv, pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,pitch_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_ENCONDE: //基本不用
        DMMotorEnable(pitch_motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE,pitch_motor);
        DMMotorinhert(&gimbal_cmd_recv, pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP,MOTOR_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP,MOTOR_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,pitch_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_AUTO: 
        DMMotorEnable(pitch_motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE,pitch_motor);
        DMMotorinhert(&gimbal_cmd_recv, pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,pitch_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_AUTO_XUNLUO:
        DMMotorEnable(pitch_motor);
        DMMotorSetMode(DM_CMD_MOTOR_MODE,pitch_motor);
        DMMotorinhert(&gimbal_cmd_recv, pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,pitch_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,pitch_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    default:
        break;
    }
    switch (gimbal_cmd_recv.big_yaw_motor_mode)    
    {
    case GIMBAL_MOTOR_RAW:
        DMMotorStop(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        break;
    case GIMBAL_MOTOR_GYRO: //基本上只用这个
        DMMotorEnable(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMMotorChangeFeed(big_yaw_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(big_yaw_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_ENCONDE: //这个基本不用
        DMMotorEnable(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMMotorChangeFeed(big_yaw_motor, ANGLE_LOOP,MOTOR_FEED);
        DMMotorChangeFeed(big_yaw_motor, SPEED_LOOP,MOTOR_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_ROTATE: 
        DMMotorEnable(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMMotorChangeFeed(big_yaw_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(big_yaw_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data);
        break;
    case GIMBAL_MOTOR_AUTO: //特殊处理，当小yaw的位置被限住时大yaw会转动一段距离
        DMMotorEnable(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMMotorChangeFeed(big_yaw_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(big_yaw_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    case GIMBAL_MOTOR_AUTO_XUNLUO:
        DMMotorEnable(big_yaw_motor);
        DMMotorinhert(&gimbal_cmd_recv, big_yaw_motor);
        DMMotorChangeFeed(big_yaw_motor, ANGLE_LOOP,OTHER_FEED);
        DMMotorChangeFeed(big_yaw_motor, SPEED_LOOP,OTHER_FEED);
        DMGet4310MotorData(Gimbal_motor_posture_data,big_yaw_motor,gimbal_IMU_data,dm_gimbal_imu_data);
        DMModeChangeControlTransmit(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data);
        DMMotorRefVerify(&gimbal_cmd_recv,big_yaw_motor,Gimbal_motor_posture_data,dm_gimbal_imu_data,yaw_motor);
        break;
    default:
        break;
    }
    
    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = big_yaw_motor->measure.angle_single_round;//这个反馈的是哨兵的相对角度
    gimbal_feedback_data.gimbal_data = Gimbal_motor_posture_data;
    gimbal_feedback_data.small_gimbal_data = dm_gimbal_imu_data;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}