/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "ins_task.h"
#include "motor_def.h"
#include "rm_referee.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "wholecardata.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define CHASSIS_SPEED_VX_MAX 2.0f
#define CHASSIS_SPEED_VX_MIN -2.0f
#define CHASSIS_SPEED_VY_MAX 1.5f
#define CHASSIS_SPEED_VY_MIN -1.5f

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
// #ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
// #include "can_comm.h"
// #include "ins_task.h"
// static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
// #endif // CHASSIS_BOARD
// #ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
// #endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI//哨兵没有ui

static PIDInstance angle_PID; // 用于底盘自旋的角度环PID

static SuperCapInstance *cap;                                       // 超级电容//目前我们用超电的希望不大
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static Chassis_Data_s* chassis_data;
static attitude_t* Chassis_IMU_data;
static float feedback;
static float delat_angle;
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{   
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .other_speed_feedback_ptr = &feedback,
            .follow_speed_PID = {
                .Kp = 10000.0f,//20000.0f, // 4.5
                .Ki = 50,//50.0f,   // 0
                .Kd = 10,//0.0f,   // 0 //100
                .IntegralLimit = 2000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000.0f,
                .Output_LPF_RC = 0.3, //输出低通滤波时间常数
                .DeadBand = 0.03,
            },
            .rotate_speed_PID = {
                .Kp =8000,//2000.0f, // 4.5 //8000
                .Ki =50,//50.0f,   // 0
                .Kd =0,//0.0f,   // 0
                .IntegralLimit = 700.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 8000.0f,
                .Output_LPF_RC = 0.3, //输出低通滤波时间常数
                .DeadBand = 0,
            }
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag =1,
        
        },

        .motor_type = M3508,


    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    //使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.tx_id = 2;//1
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config); //左前

    chassis_motor_config.can_init_config.tx_id = 1;//4
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config);//右前

    chassis_motor_config.can_init_config.tx_id = 3;//2
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config); //左后

    chassis_motor_config.can_init_config.tx_id = 4;//3
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config); //右后

    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI 
    chassis_data = (Chassis_Data_s *)malloc(sizeof(Chassis_Data_s));//分配反馈数据内存

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 9.0f, //9
        .Ki = 0.001f,
        .Kd = 0.05f,
        .IntegralLimit = 0.2f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 10.0f,
        .DeadBand = 0
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化 //待调
    PID_Init_Config_s Angle_pid_conf = {
        .Kp = 8.0f,
        .Ki = 0.1f,
        .Kd = 0.05f,
        .IntegralLimit = 0.2f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 5.0f,
        .DeadBand = 0.05
    };
    PIDInit(&angle_PID, &Angle_pid_conf);

    // SuperCap_Init_Config_s cap_conf = { //超电如果有的话，在这里改
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 0x302, // 超级电容默认接收id
    //         .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //     }};
    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
// #ifdef CHASSIS_BOARD
//     Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

//     CANComm_Init_Config_s comm_conf = {
//         .can_config = {
//             .can_handle = &hcan2,
//             .tx_id = 0x311,
//             .rx_id = 0x312,
//         },
//         .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
//         .send_data_len = sizeof(Chassis_Upload_Data_s),
//     };
//     chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
// #endif                                          // CHASSIS_BOARD

// #ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
// #endif // ONE_BOARD //哨兵暂时没双板的打算
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)//暂时就先不用了

#define MOTOR_TO_CENTER 0.2465

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()//这里改成全向轮解算
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    vt_lb = chassis_vx*my_sin(angle_to_rad_45) - chassis_vy*my_cos(angle_to_rad_45) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//0
    vt_lf = -chassis_vx*my_sin(angle_to_rad_45) - chassis_vy*my_cos(angle_to_rad_45) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//3
    vt_rb = chassis_vx*my_sin(angle_to_rad_45) + chassis_vy*my_cos(angle_to_rad_45) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//1
    vt_rf = -chassis_vx*my_sin(angle_to_rad_45) + chassis_vy*my_cos(angle_to_rad_45) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//2

    // vt_lb = chassis_vx*my_sin(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_vy*my_cos(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//0
    // vt_lf = -chassis_vx*my_sin(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_vy*my_cos(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//3
    // vt_rb = chassis_vx*my_sin(angle_to_rad_45-chassis_cmd_recv.offset_angle) + chassis_vy*my_cos(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//1
    // vt_rf = -chassis_vx*my_sin(angle_to_rad_45-chassis_cmd_recv.offset_angle) + chassis_vy*my_cos(angle_to_rad_45-chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//2

    // vt_lb = chassis_vx*my_sin(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_vy*my_cos(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//0
    // vt_lf = -chassis_vx*my_sin(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_vy*my_cos(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//3
    // vt_rb = chassis_vx*my_sin(angle_to_rad_45+chassis_cmd_recv.offset_angle) + chassis_vy*my_cos(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//1
    // vt_rf = -chassis_vx*my_sin(angle_to_rad_45+chassis_cmd_recv.offset_angle) + chassis_vy*my_cos(angle_to_rad_45+chassis_cmd_recv.offset_angle) - chassis_cmd_recv.wz * MOTOR_TO_CENTER;//2

}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}
static void motor_speed_limit()
{
   if(chassis_vx >= CHASSIS_SPEED_VX_MAX)
   {
       chassis_vx = CHASSIS_SPEED_VX_MAX;
   }
   if (chassis_vx <= CHASSIS_SPEED_VX_MIN)
   {
       chassis_vx = CHASSIS_SPEED_VX_MIN;
   }

    if(chassis_vy >= CHASSIS_SPEED_VY_MAX)
    {
         chassis_vy = CHASSIS_SPEED_VY_MAX;
    }
    if (chassis_vy <= CHASSIS_SPEED_VY_MIN)
    {
         chassis_vy = CHASSIS_SPEED_VY_MIN;
    }

}
/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
// static void EstimateSpeed()
// {
//     // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
//     // chassis_feedback_data.vx vy wz =
//     //  ...
//     chassis_move_update->vx = -chassis_move_update->motor_chassis[0].speed * my_sin((45+angle_motion)*angle_to_radian) -chassis_move_update->motor_chassis[1].speed*my_cos((45+angle_motion)*angle_to_radian) 
// 															+chassis_move_update->motor_chassis[2].speed*my_sin((45+angle_motion)*angle_to_radian) +chassis_move_update->motor_chassis[3].speed*my_cos((45+angle_motion)*angle_to_radian) ;
		
// 	chassis_move_update->vy =  chassis_move_update->motor_chassis[0].speed*my_cos((45+angle_motion)*angle_to_radian) -chassis_move_update->motor_chassis[1].speed*my_sin((45+angle_motion)*angle_to_radian)
// 															-chassis_move_update->motor_chassis[2].speed*my_cos((45+angle_motion)*angle_to_radian) +chassis_move_update->motor_chassis[3].speed*my_sin((45+angle_motion)*angle_to_radian);
		
// 	chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
// }

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);


#endif // CHASSIS_BOARD

    Chassis_IMU_data = chassis_cmd_recv.IMU_data;
    DJIGetChassisMotorData(chassis_data,&chassis_cmd_recv,Chassis_IMU_data,motor_lf,motor_rf,motor_lb,motor_rb);
    chassis_behavior_to_motor(&chassis_cmd_recv);//将底盘的行为模式转换为电机控制模式
    //这边之后有必要的话会加入模式数据继承函数
    SetPowerLimit(100);//设置功率限制
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度 //这个写法集成性更强
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        // chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);//这个是不用pid的版本
        chassis_cmd_recv.wz = -PIDCalculate(&angle_PID, chassis_cmd_recv.offset_angle,0 );//前面可能有一个负号，这个用pid,角度环的输出结果就是速度目标值
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = -3;
        //这里之后加受击改速策略
        break;
    case CHASSIS_FOLLOW_ROS_FOLLOW_GIMBAL_YAW:  //自动模式底盘跟随云台
        chassis_cmd_recv.wz = -PIDCalculate(&angle_PID, chassis_cmd_recv.offset_angle,0 );//只需要下x,y的速度
    case CHASSIS_NO_FOLLOW_YAW: //给定一个角度转过去
        delat_angle = chassis_cmd_recv.no_follow_yaw_angle - chassis_data->chassis_posture_data.car_yaw_posture;
        chassis_cmd_recv.wz = -PIDCalculate(&angle_PID, 0, delat_angle);//前面可能有一个负号，这个用pid,角度环的输出结果就是速度目标值
        break;
    case CHASSIS_AUTO_NO_FOLLOW_YAW: //哨兵变速小陀螺
       //变速逻辑后面再加
    case CHASSIS_AUTO_GUIDGENCE:  //哨兵旋转小陀螺自动导航，速度恒定，旋转速度由上位机给出，暂时写恒定
        chassis_cmd_recv.wz = -3;  //其实可以什么都不用写
    
        
    default:
        break;
    }//暂时就先只有这两种模式了

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    // cos_theta = arm_cos_f32(0 );
    // sin_theta = arm_sin_f32(0 );
    cos_theta = arm_cos_f32(-chassis_cmd_recv.offset_angle );
    sin_theta = arm_sin_f32(-chassis_cmd_recv.offset_angle );
    chassis_vx = chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
    chassis_vy = - chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    motor_speed_limit();
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    
   //对电机的参考速度进行限制

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    
    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();
    // motor_speed_limit();
    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}