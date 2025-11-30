// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "tongjimachine/message.h"  //这个应该是绝对路径，clangd找不到我的文件夹
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static RC_ctrl_t *rc_data_last;         // 上一时刻遥控器数据,用于按键边沿检测
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据
static cboard_recv_message_t *tongji_vision_recv_data; // 同济视觉接收数据指针,初始化时返回
static cboard_send_message1_t tongji_vision_send_data_1;  // 同济视觉发送数据1
static cboard_send_message2_t tongji_vision_send_data_2;  // 同济视觉发送数据2

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static attitude_t *IMU_data; //imu数据就直接放命令层了，到时候直接通过发布者发给订阅者

static int16_t mode_shoot_flag = 0; // 发射标志位,12345分别表示不同的拨弹模式
// BMI088Instance *bmi088_test; // 云台IMU
// BMI088_Data_t bmi088_data;

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad * (1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	double flag = 1;
	
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

double my_cos(double rad)
{
	double  flag = 1;
	rad += ONE_PI/2.0;
   
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}
	return my_sin(rad)*flag;
}



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个，这个串口与我们的车一样
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口，这个没问题
    //这边加一个can初始化函数当作视觉部分的初始化
    tongji_vision_recv_data = TongjiVisionInit(&hcan1); // 同济视觉通信can口初始化
    // IMU_data = INS_Init();//反馈陀螺仪数据指针
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容 ？？？
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // BIG_GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    shoot_cmd_send.load_mode = LOAD_STOP;
}
/**
 * @brief 底盘行为模式转换为电机控制模式
 * 后来发现这鬼东西一点用都没有
 */
// static void chassis_behavior_to_motor()
// {
//     if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_VECTOR_RAW; 
//     }
//     else if (chassis_cmd_send.chassis_mode == CHASSIS_NO_MOVE)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_VECTOR_RAW; 
//     }
//     else if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_VECTOR_ROTATE;
//     }
// 	else if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_ROS)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_AUTO_GUIDGENCE;
//     }
// 	else if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_ROS_FOLLOW_GIMBAL_YAW)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_AUTO_GUIDGENCE_FOLLOW_GIMBAL_YAW;
//     }
//     else if (chassis_cmd_send.chassis_mode == CHASSIS_OPEN)
//     {
//         chassis_cmd_send.chassis_motor_mode = CHASSIS_VECTOR_RAW;
//     }
// };
static void gimbal_behavior_to_motor()
{
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE)//无力
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_RAW;
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_RAW, 
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_RAW;
    }
    // else if (gimbal_cmd_send.gimbal_mode == GIMBAL_INIT)//初始化，但回中值模式总是莫名其妙的回中，所以一般不用
    // {
    //     gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;
	// 	gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_GYRO;
    //     gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_ENCONDE;
    // }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_CALI)//校准
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_RAW;
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)//底盘跟随云台
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_RAW;
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_RELATIVE_ANGLE)//小陀螺
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;  
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_ROTATE;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_cmd_send.gimbal_mode == GIMBAL_MOTIONLESS)//并不知道这是干嘛的
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }    
	else if (gimbal_cmd_send.gimbal_mode == GIMBAL_AUTO)//自瞄模式
    {
        gimbal_cmd_send.yaw_motor_mode = GIMBAL_MOTOR_AUTO;
		gimbal_cmd_send.big_yaw_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_cmd_send.pitch_motor_mode = GIMBAL_MOTOR_AUTO;
    }
    
}
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float total_angle;
    chassis_cmd_send.offset_angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度，单圈角度就是电机此时转到的角度，这里我直接在回调函数里写完了
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;  
// #endif




}//计算底盘和云台的相对角度，这个要改



/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    static int16_t yaw_channel = 0;
    static int16_t pitch_channel = 0;
    static int16_t big_yaw_channel = 0;
    static int16_t vx_channel=0, vy_channel=0;
    static float vx_set_channel=0, vy_set_channel=0;
    static first_order_filter_type_t vx_filter, vy_filter;
    static int16_t flag = 0; //单发限位标志位
    // const fp32 num[2] = {0.2f,0.2f}; // 一阶低通滤波参数,时间常数,需要调整
    
    // first_order_filter_init(&vx_filter, 0.5, num);
    // first_order_filter_init(&vy_filter, 0.5, num+1);

    chassis_cmd_send.last_chassis_mode = chassis_cmd_send.chassis_mode;//底盘的数据继承
    gimbal_cmd_send.last_big_yaw_motor_mode = gimbal_cmd_send.big_yaw_motor_mode;
    gimbal_cmd_send.last_pitch_motor_mode = gimbal_cmd_send.pitch_motor_mode;
    gimbal_cmd_send.last_yaw_motor_mode = gimbal_cmd_send.yaw_motor_mode; //为模式切换的数据继承做准备
    
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],无力模式,底盘和云台均不动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; 
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;    
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘跟随云台模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; 
        gimbal_cmd_send.gimbal_mode = GIMBAL_ABSOLUTE_ANGLE;    
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_RELATIVE_ANGLE;
    }
    else // 右侧开关状态异常,默认跟随模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }//感觉用不到
    gimbal_behavior_to_motor();

    // 云台参数,确定云台控制数据
    // if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    // {
    //     // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    //     // ...
    // }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    // { // 按照摇杆的输出大小进行角度增量,增益系数需调整
    if(gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE)//无力模式///////////////////////////////////////////////////////云台
       {
            gimbal_cmd_send.yaw = 0;
            gimbal_cmd_send.pitch = 0;
            gimbal_cmd_send.big_yaw = 0;
       }
         
    if(gimbal_cmd_send.gimbal_mode == GIMBAL_ABSOLUTE_ANGLE)//这个是哨兵的底盘跟随云台 //底盘和云台的模式选择以及云台的行为和控制模式的强大关联有待研究
       {
        //  if (rc_data[TEMP].rc.rocker_l_ == NULL || rc_data[TEMP].rc.rocker_l1 == NULL )
        // {
        // return;
        // }

    rc_deadband_limit(rc_data[TEMP].rc.rocker_l_ , yaw_channel, GIMBAL_RC_DEADBAND);
    rc_deadband_limit(rc_data[TEMP].rc.rocker_l1 , pitch_channel, GIMBAL_RC_DEADBAND);

    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = pitch_channel * PITCH_RC_SEN;
    gimbal_cmd_send.big_yaw = yaw_channel * BIG_YAW_RC_SEN; 

       }
    if(gimbal_cmd_send.gimbal_mode == GIMBAL_RELATIVE_ANGLE)//小陀螺模式
       {
        //  if (rc_data[TEMP].rc.rocker_l_ == NULL || rc_data[TEMP].rc.rocker_l1 == NULL )
        // {
        // return;
        // }

    rc_deadband_limit(rc_data[TEMP].rc.rocker_l_ , yaw_channel, GIMBAL_RC_DEADBAND);
    rc_deadband_limit(rc_data[TEMP].rc.rocker_l1 , pitch_channel, GIMBAL_RC_DEADBAND);

    gimbal_cmd_send.big_yaw = 0;
    gimbal_cmd_send.pitch = pitch_channel * PITCH_RC_SEN;
    gimbal_cmd_send.yaw = yaw_channel * YAW_RC_SEN;
       }

        // gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        // gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
        // gimbal_cmd_send.big_yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;//是否动用这个还需要判断条件//湖大老代码
    // }
    // 云台软件限位

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整  ////////////////////////////////////////////////////////////底盘
    if(chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_cmd_send.vx = 0;
        chassis_cmd_send.vy = 0;
    }
    if(chassis_cmd_send.chassis_mode == CHASSIS_ROTATE || chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        
		rc_deadband_limit(rc_data[TEMP].rc.rocker_r_, vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(rc_data[TEMP].rc.rocker_r1, vy_channel, CHASSIS_RC_DEADLINE);
		
		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;   //遥控器输入值需要有一个转换变成速度值
		vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

        chassis_cmd_send.vx = vx_set_channel;
        chassis_cmd_send.vy = vy_set_channel;
       }         
        //应该能用
    
     
    // 1数值方向
    // chassis_cmd_send.IMU_data = IMU_data;
    // gimbal_cmd_send.IMU_data = IMU_data;



    ///////////////////////////////////发射机构////////////////////////////////////////////////////////////////////////////////////////////////shoot
    // 发射参数
    shoot_cmd_send.last_lode_mode = shoot_cmd_send.load_mode;//发射的数据继承
    shoot_cmd_send.shoot_flag = shoot_fetch_data.feedback_shoot_flag;
        osDelay(10);//之后可以改用DWTDELAY
    if((switch_is_up(rc_data[TEMP].rc.switch_left))&&(!switch_is_up(rc_data[LAST].rc.switch_left))&&(shoot_cmd_send.friction_mode == FRICTION_OFF))//默认摩擦轮关闭，上拨一下打开，再拨到上面关闭
    {
       shoot_cmd_send.friction_mode = FRICTION_ON;
    }
    else if(switch_is_up(rc_data[TEMP].rc.switch_left)&&!switch_is_up(rc_data[LAST].rc.switch_left)&&shoot_cmd_send.friction_mode != FRICTION_OFF)
    {
       shoot_cmd_send.friction_mode = FRICTION_OFF;
    }

    if(switch_is_down(rc_data[TEMP].rc.switch_left)&&!switch_is_down(rc_data[LAST].rc.switch_left))
    {
         osDelay(10);

       mode_shoot_flag++;
       
       if(mode_shoot_flag == 3)
       {
        mode_shoot_flag = 0;
        shoot_cmd_send.load_mode = LOAD_STOP;
       }
       else
       {shoot_cmd_send.load_mode = mode_shoot_flag;} //连发模式切换

       if(shoot_cmd_send.load_mode == LOAD_1_BULLET && shoot_cmd_send.shoot_flag == 0)//单发模式下做一个限位
       {
         shoot_cmd_send.shoot_flag = 1;
       }
       else
       {
         shoot_cmd_send.shoot_flag = 2;
       }
    }

    shoot_cmd_send.shoot_rate = 20;//射频固定8发每秒
    shoot_cmd_send.bullet_speed = BIG_AMU_10;//设置弹速
}

// 下面是源代码
// static void RemoteControlSet()
// {
//     // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
//     if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
//     {
//         chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
//         gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
//     }
//     else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
//     {
//         chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
//         gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
//     }

//     // 云台参数,确定云台控制数据
//     if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
//     {
//         // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
//         // ...
//     }
//     // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
//     if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
//     { // 按照摇杆的输出大小进行角度增量,增益系数需调整
//         gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
//         gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
//     }
//     // 云台软件限位

//     // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
//     chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
//     chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向

//     // 发射参数
//     if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
//         ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
//     else
//         ; // 弹舱舵机控制,待添加servo_motor模块,关闭

//     // 摩擦轮控制,拨轮向上打为负,向下为正
//     if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
//         shoot_cmd_send.friction_mode = FRICTION_ON;
//     else
//         shoot_cmd_send.friction_mode = FRICTION_OFF;
//     // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
//     if (rc_data[TEMP].rc.dial < -500)
//         shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
//     else
//         shoot_cmd_send.load_mode = LOAD_STOP;
//     // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
//     shoot_cmd_send.shoot_rate = 8;
// }


/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}//哨兵不需要键盘操控

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 600 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    if(rc_data[TEMP].rc.dial > 300)
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
    if(rc_data[TEMP].rc.dial < -300)
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     robot_state = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     LOGINFO("[CMD] reinstate, robot ready");
    // }//暂时不需要
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);//接受来自三个关键部分的数据

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();//由于大小yaw的存在，所以这个函数要改
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    RemoteControlSet();
    // else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    //MouseKeySet();
    chassis_cmd_send.IMU_data = &gimbal_fetch_data.gimbal_imu_data;//把在云台初始化的陀螺仪的地址传到了底盘里面
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据 
    //  VisionSetFlag(chassis_fetch_data.enemy_color,chassis_fetch_data.bullet_speed);
    ////////////////////////////////////////////////////////////////////////////////////TongjiVisionSetFlag(double bullet_speed, Mode mode, ShootMode shoot_mode, double ft_angle);
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
