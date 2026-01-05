#include "shoot.h"
#include "can.h"
#include "motor_def.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖 //这东西跟我们没关系

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static float pid_ref = 0;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,//速度环就够了
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
     friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
     friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .absoulte_angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = -0.05,
                .Kd = 0,
                .MaxOut = 100,
            },
            .speed_PID = {
                .Kp = 20, // 10
                .Ki = 0.5, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 7000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);//拨弹论电机，单发限位其实可以用拨弹轮来做
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if(shoot_cmd_recv.load_mode == LOAD_1_BULLET && shoot_cmd_recv.last_lode_mode == LOAD_1_BULLET)
    // {
    //     if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;  
    // }
    
    // 休眠时间到达,重置休眠计时器

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJI2006MotorInhert(&shoot_cmd_recv, loader); // 继承cmd的控制权
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        loader->motor_settings.close_loop_type = SPEED_LOOP | CURRENT_LOOP; // 只开启速度环和电流环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:
    if(shoot_cmd_recv.shoot_flag == 1)//1
    {
        DJI2006MotorInhert(&shoot_cmd_recv, loader);                                                             
        loader->motor_settings.close_loop_type = ANGLE_LOOP|SPEED_LOOP; // 开启速度环和角度环双闭环控制
        loader->motor_settings.outer_loop_type = ANGLE_LOOP;
        loader->motor_controller.motor_mode = GIMBAL_MOTOR_GYRO; //写这个的目的完全是想要用绝对角度的pid控制，算是前面留的石了
        DJIMotorSetRef(loader, (loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE)); // 控制量增加一发弹丸的角度
        if(loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE <= -6.28f)
        {
            DJIMotorSetRef(loader, (loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE + 6.28f)); // 控制量增加一发弹丸的角度
        }
        // hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        // dead_time = 150;      
        // shoot_feedback_data.feedback_shoot_flag = 2;                                                 // 完成1发弹丸发射的时间
        break;
    }
    if(shoot_cmd_recv.shoot_flag == 2)
    {
        DJI2006MotorInhert(&shoot_cmd_recv, loader);
        DJIMotorSetRef(loader, loader->motor_controller.pid_ref); // 达到指定位置之前保持位置不变，持续pid控制
        shoot_feedback_data.feedback_shoot_flag = 2;// 达到指定位置之前保持位置不变，持续pid控制
    }
    if( loader->motor_controller.absoulte_angle_PID.Err >= -0.09f && shoot_cmd_recv.shoot_flag == 2)
    {
        DJI2006MotorInhert(&shoot_cmd_recv, loader);
        shoot_feedback_data.feedback_shoot_flag = 0; //发射完成反馈给cmd
        break;
    }
    break;
    case LOAD_BURSTFIRE:
        DJI2006MotorInhert(&shoot_cmd_recv, loader);
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        loader->motor_settings.close_loop_type = SPEED_LOOP;
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 6.28 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
    //如果在连发模式下电机速度过低说明卡弹，停止拨盘，下电再上电即可解决问题(最低级方法)
    //  if(loader->motor_controller.shoot_mode == LOAD_BURSTFIRE && loader->measure.speed_aps < 0.1f)
    // {
    //     DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     loader->motor_settings.close_loop_type = SPEED_LOOP | CURRENT_LOOP; // 只开启速度环和电流环
    //     DJIMotorSetRef(loader, TurnBackSpeed);             
    // }

      trigger_motor_turn_back(loader);//反转处理

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case BIG_AMU_10: //20m/s
            DJIMotorSetRef(friction_l, 952);
            DJIMotorSetRef(friction_r, 952);
            break;
        case SMALL_AMU_18: //18m/s
            DJIMotorSetRef(friction_l, 857);
            DJIMotorSetRef(friction_r, 857);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}