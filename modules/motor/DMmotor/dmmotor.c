#include "dmmotor.h"
#include "HT04.h"
#include "dmimu.h"
#include "memory.h"
#include "general_def.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "wholecardata.h"
#include <stdint.h>

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];//之后会改2
static osThreadId dm_task_handle[DM_MOTOR_CNT]; //static 函数要在.c文件中声明

fp32 motor4310_ecd_to_rad_change(int32_t ecd, int32_t offset_ecd)
{
	if(ecd>=0)
	{
	int32_t relative_ecd = ecd - offset_ecd;
	if (relative_ecd > HALF4310_ECD_RANGE)
    {
        relative_ecd -= ECD4310_RANGE;
    }
    else if (relative_ecd < -HALF4310_ECD_RANGE)
    {
        relative_ecd += ECD4310_RANGE;
    }
    return relative_ecd * MOTOR4310_ECD_TO_RAD;
	}
}//因为ecd_sum不可能是负值,不考虑为负的情况,暂时不用//规划了相对角度处于-180到180之间

fp32 motor4310_gyro_control_change(float rad, float offset_rad)
{
	
	float relative_rad = offset_rad - rad;
	  if (relative_rad > HALF4310_ECD_RANGE)
    {
        relative_rad -= ECD4310_RANGE;
    }
    else if (relative_rad < -HALF4310_ECD_RANGE)
    {
        relative_rad += ECD4310_RANGE;
    }

    // return relative_rad ;
	
	// }
} //存疑，不知道陀螺仪具体怎么算的

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 *///专门在达妙里面写这个是由于达妙有很多无符号整型量
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DMGimbalnXunLuoNoLimitRef(DMMotorInstance* gimbal_motor,Gimbal_Data_s* gimbal_data)  //无限位，小陀螺模式
{
    static fp32 angle_set;
    static fp32 add;
    add = BIG_YAW_EVERY_L;
    angle_set = gimbal_motor->pid_ref;  //在transit里把absolute_angle_set设置成了当前角度//pid_ref可能需要初始化
    gimbal_motor->pid_ref = rad_format(angle_set + add);  //更新为增加后的目标值，这个也不需要限幅//负号是为了向左转正确
     if(gimbal_motor->pid_ref == 0)
    {
        gimbal_motor->pid_ref = gimbal_data->Big_Yaw_Data.big_yaw_absoulte_angle;
    }
}

void DMGimbalnNoLimitRef(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor,Gimbal_Data_s* gimbal_data)  //无限位，小陀螺模式
{
    static fp32 angle_set;
    static fp32 add;
    add = gimbal_cmd->big_yaw;
    angle_set = gimbal_motor->pid_ref;  //在transit里把absolute_angle_set设置成了当前角度//pid_ref可能需要初始化
    gimbal_motor->pid_ref = rad_format(angle_set + add);  //更新为增加后的目标值，这个也不需要限幅//负号是为了向左转正确
     if(gimbal_motor->pid_ref == 0)
    {
        gimbal_motor->pid_ref = gimbal_data->Big_Yaw_Data.big_yaw_absoulte_angle; //拍完视频考虑删掉
    }
}


static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;//在初始化函数中，实例初始化剩下的两个部分分别被赋值并且被存入实例中
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针
    static int flag = 0;

    //不知道数据中的id和state的意义是什么

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    
    // 解析原始位置数据
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    float raw_position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
    // 使用低通滤波器对位置进行滤波，使数据变化更平稳
    measure->position = LowPassFilter_Update(&measure->position_filter, raw_position);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    float raw_velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
    measure->velocity = LowPassFilter_Update(&measure->velocity_filter, raw_velocity);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    //以下是对哨兵多圈计算的插入
    if(measure->position>=6.25&&measure->position<=12.5)
	measure->position-=6.25;
    if(measure->position<=0&&measure->position>-6.25)
	measure->position+=6.25;
    if(measure->position<=-6.25&&measure->position>-12.5)
	measure->position+=12.5;
    
    //下面是对编码总值的计算 //可以认为4310和大yaw轴是两个东西
    motor->last_ecd = motor->ecd;            //这个可能写的可能有问题，一开始可能会存在垃圾值
    motor->ecd = measure->position*8192/6.25;//把4310电机看作6020电机，用编码值对他进行运算
    if(flag <= 10)
	{
        motor->ecd_sum = motor->ecd;
        flag++;
    }
    measure ->angle_single_round = -rad_format(MOTOR4310_ECD_TO_RAD * (motor->ecd - motor->ecd_sum));//这个是相对角度，陀螺仪反馈的是绝对角度//给符号是因为想要统一往左转为负角度
    //不同位置的电机的相对角度和绝对角度并不互通，不能全部写在这里
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instace->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dm_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instace->tx_id);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->absoulte_angle_PID, &config->controller_param_init_config.absoulte_angle_PID);
    PIDInit(&motor->relative_angle_PID, &config->controller_param_init_config.relative_angle_PID);
    
    // 初始化位置低通滤波器 (CAN接收频率约1000Hz，截止频率设为100Hz)
    LowPassFilter_Init_ByFreq(&motor->measure.position_filter, 1000.0f, 30.0f);
    LowPassFilter_Init_ByFreq(&motor->measure.velocity_filter, 1000.0f, 50.0f);
    ///////////////////////////////////////////对反馈和前馈指针的初始化/////////////////////////////////////////////////////////////
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_settings.feedforward_flag = config->controller_setting_init_config.feedforward_flag;
    motor->motor_limit_left = config->controller_param_init_config.motor_limit_left;
    motor->motor_limit_right = config->controller_param_init_config.motor_limit_right;
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    motor->flag = config->controller_param_init_config.flag;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

// void DMMotorSetRef(DMMotorInstance *motor, float ref)
// {
//     motor->pid_ref = ref;
// }

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}




void DMMotorControl()
 {
  // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    // uint8_t group, num; // 电机组号和组内编号
    static float set = 0;        // 电机控制CAN发送设定值
    DMMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    DM_Motor_Measure_s *measure;           // 电机测量值
    static float pid_measure1, pid_ref2,pid_ref1,pid_measure,pid_ref;             // 电机PID测量值和设定值
    DMMotor_Send_s motor_send_mailbox;

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dm_motor_instance[i];
        motor_setting = &motor->motor_settings;
        measure = &motor->measure;
        pid_ref2 = motor->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

        // rc_deadband_limit(pid_ref2,pid_ref1,0.02f);//死区处理
        
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref2 *= -1; // 设置反转//没什么用
        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;//从陀螺仪的取值
            else
                pid_measure = motor->measure.position; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃//为什么用总角度，因为有多圈，基本不用
            if(motor->flag == 2)
        {
            if( motor->motor_mode == GIMBAL_MOTOR_ENCONDE)
            {
            pid_ref1 = PIDCalculate(&motor->relative_angle_PID, pid_measure, pid_ref2);//根本用不了这个
            }
            else if(motor->motor_mode == GIMBAL_MOTOR_GYRO || motor->motor_mode == GIMBAL_MOTOR_AUTO || motor->motor_mode == GIMBAL_MOTOR_ROTATE || motor->motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO)
            pid_ref1 = PIDCalculate(&motor->absoulte_angle_PID, pid_measure, pid_ref2);//大yaw基本用陀螺仪控制)
        }
           if(motor->flag == 3)
        {
            if( motor->motor_mode == GIMBAL_MOTOR_ENCONDE)
            {
            pid_ref1 = DMPIDCalculate(&motor->relative_angle_PID, pid_measure, pid_ref2);//之后的pitch电机应该会用这个
            }
            else if(motor->motor_mode == GIMBAL_MOTOR_GYRO || motor->motor_mode == GIMBAL_MOTOR_AUTO || motor->motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO)
            pid_ref1 = DMPIDCalculate(&motor->absoulte_angle_PID, pid_measure, pid_ref2);//大yaw基本用陀螺仪控制
            else if(motor->motor_mode == GIMBAL_MOTOR_ROTATE)
            pid_ref1 = DMPIDCalculate(&motor->relative_angle_PID, pid_measure, pid_ref2);//这个用来特殊控制大yaw
        }
           
        }
        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref1 += *motor->speed_feedforward_ptr;
            
            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure1 = *motor->other_speed_feedback_ptr;//这里用陀螺仪反馈的值也可以
            /////////////以上的两个都进不去/////////////////////////
            else // MOTOR_FEED
                pid_measure1 = measure->velocity;
                // rc_deadband_limit(pid_measure1,pid_measure,0.2f);//死区处理
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure1, pid_ref1);

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        }

        if(motor->stop_flag == MOTOR_STOP)
        {
            pid_ref = 0;
        }

        set = pid_ref;
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        // motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;
        if(motor->stop_flag == MOTOR_STOP)
         motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

         CANTransmit(motor->motor_can_instace, 1);
    }
    
}

void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dm_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

//对电机模式的传承
void DMMotorinhert(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DMMotorInstance* Instance)
{
    if(Instance->flag == 3)
     {
        switch(gimbal_cmd_recv->big_yaw_motor_mode)
     {
        case GIMBAL_MOTOR_RAW:
            Instance->motor_mode = GIMBAL_MOTOR_RAW;
            break;
        case GIMBAL_MOTOR_ENCONDE:
            Instance->motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;
        case GIMBAL_MOTOR_GYRO:
            Instance->motor_mode = GIMBAL_MOTOR_GYRO;
            break;
        case GIMBAL_MOTOR_AUTO:
            Instance->motor_mode = GIMBAL_MOTOR_AUTO;
            break;
        case GIMBAL_MOTOR_ROTATE:
            Instance->motor_mode = GIMBAL_MOTOR_ROTATE;
            break;
        case GIMBAL_MOTOR_AUTO_XUNLUO:
            Instance->motor_mode = GIMBAL_MOTOR_AUTO_XUNLUO;
            break;
     }
    }
    if(Instance->flag == 2)
    {
     switch(gimbal_cmd_recv->pitch_motor_mode)
     {
        case GIMBAL_MOTOR_RAW:
            Instance->motor_mode = GIMBAL_MOTOR_RAW;
            break;
        case GIMBAL_MOTOR_ENCONDE:
            Instance->motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;
        case GIMBAL_MOTOR_GYRO:
            Instance->motor_mode = GIMBAL_MOTOR_GYRO;
            break;
        case GIMBAL_MOTOR_AUTO:
            Instance->motor_mode = GIMBAL_MOTOR_AUTO;
            break;
        case GIMBAL_MOTOR_ROTATE:
            Instance->motor_mode = GIMBAL_MOTOR_ROTATE;
            break;
        case GIMBAL_MOTOR_AUTO_XUNLUO:
            Instance->motor_mode = GIMBAL_MOTOR_AUTO_XUNLUO;
            break;
     }
    }
}

//模式转换函数，这个要好好测一下
void DMModeChangeControlTransmit(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DMMotorInstance* Instance,Gimbal_Data_s* gimbal_posture_data)
{
   Gimbal_Ctrl_Cmd_s* gimbal_cmd = gimbal_cmd_recv;
   Big_Yaw_Data_s* big_yaw_posture_data = &gimbal_posture_data->Big_Yaw_Data;
   Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;
   if(Instance->flag == 3)
{
               if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_RAW) && (gimbal_cmd->last_big_yaw_motor_mode != GIMBAL_MOTOR_RAW))
              {
                Instance->pid_ref = 0;
              }
               if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_GYRO) && (gimbal_cmd->last_big_yaw_motor_mode != GIMBAL_MOTOR_GYRO))
              {
                Instance->pid_ref = big_yaw_posture_data->big_yaw_absoulte_angle;
              }
              if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_ENCONDE) && (gimbal_cmd->last_big_yaw_motor_mode != GIMBAL_MOTOR_ENCONDE))
              {
                Instance->pid_ref = big_yaw_posture_data->big_yaw_relative_angle;
              }
              if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_ROTATE) && (gimbal_cmd->last_big_yaw_motor_mode != GIMBAL_MOTOR_ROTATE))
              {
                Instance->pid_ref = big_yaw_posture_data->big_yaw_absoulte_angle;
              }
              if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_AUTO) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_AUTO))
              {
                Instance->pid_ref = big_yaw_posture_data->big_yaw_absoulte_angle;
              }
              if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_AUTO_XUNLUO))
              {
                Instance->pid_ref = big_yaw_posture_data->big_yaw_absoulte_angle;
              }

}
    if(Instance->flag == 2)
{
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_RAW) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_RAW))
              {
                Instance->pid_ref = 0;
              }
               if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_GYRO) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_GYRO))
              {
                Instance->pid_ref = pitch_posture_data->pitch_absoulte_angle;
              }
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_ENCONDE) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_ENCONDE))
              {
                Instance->pid_ref = pitch_posture_data->pitch_relative_angle;
              }
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_ROTATE) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_ROTATE))
              {
                Instance->pid_ref = pitch_posture_data->pitch_absoulte_angle;
              }
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_AUTO) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_AUTO))
              {
                Instance->pid_ref = pitch_posture_data->pitch_absoulte_angle;
              }
              if((gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_AUTO_XUNLUO))
              {
                Instance->pid_ref = pitch_posture_data->pitch_absoulte_angle;
              }
}

}
   

void DMMotorRefVerify(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv, DMMotorInstance* gimbal_motor, Gimbal_Data_s* gimbal_data, dm_imu_data_t* dm_imu_data)
{
    Gimbal_Ctrl_Cmd_s* gimbal_cmd = gimbal_cmd_recv; //cmd层传过来的数据
    // DM_Motor_Measure_s* gimbal_motor_measure = &gimbal_motor->measure; //电机反馈至
    if(gimbal_motor->flag == 3) //说明这个是大yaw的4310电机
    { 
    if (gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制，小陀螺，无限位
        DMGimbalnNoLimitRef(gimbal_cmd,gimbal_motor,gimbal_data);
    }
    if (gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        DMGimbalAutoRefLimit(gimbal_cmd,gimbal_motor,dm_imu_data); //这个不会被用到
    }
    if(gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        DMGimbalnNoLimitRef(gimbal_cmd,gimbal_motor,gimbal_data);
    }
    if(gimbal_cmd->big_yaw_motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO)
    {
        DMGimbalnXunLuoNoLimitRef(gimbal_motor,gimbal_data);
    }

    }

    if(gimbal_motor->flag == 2)//说明这个是pitch4310电机
    {
    if (gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制，小陀螺，无限位
        DMGimbalAutoRefLimit(gimbal_cmd,gimbal_motor,dm_imu_data);
    }
    if (gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        DMGimbalAutoRefLimit(gimbal_cmd,gimbal_motor,dm_imu_data);//这个根本不用
    }
    if(gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        DMGimbalAutoRefLimit(gimbal_cmd,gimbal_motor,dm_imu_data);
    }
    if(gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_AUTO_XUNLUO)
    {
        DMGimbalAutoXunLuoRefLimit(gimbal_cmd,gimbal_motor,dm_imu_data);
    }

    }
}

void DMGet4310MotorData(Gimbal_Data_s* gimbal_posture_data,DMMotorInstance *motor,attitude_t* gimbal_IMU_data ,dm_imu_data_t* dm_imu_data)
{
 if(motor->flag == 3) //大yaw电机
{
   DM_Motor_Measure_s* motor_measure = &motor->measure;
   Big_Yaw_Data_s* posture_data = &gimbal_posture_data->Big_Yaw_Data;
   
   posture_data->big_yaw_relative_angle = motor_measure->angle_single_round; //编码值反馈相对角度
   posture_data->big_yaw_absoulte_angle = gimbal_IMU_data->Yaw;  //绝对角度直接用陀螺仪的
}
 if(motor->flag == 2) //pitch电机
{
   DM_Motor_Measure_s* motor_measure = &motor->measure;
   Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;
   
   pitch_posture_data->pitch_relative_angle = (dm_imu_data->oula_data.roll - PITCH_MID_POS*angle_to_radian); //编码值反馈相对角度
   pitch_posture_data->pitch_absoulte_angle = (dm_imu_data->oula_data.roll); //绝对角度直接用陀螺仪的
}
}


void DMGimbalAutoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor, dm_imu_data_t* dm_imu_data)  //有限位
{ 
    fp32 bias_angle = 0.0f;
    fp32 add = 0.0f;
    static int as = 0;
    if(gimbal_motor->pid_ref == 0 && as == 0)
    {
        gimbal_motor->pid_ref =  dm_imu_data->oula_data.roll;
        as++;
    }
    add = gimbal_cmd->pitch;
    bias_angle = (gimbal_motor->pid_ref - dm_imu_data->oula_data.roll);//这边原函数减的是绝对角度，我认为是相对角度，之后调试再看
    if (dm_imu_data->oula_data.roll + bias_angle + add > gimbal_motor->motor_limit_left)
    {
        //如果是往最大机械角度控制方向左
        if (add > 0.0f)
        {
            add = gimbal_motor->motor_limit_left - dm_imu_data->oula_data.roll - bias_angle;
        }
    }
    else if (dm_imu_data->oula_data.roll + bias_angle + add  < gimbal_motor->motor_limit_right)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->motor_limit_right - dm_imu_data->oula_data.roll - bias_angle;
        }
    }
    gimbal_motor->pid_ref = gimbal_motor->pid_ref + add;
    
}

void DMGimbalAutoXunLuoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,DMMotorInstance* gimbal_motor, dm_imu_data_t* dm_imu_data)  //有限位
{
    fp32 bias_angle = 0.0f;
    static fp32 add = 0.0f;
    static int as = 0;
    if(gimbal_motor->pid_ref == 0 && as == 0)
    {
        gimbal_motor->pid_ref =  dm_imu_data->oula_data.roll;
        as++;
    }
    if(gimbal_motor->pid_ref >= PITCH_MID_POS)
    {
        add = PITCH_4310_EVERY_RAD_ADD_UP;
    }
    else 
    {
        add = PITCH_4310_EVERY_RAD_ADD_DOWN;
    }
    bias_angle = (gimbal_motor->pid_ref - dm_imu_data->oula_data.roll);
    if (dm_imu_data->oula_data.roll + bias_angle + add > gimbal_motor->motor_limit_left) //向上达到最大
    {
       add = PITCH_4310_EVERY_RAD_ADD_DOWN;
    }
    else if (dm_imu_data->oula_data.roll + bias_angle + add  < gimbal_motor->motor_limit_right) //向下达到最低
    {
       add = PITCH_4310_EVERY_RAD_ADD_UP;
    }
    gimbal_motor->pid_ref = gimbal_motor->pid_ref + add;
}
