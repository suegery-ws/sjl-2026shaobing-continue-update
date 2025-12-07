#include "dji_motor.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "user_lib.h"


static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算4
static int8_t block_time = 0; 
static int8_t reverse_time = 0;                            // 已注册的电机实例数量
/////////////////////////////////////////这部分函数可以再dji和dm通用，只对add值做出更改//////////////////////////////////////////////////////////////////////////////

 void DJIGimbalAutoRefLimit(Gimbal_Ctrl_Cmd_s* gimbal_cmd,Motor_Controller_s* gimbal_motor_control,Gimbal_Data_s* gimbal_posture_data,DJI_Motor_Measure_s* gimbal_motor_measure)  //有限位
{
    if(gimbal_motor_control->flag == 1)
    {
    fp32 bias_angle = 0.0f;
    fp32 add = 0.0f;
    // if (gimbal_motor == NULL)
    // {
    //     return;
    // }
    //now angle error
    //当前控制误差角度
    add = gimbal_cmd->yaw;
    bias_angle = rad_format(gimbal_motor_control->pid_ref - gimbal_motor_measure->total_angle);//这边原函数减的是绝对角度，我认为是相对角度，之后调试再看
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor_measure->total_angle + bias_angle + add > gimbal_motor_control->motor_limit_left)
    {
        //如果是往最大机械角度控制方向左
        if (add > 0.0f)
        {
            //calculate max add_angle
            //计算出一个最大的添加角度，
            add = gimbal_motor_control->motor_limit_left - gimbal_motor_measure->total_angle - bias_angle;
        }
    }
    else if (gimbal_motor_measure->total_angle + bias_angle + add  < gimbal_motor_control->motor_limit_right)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor_control->motor_limit_right - gimbal_motor_measure->total_angle - bias_angle;
        }
    }
    
    gimbal_motor_control->pid_ref = gimbal_motor_control->pid_ref + add;
    }
    if(gimbal_motor_control->flag == 2)
    {
    fp32 bias_angle = 0.0f;
    fp32 add = 0.0f;
    // if (gimbal_motor == NULL)
    // {
    //     return;
    // }
    //now angle error
    //当前控制误差角度
    add = gimbal_cmd->pitch;
    bias_angle = rad_format(gimbal_motor_control->pid_ref - gimbal_motor_measure->total_angle);//这边原函数减的是绝对角度，我认为是相对角度，之后调试再看
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor_measure->total_angle + bias_angle + add > gimbal_motor_control->motor_limit_left)
    {
        //如果是往最大机械角度控制方向左
        if (add > 0.0f)
        {
            //calculate max add_angle
            //计算出一个最大的添加角度，
            add = gimbal_motor_control->motor_limit_left - gimbal_motor_measure->total_angle - bias_angle;
        }
    }
    else if (gimbal_motor_measure->total_angle + bias_angle + add  < gimbal_motor_control->motor_limit_right)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor_control->motor_limit_right - gimbal_motor_measure->total_angle - bias_angle;
        }
    }
    
    gimbal_motor_control->pid_ref = gimbal_motor_control->pid_ref + add;
    }
}

 void DJIGimbalRefLimit(DJI_Motor_Measure_s* gimbal_motor_measure,Gimbal_Ctrl_Cmd_s* gimbal_cmd,Motor_Controller_s* gimbal_motor_control,Gimbal_Data_s* gimbal_posture_data)//有限幅的函数
{
    if(gimbal_motor_control->flag == 1)
    {
    fp32 add = gimbal_cmd->yaw;
    if(gimbal_motor_control->pid_ref == 0)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_measure->total_angle;
    }
	gimbal_motor_control->pid_ref += add;
    //是否超过最大 最小值，最大最小在校准里得到
    if (gimbal_motor_control->pid_ref > gimbal_motor_control->motor_limit_left)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_control->motor_limit_left;
    }
    else if (gimbal_motor_control->pid_ref < gimbal_motor_control->motor_limit_right)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_control->motor_limit_right;
    }
    }
    if(gimbal_motor_control->flag == 2)
    {
    fp32 add = gimbal_cmd->pitch;
    if(gimbal_motor_control->pid_ref == 0)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_measure->total_angle;
    }
	gimbal_motor_control->pid_ref += add;
    //是否超过最大 最小值，最大最小在校准里得到
    if (gimbal_motor_control->pid_ref > gimbal_motor_control->motor_limit_left)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_control->motor_limit_left;
    }
    else if (gimbal_motor_control->pid_ref < gimbal_motor_control->motor_limit_right)
    {
        gimbal_motor_control->pid_ref = gimbal_motor_control->motor_limit_right;
    }
    }

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//大疆专属角度转换函数
fp32 DJI_ecd_to_angle_change(int32_t ecd, int32_t offset_ecd)
{
	if(ecd>=0)
	{
	int32_t relative_ecd = ecd - offset_ecd;
	  if (relative_ecd > HALF_ECD_RANGE_DJI)
    {
        relative_ecd -= ECD_RANGE_DJI;
    }
     if (relative_ecd < -HALF_ECD_RANGE_DJI)
    {
        relative_ecd += ECD_RANGE_DJI;
    }
    return relative_ecd *ECD_RAD_COEF_DJI ;
	}
}

fp32 DJI_relative_rad_calculate(float rad,float offset_rad)
{
  
	
	float relative_rad = rad - offset_rad;
	  if (relative_rad > HALF_ECD_RANGE_DJI)
    {
        relative_rad -= RAD_RANGE_DJI;
    }
     if (relative_rad < -HALF_ECD_RANGE_DJI)
    {
        relative_rad += RAD_RANGE_DJI;
    }
    return relative_rad;
}

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // if (measure->ecd - measure->last_ecd > 4096)
    //     measure->total_round--;
    // else if (measure->ecd - measure->last_ecd < -4096)
    //     measure->total_round++;
    // measure->total_angle = (measure->total_round * 6.28 + measure->angle_single_round);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->last_ecd = measure->ecd;
    measure->angle_single_round = ECD_RAD_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_RAD_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = (measure->total_round * 6.28 + measure->angle_single_round);
    //以下是对2006的特殊处理
    if(motor->motor_type == M2006)
    {
        if(measure->total_round >= 36 || measure->total_round <= -36) 
    {
       measure->total_round = 0;
    }
        measure->total_angle = M2006_round_to_rad * measure->total_round;  
    }
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.absoulte_angle_PID, &config->controller_param_init_config.absoulte_angle_PID);
    PIDInit(&instance->motor_controller.relative_angle_PID, &config->controller_param_init_config.relative_angle_PID);
    // 反馈指针初始化，前馈指针初始化
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)
    //标志位赋值
    instance->motor_controller.flag = config->controller_param_init_config.flag;
    //限幅设置
    instance->motor_controller.motor_limit_left = config->controller_param_init_config.motor_limit_left;
    instance->motor_controller.motor_limit_right = config->controller_param_init_config.motor_limit_right;
    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)//这个id就是电机实例的地址，想要可以随时转换
    instance->motor_can_instance = CANRegister(&config->can_init_config);//这三行完整的解释了can设备实例的初始化函数，can设备实例的注册函数，电机实例的初始化函数，电机实例的注册函数是怎么联系在一起的

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转  这个只针对拨弹轮电机有意义

        if (motor->motor_type == M2006 && motor_controller->shoot_mode == LOAD_1_BULLET)
        {
            if(measure->total_angle == 0 && pid_ref >6.6f)
            {
                pid_ref = 0.785;
            }
            else if(pid_ref > 6.28)
            pid_ref = pid_ref - 6.28f;
        }
        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃//为什么用总角度，因为有多圈//这边没看见角度环的前馈，之后加
            // 更新pid_ref进入下一个环
            if(motor->motor_type == M2006)
            {
            if(motor_controller->motor_mode == GIMBAL_MOTOR_ENCONDE)
            pid_ref = DJI2006PIDCalculate(&motor_controller->relative_angle_PID, pid_measure, pid_ref);
            else if(motor_controller->motor_mode == GIMBAL_MOTOR_GYRO)
            pid_ref = DJI2006PIDCalculate(&motor_controller->absoulte_angle_PID, pid_measure, pid_ref);
            }
            else
            if(motor_controller->motor_mode == GIMBAL_MOTOR_ENCONDE)
            pid_ref = PIDCalculate(&motor_controller->relative_angle_PID, pid_measure, pid_ref);
            else if(motor_controller->motor_mode == GIMBAL_MOTOR_GYRO)
            pid_ref = PIDCalculate(&motor_controller->absoulte_angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;

        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        if (sender_enable_flag[i])
        {
          CANTransmit(&sender_assignment[i], 1);
        }
    }
}
//这个函数的目标是对cmd层传过来的数据进行校验，根据不同模式来更改，需要cmd层的命令指针，要电机数据的预设值和预设模式，最后的结果是对预设数值进行更改
//个人认为限幅问题和自不自动没半毛钱关系，完全就是怎么好限怎么来
void DJIMotorRefVerify(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv, DJIMotorInstance* gimbal_motor, Gimbal_Data_s* gimbal_posture_data)
{
    Gimbal_Ctrl_Cmd_s* gimbal_cmd = gimbal_cmd_recv; //cmd层传过来的数据//这个是欧美打法
    Motor_Controller_s* motor_controller = &gimbal_motor->motor_controller;
    Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;
    Yaw_Data_s* yaw_posture_data = &gimbal_posture_data->Yaw_Data;
    DJI_Motor_Measure_s* motor_measure = &gimbal_motor->measure;
    //////////////////////////////////////////////////////////////////////////////////////
    if(motor_controller->flag == 1)
    {
     if ( gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制，小陀螺
        DJIGimbalAutoRefLimit(gimbal_cmd,motor_controller,gimbal_posture_data, motor_measure);
    }
     if (gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制，跟随云台
        DJIGimbalRefLimit(motor_measure,gimbal_cmd,motor_controller,gimbal_posture_data);
    }    
	 if (gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        DJIGimbalAutoRefLimit(gimbal_cmd,motor_controller,gimbal_posture_data, motor_measure);
    }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if(motor_controller->flag == 2)
    {
        
    if (gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //auto模式下，陀螺仪角度控制
        DJIGimbalAutoRefLimit(gimbal_cmd,motor_controller,gimbal_posture_data, motor_measure);
    }
    else if (gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        DJIGimbalRefLimit(motor_measure,gimbal_cmd,motor_controller,gimbal_posture_data);
    }
     if (gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        DJIGimbalAutoRefLimit(gimbal_cmd,motor_controller,gimbal_posture_data, motor_measure);
    }

    }

}

//能用，控制模式的传承
void DJIMotorinhert(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DJIMotorInstance* Instance)
{
    if(Instance->motor_controller.flag == 1)//yaw
    { switch(gimbal_cmd_recv->yaw_motor_mode)
     {
        case GIMBAL_MOTOR_RAW:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_RAW;
            break;
        case GIMBAL_MOTOR_ENCONDE:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;
        case GIMBAL_MOTOR_GYRO:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_GYRO;
            break;
        case GIMBAL_MOTOR_AUTO:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_AUTO;
            break;
     }
    }
    if(Instance->motor_controller.flag == 2)//pitch
     {switch(gimbal_cmd_recv->pitch_motor_mode)
     {
        case GIMBAL_MOTOR_RAW:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_RAW;
            break;
        case GIMBAL_MOTOR_ENCONDE:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_ENCONDE;
            break;
        case GIMBAL_MOTOR_GYRO:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_GYRO;
            break;
        case GIMBAL_MOTOR_AUTO:
            Instance->motor_controller.motor_mode = GIMBAL_MOTOR_AUTO;
            break;
     }
     }
}

//模式转换的数据保存，这个放置的位置存疑 //这个函数要先写反馈数据结构体然后再写它
void DJIModeChangeControlTransmit(Gimbal_Ctrl_Cmd_s* gimbal_cmd_recv,DJIMotorInstance* Instance,Gimbal_Data_s* gimbal_posture_data)
{
   Motor_Controller_s* motor_controller = &(Instance->motor_controller);
   DJI_Motor_Measure_s* motor_measure = &(Instance->measure);
   Gimbal_Ctrl_Cmd_s* gimbal_cmd = gimbal_cmd_recv;
   Yaw_Data_s* yaw_posture_data = &gimbal_posture_data->Yaw_Data;
   Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;

    if(motor_controller->flag == 1)//yaw//这里面可能会用到电机的参数设定值
    {
               if((gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_RAW) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_RAW))
             {
                motor_controller->pid_ref = yaw_posture_data->yaw_relative_angle; //raw模式下直接输出0
             }
               if((gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_GYRO) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_GYRO))
              {
                motor_controller->pid_ref = yaw_posture_data->yaw_absoulte_angle; //这个其实感觉用不到
              }
              if((gimbal_cmd->yaw_motor_mode == GIMBAL_MOTOR_ENCONDE) && (gimbal_cmd->last_yaw_motor_mode != GIMBAL_MOTOR_ENCONDE))
              {
                motor_controller->pid_ref = motor_measure->total_angle; //
              }
              //这里以后要加一个自瞄模式的处理函数
    }
    if(motor_controller->flag == 2) //这个加了pitch4310之后其实也用不到了
    {
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_RAW) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_RAW))
             {
                motor_controller->pid_ref = yaw_posture_data->yaw_relative_angle; //raw模式下直接输出0
             }
               if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_GYRO) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_GYRO))
              {
                motor_controller->pid_ref = pitch_posture_data->pitch_absoulte_angle; //基本不用
              }
              if((gimbal_cmd->pitch_motor_mode == GIMBAL_MOTOR_ENCONDE) && (gimbal_cmd->last_pitch_motor_mode != GIMBAL_MOTOR_ENCONDE))
              {
                motor_controller->pid_ref = motor_measure->total_angle; //
              } 
              //这里以后要加一个自瞄模式的处理函数
    }
}
//这部份解算代码之后都会移到decode函数中去
void DJIGetYawMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* yaw_motor,attitude_t* gimbal_IMU_data)
{
   DJI_Motor_Measure_s* yaw_motor_measure = &yaw_motor->measure;
   Yaw_Data_s* yaw_posture_data = &gimbal_posture_data->Yaw_Data;
   Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;
// yaw_posture_data->yaw_motor_gyro = MOTOR_RPM_TO_GYRO*yaw_motor_measure->speed_aps;
   //yaw_posture_data->yaw_motor_gyro = arm_cos_f32(pitch_posture_data->pitch_relative_angle) * (gimbal_IMU_data->Gyro[2])- arm_sin_f32(pitch_posture_data->pitch_relative_angle) * (gimbal_IMU_data->Gyro[0]);//拿陀螺仪数据来解算的

   yaw_posture_data->yaw_relative_angle = GYRO2GIMBAL_DIR_YAW*DJI_relative_rad_calculate(yaw_motor->measure.total_angle,YAW_6020_OFF_SET_RAD);//这个相对角度相对的是大yaw,精度可能不够
   yaw_posture_data->yaw_absoulte_angle = yaw_posture_data->yaw_relative_angle+(fp32)(gimbal_IMU_data->Yaw);
}//验证过，可以用

void DJIGetPitchMotorData(Gimbal_Data_s* gimbal_posture_data,DJIMotorInstance* pitch_motor,attitude_t* gimbal_IMU_data)
{
   DJI_Motor_Measure_s* pitch_motor_measure = &pitch_motor->measure;
   Pitch_Data_s* pitch_posture_data = &gimbal_posture_data->Pitch_Data;
//    pitch_posture_data->pitch_motor_gyro = MOTOR_RPM_TO_GYRO*pitch_motor_measure->speed_aps;
   //pitch_posture_data->pitch_motor_gyro = gimbal_IMU_data->Gyro[1]; 陀螺仪数据解算
   pitch_posture_data->pitch_relative_angle = GYRO2GIMBAL_DIR_PITCH*DJI_relative_rad_calculate(pitch_motor->measure.total_angle,PITCH_6020_OFF_SET_RAD);;//这个相对角度相对的是大yaw
   pitch_posture_data->pitch_absoulte_angle = pitch_posture_data->pitch_relative_angle+(fp32)(gimbal_IMU_data->Pitch);
}//验证过，可以用

void DJIGetChassisMotorData(Chassis_Data_s* chassis_data, Chassis_Ctrl_Cmd_s* chassis_cmd, attitude_t* Chassis_IMU_data, DJIMotorInstance* motor_lf, DJIMotorInstance* motor_rf ,DJIMotorInstance* motor_lb, DJIMotorInstance* motor_rb)
{
   Chassis_motor_accel_data_s* chassis_motor_accel_data = &chassis_data->chassis_motor_accel_data;
//    Chassis_motor_speed_data_s* chassis_motor_speed_data = &chassis_data->chassis_motor_speed_data;
   Chassis_posture_data_s* chassis_posture_data = &chassis_data->chassis_posture_data;
   Chassis_speed_data_s* chassis_speed = &chassis_data->chassis_speed;
   DJI_Motor_Measure_s* motor_lf_measure = &motor_lf->measure;
   DJI_Motor_Measure_s* motor_rf_measure = &motor_rf->measure;
   DJI_Motor_Measure_s* motor_lb_measure = &motor_lb->measure;
   DJI_Motor_Measure_s* motor_rb_measure = &motor_rb->measure;
   PIDInstance* motor_lf_pid = &motor_lf->motor_controller.speed_PID;
   PIDInstance* motor_rf_pid = &motor_rf->motor_controller.speed_PID;
   PIDInstance* motor_lb_pid = &motor_lb->motor_controller.speed_PID;
   PIDInstance* motor_rb_pid = &motor_rb->motor_controller.speed_PID;
   ////////////////////////////////////底盘电机速度和加速度////////////////////////////////////////////////////////电机速度湖大比我分析的好，加速度也要改但暂时用不到
   //lf
//    chassis_motor_speed_data->motor_lf_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * motor_lf_measure->speed_aps;motor_rf_measure->speed_aps
   chassis_motor_accel_data->motor_lf_accel = (motor_lf_pid->Err-motor_lf_pid->Last_Err) * CHASSIS_CONTROL_FREQUENCE;
   //rf
//    chassis_motor_speed_data->motor_rf_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * motor_rf_measure->speed_aps;
   chassis_motor_accel_data->motor_rf_accel = (motor_rf_pid->Err-motor_rf_pid->Last_Err) * CHASSIS_CONTROL_FREQUENCE;
   //lb
//    chassis_motor_speed_data->motor_lb_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * motor_lb_measure->speed_aps;
   chassis_motor_accel_data->motor_lb_aceel = (motor_lb_pid->Err-motor_rf_pid->Last_Err) * CHASSIS_CONTROL_FREQUENCE;
   //rb
//    chassis_motor_speed_data->motor_rb_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * motor_rb_measure->speed_aps;
   chassis_motor_accel_data->motor_rb_accel = (motor_rb_pid->Err-motor_rb_pid->Last_Err) * CHASSIS_CONTROL_FREQUENCE;
   ///////////////////////////////////底盘速度////////////////////////////////////////////////////////////
   chassis_speed->vx = - motor_lf_measure->speed_aps * my_sin((angle_to_rad_45+chassis_cmd->offset_angle)) - motor_lb_measure->speed_aps * my_cos((angle_to_rad_45+chassis_cmd->offset_angle)) 
	+motor_rb_measure->speed_aps*my_sin((angle_to_rad_45+chassis_cmd->offset_angle)) +motor_rf_measure->speed_aps*my_cos((angle_to_rad_45+chassis_cmd->offset_angle)) ; 
		
	chassis_speed->vy =  motor_lf_measure->speed_aps * my_cos((angle_to_rad_45+chassis_cmd->offset_angle)) - motor_lb_measure->speed_aps * my_sin((angle_to_rad_45+chassis_cmd->offset_angle))
	-motor_rb_measure->speed_aps*my_cos((angle_to_rad_45+chassis_cmd->offset_angle)) +motor_rf_measure->speed_aps*my_sin((angle_to_rad_45+chassis_cmd->offset_angle));
		
	chassis_speed->wz = (-motor_rf_measure->speed_aps - motor_lf_measure->speed_aps - motor_lb_measure->speed_aps - motor_rb_measure->speed_aps 
   - motor_rf_measure->speed_aps) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
   ///////////////////////////////////底盘姿态////////////////////////////////////////////////////////////
   chassis_posture_data->car_yaw_posture = rad_format(Chassis_IMU_data->Yaw - chassis_cmd->offset_angle); 
	chassis_posture_data->car_pitch_posture = rad_format(Chassis_IMU_data->Pitch);
   chassis_posture_data->car_roll_posture = rad_format(Chassis_IMU_data->Roll);//暂时不清楚它读到的是什么制的东西
}

//底盘控制模式转换到电机控制模式
void DJI2006MotorInhert(Shoot_Ctrl_Cmd_s* shoot_cmd_recv,DJIMotorInstance* Instance)
{
    switch(shoot_cmd_recv->load_mode)
     {
        case LOAD_STOP:
            Instance->motor_controller.shoot_mode = LOAD_STOP;
            break;
        case LOAD_1_BULLET:
            Instance->motor_controller.shoot_mode = LOAD_1_BULLET;
            break;
        case LOAD_BURSTFIRE:
            Instance->motor_controller.shoot_mode = LOAD_BURSTFIRE;
            break;
        case LOAD_REVERSE:
            Instance->motor_controller.shoot_mode = LOAD_REVERSE;
            break;
     }
}


//拨弹轮反转控制
void trigger_motor_turn_back(DJIMotorInstance* motor)
{
    if( block_time < BLOCK_TIME)
    {
       motor->motor_controller.pid_ref = motor->motor_controller.pid_ref;//开启反转
    }
    else//如果卡弹时间>700，拨弹开启反转
    {
        motor->motor_controller.pid_ref = -motor->motor_controller.pid_ref;//开启反转
    }

    if(fabs(motor->measure.speed_aps) < BLOCK_TRIGGER_SPEED && block_time < BLOCK_TIME)//根据拨弹轮速度判断是否卡弹
    {
        block_time++;
        reverse_time = 0;//恢复时间
    }
    else if (block_time >= BLOCK_TIME && reverse_time < REVERSE_TIME)
    {
        reverse_time++;
    }
    else
    {
        block_time = 0;
    }
}