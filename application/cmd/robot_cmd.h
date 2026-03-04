#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
//////////////////////////////////////////////////////////////////////
#include "struct_typedef.h"

#ifndef __packed
#define __packed __attribute__((packed))
#endif
/////////////////////////////////////////////////////////////////////
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
/////////////////////////////////////////////////////////////////////
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
//////////////////////////////////////////////////////////////////////

typedef struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} __packed first_order_filter_type_t;

void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//////////////弧度制直达目标值/////////////////////////////////
double my_cos(double rad);
double my_sin(double rad);
double mx_sin(double rad);

////////////////////////////////////
#define GIMBAL_RC_DEADBAND  10
#define CHASSIS_RC_DEADLINE 10

///////////////////////////////////
#define YAW_RC_SEN    -0.000008f //0.002//灵敏度
#define PITCH_RC_SEN  0.000009f //0.005//灵敏度
#define BIG_YAW_RC_SEN 0.00002f
/////////////////////////////////
#define CHASSIS_VY_RC_SEN 0.006f
#define CHASSIS_VX_RC_SEN 0.005f

#define TONGJI_PITCH_AUTO_SEN 0.01f
#define TONGJI_YAW_AUTO_SEN 0.045f


/////////////////////////////////
#define PITCH_AUTO_SEN    0.015f                            //
#define YAW_AUTO_SEN  0.015f                                //
#define ONE_PI   (3.14159265)
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
#endif // !ROBOT_CMD_H