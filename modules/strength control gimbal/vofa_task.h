#ifndef VOFA_TASK_H
#define VOFA_TASK_H

#include  "main.h"
#include "force_gimbal_core.h"

// === 模式选择枚举 ===
typedef enum
{
    VOFA_MODE_TEST = 0,   // 【测试模式】：专用于系统辨识录数据 (对应 Python 脚本)
    VOFA_MODE_VERIFY      // 【验证模式】：专用于调 PID 看波形 (看目标值vs实际值)
} Vofa_Mode_e;

/**
 * @brief VOFA+ 数据发送函数 (封装了通道逻辑)
 * @param mode   模式选择 (VOFA_MODE_TEST / VOFA_MODE_VERIFY)
 * @param target 目标角度 (用于验证模式)
 * @param pos    当前角度 (rad)
 * @param vel    当前速度 (rad/s)
 * @param torque 当前力矩/电流 (Nm 或 Raw)
 */
// 新增：直接传轴指针的“自动挡”函数
void Vofa_Just_Sample(Vofa_Mode_e mode, ForceAxis_t *axis);
void Vofa_Gimbal_Update(Vofa_Mode_e mode, float target, float pos, float vel, float torque);

#endif