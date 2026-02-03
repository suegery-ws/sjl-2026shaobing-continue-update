#ifndef FORCE_BRIDGE_H
#define FORCE_BRIDGE_H

#include "force_gimbal_core.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"

// ================= 混搭配置区 =================
// 1: 达妙(DM), 0: 大疆(DJI)
#define PITCH_MOTOR_TYPE  1  // 例如：Pitch 用达妙
#define YAW_MOTOR_TYPE    0  // 例如：Yaw 用大疆

// ================= 物理参数区 =================
// Pitch (达妙示例参数)
#define PITCH_J 0.0f 
#define PITCH_B 0.0f
#define PITCH_C 0.0f
#define PITCH_G_COS 0.0f
// Yaw (大疆示例参数)
#define YAW_J   0.0f
#define YAW_B   0.0f
#define YAW_C   0.0f
// ============================================

// 初始化“指挥官”，传入两个电机的指针（用 void* 实现泛型）
void ForceBridge_Init(void *pitch_motor_handle, void *yaw_motor_handle);

// 核心循环，放入 GimbalTask
void ForceBridge_Update(attitude_t *imu, float target_pitch, float target_yaw, uint8_t is_enabled);

// 新增：发送波形函数 (axis_id: 0=Pitch, 1=Yaw)
void ForceBridge_View(uint8_t axis_id);


#endif