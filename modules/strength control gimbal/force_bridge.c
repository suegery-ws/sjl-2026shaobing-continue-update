#include "force_bridge.h"
#include "vofa_task.h"

static ForceAxis_t pitch_axis;
static ForceAxis_t yaw_axis;

static void *pitch_handle;
static void *yaw_handle;

static float pitch_dji_ff = 0.0f;
static float yaw_dji_ff = 0.0f;

void ForceBridge_Init(void *pitch, void *yaw) {
    pitch_handle = pitch;
    yaw_handle = yaw;

    // 1. 初始化 Pitch (修正宏名称)
    float p_scale = (PITCH_MOTOR_TYPE == 1) ? 1.0f : 25000.0f;
    ForceMotorType_e p_type = (PITCH_MOTOR_TYPE == 1) ? FORCE_MOTOR_TYPE_DM_MIT : FORCE_MOTOR_TYPE_GM6020_VOLTAGE;
    
    ForceAxis_Init(&pitch_axis, p_type, p_scale, PITCH_J, PITCH_B, PITCH_C, PITCH_G_COS, 0);

    // 2. 初始化 Yaw (修正宏名称)
    float y_scale = (YAW_MOTOR_TYPE == 1) ? 1.0f : 25000.0f;
    ForceMotorType_e y_type = (YAW_MOTOR_TYPE == 1) ? FORCE_MOTOR_TYPE_DM_MIT : FORCE_MOTOR_TYPE_GM6020_VOLTAGE;
    
    ForceAxis_Init(&yaw_axis, y_type, y_scale, YAW_J, YAW_B, YAW_C,PITCH_G_COS, 0);

    // 设置 PID (请根据实际情况调整)
    ForceAxis_SetPID(&pitch_axis, 32.0f, 0.0f, 0.5f, 0.00006f, 0, 0);
    ForceAxis_SetPID(&yaw_axis,   8.0f, 0.0f, 0.0f, 0.012f, 0.8f, 0);

    // 3. 挂载大疆前馈
    if (PITCH_MOTOR_TYPE == 0) {
        DJIMotorInstance* m = (DJIMotorInstance*)pitch_handle;
        m->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
        m->motor_controller.current_feedforward_ptr = &pitch_dji_ff;
    }
    if (YAW_MOTOR_TYPE == 0) {
        DJIMotorInstance* m = (DJIMotorInstance*)yaw_handle;
        m->motor_settings.feedforward_flag = CURRENT_FEEDFORWARD;
        m->motor_controller.current_feedforward_ptr = &yaw_dji_ff;
    }
}

void ForceBridge_Update(attitude_t *imu, float target_pitch, float target_yaw, uint8_t is_enabled) {
    // 1. 更新状态
    pitch_axis.current_pos = imu->Pitch;
    pitch_axis.current_vel = imu->Gyro[0]; 
    yaw_axis.current_pos   = imu->YawTotalAngle;
    yaw_axis.current_vel   = imu->Gyro[2];

    // 2. 运行算法 (补全参数)
    if (is_enabled) {
        ForceAxis_SetTarget(&pitch_axis, target_pitch, 0, 0);
        ForceAxis_SetTarget(&yaw_axis, target_yaw, 0, 0);
        
        ForceAxis_Calc(&pitch_axis, 3, 0.01f);
        ForceAxis_Calc(&yaw_axis, 1, 0.01f);
    } else {
        pitch_axis.total_torque = 0;
        yaw_axis.total_torque = 0;
        pitch_dji_ff = 0;
        yaw_dji_ff = 0;
    }

    // 3. 输出给电机 (修正宏名称)
    // --- Pitch ---
    if (PITCH_MOTOR_TYPE == 1) { // 达妙
        if(is_enabled) DMMotorSetRef((DMMotorInstance*)pitch_handle, pitch_axis.total_torque);
        else DMMotorStop((DMMotorInstance*)pitch_handle);
    } else { // 大疆
        pitch_dji_ff = pitch_axis.total_torque;
        if(is_enabled) DJIMotorSetRef((DJIMotorInstance*)pitch_handle, 0); 
        else DJIMotorStop((DJIMotorInstance*)pitch_handle);
    }

    // --- Yaw ---
    if (YAW_MOTOR_TYPE == 1) { // 达妙
        if(is_enabled) DMMotorSetRef((DMMotorInstance*)yaw_handle, yaw_axis.total_torque);
        else DMMotorStop((DMMotorInstance*)yaw_handle);
    } else { // 大疆
        yaw_dji_ff = yaw_axis.total_torque;
        if(is_enabled) DJIMotorSetRef((DJIMotorInstance*)yaw_handle, 0);
        else DJIMotorStop((DJIMotorInstance*)yaw_handle);
    }
}
void ForceBridge_View(uint8_t axis_id) {
    // 根据传入的 ID 选择看哪个轴的数据
    if (axis_id == 0) {
        // 看 Pitch 轴，模式选 TEST (配合你的 Python 脚本)
        Vofa_Just_Sample(VOFA_MODE_TEST, &pitch_axis);
    } else {
        // 看 Yaw 轴
        Vofa_Just_Sample(VOFA_MODE_TEST, &yaw_axis);
    }
}