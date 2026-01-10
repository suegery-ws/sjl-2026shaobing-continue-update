/**
 * @file weighted_filter_example.c
 * @brief 加权滤波器使用示例代码
 * @note 此文件仅作为参考，不需要编译到项目中
 */

#include "weighted_filter.h"

// ==================== 示例1: 陀螺仪角速度滤波 ====================
WeightedFilter_t gyro_z_filter;

void Example1_GyroFilterInit(void)
{
    // 窗口长度5，线性递减权重
    // 权重分布：[5, 4, 3, 2, 1]
    WeightedFilterInit(&gyro_z_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
}

void Example1_GyroFilterTask(void)
{
    // 假设从IMU获取原始角速度
    extern float IMU_GetGyroZ(void);
    float raw_gyro = IMU_GetGyroZ();
    
    // 滤波
    float filtered_gyro = WeightedFilterUpdate(&gyro_z_filter, raw_gyro);
    
    // 使用滤波后的数据
    // chassis_cmd.wz = filtered_gyro;
}

// ==================== 示例2: 视觉目标位置滤波 ====================
WeightedFilter_t vision_yaw_filter;
WeightedFilter_t vision_pitch_filter;

void Example2_VisionFilterInit(void)
{
    // 窗口长度8，指数衰减，alpha=0.3
    // alpha较小，平滑效果强，适合视觉数据
    WeightedFilterInit(&vision_yaw_filter, 8, WEIGHT_EXPONENTIAL, 0.3f);
    WeightedFilterInit(&vision_pitch_filter, 8, WEIGHT_EXPONENTIAL, 0.3f);
}

void Example2_VisionFilterTask(void)
{
    // 假设从视觉模块获取目标位置
    extern float GetVisionYaw(void);
    extern float GetVisionPitch(void);
    
    float raw_yaw = GetVisionYaw();
    float raw_pitch = GetVisionPitch();
    
    // 滤波
    float filtered_yaw = WeightedFilterUpdate(&vision_yaw_filter, raw_yaw);
    float filtered_pitch = WeightedFilterUpdate(&vision_pitch_filter, raw_pitch);
    
    // 使用滤波后的数据
    // gimbal_cmd.yaw = filtered_yaw;
    // gimbal_cmd.pitch = filtered_pitch;
}

// ==================== 示例3: 电机速度滤波 ====================
WeightedFilter_t motor_speed_filter[4];  // 4个电机

void Example3_MotorFilterInit(void)
{
    // 窗口长度7，三角形权重
    // 权重分布：[1, 2, 3, 4, 3, 2, 1]
    for(int i = 0; i < 4; i++)
    {
        WeightedFilterInit(&motor_speed_filter[i], 7, WEIGHT_TRIANGLE, 0.0f);
    }
}

void Example3_MotorFilterTask(void)
{
    // 假设有4个底盘电机
    extern float GetMotorSpeed(int motor_id);
    
    for(int i = 0; i < 4; i++)
    {
        float raw_speed = GetMotorSpeed(i);
        float filtered_speed = WeightedFilterUpdate(&motor_speed_filter[i], raw_speed);
        
        // 使用滤波后的速度进行PID计算
        // PIDCalculate(&speed_pid[i], filtered_speed, target_speed[i]);
    }
}

// ==================== 示例4: 自定义权重滤波 ====================
WeightedFilter_t custom_filter;

void Example4_CustomFilterInit(void)
{
    // 初始化为自定义权重类型
    WeightedFilterInit(&custom_filter, 5, WEIGHT_CUSTOM, 0.0f);
    
    // 设置自定义权重
    // 假设最新3个数据可靠，权重大；旧数据不可靠，权重小
    float custom_weights[5] = {5.0f, 5.0f, 5.0f, 1.0f, 1.0f};
    WeightedFilterSetCustomWeights(&custom_filter, custom_weights);
}

void Example4_CustomFilterTask(void)
{
    extern float GetSensorData(void);
    float raw_data = GetSensorData();
    
    float filtered_data = WeightedFilterUpdate(&custom_filter, raw_data);
    
    // 使用滤波后的数据
}

// ==================== 示例5: 模式切换时重置滤波器 ====================
WeightedFilter_t mode_switch_filter;
static int last_mode = 0;

void Example5_ModeSwitchInit(void)
{
    WeightedFilterInit(&mode_switch_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
}

void Example5_ModeSwitchTask(void)
{
    extern int GetCurrentMode(void);
    extern float GetData(void);
    
    int current_mode = GetCurrentMode();
    
    // 检测模式切换
    if(current_mode != last_mode)
    {
        // 模式切换时重置滤波器，清空历史数据
        WeightedFilterReset(&mode_switch_filter);
        last_mode = current_mode;
    }
    
    float raw_data = GetData();
    float filtered_data = WeightedFilterUpdate(&mode_switch_filter, raw_data);
}

// ==================== 示例6: 不同权重类型对比 ====================
void Example6_CompareWeightTypes(void)
{
    WeightedFilter_t filter_uniform;
    WeightedFilter_t filter_linear;
    WeightedFilter_t filter_exp;
    WeightedFilter_t filter_triangle;
    
    // 初始化不同类型的滤波器（窗口长度都是5）
    WeightedFilterInit(&filter_uniform, 5, WEIGHT_UNIFORM, 0.0f);
    WeightedFilterInit(&filter_linear, 5, WEIGHT_LINEAR_DECAY, 0.0f);
    WeightedFilterInit(&filter_exp, 5, WEIGHT_EXPONENTIAL, 0.3f);
    WeightedFilterInit(&filter_triangle, 5, WEIGHT_TRIANGLE, 0.0f);
    
    // 输入相同的数据，观察不同滤波器的输出
    float test_data = 100.0f;
    
    float out_uniform = WeightedFilterUpdate(&filter_uniform, test_data);
    float out_linear = WeightedFilterUpdate(&filter_linear, test_data);
    float out_exp = WeightedFilterUpdate(&filter_exp, test_data);
    float out_triangle = WeightedFilterUpdate(&filter_triangle, test_data);
    
    // 可以通过调试器观察不同滤波器的输出差异
}

// ==================== 示例7: 预填充滤波器避免初始波动 ====================
WeightedFilter_t prefill_filter;

void Example7_PrefillInit(void)
{
    WeightedFilterInit(&prefill_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
    
    // 获取初始值
    extern float GetInitialValue(void);
    float initial_value = GetInitialValue();
    
    // 预填充滤波器，避免启动初期的波动
    for(int i = 0; i < 5; i++)
    {
        WeightedFilterUpdate(&prefill_filter, initial_value);
    }
}

// ==================== 示例8: 实际应用 - 底盘速度控制 ====================
typedef struct
{
    WeightedFilter_t vx_filter;
    WeightedFilter_t vy_filter;
    WeightedFilter_t wz_filter;
} ChassisFilter_t;

ChassisFilter_t chassis_filter;

void Example8_ChassisFilterInit(void)
{
    // vx, vy使用三角形权重，平滑移动
    WeightedFilterInit(&chassis_filter.vx_filter, 7, WEIGHT_TRIANGLE, 0.0f);
    WeightedFilterInit(&chassis_filter.vy_filter, 7, WEIGHT_TRIANGLE, 0.0f);
    
    // wz使用线性递减，快速响应旋转
    WeightedFilterInit(&chassis_filter.wz_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
}

void Example8_ChassisFilterTask(void)
{
    // 假设从遥控器或上位机获取速度指令
    extern float GetChassisVx(void);
    extern float GetChassisVy(void);
    extern float GetChassisWz(void);
    
    float raw_vx = GetChassisVx();
    float raw_vy = GetChassisVy();
    float raw_wz = GetChassisWz();
    
    // 滤波
    float filtered_vx = WeightedFilterUpdate(&chassis_filter.vx_filter, raw_vx);
    float filtered_vy = WeightedFilterUpdate(&chassis_filter.vy_filter, raw_vy);
    float filtered_wz = WeightedFilterUpdate(&chassis_filter.wz_filter, raw_wz);
    
    // 使用滤波后的速度
    // chassis_cmd.vx = filtered_vx;
    // chassis_cmd.vy = filtered_vy;
    // chassis_cmd.wz = filtered_wz;
}

// ==================== 示例9: 实际应用 - 云台控制 ====================
typedef struct
{
    WeightedFilter_t yaw_filter;
    WeightedFilter_t pitch_filter;
} GimbalFilter_t;

GimbalFilter_t gimbal_filter;

void Example9_GimbalFilterInit(void)
{
    // 云台控制需要快速响应，使用线性递减权重
    WeightedFilterInit(&gimbal_filter.yaw_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
    WeightedFilterInit(&gimbal_filter.pitch_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
}

void Example9_GimbalFilterTask(void)
{
    // 假设从视觉或遥控器获取云台角度指令
    extern float GetGimbalYawCmd(void);
    extern float GetGimbalPitchCmd(void);
    
    float raw_yaw = GetGimbalYawCmd();
    float raw_pitch = GetGimbalPitchCmd();
    
    // 滤波
    float filtered_yaw = WeightedFilterUpdate(&gimbal_filter.yaw_filter, raw_yaw);
    float filtered_pitch = WeightedFilterUpdate(&gimbal_filter.pitch_filter, raw_pitch);
    
    // 使用滤波后的角度
    // gimbal_cmd.yaw = filtered_yaw;
    // gimbal_cmd.pitch = filtered_pitch;
}

// ==================== 示例10: 调试辅助 - 获取输出不更新 ====================
void Example10_DebugGetOutput(void)
{
    WeightedFilter_t debug_filter;
    WeightedFilterInit(&debug_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
    
    // 更新数据
    for(int i = 0; i < 10; i++)
    {
        float data = (float)i;
        WeightedFilterUpdate(&debug_filter, data);
    }
    
    // 多次读取当前输出，不更新数据（用于调试）
    float output1 = WeightedFilterGetOutput(&debug_filter);
    float output2 = WeightedFilterGetOutput(&debug_filter);
    // output1 == output2，因为没有更新数据
}
