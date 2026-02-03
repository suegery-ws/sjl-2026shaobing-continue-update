#include "force_gimbal_core.h"
#include <string.h> // memset

// --- 内部工具 ---
static float fsgn(float x) {
    return (x > 0.001f) ? 1.0f : ((x < -0.001f) ? -1.0f : 0.0f);
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// PID 计算
static float PID_Calc(ForcePID_t *pid, float target, float measure) {
    float error = target - measure;
    pid->error_sum += error;
    
    if (pid->error_sum > pid->max_iout) pid->error_sum = pid->max_iout;
    else if (pid->error_sum < -pid->max_iout) pid->error_sum = -pid->max_iout;

    float output = pid->kp * error + pid->ki * pid->error_sum + pid->kd * (error - pid->last_error);
    pid->last_error = error;
    
    if (output > pid->max_out) output = pid->max_out;
    else if (output < -pid->max_out) output = -pid->max_out;
    return output;
}

// --- 接口实现 ---

void ForceAxis_Init(ForceAxis_t *axis, ForceMotorType_e type, float scale, 
                    float j, float b, float c, float g_cos, float g_sin) {
    memset(axis, 0, sizeof(ForceAxis_t));
    axis->motor_type = type;
    axis->output_scale = scale; // ★ 关键：这里决定了是物理派还是工程派
    axis->J = j; axis->B = b; axis->C = c;
    axis->G_cos = g_cos; axis->G_sin = g_sin;
}

void ForceAxis_SetPID(ForceAxis_t *axis, float p_kp, float p_ki, float p_kd, 
                      float v_kp, float v_ki, float v_kd) {
    axis->pid_pos.kp = p_kp; axis->pid_pos.ki = p_ki; axis->pid_pos.kd = p_kd;
    axis->pid_vel.kp = v_kp; axis->pid_vel.ki = v_ki; axis->pid_vel.kd = v_kd;
    // 默认限幅，可按需修改
    axis->pid_pos.max_out = 20000.0f; axis->pid_pos.max_iout = 1000.0f;
    axis->pid_vel.max_out = 30000.0f; axis->pid_vel.max_iout = 5000.0f;
}

void ForceAxis_UpdateFeedback(ForceAxis_t *axis, float pos_rad, float vel_rads) {
    axis->current_pos = pos_rad;
    axis->current_vel = vel_rads;
    
    static uint8_t init_flag = 0;
    if (init_flag == 0) {
        axis->start_pos = pos_rad;
        init_flag = 1;
    }
}

void ForceAxis_SetTarget(ForceAxis_t *axis, float pos, float vel, float acc) {
    axis->target_pos = pos;
    axis->target_vel = vel;
    axis->target_acc = acc;
}

// ★ 核心魔法发生地 ★
void ForceAxis_Calc(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec) {
    if (mode == MODE_DISABLE) {
        axis->output_raw = 0;
        axis->total_torque = 0;
        return;
    }

    // 1. 生成测试波形 (如果是测量模式/验证模式)
    if (mode == MODE_MEASURE) {
        float amp = 0.1f; 
        float freq = 1.2f;
        
        // // 针对不同电机微调幅度
        // if(axis->motor_type == FORCE_MOTOR_TYPE_GM6020_VOLTAGE) amp = 0.08f; 

        // ★★★ 核心分流：通过是否设置了重力前馈(G_cos)来区分 Pitch 和 Yaw ★★★
        if (axis->G_cos != 0.0f) 
        {
            // =================================================
            //               Pitch 轴专用逻辑 (重力 + 虚拟弹簧)
            // =================================================
            
            // 1. 托举力 (Offset): 直接复用 Init 里的 G_cos 参数
            // 这样你只用在 gimbal_init 里改一次 -0.3，这里自动生效
            float offset = axis->G_cos; 
            
            // 2. 虚拟弹簧 (Spring): 专门解决“抬高不回落”的问题
            // 负号表示反向拉回来，Kp = 1.0 (如果拉不回来就加大到 2.0)
            float spring_torque = -1.0f * axis->current_pos;

            // 合成公式：托举 + 正弦晃动 + 弹簧拉住
            axis->total_torque = offset + amp * sinf(freq * 2.0f * 3.14159f * t_sec) + spring_torque;
        }
        else 
        {
            // =================================================
            //               Yaw 轴专用逻辑 (纯净摇摆)
            // =================================================
            
            // Yaw 轴在水平面，没有重力，也不需要弹簧(不会飘)
            // 只需要最纯净的正弦波
            axis->total_torque = amp * sinf(freq * 2.0f * 3.14159f * t_sec);
        }

        // ★ 统一操作：测量模式下，必须屏蔽 PID 和 物理模型前馈
        axis->ff_torque = 0;
        axis->pid_torque = 0;
    }
    else if (mode == MODE_VALIDATE) {
        // 闭环轨迹验证
        float freq = 1.0f; 
        float amp_rad = 0.2f; // ±11度
        float omega = 2.0f * 3.14159f * freq;
        axis->target_pos = amp_rad * sinf(omega * t_sec) + axis->start_pos;
        axis->target_vel = amp_rad * omega * cosf(omega * t_sec);
        axis->target_acc = -amp_rad * omega * omega * sinf(omega * t_sec);
    }
    
    // 2. 闭环控制 (比赛/验证模式)
    if (mode != MODE_MEASURE) {
        // A. 物理模型前馈 (The Physics/Black-Box Model)
        // 无论是真实的惯量，还是电压环的虚拟惯量，公式结构是一样的！
        axis->ff_torque = 
            axis->J * axis->target_acc +
            axis->B * axis->target_vel +
            axis->C * fsgn(axis->target_vel) +
            axis->G_cos * cosf(axis->current_pos) +
            axis->G_sin * sinf(axis->current_pos);

        // B. 串级 PID + 速度前馈
        float v_cmd = PID_Calc(&axis->pid_pos, axis->target_pos, axis->current_pos);
        axis->pid_torque = PID_Calc(&axis->pid_vel, v_cmd + axis->target_vel, axis->current_vel);
        
        // C. 合成
        axis->total_torque = axis->ff_torque + axis->pid_torque;
    }

    // 3. 硬件映射 (Hardware Mapping)
    
    // --- 达妙 MIT ---
    if (axis->motor_type == FORCE_MOTOR_TYPE_DM_MIT) {
        // 限制力矩
        if (axis->total_torque > 10.0f) axis->total_torque = 10.0f;
        if (axis->total_torque < -10.0f) axis->total_torque = -10.0f;
        
        // 填充 MIT 数据包
        uint16_t p = float_to_uint(0, MIT_P_MIN, MIT_P_MAX, 16);
        uint16_t v = float_to_uint(0, MIT_V_MIN, MIT_V_MAX, 12);
        uint16_t kp = float_to_uint(0, MIT_KP_MIN, MIT_KP_MAX, 12);
        uint16_t kd = float_to_uint(0, MIT_KD_MIN, MIT_KD_MAX, 12);
        uint16_t t = float_to_uint(axis->total_torque * axis->output_scale, MIT_T_MIN, MIT_T_MAX, 12);
        
        axis->mit_frame.data[0] = (p >> 8);
        axis->mit_frame.data[1] = p;
        axis->mit_frame.data[2] = (v >> 4);
        axis->mit_frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
        axis->mit_frame.data[4] = kp;
        axis->mit_frame.data[5] = (kd >> 4);
        axis->mit_frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
        axis->mit_frame.data[7] = t;
        
        // 为了方便调试观察，这里记录下发送的力矩
        axis->output_raw = (int16_t)(axis->total_torque * 100.0f); 
    }
    // --- GM6020 (电压或电流) ---
    else {
        // 这里的 output_scale 发挥神威！
        // 如果是 电压模式：total_torque 是归一化值，scale 是最大电压(如 25000)
        // 如果是 电流模式：total_torque 是 Nm，scale 是 7370
        float out_val = axis->total_torque * axis->output_scale;
        
        // 安全限幅 (GM6020 满值 30000)
        if (out_val > 29000.0f) out_val = 29000.0f;
        if (out_val < -29000.0f) out_val = -29000.0f;
        
        axis->output_raw = (int16_t)out_val;
    }
}