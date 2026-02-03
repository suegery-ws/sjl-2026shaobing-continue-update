#include "vofa_task.h"
#include "usart.h" // ★ 必须包含这个，才能用 huart6
#include "main.h"

// 引用外部定义的串口句柄 (确保你的 main.c 或 usart.c 里定义了 huart6)
extern UART_HandleTypeDef huart6;

// JustFloat 协议的帧尾 (固定值: 00 00 80 7F)
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

// 底层发送函数 (改用 HAL_UART_Transmit)
static void Vofa_Send_Raw(float *data, uint8_t count)
{
    // 1. 发送数据体 (count * 4 字节)
    // 使用阻塞式发送，超时时间设为 10ms (足够了，只要波特率够高)
    HAL_UART_Transmit(&huart6, (uint8_t *)data, count * sizeof(float), 10);
    
    // 2. 发送帧尾 (4 字节)
    HAL_UART_Transmit(&huart6, (uint8_t *)tail, 4, 10);
}

// 核心功能函数 (升级版：7 通道，新增速度对比)
void Vofa_Just_Sample(Vofa_Mode_e mode, ForceAxis_t *axis)
{
    // ★ 1. 改大数组：原来是 6，现在改成 7
    float send_buffer[7];

    if (mode == VOFA_MODE_TEST)
    {
        // 模式一：保持不变
        send_buffer[0] = axis->current_pos;    
        send_buffer[1] = axis->current_vel;    
        send_buffer[2] = axis->total_torque;   
        send_buffer[3] = 0.0f; 
        
        Vofa_Send_Raw(send_buffer, 4);
    }
    else
    {
        // ============================================
        // 【模式二：精准验证专用】(7 通道版)
        // ============================================
        
        float position_error = axis->target_pos - axis->current_pos;

        send_buffer[0] = axis->target_pos;    // ch0: 目标位置
        send_buffer[1] = axis->current_pos;   // ch1: 实际位置
        send_buffer[2] = position_error;      // ch2: 位置误差
        
        send_buffer[3] = axis->ff_torque;     // ch3: 前馈出力 (绿线)
        send_buffer[4] = axis->pid_torque;    // ch4: PID出力 (红线)
        send_buffer[5] = axis->total_torque;  // ch5: 总出力
        
        // ★★★ 新增通道：目标速度 ★★★
        // 为什么看“目标速度”？因为前馈里的阻尼项是算出来的 (B * target_vel)
        // 如果 ch3 (前馈) 和 ch6 (速度) 长得一模一样，那就是实锤了！
        send_buffer[6] = axis->target_vel;    // ch6: 目标速度 (对比用)
        
        // ★ 2. 发送长度改成 7
        Vofa_Send_Raw(send_buffer, 7);
    }
}