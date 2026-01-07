/**
 * @file lowpass_filter.c
 * @brief 一阶低通滤波器实现
 * @date 2026-01-07
 */

#include "lowpass_filter.h"
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化一阶低通滤波器
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL)
        return;
    
    // 限制alpha范围在(0, 1]
    if (alpha <= 0.0f)
        alpha = 0.01f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    
    filter->alpha = alpha;
    filter->output = 0.0f;
    filter->is_init = 0;
}

/**
 * @brief 通过频率参数初始化滤波器
 */
void LowPassFilter_Init_ByFreq(LowPassFilter_t *filter, float sample_freq, float cutoff_freq)
{
    if (filter == NULL || sample_freq <= 0.0f || cutoff_freq <= 0.0f)
        return;
    
    // 计算RC时间常数
    float RC = 1.0f / (2.0f * PI * cutoff_freq);
    float dt = 1.0f / sample_freq;
    
    // 计算alpha: alpha = dt / (dt + RC)
    float alpha = dt / (dt + RC);
    
    LowPassFilter_Init(filter, alpha);
}

/**
 * @brief 一阶低通滤波器更新
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input)
{
    if (filter == NULL)
        return input;
    
    // 第一次更新时，直接使用输入值作为输出，避免从0开始的突变
    if (!filter->is_init)
    {
        filter->output = input;
        filter->is_init = 1;
        return filter->output;
    }
    
    // 一阶低通滤波公式: y[n] = α * x[n] + (1 - α) * y[n-1]
    // 其中: x[n]是当前输入, y[n-1]是上一次输出, y[n]是当前输出
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    
    return filter->output;
}

/**
 * @brief 重置滤波器状态
 */
void LowPassFilter_Reset(LowPassFilter_t *filter, float init_value)
{
    if (filter == NULL)
        return;
    
    filter->output = init_value;
    filter->is_init = 1;
}

/**
 * @brief 获取当前滤波器输出值
 */
float LowPassFilter_GetOutput(LowPassFilter_t *filter)
{
    if (filter == NULL)
        return 0.0f;
    
    return filter->output;
}

/**
 * @brief 设置滤波系数
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL)
        return;
    
    // 限制alpha范围
    if (alpha <= 0.0f)
        alpha = 0.01f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    
    filter->alpha = alpha;
}
