/**
 * @file lowpass_filter.h
 * @brief 一阶低通滤波器模块
 * @details 提供平滑的数据滤波功能，减少噪声和突变
 * @date 2026-01-07
 */

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

/**
 * @brief 一阶低通滤波器结构体
 */
typedef struct
{
    float output;      // 滤波器输出值（上一次的输出）
    float alpha;       // 滤波系数 (0 < alpha <= 1)
                       // alpha越小，滤波效果越强，响应越慢
                       // alpha越大，滤波效果越弱，响应越快
    uint8_t is_init;   // 初始化标志位
} LowPassFilter_t;

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param alpha 滤波系数 (0 < alpha <= 1)
 *              推荐值: 0.1-0.3 (强滤波), 0.5-0.7 (中等), 0.8-0.95 (弱滤波)
 * @note alpha = dt / (dt + RC), 其中dt为采样周期，RC为时间常数
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);

/**
 * @brief 通过时间常数初始化一阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param sample_freq 采样频率 (Hz)
 * @param cutoff_freq 截止频率 (Hz)
 * @note 自动计算alpha = 2*PI*cutoff_freq / (2*PI*cutoff_freq + sample_freq)
 */
void LowPassFilter_Init_ByFreq(LowPassFilter_t *filter, float sample_freq, float cutoff_freq);

/**
 * @brief 一阶低通滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入的原始数据
 * @return 滤波后的输出数据
 * @note 滤波公式: output = alpha * input + (1 - alpha) * output_last
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input);

/**
 * @brief 重置滤波器状态
 * @param filter 滤波器结构体指针
 * @param init_value 初始值（可选，传入当前测量值避免启动突变）
 */
void LowPassFilter_Reset(LowPassFilter_t *filter, float init_value);

/**
 * @brief 获取当前滤波器输出值
 * @param filter 滤波器结构体指针
 * @return 当前输出值
 */
float LowPassFilter_GetOutput(LowPassFilter_t *filter);

/**
 * @brief 设置滤波系数
 * @param filter 滤波器结构体指针
 * @param alpha 新的滤波系数
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha);

#endif // LOWPASS_FILTER_H
