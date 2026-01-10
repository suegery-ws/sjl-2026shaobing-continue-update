/**
 * @file weighted_filter.h
 * @brief 加权滑动均值滤波器模块
 * @details 提供多种加权策略的滑动窗口均值滤波，用于信号平滑和噪声抑制
 * @date 2026-01-10
 */

#ifndef WEIGHTED_FILTER_H
#define WEIGHTED_FILTER_H

#include <stdint.h>
#include <string.h>

// 滤波器窗口最大长度（可根据需要调整）
#define FILTER_MAX_WINDOW_SIZE 20

/**
 * @brief 加权策略类型枚举
 */
typedef enum
{
    WEIGHT_UNIFORM = 0,      // 均匀权重（简单均值）
    WEIGHT_LINEAR_DECAY,     // 线性递减权重（最新样本权重最大）
    WEIGHT_EXPONENTIAL,      // 指数衰减权重（类似一阶低通）
    WEIGHT_TRIANGLE,         // 三角形权重（中心最大，两端最小）
    WEIGHT_CUSTOM            // 自定义权重数组
} WeightType_e;

/**
 * @brief 加权滑动均值滤波器结构体
 */
typedef struct
{
    float buffer[FILTER_MAX_WINDOW_SIZE];  // 数据缓冲区（循环队列）
    float weights[FILTER_MAX_WINDOW_SIZE]; // 权重数组
    float weight_sum;                       // 权重总和（归一化用）
    
    uint8_t window_size;                    // 窗口长度（实际使用的样本数）
    uint8_t head;                           // 循环队列头指针（最新数据位置）
    uint8_t is_full;                        // 缓冲区是否已满标志
    
    WeightType_e weight_type;               // 权重类型
    float alpha;                            // 指数衰减系数（仅用于WEIGHT_EXPONENTIAL）
} WeightedFilter_t;

/**
 * @brief 初始化加权滤波器
 * @param filter 滤波器实例指针
 * @param window_size 窗口长度（1~FILTER_MAX_WINDOW_SIZE）
 * @param weight_type 权重类型
 * @param alpha 指数衰减系数（仅WEIGHT_EXPONENTIAL需要，范围0~1，越大响应越快）
 * @retval 0: 成功, -1: 参数错误
 * @note 窗口长度越大平滑效果越强，但延迟也越大
 */
int8_t WeightedFilterInit(WeightedFilter_t *filter, uint8_t window_size, 
                          WeightType_e weight_type, float alpha);

/**
 * @brief 设置自定义权重数组
 * @param filter 滤波器实例指针
 * @param custom_weights 自定义权重数组（长度必须等于window_size）
 * @retval 0: 成功, -1: 参数错误
 * @note 调用此函数前必须先用WEIGHT_CUSTOM类型初始化滤波器
 */
int8_t WeightedFilterSetCustomWeights(WeightedFilter_t *filter, const float *custom_weights);

/**
 * @brief 更新滤波器（输入新样本并计算输出）
 * @param filter 滤波器实例指针
 * @param input 新输入样本
 * @return 滤波后的输出值
 * @note 每次调用会将新样本加入窗口，自动丢弃最旧样本
 */
float WeightedFilterUpdate(WeightedFilter_t *filter, float input);

/**
 * @brief 重置滤波器（清空缓冲区）
 * @param filter 滤波器实例指针
 * @note 权重配置不变，仅清空历史数据
 */
void WeightedFilterReset(WeightedFilter_t *filter);

/**
 * @brief 获取当前滤波器输出（不更新数据）
 * @param filter 滤波器实例指针
 * @return 当前滤波输出值
 * @note 如果缓冲区为空返回0
 */
float WeightedFilterGetOutput(WeightedFilter_t *filter);

#endif // WEIGHTED_FILTER_H
