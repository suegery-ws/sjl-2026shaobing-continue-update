/**
 * @file weighted_filter.c
 * @brief 加权滑动均值滤波器实现
 */

#include "weighted_filter.h"
#include <math.h>

/**
 * @brief 内部函数：计算并设置权重
 */
static void CalculateWeights(WeightedFilter_t *filter)
{
    uint8_t i;
    filter->weight_sum = 0.0f;
    
    switch (filter->weight_type)
    {
        case WEIGHT_UNIFORM:
            // 均匀权重：所有样本权重相同
            for (i = 0; i < filter->window_size; i++)
            {
                filter->weights[i] = 1.0f;
                filter->weight_sum += 1.0f;
            }
            break;
            
        case WEIGHT_LINEAR_DECAY:
            // 线性递减：最新样本权重最大，线性递减
            // 权重为 N, N-1, N-2, ..., 1
            for (i = 0; i < filter->window_size; i++)
            {
                filter->weights[i] = (float)(filter->window_size - i);
                filter->weight_sum += filter->weights[i];
            }
            break;
            
        case WEIGHT_EXPONENTIAL:
            // 指数衰减：w[i] = alpha * (1-alpha)^i
            // alpha越大，最新样本权重越大，响应越快
            {
                float factor = 1.0f;
                for (i = 0; i < filter->window_size; i++)
                {
                    filter->weights[i] = filter->alpha * factor;
                    filter->weight_sum += filter->weights[i];
                    factor *= (1.0f - filter->alpha);
                }
            }
            break;
            
        case WEIGHT_TRIANGLE:
            // 三角形权重：中心最大，两端最小
            // 对称分布，适合平滑但保留中心特征
            {
                uint8_t mid = filter->window_size / 2;
                for (i = 0; i < filter->window_size; i++)
                {
                    if (i <= mid)
                        filter->weights[i] = (float)(i + 1);
                    else
                        filter->weights[i] = (float)(filter->window_size - i);
                    filter->weight_sum += filter->weights[i];
                }
            }
            break;
            
        case WEIGHT_CUSTOM:
            // 自定义权重：由用户通过SetCustomWeights设置
            // 这里只计算总和
            for (i = 0; i < filter->window_size; i++)
            {
                filter->weight_sum += filter->weights[i];
            }
            break;
            
        default:
            // 默认使用均匀权重
            for (i = 0; i < filter->window_size; i++)
            {
                filter->weights[i] = 1.0f;
                filter->weight_sum += 1.0f;
            }
            break;
    }
}

/**
 * @brief 初始化加权滤波器
 */
int8_t WeightedFilterInit(WeightedFilter_t *filter, uint8_t window_size, 
                          WeightType_e weight_type, float alpha)
{
    // 参数检查
    if (filter == NULL || window_size == 0 || window_size > FILTER_MAX_WINDOW_SIZE)
        return -1;
    
    if (weight_type == WEIGHT_EXPONENTIAL && (alpha <= 0.0f || alpha > 1.0f))
        return -1;
    
    // 初始化结构体
    memset(filter->buffer, 0, sizeof(filter->buffer));
    memset(filter->weights, 0, sizeof(filter->weights));
    
    filter->window_size = window_size;
    filter->head = 0;
    filter->is_full = 0;
    filter->weight_type = weight_type;
    filter->alpha = alpha;
    filter->weight_sum = 0.0f;
    
    // 计算权重
    CalculateWeights(filter);
    
    return 0;
}

/**
 * @brief 设置自定义权重数组
 */
int8_t WeightedFilterSetCustomWeights(WeightedFilter_t *filter, const float *custom_weights)
{
    if (filter == NULL || custom_weights == NULL)
        return -1;
    
    if (filter->weight_type != WEIGHT_CUSTOM)
        return -1;
    
    uint8_t i;
    filter->weight_sum = 0.0f;
    
    for (i = 0; i < filter->window_size; i++)
    {
        filter->weights[i] = custom_weights[i];
        filter->weight_sum += custom_weights[i];
    }
    
    // 防止权重总和为0
    if (filter->weight_sum == 0.0f)
        return -1;
    
    return 0;
}

/**
 * @brief 更新滤波器
 */
float WeightedFilterUpdate(WeightedFilter_t *filter, float input)
{
    if (filter == NULL)
        return 0.0f;
    
    // 将新样本存入循环队列
    filter->buffer[filter->head] = input;
    
    // 更新头指针（循环）
    filter->head++;
    if (filter->head >= filter->window_size)
    {
        filter->head = 0;
        filter->is_full = 1;  // 标记缓冲区已满
    }
    
    // 计算加权平均
    float weighted_sum = 0.0f;
    uint8_t count = filter->is_full ? filter->window_size : filter->head;
    uint8_t i, idx;
    
    for (i = 0; i < count; i++)
    {
        // 计算实际索引：从最新到最旧
        // head指向下一个要写入的位置，所以最新数据在head-1
        if (filter->head == 0)
            idx = filter->window_size - 1 - i;
        else
            idx = (filter->head - 1 - i + filter->window_size) % filter->window_size;
        
        weighted_sum += filter->buffer[idx] * filter->weights[i];
    }
    
    // 归一化（除以权重总和）
    if (filter->weight_sum > 0.0f)
        return weighted_sum / filter->weight_sum;
    else
        return 0.0f;
}

/**
 * @brief 重置滤波器
 */
void WeightedFilterReset(WeightedFilter_t *filter)
{
    if (filter == NULL)
        return;
    
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->head = 0;
    filter->is_full = 0;
}

/**
 * @brief 获取当前滤波器输出
 */
float WeightedFilterGetOutput(WeightedFilter_t *filter)
{
    if (filter == NULL || filter->head == 0)
        return 0.0f;
    
    // 计算加权平均（不更新数据）
    float weighted_sum = 0.0f;
    uint8_t count = filter->is_full ? filter->window_size : filter->head;
    uint8_t i, idx;
    
    for (i = 0; i < count; i++)
    {
        if (filter->head == 0)
            idx = filter->window_size - 1 - i;
        else
            idx = (filter->head - 1 - i + filter->window_size) % filter->window_size;
        
        weighted_sum += filter->buffer[idx] * filter->weights[i];
    }
    
    if (filter->weight_sum > 0.0f)
        return weighted_sum / filter->weight_sum;
    else
        return 0.0f;
}
