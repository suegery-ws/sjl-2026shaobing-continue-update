# 加权滑动均值滤波器使用说明

## 一、功能概述

加权滑动均值滤波器是一个用于信号平滑和噪声抑制的模块，支持多种加权策略：
- **均匀权重**：简单均值，所有样本权重相同
- **线性递减**：最新样本权重最大，适合快速响应
- **指数衰减**：类似一阶低通滤波，可调节响应速度
- **三角形权重**：中心样本权重最大，适合平滑信号
- **自定义权重**：用户自定义权重数组

## 二、文件说明

- `weighted_filter.h` - 头文件，包含接口定义
- `weighted_filter.c` - 实现文件
- 位置：`modules/algorithm/`

## 三、快速开始

### 3.1 基本使用流程

```c
#include "weighted_filter.h"

// 1. 定义滤波器实例
WeightedFilter_t my_filter;

// 2. 初始化滤波器
WeightedFilterInit(&my_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);

// 3. 在循环中更新数据
float raw_data = GetSensorData();  // 获取原始数据
float filtered_data = WeightedFilterUpdate(&my_filter, raw_data);

// 4. 使用滤波后的数据
SetMotorSpeed(filtered_data);
```

## 四、详细使用示例

### 示例1：陀螺仪数据平滑（线性递减权重）

```c
// 适用场景：陀螺仪角速度平滑，需要快速响应但抑制高频噪声

WeightedFilter_t gyro_filter;

void GyroFilterInit(void)
{
    // 窗口长度5，线性递减权重
    // 权重分布：[5, 4, 3, 2, 1]，最新数据权重最大
    WeightedFilterInit(&gyro_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
}

void GyroTask(void)
{
    float raw_gyro = IMU_GetGyroZ();  // 获取原始角速度
    float filtered_gyro = WeightedFilterUpdate(&gyro_filter, raw_gyro);
    
    // 使用滤波后的角速度
    chassis_cmd.wz = filtered_gyro;
}
```

### 示例2：视觉目标位置平滑（指数衰减）

```c
// 适用场景：视觉识别目标位置，需要平滑但保持跟踪性能

WeightedFilter_t vision_yaw_filter;
WeightedFilter_t vision_pitch_filter;

void VisionFilterInit(void)
{
    // 窗口长度8，指数衰减，alpha=0.3（较强平滑）
    WeightedFilterInit(&vision_yaw_filter, 8, WEIGHT_EXPONENTIAL, 0.3f);
    WeightedFilterInit(&vision_pitch_filter, 8, WEIGHT_EXPONENTIAL, 0.3f);
}

void VisionTask(void)
{
    float raw_yaw = vision_data->yaw;
    float raw_pitch = vision_data->pitch;
    
    float filtered_yaw = WeightedFilterUpdate(&vision_yaw_filter, raw_yaw);
    float filtered_pitch = WeightedFilterUpdate(&vision_pitch_filter, raw_pitch);
    
    gimbal_cmd.yaw = filtered_yaw;
    gimbal_cmd.pitch = filtered_pitch;
}
```

### 示例3：电机速度平滑（三角形权重）

```c
// 适用场景：电机速度反馈平滑，需要保留中心趋势

WeightedFilter_t motor_speed_filter;

void MotorFilterInit(void)
{
    // 窗口长度7，三角形权重
    // 权重分布：[1, 2, 3, 4, 3, 2, 1]，中心权重最大
    WeightedFilterInit(&motor_speed_filter, 7, WEIGHT_TRIANGLE, 0.0f);
}

void MotorTask(void)
{
    float raw_speed = motor->measure.speed_aps;
    float filtered_speed = WeightedFilterUpdate(&motor_speed_filter, raw_speed);
    
    // 使用滤波后的速度进行PID计算
    PIDCalculate(&speed_pid, filtered_speed, target_speed);
}
```

### 示例4：自定义权重（质量加权）

```c
// 适用场景：根据数据质量动态加权

WeightedFilter_t custom_filter;

void CustomFilterInit(void)
{
    // 初始化为自定义权重类型
    WeightedFilterInit(&custom_filter, 5, WEIGHT_CUSTOM, 0.0f);
    
    // 设置自定义权重数组
    // 假设最新3个数据质量高，权重大；旧数据质量低，权重小
    float custom_weights[5] = {5.0f, 5.0f, 5.0f, 1.0f, 1.0f};
    WeightedFilterSetCustomWeights(&custom_filter, custom_weights);
}

void CustomTask(void)
{
    float raw_data = GetData();
    float filtered_data = WeightedFilterUpdate(&custom_filter, raw_data);
}
```

## 五、参数选择指南

### 5.1 窗口长度（window_size）选择

| 窗口长度 | 平滑效果 | 延迟 | 适用场景 |
|---------|---------|------|---------|
| 3-5     | 弱      | 小   | 高频控制，需要快速响应 |
| 5-10    | 中等    | 中等 | 一般传感器数据平滑 |
| 10-20   | 强      | 大   | 低频信号，噪声很大 |

**建议**：
- 控制频率高（>100Hz）：窗口3-5
- 控制频率中等（50-100Hz）：窗口5-8
- 控制频率低（<50Hz）：窗口8-15

### 5.2 权重类型选择

| 权重类型 | 特点 | 适用场景 |
|---------|------|---------|
| WEIGHT_UNIFORM | 简单均值，延迟最大 | 噪声均匀分布，不要求快速响应 |
| WEIGHT_LINEAR_DECAY | 响应快，平滑适中 | 需要快速跟踪变化的信号（如陀螺仪） |
| WEIGHT_EXPONENTIAL | 可调节响应速度 | 需要灵活调节平滑/响应平衡 |
| WEIGHT_TRIANGLE | 保留中心特征 | 信号有明显趋势，需要平滑边缘噪声 |
| WEIGHT_CUSTOM | 完全自定义 | 特殊需求，如质量加权 |

### 5.3 指数衰减系数（alpha）选择

仅用于 `WEIGHT_EXPONENTIAL` 类型：

| alpha值 | 响应速度 | 平滑效果 | 适用场景 |
|---------|---------|---------|---------|
| 0.1-0.2 | 慢      | 强      | 噪声很大，允许较大延迟 |
| 0.3-0.5 | 中等    | 中等    | 一般传感器数据 |
| 0.6-0.8 | 快      | 弱      | 需要快速响应，噪声较小 |

**公式**：`w[i] = alpha * (1-alpha)^i`
- alpha越大，最新样本权重越大，响应越快
- alpha越小，历史样本权重越均匀，平滑越强

## 六、API接口说明

### 6.1 初始化函数

```c
int8_t WeightedFilterInit(WeightedFilter_t *filter, 
                          uint8_t window_size, 
                          WeightType_e weight_type, 
                          float alpha);
```

**参数**：
- `filter`: 滤波器实例指针
- `window_size`: 窗口长度（1-20）
- `weight_type`: 权重类型（见枚举定义）
- `alpha`: 指数衰减系数（仅WEIGHT_EXPONENTIAL需要，范围0-1）

**返回值**：
- `0`: 成功
- `-1`: 参数错误

### 6.2 更新函数

```c
float WeightedFilterUpdate(WeightedFilter_t *filter, float input);
```

**参数**：
- `filter`: 滤波器实例指针
- `input`: 新输入样本

**返回值**：滤波后的输出值

**说明**：每次调用会将新样本加入窗口，自动丢弃最旧样本

### 6.3 重置函数

```c
void WeightedFilterReset(WeightedFilter_t *filter);
```

**功能**：清空历史数据缓冲区，权重配置不变

**使用场景**：
- 模式切换时需要清空历史数据
- 检测到异常数据需要重新开始

### 6.4 获取输出函数

```c
float WeightedFilterGetOutput(WeightedFilter_t *filter);
```

**功能**：获取当前滤波输出，不更新数据

**使用场景**：需要多次读取同一滤波结果

### 6.5 自定义权重函数

```c
int8_t WeightedFilterSetCustomWeights(WeightedFilter_t *filter, 
                                      const float *custom_weights);
```

**参数**：
- `filter`: 滤波器实例指针（必须先用WEIGHT_CUSTOM初始化）
- `custom_weights`: 自定义权重数组（长度等于window_size）

**返回值**：
- `0`: 成功
- `-1`: 参数错误

## 七、注意事项

### 7.1 内存占用

每个滤波器实例占用约 `(FILTER_MAX_WINDOW_SIZE * 2 + 1) * 4 + 4 = 168字节`

如果需要大量滤波器实例，可以减小 `FILTER_MAX_WINDOW_SIZE` 宏定义。

### 7.2 计算开销

- 每次 `Update` 调用需要遍历窗口进行加权求和
- 时间复杂度：O(window_size)
- 建议在控制周期内调用，避免在中断中频繁调用

### 7.3 初始化阶段

滤波器启动初期（缓冲区未满时），使用已有样本计算均值，可能与稳态输出有差异。

如需避免初始波动，可以：
```c
// 预填充滤波器
for(int i = 0; i < window_size; i++)
{
    WeightedFilterUpdate(&filter, initial_value);
}
```

### 7.4 数据跳变处理

当输入数据发生大幅跳变时（如模式切换），建议重置滤波器：
```c
if(mode_changed)
{
    WeightedFilterReset(&filter);
}
```

## 八、性能对比示例

假设输入信号：正弦波 + 高斯噪声

| 配置 | 延迟 | 噪声抑制 | 跟踪性能 |
|-----|------|---------|---------|
| 窗口5 + 均匀权重 | 中 | 中 | 中 |
| 窗口5 + 线性递减 | 小 | 中 | 好 |
| 窗口8 + 指数(0.3) | 中 | 强 | 中 |
| 窗口8 + 三角形 | 中 | 强 | 中 |

**建议测试流程**：
1. 先用小窗口（3-5）+ 线性递减测试
2. 如果噪声仍大，增加窗口或改用指数衰减
3. 如果延迟太大，减小窗口或增大alpha
4. 根据实际效果微调参数

## 九、常见问题

**Q1: 滤波后信号仍有抖动？**
- 增加窗口长度
- 改用更强平滑的权重类型（如指数衰减，alpha=0.2）

**Q2: 滤波后响应太慢？**
- 减小窗口长度
- 改用线性递减权重
- 如果用指数衰减，增大alpha值

**Q3: 如何选择合适的参数？**
- 先根据控制频率选窗口长度（见5.1节）
- 再根据应用场景选权重类型（见5.2节）
- 最后通过实际测试微调

**Q4: 可以用于角度滤波吗？**
- 可以，但需要注意角度跳变问题（如-180°到180°）
- 建议先转换为连续角度或使用专门的角度滤波器

## 十、集成到项目

### 10.1 添加到编译

在你的 `Makefile` 或 `CMakeLists.txt` 中添加：
```makefile
# Makefile
C_SOURCES += modules/algorithm/weighted_filter.c

C_INCLUDES += -Imodules/algorithm
```

### 10.2 在代码中使用

```c
// 在需要使用的文件中包含头文件
#include "weighted_filter.h"

// 定义全局或静态滤波器实例
static WeightedFilter_t gyro_filter;
static WeightedFilter_t vision_filter;

// 在初始化函数中初始化滤波器
void MyModuleInit(void)
{
    WeightedFilterInit(&gyro_filter, 5, WEIGHT_LINEAR_DECAY, 0.0f);
    WeightedFilterInit(&vision_filter, 8, WEIGHT_EXPONENTIAL, 0.3f);
}

// 在任务函数中使用
void MyTask(void)
{
    float filtered_gyro = WeightedFilterUpdate(&gyro_filter, raw_gyro);
    float filtered_vision = WeightedFilterUpdate(&vision_filter, raw_vision);
}
```

---

**版本**: v1.0  
**日期**: 2026-01-10  
**作者**: Cascade AI Assistant
