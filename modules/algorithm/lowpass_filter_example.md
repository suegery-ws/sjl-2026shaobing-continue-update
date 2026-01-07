# 一阶低通滤波器使用说明

## 功能介绍
一阶低通滤波器用于平滑数据，减少噪声和突变，使数据变化更加平稳。

## 滤波原理
滤波公式：`output = α × input + (1 - α) × output_last`

- **α (alpha)**: 滤波系数，范围 (0, 1]
  - α 越小：滤波越强，响应越慢，数据越平滑
  - α 越大：滤波越弱，响应越快，更接近原始数据

## 使用方法

### 方法1：直接指定alpha系数

```c
#include "lowpass_filter.h"

// 1. 定义滤波器实例
LowPassFilter_t pitch_filter;

// 2. 初始化滤波器 (alpha = 0.3 表示中等强度滤波)
LowPassFilter_Init(&pitch_filter, 0.3f);

// 3. 在循环中更新滤波器
float raw_pitch = 45.5f;  // 原始数据
float filtered_pitch = LowPassFilter_Update(&pitch_filter, raw_pitch);

// filtered_pitch 就是滤波后的平滑数据
```

### 方法2：通过频率参数初始化（推荐）

```c
#include "lowpass_filter.h"

// 1. 定义滤波器实例
LowPassFilter_t yaw_filter;

// 2. 通过采样频率和截止频率初始化
// 采样频率: 1000Hz (1ms采样一次)
// 截止频率: 50Hz (允许50Hz以下的信号通过)
LowPassFilter_Init_ByFreq(&yaw_filter, 1000.0f, 50.0f);

// 3. 在循环中更新
float raw_yaw = GetRawYaw();
float filtered_yaw = LowPassFilter_Update(&yaw_filter, raw_yaw);
```

## 实际应用示例

### 示例1：视觉数据滤波

```c
// 在文件开头定义滤波器
static LowPassFilter_t vision_pitch_filter;
static LowPassFilter_t vision_yaw_filter;

// 在初始化函数中
void VisionInit(void)
{
    // 视觉数据采样频率约100Hz，截止频率设为20Hz
    LowPassFilter_Init_ByFreq(&vision_pitch_filter, 100.0f, 20.0f);
    LowPassFilter_Init_ByFreq(&vision_yaw_filter, 100.0f, 20.0f);
}

// 在接收视觉数据的回调中
void VisionDataCallback(float raw_pitch, float raw_yaw)
{
    float smooth_pitch = LowPassFilter_Update(&vision_pitch_filter, raw_pitch);
    float smooth_yaw = LowPassFilter_Update(&vision_yaw_filter, raw_yaw);
    
    // 使用平滑后的数据
    SetGimbalTarget(smooth_pitch, smooth_yaw);
}
```

### 示例2：IMU数据滤波

```c
// 定义滤波器
static LowPassFilter_t gyro_x_filter;
static LowPassFilter_t gyro_y_filter;
static LowPassFilter_t gyro_z_filter;

// 初始化 (IMU采样1000Hz，截止频率100Hz)
void IMU_FilterInit(void)
{
    LowPassFilter_Init_ByFreq(&gyro_x_filter, 1000.0f, 100.0f);
    LowPassFilter_Init_ByFreq(&gyro_y_filter, 1000.0f, 100.0f);
    LowPassFilter_Init_ByFreq(&gyro_z_filter, 1000.0f, 100.0f);
}

// 在IMU更新任务中
void IMU_Task(void)
{
    float raw_gyro[3];
    IMU_GetGyro(raw_gyro);
    
    float filtered_gyro[3];
    filtered_gyro[0] = LowPassFilter_Update(&gyro_x_filter, raw_gyro[0]);
    filtered_gyro[1] = LowPassFilter_Update(&gyro_y_filter, raw_gyro[1]);
    filtered_gyro[2] = LowPassFilter_Update(&gyro_z_filter, raw_gyro[2]);
    
    // 使用滤波后的数据
    UseFilteredGyro(filtered_gyro);
}
```

### 示例3：底盘速度滤波

```c
// 在chassis.c中
static LowPassFilter_t chassis_vx_filter;
static LowPassFilter_t chassis_vy_filter;
static LowPassFilter_t chassis_wz_filter;

void ChassisInit(void)
{
    // 底盘控制频率500Hz，使用alpha系数初始化
    LowPassFilter_Init(&chassis_vx_filter, 0.5f);
    LowPassFilter_Init(&chassis_vy_filter, 0.5f);
    LowPassFilter_Init(&chassis_wz_filter, 0.5f);
}

void ChassisTask(void)
{
    // 获取原始速度指令
    float raw_vx = GetTargetVx();
    float raw_vy = GetTargetVy();
    float raw_wz = GetTargetWz();
    
    // 滤波处理
    float smooth_vx = LowPassFilter_Update(&chassis_vx_filter, raw_vx);
    float smooth_vy = LowPassFilter_Update(&chassis_vy_filter, raw_vy);
    float smooth_wz = LowPassFilter_Update(&chassis_wz_filter, raw_wz);
    
    // 使用平滑后的速度
    SetChassisSpeed(smooth_vx, smooth_vy, smooth_wz);
}
```

## 其他功能

### 重置滤波器
```c
// 重置为0
LowPassFilter_Reset(&filter, 0.0f);

// 或重置为当前测量值（避免突变）
float current_value = GetCurrentValue();
LowPassFilter_Reset(&filter, current_value);
```

### 动态调整滤波强度
```c
// 根据运动状态调整滤波强度
if (is_fast_moving) {
    LowPassFilter_SetAlpha(&filter, 0.8f);  // 快速响应
} else {
    LowPassFilter_SetAlpha(&filter, 0.2f);  // 强滤波
}
```

### 获取当前输出
```c
float current_output = LowPassFilter_GetOutput(&filter);
```

## Alpha参数选择建议

| 应用场景 | 推荐Alpha | 说明 |
|---------|----------|------|
| 高频噪声传感器 | 0.1 - 0.3 | 强滤波，响应慢 |
| 视觉数据 | 0.3 - 0.5 | 中等滤波 |
| 速度指令 | 0.5 - 0.7 | 轻度滤波 |
| 快速响应场景 | 0.8 - 0.95 | 弱滤波，快速响应 |

## 注意事项

1. **首次使用**：第一次调用 `Update` 时，输出会直接等于输入，避免从0开始的突变
2. **采样频率**：确保滤波器更新频率与数据采样频率一致
3. **相位延迟**：滤波会引入相位延迟，alpha越小延迟越大
4. **多个数据**：每个需要滤波的数据都要创建独立的滤波器实例
