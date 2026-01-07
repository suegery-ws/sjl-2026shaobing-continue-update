# DM电机位置滤波说明

## 修改内容

为DM电机的位置反馈数据添加了一阶低通滤波器，使位置数据变化更加平稳，减少噪声影响。

## 修改文件

### 1. dmmotor.h
- 添加了 `#include "lowpass_filter.h"` 引用
- 在 `DM_Motor_Measure_s` 结构体中添加了 `LowPassFilter_t position_filter;` 成员

### 2. dmmotor.c

#### 初始化函数 (DMMotorInit)
在电机初始化时添加了滤波器初始化：
```c
// 初始化位置低通滤波器 (CAN接收频率约1000Hz，截止频率设为100Hz)
LowPassFilter_Init_ByFreq(&motor->measure.position_filter, 1000.0f, 100.0f);
```

#### 解码函数 (DMMotorDecode)
修改了位置数据的处理流程：
```c
// 解析原始位置数据
tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
float raw_position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

// 使用低通滤波器对位置进行滤波，使数据变化更平稳
measure->position = LowPassFilter_Update(&measure->position_filter, raw_position);
```

## 滤波器参数

- **采样频率**: 1000Hz (CAN接收频率)
- **截止频率**: 100Hz
- **计算得到的alpha**: 约0.386

这个参数配置意味着：
- 允许100Hz以下的位置变化信号通过
- 抑制100Hz以上的高频噪声
- 响应速度适中，既能平滑数据又不会引入过大延迟

## 效果

1. **平滑性提升**: 位置数据的突变和抖动会被滤除
2. **控制稳定性**: 更平稳的位置反馈有助于PID控制的稳定性
3. **噪声抑制**: 编码器的高频噪声被有效滤除

## 调整建议

如果需要调整滤波强度，可以修改截止频率：

- **更强滤波** (响应慢，更平滑): 降低截止频率，如50Hz
  ```c
  LowPassFilter_Init_ByFreq(&motor->measure.position_filter, 1000.0f, 50.0f);
  ```

- **更弱滤波** (响应快，较平滑): 提高截止频率，如200Hz
  ```c
  LowPassFilter_Init_ByFreq(&motor->measure.position_filter, 1000.0f, 200.0f);
  ```

- **动态调整**: 可以在运行时根据运动状态调整
  ```c
  // 快速运动时使用弱滤波
  LowPassFilter_SetAlpha(&motor->measure.position_filter, 0.7f);
  
  // 精细定位时使用强滤波
  LowPassFilter_SetAlpha(&motor->measure.position_filter, 0.2f);
  ```

## 注意事项

1. 滤波会引入轻微的相位延迟，这在高速运动时可能需要考虑
2. 第一次更新时，滤波器输出会直接等于输入，避免启动突变
3. 每个电机实例都有独立的滤波器，互不影响
