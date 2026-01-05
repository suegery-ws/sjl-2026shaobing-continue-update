#include "test_uart.h"
#include "usart.h"
#include "bsp_log.h"
#include "cmsis_os.h"

// 测试用的串口句柄
extern UART_HandleTypeDef huart1;

// 测试任务
void TestUARTTask(void const *argument)
{
    uint8_t test_data[] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x55, 0xAA};
    uint32_t count = 0;
    
    // 等待1秒让系统稳定
    osDelay(1000);
    
    while (1) {
        // 每500ms发送一次测试数据
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, test_data, sizeof(test_data), 100);
        
        if (status == HAL_OK) {
            LOGINFO("[UART Test] Send data OK, count: %lu", count++);
        } else {
            LOGERROR("[UART Test] Send failed, status: %d", status);
        }
        
        osDelay(500); // 500ms间隔
    }
}

// 创建测试任务
void TestUARTTaskStart(void)
{
    osThreadDef(test_uart, TestUARTTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(test_uart), NULL);
    LOGINFO("[UART Test] Test task started");
}