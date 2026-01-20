/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

/* External USB device handle declaration */
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
static uint8_t usb_fasong_flag = 0;
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    // static uint32_t last_transmit_time = 0;
    // uint32_t current_time = HAL_GetTick();
    
    // 防止过于频繁的发送调用
    // if (current_time - last_transmit_time < 5) {  // 5ms 最小间隔
    //     usb_fasong_flag = USBD_BUSY;
    //     return USBD_BUSY;
    // }
    
    // // 检查缓冲区有效性
    // if (buffer == NULL || len == 0 || len > 64) {  // USB Full Speed 最大包长 64 字节
    //     usb_fasong_flag = USBD_FAIL;
    //     return USBD_FAIL;
    // }
    
    // last_transmit_time = current_time;
    
    // // 检查 USB 设备状态
    // if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    //     usb_fasong_flag = USBD_FAIL;
    //     return USBD_FAIL;
    // }
    
    usb_fasong_flag = CDC_Transmit_FS(buffer, len); // 发送
    // if(usb_fasong_flag == USBD_OK)
    // {
    //     return USBD_OK;
    // }
    // // 如果发送失败，记录错误但不阻塞
    // if (usb_fasong_flag != USBD_OK) {
    //     static uint32_t error_count = 0;
    //     error_count++;
    //     if (error_count % 100 == 0) {  // 每 100 次错误记录一次
    //         LOGWARNING("[USB] Transmit error: %d, count: %d", usb_fasong_flag, error_count);
    //     }
    // }
}
