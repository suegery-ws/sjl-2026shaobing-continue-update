#ifndef  MESSAGE_H
#define  MESSAGE_H

#include "bsp_can.h"
#include "master_process.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
 
#define CBOARD_RECV_SIZE 8 // 接收数据长度
#define CBOARD_SEND_ID 0xff
#define CBOARD_SEND_ID3 0x12 // 发送数据长度
#define CBOARD_SEND_ID2 0xee // 发送数据长度
#define CBOARD_RECV_ID_1 0x510
#define CBOARD_RECV_ID_2 0x002 //注意要和其他模块的id不冲突

typedef enum
{
  idle = 0,
  auto_aim,
  small_buff,
  big_buff,
  outpost

}Mode;

typedef enum
{
  left_shoot = 0,
  right_shoot,
  both_shoot

}ShootMode;


typedef __packed struct 
{
  uint8_t control;                // 是否有控制权
  uint8_t shoot;                  // 开火建议     //没有转换成浮点数的需求或者没有正负号的需求，直接uint8_t就行了
  double yaw;                  // 云台自瞄yaw目标值
  double pitch;                // 云台自瞄pitch目标值
  double horizon_distance ; // 无人机专有

}cboard_recv_message_t;

typedef __packed struct 
{
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

}cboard_send_message1_t;

typedef __packed struct 
{
  float x;
  float y;
  float z;
  float w;
  
}cboard_send_message2_t;

void TongjiVisionSetAltitude(float x, float y, float z, float w); //message1

void TongjiVisionSetFlag(double bullet_speed, Mode mode, ShootMode shoot_mode, double ft_angle); //message2

uint16_t TongjiGetMessage();

uint16_t TongjiSendMessage1(uint8_t *tx_buf, uint16_t *tx_len);

uint16_t TongjiSendMessage2(uint8_t *tx_buf, uint16_t *tx_len);

cboard_recv_message_t *TongjiVisionInit(CAN_HandleTypeDef *_handle);

void TongjiVisionSend();

static void VisionOfflineCallback2(void *id);

#endif