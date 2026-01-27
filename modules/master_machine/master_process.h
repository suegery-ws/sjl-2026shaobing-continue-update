#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 35u 
#define VISION_SEND_SIZE 34u
///////////////////////////////////////////
#define VISION_RECV_SIZE_BUBING 32u
#define VISION_SEND_SIZE_BUBING 32u
///////////////////////////////////////////
#define DAOHANG_RECV_SIZE 15u
#define DAOHANG_SEND_SIZE 15u
/////////////////////////////////////////
#define USB_RECV_SIZE 28u
#define USB_SEND_SIZE 42u 

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_18 = 18,
	BIG_AMU_20 = 20,
	SMALL_AMU_25 = 25,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;
#pragma pack()
////////////////////////////////////////////////////////////////////////////////////////

typedef struct//发送数据
{
  float x;
  float y;
  uint8_t key_board;
} RX_DATE_t;

typedef union//接收数据
{
	CTRL Rec;
	uint8_t buf[VISION_RECV_SIZE];
}BUF;

typedef struct//发送比赛状态
{
	 uint16_t game_time;
 uint8_t game_progress;
} GAME_DATE_t;


typedef union
{
//	RX_DATE_t RX_DATE;
	uint8_t rx_date[9];
}TX_DATE;

typedef union      
{
GAME_DATE_t GAME_DATE;  
	uint8_t rx_date[3];  //这里的rx_date是从裁判系统接收的数据
}TX_GAME;

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
CTRL *VisionInit(UART_HandleTypeDef *_handle);

BUBING_CTRL *BubingVisionInit(UART_HandleTypeDef *_handle);

DAOHANG_CTRL *DaohangVisionInit(UART_HandleTypeDef *_handle);

USB_CTRL *USBVisionInit(UART_HandleTypeDef *_handle);
/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

void UsbVisionSend();

void DaohangVisionSend();
/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch,float big_yaw);

void BubingVisionSetAltitude(float yaw, float pitch,float big_yaw);

void DaohangVisionSetAltitude(float yaw, float pitch);

void UsbVsioionSetAltiitude(float yaw, float pitch, float* q, float yaw_vel, float pitch_vel, float bullet_speed);


#endif // !MASTER_PROCESS_H