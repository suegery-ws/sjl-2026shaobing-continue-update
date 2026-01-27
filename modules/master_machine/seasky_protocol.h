#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
                                                                                
#define PROTOCOL_CMD_ID 0XFF
#define FRAME_TAIL 0x0D

#define SEND_CMD_BUBING 0xff
#define rece_cmd_bubing 0x0d

////////////////////////////////////////////////////////////上位机给下位机/////////////////////////////////////////////////

typedef __packed struct
{
  uint8_t head;
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint8_t crc8;
  uint8_t tail;
}USB_CTRL; //28

typedef __packed struct
{
	uint8_t FRAME_HEADER; 
	float linearx;
	float linery;
	float angularz;
	uint8_t check_byte;
	uint8_t frame_tail;
}DAOHANG_CTRL; //15

typedef __packed struct
{
	uint8_t FRAME_HEADER; 
	uint8_t fire_advice;
    uint8_t is_spining;
	uint8_t is_navigating;
	float pitch;
	float yaw;
	float distance;
	float linearx;
	float linery;
	float angularz;
	uint16_t blank;
	uint8_t check_byte;
	uint8_t frame_tail;
}BUBING_CTRL; //32

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头                                                                                                          
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验                  
} protocol_rm_struct;

typedef struct
{
	uint8_t frame_header;
  float x; 
  float y;
  float distance; 
	int shoot_mode;

	////////////导航/////////////
  float ahead;
  float ahead_y;
  float angle;
  int mode;
	//////////////////////////
	uint8_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t frame_tail ;         //帧尾
} CTRL;

////////////////////////////////////////////////////////////下位机给上位机/////////////////////////////////////////////////

typedef __packed struct
{
	uint8_t FRAME_HEADER ; //帧头
	uint8_t mode;  //探测的颜色
	float roll;
	float pitch;
	float yaw;
	float big_pitch;
	float big_yaw;
	////////////////////////////////////22

		//////////////裁判/////////////////
	uint8_t game_progress; //比赛状态
	uint16_t remaining_time; //比赛剩余时间
	uint16_t sentry_hp;    //sentry血量self
	//uint8_t able_to_resurrection;  //是否可以免费买活 1可以 0不行
	//uint8_t center_gain_point;  //是否在中心增益点
	uint16_t self_outpost_HP;  //己方前哨战血量
	uint16_t projectile_allowance_17mm; //允许发弹量
	uint8_t self_support_point;  //己方与兑换区不重叠的补给区bool 0不在 1在
	////////////////////////////////////32
	
	uint8_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t frame_tail ;         //帧尾

}AUTO_SEND_TO_NUC_DATA_t;  //34
 
typedef __packed struct
{
	uint8_t FRAME_HEADER ;       //帧头
	uint8_t mode;  //探测的颜色
	float pitch;
	float yaw;
	float chassis_yaw;
	////////////////////////////////////22

		//////////////裁判/////////////////
	uint16_t sentry_hp;    //sentry血量self
	uint32_t remaining_time; //比赛剩余时间
	uint16_t self_outpost_HP;  //己方前哨战血量
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t state;  //己方与兑换区不重叠的补给区bool 0不在 1在
	////////////////////////////////////32
	
	uint32_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t crc_check;
	uint8_t frame_tail ;         //帧尾

}BUBING_AUTO_SEND_TO_NUC_DATA_t;  //32

typedef __packed struct
{
	uint8_t FRAME_HEADER; 
	float roll;
	float pitch;
	float yaw;
	uint8_t check_byte;
	uint8_t frame_tail;
}DAOHANG_AUTO_SEND_TO_NUC_DATA_t; //15


typedef __packed struct
{
	uint8_t head;
    uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];    // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed; //弹速
    uint16_t bullet_count;  // 子弹累计发送次数
    uint8_t crc8;
	uint8_t tail;
}USB_AUTO_SEND_TO_NUC_DATA_t; //42

/*更新发送数据帧，并计算发送数据帧长度*/
void get_usb_protocol_send_data(USB_AUTO_SEND_TO_NUC_DATA_t *send_data,
                            uint8_t *tx_buf);

void bubing_get_protocol_send_data(BUBING_AUTO_SEND_TO_NUC_DATA_t *send_data,
                            uint8_t *tx_buf);     // 待发送的数据帧

void daohang_get_protocol_send_data(DAOHANG_AUTO_SEND_TO_NUC_DATA_t *send_data,
                            uint8_t *tx_buf);     // 待发送的数据帧
/*接收数据处理*/
uint16_t get_usb_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                           USB_CTRL *rx_data);         // 接收的float数据存储地址

uint16_t get_protocol_info_bubing(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                                  BUBING_CTRL *rx_data);         // 接收的float数据存储地址

uint16_t get_protocol_info_daohang(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                                  DAOHANG_CTRL *rx_data);         // 接收的float数据存储地址
						   
#endif
