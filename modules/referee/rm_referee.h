#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;
// 裁判系统服务器 ID（协议附录：0x8080 用于哨兵/雷达自主决策指令）
#define REFEREE_SERVER_ID 0x8080u

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	referee_warning_t referee_warning;                     // 0x0104
	dart_info_t dart_info;                                // 0x0105
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	robot_pos_t robot_pos;                                 //
	buff_t buff;                                           //
    ext_robot_hurt_t ext_robot_hurt;
	ext_shoot_data_t ext_shoot_data;
	projectile_allowance_t projectile_allowance;
	rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;
	ground_robot_position_t ground_robot_position;
	radar_mark_data_t radar_mark_data;
	sentry_info_t sentry_info;     
	radar_info_t radar_info;
    robot_interaction_data_t robot_interaction_data;
	sentry_cmd_t sentry_cmd;
	radar_cmd_t radar_cmd;
	// interaction_layer_delete_t interaction_layer_delete;
	map_command_t map_command;
	map_robot_data_t map_robot_data;
	map_data_t map_data;
	custom_info_t custom_info;
	remote_control_t remote_control;

	uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	chassis_mode_e chassis_mode;			 // 底盘模式
	gimbal_mode_e gimbal_mode;				 // 云台模式
	shoot_mode_e shoot_mode;				 // 发射模式设置
	friction_mode_e friction_mode;			 // 摩擦轮关闭
	lid_mode_e lid_mode;					 // 弹舱盖打开
	Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

	// 上一次的模式，用于flag判断
	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	Chassis_Power_Data_s Chassis_last_Power_Data;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

/**
 * @brief 发送哨兵自主决策指令（0x0301/0x0120）到裁判系统服务器
 *
 * @param sentry_cmd_bits sentry_cmd_t.sentry_cmd 的 32bit 命令位
 */
void RefereeSendSentryDecision(uint32_t sentry_cmd_bits);

#endif // !REFEREE_H
