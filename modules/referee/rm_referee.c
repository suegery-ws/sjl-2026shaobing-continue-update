/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "referee_parser.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define REFEREE_TX_FRAME_MIN_LEN (LEN_HEADER + LEN_CMDID + LEN_TAIL)

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据
static RefereeParsedInfo_t referee_parsed_info; // 解析后的裁判系统数据

/* 0x0301 + 0x0120 哨兵自主决策发送帧 */
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	sentry_cmd_t sentry_cmd;
	uint16_t frametail;
} __packed sentry_decision_send_frame_t;

/* 统一底层发送，保持历史 115ms 的发送节奏限制 */
static void RefereeSendRaw(uint8_t *send, uint16_t tx_len)
{
	if (send == NULL || tx_len == 0u || referee_usart_instance == NULL)
		return;

	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}

/* 发送者 ID 优先使用已确定的机器人 ID，未确定时回退到当前 robot_id */
static uint16_t RefereeGetSenderRobotID(void)
{
	if (referee_info.referee_id.Robot_ID != 0u)
		return referee_info.referee_id.Robot_ID;

	return referee_info.GameRobotState.robot_id;
}

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL; //buff[1]是因为一个字节就可以完成数据读取，所有长度都在0-255之内
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_GAME_STATE);
					break;
				case ID_game_result: // 0x0002
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_GAME_RESULT);
					break;
				case ID_game_robot_survivors: // 0x0003
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_GAME_ROBOT_HP);
					break;
				case ID_event_data: // 0x0101
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_EVENT_DATA);
					break;
				case ID_REFEREE_WARNING: // 0x0104
					memcpy(&referee_info.referee_warning, (buff + DATA_Offset), LEN_REFEREE_WARNING);
					break;
				case ID_DART_INFO: // 0x0105
					memcpy(&referee_info.dart_info, (buff + DATA_Offset), LEN_DART_INFO);
					break;
				case ID_GAME_ROBOT_STATE: // 0x0201
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_GAME_ROBOT_STATE);
					break;
				case ID_GAME_ROBOT_POS: // 0x0203
					memcpy(&referee_info.robot_pos, (buff + DATA_Offset), LEN_GAME_ROBOT_POS);
					break;
				case ID_BUFF_MUSK: // 0x0204
					memcpy(&referee_info.buff, (buff + DATA_Offset), LEN_BUFF_MUSK);
					break;
				case ID_ROBOT_HURT: // 0x0206
					memcpy(&referee_info.ext_robot_hurt, (buff + DATA_Offset), LEN_ROBOT_HURT);
					break;
				case ID_SHOOT_DATA: // 0x0207
					memcpy(&referee_info.ext_shoot_data, (buff + DATA_Offset), LEN_SHOOT_DATA);
					break;
				case ID_PROJECTILE_ALLOWANCE: // 0x0208
					memcpy(&referee_info.projectile_allowance, (buff + DATA_Offset), LEN_PROJECTILE_ALLOWANCE);
					break;
				case ID_RFID_STATUS: // 0x0209
					memcpy(&referee_info.rfid_status, (buff + DATA_Offset), LEN_RFID_STATUS);
					break;
				case ID_DART_LAUNCHING_STATE: // 0x020A
					memcpy(&referee_info.dart_client_cmd, (buff + DATA_Offset), LEN_DART_LAUNCHING_STATE);
					break;
				case ID_DART_CLIENT_CMD: // 0x020B
					memcpy(&referee_info.ground_robot_position, (buff + DATA_Offset), LEN_DART_CLIENT_CMD);
					break;
				case ID_SPECIAL_IDENTIFICATION: // 0x020C
					memcpy(&referee_info.radar_mark_data, (buff + DATA_Offset), LEN_SPECIAL_IDENTIFICATION);
					break;
				case ID_SENTRY_INFO: // 0x020D
					memcpy(&referee_info.sentry_info, (buff + DATA_Offset), LEN_SENTRY_INFO);
					break;
				case ID_RADAR_INFO: // 0x020E
					memcpy(&referee_info.radar_info, (buff + DATA_Offset), LEN_RADAR_INFO);
					break;
				case ID_ROBOT_INTERACTION: // 0x0301
				{
					/* 0x0301 数据段长度是可变的，不能按固定 127 字节拷贝。
					 * 按当前帧 DataLength 拷贝，并以结构体大小为上限，避免越界。
					 */
					uint16_t interaction_len = referee_info.FrameHeader.DataLength;
					if (interaction_len > sizeof(referee_info.robot_interaction_data))
					{
						interaction_len = sizeof(referee_info.robot_interaction_data);
					}
					memset(&referee_info.robot_interaction_data, 0, sizeof(referee_info.robot_interaction_data));
					memcpy(&referee_info.robot_interaction_data, (buff + DATA_Offset), interaction_len);
				}
					// 解析0x0301的子ID (data_cmd_id在交互数据的前2个字节)
					{
						uint16_t data_cmd_id = (buff[DATA_Offset + 1] << 8) | buff[DATA_Offset];
						switch (data_cmd_id)
						{
						case UI_Data_ID_Del: // 0x0100 删除图层
							// 数据已经在robot_interaction_data中，可以根据需要进一步处理
							break;
						case UI_Data_ID_Draw1: // 0x0101 绘制1个图形
							// 数据已经在robot_interaction_data中
							break;
						case UI_Data_ID_Draw2: // 0x0102 绘制2个图形
							// 数据已经在robot_interaction_data中
							break;
						case UI_Data_ID_Draw5: // 0x0103 绘制5个图形
							// 数据已经在robot_interaction_data中
							break;
						case UI_Data_ID_Draw7: // 0x0104 绘制7个图形
							// 数据已经在robot_interaction_data中
							break;
						case UI_Data_ID_DrawChar: // 0x0110 绘制字符
							// 数据已经在robot_interaction_data中
							break;
						case Sentinel_Autonomous_decision_making: // 0x0120 哨兵自主决策
							// 数据已经在robot_interaction_data中
							// user_data[0-3]存储哨兵决策命令 (4字节)
							memcpy(&referee_info.sentry_cmd, &referee_info.robot_interaction_data.user_data[0], sizeof(sentry_cmd_t));
							break;
						case Radar_autonomous_decision_making: // 0x0121 雷达自主决策
							// 数据已经在robot_interaction_data中
							// user_data[0-7]存储雷达决策命令 (8字节: 1+1+6)
							memcpy(&referee_info.radar_cmd, &referee_info.robot_interaction_data.user_data[0], sizeof(radar_cmd_t));
							break;
						default:
							// 0x0200-0x02FF 为用户自定义交互数据
							// 数据已经完整存储在robot_interaction_data中
							// 用户可以根据data_cmd_id自行解析user_data内容
							break;
						}
					}
					break;
				case ID_MAP_COMMAND: // 0x0303
					memcpy(&referee_info.map_command, (buff + DATA_Offset), LEN_MAP_COMMAND);
					break;
				case ID_REMOTE_CONTROL: // 0x0304
					memcpy(&referee_info.remote_control, (buff + DATA_Offset), LEN_REMOTE_CONTROL);
					break;
				case ID_MAP_ROBOT_DATA: // 0x0305
					memcpy(&referee_info.map_robot_data, (buff + DATA_Offset), LEN_MAP_ROBOT_DATA);
					break;
				case ID_MAP_PATH_DATA: // 0x0307
					memcpy(&referee_info.map_data, (buff + DATA_Offset), LEN_MAP_PATH_DATA);
					break;
				case ID_CUSTOM_INFO: // 0x0308
					memcpy(&referee_info.custom_info, (buff + DATA_Offset), LEN_CUSTOM_INFO);
					break;
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
	ParseRefereeInfo(&referee_info, &referee_parsed_info);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	if (send == NULL || tx_len == 0u)
		return;

	if (tx_len >= REFEREE_TX_FRAME_MIN_LEN && send[SOF] == REFEREE_SOF)
	{
		RefereeSendRaw(send, tx_len);
		return;
	}

	if (tx_len == sizeof(sentry_cmd_t))
	{
		sentry_cmd_t sentry_cmd;
		memcpy(&sentry_cmd, send, sizeof(sentry_cmd));
		RefereeSendSentryDecision(sentry_cmd.sentry_cmd);
		return;
	}

	/* 其它场景保持历史行为：调用方自行组包，底层透传 */
	RefereeSendRaw(send, tx_len);
}//这个函数只需要把上位机配置好的数据发送给裁判系统即可，可以在串口中断里面加一个这个函数

void RefereeSendSentryDecision(uint32_t sentry_cmd_bits)
{
	sentry_decision_send_frame_t tx_frame;
	uint16_t sender_id = RefereeGetSenderRobotID();

	memset(&tx_frame, 0, sizeof(tx_frame));

	/* 帧头：DataLength = 6(交互头) + 4(哨兵命令) */
	tx_frame.FrameHeader.SOF = REFEREE_SOF;
	tx_frame.FrameHeader.DataLength = Interactive_Data_LEN_Head + Sentinel_Autonomous_decision_making_len;
	tx_frame.FrameHeader.Seq = UI_Seq++;
	tx_frame.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&tx_frame, LEN_CRC8, 0xFF);

	/* cmd_id + 交互头 */
	tx_frame.CmdID = ID_ROBOT_INTERACTION;
	tx_frame.data_cmd_id = Sentinel_Autonomous_decision_making;
	tx_frame.sender_id = sender_id;
	tx_frame.receiver_id = REFEREE_SERVER_ID; // 协议附录指定服务器 ID 为 0x8080

	/* 业务数据 */
	tx_frame.sentry_cmd.sentry_cmd = sentry_cmd_bits;

	/* 帧尾 CRC16（不包含 frametail 自身 2 字节） */
	tx_frame.frametail = Get_CRC16_Check_Sum((uint8_t *)&tx_frame,
											 LEN_HEADER + LEN_CMDID + tx_frame.FrameHeader.DataLength,
											 0xFFFF);

	RefereeSendRaw((uint8_t *)&tx_frame, sizeof(tx_frame));
} //测试时可以直接使用这个函数
