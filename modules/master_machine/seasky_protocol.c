/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief 湖南大学RoBoMatster串口通信协议
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "master_process.h"
#include "memory.h"
#include <stdint.h>



void memory_from_buffer(uint8_t *buffer, CTRL *ctrl)
{
	//////////////////////////////////////////////////////////////////
    ctrl->frame_header = buffer[0];
	//需要的部分
    memcpy(&ctrl->x, &buffer[1], 4);
    memcpy(&ctrl->y, &buffer[1+1 * 4], 4);
    memcpy(&ctrl->distance, &buffer[1+2 * 4], 4);
	memcpy(&ctrl->shoot_mode, &buffer[1+3*4], 4);
	memcpy(&ctrl->ahead, &buffer[1+4*4], 4);
	memcpy(&ctrl->ahead_y, &buffer[1+5*4], 4);
	memcpy(&ctrl->angle, &buffer[1+6*4], 4);
	memcpy(&ctrl->mode, &buffer[1+7*4], 4);
	memcpy(&ctrl->blank, &buffer[1+7*4], 4);
	memcpy(&ctrl->frame_tail, &buffer[1+7*4+1], 4);
	///////////////////////////////////////////////////////////////////
}

/**
 * @brief 解包BUBING_CTRL类型数据
 * @param buffer 接收到的原始数据缓冲区
 * @param ctrl 解包后的BUBING_CTRL结构体指针
 * @note 数据帧格式(32字节):
 *       [0] 帧头 FRAME_HEADER (1 byte)
 *       [1] 开火建议 fire_advice (1 byte)
 *       [2] 是否小陀螺 is_spining (1 byte)
 *       [3] 是否导航 is_navigating (1 byte)
 *       [4-7] 俯仰角 pitch (4 bytes, float)
 *       [8-11] 偏航角 yaw (4 bytes, float)
 *       [12-15] 距离 distance (4 bytes, float)
 *       [16-19] 线速度X linearx (4 bytes, float)
 *       [20-23] 线速度Y linery (4 bytes, float)
 *       [24-27] 角速度Z angularz (4 bytes, float)
 *       [28-29] 空白 blank (2 bytes)
 *       [30] 校验字节 check_byte (1 byte)
 *       [31] 帧尾 frame_tail (1 byte)
 */
void bubing_memory_from_buffer(uint8_t *buffer, BUBING_CTRL *ctrl)
{
    uint8_t index = 0;
    
    // 帧头 (1 byte)
    ctrl->FRAME_HEADER = buffer[index++];
    
    // 开火建议 (1 byte)
    ctrl->fire_advice = buffer[index++];
    
    // 是否小陀螺 (1 byte)
    ctrl->is_spining = buffer[index++];
    
    // 是否导航 (1 byte)
    ctrl->is_navigating = buffer[index++];
    
    // 俯仰角 pitch (4 bytes)
    memcpy(&ctrl->pitch, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 偏航角 yaw (4 bytes)
    memcpy(&ctrl->yaw, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 距离 distance (4 bytes)
    memcpy(&ctrl->distance, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 线速度X linearx (4 bytes)
    memcpy(&ctrl->linearx, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 线速度Y linery (4 bytes)
    memcpy(&ctrl->linery, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 角速度Z angularz (4 bytes)
    memcpy(&ctrl->angularz, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 空白 blank (2 bytes)
    memcpy(&ctrl->blank, &buffer[index], sizeof(uint16_t));
    index += sizeof(uint16_t);
    
    // 校验字节 check_byte (1 byte)
    ctrl->check_byte = buffer[index++];
    
    // 帧尾 frame_tail (1 byte)
    ctrl->frame_tail = buffer[index++];
}

void daohang_memory_from_buffer(uint8_t *buffer, DAOHANG_CTRL *ctrl)
{
    uint8_t index = 0;
    
    // 帧头 (1 byte)
    ctrl->FRAME_HEADER = buffer[index++];
    
    // 线速度X linearx (4 bytes)
    memcpy(&ctrl->linearx, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 线速度Y linery (4 bytes)
    memcpy(&ctrl->linery, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 角速度Z angularz (4 bytes)
    memcpy(&ctrl->angularz, &buffer[index], sizeof(float));
    index += sizeof(float);
    
    // 校验字节 check_byte (1 byte)
    ctrl->check_byte = buffer[index++];
    
    // 帧尾 frame_tail (1 byte)
    ctrl->frame_tail = buffer[index++];
}

/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(uint8_t *rx_buf)
{
    // 检查帧头是否为PROTOCOL_CMD_ID (0xFF)
    if (rx_buf[0] == SEND_CMD_BUBING)
    {
        return 1;
    }
    
    // 帧头校验通过
    return 0;
}

static uint8_t protocol_tail_Check(uint8_t length,  uint8_t *rx_buf)
{
    // 检查帧头是否为PROTOCOL_CMD_ID (0xFF) 
    if (rx_buf[length - 1] == rece_cmd_bubing)
    {
        return 1;
    }
    
    // 帧头校验通过
    return 0;
}
/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void bubing_get_protocol_send_data(BUBING_AUTO_SEND_TO_NUC_DATA_t *send_data,
                            uint8_t *tx_buf)     // 待发送的数据帧
{
     uint8_t index = 0;
    
    // 帧头 (1 byte)
    tx_buf[index++] = SEND_CMD_BUBING;
    
    // mode (1 byte)
    tx_buf[index++] = send_data->mode;
    
    // pitch (4 bytes)
    memcpy(&tx_buf[index], &send_data->pitch, sizeof(float));
    index += sizeof(float);
    
    // yaw (4 bytes)
    memcpy(&tx_buf[index], &send_data->yaw, sizeof(float));
    index += sizeof(float);
    
    // chassis_yaw (4 bytes)
    memcpy(&tx_buf[index], &send_data->chassis_yaw, sizeof(float));
    index += sizeof(float);
    
    // sentry_hp (2 bytes)
    memcpy(&tx_buf[index], &send_data->sentry_hp, sizeof(uint16_t));
    index += sizeof(uint16_t);
    
    // remaining_time (4 bytes)
    memcpy(&tx_buf[index], &send_data->remaining_time, sizeof(uint32_t));
    index += sizeof(uint32_t);
    
    // self_outpost_HP (2 bytes)
    memcpy(&tx_buf[index], &send_data->self_outpost_HP, sizeof(uint16_t));
    index += sizeof(uint16_t);
    
    // a (1 byte)
    tx_buf[index++] = send_data->a;
    
    // b (1 byte)
    tx_buf[index++] = send_data->b;
    
    // c (1 byte)
    tx_buf[index++] = send_data->c;
    
    // state (1 byte)
    tx_buf[index++] = send_data->state;
    
    // blank (4 bytes)
    memcpy(&tx_buf[index], &send_data->blank, sizeof(uint32_t));
    index += sizeof(uint32_t);
    
    // crc_check (1 byte) - 计算CRC8校验
    // tx_buf[index++] = crc_8(tx_buf, index);
    tx_buf[index++] = 0;
    
    // 帧尾 (1 byte)
    tx_buf[index++] = rece_cmd_bubing;
    
}

void daohang_get_protocol_send_data(DAOHANG_AUTO_SEND_TO_NUC_DATA_t *send_data,
                            uint8_t *tx_buf)    // 待发送的数据帧
{
    uint8_t index = 0;
    
    // 帧头 (1 byte)
    tx_buf[index++] = PROTOCOL_CMD_ID;

    // chassis_yaw (4 bytes)
    memcpy(&tx_buf[index], &send_data->roll, sizeof(float));
    index += sizeof(float);
    
    // pitch (4 bytes)
    memcpy(&tx_buf[index], &send_data->pitch, sizeof(float));
    index += sizeof(float);
    
    // yaw (4 bytes)
    memcpy(&tx_buf[index], &send_data->yaw, sizeof(float));
    index += sizeof(float);
    
    // crc_check (1 byte) - 计算CRC8校验
    // tx_buf[index++] = crc_8(tx_buf, index);
    tx_buf[index++] = 0;
    
    // 帧尾 (1 byte)
    tx_buf[index++] = FRAME_TAIL;
}

/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                           CTRL *rx_data)         // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(rx_buf))
    {
        // date_length = OFFSET_BYTE + pro.header.data_length;
        // if (CRC8_Check_Sum(&rx_buf[0], 34))//大小为35
        {
            // memcpy(rx_data, rx_buf + 8, pro.header.data_length - 2);
            memory_from_buffer(rx_buf,rx_data);
            return 1;
        }
    }
    return 0;
}

uint16_t get_protocol_info_bubing(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                                  BUBING_CTRL *rx_data)         // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    // if (protocol_heade_Check( rx_buf) )
    if (protocol_heade_Check(rx_buf) && protocol_tail_Check(32,  rx_buf) )
    {
        {
            bubing_memory_from_buffer(rx_buf,rx_data);
            return 1;
        }
    }
    return 0;
}


uint16_t get_protocol_info_daohang(uint8_t *rx_buf,          // 接收到的原始数据 // 接收数据的16位寄存器地址
                                  DAOHANG_CTRL *rx_data)
{
    if (protocol_heade_Check(rx_buf) && protocol_tail_Check(15,  rx_buf) )
    {
        {
            daohang_memory_from_buffer(rx_buf,rx_data);
            return 1;
        }
    }
    return 0;
}



 