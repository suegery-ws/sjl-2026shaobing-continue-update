#include "message.h"
#include "bsp_can.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_can.h"

static cboard_recv_message_t recv_data;
static cboard_send_message1_t send_data_1;
static cboard_send_message2_t send_data_2;
static DaemonInstance *vision_daemon_instance_1;
static DaemonInstance *vision_daemon_instance_2;
static CANInstance *vision_can_instance_1;
static CANInstance *vision_can_instance_2;

void TongjiVisionSetAltitude(float x, float y, float z, float w)
{
    send_data_2.x = x;
    send_data_2.y = y;
    send_data_2.z = z;
    send_data_2.w = w;
}//这是第二组数据

void TongjiVisionSetFlag(double bullet_speed, Mode mode, ShootMode shoot_mode, double ft_angle)
{
    send_data_1.bullet_speed = bullet_speed;
    send_data_1.mode = mode;
    send_data_1.shoot_mode = shoot_mode;
    send_data_1.ft_angle = ft_angle;
}//这是第一组数据

static void VisionOfflineCallback1(void *id)
{
    LOGWARNING("[vision] vision offline, restart communication.");
}

static void VisionOfflineCallback2(void *id)
{
}

static void DecodeVision_1(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance_1); // 喂狗
    recv_data.control = rxbuff[0];
    recv_data.shoot = rxbuff[1];
    recv_data.yaw = ((int16_t)(rxbuff[2] << 8 | rxbuff[3])) / 1e4;
    recv_data.pitch = ((int16_t)(rxbuff[4] << 8 | rxbuff[5])) / 1e4;
    recv_data.horizon_distance = ((int16_t)(rxbuff[6] << 8 | rxbuff[7])) / 1e4;
    // get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);解包函数
    //TODO: code to resolve flag_register;
    //这边需要改成can的解包函数...
} //有无用待验证

static void DecodeVision_2(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance_2); // 喂狗
    recv_data.control = rxbuff[0];
    recv_data.shoot = rxbuff[1];
    recv_data.yaw = ((int16_t)(rxbuff[2] << 8 | rxbuff[3])) / 1e4;
    recv_data.pitch = ((int16_t)(rxbuff[4] << 8 | rxbuff[5])) / 1e4;
    recv_data.horizon_distance = ((int16_t)(rxbuff[6] << 8 | rxbuff[7])) / 1e4;
    // get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);解包函数
    // TODO: code to resolve flag_register;
    //这边需要改成can的解包函数...
} //有无用待验证

cboard_recv_message_t *TongjiVisionInit(CAN_HandleTypeDef *_handle)
{
    static CAN_Init_Config_s conf1;
    static CAN_Init_Config_s conf2;
    conf1.can_module_callback = DecodeVision_1;//直有这个有用
    conf1.tx_id = CBOARD_RECV_ID_1;
    conf1.rx_id = CBOARD_SEND_ID;//尝试在发送函数上动动手脚
    conf1.can_handle = _handle; //使用CAN1总线
    conf1.id = vision_can_instance_1;
    vision_can_instance_1 = CANRegister(&conf1);

    conf2.can_module_callback = DecodeVision_2;
    conf2.tx_id = CBOARD_RECV_ID_2;
    conf2.rx_id = CBOARD_SEND_ID2;//尝试在发送函数上动动手脚
    conf2.can_handle = _handle; //使用CAN1总线
    conf1.id = vision_can_instance_2;
    vision_can_instance_2 = CANRegister(&conf2);

    Daemon_Init_Config_s daemon_conf_1 = {
        .callback = VisionOfflineCallback1, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_can_instance_1, 
        .reload_count = 10, 
    };

    Daemon_Init_Config_s daemon_conf_2 = {
        .callback = VisionOfflineCallback2, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_can_instance_2, 
        .reload_count = 10, 
    };

    vision_daemon_instance_1 = DaemonRegister(&daemon_conf_1);
    vision_daemon_instance_2 = DaemonRegister(&daemon_conf_2);
 
    return &recv_data; //由此获得数据指针
}

void TongjiVisionSend()
{
    static uint8_t frame_data[8];
    static uint8_t frame_data_1[8];
    static int a=0;
    static int b=0;

    int16_t x_raw = (int16_t)(send_data_2.x * 10000);
    int16_t y_raw = (int16_t)(send_data_2.y * 10000);
    int16_t z_raw = (int16_t)(send_data_2.z * 10000);
    int16_t w_raw = (int16_t)(send_data_2.w * 10000);
    
    // 打包到CAN数据帧
    frame_data[0] = (uint8_t)(x_raw >> 8);   // x高8位
    frame_data[1] = (uint8_t)(x_raw);        // x低8位
    frame_data[2] = (uint8_t)(y_raw >> 8);   // y高8位
    frame_data[3] = (uint8_t)(y_raw);        // y低8位
    frame_data[4] = (uint8_t)(z_raw >> 8);   // z高8位
    frame_data[5] = (uint8_t)(z_raw);        // z低8位
    frame_data[6] = (uint8_t)(w_raw >> 8);   // w高8位
    frame_data[7] = (uint8_t)(w_raw);        // w低8位
    
    vision_can_instance_2->tx_buff[0] = frame_data[0];
    vision_can_instance_2->tx_buff[1] = frame_data[1];
    vision_can_instance_2->tx_buff[2] = frame_data[2];
    vision_can_instance_2->tx_buff[3] = frame_data[3];
    vision_can_instance_2->tx_buff[4] = frame_data[4];
    vision_can_instance_2->tx_buff[5] = frame_data[5];
    vision_can_instance_2->tx_buff[6] = frame_data[6];
    vision_can_instance_2->tx_buff[7] = frame_data[7];
    a = CANTransmit(vision_can_instance_2, 10);//发送四元数
    //////////////////////////////////////////////////////////////////////////////////////////////
    int16_t speed_raw = (int16_t)(send_data_1.bullet_speed * 100);  // 对应 /1e2
    int16_t angle_raw = (int16_t)(send_data_1.ft_angle * 10000);    // 对应 /1e4
    
    //打包到CAN数据帧
    // frame_data_1[0] = (uint8_t)(speed_raw >> 8);   // bullet_speed 高8位
    // frame_data_1[1] = (uint8_t)(speed_raw);        // bullet_speed 低8位
    // frame_data_1[2] = (uint8_t)send_data_1.mode;         // mode
    // frame_data_1[3] = (uint8_t)send_data_1.shoot_mode;   // shoot_mode
    // frame_data_1[4] = (uint8_t)(angle_raw >> 8);   // ft_angle 高8位
    // frame_data_1[5] = (uint8_t)(angle_raw);        // ft_angle 低8位
    // frame_data_1[6] = 0;  // 未使用，填充0
    // frame_data_1[7] = 0;  // 未使用，填充0

    frame_data_1[0] = (uint8_t)0;   // bullet_speed 高8位
    frame_data_1[1] = (uint8_t)1;   // bullet_speed 低8位
    frame_data_1[2] = (uint8_t)2;   // mode
    frame_data_1[3] = (uint8_t)3;   // shoot_mode
    frame_data_1[4] = (uint8_t)4;   // ft_angle 高8位
    frame_data_1[5] = (uint8_t)5;   // ft_angle 低8位
    frame_data_1[6] = (uint8_t)6;   // 未使用，填充0
    frame_data_1[7] = (uint8_t)7;   // 未使用，填充0

    vision_can_instance_1->tx_buff[0] = frame_data_1[0];
    vision_can_instance_1->tx_buff[1] = frame_data_1[1];
    vision_can_instance_1->tx_buff[2] = frame_data_1[2];
    vision_can_instance_1->tx_buff[3] = frame_data_1[3];
    vision_can_instance_1->tx_buff[4] = frame_data_1[4];
    vision_can_instance_1->tx_buff[5] = frame_data_1[5];
    vision_can_instance_1->tx_buff[6] = frame_data_1[6];
    vision_can_instance_1->tx_buff[7] = frame_data_1[7];

    b = CANTransmit(vision_can_instance_1, 10);//发送发射机构相关数据

}

//  // 分组填入发送数据
//         group = motor->sender_group;
//         num = motor->message_num;
//         sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
//         sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

//         // 若该电机处于停止状态,直接将buff置零
//         if (motor->stop_flag == MOTOR_STOP)
//             memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
//     }

//     // 遍历flag,检查是否要发送这一帧报文
//     for (size_t i = 0; i < 6; ++i)
//     {
//         if (sender_enable_flag[i])
//         {
//           CANTransmit(&sender_assignment[i], 1);
//         }
//     }