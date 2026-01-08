/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static CTRL recv_data;
static BUBING_CTRL bubing_recv_data;
static AUTO_SEND_TO_NUC_DATA_t send_data;
static BUBING_AUTO_SEND_TO_NUC_DATA_t bubing_send_data;
static DaemonInstance *vision_daemon_instance;
static USARTInstance *vision_usart_instance;
static int uart_flag = 0;
static int fsong = 0;

//数据耦合性最低的写法
void VisionSetAltitude(float yaw, float pitch,float big_yaw)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.mode = 0;
    send_data.roll = 0;
    send_data.big_yaw = big_yaw;
    send_data.big_pitch = 0;
    send_data.game_progress = 0;
    send_data.remaining_time = 0;
    send_data.sentry_hp = 0;
    send_data.projectile_allowance_17mm = 0;
    send_data.self_support_point = 0;
}

void BubingVisionSetAltitude(float yaw, float pitch,float big_yaw)
{
    bubing_send_data.yaw = yaw;
    bubing_send_data.pitch = pitch;
    bubing_send_data.mode = 0;
    bubing_send_data.a = 0;
    bubing_send_data.b = 0;
    bubing_send_data.c = 0;
    bubing_send_data.blank = 0;
    bubing_send_data.remaining_time = 0;
    bubing_send_data.sentry_hp = 0;
    bubing_send_data.self_outpost_HP = 0;
    bubing_send_data.state = 0;
    bubing_send_data.chassis_yaw = 0;
}
/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
    uart_flag = 0;

}

#ifdef VISION_USE_UART

#include "bsp_usart.h"



/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff,&recv_data);
    // TODO: code to resolve flag_register;

}

static void DecodeVisionbubing()
{
   DaemonReload(vision_daemon_instance); // 喂狗
   
   // 一次性格式化所有数据到字符串缓冲区
   static char hex_str[200]; // 静态缓冲区，避免栈溢出
   int offset = 0;
   
   // 格式化前16字节
   for(int i = 0; i < 16 && i < VISION_RECV_SIZE_BUBING; i++)
   {
       offset += sprintf(hex_str + offset, "%02X ", vision_usart_instance->recv_buff[i]);
   }
   offset += sprintf(hex_str + offset, "\n                           ");
   
   // 格式化后16字节
   for(int i = 16; i < VISION_RECV_SIZE_BUBING; i++)
   {
       offset += sprintf(hex_str + offset, "%02X ", vision_usart_instance->recv_buff[i]);
   }
   
   // 一次性打印所有数据
//    LOGINFO("[Vision] Recv[%d]:\n%s", fsong, hex_str);
   
//    // 打印帧头和帧尾
//    LOGINFO("[Vision] Header:0x%02X Tail:0x%02X", 
//            vision_usart_instance->recv_buff[0], 
//            vision_usart_instance->recv_buff[VISION_RECV_SIZE_BUBING - 1]);
   
   uart_flag = get_protocol_info_bubing(vision_usart_instance->recv_buff,&bubing_recv_data);
   
   // 打印校验结果
   if(uart_flag)
       LOGINFO(" -> PASSED\r\n");
   else
       LOGWARNING(" -> FAILED\r\n");
   
   fsong++;
}

BUBING_CTRL *BubingVisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVisionbubing;
    conf.recv_buff_size = VISION_RECV_SIZE_BUBING;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);
    
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance, 
        .reload_count = 10, 
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf); 
 
    return &bubing_recv_data;
}

CTRL *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);
    
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance, 
        .reload_count = 10, 
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf); 
 
    return &recv_data;
}
//不清楚这里为什么重复定义了
/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t send_buff[VISION_SEND_SIZE_BUBING];
    
    // 将数据转化为seasky协议的数据包
    bubing_get_protocol_send_data(&bubing_send_data, send_buff);
    
    // 使用IT发送,防止和接收DMA冲突
    // 注意：不检查gState，因为接收DMA会让gState一直是BUSY_RX
    // IT发送会自动处理TX忙的情况
    USARTSend(vision_usart_instance, send_buff, 32, USART_TRANSFER_DMA);
}

#endif  //VISION_USE_UART

 #ifdef VISION_USE_VCP

 #include "bsp_usb.h"
 static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
