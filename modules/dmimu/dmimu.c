#include "dmimu.h"
// FreeRTOS task delay API
#include "FreeRTOS.h"
#include "robot_def.h"
#include "task.h"



static dm_imu_data_t  dm_imu_data;
static CANInstance * dm_imu;
static DaemonInstance * dm_imu_instance;
static int abt=0;

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	dm_imu_data.accel_data.x_aceel=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	dm_imu_data.accel_data.y_aceel=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	dm_imu_data.accel_data.z_accel=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

static void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	dm_imu_data.gyro_data.x_gyro=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	dm_imu_data.gyro_data.y_gyro=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	dm_imu_data.gyro_data.z_gyro=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


static void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	dm_imu_data.oula_data.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16)*angle_to_radian;
	dm_imu_data.oula_data.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16)*angle_to_radian;
	dm_imu_data.oula_data.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16)*angle_to_radian;
}


static void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	dm_imu_data.quar_data.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu_data.quar_data.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu_data.quar_data.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu_data.quar_data.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

static void IMU_RequestData(CAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg)
{
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
    dm_imu->tx_buff[0] = cmd[0];
	dm_imu->tx_buff[1] = cmd[1];
    dm_imu->tx_buff[2] = cmd[2];
	dm_imu->tx_buff[3] = cmd[3];

	abt = CANTransmit(dm_imu, 10);
}

static void IMUOfflineCallback(void *id)
{
    LOGWARNING("[DMIMU] dmimu offline, restart communication.");
}

static void DecodeImu(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    uint16_t flag_register;
    DaemonReload(dm_imu_instance); // 喂狗
    switch(rxbuff[0])
	{
		case 1:
			IMU_UpdateAccel(rxbuff);
			break;
		case 2:
			IMU_UpdateGyro(rxbuff);
			break;
		case 3:
			IMU_UpdateEuler(rxbuff);
			break;
		case 4:
			IMU_UpdateQuaternion(rxbuff);
			break;
	}
    
} //有无用待验证

dm_imu_data_t *DmimuInit(CAN_HandleTypeDef *_handle)
{
    static CAN_Init_Config_s conf1;
    conf1.can_module_callback = DecodeImu;
    conf1.rx_id = DM_IMU_RX_ID;
    conf1.can_handle = _handle;
    conf1.id = dm_imu;
    dm_imu = DMCANRegister(&conf1);
	//接下来是修正部分

    Daemon_Init_Config_s daemon_conf_1 = 
    {
        .callback = IMUOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = dm_imu, 
        .reload_count = 10, 
    };
        dm_imu_instance = DaemonRegister(&daemon_conf_1);

        return &dm_imu_data;
};

void ImuTask_Function(void)
{
    
			IMU_RequestData(&hcan2,DM_IMU_TX_ID,1);
			vTaskDelay(1);
			IMU_RequestData(&hcan2,DM_IMU_TX_ID,2);
			vTaskDelay(1);
			IMU_RequestData(&hcan2,DM_IMU_TX_ID,3);
			vTaskDelay(1);
	        IMU_RequestData(&hcan2,DM_IMU_TX_ID,4);
			vTaskDelay(1);
}
