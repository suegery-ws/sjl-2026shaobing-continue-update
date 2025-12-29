#ifndef DMIMU_H
#define DMIMU_H
#include "bsp_can.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_can.h"
#include "daemon.h"
#include "seasky_protocol.h"
#include "bsp_log.h"


#define DM_IMU_RX_ID 0x11
#define DM_IMU_TX_ID 0x01
#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)


typedef struct
{
    float q[4];

}quar_t;

typedef struct
{
   float x_aceel;
   float y_aceel;
   float z_accel;

}accel_t;

typedef struct
{
   float x_gyro;
   float y_gyro;
   float z_gyro;

}gyro_t;

typedef struct
{
   float roll;
   float pitch;
   float yaw;

}oula_t;

typedef struct
{
    accel_t accel_data;
    gyro_t gyro_data;
    oula_t oula_data;
    quar_t quar_data;

}dm_imu_data_t;

void ImuTask_Function(void);
dm_imu_data_t *DmimuInit(CAN_HandleTypeDef *_handle);



#endif