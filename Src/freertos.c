/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
// osThreadId insTaskHandle;
// osThreadId robotTaskHandle;
// osThreadId motorTaskHandle;
// osThreadId daemonTaskHandle;
// osThreadId uiTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];


void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    // insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // // 后续修改为读取传感器数据准备好的中断处理

    // osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    // motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    // osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    // daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    // osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    // robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    // HTMotorControlInit(); // 没有注册HT电机则不会执行
}

  // osThreadDef(instask, StartINSTASK, osPriorityNormal, 0, 1024);
  // defaultTaskHandle = osThreadCreate(osThread(instask), NULL);//陀螺仪

  // osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
  // defaultTaskHandle = osThreadCreate(osThread(motortask), NULL);//电机闭环

  // osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 512);
  // defaultTaskHandle = osThreadCreate(osThread(daemontask), NULL);//看门狗纠错任务

  // osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
  // defaultTaskHandle = osThreadCreate(osThread(robottask), NULL);//底盘云台一堆乱七八糟的任务
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */



/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  osThreadTerminate(NULL); // 避免空置和切换占用cpu
  /* USER CODE END StartDefaultTask */
  // for(;;)
  // {
  //   //led_RGB_flow_task();闪灯任务，以后加
  //   osDelay(1);
  // }
}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// void StartINSTASK(void const * argument)
// {
//   /* USER CODE BEGIN StartINSTASK */
//   while(1)
//   {
//     INS_Task();
//     osDelay(1);
//   }
// }

// void StartMOTORTASK(void const * argument)
// {
//   /* USER CODE BEGIN StartMOTORTASK */
//   while(1)
//   {
//     MotorControlTask();
//     osDelay(2);
//   }
// }//这么写直接把电机的参数设定和报文控制隔离开了

/* USER CODE END Application */
