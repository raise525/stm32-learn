/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

#include "ssd1306.h"
#include "i2c.h"        // 需要I2C句柄
#include <stdio.h>
#include <string.h>

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
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END Variables */
/* Definitions for Task_LED */
osThreadId_t Task_LEDHandle;
const osThreadAttr_t Task_LED_attributes = {
  .name = "Task_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Display */
osThreadId_t Task_DisplayHandle;
const osThreadAttr_t Task_Display_attributes = {
  .name = "Task_Display",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Producer */
osThreadId_t ProducerHandle;
const osThreadAttr_t Producer_attributes = {
  .name = "Producer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Consumer */
osThreadId_t ConsumerHandle;
const osThreadAttr_t Consumer_attributes = {
  .name = "Consumer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void *argument);
void StartDisplayTask(void *argument);
void Producer_Task(void *argument);
void Consumer_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (10, sizeof(uint32_t), &dataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_LED */
  Task_LEDHandle = osThreadNew(StartLEDTask, NULL, &Task_LED_attributes);

  /* creation of Task_Display */
  Task_DisplayHandle = osThreadNew(StartDisplayTask, NULL, &Task_Display_attributes);

  /* creation of Producer */
  ProducerHandle = osThreadNew(Producer_Task, NULL, &Producer_attributes);

  /* creation of Consumer */
  ConsumerHandle = osThreadNew(Consumer_Task, NULL, &Consumer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief  Function implementing the Task_LED thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
	for (;;) {
		// 亮100ms
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		//osDelay(100);
		osDelay(1000);

		// 灭900ms（总共1秒周期）
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		//osDelay(900);
		osDelay(2000);
	}
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the Task_Display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	uint32_t counter = 0;
	char buffer[32];

	// 初始化OLED（需要传入I2C句柄）
	OLED_Init(hi2c1);           // 你的驱动需要I2C句柄参数
	OLED_Fill(0x00);            // 清屏（填充0x00）

	// 显示固定内容
	OLED_BOOL_DrawStr(0, 0, (uint8_t*) "FreeRTOS Day3", 0x00);
	OLED_BOOL_DrawStr(0, 16, (uint8_t*) "Queue Test", 0x00);
	OLED_BOOL_DrawStr(0, 32, (uint8_t*) "Producer->Consumer", 0x00);
	OLED_Refresh();  // 初始显示
	// 第48行留给Consumer显示队列数据

	osDelay(1000);              // 等待1秒

	for (;;) {
		counter++;

		// 清屏
		//OLED_Fill(0x00);

//		// 显示计数器
//		sprintf(buffer, "Count: %06lu", counter);
//		OLED_BOOL_DrawStr(0, 0, (uint8_t*) buffer, OLED_BOOL_Replace);
//
//		// 显示状态
//		OLED_BOOL_DrawStr(0, 16, (uint8_t*) "FreeRTOS Running",
//				OLED_BOOL_Replace);
//		OLED_BOOL_DrawStr(0, 32, (uint8_t*) "LED Task Active",
//				OLED_BOOL_Replace);
//		OLED_BOOL_DrawStr(0, 48, (uint8_t*) "Day1 Success!", OLED_BOOL_Replace);

		// 只更新计数器区域，不覆盖整个屏幕
		sprintf(buffer, "Count:%06lu", counter);
		// 在特定位置显示，比如第40行
		OLED_BOOL_DrawStr(0, 40, (uint8_t*) buffer, 0x00);

		// 刷新到屏幕
		OLED_Refresh();

		// 500ms延迟，改成两秒
		osDelay(1000);
	}
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_Producer_Task */
/**
 * @brief Function implementing the Producer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Producer_Task */
void Producer_Task(void *argument)
{
  /* USER CODE BEGIN Producer_Task */
	uint32_t send_data = 0;

	/* Infinite loop */
	for (;;) {
		send_data++;

		// 使用CMSIS-RTOS V2的队列API
		osMessageQueuePut(dataQueueHandle, &send_data, 0, pdMS_TO_TICKS(10));

		osDelay(1000);
	}
  /* USER CODE END Producer_Task */
}

/* USER CODE BEGIN Header_Consumer_Task */
/**
 * @brief Function implementing the Consumer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Consumer_Task */
void Consumer_Task(void *argument)
{
  /* USER CODE BEGIN Consumer_Task */
	uint32_t recv_data = 0;
	char buffer[32];

//	OLED_BOOL_DrawStr(0, 32, (unit8_t*)"Queue: Waiting", 0x00);
//	OLED_Refresh();
	uint32_t count = 0;  // 用于记录收到了多少个数据

	/* Infinite loop */
	for (;;) {
		// 等待队列数据
		if (osMessageQueueGet(dataQueueHandle, &recv_data, NULL, osWaitForever)
				== osOK) {
			count++;

			// 方案A：在OLED固定位置显示（不覆盖原有显示）
			// 在屏幕右下角显示队列数据
			sprintf(buffer, "Q:%lu", recv_data);
			OLED_BOOL_DrawStr(0, 48, (uint8_t*) buffer, 0x00);  // 右下角位置

			// 方案B：用LED闪烁次数表示数据值
			// 收到数据时LED快速闪烁recv_data%3+1次
			for (int i = 0; i < (recv_data % 3) + 1; i++) {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				osDelay(100);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				if (i < (recv_data % 3))
					osDelay(100);  // 闪烁间隔
			}
		}
	}
  /* USER CODE END Consumer_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

