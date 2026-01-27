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
#include "dht11.h"

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
extern osMessageQueueId_t sensorQueueHandle;

/* USER CODE END Variables */
/* Definitions for Task_LED */
osThreadId_t Task_LEDHandle;
const osThreadAttr_t Task_LED_attributes = { .name = "Task_LED", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for Task_Display */
osThreadId_t Task_DisplayHandle;
const osThreadAttr_t Task_Display_attributes = { .name = "Task_Display",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for Sensor_Task1 */
osThreadId_t Sensor_Task1Handle;
const osThreadAttr_t Sensor_Task1_attributes = { .name = "Sensor_Task1",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = { .name = "sensorQueue" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void *argument);
void StartDisplayTask(void *argument);
void Sensor_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	// 初始化LED为灭
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

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
	/* creation of sensorQueue */
	sensorQueueHandle = osMessageQueueNew(5, 8, &sensorQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Task_LED */
	Task_LEDHandle = osThreadNew(StartLEDTask, NULL, &Task_LED_attributes);

	/* creation of Task_Display */
	Task_DisplayHandle = osThreadNew(StartDisplayTask, NULL,
			&Task_Display_attributes);

	/* creation of Sensor_Task1 */
	Sensor_Task1Handle = osThreadNew(Sensor_Task, NULL,
			&Sensor_Task1_attributes);

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
void StartLEDTask(void *argument) {
	/* USER CODE BEGIN StartLEDTask */

	// 启动后先亮1秒表示系统启动
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	osDelay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	osDelay(1000);

	for (;;) {
		// 系统心跳：每3秒闪一次（亮200ms）
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(200);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		osDelay(2800);
	}

//	for (;;) {
//		// 亮1s
//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//
//		osDelay(1000);
//
//		// 灭2s（总共3秒周期）
//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//		osDelay(2000);
//	}
	/* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the Task_Display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument) {
	/* USER CODE BEGIN StartDisplayTask */
	DHT11_Data_t sensor_data;
	char buffer[32];
	uint32_t display_counter = 0;
	//uint32_t data_count = 0;

// 等待系统稳定
	osDelay(800);

	// 初始化OLED
	OLED_Init(hi2c1);
	osDelay(50);  // 等待初始化完成

	// 清屏
	OLED_Fill(0x00);
	osDelay(50);

	// 显示固定标题
	OLED_BOOL_DrawStr(0, 0, (uint8_t*) "FreeRTOS Day4", 0x00);
	OLED_BOOL_DrawStr(0, 16, (uint8_t*) "DHT11 Online", 0x00);

	// 初始化显示
	OLED_BOOL_DrawStr(0, 32, (uint8_t*) "Run:000000", 0x00);

	// 初始化温湿度显示区域（显示"--"表示等待数据）
	OLED_BOOL_DrawStr(0, 48, (uint8_t*) "T:--C", 0x00);
	OLED_BOOL_DrawStr(64, 48, (uint8_t*) "H:--%", 0x00);
	OLED_Refresh();

	for (;;) {

		display_counter++;

		// 1. 更新运行计数器
		// 方案A：先写空格清除，再写新内容（简单可靠）
		OLED_BOOL_DrawStr(24, 32, (uint8_t*) "      ", 0x00); // 清除"Run:"后面的6位数字
		sprintf(buffer, "%06lu", display_counter);
		OLED_BOOL_DrawStr(24, 32, (uint8_t*) buffer, 0x00);

		// 2. 尝试获取传感器数据
		if (osMessageQueueGet(sensorQueueHandle, &sensor_data, NULL, 0)
				== osOK) {
			// 显示温度（先清除旧温度数字）
			OLED_BOOL_DrawStr(8, 48, (uint8_t*) "  ", 0x00);  // 清除温度数字（2位）
			sprintf(buffer, "%2d", (int) sensor_data.temperature);
			OLED_BOOL_DrawStr(8, 48, (uint8_t*) buffer, 0x00);

			// 显示湿度（先清除旧湿度数字）
			OLED_BOOL_DrawStr(72, 48, (uint8_t*) "  ", 0x00);  // 清除湿度数字（2位）
			sprintf(buffer, "%2d", sensor_data.humidity);
			OLED_BOOL_DrawStr(72, 48, (uint8_t*) buffer, 0x00);

			// 可选：显示数据计数
			// OLED_BOOL_DrawStr(64, 32, (uint8_t*)"     ", 0x00);
			// sprintf(buffer, "D:%03lu", data_count);
			// OLED_BOOL_DrawStr(64, 32, (uint8_t*)buffer, 0x00);
		} else {
			// 无新数据时显示"--"
			OLED_BOOL_DrawStr(8, 48, (uint8_t*) "--", 0x00);
			OLED_BOOL_DrawStr(72, 48, (uint8_t*) "--", 0x00);

		}

		OLED_Refresh();
		osDelay(1000);
	}

	/* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_Sensor_Task */
/**
 * @brief Function implementing the Sensor_Task1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Sensor_Task */
void Sensor_Task(void *argument) {
	/* USER CODE BEGIN Sensor_Task */

	DHT11_Data_t sensor_data;
	uint8_t read_status;
	uint32_t read_count = 0;
	uint32_t success_count = 0;
	uint32_t fail_count = 0;

	// 初始化DHT11
	if (DHT11_Init() != 0) {
		// 初始化失败：快速闪烁LED然后任务结束
		printf("[DHT11] Initialization failed!\n");
		for (int i = 0; i < 10; i++) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			osDelay(100);
		}
		osThreadTerminate(osThreadGetId());
		return;
	}

	// 初始化成功：慢闪3次
	printf("[DHT11] Initialization successful\n");
	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(300);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		osDelay(300);
	}

	for (;;) {
		// 读取DHT11
		read_status = DHT11_Read(&sensor_data);
		read_count++;

		if (read_status == 0) {
			// 读取成功
			success_count++;
			// 发送到队列
			osMessageQueuePut(sensorQueueHandle, &sensor_data, 0, 0);

			// 第3天核心：输出到串口
			uint32_t current_time = HAL_GetTick();
			printf(
					"[%6lu] #%4lu | Temp: %5.1fC | Humi: %3d%% | OK:%3lu | Fail:%3lu\n",
					current_time, read_count, sensor_data.temperature,  // float
					sensor_data.humidity,         // uint8_t
					success_count, fail_count);
		} else {
			// 读取失败
			fail_count++;
			uint32_t current_time = HAL_GetTick();
			printf(
					"[%6lu] #%4lu | DHT11 read failed (error: %d) | OK:%3lu | Fail:%3lu\n",
					current_time, read_count, read_status, success_count,
					fail_count);
		}

		// DHT11需要至少1秒间隔
		osDelay(2000);
	}

	/* USER CODE END Sensor_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

