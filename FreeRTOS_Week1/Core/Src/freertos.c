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
#include "usart.h"  // 添加这行，包含huart1的声明
#include <stdarg.h>  // 需要这个头文件
#include "sensor_data.h"
#include "data_fusion.h"
#include "mpu6050.h"

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
extern UART_HandleTypeDef huart1;  // 添加这行，声明外部变量
MPU6050_t mpu_data;

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
/* Definitions for Sensor_Task1 */
osThreadId_t Sensor_Task1Handle;
const osThreadAttr_t Sensor_Task1_attributes = {
  .name = "Sensor_Task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataFusion */
osThreadId_t DataFusionHandle;
const osThreadAttr_t DataFusion_attributes = {
  .name = "DataFusion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for DHT11DataQueue */
osMessageQueueId_t DHT11DataQueueHandle;
const osMessageQueueAttr_t DHT11DataQueue_attributes = {
  .name = "DHT11DataQueue"
};
/* Definitions for MPU6050DataQueue */
osMessageQueueId_t MPU6050DataQueueHandle;
const osMessageQueueAttr_t MPU6050DataQueue_attributes = {
  .name = "MPU6050DataQueue"
};
/* Definitions for FusedDataQueue */
osMessageQueueId_t FusedDataQueueHandle;
const osMessageQueueAttr_t FusedDataQueue_attributes = {
  .name = "FusedDataQueue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// 声明safe_printf函数
void safe_printf(const char *format, ...);

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void *argument);
void StartDisplayTask(void *argument);
void Sensor_Task(void *argument);
void IMU_Task(void *argument);
void DataFusion_Task(void *argument);

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
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

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
  sensorQueueHandle = osMessageQueueNew (5, 8, &sensorQueue_attributes);

  /* creation of DHT11DataQueue */
  DHT11DataQueueHandle = osMessageQueueNew (5, 48, &DHT11DataQueue_attributes);

  /* creation of MPU6050DataQueue */
  MPU6050DataQueueHandle = osMessageQueueNew (5, 48, &MPU6050DataQueue_attributes);

  /* creation of FusedDataQueue */
  FusedDataQueueHandle = osMessageQueueNew (5, 48, &FusedDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_LED */
  Task_LEDHandle = osThreadNew(StartLEDTask, NULL, &Task_LED_attributes);

  /* creation of Task_Display */
  Task_DisplayHandle = osThreadNew(StartDisplayTask, NULL, &Task_Display_attributes);

  /* creation of Sensor_Task1 */
  Sensor_Task1Handle = osThreadNew(Sensor_Task, NULL, &Sensor_Task1_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_Task, NULL, &IMU_attributes);

  /* creation of DataFusion */
  DataFusionHandle = osThreadNew(DataFusion_Task, NULL, &DataFusion_attributes);

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
void StartDisplayTask(void *argument)
{
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
void Sensor_Task(void *argument)
{
  /* USER CODE BEGIN Sensor_Task */

	DHT11_Data_t sensor_data;
	uint8_t read_status;
	uint32_t read_count = 0;
	uint32_t success_count = 0;
	uint32_t fail_count = 0;

	// 初始化DHT11
	if (DHT11_Init() != 0) {
		// 初始化失败：快速闪烁LED然后任务结束
		//safe_printf("[DHT11] Initialization failed!\n");
		printf("[DHT11] Initialization failed!\n\n");
		for (int i = 0; i < 10; i++) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			osDelay(100);
		}
		osThreadTerminate(osThreadGetId());
		return;
	}

	// 初始化成功：慢闪3次
	safe_printf("[DHT11] Initialization successful\n\n");
	//printf("[DHT11] Initialization successful\n");
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
			safe_printf(
					"[%6lu] #%4lu | Temp: %5.1fC | Humi: %3d%% | OK:%3lu | Fail:%3lu\n\n",
					current_time, read_count, sensor_data.temperature,  // float
					sensor_data.humidity,         // uint8_t
					success_count, fail_count);
//			printf(
//					"[%6lu] #%4lu | Temp: %5.1fC | Humi: %3d%% | OK:%3lu | Fail:%3lu\n",
//					current_time, read_count, sensor_data.temperature,  // float
//					sensor_data.humidity,         // uint8_t
//					success_count, fail_count);
		} else {
			// 读取失败
			fail_count++;
			uint32_t current_time = HAL_GetTick();
			safe_printf(
					"[%6lu] #%4lu | DHT11 read failed (error: %d) | OK:%3lu | Fail:%3lu\n\n",
					current_time, read_count, read_status, success_count,
					fail_count);
//			printf(
//					"[%6lu] #%4lu | DHT11 read failed (error: %d) | OK:%3lu | Fail:%3lu\n",
//					current_time, read_count, read_status, success_count,
//					fail_count);
		}

		// DHT11需要至少1秒间隔
		osDelay(2000);
	}

  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
 * @brief Function implementing the IMU thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMU_Task */
void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */

	printf("[DEBUG] sizeof(sensor_data_t) = %d bytes\r\n", (int)sizeof(sensor_data_t));

	printf("[IMU] MPU6050任务启动...\r\n");
	printf("[IMU] 尝试初始化MPU6050...\r\n");

	/* 初始化MPU6050 */
	uint8_t init_result = MPU6050_Init(&hi2c1);

	if (init_result != 0) {
		printf("[ERROR] MPU6050初始化失败！错误码：%d\r\n", init_result);
		printf("[提示] 请检查：\r\n");
		printf("  1. I2C连接 (SCL-PB6, SDA-PB7)\r\n");
		printf("  2. MPU6050电源 (VCC-3.3V, GND-GND)\r\n");
		printf("  3. AD0引脚 (接地=0x68)\r\n");
		vTaskDelete(NULL);
	}

	printf("[IMU] MPU6050初始化成功！\r\n");
	printf("[IMU] 开始读取传感器数据...\r\n\r\n");

	/* Infinite loop */
	for (;;) {
		/* 读取所有传感器数据 */
		MPU6050_Read_All(&hi2c1, &mpu_data);

//		/* 输出到串口 - 格式化显示 */
//		printf("┌─────────────────────────────┐\r\n");
//		printf("│        MPU6050 数据         │\r\n");
//		printf("├─────────────────────────────┤\r\n");
//
//		// 加速度数据
//		printf("│ 加速度 (g)                  │\r\n");
//		printf("│   X: %+6.3f  Y: %+6.3f      │\r\n", mpu_data.Ax, mpu_data.Ay);
//		printf("│   Z: %+6.3f                 │\r\n", mpu_data.Az);
//
//		// 陀螺仪数据
//		printf("│ 陀螺仪 (°/s)                │\r\n");
//		printf("│   X: %+7.2f  Y: %+7.2f      │\r\n", mpu_data.Gx, mpu_data.Gy);
//		printf("│   Z: %+7.2f                 │\r\n", mpu_data.Gz);
//
//		// 温度
//		printf("│ 温度: %6.1f°C               │\r\n", mpu_data.Temperature);
//
//		// 卡尔曼滤波角度
//		printf("│ 角度 (°)                    │\r\n");
//		printf("│   Roll(X): %+6.2f           │\r\n", mpu_data.KalmanAngleX);
//		printf("│   Pitch(Y): %+6.2f          │\r\n", mpu_data.KalmanAngleY);
//
//		printf("└─────────────────────────────┘\r\n\r\n");

		// 简洁单行格式（只显示关键数据）
		safe_printf(
				"[IMU] Acc: %5.2f %5.2f %5.2f g | Gyro: %6.1f %6.1f %6.1f °/s | Temp: %4.1f°C | Ang: %5.1f %5.1f°\r\n\n",
				mpu_data.Ax, mpu_data.Ay, mpu_data.Az,  // 加速度 XYZ
				mpu_data.Gx, mpu_data.Gy, mpu_data.Gz,  // 陀螺仪 XYZ
				mpu_data.Temperature,                   // 温度
				mpu_data.KalmanAngleX, mpu_data.KalmanAngleY); // 角度

		/* 延迟1秒 */
		vTaskDelay(1000);
	}
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_DataFusion_Task */
/**
* @brief Function implementing the DataFusion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataFusion_Task */
void DataFusion_Task(void *argument)
{
  /* USER CODE BEGIN DataFusion_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DataFusion_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * 这是一个自定义的printf函数，用法和printf完全一样
 */
// 安全打印函数（线程安全）
void safe_printf(const char *format, ...) {

//	const char *format: 格式化字符串（类似printf的第一个参数）
//	...: 可变参数，表示可以接受不定数量的参数
//	char buffer[128];   // 临时缓冲区，存放格式化后的字符串
//	va_list args;       // 可变参数列表对象
//
//	// 尝试获取互斥锁（等待最多100ms）
//	if (osMutexAcquire(uartMutexHandle, 100) == osOK) {
//		// 格式化字符串
//		va_start(args, format);      // 初始化参数列表
//		vsnprintf(buffer, sizeof(buffer), format, args);   // 格式化到buffer
//		va_end(args);               // 清理参数列表
//
//		// 发送到串口
//		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer),
//		HAL_MAX_DELAY);
//
//		// 释放互斥锁
//		osMutexRelease(uartMutexHandle);

	// 这里先做最简单的实现
	char buffer[128];
	va_list args;

	// 获取互斥锁（如果失败就直接返回）
	if (osMutexAcquire(uartMutexHandle, 100) != osOK) {
		return;
	}

	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer),
			HAL_MAX_DELAY);

	osMutexRelease(uartMutexHandle);

}

/* USER CODE END Application */

