/*
 * data_fusion.h
 *
 *  Created on: Feb 2, 2026
 *      Author: Raise
 */

#ifndef INC_DATA_FUSION_H_
#define INC_DATA_FUSION_H_

/* 包含必要的头文件 */
#include "cmsis_os.h"       // 提供 osMessageQueueId_t 等类型定义
#include "sensor_data.h"    // 提供 sensor_data_t 结构体定义

/* 外部声明 - 必须与 freertos.c 中 CubeMX 生成的变量名和类型完全一致 */
extern osMessageQueueId_t DHT11DataQueueHandle;
extern osMessageQueueId_t MPU6050DataQueueHandle;
extern osMessageQueueId_t FusedDataQueueHandle;

/* 数据融合任务函数声明 */
void DataFusion_Task(void *argument);

#endif /* INC_DATA_FUSION_H_ */
