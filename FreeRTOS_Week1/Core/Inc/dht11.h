#ifndef __DHT11_H
#define __DHT11_H

#include "main.h"
#include "stm32f1xx_hal.h"

// DHT11引脚定义（接在PA1）
#define DHT11_GPIO_PORT  GPIOA
#define DHT11_GPIO_PIN   GPIO_PIN_1

// 温湿度数据结构
typedef struct {
    float temperature;  // 温度（摄氏度）
    uint8_t humidity;   // 湿度（百分比）
} DHT11_Data_t;

// 函数声明
uint8_t DHT11_Init(void);
uint8_t DHT11_Read(DHT11_Data_t* data);

#endif
