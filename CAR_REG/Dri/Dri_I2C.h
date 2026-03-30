#ifndef __DRI_I2C_H
#define __DRI_I2C_H

#include "stm32f10x.h"

#define ACK 0
#define NACK 1

#define OK 1
#define FAIL 0

void Driver_I2C2_Init(void);

uint8_t Driver_I2C2_Start(void);

void Driver_I2C2_Stop(void);

void Driver_I2C2_Ack(void);

void Driver_I2C2_NAck(void);

uint8_t Driver_I2C_SendAddr(uint8_t addr);

uint8_t Driver_I2C_SendByte(uint8_t byte);

uint8_t Driver_I2C_ReadByte(void);


#endif