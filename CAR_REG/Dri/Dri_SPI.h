#ifndef __DRI_SPI_
#define __DRI_SPI_

#include "stm32f10x.h"

#define CS_HIGH (GPIOA->ODR |= GPIO_ODR_ODR4)
#define CS_LOW (GPIOA->ODR &= ~GPIO_ODR_ODR4)

#define SCK_HIGH (GPIOA->ODR |= GPIO_ODR_ODR5)
#define SCK_LOW (GPIOA->ODR &= ~GPIO_ODR_ODR5)

#define MOSI_HIGH (GPIOA->ODR |= GPIO_ODR_ODR7)
#define MOSI_LOW (GPIOA->ODR &= ~GPIO_ODR_ODR7)


#define SPI_DELAY Delay_us(5)

void Driver_SPI_Init(void);


uint8_t Driver_SPI_SwapByte(uint8_t byte);



#endif