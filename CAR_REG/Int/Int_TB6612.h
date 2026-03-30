#ifndef __INT_TB6612_H
#define __INT_TB6612_H

#include "stm32f10x.h"
#include "Dri_TIM.h"

#define GO 0
#define BACK 1
#define STOP 2  //刹车

#define AIN1_L GPIOB->ODR &=~ GPIO_ODR_ODR14
#define AIN1_H GPIOB->ODR |= GPIO_ODR_ODR14

#define AIN2_L GPIOB->ODR &=~ GPIO_ODR_ODR15
#define AIN2_H GPIOB->ODR |= GPIO_ODR_ODR15

#define BIN1_L GPIOB->ODR &=~ GPIO_ODR_ODR13
#define BIN1_H GPIOB->ODR |= GPIO_ODR_ODR13

#define BIN2_L GPIOB->ODR &=~ GPIO_ODR_ODR12
#define BIN2_H GPIOB->ODR |= GPIO_ODR_ODR12

void Int_TB6612_Init(void);

void Int_TB6612_SetPWM(int pwma, int pwmb);

#endif


