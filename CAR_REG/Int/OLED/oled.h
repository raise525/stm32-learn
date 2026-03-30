#ifndef __OLED_H
#define __OLED_H 


#include "stdlib.h"	
#include "stm32f10x.h"

//-----------------OLED�˿ڶ���---------------- 

#define OLED_SCL_Clr() SCK_LOW//SCL
#define OLED_SCL_Set() SCK_HIGH

#define OLED_SDA_Clr() MOSI_LOW//SDA
#define OLED_SDA_Set() MOSI_HIGH

#define OLED_RES_Clr() (GPIOB->ODR &=~ GPIO_ODR_ODR0)//RES
#define OLED_RES_Set() (GPIOB->ODR |= GPIO_ODR_ODR0)

#define OLED_DC_Clr()  (GPIOB->ODR &=~ GPIO_ODR_ODR1)//DC
#define OLED_DC_Set()  (GPIOB->ODR |= GPIO_ODR_ODR1)
 		     
#define OLED_CS_Clr()  CS_LOW//CS
#define OLED_CS_Set()  CS_HIGH


#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);

#endif

