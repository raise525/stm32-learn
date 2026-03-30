#include "Int_Encoder.h"


void Int_Encoder_Init()
{
    /*初始化定时器为编码器模式*/
    Dri_TIM2_Init();
    Dri_TIM3_Init();
}


/**
 * 读取编码器模式的计数值，处理成带符号的值
 * timx：要读取的定时器的编号
 * return：读到的带符号的计数值
 */
int Int_Encodr_ReadCounter(uint8_t timx)
{
    int encoder_value = 0;
    switch (timx)
    {
    case 2:
    {
        /* 读取定时器2的计数值，并且转换为带符号类型，只要转化就好了，但是cnt是16位的，你也要转化为16位的，int16_t/short*/
        encoder_value = (short)TIM2->CNT;
        /* 因为单位时间内读取到一个数值是它的速度，为了下次读取不受影响，应清零*/
        TIM2->CNT = 0;
        break;
    }
        

    case 3:
    {
        /* 读取定时器3的计数值，并且转换为带符号类型，只要转化就好了，但是cnt是16位的，你也要转化为16位的，int16_t/short*/
        encoder_value = (short)TIM3->CNT;
        /* 因为单位时间内读取到一个数值是它的速度，为了下次读取不受影响，应清零*/
        TIM3->CNT = 0;
        break;
    }
        
    
    default:
        break;
    }
    /* 由于电机安装时相差180度，后续读取时，要手动同一方向符号*/
    return encoder_value;
}