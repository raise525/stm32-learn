#include  "Int_TB6612.h"

/**
 * TB6612初始化
 */
void Int_TB6612_Init(void){
    /* 初始化定时器4*/
    Dri_TIM4_Init(); //之前已经写了

    /* 初始化GPIO：PB12-PB15: 推挽输出 mode=11 cnf=00*/
    /*因为之前的pwm已经接好了，但是tb6612还需要四个管脚接stm,查原理图可知是12-15，只需要设置为推挽输出就好了，
    这里其实是因为只需要普通的0/1，只要是普通的gpio就可以，只是刚好那个小车设计师设置成了12-15*/
    //时钟不用写了，因为定时器已经写好了
    GPIOB->CRH |= (GPIO_CRH_MODE12|GPIO_CRH_MODE13|GPIO_CRH_MODE14|GPIO_CRH_MODE15);
    GPIOB->CRH &=~ (GPIO_CRH_CNF12|GPIO_CRH_CNF13|GPIO_CRH_CNF14|GPIO_CRH_CNF15);

}

/**
 * 控制电机A方向
 * direct 期望的转动方向
 */
void Int_TB6612_MatorA(uint8_t direct)
{
    if(direct == GO)
    {
        /*正转： AIN1 = 0， AIN2=1，这几个管脚就接pb，是pb输出给他们，所以直接用odr设置输出就好*/
        //GPIOB->ODR &=~ GPIO_ODR_ODR14; //本来应该怎么写，但是可读性不高所以去定义宏 
        AIN1_L;
        AIN2_H;
    }
    else if(direct == BACK)
    {
        /*反转： AIN1 = 1， AIN2= 0，查电机的手册*/
        AIN1_H;
        AIN2_L;
    }
    else
    {
        /*刹车： AIN1 = 1， AIN2= 1，查电机的手册*/
        AIN1_H;
        AIN2_H;//到这里只是完成A的，然后复制改成B就好了

    }

}

/**
 * 控制电机B方向
 * direct 期望的转动方向
 */
void Int_TB6612_MatorB(uint8_t direct)
{
    if(direct == GO)
    {
        /*正转： BIN1 = 0， BIN2=1，这几个管脚就接pb，是pb输出给他们，所以直接用odr设置输出就好*/
        //GPIOB->ODR &=~ GPIO_ODR_ODR14; //本来应该怎么写，但是可读性不高所以去定义宏 
        BIN1_L;
        BIN2_H;
    }
    else if(direct == BACK)
    {
        /*反转： BIN1 = 1， BIN2= 0，查电机的手册*/
        BIN1_H;
        BIN2_L;
    }
    else
    {
        /*刹车： BIN1 = 1， BIN2= 1，查电机的手册*/
        BIN1_H;
        BIN2_H;

    }

}

/**
 * 根据带符号的pwm值，设置转速和方向
 */
void Int_TB6612_SetPWM(int pwma, int pwmb)
{

    /* 根据带符号的pwm值：1、处理方向； 2、设置pwm占空比*/

    /* 1.处理pwma*/
    /* 1.1 根据符号处理方向*/
    if(pwma > 0)
    {
        /* 要正转*/
        Int_TB6612_MatorA(GO);
    }
    else if(pwma < 0)
    {
        /* 要反转*/
        Int_TB6612_MatorA(BACK);
        /* 为后续设置ccr做准备，取绝对值*/
        pwma = - pwma;
    }
    else
    {
        /* 刹车*/
        Int_TB6612_MatorA(STOP);
    }

    /* 1.2 设置对应的pwm值 -> 设置对应定时器通道CH4的ccr值*/
    TIM4->CCR4 = pwma;

    /* 2.处理pwmb*/
    /* 2.1 根据符号处理方向*/
    if(pwmb > 0)
    {
        /* 要正转*/
        Int_TB6612_MatorB(GO);
    }
    else if(pwmb < 0)
    {
        /* 要反转*/
        Int_TB6612_MatorB(BACK);
        /* 为后续设置ccr做准备，取绝对值*/
        pwmb = - pwmb;
    }
    else
    {
        /* 刹车*/
        Int_TB6612_MatorB(STOP);
    }

    /* 1.2 设置对应的pwm值 -> 设置对应定时器通道CH3的ccr值*/
    TIM4->CCR3 = pwmb;
}
