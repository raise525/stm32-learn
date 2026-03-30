#include "Dri_TIM.h"


/**
 * 定时器4初始化：CH3，CH4输出pwm
 */
void Dri_TIM4_Init(void){

    /*时钟，gpio的输入输出，定时器基本，pwm模式*/

    /* 1.  开启时钟*/
    /* 1.1 定时器4的时钟 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    /* 1.2 GPIO的时钟 PB */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* 2. 设置GPIO，PB8，PB9为复用推挽输出，则设置为CNF=10 MODE=11*/
    GPIOB->CRH |= GPIO_CRH_MODE8;
    GPIOB->CRH |= GPIO_CRH_CNF8_1;
    GPIOB->CRH &=~ GPIO_CRH_CNF8_0;
    
    GPIOB->CRH |= GPIO_CRH_MODE9;
    GPIOB->CRH |= GPIO_CRH_CNF9_1;
    GPIOB->CRH &=~ GPIO_CRH_CNF9_0;
    

    /* 3. 定时器基本通用配置 */
    /*pwm输出频率（1s有多少个方波） =  72m经过分频之后，得到一秒钟可以数多少个数，设定重载值（数多少个数为一个周期）后，除于重载值可得到周期数
    *pwm输出频率 = 72M时钟频率/（分频*重载值）= 10k，（推荐频率为几k到几十k，这里选择10k，则分频*重载值 = 7200）
    *频率过低相当于大跨步，稳定性低，可能机器会一卡一卡的；
    *频率过高相当于小碎步，mos管开关过于频繁，每种电机都不同
    */
    /* 3.1 预分频器的配置 */
    TIM4->PSC = 1 - 1; //分频设为1，这里的1-1是因为从零开始的
    /* 3.2 自动重装载寄存器的配置 */
    TIM4->ARR = 7200 - 1;
    /* 3.3 计数器的计数方向 0=向上 1=向下*/
    TIM4->CR1 &= ~TIM_CR1_DIR;


    /* 4.pwm模式配置：通道3 */
    /* 4.1 配置通道3的捕获比较寄存器 */
    TIM4->CCR3 = 0;   //这里是设置占空比的，
    /* 4.2 把通道3配置为输出  CCMR2_CC3S=00 */
    TIM4->CCMR2 &= ~TIM_CCMR2_CC3S;
    /* 4.3 配置通道的输出比较模式为：pwm模式1 CCMR2_OC3M=110*/
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_2;
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1;
    TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_0;
    /* 4.4 使能通道3  CCER_CC3E=1 */
    TIM4->CCER |= TIM_CCER_CC3E;
    /* 4.5 设置通道的极性 0=高电平有效  1=低电平有效 */
    TIM4->CCER &= ~TIM_CCER_CC3P;

    /* 5.pwm模式配置：通道4 */
    /* 5.1 配置通道4的捕获比较寄存器 */
    TIM4->CCR4 = 0;   //这里是设置占空比的，
    /* 5.2 把通道3配置为输出  CCMR2_CC4S=00 */
    TIM4->CCMR2 &= ~TIM_CCMR2_CC4S;
    /* 5.3 配置通道的输出比较模式为：pwm模式1 CCMR2_OC4M=110*/
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_2;
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_1;
    TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_0;
    /* 5.4 使能通道4  CCER_CC4E=1 */
    TIM4->CCER |= TIM_CCER_CC4E;
    /* 5.5 设置通道的极性 0=高电平有效  1=低电平有效 */
    TIM4->CCER &= ~TIM_CCER_CC4P;


    /* 6. 启动定时器 */
    TIM4->CR1 |= TIM_CR1_CEN;

}

/**
 * 初始化为编码器模式，返回信息给stm32（A电机）
 */
void Dri_TIM2_Init(void){
    /* 时钟，GIPIO，定时器通用，编码器模式*/
    /* 前三步和前面很像，可以直接复制粘贴上面*/

    /* 1.  开启时钟*/
    /* 1.1 定时器2的时钟 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    /* 1.2 GPIO的时钟 PA，PB，AFIO时钟（重映射功能要开一个时钟，不是默认功能）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN |RCC_APB2ENR_IOPAEN |RCC_APB2ENR_AFIOEN;

    /*2.GPIO重映射*/
    /*2.1 关闭JTAG功能，注意要保留SWD开启！！！！！SWJ_CFG=010*/
    AFIO -> MAPR &=~ AFIO_MAPR_SWJ_CFG_2;
    AFIO -> MAPR |= AFIO_MAPR_SWJ_CFG_1;
    AFIO -> MAPR &=~ AFIO_MAPR_SWJ_CFG_0;

    /*2.2 PA15 重映射为TIM2—CH1， PB3重映射为TIM2-CH2（这是查数据手册知道的）,则要将TIM2_REMAP = 11*/
    AFIO -> MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;


    /* 2.3 初始化引脚： 浮空输入：mode=00，cnf=01 */
    GPIOA->CRH &=~ GPIO_CRH_MODE15;
    GPIOA->CRH &=~ GPIO_CRH_CNF15_1;
    GPIOA->CRH |= GPIO_CRH_CNF15_0;
    
    GPIOB->CRL &=~ GPIO_CRL_MODE3;
    GPIOB->CRL &=~ GPIO_CRL_CNF3_1;
    GPIOB->CRL |= GPIO_CRL_CNF3_0;

    /* 3. 定时器基本通用配置，这是要配置成编码器模式 */
    /* 3.1 预分频器的配置 */
    TIM2->PSC = 1 - 1; //分频设为1，这里的1-1是因为从零开始的
    /* 3.2 自动重装载寄存器的配置 */
    TIM2->ARR = 65536 - 1;

    /* 4. 配置编码器模式*/
    /* 4.1 配置输入通道映射： IC1 映射到 TI1， IC2-TI2， CC1S=01, CC2S=01*/
    TIM2->CCMR1 &=~ TIM_CCMR1_CC1S_1;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;

    TIM2->CCMR1 &=~ TIM_CCMR1_CC2S_1;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;

    /* 4.2 配置不反相 CC1P=0 CCP2 = 0*/
    TIM2 -> CCER &=~ TIM_CCER_CC1P;
    TIM2 -> CCER &=~ TIM_CCER_CC2P;

    /* 4.3 两路信号都计数 ，编码器模式3 ，SMS = 011*/
    TIM2 -> SMCR &=~ TIM_SMCR_SMS_2;
    TIM2 -> SMCR |= TIM_SMCR_SMS_1;
    TIM2 -> SMCR |= TIM_SMCR_SMS_0;

    /* 5.启动定时器*/
    TIM2 -> CR1 |= TIM_CR1_CEN; //使能

}

/**
 * 初始化为编码器模式，返回信息给stm32（B电机）
 */
void Dri_TIM3_Init(void){
    /* 时钟，GIPIO，定时器通用，编码器模式*/
    /* 前三步和前面很像，可以直接复制粘贴上面*/

    /* 1.  开启时钟*/
    /* 1.1 定时器3的时钟 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    /* 1.2 GPIO的时钟 PA，PB，AFIO时钟（重映射功能要开一个时钟，不是默认功能）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN |RCC_APB2ENR_AFIOEN;

    /*2.GPIO重映射*/
    /*2.1 关闭JTAG功能，注意要保留SWD开启！！！！！SWJ_CFG=010*/
    AFIO -> MAPR &=~ AFIO_MAPR_SWJ_CFG_2;
    AFIO -> MAPR |= AFIO_MAPR_SWJ_CFG_1;
    AFIO -> MAPR &=~ AFIO_MAPR_SWJ_CFG_0;

    /*2.2 PB4 重映射为TIM3—CH1， PB5重映射为TIM3-CH2（这是查数据手册知道的）,则要将TIM3_REMAP = 10*/
    AFIO -> MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP;


    /* 2.3 初始化引脚： 浮空输入：mode=00，cnf=01 */
    GPIOB->CRL &=~ GPIO_CRL_MODE4;
    GPIOB->CRL &=~ GPIO_CRL_CNF4_1;
    GPIOB->CRL |= GPIO_CRL_CNF4_0;
    
    GPIOB->CRL &=~ GPIO_CRL_MODE5;
    GPIOB->CRL &=~ GPIO_CRL_CNF5_1;
    GPIOB->CRL |= GPIO_CRL_CNF5_0;

    /* 3. 定时器基本通用配置，这是要配置成编码器模式 */
    /* 3.1 预分频器的配置 */
    TIM3->PSC = 1 - 1; //分频设为1，这里的1-1是因为从零开始的
    /* 3.2 自动重装载寄存器的配置 */
    TIM3->ARR = 65536 - 1;

    /* 4. 配置编码器模式*/
    /* 4.1 配置输入通道映射： IC1 映射到 TI1， IC2-TI2， CC1S=01, CC2S=01*/
    TIM3->CCMR1 &=~ TIM_CCMR1_CC1S_1;
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;

    TIM3->CCMR1 &=~ TIM_CCMR1_CC2S_1;
    TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

    /* 4.2 配置不反相 CC1P=0 CCP2 = 0*/
    TIM3 -> CCER &=~ TIM_CCER_CC1P;
    TIM3 -> CCER &=~ TIM_CCER_CC2P;

    /* 4.3 两路信号都计数 ，编码器模式3 ，SMS = 011*/
    TIM3 -> SMCR &=~ TIM_SMCR_SMS_2;
    TIM3 -> SMCR |= TIM_SMCR_SMS_1;
    TIM3 -> SMCR |= TIM_SMCR_SMS_0;

    /* 5.启动定时器*/
    TIM3 -> CR1 |= TIM_CR1_CEN; //使能

}
