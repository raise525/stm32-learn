#include "Dri_SPI.h"


void Driver_SPI_Init(void)
{
    /* 1. 开启GPIO时钟 PA*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN ;

    /* 2. 设置引脚的工作模式 */
    /* 2.1 cs: 推挽输出 PA4*  MODE=11 CNF=00  */
    GPIOA->CRL |= GPIO_CRL_MODE4;
    GPIOA->CRL &= ~GPIO_CRL_CNF4;
    /* 2.2 sck: 推挽输出 PA5*/
    /* 2.3 mosi: 推挽输出 PA7*/
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7);
    GPIOA->CRL |= (GPIO_CRL_MODE5 | GPIO_CRL_MODE7);
    /* 2.4 miso: 浮空输入 PA6  CNF=01 MODE=00*/
    GPIOA->CRL &= ~(GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6);
    GPIOA->CRL |= GPIO_CRL_CNF6_0;

    /* 3. spi的模式0  sck空闲状态是 0   */
    SCK_LOW;
    /* 4. 片选默认不选中 */
    CS_HIGH;

}



uint8_t Driver_SPI_SwapByte(uint8_t byte)
{
    /* 拉低选中，*/
    CS_LOW;

    uint8_t rByte = 0x00;
    for (uint8_t i = 0; i < 8; i++)
    {
        SCK_LOW;
        /* 1. 先把数据放入到MOSI上 */
        if (byte & 0x80)
        {
            MOSI_HIGH;
        }
        else
        {
            MOSI_LOW;
        }
        byte <<= 1;
        // SPI_DELAY;
        /* 2. 拉高时钟 (产生上升沿)*/
        SCK_HIGH;
    }

    /* 拉高不选中*/
    CS_HIGH;

    return 0;
}
