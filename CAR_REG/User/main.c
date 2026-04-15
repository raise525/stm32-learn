#include "main.h"
#include "freertos.h"
#include "task.h"
#include "Int_TB6612.h"
#include "Int_Encoder.h"
#include "Dri_USART1.h"
#include "Int_MPU6050.h"
#include "APP_Car.h"
#include "Dri_ADC.h"
#include "oled.h"

/* 实现毫秒的延时*/
void for_delay_ms(uint32_t ms)
{
    uint32_t Delay = ms * 72000 / 9; /* 72M时钟频率 */
    do
    {
        __NOP(); /* 空指令（NOP）来占用 CPU 时间 */
    } while (Delay--);
}


int main(void)
{

    Driver_USART1_Init();

    Int_TB6612_Init();

    //Int_TB6612_SetPWM(3600,3600);

	Int_Encoder_Init();

    Int_MPU6050_Init();

    //Driver_ADC1_Init();

    OLED_Init();
    OLED_Clear();
   

    /**
     * 第一个参数：X坐标，水平方向
     * 第二个参数：Y坐标，垂直方向
     * 第三个参数：要显示的字符串
     * 第三个参数：字体高度（字库支持），就是要占的y高度
     * 第五个参数：显示模式，0反显（白底黑字），1正显（黑底白字）
     */
    //OLED_ShowString(0,10,"atguigu",16,1);
    OLED_ShowString(0,0,"BAT:      V",16,1);
    OLED_ShowString(0,16,"EA:",16,1);
    OLED_ShowString(0,32,"EB:",16,1);
    OLED_ShowString(0,48,"Angle:",16,1);
    OLED_Refresh();

    // short gx=0,gy=0,gz=0;
    // short ax=0,ay=0,az=0;

    printf("welcome to use FreeRTOS!!!\r\n");

   

    while(1)
    {
        /* =====================测试编码器读取值============================*/
        // printf("welcom!!!\r\n");
        // printf("tim2 cnt=%d\r\n", Int_Encodr_ReadCounter(2));
        // printf("tim3 cnt=%d\r\n", Int_Encodr_ReadCounter(3));
        // for_delay_ms(1000);

        /* =====================测试MPU6050读取值============================*/
        /*如果我要自己测的话，可能要注意MPU的放置方向，应该会影响后面的姿态控制 */
        /*静止平放时，读数接近 +16384 的那个轴，它的正方向就是竖直向上（背向地心）。因为他显示的是支持力*/

        // test_whoami();

        // Int_MPU6050_Get_Accel(&ax,&ay,&az);
        // Int_MPU6050_Get_Gyro(&gx,&gy,&gz);

        // printf("gx=%d\r\n", gx);
        // printf("gy=%d\r\n", gy);
        // printf("gz=%d\r\n", gz);
        // printf("ax=%d\r\n", ax);
        // printf("ay=%d\r\n", ay);
        // printf("az=%d\r\n", az);
        // for_delay_ms(100);


        /* ============================测试计算角度====================================*/
        //App_Car_GetAngel();
        // for_delay_ms(10);//陀螺仪采样率是100hz，就是10ms一次，要有即时性，避免影响效果
        /* 这里的测试目的就是：看一下往前倒的时候，角度的符号，并且在那一时刻，角加速度的符号应保持一致，是个趋势问题*/
        /* 前后都要测试，如果符号不统一就要去，取反gyro_y*/

        /* ============================测试oled显示====================================*/
        App_Car_GetAngel();
        App_Car_Display();
        for_delay_ms(100);//我烧录进去之后，没有角度值的显示？难道我的mpu6050没工作？明天再看看,我知道了应该显示电压那里卡了，因为我根本就没做那个，然后后面就卡死了，我现在就是直接把那个注释掉了

    }

}

extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}
