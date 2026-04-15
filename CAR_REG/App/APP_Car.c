#include "APP_Car.h"
#include <stdio.h>

short gx, gy, gz;
short ax, ay, az;
float accel_angle;  //通过加速度计算出的角度
float gyro_y;       //Y轴角速度，采样值转换成角度
extern float angle; //跨文件得到的，卡尔曼滤波后的角度

int ea, eb; // 电机编码器的值

char bat_str[5]; // 2个整数位+1个小数点+1个小数位+1个字符串结尾\0 
char ea_str[7]; // 1个符号位+5个整数位+1个字符串结尾\0   范围是-32768~32767
char eb_str[7]; // 1个符号位+5个整数位+1个字符串结尾\0   范围是-32768~32767
char angle_str[7]; // 1个符号位+3个整数位+1个小数点+1个小数位+1个字符串结尾\0   范围是-180.0~180.0

/**
 * 计算小车倾角
 */
void App_Car_GetAngel(void)
{
    /* 1. 读取MPU6050数据*/
    Int_MPU6050_Get_Accel(&ax,&ay,&az);
    Int_MPU6050_Get_Gyro(&gx,&gy,&gz);

    /* 2.通过加速度计算倾角*/
    accel_angle = atan2(ax,az) * 180 / PI;//第一个参数是分子的,得到的是弧度，要转为度，角度=弧度*180/PI

    /* 3.角速度是看gy，是绕着对应轴旋转，我们得到的是一个adc的采样值，不是具体的度数，也要转换*/
    /* 量程是+—2000°/s， 65535/4000=16.4*/
    /* 注意：角速度的符号要和角加速度的方向相同！！！*/
    gyro_y = -gy / 16.4;

    /* 4.计算的倾角和角速度，进行卡尔曼滤波*/
    Com_Filter_Kalman(accel_angle, gyro_y);

    printf("正在计算倾角...\r\n");


    /* 可以测试一下计算出来的角度和角速度的方向，向前倾时，角度是负的，角速度应该也是负的*/
    printf("accel_angle = %.1f\r\n", accel_angle);
    printf("gyro_y = %.1f\r\n", gyro_y);
    printf("angle = %.1f\r\n", angle);

    /* 将读取编码器的值，也放到获取角度的计算函数中，这样两类数据就可以同频 */
    ea = Int_Encodr_ReadCounter(2);
    eb = -Int_Encodr_ReadCounter(3);
}


/**
 * 显示任务：填充电池电压值，电机编码器的值，计算的倾角
 */
void App_Car_Display(void)
{
    /* 1.填充电压值*/
    // printf("正在显示电压值...\r\n");
    // double bat_vol = Driver_ADC1_ReadV();
    // sprintf(bat_str, "%3.1f", bat_vol); // 将电压值转换为字符串，保留一位小数
    // OLED_ShowString(32,0,bat_str,16,1);

    // /* 2.填充电机编码器的值*/
    // sprintf(ea_str, "%6d", ea); // 将编码器值转换为字符串，这里写6是为了给符号位和整数位留出位置，方便刷新时覆盖之前的值
    // sprintf(eb_str, "%6d", eb); // 将编码器值转换为字符串
    // OLED_ShowString(24,16,ea_str,16,1);// 显示在EA:后面,三个字符， x=3*8=24开始；第二行，y=16
    // OLED_ShowString(24,32,eb_str,16,1);// 显示在EB:后面,三个字符， x=3*8=24开始；第三行，y=32

    /* 3.填充计算的倾角*/
    sprintf(angle_str, "%5.1f", angle); // 将倾角值转换为字符串，保留一位小数
    OLED_ShowString(48,48,angle_str,16,1); // 显示在Angle:后面，是六个字符， x=6*8=48开始；第四行，y=48

    /* 4.刷新显示*/
    OLED_Refresh();
}