#include "APP_Car.h"

short gx, gy, gz;
short ax, ay, az;
float accel_angle;  //通过加速度计算出的角度
float gyro_y;       //Y轴角速度，采样值转换成角度
extern float angle; //跨文件得到的，卡尔曼滤波后的角度

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
    gyro_y = gy / 16.4;

    /* 4.计算的倾角和角速度，进行卡尔曼滤波*/
    Com_Filter_Kalman(accel_angle, gyro_y);


    /* 可以测试一下计算出来的角度和角速度的方向，向前倾时，角度是负的，角速度应该也是负的*/
    printf("accel_angle = %.1f\r\n", accel_angle);
    printf("gyro_y = %.1f/r/n", gyro_y);
    printf("angle = %.1f", angle);
}