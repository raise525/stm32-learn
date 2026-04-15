#include "Int_MPU6050.h"


/*起码有：初始化，封装读取写入函数，有很多寄存器，可以借用人家写的宏*/

/* 读取指定寄存器的一个字节数据*/
/**
 * 根据时序图来写
 * reg-addr:想要读取的寄存器
 * receive_byte：把读取到的数据放到的地址，为什么用指针？就是因为C语言函数只能返回一个值，但你需要把读到的数据"带回来"。
 */
uint8_t Int_MPU6050_ReadByte(uint8_t reg_addr, uint8_t *receive_byte)
{
    /* 1.起始信号，什么是假写？假写就是让mpu知道你想要读的数据是来自哪个寄存器的*/
    Driver_I2C2_Start();
    /* 2.AD+W (mpu6050时序图上面的),把想读的寄存器地址写入给6050*/
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X00);
    /* 3.发送寄存器的地址*/
    Driver_I2C_SendByte(reg_addr);
    /* 4.起始信号*/
    Driver_I2C2_Start();
    /* 5.AD+R */
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X01);
    /* 6.硬件I2C要发送NACK和停止信号*/
    Driver_I2C2_NAck();
    Driver_I2C2_Stop();
    /* 7.读取一字节数据*/
    *receive_byte = Driver_I2C_ReadByte();

    return 0;
}


/**
 * 读取指定寄存器的多个字节数据，根据时序图来写
 */
uint8_t Int_MPU6050_ReadBytes(uint8_t reg_addr, uint8_t *receive_buff, uint8_t size)
{
    /* 1.起始信号，什么是假写？*/
    Driver_I2C2_Start();
    /* 2.AD+W (mpu6050时序图上面的),把想读的寄存器地址写入给6050*/
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X00);
    /* 3.发送寄存器的地址*/
    Driver_I2C_SendByte(reg_addr);
    /* 4.起始信号*/
    Driver_I2C2_Start();
    /* 5.AD+R */
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X01);
    /*6. 连续读多个字节*/
    for(uint8_t i = 0; i < size; i++)
    {

        if(i < size - 1)
        {
            /* 说明不是最后一个字节，只需要发送ack*/
            Driver_I2C2_Ack();
        }else
        {
            /* 是最后一个字节，要发送nack和ack*/
            Driver_I2C2_NAck();
            Driver_I2C2_Stop();
        }
        receive_buff[i]=Driver_I2C_ReadByte();//还有东西，就是要发送NACK和停止信号
    }

    return 0;
}

/**
 * 向MUP6050写入一个字节，但是我没有找到时序图
 */
uint8_t Int_MPU6050_WriteByte(uint8_t reg_addr, uint8_t write_byte)
{
    /* 1.起始信号，什么是假写？*/
    Driver_I2C2_Start();
    /* 2.AD+W (mpu6050时序图上面的),把想读的寄存器地址写入给6050*/
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X00);
    /* 3.发送寄存器的地址*/
    Driver_I2C_SendByte(reg_addr);
    /* 4.写入一字节*/
    Driver_I2C_SendByte(write_byte);
    /* 5.停止信号*/
    Driver_I2C2_Stop();
    return 0;
}

/**
 * 向MUP6050写入多个字节，但是我没有找到时序图
 */
uint8_t Int_MPU6050_WriteBytes(uint8_t reg_addr, uint8_t *write_bytes, uint8_t size)
{
    /* 1.起始信号，什么是假写？*/
    Driver_I2C2_Start();
    /* 2.AD+W (mpu6050时序图上面的),把想读的寄存器地址写入给6050*/
    Driver_I2C_SendAddr(MPU_IIC_ADDR << 1 | 0X00);
    /* 3.发送寄存器的地址*/
    Driver_I2C_SendByte(reg_addr);
    /* 4.写入多字节*/
    for(uint8_t i = 0; i < size;i++)
    {
        Driver_I2C_SendByte(write_bytes[i]);
    }
    /* 5.停止信号*/
    Driver_I2C2_Stop();
    return 0;
}


/**
 * 根据采样率设置低通滤波器
 * 
 */
void Int_MPU6050_Set_DLPF_CFG(rate)
{
    /* 采样定理： 采样率 >= 2*带宽 才不会失真 ->  带宽 <= 采样率/2 */
    uint8_t cfg = 0;
    rate = rate / 2;
    if(rate > 188)
    {
        cfg = 1;
    }
    else if(rate >98)
    {
        cfg = 2;
    }
    else if(rate >42)
    {
        cfg =3;
    }
    else if(rate > 20)
    {
        cfg = 4; 
    }
    else if(rate > 10)
    {
        cfg = 5;
    }
    else 
    {
        cfg =6;
    }

    Int_MPU6050_WriteByte(MPU_CFG_REG ,cfg << 0);
}

/**
 * 设置陀螺仪的采样率
 */
void Int_MPU6050_SetGyroRate(uint16_t rate)
{
    /* 采样率 = 输出频率/（1 + 分频值） -> 我们是可以设置分频值到寄存器里面，所以，我们要先算出分频值=（输出频率/采样率）- 1*/
    /* 这里的频率手册里面有两种选择：8k和1k，我们选1k是因为比较稳*/
    /* 采样率设置多少呢？ 根据香农定理，采样率应大于频率的两倍，我们想要采样率100hz，则频率应小于50，所以低通滤波那里要选择小于50的*/
    uint8_t sample_div = 0;

    /* 1. 采样率限幅：不能让人乱写,产品说明书12页，输出频率最小是4hz，最大是8k，我们期望是1k*/
    if(rate < 4)
    {
        rate = 4;
    }
    else if(rate > 1000)
    {
        rate = 1000;
    }

    /* 2. 根据期望的采样率计算分频值*/
    sample_div = 1000 / rate -1; 

    
    /* 3. 将分频值设置到寄存器中*/
    Int_MPU6050_WriteByte(MPU_SAMPLE_RATE_REG,sample_div);

    /* 4. 根据采样率去设置低通滤波器*/
    Int_MPU6050_Set_DLPF_CFG(rate);
    
}

/**
 * 初始化
 */
void Int_MPU6050_Init(void)
{
    uint8_t dev_id = 0;
    /* 1.初始化IIC*/
    Driver_I2C2_Init();

    /* 2. 复位 -> 延迟一会 -> 唤醒,,就是要去看对应的寄存器*/
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0X80);
    for_delay_ms(300);
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0X00);

    /* 3.陀螺仪量程 +-2000°/s， frs=3*/
    Int_MPU6050_WriteByte(MPU_GYRO_CFG_REG, 3 << 3);

    /* 4.加速度量程*/
    Int_MPU6050_WriteByte(MPU_ACCEL_CFG_REG, 0 << 3);

    /* 5.其他功能设置（可选）： FIFO， 第二IIC，中断，都是在MPU6050上面的，我们只需要用IIC，其他的就关闭*/
    Int_MPU6050_WriteByte(MPU_INT_EN_REG, 0X00); //关闭所有中断
    Int_MPU6050_WriteByte(MPU_USER_CTRL_REG, 0X00); //关闭第二IIC
    Int_MPU6050_WriteByte(MPU_FIFO_EN_REG, 0x00);//关闭FIFO

    /* 6.系统时钟源，陀螺仪采样率，低通滤波设置*/
    /* 配置时钟前，确认正常工作，读一下id*/
    Int_MPU6050_ReadByte(MPU_DEVICE_ID_REG, &dev_id);
    if(dev_id == MPU_IIC_ADDR || dev_id == 0x70) //有的模块地址是0x68，有的是0x70，可能是因为接线方式不同导致的
    {
        /* 6.1配置时钟源，选择陀螺仪x轴的时钟，精度更高*/
        Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x01);

        /* 6.2设置陀螺仪采样率,就是采集数据的频率 &低通滤波*/
        Int_MPU6050_SetGyroRate(100);

        /* 6.3 让两个传感器退出待机模式，进入正常工作模式*/
        Int_MPU6050_WriteByte(MPU_PWR_MGMT2_REG,0x00);
    }
}

/**
 * 读取陀螺仪角速度的数据，拼接两个字节的数据并转成带符号类型
 */
void Int_MPU6050_Get_Gyro(short *gx, short *gy, short *gz )
{
    uint8_t buff[6];
    Int_MPU6050_ReadBytes(MPU_GYRO_XOUTH_REG,buff,6);

    /*
        buff[0]:角速度x轴高8位
        buff[0]:角速度x轴低8位
        buff[0]:角速度y轴高8位
        buff[0]:角速度y轴低8位
        buff[0]:角速度z轴高8位
        buff[0]:角速度z轴低8位
    */
    *gx =( (short)buff[0] << 8 )| buff[1];
    *gy =( (short)buff[2] << 8 )| buff[3];
    *gz =( (short)buff[4] << 8 )| buff[5];
}

/**
 * 读取加速度计的加速度数据，拼接两个字节的数据并转成带符号类型
 */
void Int_MPU6050_Get_Accel(short *ax, short *ay, short *az )
{
    uint8_t buff[6];
    Int_MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG,buff,6);

    /*
        buff[0]:加速度x轴高8位
        buff[0]:加速度x轴低8位
        buff[0]:加速度y轴高8位
        buff[0]:加速度y轴低8位
        buff[0]:加速度z轴高8位
        buff[0]:加速度z轴低8位
    */
    *ax =( (short)buff[0] << 8 )| buff[1];
    *ay =( (short)buff[2] << 8 )| buff[3];
    *az =( (short)buff[4] << 8 )| buff[5];
}

void test_whoami(void)
{
    uint8_t id = 0;
    Int_MPU6050_ReadByte(0x75, &id);
    // 通过串口打印id的值，期望是0x68
    printf("WHO_AM_I = 0x%02X\r\n", id);
}