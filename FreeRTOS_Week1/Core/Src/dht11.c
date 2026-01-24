#include "dht11.h"

// 静态函数声明
static void DHT11_Delay_us(uint16_t us);
static void DHT11_Write_Pin(uint8_t state);
static uint8_t DHT11_Read_Pin(void);
static uint8_t DHT11_Start(void);
static uint8_t DHT11_Read_Byte(void);

// 数据缓冲区
static uint8_t dht11_buffer[5];

// 微秒级延时（粗略实现）
static void DHT11_Delay_us(uint16_t us) {
    us *= 8;  // 根据72MHz系统时钟调整
    while(us--) {
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
    }
}

// 写引脚状态
static void DHT11_Write_Pin(uint8_t state) {
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, (GPIO_PinState)state);
}

// 读引脚状态
static uint8_t DHT11_Read_Pin(void) {
    return (uint8_t)HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
}

// 启动DHT11通信
static uint8_t DHT11_Start(void) {
    uint32_t timeout = 0;

    // 1. 主机拉低至少18ms
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);

    DHT11_Write_Pin(0);
    HAL_Delay(20);

    DHT11_Write_Pin(1);
    DHT11_Delay_us(30);

    // 2. 切换为输入模式，等待DHT11响应
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);

    // 等待DHT11拉低响应
    timeout = 1000;
    while(DHT11_Read_Pin() == 1) {
        if(--timeout == 0) return 1;
        DHT11_Delay_us(1);
    }

    // 等待DHT11拉高
    timeout = 1000;
    while(DHT11_Read_Pin() == 0) {
        if(--timeout == 0) return 1;
        DHT11_Delay_us(1);
    }

    // 等待DHT11拉低，开始传输数据
    timeout = 1000;
    while(DHT11_Read_Pin() == 1) {
        if(--timeout == 0) return 1;
        DHT11_Delay_us(1);
    }

    return 0;
}

// 读取一个字节
static uint8_t DHT11_Read_Byte(void) {
    uint8_t byte = 0;
    uint32_t timeout;

    for(uint8_t i = 0; i < 8; i++) {
        // 等待低电平结束
        timeout = 1000;
        while(DHT11_Read_Pin() == 0) {
            if(--timeout == 0) return 0xFF;
            DHT11_Delay_us(1);
        }

        // 延时40us后采样
        DHT11_Delay_us(40);

        // 判断是0还是1
        if(DHT11_Read_Pin() == 1) {
            byte |= (0x80 >> i);
        }

        // 等待高电平结束
        timeout = 1000;
        while(DHT11_Read_Pin() == 1) {
            if(--timeout == 0) return 0xFF;
            DHT11_Delay_us(1);
        }
    }

    return byte;
}

// 初始化DHT11
uint8_t DHT11_Init(void) {
    // 配置PA1为推挽输出，上拉
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);

    // 初始化为高电平
    DHT11_Write_Pin(1);

    // 延时等待DHT11稳定
    HAL_Delay(1000);

    // 测试通信
    return DHT11_Start();
}

 //读取温湿度数据
uint8_t DHT11_Read(DHT11_Data_t* data) {
    if(data == NULL) return 1;

    // 启动通信
    if(DHT11_Start() != 0) {
        return 1;
    }

    // 读取5个字节数据
    for(uint8_t i = 0; i < 5; i++) {
        dht11_buffer[i] = DHT11_Read_Byte();
        if(dht11_buffer[i] == 0xFF) {
            return 1;
        }
    }

    // 校验数据
    uint8_t checksum = dht11_buffer[0] + dht11_buffer[1] +
                       dht11_buffer[2] + dht11_buffer[3];

    if(dht11_buffer[4] != checksum) {
        return 1;
    }

    // 解析数据
    data->humidity = dht11_buffer[0];
    data->temperature = (float)dht11_buffer[2];
    if(dht11_buffer[3] > 0) {
        data->temperature += (float)dht11_buffer[3] / 10.0f;
    }

    return 0;
}


