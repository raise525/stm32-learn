/*
 * sensor_data.h
 *主要包含两个传感器的数据，校验码，状态
 *  Created on: Feb 2, 2026
 *      Author: Raise
 */

#ifndef INC_SENSOR_DATA_H_
#define INC_SENSOR_DATA_H_

#include <stdint.h>
typedef struct {
    // 1. 数据标识与时间
    uint32_t timestamp_ms;  // 系统时间戳 (HAL_GetTick())
    uint32_t packet_id;     // 包序号，每次发送自增

    // 2. DHT11 数据及状态
    float    temperature_c; // 温度
    uint8_t  humidity;      // 湿度
    uint8_t  dht_status;    // 状态码 (0=成功, 1=失败)

    // 3. MPU6050 数据及状态
    float    accel_x, accel_y, accel_z; // 加速度 (g)
    float    gyro_x, gyro_y, gyro_z;    // 陀螺仪 (°/s)
    float    imu_temp_c;                // IMU芯片温度
    uint8_t  imu_status;                // IMU状态 (0=正常，可预留)

    // 4. 系统状态 (可选)
    // uint8_t  battery_level;          // 为未来扩展预留
    // uint8_t  system_status;          // 系统状态码
} sensor_data_t;

#endif /* INC_SENSOR_DATA_H_ */
