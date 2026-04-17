//
// Created by lingard on 2026/4/16.
//

#ifndef TWOWHEELBALCANCE_MPU6050_H
#define TWOWHEELBALCANCE_MPU6050_H

//
// Created by 20852 on 2025/11/29.
//
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include "bsp_dwt.h"
#include <stdint.h>
#include <stdbool.h>

// 错误码枚举 (与BMI088保持一致)
typedef enum {
    MPU6050_NO_ERROR = 0,
    MPU6050_NO_SENSOR = 1 << 0,
    MPU6050_PWR_ERROR = 1 << 1,
    MPU6050_SMPLRT_ERROR = 1 << 2,
    MPU6050_CONFIG_ERROR = 1 << 3,
    MPU6050_GYRO_RANGE_ERROR = 1 << 4,
    MPU6050_ACCEL_RANGE_ERROR = 1 << 5
} MPU6050_StatusTypeDef;

// MPU6050 寄存器定义
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43

// MPU6050 配置值
#define MPU6050_RESET_VALUE    0x80
#define MPU6050_WAKE_VALUE      0x00
#define MPU6050_SMPLRT_1000HZ   0x00
#define MPU6050_FILTER_20HZ     0x04
#define MPU6050_GYRO_250DPS     0x00
#define MPU6050_ACCEL_2G        0x00

// 灵敏度转换系数
#define MPU6050_ACCEL_2G_SEN    1.0f/16384.0f   // g/LSB
#define MPU6050_GYRO_250DPS_SEN 1.0f/131.0f     // °/s/LSB
#define MPU6050_TEMP_FACTOR     1.0f/340.0f
#define MPU6050_TEMP_OFFSET     36.53f

// 延时宏
#define MPU6050_COM_WAIT_SENSOR_TIME 50
#define MPU6050_LONG_DELAY_TIME      100

// 配置寄存器数量
#define MPU6050_WRITE_REG_NUM 5
#ifdef __cplusplus
class MPU6050 {
private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t i2c_addr_;  // MPU6050 I2C地址 (默认0x68<<1)

    // 原始数据结构体
    struct RawData {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t temp;
    } raw_;

    // 转换后数据结构体
    struct RealData {
        float accel[3];
        float gyro[3];
        float temp;
    } real_;

    // 配置数组 (寄存器地址、写入值、错误码)
    static uint8_t writeConfig[MPU6050_WRITE_REG_NUM][3];

    // 底层读写函数
    inline void writeReg(uint8_t reg, uint8_t data);
    inline uint8_t readReg(uint8_t reg);
    inline void readMulti(uint8_t reg, uint8_t* data, uint16_t len);
    void readMultiReg(uint8_t reg, uint8_t* buf, uint8_t len);
    HAL_StatusTypeDef readMultiRegDMA(uint8_t reg, uint8_t* buf, uint8_t len);

    // 初始化传感器
    uint8_t sensorInit();

public:
    // 构造函数
    MPU6050(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr);

    // 初始化函数
    uint8_t init();

    // 读取原始数据 & 转换数据
    void readRawData();
    void convertData();

    // 对外读取接口
    void read(float gyro[3], float accel[3], float* temperature);
};
#endif
// C语言接口
#ifdef __cplusplus
extern "C" {
#endif

void MPU6050_Init(void);
void MPU6050_Read(float gyro[3], float accel[3], float* temperature);

#ifdef __cplusplus
}
#endif

#endif //MPU6050_H

#endif //TWOWHEELBALCANCE_MPU6050_H