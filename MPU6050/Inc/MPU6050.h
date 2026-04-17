#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"  // 根据实际芯片修改

// ========== MPU6050 寄存器定义 ==========
#define MPU6050_ADDR_DEFAULT     0xD0    // 0x68 << 1 (AD0=0)
#define MPU6050_WHO_AM_I         0x75
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_PWR_MGMT_2       0x6C
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_GYRO_XOUT_H      0x43

// ========== 配置参数 ==========
#define MPU6050_WAKE_VALUE       0x01    // 时钟源=陀螺仪X轴，唤醒
#define MPU6050_RESET_VALUE      0x80    // 设备复位
#define MPU6050_SMPLRT_1000HZ    0x00    // 1kHz 采样率（SMPLRT_DIV=0）
#define MPU6050_FILTER_20HZ      0x04    // DLPF_CFG=4，20Hz带宽
#define MPU6050_GYRO_250DPS      0x00    // ±250°/s
#define MPU6050_ACCEL_2G         0x00    // ±2g

// 灵敏度（对应配置）
#define MPU6050_GYRO_250DPS_SEN   (1.0f / 131.0f)
#define MPU6050_ACCEL_2G_SEN      (1.0f / 16384.0f)
#define MPU6050_TEMP_FACTOR       (1.0f / 340.0f)
#define MPU6050_TEMP_OFFSET       36.53f

// ========== 错误码 ==========
#define MPU6050_NO_ERROR         0x00
#define MPU6050_I2C_ERROR        0x01
#define MPU6050_NO_SENSOR        0x02
#define MPU6050_PWR_ERROR        0x03
#define MPU6050_SMPLRT_ERROR     0x04
#define MPU6050_CONFIG_ERROR     0x05
#define MPU6050_GYRO_RANGE_ERROR 0x06
#define MPU6050_ACCEL_RANGE_ERROR 0x07

// DMA 相关（可选）
#define MPU6050_DMA_MAX_LEN      14
#define MPU6050_I2C_TIMEOUT_MS   10

// ========== 数据结构 ==========
typedef struct {
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} MPU6050_RawData;

typedef struct {
    float accel[3];     // 单位: g
    float gyro[3];      // 单位: °/s
    float temp;         // 单位: ℃
} MPU6050_RealData;

// ========== C++ 类定义 ==========
#ifdef __cplusplus
class MPU6050 {
public:
    MPU6050(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr = MPU6050_ADDR_DEFAULT);
    uint8_t init();
    bool readRawData();                     // 阻塞读取原始数据，成功返回true
    void convertData();                     // 将原始数据转换为物理量
    void read(float gyro[3], float accel[3], float* temperature); // 一站式读取

    // 单字节寄存器操作（阻塞，返回状态）
    HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t data);
    HAL_StatusTypeDef readReg(uint8_t reg, uint8_t* data);

    // 多字节读取（阻塞）
    HAL_StatusTypeDef readMultiReg(uint8_t reg, uint8_t* buf, uint8_t len);

    // 多字节读取（DMA，需在CubeMX中使能I2C的DMA）
    HAL_StatusTypeDef readMultiRegDMA(uint8_t reg, uint8_t* buf, uint8_t len);

    // 检查设备是否在线
    bool isConnected();

    // 获取实时数据引用
    const MPU6050_RealData& getRealData() const { return real_; }
    const MPU6050_RawData& getRawData() const { return raw_; }

    volatile uint8_t dma_done_;
    volatile uint8_t dma_error_;

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t i2c_addr_;

    MPU6050_RawData raw_;
    MPU6050_RealData real_;

    // DMA 完成标志（若使用DMA）


    // 内部初始化子函数
    uint8_t sensorInit();
};

// ========== C 语言接口 ==========
extern "C" {
#endif

// 初始化函数，返回错误码
uint8_t MPU6050_Init(void);
// 检查传感器是否存在
uint8_t MPU6050_IsConnected(void);
// 读取数据（阻塞模式）
void MPU6050_Read(float gyro[3], float accel[3], float* temperature);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H