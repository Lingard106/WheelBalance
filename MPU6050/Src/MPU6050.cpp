//
// Created by lingard on 2026/4/16.
//

#include "../Inc/MPU6050.h"


extern I2C_HandleTypeDef hi2c1;

// 最大支持一次 DMA 读取的字节数
#ifndef MPU6050_DMA_MAX_LEN
#define MPU6050_DMA_MAX_LEN 14
#endif

static volatile uint8_t mpu6050_i2c_dma_done = 0;
static volatile uint8_t mpu6050_i2c_dma_error = 0;
static uint8_t mpu6050_dma_tx_buf[MPU6050_DMA_MAX_LEN];

// 配置数组
uint8_t MPU6050::writeConfig[MPU6050_WRITE_REG_NUM][3] = {
    {MPU6050_PWR_MGMT_1,   MPU6050_WAKE_VALUE,    MPU6050_PWR_ERROR},
    {MPU6050_SMPLRT_DIV,   MPU6050_SMPLRT_1000HZ, MPU6050_SMPLRT_ERROR},
    {MPU6050_CONFIG,       MPU6050_FILTER_20HZ,   MPU6050_CONFIG_ERROR},
    {MPU6050_GYRO_CONFIG,  MPU6050_GYRO_250DPS,   MPU6050_GYRO_RANGE_ERROR},
    {MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_2G,      MPU6050_ACCEL_RANGE_ERROR}
};

// 构造函数
MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr)
    : hi2c_(hi2c), i2c_addr_(i2c_addr) {}

// 初始化
uint8_t MPU6050::init() {
    uint8_t error = 0;
    // I2C初始化已在外部完成
    error |= sensorInit();
    return error;
}

// 传感器初始化
uint8_t MPU6050::sensorInit() {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    // 检查ID
    res = readReg(MPU6050_WHO_AM_I);
    DWT_Delay_us(MPU6050_COM_WAIT_SENSOR_TIME);

    // 软件复位
    writeReg(MPU6050_PWR_MGMT_1, MPU6050_RESET_VALUE);
    DWT_Delay_ms(MPU6050_LONG_DELAY_TIME);

    // ========== 【终极防休眠：锁死全部电源配置】 ==========
    // 1. 时钟源=陀螺仪X轴 + 唤醒 + 禁用温度休眠
    writeReg(MPU6050_PWR_MGMT_1, 0x01);
    DWT_Delay_ms(10);
    // 2. 锁死PWR_MGMT_2：禁用加速度/陀螺仪所有轴待机（核心！）
    writeReg(0x31, 0x00);
    DWT_Delay_ms(1);
    // 3. 禁用循环休眠/唤醒
    writeReg(MPU6050_CONFIG, 0x04);
    DWT_Delay_ms(1);

    // 校验ID
    res = readReg(MPU6050_WHO_AM_I);
    if (res != 0x68) return MPU6050_NO_SENSOR;

    // 写入配置
    for (write_reg_num = 0; write_reg_num < MPU6050_WRITE_REG_NUM; write_reg_num++) {
        writeReg(writeConfig[write_reg_num][0], writeConfig[write_reg_num][1]);
        DWT_Delay_us(MPU6050_COM_WAIT_SENSOR_TIME);
        res = readReg(writeConfig[write_reg_num][0]);
        if (res != writeConfig[write_reg_num][1]) return writeConfig[write_reg_num][2];
    }

    // 最后加固：再次锁死电源，防止寄存器漂移
    writeReg(MPU6050_PWR_MGMT_1, 0x01);
    writeReg(0x31, 0x00);
    return MPU6050_NO_ERROR;
}

// 单字节写寄存器
inline void MPU6050::writeReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    DWT_Delay_us(1);
}

// 单字节读寄存器
inline uint8_t MPU6050::readReg(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    return data;
}

// 多字节读寄存器
inline void MPU6050::readMulti(uint8_t reg, uint8_t* data, uint16_t len) {
    // 直接调用阻塞读取，彻底禁用DMA！！！
    readMultiReg(reg, data, len);
}

// 阻塞式多字节读取
void MPU6050::readMultiReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    HAL_I2C_Mem_Read(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
}

// DMA多字节读取
HAL_StatusTypeDef MPU6050::readMultiRegDMA(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!buf || len == 0 || len > MPU6050_DMA_MAX_LEN) return HAL_ERROR;

    mpu6050_i2c_dma_done = 0;
    mpu6050_i2c_dma_error = 0;

    if (HAL_I2C_Mem_Read_DMA(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK) {
        return HAL_ERROR;
    }

    // 等待DMA完成
    uint32_t t0 = HAL_GetTick();
    const uint32_t timeout_ms = 10;
    while (!mpu6050_i2c_dma_done && !mpu6050_i2c_dma_error) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            // F103 没有 HAL_I2C_Abort，改用「中止DMA传输 + 重置I2C状态」
            if (hi2c_->hdmarx != NULL) {
                HAL_DMA_Abort(hi2c_->hdmarx); // 中止I2C接收DMA通道
            }
            if (hi2c_->hdmatx != NULL) {
                HAL_DMA_Abort(hi2c_->hdmatx); // 中止I2C发送DMA通道
            }
            // 重置I2C状态机为就绪，避免后续传输卡住
            hi2c_->State = HAL_I2C_STATE_READY;
            hi2c_->ErrorCode = HAL_I2C_ERROR_NONE;
            return HAL_TIMEOUT;
        }
    }

    if (mpu6050_i2c_dma_error) return HAL_ERROR;
    return HAL_OK;
}

// 读取原始数据
void MPU6050::readRawData() {
    uint8_t buf[14] = {0};

    // 连续读取加速度+温度+陀螺仪
    readMulti(MPU6050_ACCEL_XOUT_H, buf, 14);

    // 加速度
    raw_.accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    raw_.accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    raw_.accel[2] = (int16_t)((buf[4] << 8) | buf[5]);

    // 温度
    raw_.temp = (int16_t)((buf[6] << 8) | buf[7]);

    // 陀螺仪
    raw_.gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
    raw_.gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    raw_.gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

// 数据转换
void MPU6050::convertData() {
    // 加速度转换 g
    real_.accel[0] = raw_.accel[0] * MPU6050_ACCEL_2G_SEN;
    real_.accel[1] = raw_.accel[1] * MPU6050_ACCEL_2G_SEN;
    real_.accel[2] = raw_.accel[2] * MPU6050_ACCEL_2G_SEN;

    // 陀螺仪转换 °/s
    real_.gyro[0] = raw_.gyro[0] * MPU6050_GYRO_250DPS_SEN;
    real_.gyro[1] = raw_.gyro[1] * MPU6050_GYRO_250DPS_SEN;
    real_.gyro[2] = raw_.gyro[2] * MPU6050_GYRO_250DPS_SEN;

    // 温度转换 ℃
    real_.temp = (float)raw_.temp * MPU6050_TEMP_FACTOR + MPU6050_TEMP_OFFSET;
}

// 对外读取接口
void MPU6050::read(float gyro[3], float accel[3], float* temperature) {
    readRawData();
    convertData();

    accel[0] = real_.accel[0];
    accel[1] = real_.accel[1];
    accel[2] = real_.accel[2];
    gyro[0] = real_.gyro[0];
    gyro[1] = real_.gyro[1];
    gyro[2] = real_.gyro[2];
    *temperature = real_.temp;
}

// C语言接口实现
extern "C" {
    // 实例化MPU6050 (I2C1, 地址0x68<<1)
    static MPU6050 mpu6050_instance(&hi2c1, 0x68 << 1);

    void MPU6050_Init() {
        mpu6050_instance.init();
    }

    void MPU6050_Read(float gyro[3], float accel[3], float* temperature) {
        mpu6050_instance.read(gyro, accel, temperature);
    }

    // I2C DMA完成回调
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
        if (hi2c == &hi2c1) {
            mpu6050_i2c_dma_done = 1;
        }
    }

    // I2C错误回调
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
        if (hi2c == &hi2c1) {
            mpu6050_i2c_dma_error = 1;
        }
    }
}