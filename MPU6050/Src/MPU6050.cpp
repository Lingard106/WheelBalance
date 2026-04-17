#include "MPU6050.h"

// ========== 外部I2C句柄声明（由用户在主文件中定义） ==========
extern I2C_HandleTypeDef hi2c1;  // 根据实际使用的I2C修改

// ========== 静态实例（供C接口使用） ==========
static MPU6050 mpu6050_instance(&hi2c1, MPU6050_ADDR_DEFAULT);

// ========== 构造函数 ==========
MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr)
    : hi2c_(hi2c), i2c_addr_(i2c_addr), dma_done_(0), dma_error_(0) {
}

// ========== 初始化公共接口 ==========
uint8_t MPU6050::init() {
    return sensorInit();
}

// ========== 传感器初始化（内部） ==========
uint8_t MPU6050::sensorInit() {
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status;

    // 1. 检查设备是否存在
    status = readReg(MPU6050_WHO_AM_I, &who_am_i);
    if (status != HAL_OK || who_am_i != 0x68) {
        return MPU6050_NO_SENSOR;
    }

    // 2. 软件复位
    status = writeReg(MPU6050_PWR_MGMT_1, MPU6050_RESET_VALUE);
    if (status != HAL_OK) return MPU6050_I2C_ERROR;
    HAL_Delay(100);  // 等待复位完成

    // 3. 唤醒并设置时钟源
    status = writeReg(MPU6050_PWR_MGMT_1, MPU6050_WAKE_VALUE);
    if (status != HAL_OK) return MPU6050_PWR_ERROR;
    HAL_Delay(10);

    // 4. 禁用所有轴待机（PWR_MGMT_2=0x00）
    status = writeReg(MPU6050_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return MPU6050_PWR_ERROR;

    // 5. 配置采样率分频（1kHz）
    status = writeReg(MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_1000HZ);
    if (status != HAL_OK) return MPU6050_SMPLRT_ERROR;

    // 6. 配置数字低通滤波器（20Hz）
    status = writeReg(MPU6050_CONFIG, MPU6050_FILTER_20HZ);
    if (status != HAL_OK) return MPU6050_CONFIG_ERROR;

    // 7. 配置陀螺仪量程（±250°/s）
    status = writeReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_250DPS);
    if (status != HAL_OK) return MPU6050_GYRO_RANGE_ERROR;

    // 8. 配置加速度计量程（±2g）
    status = writeReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_2G);
    if (status != HAL_OK) return MPU6050_ACCEL_RANGE_ERROR;

    return MPU6050_NO_ERROR;
}

// ========== 检查设备连接 ==========
bool MPU6050::isConnected() {
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status = readReg(MPU6050_WHO_AM_I, &who_am_i);
    return (status == HAL_OK && who_am_i == 0x68);
}

// ========== 单字节写寄存器 ==========
HAL_StatusTypeDef MPU6050::writeReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// ========== 单字节读寄存器 ==========
HAL_StatusTypeDef MPU6050::readReg(uint8_t reg, uint8_t* data) {
    return HAL_I2C_Mem_Read(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

// ========== 阻塞式多字节读取 ==========
HAL_StatusTypeDef MPU6050::readMultiReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (len == 0 || len > MPU6050_DMA_MAX_LEN) return HAL_ERROR;
    return HAL_I2C_Mem_Read(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

// ========== DMA 多字节读取（可选功能） ==========
HAL_StatusTypeDef MPU6050::readMultiRegDMA(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!buf || len == 0 || len > MPU6050_DMA_MAX_LEN) return HAL_ERROR;

    dma_done_ = 0;
    dma_error_ = 0;

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(hi2c_, i2c_addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
    if (ret != HAL_OK) return ret;

    // 等待 DMA 完成或超时
    uint32_t tickstart = HAL_GetTick();
    while (!dma_done_ && !dma_error_) {
        if ((HAL_GetTick() - tickstart) > MPU6050_I2C_TIMEOUT_MS) {
            // 超时：中止DMA并恢复I2C状态
            HAL_I2C_Master_Abort_IT(hi2c_, i2c_addr_);
            return HAL_TIMEOUT;
        }
    }

    if (dma_error_) return HAL_ERROR;
    return HAL_OK;
}

// ========== 读取原始数据（阻塞） ==========
bool MPU6050::readRawData() {
    uint8_t buf[14];
    HAL_StatusTypeDef status = readMultiReg(MPU6050_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return false;

    raw_.accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    raw_.accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    raw_.accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    raw_.temp    = (int16_t)((buf[6] << 8) | buf[7]);
    raw_.gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
    raw_.gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    raw_.gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

// ========== 数据转换 ==========
void MPU6050::convertData() {
    real_.accel[0] = raw_.accel[0] * MPU6050_ACCEL_2G_SEN;
    real_.accel[1] = raw_.accel[1] * MPU6050_ACCEL_2G_SEN;
    real_.accel[2] = raw_.accel[2] * MPU6050_ACCEL_2G_SEN;

    real_.gyro[0] = raw_.gyro[0] * MPU6050_GYRO_250DPS_SEN;
    real_.gyro[1] = raw_.gyro[1] * MPU6050_GYRO_250DPS_SEN;
    real_.gyro[2] = raw_.gyro[2] * MPU6050_GYRO_250DPS_SEN;

    real_.temp = raw_.temp * MPU6050_TEMP_FACTOR + MPU6050_TEMP_OFFSET;
}

// ========== 一站式读取（阻塞） ==========
void MPU6050::read(float gyro[3], float accel[3], float* temperature) {
    if (!readRawData()) {
        // 通信失败，返回零值
        accel[0] = accel[1] = accel[2] = 0.0f;
        gyro[0] = gyro[1] = gyro[2] = 0.0f;
        if (temperature) *temperature = 0.0f;
        return;
    }
    convertData();

    accel[0] = real_.accel[0];
    accel[1] = real_.accel[1];
    accel[2] = real_.accel[2];
    gyro[0] = real_.gyro[0];
    gyro[1] = real_.gyro[1];
    gyro[2] = real_.gyro[2];
    if (temperature) *temperature = real_.temp;
}

// ========== C 语言接口实现 ==========
extern "C" {

uint8_t MPU6050_Init(void) {
    return mpu6050_instance.init();
}

uint8_t MPU6050_IsConnected(void) {
    return mpu6050_instance.isConnected() ? 1 : 0;
}

void MPU6050_Read(float gyro[3], float accel[3], float* temperature) {
    mpu6050_instance.read(gyro, accel, temperature);
}

// ========== I2C DMA 回调（若使用DMA） ==========
// 注意：这些回调与具体的 I2C 句柄绑定，用户需确保 hi2c1 存在
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        mpu6050_instance.dma_done_ = 1;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        mpu6050_instance.dma_error_ = 1;
    }
}

} // extern "C"