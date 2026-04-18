//
// Created by lingard on 2026/1/24.
//

#include "../Inc/MPU6050_read.h"
#include "MPU6050.h"
#include "bsp_dwt.h"
#include "debug_vars.h"
// 将四元数转换为欧拉角 (roll, pitch, yaw) 单位：度
static void quat2Euler(const float q[4], float euler[3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f)
        euler[1] = std::copysign(3.1415926f / 2.0f, sinp);
    else
        euler[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);

    // 转换为度
    const float rad2deg = 180.0f / 3.1415926f;
    euler[0] *= rad2deg;
    euler[1] *= rad2deg;
    euler[2] *= rad2deg;
}

void MPU6050ReadTask::run()
{
    MPU6050_Init();
    DWT_Delay_ms(1000);

    float temp;
    float gyro[3], accel[3];
    float q[4];           // 四元数




    for (;;)
    {
        MPU6050_Read(gyro,accel,&temp);
        // 必须转换为弧度每秒
        const float deg2rad = 3.1415926f / 180.0f;
        float gx = gyro[0] * deg2rad;
        float gy = gyro[1] * deg2rad;
        float gz = gyro[2] * deg2rad;

        // 加速度单位 g 无需转换（算法内部会归一化）
        float ax = accel[0];
        float ay = accel[1];
        float az = accel[2];

        // 更新 AHRS
        ahrs_.update(gx, gy, gz, ax, ay, az);

        // 获取四元数
        std::array<float, 4> quat = ahrs_.getQuaternion();
        q[0] = quat[0]; // w
        q[1] = quat[1]; // x
        q[2] = quat[2]; // y
        q[3] = quat[3]; // z

        // 转换为欧拉角 (roll, pitch, yaw)，单位度
        quat2Euler(q, euler_);

        // 输出到调试变量
        debug_roll = euler_[0];   // roll
        debug_pitch = euler_[1];   // pitch
        debug_yaw = euler_[2];   // yaw

        osDelay(1);
    }

}
extern "C" {
static MPU6050ReadTask MPU6050Task;

    void MPU6050ReadTask_Init()
    {
        MPU6050Task.start((char*)"MPU6050ReadTask",128,osPriorityHigh);
    }

}
