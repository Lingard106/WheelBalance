//
// Created by lingard on 2026/1/24.
//

#ifndef MECANUM_MPU6050_READ_H
#define MECANUM_MPU6050_READ_H

#pragma once
#include "TaskBase.h"
#include "FusionAHRS.h"
#include <cmath>

#ifdef __cplusplus
class MPU6050ReadTask : public TaskBase
{
    public: void run() override;
private:
    FusionAHRS ahrs_{1000.0f};  // 采样频率 1000Hz，需与实际调用频率一致
     float euler_[3]; // 存储欧拉角 roll, pitch, yaw (单位：度)

};
#endif

#ifdef __cplusplus
extern "C"{
#endif
    void MPU6050Read_Init();

#ifdef __cplusplus
}
#endif

#endif //MECANUM_MPU6050_READ_H