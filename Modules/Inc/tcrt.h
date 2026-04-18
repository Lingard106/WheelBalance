//
// Created by lingard on 2026/4/18.
//

#ifndef TWOWHEELBALCANCE_TCRT_H
#define TWOWHEELBALCANCE_TCRT_H

#include "stm32f1xx.h"

#include <stdio.h>

#define TCRT_CHANNEL_COUNT  5   // 对管数量

#ifdef __cplusplus
class TCRT {
public:
    // 构造函数：传入 GPIO 端口和引脚数组
    TCRT(GPIO_TypeDef* port, const uint16_t pins[TCRT_CHANNEL_COUNT]);

    // 初始化（可读取初始状态）
    void init();

    // 获取所有通道状态（0 = 检测到黑线，1 = 未检测到）
    void getAllStates(uint8_t states[TCRT_CHANNEL_COUNT]) const;

    // 获取状态数组指针（供中断回调快速写入）
    volatile uint8_t* getStateArray() { return state_; }

    // 中断处理函数（由 HAL_GPIO_EXTI_Callback 调用）
    void handleInterrupt(uint16_t pin);

private:
    GPIO_TypeDef* port_;
    uint16_t pins_[TCRT_CHANNEL_COUNT];
    volatile uint8_t state_[TCRT_CHANNEL_COUNT];   // 1=未触发(高电平)，0=触发(低电平)
};

// ========== C 语言接口 ==========
extern "C" {
#endif

    void TCRT_Init(void);
    void TCRT_GetAllStates(uint8_t states[TCRT_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif


#endif //TWOWHEELBALCANCE_TCRT_H