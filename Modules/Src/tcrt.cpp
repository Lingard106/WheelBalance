//
// Created by lingard on 2026/4/18.
//

#include "../Inc/tcrt.h"

// ========== 根据你的硬件修改引脚定义 ==========
// 假设使用 GPIOA 的 PIN1 ~ PIN5
static const uint16_t tcrt_pins[TCRT_CHANNEL_COUNT] = {
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5
};
static TCRT tcrt_instance(GPIOA, tcrt_pins);

// ========== 构造函数 ==========
TCRT::TCRT(GPIO_TypeDef* port, const uint16_t pins[TCRT_CHANNEL_COUNT])
    : port_(port)
{
    for (int i = 0; i < TCRT_CHANNEL_COUNT; ++i) {
        pins_[i] = pins[i];
        state_[i] = 1;   // 初始假定未检测到黑线（高电平）
    }
}

// ========== 初始化 ==========
void TCRT::init() {
    // GPIO 已在 CubeMX 中配置为输入、上拉、双边沿中断，此处仅同步初始状态
    for (int i = 0; i < TCRT_CHANNEL_COUNT; ++i) {
        state_[i] = (HAL_GPIO_ReadPin(port_, pins_[i]) == GPIO_PIN_SET) ? 1 : 0;
    }
}

// ========== 获取所有通道状态 ==========
void TCRT::getAllStates(uint8_t states[TCRT_CHANNEL_COUNT]) const {
    for (int i = 0; i < TCRT_CHANNEL_COUNT; ++i) {
        states[i] = state_[i];
    }
}

// ========== 中断处理 ==========
void TCRT::handleInterrupt(uint16_t pin) {
    for (int i = 0; i < TCRT_CHANNEL_COUNT; ++i) {
        if (pins_[i] == pin) {
            // 读取当前引脚电平：高电平 -> 1（未触发），低电平 -> 0（触发）
            state_[i] = (HAL_GPIO_ReadPin(port_, pin) == GPIO_PIN_SET) ? 1 : 0;
            break;
        }
    }
}

// ========== C 语言接口 ==========
extern "C" {

    void TCRT_Init(void) {
        tcrt_instance.init();
    }

    void TCRT_GetAllStates(uint8_t states[TCRT_CHANNEL_COUNT]) {
        tcrt_instance.getAllStates(states);
    }

    // ========== 全局中断回调 ==========
    //注意：请将此回调放在 stm32f1xx_it.c 或统一的中断处理文件中，且只保留一份！
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        tcrt_instance.handleInterrupt(GPIO_Pin);
        // 如有其他引脚中断，可在此添加 else if 分支
    }

} // extern "C"
  

