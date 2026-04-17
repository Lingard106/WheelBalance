//
// Created by 34254 on 2026/1/12.
//

#ifndef VOFA_UART_SERIAL_H
#define VOFA_UART_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
void Serial_UART_IDLE_Callback();
#ifdef __cplusplus
}
#endif

#include "main.h"
#define BUFFER_MAX_SIZE 512

#ifdef __cplusplus
typedef union {
    float f;
    uint8_t b[4];
} send_data;

class Serial {
    send_data data[4]; //发送数据缓存区
    uint8_t frame_tail[4]; //数据帧尾

    uint8_t buffer[BUFFER_MAX_SIZE]; //环形缓冲区
    uint32_t tail; //尾指针
    uint8_t head; //头指针

    float *p;
    float *i;
    float *d;

    UART_HandleTypeDef* huart_; //串口句柄指针
public:
    Serial(UART_HandleTypeDef * huart,float* p,float* i,float* d);

    void write(float x, float y, float z); //写入数据到发送缓存区

    void receive_init(); //开启空闲中断接收

    void UART_IDLE_Callback(); //空闲中断回调函数

    void process_data(); //处理接收到的数据

    void Process_buffer_CallBack(uint8_t* buffer, uint16_t length);
};
#endif

#endif //VOFA_UART_SERIAL_H