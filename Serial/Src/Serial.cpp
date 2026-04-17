//
// Created by 34254 on 2026/1/12.
//

#include "../Inc/Serial.h"
#include "usart.h"
#include "cstring"
extern float p;
extern float i;
extern float d;
extern "C" {
    Serial serial(&huart1, &p, &i, &d);
    void Serial_UART_IDLE_Callback() {
        serial.UART_IDLE_Callback();
    }
}
Serial::Serial(UART_HandleTypeDef * huart,float* p,float* i,float* d): huart_(huart),p(p),i(i),d(d) {
    tail = 0;
    head = 0;
    uint8_t tail_bytes[4] = { 0x00, 0x00, 0x80, 0x7F }; // 这是内存顺序（小端：低字节先）
    memcpy(data[3].b, tail_bytes, 4);
}

void Serial::write(float x, float y, float z) {

    data[0].f = x;
    data[1].f = y;
    data[2].f = z;
    HAL_UART_Transmit_DMA(huart_, (uint8_t *)data, sizeof(data));

}

void Serial::receive_init() {
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart_, buffer, BUFFER_MAX_SIZE);
}

void Serial::UART_IDLE_Callback() {
    // 在外部调用__HAL_UART_CLEAR_IDLEFLAG(huart_);
    HAL_UART_DMAStop(huart_);
    // uint32_t received_length = BUFFER_MAX_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    // head = (head + received_length) % BUFFER_MAX_SIZE;
    process_data();
    HAL_UART_Receive_DMA(huart_, buffer, BUFFER_MAX_SIZE);
}

void Serial::process_data() {

    // 计算当前DMA写入位置（tail）
    uint32_t remain = __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    tail = (BUFFER_MAX_SIZE - remain) % BUFFER_MAX_SIZE;

    // 计算接收到的数据长度
    uint16_t received_len;
    if (tail >= head)
    {
        // 无绕回：head → tail
        received_len = tail - head;
    }
    else
    {
        // 有绕回：从head到缓冲区末尾 + 从缓冲区开始到tail
        received_len = (BUFFER_MAX_SIZE - head) + tail;
    }

    if (received_len > 0)
    {
        // 处理环形绕回情况
        if (tail > head)
        {
            // 情况1：数据连续，无绕回
            Process_buffer_CallBack(&buffer[head], received_len);
        }
        else
        {
            // 情况2：数据绕回，分两段处理
            // 第一段：从head到缓冲区末尾
            uint16_t first_part_len = BUFFER_MAX_SIZE - head;
             Process_buffer_CallBack(&buffer[ head], first_part_len);

            // 第二段：从缓冲区开始到tail
            if ( tail > 0)
            {
                 Process_buffer_CallBack(& buffer[0],  tail);
            }
        }

        // 更新head位置
         head =  tail;
    }
}

void Serial::Process_buffer_CallBack(uint8_t* buffer, uint16_t length) {
    uint8_t* ptr = buffer;
    uint8_t* end_ptr = buffer + length;
    for (uint8_t* ptr = buffer; ptr != end_ptr; ptr++) {
        // 处理每个字节
        if (ptr[0] == 0x00) {
            uint8_t temp[4];
            switch (ptr[1]) {
                case 0x01:
                    // 处理第一个浮点数
                    temp[0] = ptr[2];
                    temp[1] = ptr[3];
                    temp[2] = ptr[4];
                    temp[3] = ptr[5];
                    memcpy(p,&temp,4);
                    break;
                case 0x02:
                    // 处理第二个浮点数
                    temp[0] = ptr[2];
                    temp[1] = ptr[3];
                    temp[2] = ptr[4];
                    temp[3] = ptr[5];
                    memcpy(i,&temp,4);
                    break;
                case 0x03:
                    // 处理第三个浮点数
                    temp[0] = ptr[2];
                    temp[1] = ptr[3];
                    temp[2] = ptr[4];
                    temp[3] = ptr[5];
                    memcpy(d,&temp,4);
                    break;
            }
            break;
        }
    }
}