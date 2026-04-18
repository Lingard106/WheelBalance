//
// Created by lingard on 2026/4/18.
//

#include "../Inc/Track_Task.h"
#include "tcrt.h"
#include "debug_vars.h"


void TrakTask::run()
{
    TCRT_Init();
    uint8_t tube_states[5];
    for (;;)
    {
        TCRT_GetAllStates(tube_states);

        // tube_states[i] == 0 表示第 i 路检测到黑线
        // 可将其赋值给调试变量或进行循迹控制
        debug_tube1 = tube_states[0];
        debug_tube2 = tube_states[1];
        debug_tube3 = tube_states[2];
        debug_tube4 = tube_states[3];
        debug_tube5 = tube_states[4];
        osDelay(5);
    }


}
extern "C" {
static TrakTask trakTask;

    void TrakTask_Init()
    {
        trakTask.start((char*)"TrakTask",128,osPriorityHigh);
    }

}
