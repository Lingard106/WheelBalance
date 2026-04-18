//
// Created by lingard on 2026/4/18.
//

#ifndef TWOWHEELBALCANCE_TRACK_TASK_H
#define TWOWHEELBALCANCE_TRACK_TASK_H

#pragma once
#include "TaskBase.h"


#ifdef __cplusplus
class TrakTask : public TaskBase
{
public: void run() override;
};
#endif

#ifdef __cplusplus
extern "C"{
#endif
    void TrakTask_Init();

#ifdef __cplusplus
}
#endif

#endif //TWOWHEELBALCANCE_TRACK_TASK_H