/**
 * @file remote_ctrl.h
 * @author Zhou Xu
 * @brief 遥控器控制程序和数据包解析
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef REMOTE_CTRL_H
#define REMOTE_CTRL_H

#include "stm32f1xx_hal.h"
#include "valuepack.h"
#include "chassis.h"
#include "arm.h"
#include "raspCom.h"

typedef enum
{
    REMOTE_CTRL_MODE_MANUL = 0,
    REMOTE_CTRL_MODE_AUTO = 1
} Remote_CtrlMode_t;



void Remote_Init(void);
void Remote_Ctrl(void);

#endif