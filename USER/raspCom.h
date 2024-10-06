/**
 * @file raspCom.h
 * @author Zhou Xu
 * @brief 树莓派通信程序
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/**
 * 树莓派通信协议介绍 
 * 树莓派发送0xA5 1bit(成功/失败) 1bit（红/蓝）2bit0（2bit（识别到的类型）x6）0x5A
 * 单片机发送 0xA5 1byte（char 1）0x5A
 * 
 */

#ifndef RASPCOM_H
#define RASPCOM_H

#include "stm32f1xx_hal.h"
#include "remote_ctrl.h"
#include "arm.h"
#include "chassis.h"

#define RASP_COM_HEAD 0xA5
#define RASP_COM_TAIL 0x5A

//通信协议结构体
typedef struct
{
    unsigned char result;
    unsigned char color;
    unsigned char type[6]; 
} RaspCom_t;

void RaspCom_Init(void);
void RaspCom_Send(RaspCom_t *rasp_com);
void RaspCom_Receive(RaspCom_t *rasp_com);

#endif

