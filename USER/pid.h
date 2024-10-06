/**
 * @file pid.h
 * @author Zhou Xu
 * @brief 经典PID算法
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
    float max_output;
    float min_output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output, float min_output);
void PID_Calc(PID_t *pid, float target, float measure);

#endif