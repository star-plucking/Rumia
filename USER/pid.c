/**
 * @file pid.c
 * @author Zhou Xu
 * @brief 一个经典PID算法的实现
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "pid.h"

/**
 * @brief PID初始化
 * 
 * @param pid 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param max_output 
 * @param min_output 
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output, float min_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}

/**
 * @brief PID计算
 * 
 * @param pid 
 * @param target 
 * @param measure 
 */
void PID_Calc(PID_t *pid, float target, float measure)
{
    pid->error = target - measure;
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->last_error;
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < pid->min_output)
    {
        pid->output = pid->min_output;
    }
    pid->last_error = pid->error;
}

