/**
 * @file chassis.h
 * @author Zhou Xu
 * @brief 
 * @version 0.1
 * @date 2024-10-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CHASSIS_H
#define CHASSIS_H
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#define TIM_ARR 2000

typedef struct
{
    int16_t speed;
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    GPIO_TypeDef * gpio; //GPIO用来换向
    uint16_t pin;
    GPIO_PinState state;
    
} Motor_t;

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vw;

    Motor_t motor[4];

} ChassisSpeed_t;



void Chassis_Init(void);
void Chassis_Control(void);
void Chassis_SetSpeed(int16_t vx, int16_t vy, int16_t vw);
void Chassis_Mecnum_Calc(int16_t vx, int16_t vy, int16_t vw);
void Chassis_Output(Motor_t  * motor);
void Chassis_SetOmega(int16_t vw);


#endif
