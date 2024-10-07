/**
 * @file chassis.c
 * @author Zhou Xu
 * @brief 
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis.h"


/*---------------麦轮解算示意图-----------------
                   0   1
                     ↑
                   3   2
--------------------END---------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

ChassisSpeed_t chassis_speed;

/**
 * @brief 底盘初始化代码
 * @param None
 * @retval None
 * @note   None
 */
void Chassis_Init(void)
{
    chassis_speed.vx = 0;
    chassis_speed.vy = 0;
    chassis_speed.vw = 0;

    chassis_speed.motor[1].htim = &htim1;
    chassis_speed.motor[1].channel = TIM_CHANNEL_1;
    chassis_speed.motor[1].gpio = GPIOA;
    chassis_speed.motor[1].pin = GPIO_PIN_9;
    chassis_speed.motor[1].state = GPIO_PIN_RESET;

    chassis_speed.motor[2].htim = &htim3;
    chassis_speed.motor[2].channel = TIM_CHANNEL_2;
    chassis_speed.motor[2].gpio = GPIOA;
    chassis_speed.motor[2].pin = GPIO_PIN_6;
    chassis_speed.motor[2].state = GPIO_PIN_RESET;

    chassis_speed.motor[3].htim = &htim1;
    chassis_speed.motor[3].channel = TIM_CHANNEL_3;
    chassis_speed.motor[3].gpio = GPIOA;
    chassis_speed.motor[3].pin = GPIO_PIN_11;
    chassis_speed.motor[3].state = GPIO_PIN_RESET;

    chassis_speed.motor[0].htim = &htim3;
    chassis_speed.motor[0].channel = TIM_CHANNEL_4;
    chassis_speed.motor[0].gpio = GPIOB;
    chassis_speed.motor[0].pin = GPIO_PIN_0;
    chassis_speed.motor[0].state = GPIO_PIN_RESET;



    HAL_TIM_PWM_Start(chassis_speed.motor[0].htim, chassis_speed.motor[0].channel);
    HAL_TIM_PWM_Start(chassis_speed.motor[1].htim, chassis_speed.motor[1].channel);
    HAL_TIM_PWM_Start(chassis_speed.motor[2].htim, chassis_speed.motor[2].channel);
    HAL_TIM_PWM_Start(chassis_speed.motor[3].htim, chassis_speed.motor[3].channel);

    HAL_GPIO_WritePin(chassis_speed.motor[0].gpio, chassis_speed.motor[0].pin, chassis_speed.motor[0].state);
    HAL_GPIO_WritePin(chassis_speed.motor[1].gpio, chassis_speed.motor[1].pin, chassis_speed.motor[1].state);
    HAL_GPIO_WritePin(chassis_speed.motor[2].gpio, chassis_speed.motor[2].pin, chassis_speed.motor[2].state);
    HAL_GPIO_WritePin(chassis_speed.motor[3].gpio, chassis_speed.motor[3].pin, chassis_speed.motor[3].state);


    HAL_Delay(1); //延时1ms
}

/**
 * @brief 底盘参数设置
 * 
 * @param vx 
 * @param vy 
 * @param vw 
 */
void Chassis_SetSpeed(int16_t vx, int16_t vy, int16_t vw)
{
    chassis_speed.vx = vx;
    chassis_speed.vy = vy;
    chassis_speed.vw = vw;
}

/**
 * @brief 底盘麦轮解算
 * 
 * @param vx 
 * @param vy 
 * @param vw 
 * @param motor 
 */
void Chassis_Mecnum_Calc(int16_t vx, int16_t vy, int16_t vw)
{
    chassis_speed.motor[0].speed = -(vx + vy + vw);
    chassis_speed.motor[1].speed = vx - vy + vw;
    chassis_speed.motor[2].speed = -vx - vy + vw;
    chassis_speed.motor[3].speed = -vx + vy + vw;
}

/**
 * @brief 底盘输出
 * 
 * @param motor 
 */
void Chassis_Output(Motor_t * motor)
{
    for (int i = 0; i < 4; i++)
    {
        if(motor[i].speed <= 0)
        {
            motor[i].state = GPIO_PIN_SET;
            __HAL_TIM_SetCompare(chassis_speed.motor[i].htim, chassis_speed.motor[i].channel, TIM_ARR + motor[i].speed);
            HAL_GPIO_WritePin(chassis_speed.motor[i].gpio, chassis_speed.motor[i].pin, chassis_speed.motor[i].state);
        }
        else if(motor[i].speed > 0)
        {
            motor[i].state = GPIO_PIN_RESET;
            __HAL_TIM_SetCompare(chassis_speed.motor[i].htim, chassis_speed.motor[i].channel, motor[i].speed);
            HAL_GPIO_WritePin(chassis_speed.motor[i].gpio, chassis_speed.motor[i].pin, chassis_speed.motor[i].state);
        }
    }
}

/**
 * @brief 底盘控制
 * 
 */
void Chassis_Control(void)
{
    Chassis_Mecnum_Calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.vw);
    Chassis_Output(chassis_speed.motor);
}

/**
 * @brief 底盘角速度设置
 * 
 * @param vw 
 */
void Chassis_SetOmega(int16_t vw)
{
    chassis_speed.vw = vw;
}

