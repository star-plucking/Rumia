/**
 * @file arm.h
 * @author Zhou Xu
 * @brief 一个二轴带末端夹爪执行机构的驱动程序，J1为直流电机，J2为舵机
 * @version 0.1
 * @date 2024-10-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ARM_H
#define ARM_H

#include "stm32f1xx_hal.h"
#include "pid.h"

#define GRIPPER_OPEN 1000
#define GRIPPER_CLOSE 1900

#define MOTOR_J1_DECRACIO 90.0f
#define MOTOR_J1_ENCODER_LINES 22

typedef struct
{
    int16_t speed;
    float angle;     // 输出轴角度
    float angle_ref; // 角度设定值
    int16_t circles; // 驱动轴圈数
    

    TIM_HandleTypeDef *htim;
    uint32_t channel;

    GPIO_TypeDef *gpio; // GPIO用来换向
    uint16_t pin;
    GPIO_PinState state;

    PID_t pid;
} Arm_Motor_t;

typedef struct
{
    float angle;
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} Arm_Servo_t;

typedef struct
{
    float angle;
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} Arm_Gripper_t;

typedef struct
{
    float speed;
    uint16_t encoder_lines;
    TIM_HandleTypeDef *htim;
    uint32_t ch;
    int8_t direction;
    int64_t counter;
    int64_t last_counter;
} Encoder_t;

typedef enum
{
    ARM_GREPPER_OPEN = 0,
    ARM_GREPPER_CLOSE = 1
} Arm_GripperState_t;



void Arm_Init(void);
void Arm_SetAngle(float angle_j1, float angle_j2);
void Arm_output(void);
void Arm_SetGripper(uint8_t state);
void Arm_Control(void);

void Motor_J1_SetSpeed(int16_t speed);
void Motor_J1_SetOutput(void);
void Motor_J1_GetEncoderSpeed(void);
void Motor_J1_CalAngle(void);

void Motor_ReadEncoder(void);

#endif
