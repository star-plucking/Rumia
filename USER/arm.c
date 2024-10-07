/**
 * @file arm.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "arm.h"

Arm_Motor_t motor_j1;
Arm_Servo_t servo_j2;
Arm_Gripper_t gripper;
Encoder_t encoder_j1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/**
 * 鉴于操控模式的需求，需要给机械臂增加固定点功能，用于标定机械臂的位置；
 * 此处将使用一个数组来存储固定点的位置，不存储夹爪的位置；
 * 机械臂的位置将使用角度制记录。
 */
float fixed_point[4][2] = {
   {53.9781799f, 226.881f},
   {36.0945473f, 13.0f},
   {35.5709076f, 13.0f},
   {34.2f, 240.0f},
};

/**
 * @brief 小臂初始化
 * @param None
 * @retval None
 *
 */
void Arm_Init(void)
{
    motor_j1.speed = 0;
    motor_j1.angle = 0;
    motor_j1.angle_ref = 22;
    motor_j1.circles = 0;
    motor_j1.htim = &htim4;
    motor_j1.channel = TIM_CHANNEL_1;
    motor_j1.gpio = GPIOA;
    motor_j1.pin = GPIO_PIN_15;
    motor_j1.state = GPIO_PIN_RESET;

    PID_Init(&motor_j1.pid, 6000.0f, 0.005f, 0.0f, 20000.0f, -20000.0f);

    servo_j2.angle = 2400.0f;
    servo_j2.htim = &htim4;
    servo_j2.channel = TIM_CHANNEL_3;

    gripper.angle =  GRIPPER_OPEN;
    gripper.htim = &htim4;
    gripper.channel = TIM_CHANNEL_4;

    encoder_j1.encoder_lines = MOTOR_J1_ENCODER_LINES;
		encoder_j1.htim = &htim2;
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(motor_j1.htim, motor_j1.channel);
    HAL_TIM_PWM_Start(servo_j2.htim, servo_j2.channel);
    HAL_TIM_PWM_Start(gripper.htim, gripper.channel);
}

/**
 * @brief 设置机械臂角度
 *
 * @param angle_j1
 * @param angle_j2
 */
void Arm_SetAngle(float angle_j1, float angle_j2)
{
    motor_j1.angle_ref = angle_j1 ;
    servo_j2.angle = angle_j2 / 270.0f * 2000 + 500;
}

/**
 * @brief 设置机械臂速度
 *
 * @param state
 */
void Arm_SetGripper(uint8_t state)
{
    if (state == ARM_GREPPER_OPEN)
    {
        gripper.angle = GRIPPER_OPEN;
    }
    else if (state == ARM_GREPPER_CLOSE)
    {
        gripper.angle = GRIPPER_CLOSE;
    }
}

/**
 * @brief 机械臂输出
 *
 */
void Arm_output(void)
{
    __HAL_TIM_SET_COMPARE(servo_j2.htim, servo_j2.channel, servo_j2.angle);
    __HAL_TIM_SET_COMPARE(gripper.htim, gripper.channel, gripper.angle);

    if (motor_j1.speed < 0)
    {
        motor_j1.state = GPIO_PIN_SET;
        __HAL_TIM_SetCompare(motor_j1.htim, motor_j1.channel, 20000 + motor_j1.speed);
        HAL_GPIO_WritePin(motor_j1.gpio, motor_j1.pin, motor_j1.state);
    }
    else if (motor_j1.speed >= 0)
    {
        motor_j1.state = GPIO_PIN_RESET;
        __HAL_TIM_SetCompare(motor_j1.htim, motor_j1.channel, motor_j1.speed);
        HAL_GPIO_WritePin(motor_j1.gpio, motor_j1.pin, motor_j1.state);
    }
}

/**
 * @brief 机械臂控制
 *
 */
void Arm_Control(void)
{
    Motor_J1_SetOutput();
    Arm_output();
}

/**
 * @brief 机械臂电机1速度设置
 *
 * @param speed
 */
void Motor_J1_SetSpeed(int16_t speed)
{
    motor_j1.speed = speed;
}

/**
 * @brief 机械臂电机1速度读取
 * @param None
 * @retval None
 * 
 */
void Motor_J1_GetEncoderSpeed(void)
{
    motor_j1.speed = encoder_j1.speed;
}

/**
 * @brief 机械臂电机1角度计算,根据电机编码器转过的圈数和编码器线数计算角度，使用DECRACIO作为减速比
 *        霍尔码盘转 MOTOR_J1_ENCODER_LINES * MOTOR_J1_DECRACIO 为主轴输出一圈
 * @param None
 * @retval None
 * 
 */
void Motor_J1_CalAngle(void)
{
    if(encoder_j1.direction == 1)
    {
        motor_j1.circles++;
    }
    else if(encoder_j1.direction == -1)
    {
        motor_j1.circles--;
    }
    motor_j1.angle = ((float)(encoder_j1.counter + motor_j1.circles * 65535) / (float)(MOTOR_J1_ENCODER_LINES * MOTOR_J1_DECRACIO * 6.25f)) * 360.0f;
}

/**
 * @brief 电机编码器读取，严格按照1ms读取一次
 * @param None
 * @retval None
 * 
 */
void Motor_ReadEncoder(void)
{
    encoder_j1.last_counter = encoder_j1.counter;
    encoder_j1.counter = __HAL_TIM_GET_COUNTER(&htim2);
    if(encoder_j1.counter - encoder_j1.last_counter > 60000)
    {
        encoder_j1.direction = -1;
        encoder_j1.speed = (encoder_j1.counter - (encoder_j1.last_counter + 65535)) * 360.0f / encoder_j1.encoder_lines / MOTOR_J1_DECRACIO / 1000.0f;
    }
    else if(encoder_j1.counter - encoder_j1.last_counter < -60000)
    {
        encoder_j1.direction = 1;
        encoder_j1.speed = (encoder_j1.counter - encoder_j1.last_counter + 65535) * 360.0f / encoder_j1.encoder_lines / MOTOR_J1_DECRACIO / 1000.0f;
    }
    else
    {
        encoder_j1.direction = 0;
        encoder_j1.speed = (encoder_j1.counter - encoder_j1.last_counter) * 360.0f / encoder_j1.encoder_lines / MOTOR_J1_DECRACIO / 1000.0f;
    }  
}

/**
 * @brief 基于电机角度的单级PID控制
 * 
 * @param None
 * 
 */
void Motor_J1_SetOutput(void)
{
    PID_Calc(&motor_j1.pid, motor_j1.angle_ref, motor_j1.angle);
    Motor_J1_SetSpeed(motor_j1.pid.output);
}
