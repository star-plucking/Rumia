/**
 * @file remote_ctrl.c
 * @author Zhou Xu
 * @brief 遥控器控制程序及数据包解析
 * @version 0.1
 * @date 2024-10-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "remote_ctrl.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern ChassisSpeed_t chassis_speed;
extern Arm_Motor_t motor_j1;
extern Arm_Servo_t servo_j2;
extern Arm_Gripper_t gripper;
extern RxPack rx_pack;
extern RaspCom_t rasp_com;
extern float fixed_point[7][2];

uint16_t Remote_CtrlMode = 0;
uint64_t Remotr_Update_tick = 0;
uint8_t Remote_Pro_Mode = 0;
float J2_Target_Angle = 0.0f;
float J1_Target_Angle = 0.0f;
uint16_t wait4j2 = 0;

#define BASE_STRAIGHT_SPEED 1800
#define BASE_ROTATE_SPEED 1800

#define SPEED_MAGNIFICATION 14.7f

#define JOINT_DECRACION 0.02f

uint8_t Block_num = 0;

/**
 * @brief 遥控器初始化
 *
 * @param None
 * @retval None
 */
void Remote_Init(void)
{
    initValuePack(&huart2);
}

/**
 * @brief 遥控器控制
 * @param None
 * @retval None
 */
void Remote_Ctrl(void)
{
    if (HAL_GetTick() - Remotr_Update_tick > 50)
    {
        return;
    }

    // 设置遥控器控制模式
    if (rx_pack.bools[6] == 0)
    {
        Remote_CtrlMode = REMOTE_CTRL_MODE_MANUL;
    }
    else if (rx_pack.bools[6] == 1 && rasp_com.state == RASP_COM_SUCCESS)
    {
        Remote_CtrlMode = REMOTE_CTRL_MODE_AUTO;
    }

    if (Remote_CtrlMode == REMOTE_CTRL_MODE_MANUL)
    {

        // 通过杆量设置底盘速度
        int16_t temp_vx = 0;
        int16_t temp_vy = 0;
        int16_t temp_vw = 0;

        temp_vx = rx_pack.bytes[0] * SPEED_MAGNIFICATION;
        temp_vy = rx_pack.bytes[1] * SPEED_MAGNIFICATION;
        temp_vw = rx_pack.bytes[2] * SPEED_MAGNIFICATION;

        // 通过方向键设置底盘速度
        if (rx_pack.bools[0] == 1)
        {
            temp_vx = BASE_STRAIGHT_SPEED;
        }
        else if (rx_pack.bools[1] == 1)
        {
            temp_vx = -BASE_STRAIGHT_SPEED;
        }
        else if (rx_pack.bools[2] == 1)
        {
            temp_vy = BASE_STRAIGHT_SPEED;
        }
        else if (rx_pack.bools[3] == 1)
        {
            temp_vy = -BASE_STRAIGHT_SPEED;
        }

        Chassis_SetSpeed(temp_vx, temp_vy, temp_vw);

        // 通过旋转键设置底盘角速度
        if (rx_pack.bools[4] == 1)
        {
            Chassis_SetOmega(-BASE_ROTATE_SPEED);
        }
        else if (rx_pack.bools[5] == 1)
        {
            Chassis_SetOmega(BASE_ROTATE_SPEED);
        }

        // 通过遥控器控制夹爪
        if (rx_pack.bools[7] == 1)
        {
            Arm_SetGripper(ARM_GREPPER_OPEN);
        }
        else
        {
            Arm_SetGripper(ARM_GREPPER_CLOSE);
        }
        // 通过遥控器控制关节

        float temp_angle_j1 = motor_j1.angle_ref;
        float temp_angle_j2 = (servo_j2.angle - 500.0f) / 2000.0f * 270.0f;

        if (rx_pack.bools[8] == 1)
        {
            temp_angle_j1 += JOINT_DECRACION;
        }
        else if (rx_pack.bools[9] == 1)
        {
            temp_angle_j1 -= JOINT_DECRACION;
        }
        else if (rx_pack.bools[10] == 1)
        {
            temp_angle_j2 += JOINT_DECRACION * 2;
        }
        else if (rx_pack.bools[11] == 1)
        {
            temp_angle_j2 -= JOINT_DECRACION * 2;
        }

        // 固定点模式
        if (Remote_Pro_Mode == 1)
        {
            if (motor_j1.pid.error < 25.0f && motor_j1.pid.error > -25.0f)
            {
							temp_angle_j2 = J2_Target_Angle;
              Remote_Pro_Mode = 0;
            }
        }
				else if(Remote_Pro_Mode == 2)
				{
				    if(--wait4j2 == 0)
						{
						    temp_angle_j1 = J1_Target_Angle;
						    Remote_Pro_Mode = 0;
							  wait4j2 = 1000;
						}
				
				}
        else
        {
					  wait4j2 = 1000;
            Remote_Pro_Mode = 0;
            if (rx_pack.bools[12] == 1)
            {
                temp_angle_j1 = fixed_point[0][0];
                temp_angle_j2 = fixed_point[0][1];
            }
            else if (rx_pack.bools[13] == 1)
            {
                temp_angle_j1 = fixed_point[1][0];
                temp_angle_j2 = fixed_point[1][1];
            }
            else if (rx_pack.bools[14] == 1)
            {
                temp_angle_j1 = fixed_point[2][0];
                temp_angle_j2 = fixed_point[2][1];
            }
            else if (rx_pack.bools[15] == 1)
            {
                temp_angle_j1 = fixed_point[3][0];
                temp_angle_j2 = fixed_point[3][1];
            }
            else if (rx_pack.bools[16] == 1)
            {
                temp_angle_j1 = fixed_point[4][0];
                J2_Target_Angle = fixed_point[4][1];
                Remote_Pro_Mode = 1;
            }
            else if (rx_pack.bools[17] == 1)
            {
                temp_angle_j1 = fixed_point[5][0];
                J2_Target_Angle = fixed_point[5][1];
                Remote_Pro_Mode = 1;
            }
            else if (rx_pack.bools[18] == 1)
            {
                temp_angle_j1 = fixed_point[6][0];
                temp_angle_j2 = fixed_point[6][1];
            }
        }

        // 限制关节角度
        if (temp_angle_j1 > 80.0f)
        {
            temp_angle_j1 = 80.0f;
        }
        else if (temp_angle_j1 < 0.0f)
        {
            temp_angle_j1 = 0.0f;
        }
        if (temp_angle_j2 > 256.0f)
        {
            temp_angle_j2 = 256.0f;
        }
        else if (temp_angle_j2 < 13.0f)
        {
            temp_angle_j2 = 13.0f;
        }

        // 设置关节角度
        Arm_SetAngle(temp_angle_j1, temp_angle_j2);
    }
    else if (Remote_CtrlMode == REMOTE_CTRL_MODE_AUTO)
    {

        // 自动模式代码，待补充
    }
}

void Remote_Process_Mode(void)
{
    if (motor_j1.pid.error < 3.0f && motor_j1.pid.error > -3.0f)
    {
        Arm_SetAngle(motor_j1.angle_ref, J2_Target_Angle);
        Remote_Pro_Mode = 0;
    }
}
