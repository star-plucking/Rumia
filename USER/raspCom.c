/**
 * @file raspCom.c
 * @author Zhou Xu
 * @brief
 * @version 0.1
 * @date 2024-10-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "raspCom.h"

extern Arm_Motor_t motor_j1;
extern Arm_Servo_t servo_j2;
extern Arm_Gripper_t gripper;

RaspCom_t rasp_com;
uint8_t rp_rxbuff[4];

/**
 * @brief 树莓派通信初始化
 *
 * @param huart
 */
void RaspCom_Init(UART_HandleTypeDef *huart)
{
    rasp_com.state = RASP_COM_INIT;
    __HAL_UART_CLEAR_IDLEFLAG(huart);          // 清除空闲中断标志
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // 启UART的空闲中
    HAL_UART_Receive_DMA(huart, (uint8_t *)rp_rxbuff, 4);
    rasp_com.state = RASP_COM_SUCCESS;
		rasp_com.huart = huart;
}

/**
 * @brief 树莓派通信发送
 *
 * @param tx_message
 */
void RaspCom_Send(uint8_t tx_message)
{
    uint8_t tx_buffer[3];
    tx_buffer[0] = RASP_COM_HEAD;
    tx_buffer[1] = tx_message;
    tx_buffer[2] = RASP_COM_TAIL;
    HAL_UART_Transmit(rasp_com.huart, (uint8_t *)tx_buffer, 3, 10);
}

void RaspCom_Receive(RaspCom_t *rasp_com)
{
    if (rp_rxbuff[0] == RASP_COM_HEAD && rp_rxbuff[3] == RASP_COM_TAIL)
    {
        rasp_com->result = (rp_rxbuff[1] >> 7) & 0x01;
        rasp_com->color = (rp_rxbuff[1] >> 6) & 0x02;

        // 按照bit位读取,color后空开两位
        rasp_com->type[0] = (rp_rxbuff[1] >> 2) & 0x03;
        rasp_com->type[1] = (rp_rxbuff[1] >> 0) & 0x03;
        rasp_com->type[2] = (rp_rxbuff[2] >> 6) & 0x03;
        rasp_com->type[3] = (rp_rxbuff[2] >> 4) & 0x03;
        rasp_com->type[4] = (rp_rxbuff[2] >> 2) & 0x03;
        rasp_com->type[5] = (rp_rxbuff[2] >> 0) & 0x03;
        
        rasp_com->state = RASP_COM_SUCCESS;
    }
    else
    {
        rasp_com->state = RASP_COM_LOSS;
    }
}
