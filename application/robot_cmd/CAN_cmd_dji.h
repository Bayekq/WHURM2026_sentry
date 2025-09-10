/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制DJI电机 GM3508 GM2006 GM6020.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.2     Sep-10-2025     Bayek           1. 完善架构，完成多电机控制部分。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  */

#ifndef CAN_CMD_DJI_H
#define CAN_CMD_DJI_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

extern void CanCmdDjiMotor(
    uint8_t can, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);

extern void DjiMultipleControl(uint8_t can, uint8_t motor_num, Motor_s * motor_array);

#endif  //CAN_CMD_DJI_H

/************************ END OF FILE ************************/
