/**
  * @file       robot_param_omni_sentry.h
  * @brief      这里是全向轮哨兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL          // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型(无云台，底盘调试阶段)
#define SHOOT_TYPE SHOOT_NONE                    // 选择发射机构类型(无发射机构，底盘调试阶段)
#define CONTROL_TYPE CHASSIS_ONLY			           // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  // 选择机械臂类型

#define __GYRO_BIAS_YAW (0.0f)                   // (rad/s)陀螺仪零漂(调试阶段，暂未设置)

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS (0.141f)            //(m)轮子直径
#define WHEEL_CENTER_DISTANCE (0.216f)   //(m)轮子到车的距离（0.22 + 0.21）

//motor parameters ---------------------
//底盘电流发送参数
#define CHASSIS_CAN (1)
#define CHASSIS_STDID (0x200)

//电机ID ---------------------
#define WHEEL_1_ID (2)
#define WHEEL_2_ID (3)
#define WHEEL_3_ID (4)
#define WHEEL_4_ID (1)

//电机CAN ---------------------
#define WHEEL_1_CAN (1)
#define WHEEL_2_CAN (1)
#define WHEEL_3_CAN (1)
#define WHEEL_4_CAN (1)

//电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_OMNI_VEL (20.0f)
#define KI_OMNI_VEL (0.3f)
#define KD_OMNI_VEL (0.3f)
#define MAX_IOUT_OMNI_VEL (10000.0f)
#define MAX_OUT_OMNI_VEL (30000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.01f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.5f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (3.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (1.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

/*-------------------- Gimbal --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (200000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.3f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.5f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (2.3731f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (2.1246f)    //云台初始化正对齐的时候使用的yaw轴正中心量

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (9.00f)
#define KI_GIMBAL_YAW_ANGLE (0.003f)
#define KD_GIMBAL_YAW_ANGLE (0.75f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.05f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (5000.0f)
#define KI_GIMBAL_YAW_VELOCITY (1.0f)
#define KD_GIMBAL_YAW_VELOCITY (0.1f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (30000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (4.5f)
#define KI_GIMBAL_PITCH_ANGLE (0.0001f)
#define KD_GIMBAL_PITCH_ANGLE (3.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (10.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (1200.0f)
#define KI_GIMBAL_PITCH_VELOCITY (30.0f)
#define KD_GIMBAL_PITCH_VELOCITY (100.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (30000.0f)

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f             // (m)摩擦轮半径
#define BULLET_NUM 8                  // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                     // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 1.0f  // 定义英雄电机到拨弹盘的齿轮减速比

/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 8
#define FRIC_MOTOR_R_ID 6
#define FRIC_MOTOR_L_ID 5

//电机can口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 1
#define FRIC_MOTOR_L_CAN 1

//电机std_id
#define STD_ID 0x1FF
//单环拨弹速度
#define TRIGGER_SPEED (300.0f)
//摩擦轮速度
#define FRIC_R_SPEED (666.0f)
#define FRIC_L_SPEED (-666.0f)
#define FRIC_SPEED_LIMIT (600.0f)

/*ECD parameters------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 5.0f
#define BLOCK_TIME 1000
#define REVERSE_TIME 1250
#define REVERSE_SPEED (-20.0f)

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f)  //  (none)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (100.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (25.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (30.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (666.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (1.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE 80  //89

#endif /* INCLUDED_ROBOT_PARAM_H */
