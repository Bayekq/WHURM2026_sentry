/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox ****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.03.03       Harry_Wong       1.重新构建全向轮底盘，完成单底盘控制
  *  V2.0.0   2025.10.07       Bayekq           1.重新构建全向轮底盘，完成单底盘控制、跟随模式、导航模式
  *  V2.0.1   2025.10.22       Bayekq           1.将上位机控制函数融入，减少模式冗余
  *                                             2.整合修改了一下电机控制量计算，加入了方向因子
  *  V2.1.0   2025.11.22       Bayekq           1.加入滑模控制器实现底盘跟随控制
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox ****************************
*/

#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"
#include "usb_debug.h"
#include "supervisory_computer_cmd.h"

Chassis_s chassis;
PID_t chassis_pid;
SMC_t chassis_smc;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    //step1 获取所有所需变量指针
    chassis.rc = get_remote_control_point();

    //step2 PID数据清零，设置PID参数
    const static fp32 wheel_vel[3]={KP_OMNI_VEL,KI_OMNI_VEL,KD_OMNI_VEL};
    for (int i=0;i<4;++i)
    {
        PID_init(&chassis_pid.wheel_velocity[i],PID_POSITION,wheel_vel,MAX_OUT_OMNI_VEL,MAX_IOUT_OMNI_VEL);
    }
    
    SMC_init(&chassis_smc.chassis_follow,(fp32[]){J_CHASSIS_FOLLOW,K_CHASSIS_FOLLOW,C_CHASSIS_FOLLOW,0,0,0,0,0,epsilon_CHASSIS_FOLLOW},EXPONENT,SAT_LIMIT_CHASSIS_FOLLOW,U_MAX_CHASSIS_FOLLOW,POS_ESP_CHASSIS_FOLLOW);

    //step3 初始化电机
    MotorInit(&chassis.wheel[0],WHEEL_1_ID,WHEEL_1_CAN,WHEEL_1_MOTOR_TYPE,WHEEL_1_DIRECTION,WHEEL_1_RATIO,WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1],WHEEL_2_ID,WHEEL_2_CAN,WHEEL_2_MOTOR_TYPE,WHEEL_2_DIRECTION,WHEEL_2_RATIO,WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2],WHEEL_3_ID,WHEEL_3_CAN,WHEEL_3_MOTOR_TYPE,WHEEL_3_DIRECTION,WHEEL_3_RATIO,WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3],WHEEL_4_ID,WHEEL_4_CAN,WHEEL_4_MOTOR_TYPE,WHEEL_4_DIRECTION,WHEEL_4_RATIO,WHEEL_4_MODE);

    //step4 初始默认锁定底盘
    chassis.mode = CHASSIS_LOCK;
}


/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
#if (CONTROL_TYPE == CHASSIS_ONLY)
   if ((toe_is_error(DBUS_TOE)) || switch_is_down(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]))
   {
       chassis.mode = CHASSIS_LOCK;
   }
   else if (switch_is_mid(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]))
   {
       chassis.mode = ONLY_CHASSIS;
   }
   else if (switch_is_up(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]))
   {
       chassis.mode = CHASSIS_FOLLOW;
   }
#endif
#if (CONTROL_TYPE == CHASSIS_AND_GIMBAL)
   if ((toe_is_error(DBUS_TOE)) || switch_is_down(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]) || GetGimbalInitJudgeReturn() == false)
   {
       chassis.mode = CHASSIS_LOCK;
   }
   else if (switch_is_mid(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]))
   {
       chassis.mode = CHASSIS_FOLLOW;
   }
   else if (switch_is_up(chassis.rc->rc.s[CHASSIS_MODE_CHANNEL]))
   {
       chassis.mode = CHASSIS_IMU; // 哨兵上为全自动模式
   }
#endif
}


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) 
{
    for (int i=0;i<4;++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
    }

    for (int i=0;i<4;++i)
    {
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }

    chassis.yaw_delta = GetGimbalDeltaYawMid();
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    if (chassis.mode == CHASSIS_LOCK)
    {
        chassis.reference.vx=0;
        chassis.reference.vy=0;
        chassis.reference.wz=0;
    }
    else if (chassis.mode == ONLY_CHASSIS)
    {
        chassis.reference.vx=fp32_deadline(chassis.rc->rc.ch[CHASSIS_X_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
        chassis.reference.vy=fp32_deadline(-chassis.rc->rc.ch[CHASSIS_Y_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
        chassis.reference.wz=fp32_deadline(-chassis.rc->rc.ch[CHASSIS_WZ_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_VELOCITY;
    }
    else if (chassis.mode == CHASSIS_FOLLOW)
    {
        chassis.reference_rc.vx=CALIBRATION_K*fp32_deadline(chassis.rc->rc.ch[CHASSIS_X_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED + GetScCmdChassisSpeed(AX_X);
        chassis.reference_rc.vy=CALIBRATION_K*fp32_deadline(-chassis.rc->rc.ch[CHASSIS_Y_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED + GetScCmdChassisSpeed(AX_Y);

        if (chassis.rc->key.v & KEY_PRESSED_OFFSET_W) 
        {
            chassis.reference_rc.vx += CHASSIS_RC_MAX_SPEED;
        }

        else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_S) 
        {
            chassis.reference_rc.vx -= CHASSIS_RC_MAX_SPEED;
        }

        if (chassis.rc->key.v & KEY_PRESSED_OFFSET_A) 
        {
            chassis.reference_rc.vy += CHASSIS_RC_MAX_SPEED;
        }

        else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_D) 
        {
            chassis.reference_rc.vy -= CHASSIS_RC_MAX_SPEED;
        }

        chassis.reference.vx =  chassis.reference_rc.vx * cosf(chassis.yaw_delta) - chassis.reference_rc.vy * sinf(chassis.yaw_delta);
        chassis.reference.vy =  chassis.reference_rc.vx * sinf(chassis.yaw_delta) + chassis.reference_rc.vy * cosf(chassis.yaw_delta);

        SMC_posErrorUpdate(&chassis_smc.chassis_follow, 0, chassis.yaw_delta, chassis.reference.wz*WHEEL_CENTER_DISTANCE, CHASSIS_SAMPLE_TIME);
        // chassis.reference.wz=PID_calc(&chassis_pid.follow,chassis.yaw_delta,0);
        chassis.reference.wz=SMC_calc(&chassis_smc.chassis_follow);
    }
    else if (chassis.mode == CHASSIS_IMU)
    {
        chassis.reference_rc.vx=CALIBRATION_K*fp32_deadline(chassis.rc->rc.ch[CHASSIS_X_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED + GetScCmdChassisSpeed(AX_X);
        chassis.reference_rc.vy=CALIBRATION_K*fp32_deadline(-chassis.rc->rc.ch[CHASSIS_Y_CHANNEL],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED + GetScCmdChassisSpeed(AX_Y);

        if (chassis.rc->key.v & KEY_PRESSED_OFFSET_W) 
        {
            chassis.reference_rc.vx += CHASSIS_RC_MAX_SPEED;
        }

        else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_S) 
        {
            chassis.reference_rc.vx -= CHASSIS_RC_MAX_SPEED;
        }

        if (chassis.rc->key.v & KEY_PRESSED_OFFSET_A) 
        {
            chassis.reference_rc.vy += CHASSIS_RC_MAX_SPEED;
        }

        else if (chassis.rc->key.v & KEY_PRESSED_OFFSET_D) 
        {
            chassis.reference_rc.vy -= CHASSIS_RC_MAX_SPEED;
        }

        chassis.reference.vx =  chassis.reference_rc.vx * cosf(chassis.yaw_delta) - chassis.reference_rc.vy * sinf(chassis.yaw_delta);
        chassis.reference.vy =  chassis.reference_rc.vx * sinf(chassis.yaw_delta) + chassis.reference_rc.vy * cosf(chassis.yaw_delta);
        
        chassis.reference.wz =  GetScCmdChassisSpeed(AX_Z);
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    chassis.set[0] = (sqrt(0.5)*(  chassis.reference.vx - chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[0].reduction_ratio;
    chassis.set[1] = (sqrt(0.5)*(  chassis.reference.vx + chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[1].reduction_ratio;
    chassis.set[2] = (sqrt(0.5)*( -chassis.reference.vx + chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[2].reduction_ratio;
    chassis.set[3] = (sqrt(0.5)*( -chassis.reference.vx - chassis.reference.vy ) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[3].reduction_ratio; 

    for (int i=0;i<4;++i)
    {
        chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity[i], chassis.feedback[i], chassis.set[i]);
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void){
    CanCmdDjiMotor(CHASSIS_CAN,CHASSIS_STDID,chassis.wheel[3].set.curr,chassis.wheel[0].set.curr,chassis.wheel[1].set.curr,chassis.wheel[2].set.curr);
}

#endif
