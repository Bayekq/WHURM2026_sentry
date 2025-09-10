/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024-04-01      Penguin         1. done
  *  V1.2.0     2025-02-26      Harry_Wong      1.删除了不需要的变量
  *                                             2.剩余更新详情请见.c文件
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#ifndef GIMBAL_YAW_PITCH_H
#define GIMBAL_YAW_PITCH_H
#include "IMU.h"//陀螺仪文件
#include "gimbal.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"
#include "struct_typedef.h"
#include  "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "usb_debug.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "math.h"
#include "macro_typedef.h"
#include "supervisory_computer_cmd.h"


/**
 * @brief 云台模式
 */
typedef enum {
    GIMBAL_ZERO_FORCE,  // 云台无力，所有控制量置0
    GIMBAL_IMU,         // 云台陀螺仪控制(角度控制)
    GIMBAL_INIT,        //云台矫正模式
    GIMBAL_DBUS_ERR,    //遥控器断联相关处理任务
    GIMBAL_GAP,         //跳出矫正进入IMU/AUTO_AIM模式之前的存储数据模式
    GIMBAL_AUTO_AIM,    //自瞄模式
} GimbalMode_e;


/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
} Values_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;  //角速度

    pid_type_def pitch_angle;
    pid_type_def pitch_velocity;
    
} PID_t;

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw,pitch;
    /*-------------------- Values --------------------*/
    Values_t reference;    // 期望值
    Values_t feedback_pos,feedback_vel;     // 状态值(目前专供给IMU数据)
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器

    float angle_zero_for_imu; //pitch电机处于中值时imupitch的角度

    uint32_t init_start_time,init_timer;

    bool init_continue; //是否继续进行校准模式
} Gimbal_s;

extern void GimbalInit(void);

extern void GimbalHandleException(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void GimbalSendCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH
