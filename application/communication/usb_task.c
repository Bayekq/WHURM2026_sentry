/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "gimbal.h"
#include "IMU.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_Event       10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_GroundRobotPosition   10// ms
#define SEND_DURATION_RfidStatus   10// ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_JointState   10// ms
#define SEND_DURATION_Buff         10// ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataPidDebug_s    SEND_DATA_PID;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
static SendDataJointState_s  SEND_JOINT_STATE_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;

// clang-format on

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t Event;
    uint32_t Pid;
    uint32_t AllRobotHp;
    uint32_t GameStatus;
    uint32_t RobotMotion;
    uint32_t GroundRobotPosition;
    uint32_t RfidStatus;
    uint32_t RobotStatus;
    uint32_t JointState;
    uint32_t Buff;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void);
static void UsbSendImuData(void);
static void UsbSendRobotStateInfoData(void);
static void UsbSendEventData(void);
// static void UsbSendPIdDebugData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotMotionData(void);
static void UsbSendGroundRobotPositionData(void);
static void UsbSendRfidStatusData(void);
static void UsbSendRobotStatusData(void);
static void UsbSendJointStateData(void);
static void UsbSendBuffData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = Subscribe(IMU_NAME);                             // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum
        ((uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 5.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 7.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));
    
    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
    
    // 9.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len =(uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 10.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header),
        sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 11.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),
        sizeof(SEND_ROBOT_STATUS_DATA.frame_header));
    
    // 12.初始化云台状态数据
    SEND_JOINT_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_JOINT_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    SEND_JOINT_STATE_DATA.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_JOINT_STATE_DATA.frame_header),
        sizeof(SEND_JOINT_STATE_DATA.frame_header));

    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_BUFF_DATA.frame_header),
        sizeof(SEND_BUFF_DATA.frame_header));
}   

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送Debug数据
    CheckDurationAndSend(Debug);
    // 发送Imu数据
    CheckDurationAndSend(Imu);
    // 发送RobotStateInfo数据
    CheckDurationAndSend(RobotStateInfo);
    // 发送Event数据
    CheckDurationAndSend(Event);
    // 发送PidDebug数据
    // CheckDurationAndSend(Pid);
    // 发送AllRobotHp数据
    CheckDurationAndSend(AllRobotHp); 
    // 发送GameStatus数据
    CheckDurationAndSend(GameStatus);
    // 发送RobotMotion数据
    CheckDurationAndSend(RobotMotion);
    // 发送GroundRobotPosition数据
    CheckDurationAndSend(GroundRobotPosition);
    // 发送RfidStatus数据
    CheckDurationAndSend(RfidStatus);
    // 发送RobotStatus数据
    CheckDurationAndSend(RobotStatus);
    // 发送JointState数据
    CheckDurationAndSend(JointState);
    // 发送Buff数据
    CheckDurationAndSend(Buff);
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/


/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = IMU->angle[AX_Z];
    SEND_DATA_IMU.data.pitch = IMU->angle[AX_Y];
    SEND_DATA_IMU.data.roll = IMU->angle[AX_X];

    SEND_DATA_IMU.data.yaw_vel = IMU->gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = IMU->gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = IMU->gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();


    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}


/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USB_Transmit((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送PidDubug数据
 * @param duration 发送周期
 */
// static void UsbSendPidDebugData(void)
// {
//     SEND_DATA_PID.time_stamp = HAL_GetTick();
//     append_CRC16_check_sum((uint8_t *)&SEND_DATA_PID, sizeof(SendDataPidDebug_s));
// }

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    SEND_DATA_ALL_ROBOT_HP.data.red_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.red_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.red_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.red_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.red_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.red_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.red_base_hp = 9;
    SEND_DATA_ALL_ROBOT_HP.data.blue_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.blue_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.blue_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.blue_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.blue_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.blue_base_hp = 9;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_progress = 1;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = 100;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USB_Transmit((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = HAL_GetTick();
    SEND_JOINT_STATE_DATA.data.pitch = CmdGimbalJointState(AX_PITCH);
    SEND_JOINT_STATE_DATA.data.yaw = CmdGimbalJointState(AX_YAW);
    append_CRC16_check_sum((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
    USB_Transmit((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
}

/**
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USB_Transmit((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
}
/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    SEND_DATA_DEBUG.packages[index].data = data;

    if (SEND_DATA_DEBUG.packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline bool GetScCmdFire(void)
{
    return ROBOT_CMD_DATA.shoot.fire;
}


/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline bool GetScCmdFricOn(void)
{
    return ROBOT_CMD_DATA.shoot.fric_on;
}
/*------------------------------ End of File ------------------------------*/
