/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"
#include "print_task.h"
#include "SEGGER_RTT.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        if (((ptr)->ecd - (ptr)->last_ecd < -6500) && ((ptr)->init_flag == 1))  \
        {                                                               \
            (ptr)->turnCount++;                                         \
        }                                                               \
        if (((ptr)->last_ecd - (ptr)->ecd < -6500) && ((ptr)->init_flag == 1))  \
        {                                                               \
            (ptr)->turnCount--;                                         \
        }                                                               \
        (ptr)->total_ecd = (ptr)->ecd+(8192 * (ptr)->turnCount);        \
        if((ptr)->init_flag == 0)                                       \
        {                                                               \
           (ptr)->init_flag = 1;                                        \
        }                                                               \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t can1_motor_data[7];
static motor_measure_t can2_motor_data[7];

static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t i = 0;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (hcan->Instance == CAN1) {
        switch (rx_header.StdId) {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            case CAN_YAW_MOTOR_ID: {

                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&can1_motor_data[i], rx_data);
                detect_hook(CHASSIS_MOTOR1_TOE + i);
                break;
            }

            default: {
                break;
            }
        }

    } else if (hcan->Instance == CAN2) {
//        id_test = rx_header.StdId;//for_test
        switch (rx_header.StdId) {
            case CAN_PITCH_MOTOR_ID: {

                get_motor_measure(&can2_motor_data[5], rx_data);
                detect_hook(PITCH_GIMBAL_MOTOR_TOE);
                break;
            }
            case CAN_3508_M1_ID:
            case CAN_3508_M4_ID:
            case CAN_TRIGGER_MOTOR_ID: {
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                id_test = i;//for_test
                get_motor_measure(&can2_motor_data[i], rx_data);
                detect_hook(CHASSIS_MOTOR1_TOE + i);
                break;
            }

            default: {
                break;
            }
        }
    }
//    switch (rx_header.StdId)
//    {
//        case CAN_3508_M1_ID:
//        case CAN_3508_M2_ID:
//        case CAN_3508_M3_ID:
//        case CAN_3508_M4_ID:
//        case CAN_YAW_MOTOR_ID:
//        case CAN_PITCH_MOTOR_ID:
//        case CAN_TRIGGER_MOTOR_ID:
//        {
//            static uint8_t i = 0;
//            //get motor id
//            i = rx_header.StdId - CAN_3508_M1_ID;
//            get_motor_measure(&can1_motor_data[i], rx_data);
//            detect_hook(CHASSIS_MOTOR1_TOE + i);
//            break;
//        }
//
//        default:
//        {
//            break;
//        }
//    }
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      ID5: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      ID6: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      ID7: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      ID8: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN2_cmd_0x1ff(int16_t ID5, int16_t ID6, int16_t ID7, int16_t ID8) {
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_IDENTIFIER_0X1FF;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (ID5 >> 8);
    can_send_data[1] = ID5;
    can_send_data[2] = (ID6 >> 8);
    can_send_data[3] = ID6;
    can_send_data[4] = (ID7 >> 8);
    can_send_data[5] = ID7;
    can_send_data[6] = (ID8 >> 8);
    can_send_data[7] = ID8;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      ID5: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      ID6: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      ID7: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      ID8: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN1_cmd_0x1ff(int16_t ID5, int16_t ID6, int16_t ID7, int16_t ID8) {
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_IDENTIFIER_0X1FF;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (ID5 >> 8);
    can_send_data[1] = ID5;
    can_send_data[2] = (ID6 >> 8);
    can_send_data[3] = ID6;
    can_send_data[4] = (ID7 >> 8);
    can_send_data[5] = ID7;
    can_send_data[6] = (ID8 >> 8);
    can_send_data[7] = ID8;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          发送CAN2电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      ID1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN2_cmd_0x200(int16_t ID1, int16_t ID2, int16_t ID3, int16_t ID4) {
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_IDENTIFIER_0X200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (ID1 >> 8);
    can_send_data[1] = ID1;
    can_send_data[2] = (ID2 >> 8);
    can_send_data[3] = ID2;
    can_send_data[4] = (ID3 >> 8);
    can_send_data[5] = ID3;
    can_send_data[6] = (ID4 >> 8);
    can_send_data[7] = ID4;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          发送CAN1电机控制电流(0x201,0x202,0x203,0x204)-底盘4个3508
  * @param[in]      ID1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      ID4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN1_cmd_0x200(int16_t ID1, int16_t ID2, int16_t ID3, int16_t ID4) {
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_IDENTIFIER_0X200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ID1 >> 8;
    can_send_data[1] = ID1;
    can_send_data[2] = ID2 >> 8;
    can_send_data[3] = ID2;
    can_send_data[4] = ID3 >> 8;
    can_send_data[5] = ID3;
    can_send_data[6] = ID4 >> 8;
    can_send_data[7] = ID4;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void) {
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) {
    return &can1_motor_data[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) {
    return &can2_motor_data[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void) {
    return &can2_motor_data[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) {
    return &can1_motor_data[(i & 0x03)];
}

const motor_measure_t *get_trigger_motor1_measure_point(void) {
    return &can2_motor_data[0];
}

const motor_measure_t *get_trigger_motor2_measure_point(void) {
    return &can2_motor_data[3];
}