//
// Created by Ken_n on 2022/1/6.
//

#ifndef STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H
#define STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H

#include "referee_task.h"
/************ WMM define Start*******************/
//x point to north
//WMM model mag is true north value
//WMM model is NED
#define North_Comp  (28398.5f) //nT，when use it please scale to uT
#define East_Comp    (-3717.0f) //nT，when use it please scale to uT
#define Vertical_Comp    (46139.5f) //nT，when use it please scale to uT
#define Total_Field    (54306.0f) //nT，when use it please scale to uT
#define Declination (-7.4570f) //degree,need to convert to rad
/************ WMM define End*******************/

/************ Self Data define Start*******************/
#define SELF_ID  infantry3_red //get from referee_robot_ID
#define FIRMWARE_VERSION 12345 //for self_test
#define YAW_LIMIT YAW_NO_LIMIT
#define YAW_LIMIT_TURN 0

#define YAW_HAVE_LIMIT 1
#define YAW_NO_LIMIT 0

#define PITCH_MOTOR_REDUCTION 3.4f
#define YAW_MOTOR_REDUCTION 1.0f
/************ Self Data define End*******************/

/************ PID Auto Tune Start *******************/
#define OFF    0
#define ON     1
#define PID_AUTO_TUNE OFF
#define PID_AUTO_TUNE_CHASSIS_FOLLOW 0
/************ PID Auto Tune End *******************/

/************ Init To Offset Start*******************/
#define NO_INIT    0
#define INIT    1

#define PITCH_INIT  INIT
#define YAW_INIT    INIT

#define INIT_ONLY_FIRST_TIME 0
/************ Init To Offset End*******************/

/************ Motor turn define Start*******************/
#define NO_TURN    0
#define TURN    1
#define PITCH_TURN  NO_TURN
#define YAW_TURN    TURN
#define SHOOT_TRIGGER_TURN    NO_TURN
/************ Motor turn define End*******************/

/************ Choose Print Mode Start*******************/
#define USB_MODE 0
#define RTT_MODE 1
//#define PRINTF_MODE USB_MODE //print from usb
#define PRINTF_MODE RTT_MODE //print from RTT terminal
/************ Choose Print Mode End*******************/

///************ Choose UART Send Mode Start*******************/
//#define Byte_MODE 0
//#define Bytes_MODE 1
////#define UART_SEND_MODE Byte_MODE // UART sent one byte //not_use
//#define UART_SEND_MODE Bytes_MODE //UART sent full data bytes
///************ Choose UART Send Mode End*******************/

/************ Choose UART1 TX Target Start*******************/
#define Vision_MODE 0
#define Matlab_MODE 1
#define Vision_rx_Matlab_tx_MODE 2
#define UART1_TARGET_MODE Vision_MODE // UART1 rx_tx to vision
//#define UART1_TARGET_MODE Matlab_MODE //UART1 rx_tx to matlab
//#define UART1_TARGET_MODE Vision_rx_Matlab_tx_MODE //UART1 rx to vision tx to matlab //not_use
/************ Choose UART1 TX Target End*******************/

/************ Choose Detect Block Device Start*******************/
#define Block_Buzzer 1
#define Block_None_Device 0
#define Block_All_Device 1
#define Block_All_Device_ecp_Control 2
//#define DETECT_BLOCK Block_None_Device // do not block devices
//#define DETECT_BLOCK Block_All_Device // block whole device
#define DETECT_BLOCK Block_All_Device_ecp_Control //block whole device except Radio Control
/************ Choose Detect Block Device End*******************/

/************ Choose Calibrate Block Start*******************/
#define Cali_Manually 0
#define Cali_Auto 1
#define Cali_Active 2
#define CALI_BLOCK Cali_Manually //Calibrate manually
//#define CALI_BLOCK Cali_Auto //Calibrate Auto start
//#define CALI_BLOCK Cali_Active  //Calibrate by task notification //not realize yet
/************ Choose Detect Block Device End*******************/

/************ Keil Feature Define Start*******************/
#if __CC_ARM
#define Enable_EventRecorder 1
#define Disable_EventRecorder 0
#define EventRecorder_MODE Enable_EventRecorder
//#define EventRecorder_MODE Disable_EventRecorder
#endif
/************ Keil Feature Define End*******************/

/************ FreeRTOS Debug Define Start*******************/

#define Enable_FreeRTOS_Heap_Data 1
#define Disable_FreeRTOS_Heap_Data 0
#define FreeRTOS_Date_MODE Enable_EventRecorder
//#define FreeRTOS_Date_MODE Disable_FreeRTOS_Heap_Data

/************ FreeRTOS Debug Define End*******************/

/************ Lack Define Warning Start*******************/
#if !defined(PRINTF_MODE)
#error "You mast define PRINTF_MODE to chose a printf option"
#endif

//#if !defined(UART_SEND_MODE)
//#error "You mast define UART_SEND_MODE to choose a UART send option"
//#endif

#if !defined(UART1_TARGET_MODE)
#error "You mast define UART1_TARGET_MODE to chose a UART1 target"
#endif

#if !defined(DETECT_BLOCK)
#error "You mast define DETECT_BLOCK to chose a block mode"
#endif

#if !defined(CALI_BLOCK)
#error "You mast define CALI_BLOCK to chose a block mode"
#endif

#if !defined(CALI_BLOCK)
#error "You mast define CALI_BLOCK to chose a block mode"
#endif

#if !defined(YAW_LIMIT)
#error "You mast define YAW_LIMIT to limit yaw angle"
#endif

#if !defined(PITCH_INIT)
#error "You mast define PITCH_INIT to choose if let pitch init to offset or not"
#endif

#if !defined(YAW_INIT)
#error "You mast define YAW_INIT to choose if let yaw init to offset or not"
#endif

#if !defined(YAW_LIMIT_TURN)
#error "You mast define YAW_LIMIT_TURN to limit yaw turn cnt"
#endif

#if !defined(PID_AUTO_TUNE)
#error "You mast define PID_AUTO_TUNE to choose if let pid auto tune or not"
#endif

#if __CC_ARM
#if !defined(EventRecorder_MODE)
#error "You mast define EventRecorder_MODE to chose enalbe or not"
#endif
#endif
/************ Lack Define Warning End*******************/
#endif //STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H