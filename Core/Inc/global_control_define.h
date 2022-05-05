//
// Created by Ken_n on 2022/1/6.
//

#ifndef STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H
#define STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H

#endif //STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H


/************ Choose Print Mode Start*******************/
#define USB_MODE 0
#define RTT_MODE 1
//#define PRINTF_MODE USB_MODE //print from usb
#define PRINTF_MODE RTT_MODE //print from RTT terminal
/************ Choose Print Mode End*******************/

/************ Choose UART Send Mode Start*******************/
#define Byte_MODE 0
#define Bytes_MODE 1
//#define UART_SEND_MODE Byte_MODE // UART sent one byte
#define UART_SEND_MODE Bytes_MODE //UART sent full data bytes
/************ Choose UART Send Mode End*******************/

/************ Choose UART1 TX Target Start*******************/
#define Vision_MODE 0
#define Matlab_MODE 1
#define Vision_rx_Matlab_tx_MODE 2
//#define UART1_TARGET_MODE Vision_MODE // UART1 rx_tx to vision
#define UART1_TARGET_MODE Matlab_MODE //UART1 rx_tx to matlab
//#define UART1_TARGET_MODE Vision_rx_Matlab_tx_MODE //UART1 rx to vision tx to matlab //not_use
/************ Choose UART1 TX Target End*******************/

/************ Choose Detect Block Device Start*******************/
#define Block_None_Device 0
#define Block_All_Device 1
#define Block_All_Device_ecp_Control 2
//#define DETECT_BLOCK Block_None_Device // do not block devices
#define DETECT_BLOCK Block_All_Device // block whole device
//#define DETECT_BLOCK Block_All_Device_ecp_Control //block whole device except Radio Control
/************ Choose Detect Block Device End*******************/

/************ Choose Calibrate Block Start*******************/
#define Cali_Manually 0
#define Cali_Auto 1
#define Cali_Active 2
#define CALI_BLOCK Cali_Manually //Calibrate manually
//#define CALI_BLOCK Cali_Auto //Calibrate Auto start
//#define CALI_BLOCK Cali_Active  //Calibrate by task notification //not realize yet
/************ Choose Detect Block Device End*******************/

/************ Lack Define Warning Start*******************/
#if !defined(PRINTF_MODE)
#error "You mast define PRINTF_MODE to chose a printf option"
#endif

#if !defined(UART_SEND_MODE)
#error "You mast define UART_SEND_MODE to chose a UART send option"
#endif

#if !defined(UART1_TARGET_MODE)
#error "You mast define UART1_TARGET_MODE to chose a UART1 target"
#endif

#if !defined(DETECT_BLOCK)
#error "You mast define DETECT_BLOCK to chose a block mode"
#endif

#if !defined(CALI_BLOCK)
#error "You mast define CALI_BLOCK to chose a block mode"
#endif
/************ Lack Define Warning End*******************/
