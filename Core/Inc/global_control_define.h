//
// Created by Ken_n on 2022/1/6.
//

#ifndef STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H
#define STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H

#endif //STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H


/************ Choose Print Mode Start*******************/
#define USB_MODE 0
#define RTT_MODE 1
//#define PRINTF_MODE 0 //print from usb
#define PRINTF_MODE 1 //print from RTT terminal
/************ Choose Print Mode End*******************/

/************ Choose UART Send Mode Start*******************/
#define Byte_MODE 0
#define Bytes_MODE 1
//#define UART_SEND_MODE 0 // UART sent one byte
#define UART_SEND_MODE 1 //UART sent full data bytes
/************ Choose UART Send Mode End*******************/


/************ Lack Define Warning Start*******************/
#if !defined(PRINTF_MODE)
#error "You mast define PRINTF_MODE to chose a printf option"
#endif
#if !defined(UART_SEND_MODE)
#error "You mast define UART_SEND_MODE to chose a UART send option"
#endif
/************ Lack Define Warning End*******************/
