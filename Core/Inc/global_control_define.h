//
// Created by Ken_n on 2022/1/6.
//

#ifndef STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H
#define STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H

#endif //STANDARDROBOTBASICCODE_GLOBAL_CONTROL_DEFINE_H


/************ Choose Print Mode Start*******************/
#define USB_MODE 0
#define USART_MODE 1
//#define PRINTF_MODE 0 //print from usb
#define PRINTF_MODE 1 //print from usart_dma
/************ Choose Print Mode End*******************/




/************ Lack Define Warning Start*******************/
#if !defined(PRINTF_MODE)
#error "You mast define PRINTF_MODE to chose a printf option"
#endif
/************ Lack Define Warning End*******************/