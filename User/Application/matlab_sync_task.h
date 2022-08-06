//
// Created by Ken_n on 2022/4/18.
//

#ifndef ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H
#define ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "struct_typedef.h"
#include "fifo.h"
#include "cmsis_os.h"

#define USART1_MATLAB_TX_BUF_LENGHT     128
#define MATLAB_FIFO_BUF_LENGTH 1024

extern volatile uint8_t Matlab_No_DMA_IRQHandler;
extern volatile uint8_t matlab_dma_send_data_len;
extern volatile uint8_t Matlab_IRQ_Return_Before;

extern fifo_s_t matlab_tx_len_fifo;
extern fifo_s_t matlab_tx_fifo;
extern uint8_t usart1_matlab_tx_buf[2][USART1_MATLAB_TX_BUF_LENGHT];
extern TaskHandle_t matlab_tx_task_local_handler;

#pragma pack(push, 1)

//typedef struct {
//    char data1;
//    char data2;
//    char data3;
//    uint8_t data4;
//    char data5;
//} SyncStruct;

typedef struct {
    char sync_char;
    uint8_t data_num;
    uint16_t data1;
    uint16_t data2;
    uint16_t data3;
    uint16_t data4;
    uint16_t data5;
} SyncStruct;

#pragma pack(pop)

/**
  * @brief          matlab发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void matlab_sync_task(void const *argument);

void init_matlab_struct_data(void);

void MY_USART_DMA_Stream7_Matlab_TX_IRQHandler(void);

void send_sync_char(void);

void data_sync(int data_len);

/**
  * @brief          获取matlab_sync_task栈大小
  * @param[in]      none
  * @retval         matlab_sync_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_matlab_sync_task(void);

#endif //ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H
