//
// Created by Ken_n on 2022/4/18.
//

#ifndef ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H
#define ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT     64
#define USART_TX_BUF_LENGHT     64
#define MATLAB_FIFO_BUF_LENGTH 256

static volatile uint8_t Matlab_No_DMA_IRQHandler = 1;
static volatile uint8_t matlab_dma_send_data_len = 0;

extern fifo_s_t matlab_tx_len_fifo;
extern fifo_s_t matlab_tx_fifo;
extern uint8_t usart1_matlab_tx_buf[2][USART_TX_BUF_LENGHT];

#pragma pack(push, 1)

typedef struct {
    uint8_t data1;
} SyncStruct;

#pragma pack(pop)

/**
  * @brief          matlab发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void matlab_sync_task(void const *argument);

void MY_USART_DMA_Stream7_Matlab_TX_IRQHandler(void);

void data_sync(int data_len);

#endif //ROBOMASTERROBOTCODE_MATLAB_SYNC_TASK_H
