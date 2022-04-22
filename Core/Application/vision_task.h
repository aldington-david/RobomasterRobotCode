//
// Created by Ken_n on 2022/4/18.
//

#ifndef ROBOMASTERROBOTCODE_VISION_TASK_H
#define ROBOMASTERROBOTCODE_VISION_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"
#include "fifo.h"
#include "cmsis_os.h"

#define USART1_RX_BUF_LENGHT     64
#define USART1_VISION_TX_BUF_LENGHT     64
#define VISION_FIFO_BUF_LENGTH 256

extern volatile uint8_t Vision_No_DMA_IRQHandler;
extern volatile uint8_t vision_dma_send_data_len;

extern fifo_s_t vision_rx_fifo;
extern fifo_s_t vision_tx_len_fifo;
extern fifo_s_t vision_tx_fifo;
extern uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];
extern uint8_t usart1_vision_tx_buf[2][USART1_VISION_TX_BUF_LENGHT];

/**
  * @brief          视觉接收任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void vision_rx_task(void const *argument);

/**
  * @brief          视觉发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void vision_tx_task(void const *argument);

/**
  * @brief          single byte upacked
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void vision_unpack_fifo_data(void);

void MY_USART_DMA_Stream7_Vision_TX_IRQHandler(void);

extern TaskHandle_t USART1TX_active_task_local_handler;

/**
  * @brief          usart1发送启动任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART1TX_active_task(void const *pvParameters);

#endif //ROBOMASTERROBOTCODE_VISION_TASK_H
