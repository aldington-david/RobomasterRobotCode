//
// Created by Ken_n on 2022/4/16.
//

#include "main.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "referee_task.h"
#include "fifo.h"
#include "usart.h"
#include "USART_task.h"

TaskHandle_t USART_task_local_handler;

void USART_task(void const *pvParameters) {
    USART_task_local_handler = xTaskGetCurrentTaskHandle();
    while (1) {
        while (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != pdPASS) {
        }
        if (No_DMA_IRQHandler) {
            if (fifo_s_used(&referee_tx_fifo)) {
                if (fifo_s_used(&referee_tx_len_fifo)) {
                    if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1");
                        __HAL_DMA_DISABLE(huart6.hdmatx);
                        dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], dma_send_data_len);
                        huart6.hdmatx->Instance->CR |= DMA_SxCR_CT;
                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
                        __HAL_DMA_ENABLE(huart6.hdmatx);
                        No_DMA_IRQHandler = 0;
                        detect_hook(REFEREE_TX_TOE);
                        if (fifo_s_used(&referee_tx_fifo)) {
                            if (fifo_s_used(&referee_tx_len_fifo)) {
                                dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                                memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                                fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], dma_send_data_len);
                            }
                        } else {
                            dma_send_data_len = 0;
                            No_DMA_IRQHandler = 1;
                            memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                        }
                    } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0");
                        __HAL_DMA_DISABLE(huart6.hdmatx);
                        dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], dma_send_data_len);
                        huart6.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
                        __HAL_DMA_ENABLE(huart6.hdmatx);
                        No_DMA_IRQHandler = 0;
                        detect_hook(REFEREE_TX_TOE);
                        if (fifo_s_used(&referee_tx_fifo)) {
                            if (fifo_s_used(&referee_tx_len_fifo)) {
                                dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                                memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                                fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], dma_send_data_len);
                            }
                        } else {
                            dma_send_data_len = 0;
                            No_DMA_IRQHandler = 1;
                            memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                        }
                    }
                }
            }
        }

    }


}