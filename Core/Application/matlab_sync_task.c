//
// Created by Ken_n on 2022/4/18.
//

#include "matlab_sync_task.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "global_control_define.h"
#include "main.h"
#include "task.h"


uint8_t matlab_fifo_tx_len_buf[MATLAB_FIFO_BUF_LENGTH];
uint8_t matlab_fifo_tx_buf[MATLAB_FIFO_BUF_LENGTH];
fifo_s_t matlab_tx_len_fifo;
fifo_s_t matlab_tx_fifo;
uint8_t usart1_matlab_tx_buf[2][USART_TX_BUF_LENGHT];

/**
  * @brief          matlab发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void matlab_sync_task(void const *argument) {
//    init_referee_struct_data();
    fifo_s_init(&matlab_tx_len_fifo, matlab_fifo_tx_len_buf, MATLAB_FIFO_BUF_LENGTH);
    fifo_s_init(&matlab_tx_fifo, matlab_fifo_tx_buf, MATLAB_FIFO_BUF_LENGTH);
    usart1_tx_init(usart1_matlab_tx_buf[0], usart1_matlab_tx_buf[1], USART_TX_BUF_LENGHT);
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
//        referee_unpack_fifo_data();
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

void MY_USART_DMA_Stream7_Matlab_TX_IRQHandler(void)
{
    __HAL_DMA_DISABLE(huart1.hdmatx);
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart1.hdmatx));
    if (Matlab_No_DMA_IRQHandler) {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, Matlab_No_DMA_IRQHandler);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            detect_hook(USART1_TX_TOE);
            if (UART_SEND_MODE == Bytes_MODE) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
                        Matlab_No_DMA_IRQHandler = fifo_s_get(&matlab_tx_len_fifo);
                        memset(&usart1_matlab_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1], Matlab_No_DMA_IRQHandler);
                    }
                } else {
                    Matlab_No_DMA_IRQHandler = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                }
            }
            if (UART_SEND_MODE == Byte_MODE) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    memset(&usart1_matlab_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                    Matlab_No_DMA_IRQHandler = 1;
                    fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1], Matlab_No_DMA_IRQHandler);

                } else {
                    Matlab_No_DMA_IRQHandler = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                }
            }

        } else {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, Matlab_No_DMA_IRQHandler);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            detect_hook(USART1_TX_TOE);
            if (UART_SEND_MODE == Bytes_MODE) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
                        Matlab_No_DMA_IRQHandler = fifo_s_get(&matlab_tx_len_fifo);
                        memset(&usart1_matlab_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0], Matlab_No_DMA_IRQHandler);
                    }
                } else {
                    Matlab_No_DMA_IRQHandler = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                }
            } else if (UART_SEND_MODE == Byte_MODE) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    memset(&usart1_matlab_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                    Matlab_No_DMA_IRQHandler = 1;
                    fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0], Matlab_No_DMA_IRQHandler);

                } else {
                    Matlab_No_DMA_IRQHandler = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                }
            }
        }
    } else {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            memset(&usart1_matlab_tx_buf[1], 0, USART_TX_BUF_LENGHT);
        } else {
            memset(&usart1_matlab_tx_buf[0], 0, USART_TX_BUF_LENGHT);
        }
    }
}
