//
// Created by Ken_n on 2022/4/18.
//
#include "vision_task.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "global_control_define.h"
#include "main.h"
#include "matlab_sync_task.h"

uint8_t vision_fifo_rx_buf[VISION_FIFO_BUF_LENGTH];
uint8_t vision_fifo_tx_len_buf[VISION_FIFO_BUF_LENGTH];
uint8_t vision_fifo_tx_buf[VISION_FIFO_BUF_LENGTH];
fifo_s_t vision_rx_fifo;
fifo_s_t vision_tx_len_fifo;
fifo_s_t vision_tx_fifo;
uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];
uint8_t usart1_vision_tx_buf[2][USART1_VISION_TX_BUF_LENGHT];
TaskHandle_t USART1TX_active_task_local_handler;
volatile uint8_t Vision_No_DMA_IRQHandler = 1;
volatile uint8_t vision_dma_send_data_len = 0;

/**
  * @brief          视觉接收任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void vision_rx_task(void const *argument) {
//    init_referee_struct_data();
    fifo_s_init(&vision_rx_fifo, vision_fifo_rx_buf, VISION_FIFO_BUF_LENGTH);
    usart1_rx_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_LENGHT);
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
//        referee_unpack_fifo_data();
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

/**
  * @brief          视觉发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void vision_tx_task(void const *argument) {
//    init_referee_struct_data();
    fifo_s_init(&vision_tx_len_fifo, vision_fifo_tx_len_buf, VISION_FIFO_BUF_LENGTH);
    fifo_s_init(&vision_tx_fifo, vision_fifo_tx_buf, VISION_FIFO_BUF_LENGTH);
    usart1_tx_init(usart1_vision_tx_buf[0], usart1_vision_tx_buf[1], USART1_VISION_TX_BUF_LENGHT);
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
//        referee_unpack_fifo_data();
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

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
//void vision_unpack_fifo_data(void) {
//    uint8_t byte = 0;
//    uint8_t sof = HEADER_SOF;
//    unpack_data_t *p_obj = &referee_unpack_obj;
//
//    while (fifo_s_used(&referee_rx_fifo)) {
//        byte = fifo_s_get(&referee_rx_fifo);
//        switch (p_obj->unpack_step) {
//            case STEP_HEADER_SOF: {
//                if (byte == sof) {
//                    p_obj->unpack_step = STEP_LENGTH_LOW;
//                    p_obj->protocol_packet[p_obj->index++] = byte;
//                } else {
//                    p_obj->index = 0;
//                }
//            }
//                break;
//
//            case STEP_LENGTH_LOW: {
//                p_obj->data_len = byte;
//                p_obj->protocol_packet[p_obj->index++] = byte;
//                p_obj->unpack_step = STEP_LENGTH_HIGH;
//            }
//                break;
//
//            case STEP_LENGTH_HIGH: {
//                p_obj->data_len |= (byte << 8);
//                p_obj->protocol_packet[p_obj->index++] = byte;
//
//                if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
//                    p_obj->unpack_step = STEP_FRAME_SEQ;
//                } else {
//                    p_obj->unpack_step = STEP_HEADER_SOF;
//                    p_obj->index = 0;
//                }
//            }
//                break;
//            case STEP_FRAME_SEQ: {
//                p_obj->protocol_packet[p_obj->index++] = byte;
//                p_obj->unpack_step = STEP_HEADER_CRC8;
//            }
//                break;
//
//            case STEP_HEADER_CRC8: {
//                p_obj->protocol_packet[p_obj->index++] = byte;
//
//                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
//                    if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
//                        p_obj->unpack_step = STEP_DATA_CRC16;
//                    } else {
//                        p_obj->unpack_step = STEP_HEADER_SOF;
//                        p_obj->index = 0;
//                    }
//                }
//            }
//                break;
//
//            case STEP_DATA_CRC16: {
//                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
//                    p_obj->protocol_packet[p_obj->index++] = byte;
//                }
//                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
//                    p_obj->unpack_step = STEP_HEADER_SOF;
//                    p_obj->index = 0;
//
//                    if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
//                        judge_update(p_obj->protocol_packet);
//                    }
//                }
//            }
//                break;
//
//            default: {
//                p_obj->unpack_step = STEP_HEADER_SOF;
//                p_obj->index = 0;
//            }
//                break;
//        }
//    }
//}


/**
  * @brief          usart1发送启动任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART1TX_active_task(void const *pvParameters) {
    USART1TX_active_task_local_handler = xTaskGetCurrentTaskHandle();
    while (1) {
        while (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != pdPASS) {
        }
        vTaskSuspendAll();
        if (UART_SEND_MODE == Vision_MODE) {
            if (Vision_No_DMA_IRQHandler) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    if (fifo_s_used(&vision_tx_len_fifo)) {
                        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                            memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                            fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[1], vision_dma_send_data_len);
                            huart1.hdmatx->Instance->CR |= DMA_SxCR_CT;
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Vision_No_DMA_IRQHandler = 0;
                            detect_hook(USART1_TX_TOE);
                            if (fifo_s_used(&vision_tx_fifo)) {
                                if (fifo_s_used(&vision_tx_len_fifo)) {
                                    vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                                    memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                                    fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[0],
                                                vision_dma_send_data_len);
                                }
                            } else {
                                vision_dma_send_data_len = 0;
                                Vision_No_DMA_IRQHandler = 1;
                                memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                            }
                        } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                            memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                            fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[0], vision_dma_send_data_len);
                            huart1.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Vision_No_DMA_IRQHandler = 0;
                            detect_hook(USART1_TX_TOE);
                            if (fifo_s_used(&vision_tx_fifo)) {
                                if (fifo_s_used(&vision_tx_len_fifo)) {
                                    vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                                    memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                                    fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[1],
                                                vision_dma_send_data_len);
                                }
                            } else {
                                vision_dma_send_data_len = 0;
                                Vision_No_DMA_IRQHandler = 1;
                                memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                            }
                        }
                    }
                }
            }
        } else if (UART1_TARGET_MODE == Matlab_MODE) {
            if (Matlab_No_DMA_IRQHandler) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
                        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                            memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1], matlab_dma_send_data_len);
                            huart1.hdmatx->Instance->CR |= DMA_SxCR_CT;
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Matlab_No_DMA_IRQHandler = 0;
                            detect_hook(USART1_TX_TOE);
                            if (fifo_s_used(&matlab_tx_fifo)) {
                                if (fifo_s_used(&matlab_tx_len_fifo)) {
                                    matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                                    memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                    fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0],
                                                matlab_dma_send_data_len);
                                }
                            } else {
                                matlab_dma_send_data_len = 0;
                                Matlab_No_DMA_IRQHandler = 1;
                                memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            }
                        } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                            memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0], matlab_dma_send_data_len);
                            huart1.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Matlab_No_DMA_IRQHandler = 0;
                            detect_hook(USART1_TX_TOE);
                            if (fifo_s_used(&matlab_tx_fifo)) {
                                if (fifo_s_used(&matlab_tx_len_fifo)) {
                                    matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                                    memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                    fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1],
                                                matlab_dma_send_data_len);
                                }
                            } else {
                                matlab_dma_send_data_len = 0;
                                Matlab_No_DMA_IRQHandler = 1;
                                memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            }
                        }
                    }
                }
            }
        }
        xTaskResumeAll();
    }
}

/**
  * @brief          视觉串口接收（USART1）回调函数
  * @param[in]      void
  * @retval         none
  */
void USART1_IRQHandler(void) {
    //    SEGGER_RTT_WriteString(0,"TEST");
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        static uint16_t this_time_rx_len = 0;
        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART1_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&vision_rx_fifo, (char *) usart1_rx_buf[0], this_time_rx_len);
            detect_hook(VISION_RX_TOE);
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART1_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&vision_rx_fifo, (char *) usart1_rx_buf[1], this_time_rx_len);
            detect_hook(VISION_RX_TOE);
        }
    }
}

void MY_USART_DMA_Stream7_Vision_TX_IRQHandler(void) {
    __HAL_DMA_DISABLE(huart1.hdmatx);
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart1.hdmatx));
    if (vision_dma_send_data_len) {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            detect_hook(USART1_TX_TOE);
            if (UART_SEND_MODE == Bytes_MODE) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    if (fifo_s_used(&vision_tx_len_fifo)) {
                        vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                        memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                        fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[1], vision_dma_send_data_len);
                    }
                } else {
                    vision_dma_send_data_len = 0;
                    Vision_No_DMA_IRQHandler = 1;
                    memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                }
            }
            if (UART_SEND_MODE == Byte_MODE) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                    vision_dma_send_data_len = 1;
                    fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[1], vision_dma_send_data_len);

                } else {
                    vision_dma_send_data_len = 0;
                    Vision_No_DMA_IRQHandler = 1;
                    memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                }
            }

        } else {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            detect_hook(USART1_TX_TOE);
            if (UART_SEND_MODE == Bytes_MODE) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    if (fifo_s_used(&vision_tx_len_fifo)) {
                        vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                        memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                        fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[0], vision_dma_send_data_len);
                    }
                } else {
                    vision_dma_send_data_len = 0;
                    Vision_No_DMA_IRQHandler = 1;
                    memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                }
            } else if (UART_SEND_MODE == Byte_MODE) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                    vision_dma_send_data_len = 1;
                    fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[0], vision_dma_send_data_len);

                } else {
                    vision_dma_send_data_len = 0;
                    Vision_No_DMA_IRQHandler = 1;
                    memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                }
            }
        }
    } else {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
        } else {
            memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
        }
    }
}