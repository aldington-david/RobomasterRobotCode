//
// Created by Ken_n on 2022/4/18.
//
#include <stdlib.h>
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
#include "print_task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t vision_rx_task_stack;
uint32_t vision_tx_task_stack;
uint32_t USART1TX_active_task_stack;
#endif

uint8_t vision_fifo_rx_buf[VISION_FIFO_BUF_LENGTH];
uint8_t vision_fifo_tx_len_buf[VISION_FIFO_BUF_LENGTH];
uint8_t vision_fifo_tx_buf[VISION_FIFO_BUF_LENGTH];
fifo_s_t vision_rx_fifo;
fifo_s_t vision_tx_len_fifo;
fifo_s_t vision_tx_fifo;
uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];
uint8_t usart1_vision_tx_buf[2][USART1_VISION_TX_BUF_LENGHT];
TaskHandle_t USART1TX_active_task_local_handler;
TaskHandle_t vision_rx_task_local_handler;
volatile uint8_t Vision_No_DMA_IRQHandler = 1;
volatile uint8_t vision_dma_send_data_len = 0;
volatile uint8_t Vision_IRQ_Return_Before = 0;
vision_unpack_data_t vision_unpack_obj;
vision_info_t global_vision_info;

/**
  * @brief          视觉接收任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void vision_rx_task(void const *argument) {
    init_vision_struct_data();
    vision_rx_task_local_handler = xTaskGetCurrentTaskHandle();
    fifo_s_init(&vision_rx_fifo, vision_fifo_rx_buf, VISION_FIFO_BUF_LENGTH);
    usart1_rx_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_LENGHT);
    while (1) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }
        vision_unpack_fifo_data();
#if INCLUDE_uxTaskGetStackHighWaterMark
        vision_rx_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
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
#if INCLUDE_uxTaskGetStackHighWaterMark
        vision_tx_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

/**
  * @brief          获取vision_tx_task栈大小
  * @param[in]      none
  * @retval         vision_tx_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_vision_tx_task(void) {
    return vision_tx_task_stack;
}

/**
  * @brief          获取vision_rx_task栈大小
  * @param[in]      none
  * @retval         vision_rx_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_vision_rx_task(void) {
    return vision_rx_task_stack;
}

/**
  * @brief          获取USART1TX_active_task栈大小
  * @param[in]      none
  * @retval         USART1TX_active_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_USART1TX_active_task(void) {
    return USART1TX_active_task_stack;
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
void vision_unpack_fifo_data(void) {
    uint8_t byte = 0;
    vision_unpack_data_t *p_obj = &vision_unpack_obj;
    while (fifo_s_used(&vision_rx_fifo)) {
        byte = fifo_s_get(&vision_rx_fifo);
        switch (p_obj->unpack_step) {
            case VISION_STEP_HEADER_SOF: {
                if (byte == VISION_HEADER_SOF) {
//                    SEGGER_RTT_printf(0,"head=%d,sep=%d,end=%d\r\n",p_obj->head_sof_cnt,p_obj->separate_sof_cnt,p_obj->end_sof_cnt);
                    p_obj->protocol_packet[p_obj->index++] = byte;
                    if (fifo_s_preread(&vision_rx_fifo, 0) != VISION_HEADER_SOF) {
                        p_obj->unpack_step = VISION_STEP_LEN;
                    } else {
                        p_obj->index = 0;
                    }
                } else {
                    p_obj->index = 0;
                }
            }
                break;

            case VISION_STEP_LEN: {
//                SEGGER_RTT_WriteString(0, "have_date1\r\n");
                p_obj->frame.header.data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = VISION_STEP_FRAME_CRC16;
            }
                break;
            case VISION_STEP_FRAME_CRC16: {
//                SEGGER_RTT_WriteString(0, "have_date2\r\n");
                if (p_obj->index < (sizeof(vision_sync_struct))) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (sizeof(vision_sync_struct))) {
                    p_obj->unpack_step = VISION_STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
                if (verify_CRC16_check_sum(p_obj->protocol_packet,
                                           (sizeof(vision_sync_struct)))) {
                    vision_update(p_obj->protocol_packet);
                }
                break;

                default: {
                    p_obj->unpack_step = VISION_STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
                break;
            }
        }
    }
}


void vision_update(uint8_t *rxBuf) {
    uint8_t res = false;
    vision_info_t *vision_info = &global_vision_info;
    vision_info->pack_info = &vision_unpack_obj;
    if (rxBuf[0] == VISION_HEADER_SOF) {
        if (verify_CRC16_check_sum(rxBuf, (sizeof(vision_frame_header) + vision_info->pack_info->frame.header.data_len +
                                           sizeof(vision_frame_tail))) == true) {
            res = true;
            memcpy(&global_vision_info.vision_control, (rxBuf + sizeof(vision_frame_header)),
                   vision_info->pack_info->frame.header.data_len);
        }
        if (rxBuf[sizeof(vision_sync_struct)] == VISION_HEADER_SOF) {
            vision_update(&rxBuf[sizeof(vision_sync_struct)]);
        }

    }
    vision_info->pack_info->data_valid = res;
}


/**
  * @brief          获取视觉数据指针
  * @param[in]      none
  * @retval         视觉数据指针
  */
const volatile vision_control_t *get_vision_control_point(void) {
    return &global_vision_info.vision_control;
}

/**
  * @brief  视觉数据内存空间初始化
  * @param  None
  * @retval None
  */
void init_vision_struct_data(void) {
    memset(&global_vision_info, 0, sizeof(vision_info_t));
}

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
        if (UART1_TARGET_MODE == Vision_MODE) {
            if (Vision_No_DMA_IRQHandler || Vision_IRQ_Return_Before) {
                if (fifo_s_used(&vision_tx_fifo)) {
                    if (fifo_s_used(&vision_tx_len_fifo)) {
                        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                            if (vision_dma_send_data_len > 1) {
                                Vision_IRQ_Return_Before = 0;
                            }
                            memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
                            fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[1],
                                        vision_dma_send_data_len);
                            huart1.hdmatx->Instance->CR |= DMA_SxCR_CT;
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Vision_No_DMA_IRQHandler = 0;
                            vision_dma_send_data_len = 0;
                            detect_hook(USART1_TX_TOE);
                            taskENTER_CRITICAL();
                            if (!Vision_IRQ_Return_Before) {
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
                            }
                            taskEXIT_CRITICAL();
                        } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            vision_dma_send_data_len = fifo_s_get(&vision_tx_len_fifo);
                            if (vision_dma_send_data_len > 1) {
                                Vision_IRQ_Return_Before = 0;
                            }
                            memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
                            fifo_s_gets(&vision_tx_fifo, (char *) usart1_vision_tx_buf[0],
                                        vision_dma_send_data_len);
                            huart1.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Vision_No_DMA_IRQHandler = 0;
                            vision_dma_send_data_len = 0;
                            detect_hook(USART1_TX_TOE);
                            taskENTER_CRITICAL();
                            if (!Vision_IRQ_Return_Before) {
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
                            taskEXIT_CRITICAL();
#if INCLUDE_uxTaskGetStackHighWaterMark
                            USART1TX_active_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
                        }
                    }
                }
            }
        } else if (UART1_TARGET_MODE == Matlab_MODE) {
            if (Matlab_No_DMA_IRQHandler || Matlab_IRQ_Return_Before) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
                        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                            SEGGER_RTT_WriteString(0, "ST0DMA_1\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                            if (matlab_dma_send_data_len > 1) {
                                Matlab_IRQ_Return_Before = 0;
                            }
                            memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1],
                                        matlab_dma_send_data_len);
//                            SEGGER_RTT_printf(0, "2len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                            SEGGER_RTT_printf(0, "2fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                            huart1.hdmatx->Instance->CR |= DMA_SxCR_CT;
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Matlab_No_DMA_IRQHandler = 0;
                            matlab_dma_send_data_len = 0;
                            detect_hook(USART1_TX_TOE);
                            taskENTER_CRITICAL();
                            if (!Matlab_IRQ_Return_Before) {
                                if (fifo_s_used(&matlab_tx_fifo)) {
                                    if (fifo_s_used(&matlab_tx_len_fifo)) {
//                                        SEGGER_RTT_printf(0, "3len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                                        SEGGER_RTT_printf(0, "3fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                                        matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                                        memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0],
                                                    matlab_dma_send_data_len);
                                    }
                                } else {
//                                    SEGGER_RTT_WriteString(0, "in_else\r\n");
                                    matlab_dma_send_data_len = 0;
                                    Matlab_No_DMA_IRQHandler = 1;
                                    memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                }
                            }
                            taskEXIT_CRITICAL();
                        } else {
//                            SEGGER_RTT_WriteString(0, "ST0DMA_0\r\n");
                            __HAL_DMA_DISABLE(huart1.hdmatx);
                            matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                            if (matlab_dma_send_data_len > 1) {
                                Matlab_IRQ_Return_Before = 0;
                            }
                            memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                            fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0],
                                        matlab_dma_send_data_len);
//                            SEGGER_RTT_printf(0, "2len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                            SEGGER_RTT_printf(0, "2fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                            huart1.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
                            __HAL_DMA_ENABLE(huart1.hdmatx);
                            Matlab_No_DMA_IRQHandler = 0;
                            matlab_dma_send_data_len = 0;
                            detect_hook(USART1_TX_TOE);
                            taskENTER_CRITICAL();
//                            SEGGER_RTT_printf(0, "return0=%d\r\n", Matlab_IRQ_Return_Before);
                            if (!Matlab_IRQ_Return_Before) {
                                if (fifo_s_used(&matlab_tx_fifo)) {
                                    if (fifo_s_used(&matlab_tx_len_fifo)) {
//                                        SEGGER_RTT_printf(0, "3len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                                        SEGGER_RTT_printf(0, "3fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                                        matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                                        memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1],
                                                    matlab_dma_send_data_len);
                                    }
                                } else {
//                                    SEGGER_RTT_WriteString(0, "in_else\r\n");
                                    matlab_dma_send_data_len = 0;
                                    Matlab_No_DMA_IRQHandler = 1;
                                    memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                                }
                            }
                            taskEXIT_CRITICAL();
                        }
                    }
                }
            }
        }
#if INCLUDE_uxTaskGetStackHighWaterMark
        USART1TX_active_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
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
            if (UART1_TARGET_MODE == Vision_MODE) {
                fifo_s_puts(&vision_rx_fifo, (char *) usart1_rx_buf[0], this_time_rx_len);
                detect_hook(VISION_RX_TOE);
                if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                    static BaseType_t xHigherPriorityTaskWoken;
                    vTaskNotifyGiveFromISR(vision_rx_task_local_handler, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            } else if (UART1_TARGET_MODE == Matlab_MODE) {
                if (memchr(usart1_rx_buf[0], '$', this_time_rx_len) != NULL) {
                    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                        static BaseType_t xHigherPriorityTaskWoken;
                        vTaskNotifyGiveFromISR(matlab_tx_task_local_handler, &xHigherPriorityTaskWoken);
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                }
            }
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART1_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (UART1_TARGET_MODE == Vision_MODE) {
                fifo_s_puts(&vision_rx_fifo, (char *) usart1_rx_buf[1], this_time_rx_len);
                detect_hook(VISION_RX_TOE);
                if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                    static BaseType_t xHigherPriorityTaskWoken;
                    vTaskNotifyGiveFromISR(vision_rx_task_local_handler, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            } else if (UART1_TARGET_MODE == Matlab_MODE) {
                if (memchr(usart1_rx_buf[1], '$', this_time_rx_len) != NULL) {
                    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                        static BaseType_t xHigherPriorityTaskWoken;
                        vTaskNotifyGiveFromISR(matlab_tx_task_local_handler, &xHigherPriorityTaskWoken);
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                }
            }
        }
    }
}

/**
  * @brief          发送串口（USART1）DMA回调函数
  * @param[in]      void
  * @retval         none
  */
void DMA2_Stream7_IRQHandler(void) {
    if (UART1_TARGET_MODE == Vision_MODE) {
        if (__HAL_DMA_GET_FLAG(huart1.hdmatx, DMA_HISR_TCIF7) != RESET) {
            MY_USART_DMA_Stream7_Vision_TX_IRQHandler();
        }
    } else if (UART1_TARGET_MODE == Matlab_MODE) {
        if (__HAL_DMA_GET_FLAG(huart1.hdmatx, DMA_HISR_TCIF7) != RESET) {
            MY_USART_DMA_Stream7_Matlab_TX_IRQHandler();
        }
    }
}


void MY_USART_DMA_Stream7_Vision_TX_IRQHandler(void) {
    __HAL_DMA_DISABLE(huart1.hdmatx);
    while (huart1.hdmatx->Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(huart1.hdmatx);
    }
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart1.hdmatx));
    if ((!fifo_s_used(&vision_tx_fifo)) || (!fifo_s_used(&vision_tx_len_fifo))) {
        if ((fifo_s_used(&vision_tx_fifo)) || (fifo_s_used(&vision_tx_len_fifo))) {
            //should not come in here
            fifo_s_flush(&vision_tx_fifo);
            fifo_s_flush(&vision_tx_len_fifo);
        }
    }
    if (vision_dma_send_data_len == 1) {
        Vision_IRQ_Return_Before = 1;
    }
    if (vision_dma_send_data_len) {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            vision_dma_send_data_len = 0;
            detect_hook(USART1_TX_TOE);
            if (!Vision_IRQ_Return_Before) {
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
        } else {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, vision_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            vision_dma_send_data_len = 0;
            detect_hook(USART1_TX_TOE);
            if (!Vision_IRQ_Return_Before) {
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
            }
        }
    } else {
        memset(&usart1_vision_tx_buf[1], 0, USART1_VISION_TX_BUF_LENGHT);
        memset(&usart1_vision_tx_buf[0], 0, USART1_VISION_TX_BUF_LENGHT);
        if (!Vision_No_DMA_IRQHandler) {
            Vision_IRQ_Return_Before = 1;
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
//                SEGGER_RTT_WriteString(0, "yield\r\n");
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(USART1TX_active_task_local_handler, &xHigherPriorityTaskWoken);
            }
        }
    }
}