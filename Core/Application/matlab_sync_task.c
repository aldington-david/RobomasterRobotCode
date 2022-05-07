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
#include "vision_task.h"


uint8_t matlab_fifo_tx_len_buf[MATLAB_FIFO_BUF_LENGTH];
uint8_t matlab_fifo_tx_buf[MATLAB_FIFO_BUF_LENGTH];
fifo_s_t matlab_tx_len_fifo;
fifo_s_t matlab_tx_fifo;
uint8_t usart1_matlab_tx_buf[2][USART1_MATLAB_TX_BUF_LENGHT];
volatile uint8_t Matlab_No_DMA_IRQHandler = 1;
volatile uint8_t matlab_dma_send_data_len = 0;
volatile uint8_t Matlab_IRQ_Return_Before = 0;
TaskHandle_t matlab_tx_task_local_handler;
/* 发送数据包缓存区，最大128字节 */
uint8_t matlab_transmit_pack[128];
static char sync_char = '$';
SyncStruct test1;

/**
  * @brief          matlab发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void matlab_sync_task(void const *argument) {
    init_matlab_struct_data();
    fifo_s_init(&matlab_tx_len_fifo, matlab_fifo_tx_len_buf, MATLAB_FIFO_BUF_LENGTH);
    fifo_s_init(&matlab_tx_fifo, matlab_fifo_tx_buf, MATLAB_FIFO_BUF_LENGTH);
    usart1_tx_init(usart1_matlab_tx_buf[0], usart1_matlab_tx_buf[1], USART1_MATLAB_TX_BUF_LENGHT);
    matlab_tx_task_local_handler = xTaskGetCurrentTaskHandle();
//    TickType_t LoopStartTime;
    while (1) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }
//        LoopStartTime = xTaskGetTickCount();
        test1.data1 = rand() % 65535;
        test1.data2 = rand() % 65535;
        test1.data3 = rand() % 65535;
        test1.data4 = rand() % 65535;
        test1.data5 = rand() % 65535;
        memcpy((void *) matlab_transmit_pack, &test1, sizeof(test1));
        data_sync(sizeof(test1));
//        data_sync(sizeof(test1));
//        data_sync(sizeof(test1));
//        data_sync(sizeof(test1));
//        referee_unpack_fifo_data();

//        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

void init_matlab_struct_data(void) {
    memset(&matlab_fifo_tx_len_buf, 0, MATLAB_FIFO_BUF_LENGTH);
    memset(&matlab_fifo_tx_buf, 0, MATLAB_FIFO_BUF_LENGTH);
    memset(&matlab_transmit_pack, 0, 128);
    memset(&usart1_matlab_tx_buf, 0, 2 * USART1_MATLAB_TX_BUF_LENGHT);
    test1.sync_char = '$';
}
//not_use
void send_sync_char(void) {
    fifo_s_put(&matlab_tx_len_fifo, sizeof(sync_char));
    fifo_s_puts(&matlab_tx_fifo, (char *) &sync_char, sizeof(sync_char));
}

void data_sync(int data_len) {
//    send_sync_char();
    fifo_s_put(&matlab_tx_len_fifo, data_len);
    fifo_s_puts(&matlab_tx_fifo, (char *) &matlab_transmit_pack, data_len);
//    SEGGER_RTT_printf(0, "0len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//    SEGGER_RTT_printf(0, "0fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
    if (Matlab_No_DMA_IRQHandler || Matlab_IRQ_Return_Before) {
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            xTaskNotifyGive(USART1TX_active_task_local_handler);
//            portYIELD();
        }
    }
}

void MY_USART_DMA_Stream7_Matlab_TX_IRQHandler(void) {
    __HAL_DMA_DISABLE(huart1.hdmatx);
    while (huart1.hdmatx->Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(huart1.hdmatx);
    }
    __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart1.hdmatx));;
//    SEGGER_RTT_printf(0, "5len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//    SEGGER_RTT_printf(0, "5fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
//    SEGGER_RTT_printf(0, "return1=%d\r\n", Matlab_IRQ_Return_Before);
    if ((!fifo_s_used(&matlab_tx_fifo)) || (!fifo_s_used(&matlab_tx_len_fifo))) {
        if ((fifo_s_used(&matlab_tx_fifo)) || (fifo_s_used(&matlab_tx_len_fifo))) {
            //should not come in here
            fifo_s_flush(&matlab_tx_fifo);
            fifo_s_flush(&matlab_tx_len_fifo);
        }
    }
//    SEGGER_RTT_printf(0, "len=%d\r\n", matlab_dma_send_data_len);
    if (matlab_dma_send_data_len == 1) {
        Matlab_IRQ_Return_Before = 1;
    }
    if (matlab_dma_send_data_len) {
        if ((huart1.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0\r\n");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            matlab_dma_send_data_len = 0;
            detect_hook(USART1_TX_TOE);
            if (!Matlab_IRQ_Return_Before) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
//                        SEGGER_RTT_printf(0, "7len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                        SEGGER_RTT_printf(0, "7fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                        matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                        memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[1], matlab_dma_send_data_len);
                    }
                } else {
                    matlab_dma_send_data_len = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
                }
            }
        } else {
            __HAL_DMA_DISABLE(huart1.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart1.hdmatx, DMA_HISR_TCIF7);
            __HAL_DMA_SET_COUNTER(huart1.hdmatx, matlab_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1\r\n");
            __HAL_DMA_ENABLE(huart1.hdmatx);
            matlab_dma_send_data_len = 0;
            detect_hook(USART1_TX_TOE);
            if (!Matlab_IRQ_Return_Before) {
                if (fifo_s_used(&matlab_tx_fifo)) {
                    if (fifo_s_used(&matlab_tx_len_fifo)) {
//                        SEGGER_RTT_printf(0, "7len_fifo=%d\r\n", fifo_s_used(&matlab_tx_len_fifo));
//                        SEGGER_RTT_printf(0, "7fifo=%d\r\n", fifo_s_used(&matlab_tx_fifo));
                        matlab_dma_send_data_len = fifo_s_get(&matlab_tx_len_fifo);
                        memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                        fifo_s_gets(&matlab_tx_fifo, (char *) usart1_matlab_tx_buf[0], matlab_dma_send_data_len);
                    }
                } else {
                    matlab_dma_send_data_len = 0;
                    Matlab_No_DMA_IRQHandler = 1;
                    memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
                }
            }
        }
    } else {
        memset(&usart1_matlab_tx_buf[1], 0, USART1_MATLAB_TX_BUF_LENGHT);
        memset(&usart1_matlab_tx_buf[0], 0, USART1_MATLAB_TX_BUF_LENGHT);
        if (!Matlab_No_DMA_IRQHandler) {
            Matlab_IRQ_Return_Before = 1;
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
//                SEGGER_RTT_WriteString(0, "yield\r\n");
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(USART1TX_active_task_local_handler, &xHigherPriorityTaskWoken);
            }
        }
    }
}
