//
// Created by Ken_n on 2022/4/18.
//

#ifndef ROBOMASTERROBOTCODE_VISION_TASK_H
#define ROBOMASTERROBOTCODE_VISION_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "struct_typedef.h"
#include "fifo.h"
#include "cmsis_os.h"

#define USART1_RX_BUF_LENGHT     512
#define USART1_VISION_TX_BUF_LENGHT     128
#define VISION_FIFO_BUF_LENGTH 1024
/*************define for unpack start*********************/
#define VISION_HEADER_SOF 0x24
#define Vision_PROTOCOL_FRAME_MAX_SIZE         128

typedef enum {
    VISION_STEP_HEADER_SOF = 0,
    VISION_STEP_LEN,
    VISION_STEP_FRAME_CRC16,
} vision_unpack_step_e;


#pragma pack(push, 1)


typedef struct {
    char sof;
    uint8_t data_len;
} vision_frame_header;

typedef struct {
    float data1;
    float data2;
    uint16_t data3;
} vision_frame_data;

typedef struct {
    uint16_t data_CRC16;
} vision_frame_tail;

typedef struct {
    vision_frame_header header;
    vision_frame_data data;
    vision_frame_tail tail;
} vision_sync_struct;

typedef struct {
    bool data_valid;
    vision_sync_struct frame;
    uint8_t protocol_packet[Vision_PROTOCOL_FRAME_MAX_SIZE];
    vision_unpack_step_e unpack_step;
    uint16_t index;
} vision_unpack_data_t;

typedef volatile struct {
    fp32 pitch_angle;
    fp32 yaw_angle;
    uint16_t fps;
    volatile bool update_flag;
} vision_control_t;

typedef volatile struct {
    vision_control_t vision_control;
    vision_unpack_data_t *pack_info;
} vision_info_t;

#pragma pack(pop)
/*************define for unpack end*********************/

extern volatile uint8_t Vision_No_DMA_IRQHandler;
extern volatile uint8_t vision_dma_send_data_len;
extern volatile uint8_t Vision_IRQ_Return_Before;

extern fifo_s_t vision_rx_fifo;
extern fifo_s_t vision_tx_len_fifo;
extern fifo_s_t vision_tx_fifo;
extern uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];
extern uint8_t usart1_vision_tx_buf[2][USART1_VISION_TX_BUF_LENGHT];
extern vision_info_t global_vision_info;

void USART1_IRQHandler(void);

void DMA2_Stream7_IRQHandler(void);

/**
  * @brief          获取vision_tx_task栈大小
  * @param[in]      none
  * @retval         vision_tx_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_vision_tx_task(void);

/**
  * @brief          清除视觉更新标志，外部函数调用(const安全性)
  * @param[in]      none
  * @retval         none
  */
extern void clear_vision_update_flag(void);

/**
  * @brief          获取vision_rx_task栈大小
  * @param[in]      none
  * @retval         vision_rx_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_vision_rx_task(void);

/**
  * @brief          获取USART1TX_active_task栈大小
  * @param[in]      none
  * @retval         USART1TX_active_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_USART1TX_active_task(void);

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

void vision_update(uint8_t *rxBuf);

/**
  * @brief          获取视觉数据指针
  * @param[in]      none
  * @retval         视觉数据指针
  */
extern const vision_control_t *get_vision_control_point(void);

/**
  * @brief  视觉数据内存空间初始化
  * @param  None
  * @retval None
  */
void init_vision_struct_data(void);

void MY_USART_DMA_Stream7_Vision_TX_IRQHandler(void);

extern TaskHandle_t USART1TX_active_task_local_handler;
extern TaskHandle_t vision_rx_task_local_handler;

/**
  * @brief          usart1发送启动任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART1TX_active_task(void const *pvParameters);

#endif //ROBOMASTERROBOTCODE_VISION_TASK_H
