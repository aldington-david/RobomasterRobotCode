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

#define USART1_RX_BUF_LENGHT     512
#define USART1_VISION_TX_BUF_LENGHT     128
#define VISION_FIFO_BUF_LENGTH 1024
/*************define for unpack start*********************/
#define VISION_HEADER_SOF 0x24
#define VISION_SEPARATE_SOF 0x2C
#define VISION_END_SOF 0x40
#define Vision_PROTOCOL_FRAME_MAX_SIZE         128

typedef enum {
    VISION_STEP_HEADER_SOF = 0,
    VISION_STEP_SEPARATE_SOF = 1,
    VISION_STEP_END_SOF = 2,
    VISION_STEP_DATA_1,
    VISION_STEP_DATA_2,
} vision_unpack_step_e;


#pragma pack(push, 1)

typedef struct {
    bool data_valid;
    char Original_pitch_angle[128];
    char Original_yaw_angle[128];

    uint8_t protocol_packet[Vision_PROTOCOL_FRAME_MAX_SIZE];
    vision_unpack_step_e unpack_step;
    uint16_t index;
    uint16_t head_index;
    uint16_t separate_index;
    uint16_t end_index;
    uint16_t data1_len;
    uint16_t data2_len;
    uint8_t head_sof_cnt;
    uint8_t separate_sof_cnt;
    uint8_t end_sof_cnt;
} vision_unpack_data_t;

typedef volatile struct{
    fp32 pitch_angle;
    fp32 yaw_angle;
} vision_control_t;

typedef volatile struct{
    vision_control_t vision_control;
    vision_unpack_data_t *pack_info;
} vision_info_t;

#pragma pack(pop)
/*************define for unpack end*********************/

extern volatile uint8_t Vision_No_DMA_IRQHandler;
extern volatile uint8_t vision_dma_send_data_len;

extern fifo_s_t vision_rx_fifo;
extern fifo_s_t vision_tx_len_fifo;
extern fifo_s_t vision_tx_fifo;
extern uint8_t usart1_rx_buf[2][USART1_RX_BUF_LENGHT];
extern uint8_t usart1_vision_tx_buf[2][USART1_VISION_TX_BUF_LENGHT];
extern vision_info_t global_vision_info;

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
