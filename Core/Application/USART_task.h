//
// Created by Ken_n on 2022/4/16.
//

#include "task.h"

#ifndef ROBOMASTERROBOTCODE_USART_TASK_H
#define ROBOMASTERROBOTCODE_USART_TASK_H

extern TaskHandle_t USART_task_local_handler;

/**
  * @brief          串口发送任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART_task(void const *pvParameters);


//for_backup
void USART_sent_init(void);

#endif //ROBOMASTERROBOTCODE_USART_TASK_H

