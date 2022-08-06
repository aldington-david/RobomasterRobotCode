/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB灯效。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef LED_TRIGGER_TASK_H
#define LED_TRIGGER_TASK_H

#include <stdint.h>
#include "struct_typedef.h"


/**
  * @brief          获取led_RGB_flow_task栈大小
  * @param[in]      none
  * @retval         led_RGB_flow_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_led_RGB_flow_task(void);

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void led_RGB_flow_task(void const * argument);

#endif



