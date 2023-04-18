/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "global_control_define.h"
#include "DWT.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;



/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const *argument) {
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();
    TickType_t LoopStartTime;
    while (1) {
        DWT_get_time_interval_us(&global_task_time.tim_test_task);
        LoopStartTime = xTaskGetTickCount();
        error = 0;

        //find error
        //发现错误
        for (error_num = 0; error_num < REFEREE_RX_TOE; error_num++) {
            if (error_list_test_local[error_num].error_exist) {
                error = 1;
                break;
            }
        }

        //no error, stop buzzer
        //没有错误, 停止蜂鸣器
        if (error == 0 && last_error != 0) {
            buzzer_off();
        }
        //have error
        //有错误
        if (error) {
#if Block_Buzzer
#else
            buzzer_warn_error(error_num + 1);
#endif
        }
        last_error = error;
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num) {
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if (show_num == 0 && stop_num == 0) {
        show_num = num;
        stop_num = 100;
    } else if (show_num == 0) {
        stop_num--;
        buzzer_off();
    } else {
        static uint8_t tick = 0;
        tick++;
        if (tick < 50) {
            buzzer_off();
        } else if (tick < 100) {
            buzzer_on(1, VOLUME(50));
        } else {
            tick = 0;
            show_num--;
        }
    }
}


