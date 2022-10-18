//
// Created by Ken_n on 2022/9/23.
//
#include "main.h"
#include "DWT.h"
#include <string.h>

task_time_record_t global_task_time;

void DWT_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    memset(&global_task_time, 0, sizeof(task_time_record_t));
}

void DWT_stop(void) {
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_get_tick(void) {
    return DWT->CYCCNT;
}

void DWT_update_task_time_us(time_record_struct *task_time) {
    if (DWT_get_tick() >= task_time->last_time) {
        task_time->time = (DWT_get_tick() - task_time->last_time) / (HAL_RCC_GetHCLKFreq() / 1000000U);
    } else {
        task_time->time = (DWT_get_tick() + (~(task_time->last_time) +1U)) / (HAL_RCC_GetHCLKFreq() / 1000000U);
    }
        task_time->last_time = DWT_get_tick();

}

