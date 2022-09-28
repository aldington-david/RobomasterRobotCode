/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       voltage_task.c/h
  * @brief      24v power voltage ADC task, get voltage and calculate electricity
  *             percentage.24电源电压ADC任务,获取电压并且计算电量百分比.
  * @note       when power is not derectly link to delelopment, please change VOLTAGE_DROP
  *             当电源不直连开发板,请修改VOLTAGE_DROP
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
#include "voltage_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "user_lib.h"
#include "DWT.h"

#define FULL_BATTER_VOLTAGE     25.2f
#define LOW_BATTER_VOLTAGE      22.2f   //about 20% 

#define VOLTAGE_DROP            0.00f


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t battery_voltage_task_stack;
#endif

static float32_t calc_battery_percentage(float voltage);


float32_t battery_voltage;
float32_t electricity_percentage;

/**
  * @brief          power ADC and calculate electricity percentage
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void battery_voltage_task(void const *argument) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    //use inner 1.2v to calbrate
    init_vrefint_reciprocal();
    TickType_t LoopStartTime;
    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_battery_voltage_task);
        LoopStartTime = xTaskGetTickCount();
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
        electricity_percentage = calc_battery_percentage(battery_voltage);
#if INCLUDE_uxTaskGetStackHighWaterMark
        battery_voltage_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(100));
    }
}

/**
  * @brief          获取battery_voltage_task栈大小
  * @param[in]      none
  * @retval         battery_voltage_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_battery_voltage_task(void) {
    return battery_voltage_task_stack;
}

static float32_t calc_battery_percentage(float voltage) {
    float32_t percentage;
    float32_t voltage_2 = voltage * voltage;
    float32_t voltage_3 = voltage_2 * voltage;

    if (voltage < 19.5f) {
        percentage = 0.0f;
    } else if (voltage < 21.9f) {
        percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
    } else if (voltage < 25.5f) {
        percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
    } else {
        percentage = 1.0f;
    }
    if (percentage < 0.0f) {
        percentage = 0.0f;
    } else if (percentage > 1.0f) {
        percentage = 1.0f;
    }
    //another formulas
    //另一套公式
//    if(voltage < 19.5f)
//    {
//        percentage = 0.0f;
//    }
//    else if(voltage < 22.5f)
//    {
////        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f * voltage + 383.4f) + 0.5f;
//        percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f * voltage - 427.02953125f;
//    }
//    else if(voltage < 25.5f)
//    {
////        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f * voltage + 637.0f) + 0.5f;
//        percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f * voltage - 254.903125f;
//    }
//    else
//    {
//        percentage = 1.0f;
//    }

    return percentage;
}

uint16_t get_battery_percentage(void) {
    return (uint16_t) (electricity_percentage * 100.0f);
}


