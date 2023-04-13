//
// Created by Ken_n on 2023/4/12.
//

#include "super_capacitance_control_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DWT.h"
#include "CAN_receive.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t super_capacitance_control_stack;
#endif
const volatile RC_ctrl_t *super_capacitance_rc;
/**
  * @brief          超级电容输入功率控制任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void super_capacitance_control_task(void const *pvParameters) {
    super_capacitance_rc = get_remote_control_point();
    TickType_t LoopStartTime;
    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_super_capacitance_control_task);
        LoopStartTime = xTaskGetTickCount();
        if (toe_is_error(DBUS_TOE)) {
            CAN1_cmd_0x210(SUPER_CAPACITANCE_30W_MIN);
        } else{
            CAN1_cmd_0x210(SUPER_CAPACITANCE_80W);
        }
//        if (!toe_is_error(DBUS_TOE)) {
//            bullet_box_control();
//        }


#if INCLUDE_uxTaskGetStackHighWaterMark
        super_capacitance_control_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(100));
    }
}

/**
  * @brief          获取servo_task栈大小
  * @param[in]      none
  * @retval         servo_task:任务堆栈大小
  */
uint32_t get_stack_of_super_capacitance_control_task(void) {
    return super_capacitance_control_stack;
}
