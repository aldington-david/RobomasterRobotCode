//
// Created by Ken_n on 2023/4/12.
//

#include "super_capacitance_control_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DWT.h"
#include "CAN_receive.h"
#include "chassis_task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t super_capacitance_control_stack;
#endif
const volatile RC_ctrl_t *super_capacitance_rc;
volatile bool_t super_capacitance_enable_flag =0;

/**
  * @brief          超级电容输入功率控制任务，CAN ReSynchronization Jump Width 务必设高点，另外CAN芯片要给板子另行接电才工作
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void super_capacitance_control_task(void const *pvParameters) {
    super_capacitance_rc = get_remote_control_point();
    TickType_t LoopStartTime;
    static uint32_t super_capacitance_count = 0;
    static uint16_t key_count = 0;
    static uint16_t power_limit_preserve = 0;
    while (1) {
        DWT_get_time_interval_us(&global_task_time.tim_super_capacitance_control_task);
        LoopStartTime = xTaskGetTickCount();
        if (!toe_is_error(SUPER_CAPACITANCE_TOE)) {
            if (toe_is_error(DBUS_TOE)) {
                CAN1_cmd_0x210(SUPER_CAPACITANCE_30W_MIN);
            } else {
                if ((switch_is_up(rc_ctrl.rc.s[RADIO_CONTROL_SWITCH_L])) &&
                    (switch_is_up(rc_ctrl.rc.s[RADIO_CONTROL_SWITCH_R]))) {
                    if ((rc_ctrl.key.v & (KEY_PRESSED_OFFSET_C)) && key_count == 0) {
                        if ((chassis_move.soft_power_limit - power_limit_preserve != SUPER_CAPACITANCE_ADD_W) &&
                            (chassis_move.soft_power_limit != 0)) {
                            key_count = 250;
                            power_limit_preserve = chassis_move.soft_power_limit;
                            chassis_move.soft_power_limit += SUPER_CAPACITANCE_ADD_W;
                            super_capacitance_enable_flag = 1;
                        }
                    }
                }
                if (key_count > 0) {
                    key_count--;
                }
                if (chassis_move.power_limit >= 30) {
                    CAN1_cmd_0x210(chassis_move.power_limit * 100);
                } else {
                    CAN1_cmd_0x210(SUPER_CAPACITANCE_30W_MIN);
                }
            }
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
