/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "detect_task.h"
#include "SEGGER_RTT.h"
#include "DWT.h"

#define SERVO_MIN_PWM   892
#define SERVO_MAX_PWM   1980
#define PWM_DETAL_VALUE 10

#define Bullet_Box_Open_PWM   1980
#define Bullet_Box_Close_PWM   892

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t servo_task_stack;
#endif

static servo_mode_e Bullet_Box_Mode = Bullet_Box_Min;
const volatile RC_ctrl_t *servo_rc;
//const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
//uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
uint16_t volatile bullet_box_pwm = SERVO_MIN_PWM;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const *argument) {
    servo_rc = get_remote_control_point();
    TickType_t LoopStartTime;
    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_referee_rx_task);
        LoopStartTime = xTaskGetTickCount();
        if (toe_is_error(DBUS_TOE)) {
            for (uint8_t i = 0; i < 7; i++) {
                servo_pwm_set(0, i);
            }
        }
        if (!toe_is_error(DBUS_TOE)) {
            bullet_box_control();
        }

        //abundant
//        for(uint8_t i = 0; i < 4; i++)
//        {
//
//            if( (servo_rc->key.v & SERVO_MINUS_PWM_KEY) && (servo_rc->key.v & servo_key[i]))
//            {
//                servo_pwm[i] -= PWM_DETAL_VALUE;
//            }
//            else if(servo_rc->key.v & servo_key[i])
//            {
//                servo_pwm[i] += PWM_DETAL_VALUE;
//            }
//
//            //limit the pwm
//           //限制pwm
//            if(servo_pwm[i] < SERVO_MIN_PWM)
//            {
//                servo_pwm[i] = SERVO_MIN_PWM;
//            }
//            else if(servo_pwm[i] > SERVO_MAX_PWM)
//            {
//                servo_pwm[i] = SERVO_MAX_PWM;
//            }
//
//            servo_pwm_set(servo_pwm[i], i);
//        }

#if INCLUDE_uxTaskGetStackHighWaterMark
        servo_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(13));
    }
}

/**
  * @brief          获取servo_task栈大小
  * @param[in]      none
  * @retval         servo_task:任务堆栈大小
  */
uint32_t get_stack_of_servo_task(void) {
    return servo_task_stack;
}

void bullet_box_control(void) {
    static char last_s = RC_SW_DOWN;
    static char last_s_switch = RC_SW_DOWN;
    if ((switch_is_up(servo_rc->rc.s[RADIO_CONTROL_SWITCH_L])) &&
        (switch_is_up(servo_rc->rc.s[RADIO_CONTROL_SWITCH_R]))) {
        if (servo_rc->key.v & KEY_PRESSED_OFFSET_V) {
            Bullet_Box_Mode = Bullet_Box_Max;
        } else if (servo_rc->key.v & KEY_PRESSED_OFFSET_B) {
            Bullet_Box_Mode = Bullet_Box_Min;
        }
    } else if (switch_is_up(servo_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
               switch_is_down(servo_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && !switch_is_down(last_s) &&
               !switch_is_down(last_s_switch) && Bullet_Box_Mode == Bullet_Box_Min) {
        Bullet_Box_Mode = Bullet_Box_Max;
    } else if (switch_is_up(servo_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
               switch_is_down(servo_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && !switch_is_down(last_s) &&
               !switch_is_down(last_s_switch) && Bullet_Box_Mode == Bullet_Box_Max) {
        Bullet_Box_Mode = Bullet_Box_Min;
    }
    if (Bullet_Box_Mode == Bullet_Box_Max) {
        bullet_box_pwm = Bullet_Box_Open_PWM;
    } else if (Bullet_Box_Mode == Bullet_Box_Min) {
        bullet_box_pwm = Bullet_Box_Close_PWM;
    }


//limit the pwm
//限制pwm
    if (bullet_box_pwm < SERVO_MIN_PWM) {
        bullet_box_pwm = SERVO_MIN_PWM;
    } else if (bullet_box_pwm > SERVO_MAX_PWM) {
        bullet_box_pwm = SERVO_MAX_PWM;
    }
//    SEGGER_RTT_printf(0,"bullet_box_init_pwm=%d",bullet_box_pwm);
    servo_pwm_set(bullet_box_pwm,
                  0);

    if (switch_is_up(servo_rc->rc.s[RADIO_CONTROL_SWITCH_L])) {
        last_s = servo_rc->rc.s[RADIO_CONTROL_SWITCH_R];
    }
    last_s_switch = servo_rc->rc.s[RADIO_CONTROL_SWITCH_R];
}

