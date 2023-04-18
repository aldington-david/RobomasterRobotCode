//
// Created by Ken_n on 2023/4/14.
//

#include "pid_auto_tune_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "DWT.h"
#include "remote_control.h"
#include "detect_task.h"
#include "CAN_receive.h"
#include "PID_AutoTune.h"
#include "gimbal_task.h"
#include "global_control_define.h"
#include "SEGGER_RTT.h"
#include "print_task.h"
#include "chassis_task.h"
#include "shoot.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t pid_auto_tune_stack;
#endif

pid_auto_tune_t pid_auto_tune_data;
const volatile RC_ctrl_t *pid_auto_tune_rc;

//调整顺序，1.底盘电流环,放地上，四个电机给同一电流，自转 2.底盘速度，放地上带摩擦力调（不跟随）3，云台速度，角度（不跟随，陀螺）4，底盘跟随角度，以云台绝对角度为准，设定到保持当前云台角度下动底盘(理论前面调好了P直接设为2)，5，小陀螺云台逆向旋转前馈，6，拨盘角度速度
//克服重力调整务必找到平衡时电流并以此为起点上下震动，例如所有的pitch
//电流起始2000
//除了pitch需要测起始值一般不用设StartValue,0即可
//自动调前取消yaw pitch init
//外环pid（包括速度环）要乘ostep
//反转的电机给定ostep与输出pid值都为负，自行取正
/**
  * @brief          pid自动调参任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void pid_auto_tune_task(void const *pvParameters) {
    vTaskDelay(1000);
    pid_auto_tune_rc = get_remote_control_point();
    //底盘速度-电流
//    pid_auto_tune_init(&pid_auto_tune_data, &chassis_move.motor_chassis[0].speed,
//                       &pid_auto_tune_data.control_list.wheel_control_value, USE_PI, 0.35f, 2000.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_CURRENT, 0);
    //底盘速度vx
//    pid_auto_tune_init(&pid_auto_tune_data, &chassis_move.vx,
//                       &pid_auto_tune_data.control_list.vx_control_value, USE_P, 0.1f, 3.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_SPEED, 0);
    //底盘速度vy
//    pid_auto_tune_init(&pid_auto_tune_data, &chassis_move.vy,
//                       &pid_auto_tune_data.control_list.vy_control_value, USE_P, 0.1f, 3.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_SPEED, 0);
    //底盘速度wz
//    pid_auto_tune_init(&pid_auto_tune_data, &chassis_move.wz,
//                       &pid_auto_tune_data.control_list.wz_control_value, USE_P, 0.0f, 3.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_SPEED, 0);
    //yaw_speed
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_yaw_motor.motor_gyro,
//                       &pid_auto_tune_data.control_list.yaw_control_value, USE_PI, 0.0f, -3000.0f,
//                       1, 0.0f, 0.0f,SPEED_TO_CURRENT,0);
    //yaw_relative_angle
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_yaw_motor.relative_angle,
//                       &pid_auto_tune_data.control_list.yaw_control_value, USE_PID, 0.0f, 2.0f,
//                       1, 3.0f, 0.0f, ANGLE_TO_SPEED, 0);
    //yaw_absolute_angle
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_yaw_motor.absolute_angle,
//                       &pid_auto_tune_data.control_list.yaw_control_value, USE_PID, 0.0f, 1.9f,
//                       1, 3.0f, 0.0f, ANGLE_TO_SPEED, 0);
    //pitch_speed
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_pitch_motor.motor_gyro,
//                       &pid_auto_tune_data.control_list.pitch_control_value, USE_PI, 0.0f, 3500.0f,
//                       5, 0.0f, 0.0f,SPEED_TO_CURRENT,0);
    //pitch_relative_angle
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_pitch_motor.relative_angle,
//                       &pid_auto_tune_data.control_list.pitch_control_value, USE_PID, 0.0f, 0.8f,
//                       5, 0.0f, 0.0f,ANGLE_TO_SPEED,0);
    //pitch_absolute_angle
//    pid_auto_tune_init(&pid_auto_tune_data, &gimbal_control.gimbal_pitch_motor.absolute_angle,
//                       &pid_auto_tune_data.control_list.pitch_control_value, USE_PID, 0.0f, 0.8f,
//                       5, 0.0f, 0.0f,ANGLE_TO_SPEED,0);
    //trigger_speed
//    pid_auto_tune_init(&pid_auto_tune_data, &shoot_control.speed,
//                       &pid_auto_tune_data.control_list.trigger_control_value, USE_PI, 0.0f, 3500.0f,
//                       1, 0.0f, 0.0f,SPEED_TO_CURRENT,0);
    //拨盘电机角度环
//    pid_auto_tune_init(&pid_auto_tune_data, &shoot_control.angle,
//                       &pid_auto_tune_data.control_list.trigger_control_value, USE_PID, 0.0f, 1.0f,
//                       1, 0.0f, 0.0f, ANGLE_TO_SPEED, 0);
    //摩擦轮1速度环
//    pid_auto_tune_init(&pid_auto_tune_data, &shoot_control.fric1_speed,
//                       &pid_auto_tune_data.control_list.fric1_control_value, USE_PI, 0.0f, 3000.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_CURRENT, 0);
    //摩擦轮2速度环
//    pid_auto_tune_init(&pid_auto_tune_data, &shoot_control.fric2_speed,
//                       &pid_auto_tune_data.control_list.fric2_control_value, USE_PI, 0.0f, 3000.0f,
//                       1, 0.0f, 0.0f, SPEED_TO_CURRENT, 0);
    TickType_t LoopStartTime;
    while (1) {
        DWT_get_time_interval_us(&global_task_time.tim_pid_auto_tune_task);
        LoopStartTime = xTaskGetTickCount();
        if (PID_AUTO_TUNE) {
            if (toe_is_error(DBUS_TOE)) {
                CAN2_cmd_0x1ff(0, 0, 0, 0);
                CAN1_cmd_0x1ff(0, 0, 0, 0);
                CAN2_cmd_0x200(0, 0, 0, 0);
                CAN1_cmd_0x200(0, 0, 0, 0);
//                pid_auto_tune_cancel(&pid_auto_tune_data);
            } else {
                if (pid_auto_tune_data.check_StartValue_mode) {
                    pid_auto_tune_set_check_StartValue_loop(&pid_auto_tune_data);
                } else {
                    if (running_flag) {
                        if (!pid_auto_tune_runtime(&pid_auto_tune_data)) {
                            //底盘速度-电流
//                            RTT_PrintWave(2,
//                                          &chassis_move.motor_chassis[0].speed,
//                                          &pid_auto_tune_data.control_list.wheel_control_value);
//                            //底盘速度vx
//                            RTT_PrintWave(2,
//                                          &chassis_move.vx,
//                                          &pid_auto_tune_data.control_list.vx_control_value);
                            //底盘速度vy
//                            RTT_PrintWave(2,
//                                          &chassis_move.vy,
//                                          &pid_auto_tune_data.control_list.vy_control_value);
                            //底盘速度vy
//                            RTT_PrintWave(2,
//                                          &chassis_move.wz,
//                                          &pid_auto_tune_data.control_list.wz_control_value);
                            //yaw_speed
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_yaw_motor.motor_gyro,
//                                          &pid_auto_tune_data.control_list.yaw_control_value);
                            //yaw_relative_angle
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_yaw_motor.relative_angle,
//                                          &pid_auto_tune_data.control_list.yaw_control_value);
                            //yaw_absolute_angle
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_yaw_motor.absolute_angle,
//                                          &pid_auto_tune_data.control_list.yaw_control_value);
                            //pitch_speed
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_pitch_motor.motor_gyro,
//                                          &pid_auto_tune_data.control_list.pitch_control_value);
                            //pitch_relative_angle
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_pitch_motor.relative_angle,
//                                          &pid_auto_tune_data.control_list.pitch_control_value);
                            //底盘跟随
//                            RTT_PrintWave(2,
//                                          &gimbal_control.gimbal_yaw_motor.relative_angle,
//                                          &pid_auto_tune_data.control_list.wz_control_value);
                            //拨盘速度
                            RTT_PrintWave(2,
                                          &shoot_control.speed,
                                          &pid_auto_tune_data.control_list.trigger_control_value);
                            //拨盘角度
                            RTT_PrintWave(2,
                                          &shoot_control.angle,
                                          &pid_auto_tune_data.control_list.trigger_control_value);
                            //摩擦轮1速度
                            RTT_PrintWave(2,
                                          &shoot_control.fric1_speed,
                                          &pid_auto_tune_data.control_list.fric1_control_value);
                            //摩擦轮2速度
                            RTT_PrintWave(2,
                                          &shoot_control.fric2_speed,
                                          &pid_auto_tune_data.control_list.fric2_control_value);


                        } else {
                            SEGGER_RTT_printf(0, "kp=%f,ki=%f,kd=%f\r\n", pid_auto_tune_getKp(&pid_auto_tune_data),
                                              pid_auto_tune_getKi(&pid_auto_tune_data),
                                              pid_auto_tune_getKd(&pid_auto_tune_data));
                            pid_auto_tune_data.running = false;
                        }
                    }
                }
            }
        }
#if INCLUDE_uxTaskGetStackHighWaterMark
        pid_auto_tune_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(1));
    }
}

/**
  * @brief          获取pid自动调谐控制指针
  * @param[in]      none
  * @retval         pid自动调谐控制指针
  */
const volatile pid_auto_tune_t *get_pid_auto_tune_data_point(void) {
    return &pid_auto_tune_data;
}

void pid_auto_tune_set_check_StartValue_loop(pid_auto_tune_t *pidtune) {
    static uint32_t print_time = 0;
    print_time = HAL_GetTick();
    pidtune->oStep = 0;
    pidtune->setpoint = 0;
    pidtune->noiseBand = 0;
    if (pidtune->tune_type == SPEED_TO_CURRENT) {
        if (fabsf(*pidtune->input) > 1.0f) {
            CAN2_cmd_0x1ff(0, 0, 0, 0);
            CAN1_cmd_0x1ff(0, 0, 0, 0);
            CAN2_cmd_0x200(0, 0, 0, 0);
            SEGGER_RTT_SetTerminal(1);
            SEGGER_RTT_WriteString(0, "TOO_LARGE\r\n");
        } else {
            if ((HAL_GetTick() - print_time) > 1000 && (HAL_GetTick() - print_time) < 294967295) {
                SEGGER_RTT_SetTerminal(1);
                SEGGER_RTT_printf(0, "now_set=%f\r\n", *pidtune->output);
            }
        }
    } else if (pidtune->tune_type == ANGLE_TO_SPEED) {
        if (fabsf(*pidtune->input) > 0.5f) {
            CAN2_cmd_0x1ff(0, 0, 0, 0);
            CAN1_cmd_0x1ff(0, 0, 0, 0);
            CAN2_cmd_0x200(0, 0, 0, 0);
            SEGGER_RTT_SetTerminal(1);
            SEGGER_RTT_WriteString(0, "TOO_LARGE\r\n");
        } else {
            if ((HAL_GetTick() - print_time) > 1000 && (HAL_GetTick() - print_time) < 294967295) {
                SEGGER_RTT_SetTerminal(1);
                SEGGER_RTT_printf(0, "now_set=%f\r\n", *pidtune->output);
            }
        }
    } else if (pidtune->tune_type == SPEED_TO_SPEED) {
        if (fabsf(*pidtune->input) > 1.1f) {
            CAN2_cmd_0x1ff(0, 0, 0, 0);
            CAN1_cmd_0x1ff(0, 0, 0, 0);
            CAN2_cmd_0x200(0, 0, 0, 0);
            SEGGER_RTT_SetTerminal(1);
            SEGGER_RTT_WriteString(0, "TOO_LARGE\r\n");
        } else {
            if ((HAL_GetTick() - print_time) > 1000 && (HAL_GetTick() - print_time) < 294967295) {
                SEGGER_RTT_SetTerminal(1);
                SEGGER_RTT_printf(0, "now_set=%f\r\n", *pidtune->output);
            }
        }
    }
}
