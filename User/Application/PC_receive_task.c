//
// Created by Ken_n on 2022/2/1.
//

#include "PC_receive_task.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "referee_task.h"
#include "print_task.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "shoot.h"
#include "vision_task.h"
#include "gimbal_behaviour.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t PC_receive_task_stack;
#endif

char Char_Receive_Buffer[256];
char *p;
int param_num;
char param_value[16];
int32_t param_value_int;
float param_value_float;
void *PC_receive_data[29];

//$param0=12;$param1=13;$param2=12.32;$param3=11.21;
void PC_receive_task(void const *argument) {
    /********vision start***********/
//    PC_receive_data[param0] = (void *) &vision_pitch_angle_deadband_sen;
//    PC_receive_data[param1] = (void *) &vision_yaw_angle_deadband_sen;
//    PC_receive_data[param2] = (void *) &vision_pitch_control_sen;
//    PC_receive_data[param3] = (void *) &vision_yaw_control_sen;
//    PC_receive_data[param4] = (void *) &vision_pitch_lpf_factor;
//    PC_receive_data[param5] = (void *) &vision_yaw_control_lpf_factor;

    /********vision end***********/

    /********pitch start***********/
//    PC_receive_data[param0] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.kp;
//    PC_receive_data[param1] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.ki;

//    PC_receive_data[param0] = (void *) &shoot_control.fric_all_speed;
////    PC_receive_data[param1] = (void *) &shoot_control.fric2_speed_set;
//    PC_receive_data[param2] = (void *) &shoot_control.fric1_motor_pid.Kp;
//    PC_receive_data[param3] = (void *) &shoot_control.fric1_motor_pid.Ki;
//    PC_receive_data[param4] = (void *) &shoot_control.fric1_motor_pid.Kd;
//    PC_receive_data[param5] = (void *) &shoot_control.fric1_motor_pid.Integral_Separation;
//    PC_receive_data[param6] = (void *) &shoot_control.fric2_motor_pid.Kp;
//    PC_receive_data[param7] = (void *) &shoot_control.fric2_motor_pid.Ki;
//    PC_receive_data[param8] = (void *) &shoot_control.fric2_motor_pid.Kd;
//    PC_receive_data[param9] = (void *) &shoot_control.fric2_motor_pid.Integral_Separation;

//    PC_receive_data[param0] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kp;
//    PC_receive_data[param1] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.ki;
//    PC_receive_data[param2] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kd;
//    PC_receive_data[param3] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kp;
//    PC_receive_data[param4] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Ki;
//    PC_receive_data[param5] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kd;
//    PC_receive_data[param6] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Integral_Separation;
//    PC_receive_data[param7] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.max_iout;
//    PC_receive_data[param8] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.max_out;
//    PC_receive_data[param9] = (void *) &gimbal_control.gimbal_pitch_motor.max_relative_angle;
//    PC_receive_data[param10] = (void *) &gimbal_control.gimbal_pitch_motor.min_relative_angle;
//    PC_receive_data[param11] = (void *) &gimbal_control.gimbal_pitch_motor.relative_angle_set;
//    PC_receive_data[param12] = (void *) &gimbal_control.gimbal_pitch_motor.Cloud_MotorAngle_Error_Kalman.R;
//    PC_receive_data[param13] = (void *) &gimbal_control.gimbal_pitch_motor.Cloud_MotorAngle_Error_Kalman.Q;
//    PC_receive_data[param14] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.R;
//    PC_receive_data[param15] = (void *) &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.Q;
//    PC_receive_data[param16] = (void *) &gimbal_control.gimbal_pitch_motor.LpfFactor;
    /********pitch end***********/

    /********yaw start***********/
    PC_receive_data[param0] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Kp;
    PC_receive_data[param1] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Ki;
    PC_receive_data[param2] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Kd;
    PC_receive_data[param3] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp;
    PC_receive_data[param4] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki;
    PC_receive_data[param5] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd;
    PC_receive_data[param6] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.max_iout;
    PC_receive_data[param7] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.max_out;
    PC_receive_data[param8] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Integral_Separation;
    PC_receive_data[param9] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Integral_Separation;

    PC_receive_data[param10] = (void *) &gimbal_control.gimbal_yaw_motor.Cloud_MotorAngle_Error_Kalman.R;
    PC_receive_data[param11] = (void *) &gimbal_control.gimbal_yaw_motor.Cloud_MotorAngle_Error_Kalman.Q;
    PC_receive_data[param12] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Kalman.R;
    PC_receive_data[param13] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Kalman.Q;

    PC_receive_data[param14] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I;
    PC_receive_data[param15] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I_Down;
    PC_receive_data[param16] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I_UP;
    PC_receive_data[param17] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I;
    PC_receive_data[param18] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_Down;
    PC_receive_data[param19] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_UP;


    PC_receive_data[param20] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.NF_D;
    PC_receive_data[param21] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Alpha;
    PC_receive_data[param22] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.NF_D;
    PC_receive_data[param23] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Alpha;
    PC_receive_data[param24] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_First;
    PC_receive_data[param25] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Filter_Ratio;
    PC_receive_data[param26] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_First;
    PC_receive_data[param27] = (void *) &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Filter_Ratio;

    PC_receive_data[param28] = (void *) &gimbal_control.gimbal_yaw_motor.LpfFactor;

//    PC_receive_data[param9] = (void *) &gimbal_control.gimbal_yaw_motor.max_relative_angle;
//    PC_receive_data[param10] = (void *) &gimbal_control.gimbal_yaw_motor.min_relative_angle;
//    PC_receive_data[param12] = (void *) &gimbal_control.gimbal_yaw_motor.relative_angle_set;
//    PC_receive_data[param13] = (void *) &gimbal_control.gimbal_yaw_motor.relative_angle;
//    PC_receive_data[param14] = (void *) &gimbal_control.gimbal_yaw_motor.offset_ecd;
    /********yaw end***********/

    /********trigger start***********/
//    PC_receive_data[param0] = (void *) &shoot_control.trigger_motor_speed_pid.Kp;
//    PC_receive_data[param1] = (void *) &shoot_control.trigger_motor_speed_pid.Ki;
//    PC_receive_data[param2] = (void *) &shoot_control.trigger_motor_speed_pid.Kd;
//    PC_receive_data[param3] = (void *) &shoot_control.trigger_speed_set;
    /********trigger end***********/
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
//        SEGGER_RTT_printf(0,"%d",i);
        if (SEGGER_RTT_HasKey()) {
            uint16_t Char_Buffer_len = sizeof(Char_Receive_Buffer);
            uint16_t NumBytes = SEGGER_RTT_Read(0, &Char_Receive_Buffer[0], Char_Buffer_len);
            if ((NumBytes < Char_Buffer_len)) {
                for (p = &Char_Receive_Buffer[0]; p != NULL; p = strstr(p, "$param")) {
//                    taskENTER_CRITICAL();
                    sscanf(p, "$param%5d[^=]", &param_num);
                    p = strstr(p, "=");
                    sscanf(p, "=%15[^;]", param_value);
                    if (strstr(param_value, ".") == NULL) {
                        param_value_int = strtol(param_value, NULL, 0);
                        int32_t *tmp = (int32_t *) PC_receive_data[param_num];
                        if (param_num == 10) {
                            *(tmp) = int32_constrain(param_value_int,
                                                     gimbal_control.gimbal_pitch_motor.min_relative_angle,
                                                     gimbal_control.gimbal_pitch_motor.max_relative_angle);
                        } else {
                            *(tmp) = param_value_int;
                        }
                    } else {
                        param_value_float = strtof(param_value, NULL);
                        float *tmp = (float *) PC_receive_data[param_num];
                        if (param_num == 11) {
                            *(tmp) = fp32_constrain(param_value_float,
                                                    gimbal_control.gimbal_pitch_motor.min_relative_angle,
                                                    gimbal_control.gimbal_pitch_motor.max_relative_angle);;
                        } else {
                            *(tmp) = param_value_float;
                        }

                    }
//                    taskEXIT_CRITICAL();
                }

            } else {
                SEGGER_RTT_WriteString(0, "Too many params.");
            }
        }
#if INCLUDE_uxTaskGetStackHighWaterMark
        PC_receive_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(13));
    }
}

/**
  * @brief          获取PC_receive_task栈大小
  * @param[in]      none
  * @retval         PC_receive_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_PC_receive_task(void){
    return PC_receive_task_stack;
}