/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       print_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef PRINT_TASK_H
#define PRINT_TASK_H

#include "struct_typedef.h"
#include "stdint-gcc.h"


extern void print_task(void const *argument);

static void usb_printf(const char *fmt, ...);

static void Print_RTT_ReadBuffer(void);

static void Use_RTT_SetConfig(void *const variable);

//extern void RTT_PrintWave(fp32 *param1, fp32 *param2, fp32 *param3, fp32 *param4, fp32 *param5, fp32 *param6);
extern void RTT_PrintWave(int num_args, ...);

extern fp32 bias_angle_test;
extern fp32 add_angle_test;
extern int8_t imu_temp;
extern fp32 sp_err;
extern fp32 re_err;
extern fp32 kalman_test;
extern char switch_test;
extern uint32_t id_test;
extern fp32 Dout1_test;
extern fp32 Dout2_test;
extern fp32 KF_Dout1_test;
extern fp32 KF_Dout2_test;
extern fp32 tracer1;
extern fp32 vision_pitch_probe;
extern fp32 vision_yaw_probe;
//extern char vision_or_probe[128];

#endif
