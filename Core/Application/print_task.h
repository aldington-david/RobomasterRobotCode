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


extern void print_task(void const *argument);

static void usb_printf(const char *fmt, ...);

static void Print_RTT_ReadBuffer(void);

static void Use_RTT_SetConfig(void *const variable);

extern void RTT_PrintWave(fp32 *paramin, fp32 *paramrf, fp32 *paramout, fp32 *param3);

extern fp32 bias_angle_test;
extern fp32 add_angle_test;
extern int8_t imu_temp;
#endif
