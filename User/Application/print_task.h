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
#include <stdint.h>


/**
  * @brief          获取print_task栈大小
  * @param[in]      none
  * @retval         print_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_print_task(void);

extern void print_task(void const *argument);

static void usb_printf(const char *fmt, ...);

//static void Print_RTT_ReadBuffer(void);

//static void Use_RTT_SetConfig(void *const variable);

/**
  * @brief          RTT波形打印，格式为富莱安H7-tool显示格式，参数为浮点数指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
extern void RTT_PrintWave(int num_args, ...);

/**
  * @brief          RTT波形打印(整形用)，格式为富莱安H7-tool显示格式，参数为整形转浮点数非指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
extern void RTT_PrintWave_np(int num_args, ...);

/**
  * @brief          Timer7溢出中断调用打印，Timer7配置务必在STM32Cubemx调节，APB1 82Mhz,建议时间不小于60us
  * @param[in]      none
  * @retval         none
  */
extern void RTT_timer_trigger(void);

extern float32_t bias_angle_test;
extern float32_t add_angle_test;
extern int8_t imu_temp;
extern float32_t sp_err;
extern float32_t re_err;
extern float32_t kalman_test;
extern char switch_test;
extern uint32_t id_test;
extern float32_t Dout1_test;
extern float32_t Dout2_test;
extern float32_t KF_Dout1_test;
extern float32_t KF_Dout2_test;
extern float32_t tracer1;
extern float32_t vision_pitch_probe;
extern float32_t vision_yaw_probe;
extern char vision_or_probe[128];


extern float32_t pid_out_probe;
extern float32_t pid_pout_probe;
extern float32_t pid_iout_probe;
extern float32_t pid_dout_probe;
#endif
