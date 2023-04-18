//
// Created by Ken_n on 2022/9/23.
//

#ifndef ROBOMASTERROBOTCODE_DWT_H
#define ROBOMASTERROBOTCODE_DWT_H
#include <stdint.h>
typedef struct {
    uint32_t last_time;
    uint16_t time;
} time_record_struct;

typedef struct {
    time_record_struct tim_test_task;
    time_record_struct tim_calibrate_task;
    time_record_struct tim_detect_task;
    time_record_struct tim_chassis_task;
    time_record_struct tim_gimbal_task;
    time_record_struct tim_INS_task;
    time_record_struct tim_vision_rx_task;
    time_record_struct tim_servo_task;
    time_record_struct tim_referee_rx_task;
    time_record_struct tim_USART6TX_active_task;
    time_record_struct tim_USART1TX_active_task;
    time_record_struct tim_referee_tx_task;
    time_record_struct tim_vision_tx_task;
    time_record_struct tim_matlab_sync_task;
    time_record_struct tim_print_task;
    time_record_struct tim_PC_receive_task;
    time_record_struct tim_battery_voltage_task;
    time_record_struct tim_led_RGB_flow_task;
    time_record_struct tim_super_capacitance_control_task;
    time_record_struct tim_pid_auto_tune_task;
} task_time_record_t;

typedef struct {
    time_record_struct gyro;
    time_record_struct accel;
    time_record_struct mag;
} AHRS_time_record_t;

extern task_time_record_t global_task_time;

extern void DWT_init(void);

extern void DWT_stop(void);

extern uint32_t DWT_get_tick(void);

extern void DWT_get_time_interval_us(time_record_struct *task_time);

#endif //ROBOMASTERROBOTCODE_DWT_H
