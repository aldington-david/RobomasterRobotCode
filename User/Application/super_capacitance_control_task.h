//
// Created by Ken_n on 2023/4/12.
//

#ifndef ROBOMASTERROBOTCODE_SUPER_CAPACITANCE_CONTROL_TASK_H
#define ROBOMASTERROBOTCODE_SUPER_CAPACITANCE_CONTROL_TASK_H
#include "remote_control.h"
#include "detect_task.h"
#include "struct_typedef.h"

#define SUPER_CAPACITANCE_30W_MIN 3000

#define SUPER_CAPACITANCE_40W 4000

#define SUPER_CAPACITANCE_60W 6000
#define SUPER_CAPACITANCE_80W 8000
#define SUPER_CAPACITANCE_100W 10000

#define SUPER_CAPACITANCE_45W 4500
#define SUPER_CAPACITANCE_50W 5000
#define SUPER_CAPACITANCE_55W 5500

#define SUPER_CAPACITANCE_125W 12500

#define SUPER_CAPACITANCE_130W_MAX 13000

typedef struct {
    float32_t InputVoltage;
    float32_t CapacitanceVoltage;
    float32_t InputCurrent;
    float32_t Target_Power;
} super_capacitance_measure_t;

extern void super_capacitance_control_task(void const *pvParameters);

#endif //ROBOMASTERROBOTCODE_SUPER_CAPACITANCE_CONTROL_TASK_H
