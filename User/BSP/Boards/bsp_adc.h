#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

extern void init_vrefint_reciprocal(void);
extern float32_t get_temprate(void);
extern float32_t get_battery_voltage(void);
extern uint8_t get_hardware_version(void);
#endif
