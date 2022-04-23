#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

#define RADIO_CONTROL_SWITCH_R       0
#define RADIO_CONTROL_SWITCH_L       1

extern volatile uint16_t bullet_box_pwm;
typedef enum
{
    Bullet_Box_Min = 0,
    Bullet_Box_Max,
} servo_mode_e;


extern void servo_task(void const * argument);

static void bullet_box_control(void);

#endif
