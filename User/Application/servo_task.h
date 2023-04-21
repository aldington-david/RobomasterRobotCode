#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include <stdint.h>
#include "struct_typedef.h"

#define RADIO_CONTROL_SWITCH_R       0
#define RADIO_CONTROL_SWITCH_L       1

extern volatile uint16_t bullet_box_pwm;
typedef enum
{
    Bullet_Box_CLOSE = 0,
    Bullet_Box_OPEN,
} servo_mode_e;


/**
  * @brief          获取servo_task栈大小
  * @param[in]      none
  * @retval         servo_task:任务堆栈大小
  */
extern uint32_t get_stack_of_servo_task(void);

extern void servo_task(void const * argument);

static void bullet_box_control(void);

#endif
