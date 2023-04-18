//
// Created by Ken_n on 2023/4/14.
//

#ifndef ROBOMASTERROBOTCODE_PID_AUTO_TUNE_TASK_H
#define ROBOMASTERROBOTCODE_PID_AUTO_TUNE_TASK_H
#include "struct_typedef.h"
#include "PID_AutoTune.h"

extern pid_auto_tune_t pid_auto_tune_data;
extern void pid_auto_tune_task(void const *pvParameters);

/**
  * @brief          获取pid自动调谐控制指针
  * @param[in]      none
  * @retval         pid自动调谐控制指针
  */
extern const volatile pid_auto_tune_t *get_pid_auto_tune_data_point(void);
#endif //ROBOMASTERROBOTCODE_PID_AUTO_TUNE_TASK_H
