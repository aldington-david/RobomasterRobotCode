//
// Created by Ken_n on 2022/2/1.
//

#ifndef STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
#define STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"

typedef struct {
    uint16_t param1;
    uint16_t param2;
    uint16_t param3;
} PC_param_t;
extern PC_param_t PC_receive_data;
#endif //STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
