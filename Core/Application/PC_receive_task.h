//
// Created by Ken_n on 2022/2/1.
//

#ifndef STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
#define STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"

enum {
    param0 = 0,
    param1,
    param2,
    param3,
};

typedef union {
    int8_t *varInt8;
    int16_t *varInt16;
    int32_t *varInt32;
    uint8_t *varUint8;
    uint16_t *varUint16;
    uint32_t *varUint32;
    float *varFloat;
} DebugVarTrans;

extern void *PC_receive_data[4];

extern void PC_receive_task(void const *argument);

#endif //STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
