//
// Created by Ken_n on 2022/2/1.
//

#ifndef STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
#define STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"

enum {
    param0 = 0,//re/abangle-kp
    param1,//re/abangle-ki
    param2,//re/abangle-kd
    param3,//gimspeed-kp
    param4,//gimspeed-ki
    param5,//gimspeed-kd
    param6,//gimspeed-IS
    param7,//re/abangle-maxiout
    param8,//re/abangle-maxout
    param9,//max_relative_angle
    param10,//min_relative_angle
    param11,//relative_angle_set
    param12,//err_kal_R
    param13,//err_kal_Q
    param14,//pid_kal_R
    param15,//pid_kal_Q
    param16,//LpfFactor
    param17,//Variable_I
    param18,//Variable_I_Down
    param19,//Variable_I_UP
    param20,//re-D_First
    param21,//re-D_Filter_Ratio 0-1
    param22,//sp-D_First
    param23,//sp-D_Filter_Ratio 0-1
    param24,//D_Low_Pass
    param25,//D_Low_Pass_Filter_num 0-1


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

extern void *PC_receive_data[26];

extern void PC_receive_task(void const *argument);

#endif //STANDARDROBOTBASICCODE_PC_RECEIVE_TASK_H
