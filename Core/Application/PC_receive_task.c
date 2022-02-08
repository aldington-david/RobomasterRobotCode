//
// Created by Ken_n on 2022/2/1.
//

#include "PC_receive_task.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "referee_task.h"
#include "print_task.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"
#include "task.h"

char Char_Receive_Buffer[256];
char *p;
int param_num;
char param_value[16];
int32_t param_value_int;
double param_value_float;
void *PC_receive_data[4];
int i;

//$param0=12;$param1=13;$param2=12.32;$param3=11.21;
void PC_receive_task(void const *argument) {
    PC_receive_data[param0] = (void *) &test_var;
    PC_receive_data[param1] = (void *) &test_var1;
    PC_receive_data[param2] = (void *) &test_var2;
    PC_receive_data[param3] = (void *) &test_var3;
    while (1) {
//        SEGGER_RTT_printf(0,"%d",i);
        if (SEGGER_RTT_HasKey()) {
            uint16_t Char_Buffer_len = sizeof(Char_Receive_Buffer);
            uint16_t NumBytes = SEGGER_RTT_Read(0, &Char_Receive_Buffer[0], Char_Buffer_len);
            if ((NumBytes < Char_Buffer_len)) {
                taskENTER_CRITICAL();
                for (p = &Char_Receive_Buffer[0]; p != NULL; p = strstr(p, "$param")) {
                    i++;
                    sscanf(p, "$param%5d[^=]", &param_num);
                    p = strstr(p, "=");
                    sscanf(p, "=%15[^;]", param_value);
                    if (strstr(param_value, ".") == NULL) {
                        param_value_int = strtol(param_value, NULL, 0);
                        int32_t *tmp = PC_receive_data[param_num];
                        *(tmp) = param_value_int;
                    } else {
                        param_value_float = strtod(param_value, NULL);
                        float *tmp = PC_receive_data[param_num];
                        *(tmp) = param_value_float;
                    }
                }
//                taskEXIT_CRITICAL();
            } else {
                SEGGER_RTT_WriteString(0, "Too many params.");
            }
        }
    }
}