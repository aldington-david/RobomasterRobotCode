//
// Created by Ken_n on 2023/4/14.
//

#ifndef ROBOMASTERROBOTCODE_PID_AUTOTUNE_H
#define ROBOMASTERROBOTCODE_PID_AUTOTUNE_H

#include "struct_typedef.h"

typedef enum {
    SPEED_TO_CURRENT = 1,//内环
    ANGLE_TO_SPEED,//外环
    SPEED_TO_SPEED,//外环，速度闭环，相当于前馈,尽量只用P
} pid_auto_tune_control_e;

typedef enum {
    USE_P = 0,
    USE_PI,
    USE_PID
} pid_auto_tune_type_e;

typedef struct {
    float32_t yaw_control_value;
    float32_t pitch_control_value;

    float32_t trigger_control_value;
    float32_t fric1_control_value;
    float32_t fric2_control_value;

    float32_t vx_control_value;
    float32_t vy_control_value;
    float32_t wz_control_value;

    float32_t wheel_control_value;


    float32_t regular_control_value;//单独给定值，该值不在同一循环控制两个不同电机
} pid_auto_tune_control_t;

typedef struct {
    bool_t isMax, isMin;
    float32_t *input, *output;//input本环reference反馈值,output本环输出值
    float32_t setpoint;
    float32_t noiseBand;
    pid_auto_tune_type_e controlType; //(0=PI, 1=PID)
    bool_t running;
    uint32_t peak1, peak2, lastTime;
    int32_t sampleTime;
    int32_t nLookBack;
    int32_t peakType;
    float32_t lastInputs[101];
    float32_t peaks[10];
    int32_t peakCount;
    bool_t justchanged;
    float32_t absMax, absMin;
    float32_t oStep;
    float32_t outputStart;
    float32_t Ku, Pu;

    pid_auto_tune_control_e tune_type;//选择内外环

    pid_auto_tune_control_t control_list;

    bool_t check_StartValue_mode;
} pid_auto_tune_t;

extern bool_t running_flag;

extern void
pid_auto_tune_init(pid_auto_tune_t *pidtune, float32_t *Input, float32_t *output, pid_auto_tune_type_e controlType,
                   float32_t noiseBand, float32_t oStep, int32_t LookbackSec, float32_t setpoint,
                   float32_t StartValue, pid_auto_tune_control_e control_value_type, bool_t check_StartValue_mode);

extern bool_t pid_auto_tune_runtime(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_cancel(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_setOutputStep(pid_auto_tune_t *pidtune, float32_t Step);

extern float32_t pid_auto_tune_getOutputStep(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_setControlType(pid_auto_tune_t *pidtune, int32_t Type);

extern int32_t pid_auto_tune_getControlType(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_setLookbackSec(pid_auto_tune_t *pidtune,
                                         int32_t value);//integer. think about how far apart the peaks are. 1/4-1/2 of this distance is a good value
extern int32_t pid_auto_tune_getLookbackSec(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_setNoiseBand(pid_auto_tune_t *pidtune,
                                       float32_t Band);//noise: how much noise to ignore. Try to make this as small as possible, while still preventing output chatter.
extern float32_t pid_auto_tune_getNoiseBand(pid_auto_tune_t *pidtune);

extern float32_t pid_auto_tune_getKp(pid_auto_tune_t *pidtune);

extern float32_t pid_auto_tune_getKi(pid_auto_tune_t *pidtune);

extern float32_t pid_auto_tune_getKd(pid_auto_tune_t *pidtune);

extern void pid_auto_tune_set_check_StartValue_loop(pid_auto_tune_t *pidtune);
#endif //ROBOMASTERROBOTCODE_PID_AUTOTUNE_H
