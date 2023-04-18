//
// Created by Ken_n on 2023/4/14.
//

#include "PID_AutoTune.h"
#include "main.h"
#include "math.h"
#include "SEGGER_RTT.h"
#include "arm_math.h"

static void FinishUp(pid_auto_tune_t *pidtune);

void pid_auto_tune_init(pid_auto_tune_t *pidtune, float32_t *Input, float32_t *output, pid_auto_tune_type_e controlType,
                        float32_t noiseBand, float32_t oStep, int32_t LookbackSec, float32_t setpoint,
                        float32_t StartValue, pid_auto_tune_control_e control_value_type,bool_t check_StartValue_mode) {
    pidtune->input = Input;
    pidtune->output = output;
    pidtune->controlType = controlType;
    pidtune->noiseBand = noiseBand;
    pidtune->running = true;
    pidtune->oStep = oStep;
    pidtune->setpoint = setpoint;
    pidtune->outputStart = StartValue;
    pidtune->check_StartValue_mode = check_StartValue_mode;
    pid_auto_tune_setLookbackSec(pidtune, LookbackSec);
    pidtune->lastTime = HAL_GetTick();
    memset(&pidtune->control_list, 0, sizeof(pid_auto_tune_control_t));
}

bool_t pid_auto_tune_runtime(pid_auto_tune_t *pidtune) {
    if (pidtune->peakCount > 9 && pidtune->running) {
        pidtune->running = false;
        FinishUp(pidtune);
        return 1;
    }
    uint32_t now = HAL_GetTick();

    if ((now - pidtune->lastTime) < pidtune->sampleTime) return false;
    pidtune->lastTime = now;
    float32_t refVal = *pidtune->input;
    if (!pidtune->running) { //initialize working variables the first time around
        pidtune->peakType = 0;
        pidtune->peakCount = 0;
        pidtune->justchanged = false;
        pidtune->absMax = refVal;
        pidtune->absMin = refVal;
        pidtune->setpoint = refVal;
        pidtune->running = true;
        pidtune->outputStart = *pidtune->output;
        *pidtune->output = pidtune->outputStart + pidtune->oStep;
    } else {
        if (refVal > pidtune->absMax)pidtune->absMax = refVal;
        if (refVal < pidtune->absMin)pidtune->absMin = refVal;
    }

    //oscillate the pidtune->output base on the input's relation to the pidtune->setpoint

    if (refVal > pidtune->setpoint + pidtune->noiseBand) {

        *pidtune->output = pidtune->outputStart - pidtune->oStep;
    } else if (refVal < pidtune->setpoint - pidtune->noiseBand) {
        *pidtune->output = pidtune->outputStart + pidtune->oStep;
    }


    //bool pidtune->isMax=true, pidtune->isMin=true;
    pidtune->isMax = true;
    pidtune->isMin = true;
    //id pidtune->peaks
    for (int i = pidtune->nLookBack - 1; i >= 0; i--) {
        float32_t val = pidtune->lastInputs[i];
        if (pidtune->isMax) {
            pidtune->isMax = refVal > val;
        }
        if (pidtune->isMin) {
            pidtune->isMin = refVal < val;
        }
        pidtune->lastInputs[i + 1] = pidtune->lastInputs[i];
    }
    pidtune->lastInputs[0] = refVal;
    if (pidtune->lastInputs[pidtune->nLookBack] ==
        0) {  //we don't want to trust the maxes or mins until the inputs array has been filled
        return 0;
    }

    if (pidtune->isMax) {
        if (pidtune->peakType == 0)pidtune->peakType = 1;
        if (pidtune->peakType == -1) {
            pidtune->peakType = 1;
            pidtune->justchanged = true;
            pidtune->peak2 = pidtune->peak1;
        }
        pidtune->peak1 = now;
        pidtune->peaks[pidtune->peakCount] = refVal;

    } else if (pidtune->isMin) {
        if (pidtune->peakType == 0)pidtune->peakType = -1;
        if (pidtune->peakType == 1) {
            pidtune->peakType = -1;
            pidtune->peakCount++;
            pidtune->justchanged = true;
        }

        if (pidtune->peakCount < 10)pidtune->peaks[pidtune->peakCount] = refVal;
    }

    if (pidtune->justchanged &&
        pidtune->peakCount > 2) { //we've transitioned.  check if we can autotune based on the last pidtune->peaks
        float32_t avgSeparation =
                (fabsf(pidtune->peaks[pidtune->peakCount - 1] - pidtune->peaks[pidtune->peakCount - 2]) +
                 fabsf(pidtune->peaks[pidtune->peakCount - 2] - pidtune->peaks[pidtune->peakCount - 3])) / 2;
//        SEGGER_RTT_printf(0, "%f,%f\r\n", avgSeparation, 0.05f * (pidtune->absMax - pidtune->absMin));
        if (avgSeparation < 0.05f * (pidtune->absMax - pidtune->absMin)) {
            FinishUp(pidtune);
            pidtune->running = false;
            return 1;

        }
    }
    pidtune->justchanged = false;
    return 0;
}

static void FinishUp(pid_auto_tune_t *pidtune) {
    *(pidtune->output) = 0;
    pidtune->Ku = 4 * (2 * pidtune->oStep) / ((pidtune->absMax - pidtune->absMin) * PI);
    pidtune->Pu = (float32_t) (pidtune->peak1 - pidtune->peak2) / 1000;
    SEGGER_RTT_WriteString(0, "DONE\r\n");
}

float32_t pid_auto_tune_getKp(pid_auto_tune_t *pidtune) {
    float32_t return_value = 0;
    if (pidtune->controlType == USE_P) {
        return_value = 0.5f * pidtune->Ku;
    } else if (pidtune->controlType == USE_PI) {
        return_value = 0.4f * pidtune->Ku;
    } else if (pidtune->controlType == USE_PID) {
        return_value = 0.6f * pidtune->Ku;
    }
    return return_value;
}

float32_t pid_auto_tune_getKi(pid_auto_tune_t *pidtune) {
    float32_t return_value = 0;
    if (pidtune->controlType == USE_P) {
        return_value = 0;
    } else if (pidtune->controlType == USE_PI) {
        return_value = 0.48f * pidtune->Ku / pidtune->Pu;
    } else if (pidtune->controlType == USE_PID) {
        return_value = 1.2f * pidtune->Ku / pidtune->Pu;
    }
    return return_value;
}

float32_t pid_auto_tune_getKd(pid_auto_tune_t *pidtune) {
    float32_t return_value = 0;
    if (pidtune->controlType == USE_P) {
        return_value = 0;
    } else if (pidtune->controlType == USE_PI) {
        return_value = 0;
    } else if (pidtune->controlType == USE_PID) {
        return_value = 0.075f * pidtune->Ku * pidtune->Pu;
    }
    return return_value;
}

void pid_auto_tune_setOutputStep(pid_auto_tune_t *pidtune, float32_t Step) {
    pidtune->oStep = Step;
}

float32_t pid_auto_tune_getOutputStep(pid_auto_tune_t *pidtune) {
    return pidtune->oStep;
}

void pid_auto_tune_setControlType(pid_auto_tune_t *pidtune, int32_t Type) {
    pidtune->controlType = Type;
}

int32_t pid_auto_tune_getControlType(pid_auto_tune_t *pidtune) {
    return pidtune->controlType;
}

void pid_auto_tune_setLookbackSec(pid_auto_tune_t *pidtune, int32_t value) {
    if (value < 1) value = 1;
    if (value < 25) {
        pidtune->nLookBack = value * 4;
        pidtune->sampleTime = 250;
    } else {
        pidtune->nLookBack = 100;
        pidtune->sampleTime = value * 10;
    }
}

int32_t pid_auto_tune_getLookbackSec(pid_auto_tune_t *pidtune) {
    return pidtune->nLookBack * pidtune->sampleTime / 1000;
}

void pid_auto_tune_setNoiseBand(pid_auto_tune_t *pidtune, float32_t Band) {
    pidtune->noiseBand = Band;
}

float32_t pid_auto_tune_getNoiseBand(pid_auto_tune_t *pidtune) {
    return pidtune->noiseBand;
}

void pid_auto_tune_cancel(pid_auto_tune_t *pidtune) {
    pidtune->running = false;
    *(pidtune->output) = 0;
}