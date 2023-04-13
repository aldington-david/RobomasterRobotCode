#include "user_lib.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "SEGGER_RTT.h"

//快速指数
float32_t invpow(float32_t x, float32_t n) {
    n = n * 1.4427f + 1.4427f; // 1.4427f --> 1/ln(2)
    return exp2(x * n - n);
}

//快速开方
float32_t invSqrt(float32_t num) {
    float32_t halfnum = 0.5f * num;
    float32_t y = num;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float32_t *) &i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float32_t frame_period, float32_t max, float32_t min) {
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float32_t input) {
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value) {
        ramp_source_type->out = ramp_source_type->max_value;
    } else if (ramp_source_type->out < ramp_source_type->min_value) {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void
first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float32_t frame_period, float32_t num) {
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

void first_order_filter_clear(first_order_filter_type_t *first_order_filter_type) {
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float32_t input) {
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num /
            (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out +
            first_order_filter_type->frame_period /
            (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(float32_t *num, float32_t Limit) {
    if (*num > Limit) {
        *num = Limit;
    } else if (*num < -Limit) {
        *num = -Limit;
    }
}

//判断符号位
float32_t sign(float32_t value) {
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

//浮点死区
float32_t fp32_deadline(float32_t Value, float32_t minValue, float32_t maxValue) {
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
    if (Value < maxValue && Value > minValue) {
        Value = 0;
    }
    return Value;
}

//限幅函数
float32_t fp32_constrain(float32_t Value, float32_t minValue, float32_t maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
float32_t loop_fp32_constrain(float32_t Input, float32_t minValue, float32_t maxValue) {
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        float32_t len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        float32_t len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

uint16_t loop_uint16_constrain(uint16_t Input, uint16_t minValue, uint16_t maxValue) {
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        uint16_t len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        uint16_t len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
float32_t theta_format(float32_t Ang) {
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

/***********************/
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int floatEqual_0(float num) {//用于处理判断float变量是否为0
    if (num < (float) flaot0) {
        return 1;
    }
    return 0;
}

/***********************/
void lms_filter_init(lms_filter_type_t *lmsFilterType, float32_t step_len, float32_t *lmsstate, float32_t *lmscoeffs) {
    lmsFilterType->outputF32 = 0;
    lmsFilterType->outputERR = 0;
    lmsFilterType->step_len = step_len;
    lmsFilterType->lmsStateF32 = lmsstate;
    lmsFilterType->lmsCoeffs32 = lmscoeffs;
    memset(&lmsFilterType->lmsS, 0, sizeof(lmsFilterType->lmsS));
}

float32_t jump_error(float32_t err, float32_t err_maxValue) {
    if (err > (err_maxValue / 2.0f)) {
        err = err - err_maxValue;
    } else if (err < (-err_maxValue / 2.0f)) {
        err = err + err_maxValue;
    }
    return err;
}

void sigmoidInterpolation(float32_t startValue, float32_t endValue, uint8_t numPoints, float32_t output[14]) {
    if (endValue != 0) {
        float32_t step = 12.0f / (numPoints - 1);
        float32_t prevY = startValue;
        for (uint8_t i = 0; i < numPoints - 1; ++i) {
            float32_t x = -6.0 + i * step;
            float32_t y = startValue + (endValue - startValue) / (1.0 + expf(-x));
            output[i] = y - prevY;
            prevY = y;
//            SEGGER_RTT_printf(0,"%f\r\n",output[i]);
        }
    } else {
        for (uint8_t i = 0; i < numPoints - 1; ++i) {
            output[i] = 0;
        }
    }
}
