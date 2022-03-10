#ifndef USER_LIB_H
#define USER_LIB_H
#include "stdint-gcc.h"
#include "struct_typedef.h"
#include "arm_math.h"

typedef struct __attribute__((packed))
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

typedef struct __attribute__((packed)) {
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num;       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct {
    fp32 step_len;
    fp32 outputF32;          //滤波输出的数据
    fp32 outputERR; //误差数据
    fp32 *lmsStateF32; // 状态缓存，大小numTaps + blockSize - 1
    fp32 *lmsCoeffs32; //滤波器系数
    arm_lms_norm_instance_f32 lmsS;
} lms_filter_type_t;

//快速开方
extern fp32 invSqrt(fp32 num);

//快速指数
extern fp32 invpow(fp32 x, fp32 n);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num);

//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

//限幅函数
extern int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue);

//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
/******************************/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>  //机器人的默认配置文件。

#define HIGH    0x1
#define LOW    0x0
#define flaot0  1e-6  //当变量的绝对值小于此时，变量float为0
//#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f

#ifdef abs
#undef abs
#endif

#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define getBit(value, pos) ((value >> pos) & 1)
#define setBit(value, pos) value|(1 << pos)
#define clrBit(value, pos) value&(~(1 << pos))
#define toggleBit(value, pos) value^(1 << pos)

long map(long, long, long, long, long);

int floatEqual_0(float num);

#define  IndexOutofBounds(index, length) (index<0||index>length-1)


/******************************/
void lms_filter_init(lms_filter_type_t *lmsFilterType, fp32 step_len, fp32 *lmsstate, fp32 *lmscoeffs);
#endif
