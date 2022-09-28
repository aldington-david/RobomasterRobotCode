
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      姿态解算中间层，为姿态解算提供相关函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H

//重新对应的数据类型
typedef signed char int8_t;
typedef signed short int int16_t;
//typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float float32_t;
typedef double float64_t;

//定义 NULL
#ifndef NULL
#define NULL 0
#endif

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(float32_t *high);
extern void AHRS_get_latitude(float32_t *latitude);
extern float32_t AHRS_invSqrt(float32_t num);
extern float32_t AHRS_sinf(float32_t angle);
extern float32_t AHRS_cosf(float32_t angle);
extern float32_t AHRS_tanf(float32_t angle);
extern float32_t AHRS_asinf(float32_t sin);
extern float32_t AHRS_acosf(float32_t cos);
extern float32_t AHRS_atan2f(float32_t y, float32_t x);
#endif
