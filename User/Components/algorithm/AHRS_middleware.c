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

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"
/**
 * @brief          用于获取当前高度
 * @author         RM
 * @param[in]      高度的指针，float32_t
 * @retval         返回空
 */

void AHRS_get_height(float32_t* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          用于获取当前纬度
 * @author         RM
 * @param[in]      纬度的指针，float32_t
 * @retval         返回空
 */

void AHRS_get_latitude(float32_t* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          快速开方函数，
 * @author         RM
 * @param[in]      输入需要开方的浮点数，float32_t
 * @retval         返回1/sqrt 开方后的倒数
 */

float32_t AHRS_invSqrt(float32_t num)
{
    return 1/sqrtf(num);

//    float32_t halfnum = 0.5f * num;
//    float32_t y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float32_t*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          sin函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的sin值
 */

float32_t AHRS_sinf(float32_t angle)
{
    return arm_sin_f32(angle);
}
/**
 * @brief          cos函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的cos值
 */

float32_t AHRS_cosf(float32_t angle)
{
    return arm_cos_f32(angle);
}

/**
 * @brief          tan函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的tan值
 */

float32_t AHRS_tanf(float32_t angle)
{
    return tanf(angle);
}
/**
 * @brief          用于32位浮点数的反三角函数 asin函数
 * @author         RM
 * @param[in]      输入sin值，最大1.0f，最小-1.0f
 * @retval         返回角度 单位弧度
 */

float32_t AHRS_asinf(float32_t sin)
{

    return asinf(sin);
}

/**
 * @brief          反三角函数acos函数
 * @author         RM
 * @param[in]      输入cos值，最大1.0f，最小-1.0f
 * @retval         返回对应的角度 单位弧度
 */

float32_t AHRS_acosf(float32_t cos)
{

    return acosf(cos);
}

/**
 * @brief          反三角函数atan函数
 * @author         RM
 * @param[in]      输入tan值中的y值 最大正无穷，最小负无穷
 * @param[in]      输入tan值中的x值 最大正无穷，最小负无穷
 * @retval         返回对应的角度 单位弧度
 */

float32_t AHRS_atan2f(float32_t y, float32_t x)
{
    return atan2f(y, x);
}
